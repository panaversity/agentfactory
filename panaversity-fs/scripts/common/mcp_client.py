"""MCP Client for PanaversityFS tool invocation.

This module provides a client for invoking PanaversityFS MCP tools using the
official MCP SDK with StreamableHTTP (SSE) transport.

It supports the key tools needed for hydration and ingestion workflows:
- plan_build: Delta detection for incremental builds
- read_content: Download individual files
- write_content: Upload content with conflict detection
- delete_content: Remove files from storage
- glob_search: Find files by pattern
- upload_asset: Upload binary assets
"""

import os
import json
import asyncio
from typing import Any
from dataclasses import dataclass

from mcp.client.streamable_http import streamablehttp_client
from mcp.client.session import ClientSession
from mcp import types

from scripts.common.log_sanitizer import sanitize_message


class MCPError(Exception):
    """Base exception for MCP client errors."""
    pass


class MCPConnectionError(MCPError):
    """Raised when connection to MCP server fails."""
    pass


class MCPToolError(MCPError):
    """Raised when MCP tool returns an error."""
    def __init__(self, message: str, tool_name: str, error_type: str | None = None):
        super().__init__(message)
        self.tool_name = tool_name
        self.error_type = error_type


@dataclass
class MCPConfig:
    """Configuration for MCP client."""
    base_url: str
    timeout_seconds: float = 120.0
    max_retries: int = 3
    retry_delay_seconds: float = 1.0

    @classmethod
    def from_env(cls) -> "MCPConfig":
        """Load configuration from environment variables."""
        base_url = os.environ.get("PANAVERSITY_MCP_URL", "http://localhost:8000")
        # Ensure URL ends with /mcp for StreamableHTTP
        if not base_url.endswith("/mcp"):
            base_url = base_url.rstrip("/") + "/mcp"
        timeout = float(os.environ.get("PANAVERSITY_MCP_TIMEOUT", "120"))
        return cls(base_url=base_url, timeout_seconds=timeout)


class MCPClient:
    """MCP client using official SDK with StreamableHTTP (SSE) transport.

    This client uses the MCP SDK's streamable_http transport for proper
    MCP protocol communication over HTTP with Server-Sent Events (SSE).

    Usage:
        async with MCPClient() as client:
            result = await client.plan_build("my-book")
            print(f"Manifest hash: {result['manifest_hash']}")
    """

    def __init__(self, config: MCPConfig | None = None):
        """Initialize MCP client.

        Args:
            config: Optional configuration. If not provided, loads from environment.
        """
        self.config = config or MCPConfig.from_env()
        self._session: ClientSession | None = None
        self._streams_context = None

    async def __aenter__(self) -> "MCPClient":
        """Enter async context manager."""
        try:
            # Create streamable HTTP client
            self._streams_context = streamablehttp_client(
                url=self.config.base_url,
                timeout=self.config.timeout_seconds,
                sse_read_timeout=self.config.timeout_seconds + 180  # Extra time for SSE
            )

            # Get read/write streams
            read_stream, write_stream, _ = await self._streams_context.__aenter__()

            # Initialize MCP session
            self._session = ClientSession(read_stream, write_stream)
            await self._session.__aenter__()

            # Initialize the session
            await self._session.initialize()

            return self

        except Exception as e:
            safe_error = sanitize_message(str(e))
            safe_url = sanitize_message(self.config.base_url)
            raise MCPConnectionError(f"Failed to connect to MCP server at {safe_url}: {safe_error}")

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Exit async context manager."""
        try:
            if self._session:
                await self._session.__aexit__(exc_type, exc_val, exc_tb)
                self._session = None
            if self._streams_context:
                await self._streams_context.__aexit__(exc_type, exc_val, exc_tb)
                self._streams_context = None
        except Exception:
            pass  # Ignore cleanup errors

    async def call_tool(self, tool_name: str, params: dict[str, Any]) -> dict[str, Any]:
        """Invoke an MCP tool with retry logic.

        Args:
            tool_name: Name of the MCP tool (e.g., "plan_build", "read_content")
            params: Tool parameters as a dictionary

        Returns:
            Tool response as a dictionary

        Raises:
            MCPConnectionError: If connection to server fails after retries
            MCPToolError: If tool returns an error response
        """
        if not self._session:
            raise MCPError("Client not initialized. Use 'async with MCPClient()' context manager.")

        last_error: Exception | None = None

        for attempt in range(self.config.max_retries):
            try:
                # Call the tool using MCP SDK
                # FastMCP expects arguments wrapped in a "params" field
                wrapped_args = {"params": params} if params else {}
                result: types.CallToolResult = await self._session.call_tool(
                    name=tool_name,
                    arguments=wrapped_args
                )

                # Check if tool returned an error
                if result.isError:
                    error_msg = sanitize_message(str(result.content))
                    raise MCPToolError(
                        f"Tool '{tool_name}' returned error: {error_msg}",
                        tool_name=tool_name,
                        error_type="tool_error"
                    )

                # Extract result from content
                # MCP tools return TextContent or ImageContent
                for content_item in result.content:
                    if isinstance(content_item, types.TextContent):
                        # Parse JSON response
                        try:
                            return json.loads(content_item.text)
                        except json.JSONDecodeError:
                            # Return as-is if not JSON
                            return {"result": content_item.text}

                # If no text content, return empty dict
                return {}

            except MCPToolError:
                # Don't retry tool errors
                raise

            except Exception as e:
                safe_error = sanitize_message(str(e))
                last_error = MCPConnectionError(f"Tool call failed: {safe_error}")
                if attempt < self.config.max_retries - 1:
                    await asyncio.sleep(self.config.retry_delay_seconds * (attempt + 1))
                    continue

        if last_error:
            raise last_error
        raise MCPError("Unknown error during tool call")

    # Convenience methods for common tools

    async def plan_build(
        self,
        book_id: str,
        target_manifest_hash: str | None = None
    ) -> dict[str, Any]:
        """Get build plan with delta detection.

        Args:
            book_id: Book identifier
            target_manifest_hash: Previous manifest hash for incremental detection

        Returns:
            Plan build response with:
            - status: "unchanged" or "changed"
            - manifest_hash: Current manifest hash
            - files: List of changed files
            - changed_count: Number of changed files
            - total_files: Total files in book
        """
        params = {"book_id": book_id}
        if target_manifest_hash:
            params["target_manifest_hash"] = target_manifest_hash
        return await self.call_tool("plan_build", params)

    async def read_content(
        self,
        book_id: str,
        path: str,
        user_id: str | None = None
    ) -> dict[str, Any]:
        """Read content from storage.

        Args:
            book_id: Book identifier
            path: Content path within book
            user_id: Optional user ID for overlay content

        Returns:
            Content response with:
            - content: File content as string
            - file_hash_sha256: SHA256 hash of content
            - file_size: Size in bytes
            - source: "base" or "overlay"
        """
        params = {"book_id": book_id, "path": path}
        if user_id:
            params["user_id"] = user_id
        return await self.call_tool("read_content", params)

    async def write_content(
        self,
        book_id: str,
        path: str,
        content: str,
        expected_hash: str | None = None,
        user_id: str | None = None
    ) -> dict[str, Any]:
        """Write content to storage.

        Args:
            book_id: Book identifier
            path: Content path within book
            content: Markdown content to write
            expected_hash: Required for updates (conflict detection)
            user_id: Optional user ID for overlay content

        Returns:
            Write response with:
            - status: "success" or "error"
            - mode: "created" or "updated"
            - file_hash: New SHA256 hash
        """
        params = {"book_id": book_id, "path": path, "content": content}
        if expected_hash:
            params["expected_hash"] = expected_hash
        if user_id:
            params["user_id"] = user_id
        return await self.call_tool("write_content", params)

    async def delete_content(
        self,
        book_id: str,
        path: str,
        user_id: str | None = None
    ) -> dict[str, Any]:
        """Delete content from storage.

        Args:
            book_id: Book identifier
            path: Content path within book
            user_id: Optional user ID for overlay content

        Returns:
            Delete response with:
            - status: "success"
            - existed: Whether file existed before delete
        """
        params = {"book_id": book_id, "path": path}
        if user_id:
            params["user_id"] = user_id
        return await self.call_tool("delete_content", params)

    async def glob_search(
        self,
        book_id: str,
        pattern: str
    ) -> dict[str, Any]:
        """Search for files matching glob pattern.

        Args:
            book_id: Book identifier
            pattern: Glob pattern (e.g., "content/**/*.md")

        Returns:
            Search response with:
            - matches: List of matching file paths
            - count: Number of matches
        """
        return await self.call_tool("glob_search", {"book_id": book_id, "pattern": pattern})

    async def upload_asset(
        self,
        book_id: str,
        asset_type: str,
        filename: str,
        binary_data: str
    ) -> dict[str, Any]:
        """Upload binary asset.

        Args:
            book_id: Book identifier
            asset_type: Asset type (img, slides, videos, audio)
            filename: Original filename
            binary_data: Base64-encoded binary content

        Returns:
            Upload response with:
            - status: "success"
            - cdn_url: URL to access asset
            - file_size: Size in bytes
        """
        return await self.call_tool("upload_asset", {
            "book_id": book_id,
            "asset_type": asset_type,
            "filename": filename,
            "binary_data": binary_data
        })
