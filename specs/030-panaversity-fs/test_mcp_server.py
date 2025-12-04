#!/usr/bin/env python3
"""
Minimal Test MCP Server - Validates FastMCP + OpenDAL Integration

This test server demonstrates:
1. FastMCP server initialization with Stateless Streamable HTTP transport
2. OpenDAL AsyncOperator for local filesystem operations
3. Pydantic input validation
4. MCP tool annotations (readOnlyHint, destructiveHint, idempotentHint)
5. Tool registration via @mcp.tool decorator

Usage:
    python test_mcp_server.py

    Server runs at http://localhost:8000/mcp
    Test with: npx @modelcontextprotocol/inspector http://localhost:8000/mcp
"""

from mcp.server.fastmcp import FastMCP
from pydantic import BaseModel, Field, ConfigDict
import opendal
import asyncio
import os
from typing import List

# Initialize FastMCP server with stateless HTTP transport
mcp = FastMCP(
    "test_fs",
    stateless_http=True,      # Enable Stateless Streamable HTTP
    json_response=True        # Disable SSE, use pure JSON responses
)

# Initialize OpenDAL operator (local filesystem for testing)
# Create test directory if it doesn't exist
TEST_ROOT = "/tmp/test-panaversity-fs"
os.makedirs(TEST_ROOT, exist_ok=True)

op = opendal.AsyncOperator("fs", root=TEST_ROOT)


# Pydantic Models for Input Validation
class WriteFileInput(BaseModel):
    """Input model for write_test_file tool."""

    model_config = ConfigDict(
        str_strip_whitespace=True,
        validate_assignment=True,
        extra='forbid'
    )

    path: str = Field(
        ...,
        description="File path relative to root (e.g., 'books/test/lesson.md')",
        min_length=1,
        max_length=255,
        pattern=r'^[a-z0-9/_.-]+$'
    )
    content: str = Field(
        ...,
        description="File content to write",
        min_length=1,
        max_length=100_000
    )


class ReadFileInput(BaseModel):
    """Input model for read_test_file tool."""

    model_config = ConfigDict(
        str_strip_whitespace=True,
        validate_assignment=True,
        extra='forbid'
    )

    path: str = Field(
        ...,
        description="File path relative to root",
        min_length=1,
        max_length=255,
        pattern=r'^[a-z0-9/_.-]+$'
    )


class ListFilesInput(BaseModel):
    """Input model for list_test_files tool."""

    model_config = ConfigDict(
        str_strip_whitespace=True,
        validate_assignment=True,
        extra='forbid'
    )

    directory: str = Field(
        default="",
        description="Directory path to list (empty for root)",
        max_length=255,
        pattern=r'^[a-z0-9/_.-]*$'
    )


# MCP Tool Definitions
@mcp.tool(
    name="write_test_file",
    annotations={
        "title": "Write Test File",
        "readOnlyHint": False,
        "destructiveHint": True,  # Overwrites existing files
        "idempotentHint": True,   # Same content → same result
        "openWorldHint": False
    }
)
async def write_test_file(params: WriteFileInput) -> str:
    """Write content to a test file using OpenDAL.

    This tool demonstrates:
    - Pydantic input validation with Field constraints
    - OpenDAL async write operation
    - Proper error handling
    - Structured response format

    Args:
        params (WriteFileInput): Validated input containing:
            - path (str): File path relative to root
            - content (str): File content to write

    Returns:
        str: JSON-formatted response with operation details
    """
    try:
        # Write file using OpenDAL AsyncOperator
        await op.write(params.path, params.content.encode('utf-8'))

        # Get file metadata to confirm write
        metadata = await op.stat(params.path)

        return f"""File written successfully:
- Path: {params.path}
- Size: {metadata.content_length} bytes
- Content: {len(params.content)} characters
- Location: {TEST_ROOT}/{params.path}"""

    except Exception as e:
        return f"Error writing file: {type(e).__name__}: {str(e)}"


@mcp.tool(
    name="read_test_file",
    annotations={
        "title": "Read Test File",
        "readOnlyHint": True,     # Does not modify environment
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def read_test_file(params: ReadFileInput) -> str:
    """Read content from a test file using OpenDAL.

    Args:
        params (ReadFileInput): Validated input containing:
            - path (str): File path relative to root

    Returns:
        str: File content or error message
    """
    try:
        # Read file using OpenDAL AsyncOperator
        content_bytes = await op.read(params.path)
        content = content_bytes.decode('utf-8')

        # Get file metadata
        metadata = await op.stat(params.path)

        return f"""File read successfully:
- Path: {params.path}
- Size: {metadata.content_length} bytes
- Content:
---
{content}
---"""

    except Exception as e:
        return f"Error reading file: {type(e).__name__}: {str(e)}"


@mcp.tool(
    name="list_test_files",
    annotations={
        "title": "List Test Files",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def list_test_files(params: ListFilesInput) -> str:
    """List files in a directory using OpenDAL.

    Args:
        params (ListFilesInput): Validated input containing:
            - directory (str): Directory path to list (empty for root)

    Returns:
        str: Formatted list of files with metadata
    """
    try:
        # List directory using OpenDAL AsyncOperator
        path = params.directory if params.directory else ""
        entries = await op.list(path)

        if not entries:
            return f"Directory '{path or 'root'}' is empty"

        lines = [f"Files in '{path or 'root'}':", ""]

        for entry in entries:
            try:
                metadata = await op.stat(entry.path)
                lines.append(f"- {entry.path} ({metadata.content_length} bytes)")
            except Exception as e:
                lines.append(f"- {entry.path} (metadata error: {e})")

        return "\n".join(lines)

    except Exception as e:
        return f"Error listing directory: {type(e).__name__}: {str(e)}"


@mcp.tool(
    name="delete_test_file",
    annotations={
        "title": "Delete Test File",
        "readOnlyHint": False,
        "destructiveHint": True,  # Irreversible operation
        "idempotentHint": True,   # Deleting non-existent file is OK
        "openWorldHint": False
    }
)
async def delete_test_file(params: ReadFileInput) -> str:
    """Delete a test file using OpenDAL.

    Args:
        params (ReadFileInput): Validated input containing:
            - path (str): File path relative to root

    Returns:
        str: Confirmation message or error
    """
    try:
        # Check if file exists first
        try:
            await op.stat(params.path)
            exists = True
        except:
            exists = False

        # Delete file using OpenDAL AsyncOperator
        await op.delete(params.path)

        if exists:
            return f"File deleted successfully: {params.path}"
        else:
            return f"File did not exist (idempotent delete): {params.path}"

    except Exception as e:
        return f"Error deleting file: {type(e).__name__}: {str(e)}"


if __name__ == "__main__":
    print(f"Test MCP Server Starting...")
    print(f"Storage Root: {TEST_ROOT}")
    print(f"Server Name: test_fs")
    print(f"Transport: Stateless Streamable HTTP")
    print(f"URL: http://localhost:8000/mcp")
    print(f"\nTools:")
    print(f"  - write_test_file (destructive, idempotent)")
    print(f"  - read_test_file (readOnly)")
    print(f"  - list_test_files (readOnly)")
    print(f"  - delete_test_file (destructive, idempotent)")
    print(f"\nTest with MCP Inspector:")
    print(f"  npx @modelcontextprotocol/inspector http://localhost:8000/mcp")
    print()

    mcp.run(transport="streamable-http")
