"""
MCP (Model Context Protocol) Integration for Google ADK

Demonstrates connecting ADK agents to MCP servers:
- Stdio connection for local MCP servers
- SSE connection for remote MCP servers
- Error handling and reconnection
- Combining MCP tools with native ADK tools
- Tool filtering for security

MCP enables ADK agents to use standardized tools from any MCP server,
extending capabilities without custom implementation.

Prerequisites:
    - Node.js installed (for npx-based MCP servers)
    - MCP server packages (e.g., @modelcontextprotocol/server-filesystem)

Usage:
    export GOOGLE_API_KEY=your_api_key
    python mcp_integration.py
"""

import asyncio
import os
from typing import Dict, List, Optional

from dotenv import load_dotenv
from google.adk import Runner
from google.adk.agents import LlmAgent
from google.adk.sessions import InMemorySessionService
from google.adk.tools.mcp_tool import McpToolset
from google.adk.tools.mcp_tool.mcp_session_manager import (
    SseConnectionParams,
    StdioConnectionParams,
)
from google.genai import types
from mcp import StdioServerParameters

# Load environment variables
load_dotenv()


# =============================================================================
# BASIC MCP FILESYSTEM AGENT
# =============================================================================


def create_mcp_filesystem_agent(
    allowed_paths: Optional[List[str]] = None,
    tool_filter: Optional[List[str]] = None,
) -> LlmAgent:
    """
    Create an agent with MCP filesystem tools.

    Args:
        allowed_paths: Directories the agent can access (defaults to /tmp)
        tool_filter: Specific tools to expose (e.g., ['read_file', 'list_directory'])

    Returns:
        LlmAgent configured with filesystem MCP tools.
    """
    allowed_paths = allowed_paths or ["/tmp"]

    # Build npx args for the filesystem server
    npx_args = ["-y", "@modelcontextprotocol/server-filesystem"]
    npx_args.extend(allowed_paths)

    mcp_config = {
        "connection_params": StdioConnectionParams(
            server_params=StdioServerParameters(
                command="npx",
                args=npx_args,
            )
        ),
    }

    if tool_filter:
        mcp_config["tool_filter"] = tool_filter

    return LlmAgent(
        model="gemini-2.5-flash",
        name="filesystem_agent",
        instruction=f"""You are a filesystem assistant.
You can read, write, and manage files in these directories: {', '.join(allowed_paths)}

Available operations:
- List directory contents
- Read file contents
- Write files
- Create directories
- Move/copy files

Always confirm before making changes to files.
Never access files outside allowed directories.""",
        tools=[McpToolset(**mcp_config)],
    )


# =============================================================================
# MCP WITH NATIVE TOOLS
# =============================================================================


def analyze_content(content: str, analysis_type: str) -> Dict:
    """
    Analyze text content (native ADK tool).

    Args:
        content: Text to analyze
        analysis_type: Type of analysis (summary, sentiment, keywords)

    Returns:
        Analysis results.
    """
    results = {
        "content_length": len(content),
        "word_count": len(content.split()),
        "analysis_type": analysis_type,
    }

    if analysis_type == "summary":
        # Simple extractive summary (first 100 chars)
        results["summary"] = content[:100] + "..." if len(content) > 100 else content
    elif analysis_type == "sentiment":
        # Placeholder sentiment analysis
        positive_words = ["good", "great", "excellent", "happy", "love"]
        negative_words = ["bad", "terrible", "hate", "angry", "sad"]
        words = content.lower().split()
        pos = sum(1 for w in words if w in positive_words)
        neg = sum(1 for w in words if w in negative_words)
        results["sentiment"] = "positive" if pos > neg else "negative" if neg > pos else "neutral"
    elif analysis_type == "keywords":
        # Simple keyword extraction (most common words > 4 chars)
        words = [w.lower() for w in content.split() if len(w) > 4]
        from collections import Counter

        results["keywords"] = [w for w, c in Counter(words).most_common(5)]

    return results


def save_analysis_report(
    filename: str, analysis_results: str, tool_context=None
) -> Dict:
    """
    Save analysis report to session state (native tool with state access).

    Args:
        filename: Name for the report
        analysis_results: Analysis content to save

    Returns:
        Confirmation of saved report.
    """
    if tool_context:
        reports = tool_context.state.get("reports", {})
        reports[filename] = {
            "content": analysis_results,
            "saved_at": "2024-01-15T10:30:00Z",  # Would use datetime in production
        }
        tool_context.state["reports"] = reports

    return {
        "status": "saved",
        "filename": filename,
        "report_count": len(tool_context.state.get("reports", {})) if tool_context else 1,
    }


def create_mcp_with_native_tools_agent(
    allowed_paths: Optional[List[str]] = None,
) -> LlmAgent:
    """
    Create an agent that combines MCP tools with native ADK tools.

    This pattern is powerful for:
    - Reading files via MCP, analyzing with native tools
    - Using MCP for external access, native tools for business logic
    - Maintaining state with native tools while using MCP for I/O

    Args:
        allowed_paths: Directories for MCP filesystem access

    Returns:
        LlmAgent with both MCP and native tools.
    """
    allowed_paths = allowed_paths or ["/tmp"]

    npx_args = ["-y", "@modelcontextprotocol/server-filesystem"]
    npx_args.extend(allowed_paths)

    mcp_toolset = McpToolset(
        connection_params=StdioConnectionParams(
            server_params=StdioServerParameters(
                command="npx",
                args=npx_args,
            )
        ),
        # Only expose read operations for safety
        tool_filter=["read_file", "list_directory", "get_file_info"],
    )

    return LlmAgent(
        model="gemini-2.5-flash",
        name="file_analyzer",
        instruction="""You are a file analysis assistant.

Workflow:
1. Use list_directory to explore available files
2. Use read_file to get file contents
3. Use analyze_content to analyze the text (summary, sentiment, or keywords)
4. Use save_analysis_report to store results

Always analyze files before reporting findings.
Provide clear summaries of your analysis.""",
        tools=[
            mcp_toolset,
            analyze_content,
            save_analysis_report,
        ],
    )


# =============================================================================
# SSE CONNECTION (REMOTE MCP SERVERS)
# =============================================================================


def create_remote_mcp_agent(
    server_url: str,
    auth_token: Optional[str] = None,
    tool_filter: Optional[List[str]] = None,
) -> LlmAgent:
    """
    Create an agent connected to a remote MCP server via SSE.

    Use this for:
    - Cloud-hosted MCP servers
    - Shared enterprise tools
    - Third-party integrations

    Args:
        server_url: SSE endpoint URL for the MCP server
        auth_token: Bearer token for authentication
        tool_filter: Tools to expose from the server

    Returns:
        LlmAgent connected to remote MCP server.
    """
    headers = {}
    if auth_token:
        headers["Authorization"] = f"Bearer {auth_token}"

    connection_config = {"url": server_url}
    if headers:
        connection_config["headers"] = headers

    mcp_config = {
        "connection_params": SseConnectionParams(**connection_config),
    }
    if tool_filter:
        mcp_config["tool_filter"] = tool_filter

    return LlmAgent(
        model="gemini-2.5-flash",
        name="remote_tools_agent",
        instruction="""You are an assistant with access to remote tools.
Use the available tools to help users accomplish their tasks.
Always verify results and handle errors gracefully.""",
        tools=[McpToolset(**mcp_config)],
    )


# =============================================================================
# ERROR HANDLING WRAPPER
# =============================================================================


class McpConnectionManager:
    """
    Manages MCP connections with error handling and reconnection.

    Use this wrapper when you need:
    - Automatic reconnection on failure
    - Connection health monitoring
    - Graceful degradation
    """

    def __init__(
        self,
        connection_params: StdioConnectionParams | SseConnectionParams,
        max_retries: int = 3,
        retry_delay: float = 1.0,
    ):
        self.connection_params = connection_params
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        self._connected = False

    async def connect_with_retry(self) -> McpToolset:
        """
        Attempt to connect with retries.

        Returns:
            Connected McpToolset

        Raises:
            ConnectionError: After all retries exhausted
        """
        last_error = None

        for attempt in range(self.max_retries):
            try:
                print(f"MCP connection attempt {attempt + 1}/{self.max_retries}")
                toolset = McpToolset(connection_params=self.connection_params)
                self._connected = True
                print("MCP connection successful")
                return toolset

            except Exception as e:
                last_error = e
                print(f"Connection failed: {e}")
                if attempt < self.max_retries - 1:
                    print(f"Retrying in {self.retry_delay}s...")
                    await asyncio.sleep(self.retry_delay)

        raise ConnectionError(
            f"Failed to connect after {self.max_retries} attempts: {last_error}"
        )

    @property
    def is_connected(self) -> bool:
        return self._connected


# =============================================================================
# DEMONSTRATION
# =============================================================================


async def demonstrate_mcp_filesystem():
    """Demonstrate MCP filesystem integration."""

    print("\n" + "=" * 60)
    print("MCP FILESYSTEM INTEGRATION DEMO")
    print("=" * 60)

    # Check if npx is available
    npx_available = os.system("which npx > /dev/null 2>&1") == 0
    if not npx_available:
        print("\nWARNING: npx not found. Install Node.js to run MCP servers.")
        print("Showing configuration only.\n")

    # Create test files
    test_dir = "/tmp/mcp_demo"
    os.makedirs(test_dir, exist_ok=True)

    with open(f"{test_dir}/sample.txt", "w") as f:
        f.write("This is a sample file for MCP demonstration.\n")
        f.write("It contains multiple lines of text.\n")
        f.write("Great for testing file operations!")

    with open(f"{test_dir}/data.json", "w") as f:
        f.write('{"name": "test", "value": 42}')

    print(f"\nCreated test files in {test_dir}")

    if not npx_available:
        print("\nSkipping live demo - npx not available")
        return

    # Create agent
    agent = create_mcp_filesystem_agent(
        allowed_paths=[test_dir],
        tool_filter=["read_file", "list_directory"],
    )

    session_service = InMemorySessionService()
    runner = Runner(
        app_name="mcp_demo",
        agent=agent,
        session_service=session_service,
    )

    session = await session_service.create_session(
        app_name="mcp_demo",
        user_id="demo_user",
    )

    # Test queries
    queries = [
        f"What files are in {test_dir}?",
        f"Read the contents of {test_dir}/sample.txt",
    ]

    for query in queries:
        print(f"\n--- Query: {query} ---")

        message = types.Content(
            role="user",
            parts=[types.Part(text=query)],
        )

        try:
            response = ""
            async for event in runner.run_async(
                user_id="demo_user",
                session_id=session.id,
                new_message=message,
            ):
                if event.content and event.content.parts:
                    for part in event.content.parts:
                        if part.text:
                            response += part.text

            print(f"Response: {response[:300]}...")

        except Exception as e:
            print(f"Error: {e}")


async def demonstrate_combined_tools():
    """Demonstrate combining MCP with native tools."""

    print("\n" + "=" * 60)
    print("COMBINED MCP + NATIVE TOOLS DEMO")
    print("=" * 60)

    # This demonstrates the pattern even if MCP isn't available
    print("\nAgent configuration:")
    print("- MCP tools: read_file, list_directory, get_file_info")
    print("- Native tools: analyze_content, save_analysis_report")
    print("\nWorkflow: Read file (MCP) -> Analyze (Native) -> Save (Native)")

    # Test the native tools directly
    test_content = "This is excellent content! I love how great it explains the concept."

    print("\n--- Testing native analyze_content ---")
    for analysis_type in ["summary", "sentiment", "keywords"]:
        result = analyze_content(test_content, analysis_type)
        print(f"{analysis_type}: {result}")


if __name__ == "__main__":
    print("Google ADK MCP Integration Examples")
    print("-" * 40)

    # Run demos
    asyncio.run(demonstrate_mcp_filesystem())
    asyncio.run(demonstrate_combined_tools())
