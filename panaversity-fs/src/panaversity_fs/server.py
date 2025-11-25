"""FastMCP server for PanaversityFS.

Main entry point for the MCP server with Stateless Streamable HTTP transport.
"""

from panaversity_fs.app import mcp  # Import from app.py to avoid double-import issue
from panaversity_fs.config import get_config

# Import all tool modules to register their @mcp.tool() decorators
# These imports have side effects: they register tools with the mcp instance
import panaversity_fs.tools.content  # noqa: F401 - read_content, write_content, delete_content
import panaversity_fs.tools.assets   # noqa: F401 - upload_asset, get_asset, list_assets
import panaversity_fs.tools.registry # noqa: F401 - list_books
import panaversity_fs.tools.search   # noqa: F401 - glob_search, grep_search
import panaversity_fs.tools.bulk     # noqa: F401 - get_book_archive

# Load and validate configuration
config = get_config()

# Create the Starlette app for ASGI servers (uvicorn)
streamable_http_app = mcp.streamable_http_app()

if __name__ == "__main__":
    """Run the MCP server.

    Usage:
        python -m panaversity_fs.server

    Server runs at http://0.0.0.0:8000/mcp by default.
    Configure via environment variables (PANAVERSITY_*).
    """
    import uvicorn

    print("PanaversityFS MCP Server")
    print(f"Storage Backend: {config.storage_backend}")
    print(f"Server: http://{config.server_host}:{config.server_port}/mcp")
    print(f"Auth: {'Enabled (API Key)' if config.api_key else 'Disabled (Dev Mode)'}")
    print(f"Tools: {len(mcp._tool_manager._tools)} registered")

    # Use correct module path (not src.panaversity_fs, just panaversity_fs)
    uvicorn.run(
        "panaversity_fs.server:streamable_http_app",
        host=config.server_host,
        port=config.server_port,
        reload=True
    )