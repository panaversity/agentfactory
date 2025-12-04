"""FastMCP server for PanaversityFS.

Main entry point for the MCP server with Stateless Streamable HTTP transport.

Follows MCP SDK best practices:
- Lifespan management for database connections (in app.py)
- Session manager lifecycle for Starlette mount
- Proper resource cleanup on shutdown
"""

import contextlib

from starlette.applications import Starlette
from starlette.middleware.cors import CORSMiddleware
from starlette.routing import Mount

from panaversity_fs.app import mcp  # Import from app.py to avoid double-import issue
from panaversity_fs.config import get_config

# Import all tool modules to register their @mcp.tool() decorators
# These imports have side effects: they register tools with the mcp instance
import panaversity_fs.tools.content  # noqa: F401 - read_content, write_content, delete_content
import panaversity_fs.tools.assets   # noqa: F401 - upload_asset, get_asset, list_assets
import panaversity_fs.tools.registry # noqa: F401 - list_books
import panaversity_fs.tools.search   # noqa: F401 - glob_search, grep_search
import panaversity_fs.tools.bulk     # noqa: F401 - get_book_archive
import panaversity_fs.tools.validate # noqa: F401 - validate_book
import panaversity_fs.tools.delta    # noqa: F401 - delta_build, plan_build

# Load and validate configuration
config = get_config()


@contextlib.asynccontextmanager
async def starlette_lifespan(app: Starlette):
    """Manage Starlette app lifecycle with MCP session manager.

    This ensures the MCP session manager is properly started and stopped
    when running as an ASGI app via uvicorn.

    Following MCP SDK best practices for streamable HTTP transport.
    """
    async with mcp.session_manager.run():
        yield


# Create the Starlette app for ASGI servers (uvicorn)
# Using proper lifespan management for session cleanup
_starlette_app = Starlette(
    routes=[
        Mount("/", app=mcp.streamable_http_app()),
    ],
    lifespan=starlette_lifespan,
)

# Wrap with CORS middleware to allow MCP Inspector and other browser-based clients
# CORS middleware must be applied to allow OPTIONS preflight requests without auth
# Per MCP SDK docs: https://modelcontextprotocol.io/docs/tools/mcp-python-sdk
streamable_http_app = CORSMiddleware(
    _starlette_app,
    allow_origins=["*"],  # Allow all origins for MCP clients
    allow_methods=["GET", "POST", "DELETE", "OPTIONS"],
    allow_headers=["*"],  # Allow Authorization header
    expose_headers=["Mcp-Session-Id"],
)

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
    print(f"Auth: {'Enabled (JWT)' if config.auth_enabled else 'Disabled (Dev Mode)'}")
    print(f"Tools: {len(mcp._tool_manager._tools)} registered")

    # Use correct module path (not src.panaversity_fs, just panaversity_fs)
    uvicorn.run(
        "panaversity_fs.server:streamable_http_app",
        host=config.server_host,
        port=config.server_port,
        reload=True
    )