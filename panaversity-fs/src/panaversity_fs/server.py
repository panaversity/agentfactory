"""FastMCP server for PanaversityFS.

Main entry point for the MCP server with Stateless Streamable HTTP transport.
"""

from mcp.server.fastmcp import FastMCP
from panaversity_fs.config import get_config
import sys

# Initialize FastMCP server with stateless HTTP transport
# This singleton is imported by all tool modules
mcp = FastMCP(
    "panaversity_fs",
    stateless_http=True,      # Enable Stateless Streamable HTTP (FR-004)
    json_response=True        # Disable SSE, use pure JSON responses
)


def main():
    """Run the MCP server.

    Usage:
        python -m panaversity_fs.server

    Server runs at http://0.0.0.0:8000/mcp by default.
    Configure via environment variables (PANAVERSITY_*).
    """
    try:
        # Load and validate configuration
        config = get_config()

        print(f"PanaversityFS MCP Server", file=sys.stderr)
        print(f"Storage Backend: {config.storage_backend}", file=sys.stderr)
        print(f"Server: http://{config.server_host}:{config.server_port}/mcp", file=sys.stderr)
        print(f"Auth: {'Enabled (API Key)' if config.api_key else 'Disabled (Dev Mode)'}", file=sys.stderr)
        print(f"", file=sys.stderr)
        print(f"Importing tools...", file=sys.stderr)

        # Import all tool modules to register tools
        # This must happen after mcp is created
        from panaversity_fs.tools import content
        from panaversity_fs.tools import assets
        from panaversity_fs.tools import summaries
        # Additional tool imports will be added as they're implemented

        print(f"Tools registered successfully", file=sys.stderr)
        print(f"Starting server...", file=sys.stderr)

        # Run server with streamable HTTP transport
        mcp.run(transport="streamable-http")

    except Exception as e:
        print(f"ERROR: Failed to start server: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
