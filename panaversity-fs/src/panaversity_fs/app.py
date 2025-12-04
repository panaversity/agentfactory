"""FastMCP application instance for PanaversityFS.

This module holds the singleton FastMCP instance that both the server
and all tool modules import. This prevents the double-import issue
when running as `python -m panaversity_fs.server`.

Issue: When running with `-m`, Python loads the main module as `__main__`,
but when tools do `from panaversity_fs.server import mcp`, Python loads
it again as `panaversity_fs.server`, creating two different mcp instances.

Solution: Move the mcp instance to a separate module (this file) that both
server.py and tools import consistently.

Authentication:
- If PANAVERSITY_JWT_SECRET is set: JWT auth is enabled
- If not set: Server runs in dev mode without auth
"""

from mcp.server.fastmcp import FastMCP
from panaversity_fs.config import get_config


def _create_mcp() -> FastMCP:
    """Create FastMCP instance with optional authentication.

    Returns:
        FastMCP: Configured MCP server instance
    """
    config = get_config()

    # Base configuration
    kwargs = {
        "stateless_http": True,  # Enable Stateless Streamable HTTP (FR-004)
        "json_response": True    # Disable SSE, use pure JSON responses
    }

    # Add authentication if configured
    if config.auth_enabled:
        from panaversity_fs.auth import get_auth_settings
        token_verifier, auth_settings = get_auth_settings()

        if token_verifier:
            kwargs["token_verifier"] = token_verifier
        if auth_settings:
            kwargs["auth"] = auth_settings

        print(f"[PanaversityFS] JWT authentication enabled")
        if config.auth_issuer:
            print(f"[PanaversityFS] Issuer: {config.auth_issuer}")
        print(f"[PanaversityFS] Required scopes: {config.required_scopes}")
    else:
        print("[PanaversityFS] Running in dev mode (no authentication)")

    return FastMCP("panaversity_fs", **kwargs)


# Initialize FastMCP server with stateless HTTP transport
# This singleton is imported by all tool modules
mcp = _create_mcp()
