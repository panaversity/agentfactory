"""MCP tools for PanaversityFS.

This package contains 13 MCP tool implementations organized by category (ADR-0018):
- content.py: Content operations (read_content, write_content, delete_content)
              Also handles summaries via .summary.md naming convention
- assets.py: Asset management (upload_asset, get_asset, list_assets)
- search.py: Search operations (glob_search, grep_search)
- registry.py: Registry operations (list_books)
- bulk.py: Bulk operations (get_book_archive)
- validate.py: Schema validation (validate_book)
- delta.py: Delta build detection (delta_build, plan_build)

Each tool module imports the shared FastMCP instance from server.py
and registers tools using @mcp.tool decorator.

See docs/MCP-TOOLS.md for complete API documentation.
"""

__all__ = []
