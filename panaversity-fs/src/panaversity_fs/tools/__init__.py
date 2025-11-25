"""MCP tools for PanaversityFS.

This package contains 12 MCP tool implementations organized by category:
- content.py: Content operations (read_content, write_content, delete_content)
- summaries.py: Summary operations (read_summary, write_summary, delete_summary)
- assets.py: Asset management (upload_asset, get_asset, list_assets)
- search.py: Search operations (glob_search, grep_search)
- registry.py: Registry operations (list_books)
- bulk.py: Bulk operations (get_book_archive)

Each tool module imports the shared FastMCP instance from server.py
and registers tools using @mcp.tool decorator.

See docs/MCP-TOOLS.md for complete API documentation.
"""

__all__ = []
