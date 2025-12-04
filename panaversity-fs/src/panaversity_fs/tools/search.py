"""Search operation tools for PanaversityFS.

Implements 2 MCP tools for content search:
- glob_search: File pattern matching with glob syntax
- grep_search: Content regex search across markdown files
"""

from mcp.server.fastmcp.server import Context

from panaversity_fs.app import mcp
from panaversity_fs.models import GlobSearchInput, GrepSearchInput, OperationType, OperationStatus
from panaversity_fs.storage import get_operator
from panaversity_fs.audit import log_operation
from datetime import datetime, timezone
import json
import re
from fnmatch import fnmatch


@mcp.tool(
    name="glob_search",
    annotations={
        "title": "Search Files by Pattern",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def glob_search(params: GlobSearchInput, ctx: Context) -> str:
    """Search for files matching glob pattern (FR-026).

    Supports glob patterns like **/*.md, assets/images/**/*.png.
    Returns matching file paths within specified book scope.

    Args:
        params (GlobSearchInput): Validated input containing:
            - book_id (str): Book identifier
            - pattern (str): Glob pattern (e.g., '**/*.md')
            - all_books (bool): Search across all books (default: False)

    Returns:
        str: JSON array of matching file paths

    Example:
        ```
        Input: {
          "book_id": "ai-native-python",
          "pattern": "**/*.md"
        }
        Output: [
          "books/ai-native-python/lessons/part-1/chapter-01/lesson-01.md",
          "books/ai-native-python/lessons/part-1/chapter-01/lesson-02.md",
          "books/ai-native-python/chapters/chapter-01/.summary.md"
        ]

        Input: {
          "book_id": "ai-native-python",
          "pattern": "assets/images/**/*.png"
        }
        Output: [
          "books/ai-native-python/assets/images/diagram.png",
          "books/ai-native-python/assets/images/screenshots/ui.png"
        ]
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Get operator
        op = get_operator()

        # Determine search scope
        if params.all_books:
            search_base = "books/"
        else:
            search_base = f"books/{params.book_id}/"

        # Collect matching files
        matches = []

        try:
            # Scan all files recursively (scan() is recursive, list() is not)
            entries = await op.scan(search_base)

            async for entry in entries:
                # Skip directories
                if entry.path.endswith('/'):
                    continue

                # Extract relative path from search base
                if entry.path.startswith(search_base):
                    relative_path = entry.path[len(search_base):]

                    # Match against glob pattern
                    # Handle ** patterns by checking if pattern matches any part of path
                    if '**' in params.pattern:
                        # Convert glob pattern to regex-like matching
                        if _glob_matches(relative_path, params.pattern):
                            matches.append(entry.path)
                    else:
                        # Simple fnmatch for non-recursive patterns
                        if fnmatch(relative_path, params.pattern):
                            matches.append(entry.path)

        except Exception:
            # Directory doesn't exist
            pass

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.GLOB_SEARCH,
            path=search_base,
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time
        )

        return json.dumps(matches, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.GLOB_SEARCH,
            path=f"books/{params.book_id}/",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return f"Error in glob search: {type(e).__name__}: {str(e)}"


def _glob_matches(path: str, pattern: str) -> bool:
    """Check if path matches glob pattern with ** support.

    Args:
        path: Relative file path
        pattern: Glob pattern with potential **

    Returns:
        bool: True if path matches pattern
    """
    # Convert glob pattern to regex
    # Order matters: process ** before * to avoid double-replacement

    # First, escape special regex chars except glob wildcards
    regex_pattern = re.escape(pattern)

    # Now unescape and convert glob patterns
    # \*\*/ -> match zero or more path segments
    regex_pattern = regex_pattern.replace(r'\*\*/', '(?:[^/]+/)*')
    # \*\* -> match everything (including /)
    regex_pattern = regex_pattern.replace(r'\*\*', '.*')
    # \* -> match anything except /
    regex_pattern = regex_pattern.replace(r'\*', '[^/]*')
    # \? -> match single char except /
    regex_pattern = regex_pattern.replace(r'\?', '[^/]')

    regex_pattern = f'^{regex_pattern}$'

    return bool(re.match(regex_pattern, path))


@mcp.tool(
    name="grep_search",
    annotations={
        "title": "Search Content by Regex",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def grep_search(params: GrepSearchInput, ctx: Context) -> str:
    """Search content using regex pattern (FR-027).

    Searches across all lesson markdown files and returns matches with context.

    Args:
        params (GrepSearchInput): Validated input containing:
            - book_id (str): Book identifier
            - pattern (str): Regex pattern to search for
            - all_books (bool): Search across all books (default: False)
            - max_results (int): Maximum results to return (default: 100, max: 1000)

    Returns:
        str: JSON array of matches with file_path, line_number, matched_line_content

    Example:
        ```
        Input: {
          "book_id": "ai-native-python",
          "pattern": "OpenDAL"
        }
        Output: [
          {
            "file_path": "books/ai-native-python/lessons/part-4/chapter-12/lesson-03.md",
            "line_number": 42,
            "matched_line": "...OpenDAL provides unified storage abstraction..."
          },
          {
            "file_path": "books/ai-native-python/lessons/part-4/chapter-12/lesson-04.md",
            "line_number": 15,
            "matched_line": "We use OpenDAL to access S3 and local filesystem..."
          }
        ]

        Input: {
          "book_id": "ai-native-python",
          "pattern": "def\\s+\\w+\\(",
          "max_results": 50
        }
        Output: [
          {
            "file_path": "books/ai-native-python/lessons/part-3/chapter-08/lesson-01.md",
            "line_number": 25,
            "matched_line": "def hello_world():"
          }
        ]
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Compile regex pattern
        try:
            regex = re.compile(params.pattern)
        except re.error as e:
            return json.dumps({
                "error": "invalid_regex",
                "message": f"Invalid regex pattern: {str(e)}"
            }, indent=2)

        # Get operator
        op = get_operator()

        # Determine search scope
        if params.all_books:
            search_base = "books/"
        else:
            search_base = f"books/{params.book_id}/"

        # Collect matches
        matches = []
        result_count = 0

        try:
            # Scan all files recursively (scan() is recursive, list() is not)
            entries = await op.scan(search_base)

            async for entry in entries:
                # Only search markdown files
                if not entry.path.endswith('.md'):
                    continue

                # Skip directories
                if entry.path.endswith('/'):
                    continue

                try:
                    # Read file content
                    content_bytes = await op.read(entry.path)
                    content = content_bytes.decode('utf-8')

                    # Search line by line
                    for line_num, line in enumerate(content.splitlines(), start=1):
                        if regex.search(line):
                            matches.append({
                                "file_path": entry.path,
                                "line_number": line_num,
                                "matched_line": line.strip()
                            })

                            result_count += 1

                            # Check max results limit
                            if result_count >= params.max_results:
                                # Add truncation warning
                                if result_count == params.max_results:
                                    matches.append({
                                        "warning": "Results truncated",
                                        "message": f"Showing first {params.max_results} results. Refine pattern to see more."
                                    })
                                break

                except Exception:
                    # Skip files that can't be read
                    continue

                # Break outer loop if max results reached
                if result_count >= params.max_results:
                    break

        except Exception:
            # Directory doesn't exist
            pass

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.GREP_SEARCH,
            path=search_base,
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time
        )

        return json.dumps(matches, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.GREP_SEARCH,
            path=f"books/{params.book_id}/",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return f"Error in grep search: {type(e).__name__}: {str(e)}"
