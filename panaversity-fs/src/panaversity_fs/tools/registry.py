"""Registry management tools for PanaversityFS.

Implements 1 MCP tool for book registry operations:
- list_books: List all books by scanning books/ directory (dynamic discovery)
"""

from panaversity_fs.app import mcp
from panaversity_fs.models import ListBooksInput, OperationType, OperationStatus
from panaversity_fs.storage import get_operator
from panaversity_fs.audit import log_operation
from panaversity_fs.config import get_config
from datetime import datetime, timezone
import json


async def _get_content_structure(op, book_id: str) -> dict:
    """Build hierarchical content structure (parts/chapters/lessons)."""
    content_path = f"books/{book_id}/content/"
    structure = {"parts": []}

    try:
        # Scan content directory recursively
        entries = await op.scan(content_path)
        files = []
        async for entry in entries:
            if not entry.path.endswith('/'):
                # Get path relative to content/
                rel_path = entry.path[len(content_path):]
                files.append(rel_path)

        # Parse into hierarchical structure
        # Format: {Part}/{Chapter}/{Lesson}.md
        parts_dict = {}
        for file_path in sorted(files):
            parts = file_path.split('/')
            if len(parts) >= 1:
                part_id = parts[0] if len(parts) > 1 else None
                chapter_id = parts[1] if len(parts) > 2 else None
                lesson = parts[-1]

                if part_id:
                    if part_id not in parts_dict:
                        parts_dict[part_id] = {"id": part_id, "chapters": {}}

                    if chapter_id:
                        if chapter_id not in parts_dict[part_id]["chapters"]:
                            parts_dict[part_id]["chapters"][chapter_id] = {
                                "id": chapter_id,
                                "lessons": []
                            }
                        parts_dict[part_id]["chapters"][chapter_id]["lessons"].append(lesson)
                    else:
                        # File directly in part directory (e.g., README.md)
                        if "_files" not in parts_dict[part_id]:
                            parts_dict[part_id]["_files"] = []
                        parts_dict[part_id]["_files"].append(lesson)

        # Convert to list format
        for part_id in sorted(parts_dict.keys()):
            part_data = parts_dict[part_id]
            part_entry = {"id": part_id}
            if "_files" in part_data:
                part_entry["files"] = part_data["_files"]
            part_entry["chapters"] = [
                {"id": ch_id, "lessons": ch_data["lessons"]}
                for ch_id, ch_data in sorted(part_data["chapters"].items())
            ]
            structure["parts"].append(part_entry)

    except Exception:
        pass

    return structure


async def _get_assets_structure(op, book_id: str) -> dict:
    """Build assets structure (images/slides)."""
    static_path = f"books/{book_id}/static/"
    structure = {"images": [], "slides": [], "other": []}

    try:
        entries = await op.scan(static_path)
        async for entry in entries:
            if not entry.path.endswith('/'):
                rel_path = entry.path[len(static_path):]
                if rel_path.startswith("images/"):
                    structure["images"].append(rel_path[7:])  # Remove "images/" prefix
                elif rel_path.startswith("slides/"):
                    structure["slides"].append(rel_path[7:])  # Remove "slides/" prefix
                else:
                    structure["other"].append(rel_path)

        # Remove empty lists
        if not structure["other"]:
            del structure["other"]

    except Exception:
        pass

    return structure


@mcp.tool(
    name="list_books",
    annotations={
        "title": "List Books",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def list_books(params: ListBooksInput) -> str:
    """List all books by scanning books/ directory (FR-024).

    Dynamically discovers books by scanning subdirectories under books/.
    Each subdirectory name is treated as a book_id.

    Args:
        params (ListBooksInput): Input with optional parameters:
            - include_structure: "none", "content", "assets", or "all"
            - book_id: Filter to specific book (optional)

    Returns:
        str: JSON array of book entries with book_id, storage_backend, and optional structure

    Example:
        ```
        Input: {"include_structure": "all", "book_id": "ai-native-dev"}
        Output: [
          {
            "book_id": "ai-native-dev",
            "storage_backend": "s3",
            "content": {
              "parts": [
                {"id": "01-Part", "chapters": [{"id": "01-chapter", "lessons": ["01-lesson.md"]}]}
              ]
            },
            "assets": {
              "images": ["diagram.png"],
              "slides": ["chapter-01.pdf"]
            }
          }
        ]
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Get operator and config
        op = get_operator()
        config = get_config()

        # List books/ directory
        books_path = "books/"
        book_list = []

        try:
            # List immediate children of books/
            entries = await op.list(books_path)

            async for entry in entries:
                # Only include directories (paths ending with /)
                if entry.path.endswith('/'):
                    # Extract book_id from path: "books/ai-native-python/" -> "ai-native-python"
                    path_parts = entry.path.rstrip('/').split('/')
                    if len(path_parts) >= 2:
                        book_id = path_parts[-1]
                        # Skip hidden directories and special directories
                        if not book_id.startswith('.') and book_id != 'books':
                            # Filter by book_id if specified
                            if params.book_id and params.book_id != book_id:
                                continue

                            book_entry = {
                                "book_id": book_id,
                                "storage_backend": config.storage_backend
                            }

                            # Add structure if requested
                            include = params.include_structure or "none"
                            if include in ("content", "all"):
                                book_entry["content"] = await _get_content_structure(op, book_id)
                            if include in ("assets", "all"):
                                book_entry["assets"] = await _get_assets_structure(op, book_id)

                            book_list.append(book_entry)

        except Exception:
            # books/ directory doesn't exist yet, return empty array
            pass

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.LIST_BOOKS,
            path=books_path,
            agent_id="system",
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time
        )

        return json.dumps(book_list, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.LIST_BOOKS,
            path="books/",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return f"Error listing books: {type(e).__name__}: {str(e)}"
