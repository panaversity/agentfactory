"""Content operation tools for PanaversityFS.

Implements 3 MCP tools for content management (lessons and summaries per ADR-0018):
- read_content: Read markdown content with metadata
- write_content: Upsert with conflict detection (file_hash)
- delete_content: Delete content file

Path structure (Docusaurus-aligned):
- Lessons: content/{part}/{chapter}/{lesson}.md
- Summaries: content/{part}/{chapter}/{lesson}.summary.md
"""

from panaversity_fs.app import mcp
from panaversity_fs.models import ReadContentInput, WriteContentInput, DeleteContentInput, ContentMetadata, ContentScope
from panaversity_fs.storage import get_operator
from panaversity_fs.storage_utils import compute_sha256, validate_path
from panaversity_fs.errors import ContentNotFoundError, ConflictError, InvalidPathError
from panaversity_fs.audit import log_operation
from panaversity_fs.models import OperationType, OperationStatus
from panaversity_fs.config import get_config
from datetime import datetime, timezone
import json
import fnmatch


@mcp.tool(
    name="read_content",
    annotations={
        "title": "Read Content",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def read_content(params: ReadContentInput) -> str:
    """Read markdown content with metadata (FR-009).

    Supports four scopes:
    - file (default): Read single file, return content + metadata
    - chapter: Read all .md files in a chapter directory
    - part: Read all .md files in a part directory (all chapters)
    - book: Read all .md files in the entire book's content/ directory

    Args:
        params (ReadContentInput): Validated input containing:
            - book_id (str): Book identifier (e.g., 'ai-native-python')
            - path (str): Content path relative to book root (ignored for book scope)
            - scope (ContentScope): file/chapter/part/book (default: file)

    Returns:
        str: JSON-formatted response
            - scope=file: Single ContentMetadata object
            - scope=chapter/part/book: Array of ContentMetadata objects with path field

    Example:
        ```
        # Read single file (default)
        Input: {"book_id": "my-book", "path": "content/01-Part/01-Chapter/01-lesson.md"}
        Output: {"content": "...", "file_size": 1234, ...}

        # Read entire chapter
        Input: {"book_id": "my-book", "path": "content/01-Part/01-Chapter", "scope": "chapter"}
        Output: [
          {"path": "content/01-Part/01-Chapter/README.md", "content": "...", ...},
          {"path": "content/01-Part/01-Chapter/01-lesson.md", "content": "...", ...}
        ]

        # Read entire part
        Input: {"book_id": "my-book", "path": "content/01-Part", "scope": "part"}
        Output: [
          {"path": "content/01-Part/README.md", "content": "...", ...},
          {"path": "content/01-Part/01-Chapter/README.md", "content": "...", ...},
          ...
        ]

        # Read entire book
        Input: {"book_id": "my-book", "scope": "book"}
        Output: [
          {"path": "content/01-Part/README.md", "content": "...", ...},
          {"path": "content/01-Part/01-Chapter/README.md", "content": "...", ...},
          {"path": "content/01-Part/01-Chapter/01-lesson.md", "content": "...", ...},
          {"path": "content/02-Part/README.md", "content": "...", ...},
          ...
        ]
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Validate path
        if not validate_path(params.path):
            raise InvalidPathError(params.path, "Path contains invalid characters or traversal attempts")

        # Get operator
        op = get_operator()
        config = get_config()

        # Handle different scopes
        if params.scope == ContentScope.FILE:
            # Original single-file behavior
            full_path = f"books/{params.book_id}/{params.path}"

            # Read content
            content_bytes = await op.read(full_path)
            content = content_bytes.decode('utf-8')

            # Get metadata
            metadata = await op.stat(full_path)

            # Compute hash
            file_hash = compute_sha256(content_bytes)

            # Build response
            response = ContentMetadata(
                file_size=metadata.content_length,
                last_modified=metadata.last_modified,
                storage_backend=config.storage_backend,
                file_hash_sha256=file_hash,
                content=content
            )

            # Log success
            execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
            await log_operation(
                operation=OperationType.READ_CONTENT,
                path=full_path,
                agent_id="system",
                status=OperationStatus.SUCCESS,
                execution_time_ms=execution_time
            )

            return response.model_dump_json(indent=2)

        else:
            # Bulk read: chapter, part, or book scope
            if params.scope == ContentScope.BOOK:
                # For book scope, always scan content/ directory
                base_path = f"books/{params.book_id}/content/"
            else:
                base_path = f"books/{params.book_id}/{params.path.rstrip('/')}/"
            results = []

            # List entries: use scan() for PART/BOOK (recursive), list() for CHAPTER (one level)
            try:
                if params.scope in (ContentScope.PART, ContentScope.BOOK):
                    # Recursive scan for parts and books (includes all subdirectories)
                    entries = await op.scan(base_path)
                else:
                    # Non-recursive list for chapters (only direct children)
                    entries = await op.list(base_path)

                async for entry in entries:
                    # Skip directories
                    if entry.path.endswith('/'):
                        continue

                    # Only process .md files
                    if not entry.path.endswith('.md'):
                        continue

                    # For chapter scope, only include files directly in the chapter
                    if params.scope == ContentScope.CHAPTER:
                        # Check if file is directly in this directory (not in a subdirectory)
                        rel_path = entry.path[len(base_path):]
                        if '/' in rel_path:
                            continue

                    try:
                        # Read file content
                        content_bytes = await op.read(entry.path)
                        content = content_bytes.decode('utf-8')

                        # Get metadata
                        metadata = await op.stat(entry.path)

                        # Compute hash
                        file_hash = compute_sha256(content_bytes)

                        # Extract relative path from book root
                        rel_path = entry.path.replace(f"books/{params.book_id}/", "")

                        # Build result with path included
                        result = {
                            "path": rel_path,
                            "content": content,
                            "file_size": metadata.content_length,
                            "last_modified": metadata.last_modified.isoformat() if metadata.last_modified else None,
                            "storage_backend": config.storage_backend,
                            "file_hash_sha256": file_hash
                        }
                        results.append(result)

                    except Exception:
                        # Skip files that can't be read
                        continue

            except Exception:
                # Directory doesn't exist
                raise ContentNotFoundError(base_path)

            # Sort results by path for consistent ordering
            results.sort(key=lambda x: x["path"])

            # Log success
            execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
            await log_operation(
                operation=OperationType.READ_CONTENT,
                path=base_path,
                agent_id="system",
                status=OperationStatus.SUCCESS,
                execution_time_ms=execution_time
            )

            return json.dumps(results, indent=2)

    except FileNotFoundError:
        # Log error
        await log_operation(
            operation=OperationType.READ_CONTENT,
            path=f"books/{params.book_id}/{params.path}",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message="Content not found"
        )

        raise ContentNotFoundError(f"books/{params.book_id}/{params.path}")

    except ContentNotFoundError:
        # Re-raise without wrapping
        raise

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.READ_CONTENT,
            path=f"books/{params.book_id}/{params.path}",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return f"Error reading content: {type(e).__name__}: {str(e)}"


@mcp.tool(
    name="write_content",
    annotations={
        "title": "Write Content (Upsert)",
        "readOnlyHint": False,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def write_content(params: WriteContentInput) -> str:
    """Write content with upsert semantics and conflict detection (FR-007, FR-008).

    Works for lessons and summaries (ADR-0018).

    Supports two modes:
    - Update mode (file_hash provided): Verify hash matches before write, detect conflicts
    - Create mode (file_hash omitted): Create new file or overwrite existing

    Args:
        params (WriteContentInput): Validated input containing:
            - book_id (str): Book identifier
            - path (str): Content path relative to book root
            - content (str): Markdown content
            - file_hash (str | None): SHA256 hash for conflict detection (optional)

    Returns:
        str: Success message with file metadata

    Example:
        ```
        # Create lesson
        Input: {
          "book_id": "my-book",
          "path": "content/01-Part/01-Chapter/01-lesson.md",
          "content": "# Lesson 1\\n\\nContent..."
        }

        # Create summary for that lesson
        Input: {
          "book_id": "my-book",
          "path": "content/01-Part/01-Chapter/01-lesson.summary.md",
          "content": "# Summary\\n\\nKey points..."
        }

        # Update with conflict detection
        Input: {
          "book_id": "my-book",
          "path": "content/01-Part/01-Chapter/01-lesson.md",
          "content": "# Lesson 1 (Updated)\\n\\nNew content...",
          "file_hash": "a591a6d40bf420404a011733cfb7b190..."
        }
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Validate path
        if not validate_path(params.path):
            raise InvalidPathError(params.path, "Path contains invalid characters or traversal attempts")

        # Build full path
        full_path = f"books/{params.book_id}/{params.path}"

        # Get operator
        op = get_operator()

        # If file_hash provided, verify it matches (conflict detection)
        if params.file_hash:
            try:
                existing_content = await op.read(full_path)
                existing_hash = compute_sha256(existing_content)

                if existing_hash != params.file_hash:
                    # Conflict detected
                    await log_operation(
                        operation=OperationType.WRITE_CONTENT,
                        path=full_path,
                        agent_id="system",
                        status=OperationStatus.CONFLICT,
                        error_message=f"Hash mismatch: expected {params.file_hash}, got {existing_hash}"
                    )

                    raise ConflictError(full_path, params.file_hash, existing_hash)

            except FileNotFoundError:
                # File doesn't exist, can't verify hash - treat as create
                pass

        # Write content
        content_bytes = params.content.encode('utf-8')
        await op.write(full_path, content_bytes)

        # Get metadata of written file
        metadata = await op.stat(full_path)
        new_hash = compute_sha256(content_bytes)

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=full_path,
            agent_id="system",
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time
        )

        # Build response
        response = {
            "status": "success",
            "path": full_path,
            "file_size": metadata.content_length,
            "file_hash": new_hash,
            "mode": "updated" if params.file_hash else "created"
        }

        return json.dumps(response, indent=2)

    except ConflictError:
        raise  # Re-raise ConflictError as-is

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{params.book_id}/{params.path}",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return f"Error writing content: {type(e).__name__}: {str(e)}"


@mcp.tool(
    name="delete_content",
    annotations={
        "title": "Delete Content",
        "readOnlyHint": False,
        "destructiveHint": True,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def delete_content(params: DeleteContentInput) -> str:
    """Delete content file (lesson or summary).

    Idempotent: Deleting non-existent file returns success.
    Works for lessons and summaries (ADR-0018).

    Args:
        params (DeleteContentInput): Validated input containing:
            - book_id (str): Book identifier
            - path (str): Content path to delete

    Returns:
        str: Success confirmation message

    Example:
        ```
        # Delete lesson
        Input: {"book_id": "my-book", "path": "content/01-Part/01-Chapter/01-lesson.md"}

        # Delete summary
        Input: {"book_id": "my-book", "path": "content/01-Part/01-Chapter/01-lesson.summary.md"}

        Output: {"status": "success", "path": "books/my-book/...", "existed": true}
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Validate path
        if not validate_path(params.path):
            raise InvalidPathError(params.path, "Path contains invalid characters or traversal attempts")

        # Build full path
        full_path = f"books/{params.book_id}/{params.path}"

        # Get operator
        op = get_operator()

        # Check if file exists
        existed = True
        try:
            await op.stat(full_path)
        except:
            existed = False

        # Delete file (idempotent)
        await op.delete(full_path)

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.DELETE_CONTENT,
            path=full_path,
            agent_id="system",
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time
        )

        # Build response
        response = {
            "status": "success",
            "path": full_path,
            "existed": existed,
            "message": f"File {'deleted' if existed else 'did not exist (idempotent delete)'}"
        }

        return json.dumps(response, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.DELETE_CONTENT,
            path=f"books/{params.book_id}/{params.path}",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return f"Error deleting content: {type(e).__name__}: {str(e)}"
