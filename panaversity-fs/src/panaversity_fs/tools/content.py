"""Content operation tools for PanaversityFS.

Implements 3 MCP tools for content management (lessons and summaries per ADR-0018):
- read_content: Read markdown content with metadata
- write_content: Upsert with conflict detection via FileJournal (FR-002, FR-003, FR-004, FR-005)
- delete_content: Delete content file

Path structure (Docusaurus-aligned):
- Lessons: content/{part}/{chapter}/{lesson}.md
- Summaries: content/{part}/{chapter}/{lesson}.summary.md
"""

from panaversity_fs.app import mcp
from panaversity_fs.models import ReadContentInput, WriteContentInput, DeleteContentInput, ContentMetadata, ContentScope
from panaversity_fs.storage import get_operator
from panaversity_fs.storage_utils import compute_sha256, validate_path
from panaversity_fs.errors import ContentNotFoundError, ConflictError, InvalidPathError, HashRequiredError
from panaversity_fs.audit import log_operation
from panaversity_fs.models import OperationType, OperationStatus
from panaversity_fs.config import get_config
from panaversity_fs.database.connection import get_session
from panaversity_fs.database.models import FileJournal
from panaversity_fs.metrics import instrument_write
from sqlalchemy import select
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
@instrument_write
async def write_content(params: WriteContentInput) -> str:
    """Write content with journal-backed conflict detection (FR-002, FR-003, FR-004, FR-005).

    Works for lessons and summaries (ADR-0018).

    Conflict detection protocol:
    - FR-003: If expected_hash provided, verify it matches journal hash before write
    - FR-004: If expected_hash omitted AND file exists in journal, reject with HASH_REQUIRED
    - FR-005: If expected_hash omitted AND file doesn't exist, create operation succeeds
    - FR-002: Journal entry recorded BEFORE returning success; atomic rollback on failure

    Args:
        params (WriteContentInput): Validated input containing:
            - book_id (str): Book identifier
            - path (str): Content path relative to book root
            - content (str): Markdown content
            - expected_hash (str | None): SHA256 hash for conflict detection (REQUIRED for updates)

    Returns:
        str: JSON response with status, path, file_size, file_hash, mode ("created"|"updated")

    Example:
        ```
        # Create lesson (no expected_hash needed for new files)
        Input: {
          "book_id": "my-book",
          "path": "content/01-Part/01-Chapter/01-lesson.md",
          "content": "# Lesson 1\\n\\nContent..."
        }
        Output: {"status": "success", "mode": "created", ...}

        # Update with conflict detection (expected_hash REQUIRED)
        Input: {
          "book_id": "my-book",
          "path": "content/01-Part/01-Chapter/01-lesson.md",
          "content": "# Lesson 1 (Updated)\\n\\nNew content...",
          "expected_hash": "a591a6d40bf420404a011733cfb7b190..."
        }
        Output: {"status": "success", "mode": "updated", ...}
        ```

    Raises:
        ConflictError: expected_hash doesn't match current journal hash (FR-003)
        HashRequiredError: Updating existing file without expected_hash (FR-004)
        InvalidPathError: Path contains traversal or invalid characters
    """
    start_time = datetime.now(timezone.utc)
    config = get_config()

    try:
        # Validate path
        if not validate_path(params.path):
            raise InvalidPathError(params.path, "Path contains invalid characters or traversal attempts")

        # Build full path
        full_path = f"books/{params.book_id}/{params.path}"

        # Get operator
        op = get_operator()

        # Compute new content hash before any DB operations
        content_bytes = params.content.encode('utf-8')
        new_hash = compute_sha256(content_bytes)

        # Use atomic transaction for journal + storage (FR-002)
        async with get_session() as session:
            # Query FileJournal for existing entry (FR-002)
            stmt = select(FileJournal).where(
                FileJournal.book_id == params.book_id,
                FileJournal.path == params.path,
                FileJournal.user_id == "__base__"  # Base content, not overlay
            )
            result = await session.execute(stmt)
            existing_entry = result.scalar_one_or_none()

            # Determine mode and validate hash requirements
            if existing_entry:
                # File exists in journal
                if params.expected_hash is None:
                    # FR-004: Reject update without expected_hash
                    await log_operation(
                        operation=OperationType.WRITE_CONTENT,
                        path=full_path,
                        agent_id="system",
                        status=OperationStatus.ERROR,
                        error_message=f"HASH_REQUIRED: Cannot update existing file without expected_hash"
                    )
                    raise HashRequiredError(full_path, existing_entry.sha256)

                if params.expected_hash != existing_entry.sha256:
                    # FR-003: Conflict detected - hash mismatch
                    await log_operation(
                        operation=OperationType.WRITE_CONTENT,
                        path=full_path,
                        agent_id="system",
                        status=OperationStatus.CONFLICT,
                        error_message=f"Hash mismatch: expected {params.expected_hash}, journal has {existing_entry.sha256}"
                    )
                    raise ConflictError(full_path, params.expected_hash, existing_entry.sha256)

                # Valid update: hash matches
                mode = "updated"
                existing_entry.sha256 = new_hash
                existing_entry.last_written_at = datetime.now(timezone.utc)
                existing_entry.storage_backend = config.storage_backend

            else:
                # FR-005: File doesn't exist - create operation
                if params.expected_hash is not None:
                    # User provided expected_hash for non-existent file
                    # This could indicate they thought file existed - warn them
                    await log_operation(
                        operation=OperationType.WRITE_CONTENT,
                        path=full_path,
                        agent_id="system",
                        status=OperationStatus.ERROR,
                        error_message=f"NOT_FOUND: Cannot update non-existent file with expected_hash"
                    )
                    raise ContentNotFoundError(full_path)

                mode = "created"
                new_entry = FileJournal(
                    book_id=params.book_id,
                    path=params.path,
                    user_id="__base__",
                    sha256=new_hash,
                    last_written_at=datetime.now(timezone.utc),
                    storage_backend=config.storage_backend
                )
                session.add(new_entry)

            # Write to storage (within transaction scope for rollback)
            try:
                await op.write(full_path, content_bytes)
            except Exception as storage_error:
                # Storage write failed - transaction will rollback
                await log_operation(
                    operation=OperationType.WRITE_CONTENT,
                    path=full_path,
                    agent_id="system",
                    status=OperationStatus.ERROR,
                    error_message=f"Storage write failed: {str(storage_error)}"
                )
                raise

            # Session commits on context exit if no exception

        # Get metadata of written file (outside transaction)
        metadata = await op.stat(full_path)

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=full_path,
            agent_id="system",
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time
        )

        # Build response (FR-005: mode indicates created vs updated)
        response = {
            "status": "success",
            "path": full_path,
            "file_size": metadata.content_length,
            "file_hash": new_hash,
            "mode": mode
        }

        return json.dumps(response, indent=2)

    except (ConflictError, HashRequiredError, ContentNotFoundError, InvalidPathError):
        raise  # Re-raise known errors as-is

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
