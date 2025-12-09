"""Content operation tools for PanaversityFS.

Implements 3 MCP tools for content management (lessons and summaries per ADR-0018):
- read_content: Read markdown content with metadata (FR-016: overlay support)
- write_content: Upsert with conflict detection via FileJournal (FR-002, FR-003, FR-004, FR-005, FR-017)
- delete_content: Delete content file (FR-018: overlay support)

Path structure (Docusaurus-aligned):
- Lessons: content/{part}/{chapter}/{lesson}.md
- Summaries: content/{part}/{chapter}/{lesson}.summary.md

Overlay structure (FR-015):
- User overlays: books/{book}/users/{user_id}/content/...
"""

from mcp.server.fastmcp.server import Context

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
from panaversity_fs.path_utils import validate_overlay_path, validate_content_path
from sqlalchemy import select
from datetime import datetime, timezone
import json
import fnmatch


# =============================================================================
# Helper Functions for Overlay Support
# =============================================================================

def build_storage_path(book_id: str, path: str, user_id: str | None = None) -> str:
    """Build full storage path, optionally for overlay namespace.

    Args:
        book_id: Book identifier
        path: Relative content path
        user_id: Optional user ID for overlay (FR-015, FR-017)

    Returns:
        Full storage path
    """
    if user_id:
        # Overlay namespace: books/{book}/users/{user_id}/{path}
        return f"books/{book_id}/users/{user_id}/{path}"
    else:
        # Base namespace: books/{book}/{path}
        return f"books/{book_id}/{path}"


def get_journal_user_id(user_id: str | None) -> str:
    """Get user_id value for journal queries.

    Args:
        user_id: Optional user ID from request

    Returns:
        User ID or "__base__" for base content
    """
    return user_id if user_id else "__base__"


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
async def read_content(params: ReadContentInput, ctx: Context) -> str:
    """Read markdown content with metadata (FR-009, FR-016 overlay support).

    Supports four scopes:
    - file (default): Read single file, return content + metadata
    - chapter: Read all .md files in a chapter directory
    - part: Read all .md files in a part directory (all chapters)
    - book: Read all .md files in the entire book's content/ directory

    Asset Hash Queries (v1 Incremental Build):
    When path starts with "static/", returns ONLY the file_hash_sha256 from FileJournal
    (no content or metadata). This enables client-side incremental build for assets.
    - Input: {"book_id": "my-book", "path": "static/images/diagram.png"}
    - Output: {"file_hash_sha256": "abc123...", "exists": true}
    - If not in FileJournal: {"exists": false}

    Overlay Support (FR-016):
    When user_id is provided, reads from overlay first, falls back to base if not found.
    - Overlay path: books/{book_id}/users/{user_id}/content/...
    - Base path: books/{book_id}/content/...
    - Response includes "source" field: "overlay" or "base"

    Args:
        params (ReadContentInput): Validated input containing:
            - book_id (str): Book identifier (e.g., 'ai-native-python')
            - path (str): Content path relative to book root (ignored for book scope)
            - scope (ContentScope): file/chapter/part/book (default: file)
            - user_id (str | None): Optional user ID for overlay content (FR-016)

    Returns:
        str: JSON-formatted response
            - scope=file: Single ContentMetadata object (with source field if user_id)
            - scope=chapter/part/book: Array of ContentMetadata objects with path field
            - static/ paths: {"file_hash_sha256": "...", "exists": true/false}

    Example:
        ```
        # Read single file (default)
        Input: {"book_id": "my-book", "path": "content/01-Part/01-Chapter/01-lesson.md"}
        Output: {"content": "...", "file_size": 1234, ...}

        # Read asset hash (v1 incremental build)
        Input: {"book_id": "my-book", "path": "static/images/diagram.png"}
        Output: {"file_hash_sha256": "ebf4f635a17d10d6eb46ba680b70142419aa3220...", "exists": true}

        # Read with overlay support (FR-016)
        Input: {"book_id": "my-book", "path": "content/01-Part/01-Chapter/01-lesson.md", "user_id": "user123"}
        Output: {"content": "...", "file_size": 1234, "source": "overlay", ...}
        # Falls back to base if no overlay exists

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
        # Security validation (traversal, null bytes, etc.)
        if not validate_path(params.path):
            raise InvalidPathError(params.path, "Path contains invalid characters or traversal attempts")

        # ASSET HASH QUERY PATH (v1 Incremental Build)
        # If path starts with "static/", return hash from FileJournal (no content/metadata)
        if params.path.startswith("static/"):
            # Extract journal path (remove "static/" prefix to match FileJournal.path format)
            journal_path = '/'.join(params.path.split('/')[2:])  # e.g., "static/images/diagram.png" → "diagram.png"

            # Query FileJournal for hash
            async with get_session() as session:
                stmt = select(FileJournal).where(
                    FileJournal.book_id == params.book_id,
                    FileJournal.path == params.path,  # Full path including "static/images/"
                    FileJournal.user_id == "__base__"  # Assets are always base content
                )
                result = await session.execute(stmt)
                existing = result.scalar_one_or_none()

                if existing:
                    # Hash found in FileJournal
                    response = {
                        "file_hash_sha256": existing.sha256,
                        "exists": True
                    }
                else:
                    # Not in FileJournal
                    response = {
                        "exists": False
                    }

                # Log success
                execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
                await log_operation(
                    operation=OperationType.READ_CONTENT,
                    path=params.path,
                    status=OperationStatus.SUCCESS,
                    execution_time_ms=execution_time,
                    book_id=params.book_id
                )

                return json.dumps(response, indent=2)

        # FR-007: Schema validation for file scope (single file reads)
        # For chapter/part/book scopes, validation happens per-file during iteration
        if params.scope == ContentScope.FILE:
            schema_result = validate_content_path(params.path)
            if not schema_result.is_valid:
                error_msg = f"SCHEMA_VIOLATION: {'; '.join(schema_result.errors)}"
                await log_operation(
                    operation=OperationType.READ_CONTENT,
                    path=f"books/{params.book_id}/{params.path}",
                    status=OperationStatus.ERROR,
                    error_message=error_msg,
                    book_id=params.book_id,
                    user_id=params.user_id
                )
                raise InvalidPathError(params.path, error_msg)

        # Get operator
        op = get_operator()
        config = get_config()

        # Handle different scopes
        if params.scope == ContentScope.FILE:
            # FR-016: Overlay-first, then base fallback for single files
            source = "base"  # Track where content came from

            if params.user_id:
                # Try overlay path first
                overlay_path = build_storage_path(params.book_id, params.path, params.user_id)
                try:
                    content_bytes = await op.read(overlay_path)
                    full_path = overlay_path
                    source = "overlay"
                except Exception:
                    # Overlay doesn't exist, fall back to base
                    full_path = build_storage_path(params.book_id, params.path)
                    content_bytes = await op.read(full_path)
            else:
                # No user_id, read from base
                full_path = build_storage_path(params.book_id, params.path)
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
                status=OperationStatus.SUCCESS,
                execution_time_ms=execution_time,
                new_hash=file_hash,  # Include hash for provenance
                book_id=params.book_id,
                user_id=params.user_id
            )

            # Include source field if user_id was provided (FR-016)
            if params.user_id:
                response_dict = response.model_dump()
                response_dict["source"] = source
                return json.dumps(response_dict, indent=2, default=str)
            else:
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
                status=OperationStatus.SUCCESS,
                execution_time_ms=execution_time,
                book_id=params.book_id
            )

            return json.dumps(results, indent=2)

    except FileNotFoundError:
        # Log error
        await log_operation(
            operation=OperationType.READ_CONTENT,
            path=f"books/{params.book_id}/{params.path}",
            status=OperationStatus.ERROR,
            error_message="Content not found",
            book_id=params.book_id,
            user_id=params.user_id
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
            status=OperationStatus.ERROR,
            error_message=str(e),
            book_id=params.book_id,
            user_id=params.user_id
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
async def write_content(params: WriteContentInput, ctx: Context) -> str:
    """Write content with journal-backed conflict detection (FR-002, FR-003, FR-004, FR-005, FR-017).

    Works for lessons and summaries (ADR-0018).

    Conflict detection protocol:
    - FR-003: If expected_hash provided, verify it matches journal hash before write
    - FR-004: If expected_hash omitted AND file exists in journal, reject with HASH_REQUIRED
    - FR-005: If expected_hash omitted AND file doesn't exist, create operation succeeds
    - FR-002: Journal entry recorded BEFORE returning success; atomic rollback on failure

    Overlay Support (FR-017):
    When user_id is provided, writes to user's overlay namespace:
    - Overlay path: books/{book_id}/users/{user_id}/content/...
    - Base content is NOT modified (user gets personalized version)
    - Conflict detection uses overlay journal entry (not base)

    Args:
        params (WriteContentInput): Validated input containing:
            - book_id (str): Book identifier
            - path (str): Content path relative to book root
            - content (str): Markdown content
            - expected_hash (str | None): SHA256 hash for conflict detection (REQUIRED for updates)
            - user_id (str | None): Optional user ID for overlay writes (FR-017)

    Returns:
        str: JSON response with status, path, file_size, file_hash, mode ("created"|"updated")
             Includes "namespace": "overlay" when user_id is provided

    Example:
        ```
        # Create lesson (no expected_hash needed for new files)
        Input: {
          "book_id": "my-book",
          "path": "content/01-Part/01-Chapter/01-lesson.md",
          "content": "# Lesson 1\\n\\nContent..."
        }
        Output: {"status": "success", "mode": "created", ...}

        # Write to user overlay (FR-017)
        Input: {
          "book_id": "my-book",
          "path": "content/01-Part/01-Chapter/01-lesson.md",
          "content": "# My Notes\\n\\nPersonalized...",
          "user_id": "user123"
        }
        Output: {"status": "success", "mode": "created", "namespace": "overlay", ...}

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
        # Security validation (traversal, null bytes, etc.)
        if not validate_path(params.path):
            raise InvalidPathError(params.path, "Path contains invalid characters or traversal attempts")

        # FR-007: Schema validation - content paths must match Docusaurus pattern
        # Validate the path structure (content/{NN-Name}/{NN-Name}/{NN-name}.md)
        schema_result = validate_content_path(params.path)
        if not schema_result.is_valid:
            error_msg = f"SCHEMA_VIOLATION: {'; '.join(schema_result.errors)}"
            await log_operation(
                operation=OperationType.WRITE_CONTENT,
                path=f"books/{params.book_id}/{params.path}",
                status=OperationStatus.ERROR,
                error_message=error_msg,
                book_id=params.book_id,
                user_id=params.user_id
            )
            raise InvalidPathError(params.path, error_msg)

        # FR-017: Build full path (base or overlay namespace)
        full_path = build_storage_path(params.book_id, params.path, params.user_id)

        # Determine journal user_id (actual user_id or "__base__")
        journal_user_id = get_journal_user_id(params.user_id)

        # Track namespace for response
        namespace = "overlay" if params.user_id else "base"

        # Get operator
        op = get_operator()

        # Compute new content hash before any DB operations
        content_bytes = params.content.encode('utf-8')
        new_hash = compute_sha256(content_bytes)

        # Use atomic transaction for journal + storage (FR-002)
        async with get_session() as session:
            # Query FileJournal for existing entry (FR-002)
            # FR-017: Query overlay journal if user_id provided
            stmt = select(FileJournal).where(
                FileJournal.book_id == params.book_id,
                FileJournal.path == params.path,
                FileJournal.user_id == journal_user_id  # Base or overlay
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
                        status=OperationStatus.ERROR,
                        error_message=f"HASH_REQUIRED: Cannot update existing file without expected_hash",
                        book_id=params.book_id,
                        user_id=params.user_id
                    )
                    raise HashRequiredError(full_path, existing_entry.sha256)

                if params.expected_hash != existing_entry.sha256:
                    # FR-003: Conflict detected - hash mismatch
                    await log_operation(
                        operation=OperationType.WRITE_CONTENT,
                        path=full_path,
                        status=OperationStatus.CONFLICT,
                        error_message=f"Hash mismatch: expected {params.expected_hash}, journal has {existing_entry.sha256}",
                        book_id=params.book_id,
                        user_id=params.user_id
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
                        status=OperationStatus.ERROR,
                        error_message=f"NOT_FOUND: Cannot update non-existent file with expected_hash",
                        book_id=params.book_id,
                        user_id=params.user_id
                    )
                    raise ContentNotFoundError(full_path)

                mode = "created"
                new_entry = FileJournal(
                    book_id=params.book_id,
                    path=params.path,
                    user_id=journal_user_id,  # FR-017: Use overlay user_id or "__base__"
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
                    status=OperationStatus.ERROR,
                    error_message=f"Storage write failed: {str(storage_error)}",
                    book_id=params.book_id,
                    user_id=params.user_id
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
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time,
            new_hash=new_hash,  # Hash chain: new_hash for this operation
            book_id=params.book_id,
            user_id=params.user_id
        )

        # Build response (FR-005: mode indicates created vs updated)
        response = {
            "status": "success",
            "path": full_path,
            "file_size": metadata.content_length,
            "file_hash": new_hash,
            "mode": mode
        }

        # FR-017: Include namespace in response for overlay writes
        if params.user_id:
            response["namespace"] = namespace

        return json.dumps(response, indent=2)

    except (ConflictError, HashRequiredError, ContentNotFoundError, InvalidPathError):
        raise  # Re-raise known errors as-is

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{params.book_id}/{params.path}",
            status=OperationStatus.ERROR,
            error_message=str(e),
            book_id=params.book_id,
            user_id=params.user_id
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
async def delete_content(params: DeleteContentInput, ctx: Context) -> str:
    """Delete content file (lesson or summary) with overlay support (FR-018).

    Idempotent: Deleting non-existent file returns success (R3 invariant).
    Works for lessons and summaries (ADR-0018).

    IMPORTANT: Deletes BOTH storage file AND FileJournal entry atomically
    to maintain R2 (journal-storage consistency) invariant.

    Overlay Support (FR-018):
    When user_id is provided, ONLY deletes from user's overlay namespace:
    - Overlay path: books/{book_id}/users/{user_id}/content/...
    - Base content is NEVER deleted (user personalization only)
    - This effectively "resets" user's personalized content to base version

    Args:
        params (DeleteContentInput): Validated input containing:
            - book_id (str): Book identifier
            - path (str): Content path to delete
            - user_id (str | None): Optional user ID for overlay delete (FR-018)

    Returns:
        str: Success confirmation message with namespace info

    Example:
        ```
        # Delete lesson from base (admin only, no user_id)
        Input: {"book_id": "my-book", "path": "content/01-Part/01-Chapter/01-lesson.md"}
        Output: {"status": "success", "path": "books/my-book/...", "existed": true}

        # Delete user's overlay (FR-018) - resets to base
        Input: {"book_id": "my-book", "path": "content/01-Part/01-Chapter/01-lesson.md", "user_id": "user123"}
        Output: {"status": "success", "path": "books/my-book/users/user123/...", "existed": true, "namespace": "overlay"}
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Security validation (traversal, null bytes, etc.)
        if not validate_path(params.path):
            raise InvalidPathError(params.path, "Path contains invalid characters or traversal attempts")

        # FR-007: Schema validation - content paths must match Docusaurus pattern
        schema_result = validate_content_path(params.path)
        if not schema_result.is_valid:
            error_msg = f"SCHEMA_VIOLATION: {'; '.join(schema_result.errors)}"
            await log_operation(
                operation=OperationType.DELETE_CONTENT,
                path=f"books/{params.book_id}/{params.path}",
                status=OperationStatus.ERROR,
                error_message=error_msg,
                book_id=params.book_id,
                user_id=params.user_id
            )
            raise InvalidPathError(params.path, error_msg)

        # FR-018: Build full path (base or overlay namespace)
        full_path = build_storage_path(params.book_id, params.path, params.user_id)

        # Determine journal user_id (actual user_id or "__base__")
        journal_user_id = get_journal_user_id(params.user_id)

        # Track namespace for response
        namespace = "overlay" if params.user_id else "base"

        # Get operator
        op = get_operator()

        # Check if file exists in storage
        existed = True
        try:
            await op.stat(full_path)
        except:
            existed = False

        # Atomic deletion: Remove from BOTH journal AND storage (R2 invariant)
        async with get_session() as session:
            # Delete FileJournal entry if exists
            stmt = select(FileJournal).where(
                FileJournal.book_id == params.book_id,
                FileJournal.path == params.path,
                FileJournal.user_id == journal_user_id
            )
            result = await session.execute(stmt)
            existing_entry = result.scalar_one_or_none()

            if existing_entry:
                await session.delete(existing_entry)

            # Delete from storage (idempotent - R3 invariant)
            try:
                await op.delete(full_path)
            except Exception as storage_error:
                # Storage delete failed - transaction will rollback journal delete
                raise storage_error

            # Session commits on context exit if no exception

        # Log success (new_hash=None for deletes)
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.DELETE_CONTENT,
            path=full_path,
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time,
            new_hash=None,  # Deleted files have no hash
            book_id=params.book_id,
            user_id=params.user_id
        )

        # Build response
        response = {
            "status": "success",
            "path": full_path,
            "existed": existed,
            "message": f"File {'deleted' if existed else 'did not exist (idempotent delete)'}"
        }

        # FR-018: Include namespace in response for overlay deletes
        if params.user_id:
            response["namespace"] = namespace

        return json.dumps(response, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.DELETE_CONTENT,
            path=f"books/{params.book_id}/{params.path}",
            status=OperationStatus.ERROR,
            error_message=str(e),
            book_id=params.book_id,
            user_id=params.user_id
        )

        return f"Error deleting content: {type(e).__name__}: {str(e)}"
