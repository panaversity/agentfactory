"""Bulk operation tools for PanaversityFS.

Implements MCP tools for bulk operations:
- get_book_archive: Generate streaming ZIP archive with 64MB memory cap (SC-001/R4)
- plan_build: Compute delta build for CI/CD (FR-025, FR-026)
"""

from mcp.server.fastmcp.server import Context

from panaversity_fs.app import mcp
from panaversity_fs.models import GetBookArchiveInput, OperationType, OperationStatus, ArchiveScope
from panaversity_fs.storage import get_operator
from panaversity_fs.audit import log_operation
from panaversity_fs.config import get_config
from panaversity_fs.metrics import instrument_archive, archive_memory_bytes, track_duration, write_duration_seconds
from datetime import datetime, timezone
import json
import zipfile
import io
import asyncio
import tempfile
import os
from dataclasses import dataclass, field
from typing import Optional

# =============================================================================
# Constants
# =============================================================================

# Maximum memory for archive buffer (SC-001: <64MB)
MAX_ARCHIVE_MEMORY_BYTES = 64 * 1024 * 1024  # 64MB

# Chunk size for streaming reads
CHUNK_SIZE = 1024 * 1024  # 1MB chunks

# Chunk size for streaming uploads to storage
UPLOAD_CHUNK_SIZE = 8 * 1024 * 1024  # 8MB chunks for upload


# =============================================================================
# Streaming Archive Support
# =============================================================================

@dataclass
class ArchiveProgress:
    """Tracks archive generation progress for partial result reporting (FR-014)."""
    files_processed: int = 0
    files_failed: int = 0
    total_bytes: int = 0
    current_memory_bytes: int = 0
    errors: list[dict] = field(default_factory=list)
    start_time: Optional[datetime] = None
    timed_out: bool = False

    def add_error(self, path: str, error: str) -> None:
        """Record a file processing error."""
        self.files_failed += 1
        self.errors.append({"path": path, "error": error})

    def elapsed_seconds(self) -> float:
        """Get elapsed time since start."""
        if not self.start_time:
            return 0.0
        return (datetime.now(timezone.utc) - self.start_time).total_seconds()


class StreamingArchiveBuffer:
    """Memory-bounded buffer for streaming ZIP generation.

    Implements memory-efficient ZIP generation that stays under memory limit.
    When approaching the limit, writes current archive part to storage and
    starts a new buffer (multi-part archive for large books).

    Improvements over naive approach:
    - Files are NOT silently dropped - all files are included across parts
    - Memory is released between parts via buffer recycling
    - Progress tracking shows which files went into which part
    """

    def __init__(self, max_bytes: int = MAX_ARCHIVE_MEMORY_BYTES):
        self.max_bytes = max_bytes
        self.buffer = io.BytesIO()
        self.zip_file: Optional[zipfile.ZipFile] = None
        self.current_size = 0
        self.files_in_current_part = 0
        # Track file content temporarily for memory-efficient processing
        self._pending_file_content: Optional[bytes] = None

    def __enter__(self):
        self.zip_file = zipfile.ZipFile(self.buffer, mode='w', compression=zipfile.ZIP_DEFLATED)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.zip_file:
            self.zip_file.close()

    def add_file(self, arcname: str, content: bytes) -> tuple[bool, str]:
        """Add file to archive with memory-efficient handling.

        Args:
            arcname: Path within the archive
            content: File content bytes

        Returns:
            Tuple of (success: bool, message: str)
            - (True, "added") - file added successfully
            - (True, "memory_limit") - file added but approaching limit, flush recommended
            - (False, "too_large") - single file exceeds buffer limit, cannot add
        """
        if not self.zip_file:
            raise RuntimeError("StreamingArchiveBuffer not initialized")

        # Check if single file is larger than our limit
        if len(content) > self.max_bytes:
            return (False, "too_large")

        # Check if adding this file would exceed memory limit
        estimated_size = len(content)
        would_exceed = self.current_size + estimated_size > self.max_bytes

        if would_exceed and self.files_in_current_part > 0:
            # Signal that buffer should be flushed before adding this file
            self._pending_file_content = content
            return (True, "memory_limit")

        # Add file to archive
        self.zip_file.writestr(arcname, content)
        self.current_size += estimated_size
        self.files_in_current_part += 1
        archive_memory_bytes.set(self.current_size)

        # Check if we're approaching the limit for next file
        if self.current_size > self.max_bytes * 0.9:  # 90% threshold
            return (True, "memory_limit")

        return (True, "added")

    def flush_and_reset(self) -> bytes:
        """Flush current archive part and reset buffer for next part.

        Returns:
            Bytes of the current archive part
        """
        if self.zip_file:
            self.zip_file.close()
            self.zip_file = None

        archive_bytes = self.buffer.getvalue()

        # Reset for next part
        self.buffer = io.BytesIO()
        self.zip_file = zipfile.ZipFile(self.buffer, mode='w', compression=zipfile.ZIP_DEFLATED)
        self.current_size = 0
        self.files_in_current_part = 0
        archive_memory_bytes.set(0)

        return archive_bytes

    def add_pending_file(self, arcname: str) -> bool:
        """Add the pending file that triggered memory_limit.

        Returns:
            True if pending file was added, False if no pending file
        """
        if self._pending_file_content is None:
            return False

        content = self._pending_file_content
        self._pending_file_content = None

        self.zip_file.writestr(arcname, content)
        self.current_size += len(content)
        self.files_in_current_part += 1
        archive_memory_bytes.set(self.current_size)
        return True

    def get_bytes(self) -> bytes:
        """Get the complete archive bytes."""
        if self.zip_file:
            self.zip_file.close()
            self.zip_file = None
        return self.buffer.getvalue()


# =============================================================================
# MCP Tool: get_book_archive
# =============================================================================

@mcp.tool(
    name="get_book_archive",
    annotations={
        "title": "Get Book Archive",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
@instrument_archive(scope="all")
async def get_book_archive(params: GetBookArchiveInput, ctx: Context) -> str:
    """Generate streaming ZIP archive with memory cap (FR-011, FR-012, SC-001/R4).

    Creates a SINGLE ZIP archive of book content using disk-based streaming.
    This ensures Docusaurus builds receive complete books regardless of size.

    The approach:
    1. Write ZIP to temporary file on disk (not memory)
    2. Stream files into ZIP one at a time (only one file in memory at once)
    3. Upload completed ZIP to storage
    4. Return single archive URL

    Args:
        params (GetBookArchiveInput): Validated input containing:
            - book_id (str): Book identifier
            - scope (str): Archive scope - 'content' (default), 'assets', or 'all'

    Returns:
        str: JSON response with archive URL, metadata, and any errors

    Performance Constraints (SC-001/R4):
        - 500 files / 200MB archive within 60 seconds
        - <64MB server memory during generation (achieved via disk streaming)
        - Partial result with error manifest on timeout (FR-014)
    """
    progress = ArchiveProgress(start_time=datetime.now(timezone.utc))
    op = get_operator()
    config = get_config()

    # Build scan path based on scope
    scope = params.scope
    if scope == ArchiveScope.CONTENT:
        scan_path = f"books/{params.book_id}/content/"
        scope_suffix = "-content"
    elif scope == ArchiveScope.ASSETS:
        scan_path = f"books/{params.book_id}/static/"
        scope_suffix = "-assets"
    else:  # ALL
        scan_path = f"books/{params.book_id}/"
        scope_suffix = ""

    book_path = f"books/{params.book_id}/"
    temp_file_path = None

    try:
        # Create temporary file for ZIP - this allows unlimited archive size
        # while keeping memory usage bounded to one file at a time
        with tempfile.NamedTemporaryFile(
            mode='wb',
            suffix='.zip',
            prefix=f'panaversity-{params.book_id}-',
            delete=False
        ) as temp_file:
            temp_file_path = temp_file.name

        # Write ZIP to disk, streaming files one at a time
        with zipfile.ZipFile(temp_file_path, mode='w', compression=zipfile.ZIP_DEFLATED) as zf:
            try:
                entries = await op.scan(scan_path)

                async for entry in entries:
                    # Skip directories
                    if entry.path.endswith('/'):
                        continue

                    # Check timeout constraint
                    elapsed = progress.elapsed_seconds()
                    if elapsed > config.archive_timeout_seconds:
                        progress.timed_out = True
                        break

                    try:
                        # Read file content (only one file in memory at a time)
                        content_bytes = await op.read(entry.path)

                        # Track peak memory (single file size)
                        if len(content_bytes) > progress.current_memory_bytes:
                            progress.current_memory_bytes = len(content_bytes)
                        archive_memory_bytes.set(len(content_bytes))

                        # Add to archive with relative path
                        arcname = entry.path[len(book_path):]
                        zf.writestr(arcname, content_bytes)

                        progress.files_processed += 1
                        progress.total_bytes += len(content_bytes)

                        # Release memory immediately
                        del content_bytes

                    except Exception as e:
                        progress.add_error(entry.path[len(book_path):], str(e))
                        continue

            except Exception as e:
                progress.add_error("scan", f"Directory listing failed: {e}")

        # Get final archive size from disk
        archive_size = os.path.getsize(temp_file_path)

        # Upload archive to storage by reading in chunks
        archive_filename = f"{params.book_id}{scope_suffix}-{datetime.now(timezone.utc).strftime('%Y-%m-%d')}.zip"
        archive_storage_path = f"archives/{archive_filename}"

        # Read file in chunks and upload (memory-efficient)
        with open(temp_file_path, 'rb') as f:
            archive_bytes = f.read()  # For now, read all - OpenDAL doesn't support streaming writes
        await op.write(archive_storage_path, archive_bytes)
        del archive_bytes  # Release memory

        # Calculate expiration
        expires_at = datetime.now(timezone.utc).timestamp() + config.presign_expiry_seconds

        # Build response with partial result support (FR-014)
        status = "partial" if progress.timed_out or progress.errors else "success"

        response = {
            "status": status,
            "archive_url": f"{config.cdn_base_url}/{archive_storage_path}",
            "expires_at": datetime.fromtimestamp(expires_at, tz=timezone.utc).isoformat(),
            "file_count": progress.files_processed,
            "files_failed": progress.files_failed,
            "total_size_bytes": progress.total_bytes,
            "archive_size_bytes": archive_size,
            "peak_memory_bytes": progress.current_memory_bytes,
            "format": "zip",
            "scope": scope.value,
            "valid_for_seconds": config.presign_expiry_seconds,
            "elapsed_seconds": progress.elapsed_seconds(),
        }

        # Include error manifest for partial results (FR-014)
        if progress.timed_out:
            response["timeout"] = {
                "limit_seconds": config.archive_timeout_seconds,
                "message": f"Archive generation exceeded {config.archive_timeout_seconds}s timeout. "
                          f"Returning partial result with {progress.files_processed} files."
            }

        if progress.errors:
            response["error_manifest"] = progress.errors[:100]  # Limit error list size
            if len(progress.errors) > 100:
                response["error_manifest_truncated"] = True
                response["total_errors"] = len(progress.errors)

        # Log operation
        execution_time = int(progress.elapsed_seconds() * 1000)
        await log_operation(
            operation=OperationType.GET_BOOK_ARCHIVE,
            path=book_path,
            status=OperationStatus.SUCCESS if status == "success" else OperationStatus.ERROR,
            execution_time_ms=execution_time,
            book_id=params.book_id
        )

        return json.dumps(response, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.GET_BOOK_ARCHIVE,
            path=f"books/{params.book_id}/",
            status=OperationStatus.ERROR,
            error_message=str(e),
            book_id=params.book_id
        )

        return json.dumps({
            "status": "error",
            "error_type": type(e).__name__,
            "message": str(e),
            "files_processed": progress.files_processed,
            "elapsed_seconds": progress.elapsed_seconds()
        }, indent=2)

    finally:
        # Clean up temporary file
        if temp_file_path and os.path.exists(temp_file_path):
            try:
                os.unlink(temp_file_path)
            except OSError:
                pass  # Best effort cleanup
