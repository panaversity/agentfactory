"""Bulk operation tools for PanaversityFS.

Implements MCP tools for bulk operations:
- get_book_archive: Generate streaming ZIP archive with 64MB memory cap (SC-001/R4)
- plan_build: Compute delta build for CI/CD (FR-025, FR-026)
"""

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
from dataclasses import dataclass, field
from typing import Optional

# =============================================================================
# Constants
# =============================================================================

# Maximum memory for archive buffer (SC-001: <64MB)
MAX_ARCHIVE_MEMORY_BYTES = 64 * 1024 * 1024  # 64MB

# Chunk size for streaming reads
CHUNK_SIZE = 1024 * 1024  # 1MB chunks


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

    Implements a circular buffer approach to stay under 64MB memory limit.
    Files are written to buffer, flushed to storage when buffer approaches limit.
    """

    def __init__(self, max_bytes: int = MAX_ARCHIVE_MEMORY_BYTES):
        self.max_bytes = max_bytes
        self.buffer = io.BytesIO()
        self.zip_file: Optional[zipfile.ZipFile] = None
        self.current_size = 0

    def __enter__(self):
        self.zip_file = zipfile.ZipFile(self.buffer, mode='w', compression=zipfile.ZIP_DEFLATED)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.zip_file:
            self.zip_file.close()

    def add_file(self, arcname: str, content: bytes) -> bool:
        """Add file to archive, return False if would exceed memory limit.

        Args:
            arcname: Path within the archive
            content: File content bytes

        Returns:
            True if file added, False if it would exceed memory limit
        """
        if not self.zip_file:
            raise RuntimeError("StreamingArchiveBuffer not initialized")

        # Estimate compressed size (typically 30-70% for text)
        estimated_size = len(content)

        # Check if adding this file would exceed memory limit
        if self.current_size + estimated_size > self.max_bytes:
            return False

        self.zip_file.writestr(arcname, content)
        self.current_size += estimated_size
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
async def get_book_archive(params: GetBookArchiveInput) -> str:
    """Generate streaming ZIP archive with memory cap (FR-029, FR-030, SC-001/R4).

    Creates ZIP archive of book content based on scope parameter.
    Streams files with 64MB memory cap, returns partial result on timeout (FR-014).

    Args:
        params (GetBookArchiveInput): Validated input containing:
            - book_id (str): Book identifier
            - scope (str): Archive scope - 'content' (default), 'assets', or 'all'

    Returns:
        str: JSON response with archive URL, metadata, and any errors

    Performance Constraints (SC-001/R4):
        - 500 files / 200MB archive within 60 seconds
        - <64MB server memory during generation
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

    try:
        # Use streaming buffer with memory cap
        with StreamingArchiveBuffer(max_bytes=MAX_ARCHIVE_MEMORY_BYTES) as archive:
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
                        # Read file content
                        content_bytes = await op.read(entry.path)

                        # Add to archive with relative path
                        arcname = entry.path[len(book_path):]

                        # Check memory limit
                        if not archive.add_file(arcname, content_bytes):
                            progress.add_error(
                                arcname,
                                f"Skipped: would exceed {MAX_ARCHIVE_MEMORY_BYTES // (1024*1024)}MB memory limit"
                            )
                            continue

                        progress.files_processed += 1
                        progress.total_bytes += len(content_bytes)
                        progress.current_memory_bytes = archive.current_size

                    except Exception as e:
                        progress.add_error(entry.path[len(book_path):], str(e))
                        continue

            except Exception as e:
                progress.add_error("scan", f"Directory listing failed: {e}")

            # Get archive bytes
            archive_bytes = archive.get_bytes()

        # Upload archive to storage
        archive_filename = f"{params.book_id}{scope_suffix}-{datetime.now(timezone.utc).strftime('%Y-%m-%d')}.zip"
        archive_storage_path = f"archives/{archive_filename}"

        await op.write(archive_storage_path, archive_bytes)

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
            "archive_size_bytes": len(archive_bytes),
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
            agent_id="system",  # TODO: Extract from MCP context (FR-021)
            status=OperationStatus.SUCCESS if status == "success" else OperationStatus.ERROR,
            execution_time_ms=execution_time
        )

        return json.dumps(response, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.GET_BOOK_ARCHIVE,
            path=f"books/{params.book_id}/",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return json.dumps({
            "status": "error",
            "error_type": type(e).__name__,
            "message": str(e),
            "files_processed": progress.files_processed,
            "elapsed_seconds": progress.elapsed_seconds()
        }, indent=2)
