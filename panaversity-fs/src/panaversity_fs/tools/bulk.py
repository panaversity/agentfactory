"""Bulk operation tools for PanaversityFS.

Implements 1 MCP tool for bulk operations:
- get_book_archive: Generate presigned URL for downloading entire book as archive
"""

from panaversity_fs.app import mcp
from panaversity_fs.models import GetBookArchiveInput, OperationType, OperationStatus, ArchiveScope
from panaversity_fs.storage import get_operator
from panaversity_fs.audit import log_operation
from panaversity_fs.config import get_config
from datetime import datetime, timezone
import json
import zipfile
import tempfile
import os


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
async def get_book_archive(params: GetBookArchiveInput) -> str:
    """Generate presigned URL for downloading book content as archive (FR-029, FR-030).

    Creates ZIP archive of book content based on scope parameter.
    Returns presigned download URL valid for 1 hour.

    Args:
        params (GetBookArchiveInput): Validated input containing:
            - book_id (str): Book identifier
            - scope (str): Archive scope - 'content' (default), 'assets', or 'all'

    Returns:
        str: JSON response with archive URL and metadata

    Example:
        ```
        Input: {"book_id": "ai-native-python", "scope": "content"}
        Output: {
          "status": "success",
          "archive_url": "https://storage.panaversity.com/archives/ai-native-python-content-2025-11-24.zip?token=...",
          "expires_at": "2025-11-24T13:00:00Z",
          "file_count": 300,
          "total_size_bytes": 15432100,
          "format": "zip",
          "scope": "content",
          "valid_for_seconds": 3600
        }
        ```

    Note:
        - scope='content' (default): Only markdown files from content/ directory (~300 files, fast)
        - scope='assets': Only files from static/ directory (images, slides)
        - scope='all': Entire book - may timeout for large books with many assets
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Get operator and config
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

        # Create temporary ZIP file
        with tempfile.NamedTemporaryFile(mode='wb', delete=False, suffix='.zip') as temp_zip:
            temp_zip_path = temp_zip.name

            # Create ZIP archive
            with zipfile.ZipFile(temp_zip, mode='w', compression=zipfile.ZIP_DEFLATED) as archive:
                file_count = 0
                total_size = 0

                try:
                    # Recursively scan files based on scope
                    entries = await op.scan(scan_path)

                    async for entry in entries:
                        # Skip directories
                        if entry.path.endswith('/'):
                            continue

                        try:
                            # Read file content
                            content_bytes = await op.read(entry.path)

                            # Add to archive with relative path
                            arcname = entry.path[len(book_path):]  # Remove book_path prefix
                            archive.writestr(arcname, content_bytes)

                            file_count += 1
                            total_size += len(content_bytes)

                            # Check timeout constraint (60 seconds for 500 files / 200MB)
                            elapsed = (datetime.now(timezone.utc) - start_time).total_seconds()
                            if elapsed > config.archive_timeout_seconds:
                                raise TimeoutError(
                                    f"Archive generation exceeded timeout ({config.archive_timeout_seconds}s). "
                                    f"Processed {file_count} files, {total_size / 1024 / 1024:.2f}MB."
                                )

                        except Exception as e:
                            # Log file read error but continue
                            continue

                except Exception as e:
                    # Log directory listing error
                    os.unlink(temp_zip_path)
                    raise

        # Upload archive to storage with presigned URL
        # For MVP: Return local file path as placeholder
        # TODO: Implement presigned URL generation for production

        archive_filename = f"{params.book_id}{scope_suffix}-{datetime.now(timezone.utc).strftime('%Y-%m-%d')}.zip"
        archive_storage_path = f"archives/{archive_filename}"

        # Read archive bytes
        with open(temp_zip_path, 'rb') as f:
            archive_bytes = f.read()

        # Write to storage
        await op.write(archive_storage_path, archive_bytes)

        # Clean up temporary file
        os.unlink(temp_zip_path)

        # Calculate expiration (use config for presign expiry)
        expires_at = datetime.now(timezone.utc).timestamp() + config.presign_expiry_seconds

        # Build response
        # TODO: Generate actual presigned URL using OpenDAL presign API
        response = {
            "status": "success",
            "archive_url": f"{config.cdn_base_url}/{archive_storage_path}",
            "expires_at": datetime.fromtimestamp(expires_at, tz=timezone.utc).isoformat(),
            "file_count": file_count,
            "total_size_bytes": total_size,
            "format": "zip",
            "scope": scope.value,
            "valid_for_seconds": config.presign_expiry_seconds,
            "note": "Presigned URL generation not yet implemented. URL is public CDN path for now."
        }

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.GET_BOOK_ARCHIVE,
            path=book_path,
            agent_id="system",
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time
        )

        return json.dumps(response, indent=2)

    except TimeoutError as e:
        # Log timeout error
        await log_operation(
            operation=OperationType.GET_BOOK_ARCHIVE,
            path=f"books/{params.book_id}/",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return json.dumps({
            "status": "error",
            "error_type": "timeout",
            "message": str(e)
        }, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.GET_BOOK_ARCHIVE,
            path=f"books/{params.book_id}/",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return f"Error generating book archive: {type(e).__name__}: {str(e)}"
