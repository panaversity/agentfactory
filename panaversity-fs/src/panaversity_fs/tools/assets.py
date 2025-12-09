"""Asset management tools for PanaversityFS.

Implements 3 MCP tools for binary asset management:
- upload_asset: Hybrid upload (direct <10MB, presigned ≥10MB)
- get_asset: Get asset metadata + CDN URL (optional: include binary data)
- list_assets: List assets with filtering

Storage path (ADR-0018 Docusaurus-aligned):
    books/{book_id}/static/{asset_type}/{filename}
"""

from mcp.server.fastmcp.server import Context

from panaversity_fs.app import mcp
from panaversity_fs.models import (
    UploadAssetInput, GetAssetInput, ListAssetsInput,
    AssetMetadata, OperationType, OperationStatus
)
from panaversity_fs.storage import get_operator, presign_write, supports_presign
from panaversity_fs.storage_utils import (
    compute_sha256, validate_path, sanitize_filename,
    get_mime_type, build_cdn_url
)
from panaversity_fs.errors import ContentNotFoundError, InvalidPathError
from panaversity_fs.audit import log_operation
from panaversity_fs.config import get_config
from panaversity_fs.database.connection import get_session
from panaversity_fs.database.models import FileJournal
from sqlalchemy import select
from datetime import datetime, timezone
import json
import base64
import hashlib


@mcp.tool(
    name="upload_asset",
    annotations={
        "title": "Upload Binary Asset",
        "readOnlyHint": False,
        "destructiveHint": True,
        "idempotentHint": False,
        "openWorldHint": False
    }
)
async def upload_asset(params: UploadAssetInput, ctx: Context) -> str:
    """Upload binary asset with hybrid pattern (FR-010).

    Supports two upload methods:
    - Direct upload (binary_data provided, <10MB): Decode base64, write to storage
    - Presigned URL (file_size provided, ≥10MB): Generate presigned upload URL

    Args:
        params (UploadAssetInput): Validated input containing:
            - book_id (str): Book identifier
            - asset_type (AssetType): Asset category (slides/images/videos/audio)
            - filename (str): Original filename
            - binary_data (str | None): Base64-encoded binary data for direct upload
            - file_size (int | None): File size in bytes for presigned URL method

    Returns:
        str: JSON response with upload result

    Example:
        ```
        # Direct upload (<10MB)
        Input: {
          "book_id": "ai-native-python",
          "asset_type": "images",
          "filename": "diagram.png",
          "binary_data": "iVBORw0KGgoAAAANSUhEUgAA..."
        }
        Output: {
          "status": "success",
          "method": "direct",
          "cdn_url": "https://cdn.panaversity.com/books/ai-native-python/static/images/diagram.png",
          "file_size": 45231
        }

        # Presigned URL (≥10MB)
        Input: {
          "book_id": "ai-native-python",
          "asset_type": "videos",
          "filename": "tutorial.mp4",
          "file_size": 52428800
        }
        Output: {
          "status": "presigned_url",
          "method": "presigned",
          "upload_url": "https://...",
          "cdn_url": "https://cdn.panaversity.com/books/ai-native-python/static/videos/tutorial.mp4",
          "expires_in": 3600
        }
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Sanitize filename
        safe_filename = sanitize_filename(params.filename)

        # Build storage path
        asset_path = f"books/{params.book_id}/static/{params.asset_type.value}/{safe_filename}"

        # Get operator and config
        op = get_operator()
        config = get_config()

        # Method 1: Direct upload (<10MB)
        if params.binary_data:
            # Decode base64
            try:
                binary_content = base64.b64decode(params.binary_data)
            except Exception as e:
                raise ValueError(f"Invalid base64 data: {e}")

            file_size = len(binary_content)

            # Check size limit for direct upload (use config)
            max_direct_bytes = config.max_direct_upload_mb * 1024 * 1024
            if file_size >= max_direct_bytes:
                raise ValueError(
                    f"File size {file_size} bytes exceeds direct upload limit ({config.max_direct_upload_mb}MB). "
                    "Use presigned URL method instead (provide file_size, omit binary_data)."
                )

            # Compute hash for FileJournal
            content_hash = hashlib.sha256(binary_content).hexdigest()

            # Check FileJournal BEFORE uploading (incremental build optimization)
            # Build relative storage path (without books/ prefix)
            # asset_path is "books/{book_id}/static/..."
            # FileJournal expects "static/..."
            journal_path = '/'.join(asset_path.split('/')[2:])  # Remove "books/{book_id}/"

            # Query FileJournal for existing hash
            async with get_session() as session:
                stmt = select(FileJournal).where(
                    FileJournal.book_id == params.book_id,
                    FileJournal.path == journal_path,
                    FileJournal.user_id == "__base__"
                )
                result = await session.execute(stmt)
                existing = result.scalar_one_or_none()

                # If hash matches, skip upload (incremental build)
                if existing and existing.sha256 == content_hash:
                    # Asset unchanged - return success without uploading
                    cdn_url = build_cdn_url(
                        config.cdn_base_url,
                        params.book_id,
                        params.asset_type.value,
                        safe_filename
                    )

                    response = {
                        "status": "skipped",  # Indicate no upload happened
                        "method": "direct",
                        "cdn_url": cdn_url,
                        "file_size": file_size,
                        "mime_type": get_mime_type(safe_filename),
                        "path": asset_path,
                        "file_hash_sha256": content_hash,
                        "message": "Asset unchanged, skipped upload"
                    }

                    return json.dumps(response, indent=2)

            # Hash different or asset doesn't exist - proceed with upload
            # Write to storage
            await op.write(asset_path, binary_content)

            # Get metadata
            metadata = await op.stat(asset_path)

            # Write to FileJournal (THE FIX! - track assets like markdown)
            async with get_session() as session:
                # Build relative storage path (without books/ prefix)
                # asset_path is "books/{book_id}/static/..."
                # FileJournal expects "static/..."
                journal_path = '/'.join(asset_path.split('/')[2:])  # Remove "books/{book_id}/"

                # Upsert FileJournal entry
                stmt = select(FileJournal).where(
                    FileJournal.book_id == params.book_id,
                    FileJournal.path == journal_path,
                    FileJournal.user_id == "__base__"
                )
                result = await session.execute(stmt)
                existing = result.scalar_one_or_none()

                if existing:
                    existing.sha256 = content_hash
                    existing.last_written_at = datetime.now(timezone.utc)
                    existing.storage_backend = config.storage_backend
                else:
                    new_entry = FileJournal(
                        book_id=params.book_id,
                        path=journal_path,
                        user_id="__base__",
                        sha256=content_hash,
                        last_written_at=datetime.now(timezone.utc),
                        storage_backend=config.storage_backend
                    )
                    session.add(new_entry)

                await session.commit()

            # Build CDN URL
            cdn_url = build_cdn_url(
                config.cdn_base_url,
                params.book_id,
                params.asset_type.value,
                safe_filename
            )

            # Log success
            execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
            await log_operation(
                operation=OperationType.UPLOAD_ASSET,
                path=asset_path,
                status=OperationStatus.SUCCESS,
                execution_time_ms=execution_time,
                book_id=params.book_id
            )

            # Build response
            response = {
                "status": "success",
                "method": "direct",
                "cdn_url": cdn_url,
                "file_size": file_size,
                "mime_type": get_mime_type(safe_filename),
                "path": asset_path,
                "file_hash_sha256": content_hash  # NEW: Return hash for client verification
            }

            return json.dumps(response, indent=2)

        # Method 2: Presigned URL (≥10MB)
        elif params.file_size:
            # Check size is reasonable for presigned URL (use config)
            max_direct_bytes = config.max_direct_upload_mb * 1024 * 1024
            if params.file_size < max_direct_bytes:
                raise ValueError(
                    f"File size {params.file_size} bytes is below presigned URL threshold ({config.max_direct_upload_mb}MB). "
                    "Use direct upload instead (provide base64 binary_data, omit file_size)."
                )

            # Build CDN URL (asset will be available after upload completes)
            cdn_url = build_cdn_url(
                config.cdn_base_url,
                params.book_id,
                params.asset_type.value,
                safe_filename
            )

            # Generate presigned write URL using OpenDAL
            if not supports_presign():
                # Filesystem backend doesn't support presigned URLs
                # Log error
                await log_operation(
                    operation=OperationType.UPLOAD_ASSET,
                    path=asset_path,
                    status=OperationStatus.ERROR,
                    error_message="Backend does not support presigned URLs",
                    book_id=params.book_id
                )

                return json.dumps({
                    "status": "error",
                    "error_type": "UnsupportedOperation",
                    "message": f"Storage backend '{config.storage_backend}' does not support presigned URLs. "
                              "For large file uploads, use S3 or Supabase backend.",
                    "cdn_url": cdn_url,
                    "file_size": params.file_size,
                    "path": asset_path
                }, indent=2)

            # Generate presigned URL
            presigned_url = await presign_write(asset_path, config.presign_expiry_seconds)

            if not presigned_url:
                # Presign failed
                await log_operation(
                    operation=OperationType.UPLOAD_ASSET,
                    path=asset_path,
                    status=OperationStatus.ERROR,
                    error_message="Failed to generate presigned URL",
                    book_id=params.book_id
                )

                return json.dumps({
                    "status": "error",
                    "error_type": "PresignError",
                    "message": "Failed to generate presigned URL. Check storage backend configuration.",
                    "path": asset_path
                }, indent=2)

            # Log presigned URL request
            execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
            await log_operation(
                operation=OperationType.UPLOAD_ASSET,
                path=asset_path,
                status=OperationStatus.SUCCESS,
                execution_time_ms=execution_time,
                book_id=params.book_id
            )

            response = {
                "status": "presigned_url",
                "method": "presigned",
                "upload_url": presigned_url,
                "cdn_url": cdn_url,
                "file_size": params.file_size,
                "path": asset_path,
                "expires_in_seconds": config.presign_expiry_seconds,
                "upload_instructions": {
                    "method": "PUT",
                    "content_type": get_mime_type(safe_filename),
                    "note": "Upload file content directly via HTTP PUT to the upload_url"
                }
            }

            return json.dumps(response, indent=2)

        else:
            raise ValueError("Must provide either binary_data (direct upload) or file_size (presigned URL)")

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.UPLOAD_ASSET,
            path=f"books/{params.book_id}/static/{params.asset_type.value}/{params.filename}",
            status=OperationStatus.ERROR,
            error_message=str(e),
            book_id=params.book_id
        )

        return f"Error uploading asset: {type(e).__name__}: {str(e)}"


@mcp.tool(
    name="get_asset",
    annotations={
        "title": "Get Asset Metadata",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def get_asset(params: GetAssetInput, ctx: Context) -> str:
    """Get asset metadata including CDN URL (FR-012).

    Optionally include base64-encoded binary data for direct download.

    Args:
        params (GetAssetInput): Validated input containing:
            - book_id (str): Book identifier
            - asset_type (AssetType): Asset category
            - filename (str): Asset filename
            - include_binary (bool): Include base64 binary data (default: false)

    Returns:
        str: JSON response with asset metadata (and binary_data if requested)

    Example:
        ```
        # Metadata only (default)
        Input: {
          "book_id": "ai-native-python",
          "asset_type": "images",
          "filename": "diagram.png"
        }
        Output: {
          "cdn_url": "https://cdn.panaversity.com/books/ai-native-python/static/images/diagram.png",
          "file_size": 45231,
          "mime_type": "image/png",
          ...
        }

        # With binary data (for Docusaurus plugin)
        Input: {
          "book_id": "ai-native-python",
          "asset_type": "images",
          "filename": "diagram.png",
          "include_binary": true
        }
        Output: {
          "cdn_url": "...",
          "file_size": 45231,
          "binary_data": "iVBORw0KGgoAAAANSUhEUgAA..."
        }
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Sanitize filename
        safe_filename = sanitize_filename(params.filename)

        # Build storage path
        asset_path = f"books/{params.book_id}/static/{params.asset_type.value}/{safe_filename}"

        # Get operator and config
        op = get_operator()
        config = get_config()

        # Check if asset exists and get metadata
        try:
            metadata = await op.stat(asset_path)
        except:
            raise ContentNotFoundError(asset_path)

        # Build CDN URL
        cdn_url = build_cdn_url(
            config.cdn_base_url,
            params.book_id,
            params.asset_type.value,
            safe_filename
        )

        # Optionally read binary data
        binary_data_b64 = None
        if params.include_binary:
            binary_content = await op.read(asset_path)
            binary_data_b64 = base64.b64encode(binary_content).decode('ascii')

        # Build response
        asset_metadata = AssetMetadata(
            cdn_url=cdn_url,
            file_size=metadata.content_length,
            mime_type=get_mime_type(safe_filename),
            upload_timestamp=metadata.last_modified,
            uploaded_by_agent_id="system",  # TODO: Track uploader in metadata
            asset_type=params.asset_type,
            filename=safe_filename,
            binary_data=binary_data_b64
        )

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.GET_ASSET,
            path=asset_path,
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time,
            book_id=params.book_id
        )

        return asset_metadata.model_dump_json(indent=2)

    except ContentNotFoundError:
        # Log error
        await log_operation(
            operation=OperationType.GET_ASSET,
            path=f"books/{params.book_id}/static/{params.asset_type.value}/{params.filename}",
            status=OperationStatus.ERROR,
            error_message="Asset not found",
            book_id=params.book_id
        )

        raise

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.GET_ASSET,
            path=f"books/{params.book_id}/static/{params.asset_type.value}/{params.filename}",
            status=OperationStatus.ERROR,
            error_message=str(e),
            book_id=params.book_id
        )

        return f"Error getting asset: {type(e).__name__}: {str(e)}"


@mcp.tool(
    name="list_assets",
    annotations={
        "title": "List Book Assets",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def list_assets(params: ListAssetsInput, ctx: Context) -> str:
    """List assets for a book with optional type filtering (FR-014).

    Args:
        params (ListAssetsInput): Validated input containing:
            - book_id (str): Book identifier
            - asset_type (AssetType | None): Optional filter by asset type

    Returns:
        str: JSON array of asset metadata objects

    Example:
        ```
        # List all assets
        Input: {"book_id": "ai-native-python"}
        Output: [
          {
            "cdn_url": "https://.../images/diagram.png",
            "file_size": 45231,
            "mime_type": "image/png",
            ...
          },
          {
            "cdn_url": "https://.../videos/tutorial.mp4",
            "file_size": 52428800,
            "mime_type": "video/mp4",
            ...
          }
        ]

        # List only images
        Input: {"book_id": "ai-native-python", "asset_type": "images"}
        Output: [{"cdn_url": "https://.../images/diagram.png", ...}]
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Get operator and config
        op = get_operator()
        config = get_config()

        # Determine search paths
        if params.asset_type:
            # List specific type
            search_paths = [f"books/{params.book_id}/static/{params.asset_type.value}/"]
        else:
            # List all types
            from panaversity_fs.models import AssetType
            search_paths = [
                f"books/{params.book_id}/static/{asset_type.value}/"
                for asset_type in AssetType
            ]

        # Collect all assets
        assets = []

        for search_path in search_paths:
            # Extract asset_type from path
            # Path format: books/{book_id}/static/{asset_type}/
            path_parts = search_path.rstrip('/').split('/')
            current_asset_type = path_parts[-1]  # Last part is asset_type

            try:
                # List files in this directory
                # OpenDAL list returns async iterator of Entry objects
                entries = await op.list(search_path)

                async for entry in entries:
                    # Skip directories
                    if entry.path.endswith('/'):
                        continue

                    # Get metadata
                    try:
                        metadata = await op.stat(entry.path)

                        # Extract filename from path
                        filename = entry.path.split('/')[-1]

                        # Build CDN URL
                        cdn_url = build_cdn_url(
                            config.cdn_base_url,
                            params.book_id,
                            current_asset_type,
                            filename
                        )

                        # Create asset metadata
                        from panaversity_fs.models import AssetType
                        asset_metadata = AssetMetadata(
                            cdn_url=cdn_url,
                            file_size=metadata.content_length,
                            mime_type=get_mime_type(filename),
                            upload_timestamp=metadata.last_modified,
                            uploaded_by_agent_id="system",
                            asset_type=AssetType(current_asset_type),
                            filename=filename
                        )

                        assets.append(asset_metadata.model_dump(mode='json'))

                    except Exception as e:
                        # Skip files that can't be accessed
                        continue

            except Exception:
                # Directory doesn't exist, skip
                continue

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.LIST_ASSETS,
            path=f"books/{params.book_id}/static/",
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time,
            book_id=params.book_id
        )

        return json.dumps(assets, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.LIST_ASSETS,
            path=f"books/{params.book_id}/static/",
            status=OperationStatus.ERROR,
            error_message=str(e),
            book_id=params.book_id
        )

        return f"Error listing assets: {type(e).__name__}: {str(e)}"
