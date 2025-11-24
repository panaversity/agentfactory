"""Asset management tools for PanaversityFS.

Implements 3 MCP tools for binary asset management:
- upload_asset: Hybrid upload (direct <10MB, presigned ≥10MB)
- get_asset: Get asset metadata + CDN URL
- list_assets: List assets with filtering
"""

from panaversity_fs.server import mcp
from panaversity_fs.models import (
    UploadAssetInput, GetAssetInput, ListAssetsInput,
    AssetMetadata, OperationType, OperationStatus
)
from panaversity_fs.storage import get_operator
from panaversity_fs.storage_utils import (
    compute_sha256, validate_path, sanitize_filename,
    get_mime_type, build_cdn_url
)
from panaversity_fs.errors import ContentNotFoundError, InvalidPathError
from panaversity_fs.audit import log_operation
from panaversity_fs.config import get_config
from datetime import datetime, timezone
import json
import base64


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
async def upload_asset(params: UploadAssetInput) -> str:
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
          "cdn_url": "https://cdn.panaversity.com/books/ai-native-python/assets/images/diagram.png",
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
          "cdn_url": "https://cdn.panaversity.com/books/ai-native-python/assets/videos/tutorial.mp4",
          "expires_in": 3600
        }
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Sanitize filename
        safe_filename = sanitize_filename(params.filename)

        # Build storage path
        asset_path = f"books/{params.book_id}/assets/{params.asset_type.value}/{safe_filename}"

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

            # Check size limit for direct upload
            if file_size >= 10 * 1024 * 1024:  # 10MB
                raise ValueError(
                    f"File size {file_size} bytes exceeds direct upload limit (10MB). "
                    "Use presigned URL method instead (provide file_size, omit binary_data)."
                )

            # Write to storage
            await op.write(asset_path, binary_content)

            # Get metadata
            metadata = await op.stat(asset_path)

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
                agent_id="system",  # TODO: Get from auth context
                status=OperationStatus.SUCCESS,
                execution_time_ms=execution_time
            )

            # Build response
            response = {
                "status": "success",
                "method": "direct",
                "cdn_url": cdn_url,
                "file_size": file_size,
                "mime_type": get_mime_type(safe_filename),
                "path": asset_path
            }

            return json.dumps(response, indent=2)

        # Method 2: Presigned URL (≥10MB)
        elif params.file_size:
            # Check size is reasonable for presigned URL
            if params.file_size < 10 * 1024 * 1024:  # 10MB
                raise ValueError(
                    f"File size {params.file_size} bytes is below presigned URL threshold (10MB). "
                    "Use direct upload instead (provide base64 binary_data, omit file_size)."
                )

            # Build CDN URL (asset will be available after upload completes)
            cdn_url = build_cdn_url(
                config.cdn_base_url,
                params.book_id,
                params.asset_type.value,
                safe_filename
            )

            # TODO: Generate presigned write URL using OpenDAL presign API
            # This requires checking if the storage backend supports presigning
            # For now, return placeholder indicating feature is not yet implemented

            # Log presigned URL request
            execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
            await log_operation(
                operation=OperationType.UPLOAD_ASSET,
                path=asset_path,
                agent_id="system",
                status=OperationStatus.SUCCESS,
                execution_time_ms=execution_time
            )

            response = {
                "status": "presigned_url_not_implemented",
                "method": "presigned",
                "message": "Presigned URL generation not yet implemented. Please use direct upload for files <10MB.",
                "cdn_url": cdn_url,
                "file_size": params.file_size,
                "path": asset_path
            }

            return json.dumps(response, indent=2)

        else:
            raise ValueError("Must provide either binary_data (direct upload) or file_size (presigned URL)")

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.UPLOAD_ASSET,
            path=f"books/{params.book_id}/assets/{params.asset_type.value}/{params.filename}",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
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
async def get_asset(params: GetAssetInput) -> str:
    """Get asset metadata including CDN URL (FR-012).

    Args:
        params (GetAssetInput): Validated input containing:
            - book_id (str): Book identifier
            - asset_type (AssetType): Asset category
            - filename (str): Asset filename

    Returns:
        str: JSON response with asset metadata

    Example:
        ```
        Input: {
          "book_id": "ai-native-python",
          "asset_type": "images",
          "filename": "diagram.png"
        }
        Output: {
          "cdn_url": "https://cdn.panaversity.com/books/ai-native-python/assets/images/diagram.png",
          "file_size": 45231,
          "mime_type": "image/png",
          "upload_timestamp": "2025-11-24T12:00:00Z",
          "uploaded_by_agent_id": "system",
          "asset_type": "images",
          "filename": "diagram.png"
        }
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Sanitize filename
        safe_filename = sanitize_filename(params.filename)

        # Build storage path
        asset_path = f"books/{params.book_id}/assets/{params.asset_type.value}/{safe_filename}"

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

        # Build response
        asset_metadata = AssetMetadata(
            cdn_url=cdn_url,
            file_size=metadata.content_length,
            mime_type=get_mime_type(safe_filename),
            upload_timestamp=metadata.last_modified,
            uploaded_by_agent_id="system",  # TODO: Track uploader in metadata
            asset_type=params.asset_type,
            filename=safe_filename
        )

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.GET_ASSET,
            path=asset_path,
            agent_id="system",
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time
        )

        return asset_metadata.model_dump_json(indent=2)

    except ContentNotFoundError:
        # Log error
        await log_operation(
            operation=OperationType.GET_ASSET,
            path=f"books/{params.book_id}/assets/{params.asset_type.value}/{params.filename}",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message="Asset not found"
        )

        raise

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.GET_ASSET,
            path=f"books/{params.book_id}/assets/{params.asset_type.value}/{params.filename}",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
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
async def list_assets(params: ListAssetsInput) -> str:
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
            search_paths = [f"books/{params.book_id}/assets/{params.asset_type.value}/"]
        else:
            # List all types
            from panaversity_fs.models import AssetType
            search_paths = [
                f"books/{params.book_id}/assets/{asset_type.value}/"
                for asset_type in AssetType
            ]

        # Collect all assets
        assets = []

        for search_path in search_paths:
            # Extract asset_type from path
            # Path format: books/{book_id}/assets/{asset_type}/
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

                        assets.append(asset_metadata.model_dump())

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
            path=f"books/{params.book_id}/assets/",
            agent_id="system",
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time
        )

        return json.dumps(assets, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.LIST_ASSETS,
            path=f"books/{params.book_id}/assets/",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return f"Error listing assets: {type(e).__name__}: {str(e)}"
