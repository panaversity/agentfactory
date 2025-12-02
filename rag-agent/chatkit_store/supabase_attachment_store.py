"""Supabase Storage-based AttachmentStore implementation for ChatKit."""

import asyncio
import logging
import mimetypes
from datetime import datetime, timedelta
from typing import Any
from uuid import uuid4

import httpx
from pydantic import BaseModel, Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from chatkit.store import AttachmentStore, NotFoundError
from chatkit.types import Attachment, AttachmentCreateParams

from .context import RequestContext

logger = logging.getLogger(__name__)


class SupabaseStorageConfig(BaseSettings):
    """
    Supabase Storage configuration.

    All settings can be overridden via environment variables with the
    CHATKIT_SUPABASE_ prefix.
    """

    model_config = SettingsConfigDict(
        env_prefix="CHATKIT_SUPABASE_",
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore",
    )

    # Supabase Configuration
    project_url: str = Field(
        ..., description="Supabase project URL (e.g., https://xxxxx.supabase.co)"
    )

    service_role_key: str = Field(
        ..., description="Supabase service role key (for server-side operations)"
    )

    bucket_name: str = Field(
        default="chatkit-attachments", description="Supabase storage bucket name"
    )

    # URL signing
    presigned_url_expiration: int = Field(
        default=3600,
        description="Pre-signed URL expiration in seconds",
        ge=60,
        le=604800,  # Max 7 days
    )

    # Upload constraints
    max_file_size: int = Field(
        default=100 * 1024 * 1024,  # 100MB
        description="Maximum file size in bytes",
        ge=1024,
        le=5 * 1024 * 1024 * 1024,  # 5GB
    )

    # Path organization
    path_prefix: str = Field(
        default="attachments", description="Prefix for all storage paths"
    )


class SupabaseAttachmentStore(AttachmentStore[RequestContext]):
    """
    Supabase Storage-based attachment store.

    Features:
    - User-scoped file organization
    - Signed URLs for secure uploads/downloads
    - Automatic MIME type detection
    - File size validation
    - Metadata storage

    Supabase Storage is S3-compatible but cheaper and easier to set up.
    """

    def __init__(self, config: SupabaseStorageConfig | None = None):
        """
        Initialize the Supabase storage client.

        Args:
            config: Storage configuration. If None, loads from environment.
        """
        self.config = config or SupabaseStorageConfig()

        # Supabase Storage API base URL
        self.storage_url = f"{self.config.project_url}/storage/v1"

        # HTTP client for API calls
        self.client = httpx.AsyncClient(
            headers={
                "apikey": self.config.service_role_key,
                "Authorization": f"Bearer {self.config.service_role_key}",
            },
            timeout=30.0,
        )

        logger.info(
            f"SupabaseAttachmentStore initialized with bucket: {self.config.bucket_name}"
        )

    async def close(self) -> None:
        """Close the HTTP client."""
        await self.client.aclose()

    def _generate_storage_path(
        self, user_id: str, attachment_id: str, file_name: str
    ) -> str:
        """
        Generate storage path with user isolation.

        Path format: {prefix}/users/{user_id}/attachments/{attachment_id}/{filename}
        """
        return f"{self.config.path_prefix}/users/{user_id}/attachments/{attachment_id}/{file_name}"

    def generate_attachment_id(
        self, mime_type: str | None, context: RequestContext
    ) -> str:
        """Generate unique attachment ID."""
        return f"att_{uuid4().hex[:16]}"

    async def create_attachment(
        self, input: AttachmentCreateParams, context: RequestContext
    ) -> Attachment:
        """
        Create attachment metadata and generate signed upload URL.

        This implements the AttachmentStore interface.
        """
        from chatkit.types import ImageAttachment, FileAttachment

        # Generate a unique ID for the attachment.
        attachment_id = self.generate_attachment_id(input.mime_type, context)

        # Validate file size
        if input.size and input.size > self.config.max_file_size:
            raise ValueError(
                f"File size {input.size} exceeds maximum {self.config.max_file_size}"
            )

        # Detect MIME type if not provided
        mime_type = input.mime_type
        if not mime_type:
            mime_type, _ = mimetypes.guess_type(input.name)
            mime_type = mime_type or "application/octet-stream"

        # Generate storage path
        storage_path = self._generate_storage_path(
            context.user_id, attachment_id, input.name
        )

        try:
            # Generate signed upload URL
            upload_url = await self._create_signed_upload_url(
                storage_path,
                mime_type,
            )

            logger.info(
                f"Created attachment upload for user {context.user_id}: "
                f"{attachment_id} ({input.name})"
            )

            # Determine if this is an image
            is_image = mime_type.startswith("image/")

            # Create the appropriate attachment type
            if is_image:
                attachment = ImageAttachment(
                    id=attachment_id,
                    name=input.name,
                    mime_type=mime_type,
                    upload_url=upload_url,
                    preview_url=upload_url,  # Use same URL for preview; can optimize later
                )
            else:
                attachment = FileAttachment(
                    id=attachment_id,
                    name=input.name,
                    mime_type=mime_type,
                    upload_url=upload_url,
                )

            return attachment

        except Exception as e:
            logger.error(f"Failed to create attachment: {e}")
            raise

    async def _create_signed_upload_url(
        self,
        storage_path: str,
        content_type: str,
    ) -> str:
        """
        Generate a signed URL for uploading to Supabase Storage.

        Args:
            storage_path: Full path in storage
            content_type: MIME type of file

        Returns:
            Signed upload URL
        """
        # Create signed URL for upload
        url = f"{self.storage_url}/object/upload/sign/{self.config.bucket_name}/{storage_path}"

        response = await self.client.post(
            url,
            json={
                "expiresIn": self.config.presigned_url_expiration,
                "contentType": content_type,
            },
        )

        if response.status_code != 200:
            error_detail = response.text
            logger.error(f"Failed to create signed upload URL: {error_detail}")
            raise RuntimeError(f"Failed to create upload URL: {error_detail}")

        data = response.json()

        # Construct full signed URL
        signed_url = f"{self.config.project_url}/storage/v1/object/upload/sign/{self.config.bucket_name}/{storage_path}?token={data['token']}"

        return signed_url

    async def upload_file_bytes(
        self,
        attachment_id: str,
        file_name: str,
        file_bytes: bytes,
        mime_type: str,
        context: RequestContext,
    ) -> str:
        """
        Upload file bytes directly to Supabase Storage.

        Args:
            attachment_id: Attachment identifier
            file_name: Name of the file
            file_bytes: File content as bytes
            mime_type: MIME type of the file
            context: Request context with user_id

        Returns:
            Storage path of uploaded file
        """
        # Generate storage path
        storage_path = self._generate_storage_path(
            context.user_id, attachment_id, file_name
        )

        # Upload to Supabase Storage
        upload_url = (
            f"{self.storage_url}/object/{self.config.bucket_name}/{storage_path}"
        )

        response = await self.client.post(
            upload_url,
            content=file_bytes,
            headers={
                "Content-Type": mime_type,
                "apikey": self.config.service_role_key,
                "Authorization": f"Bearer {self.config.service_role_key}",
            },
        )

        if response.status_code not in (200, 201):
            error_detail = response.text
            logger.error(f"Failed to upload file: {error_detail}")
            raise RuntimeError(f"Failed to upload file: {error_detail}")

        logger.info(f"Uploaded file to {storage_path}")

        return storage_path

    async def get_download_url(
        self, attachment_id: str, file_name: str, context: RequestContext
    ) -> str:
        """
        Generate a signed download URL for an attachment.

        Args:
            attachment_id: Attachment identifier
            file_name: Name of the file
            context: Request context with user_id

        Returns:
            Signed download URL
        """
        storage_path = self._generate_storage_path(
            context.user_id, attachment_id, file_name
        )
        return await self._create_signed_url(storage_path)

    async def load_attachment(
        self, attachment_id: str, context: RequestContext
    ) -> Attachment:
        """
        Load attachment and generate signed download URL.

        Args:
            attachment_id: Attachment identifier
            context: Request context with user_id

        Returns:
            Attachment with download URL
        """
        from chatkit.types import ImageAttachment, FileAttachment

        # List files in user's attachment directory
        prefix = f"{self.config.path_prefix}/users/{context.user_id}/attachments/{attachment_id}/"

        try:
            # List objects with prefix
            list_url = f"{self.storage_url}/object/list/{self.config.bucket_name}"

            response = await self.client.post(
                list_url,
                json={
                    "prefix": prefix,
                    "limit": 1,
                },
            )

            if response.status_code != 200:
                raise NotFoundError(f"Attachment {attachment_id} not found")

            files = response.json()

            if not files:
                raise NotFoundError(f"Attachment {attachment_id} not found")

            file_info = files[0]
            file_name = file_info["name"].split("/")[-1]  # Extract filename from path
            storage_path = file_info["name"]

            # Generate signed download URL
            download_url = await self._create_signed_url(storage_path)

            # Get mime type
            mime_type = file_info.get("metadata", {}).get(
                "mimetype", "application/octet-stream"
            )
            is_image = mime_type.startswith("image/")

            # Create the appropriate attachment type
            if is_image:
                attachment = ImageAttachment(
                    id=attachment_id,
                    name=file_name,
                    mime_type=mime_type,
                    upload_url=download_url,  # For loaded attachments, this is the download URL
                    preview_url=download_url,  # Use same URL for preview; can optimize later
                )
            else:
                attachment = FileAttachment(
                    id=attachment_id,
                    name=file_name,
                    mime_type=mime_type,
                    upload_url=download_url,  # For loaded attachments, this is the download URL
                )

            logger.info(f"Loaded attachment {attachment_id} for user {context.user_id}")

            return attachment

        except NotFoundError:
            raise
        except Exception as e:
            logger.error(f"Failed to load attachment {attachment_id}: {e}")
            raise

    async def _create_signed_url(self, storage_path: str) -> str:
        """
        Create signed download URL for a file.

        Args:
            storage_path: Full path in storage

        Returns:
            Signed download URL
        """
        url = f"{self.storage_url}/object/sign/{self.config.bucket_name}/{storage_path}"

        response = await self.client.post(
            url,
            json={
                "expiresIn": self.config.presigned_url_expiration,
            },
        )

        if response.status_code != 200:
            error_detail = response.text
            logger.error(f"Failed to create signed URL: {error_detail}")
            raise RuntimeError(f"Failed to create signed URL: {error_detail}")

        data = response.json()
        signed_path = data.get("signedURL")

        # Construct full URL with proper storage/v1 prefix
        # The signedURL from Supabase starts with /object/...
        # but we need /storage/v1/object/...
        if signed_path.startswith("/object/"):
            signed_path = f"/storage/v1{signed_path}"

        return f"{self.config.project_url}{signed_path}"

    async def delete_attachment(
        self, attachment_id: str, context: RequestContext
    ) -> None:
        """
        Delete attachment from storage.

        Args:
            attachment_id: Attachment identifier
            context: Request context with user_id
        """
        # Delete all files in the attachment directory
        prefix = f"{self.config.path_prefix}/users/{context.user_id}/attachments/{attachment_id}/"

        try:
            # List files first
            list_url = f"{self.storage_url}/object/list/{self.config.bucket_name}"

            response = await self.client.post(
                list_url,
                json={
                    "prefix": prefix,
                },
            )

            if response.status_code != 200:
                logger.warning(f"Attachment {attachment_id} not found for deletion")
                return

            files = response.json()

            if not files:
                logger.warning(f"No files found for attachment {attachment_id}")
                return

            # Delete each file
            file_paths = [file["name"] for file in files]

            delete_url = f"{self.storage_url}/object/{self.config.bucket_name}"

            response = await self.client.delete(
                delete_url,
                json={
                    "prefixes": file_paths,
                },
            )

            if response.status_code not in (200, 204):
                error_detail = response.text
                logger.error(f"Failed to delete attachment: {error_detail}")
                raise RuntimeError(f"Failed to delete attachment: {error_detail}")

            logger.info(
                f"Deleted attachment {attachment_id} for user {context.user_id}"
            )

        except Exception as e:
            logger.error(f"Failed to delete attachment {attachment_id}: {e}")
            raise

    async def ensure_bucket_exists(self) -> None:
        """
        Ensure the storage bucket exists. Create if it doesn't.

        Call this during application startup.
        """
        try:
            # Check if bucket exists
            url = f"{self.storage_url}/bucket/{self.config.bucket_name}"

            response = await self.client.get(url)

            if response.status_code == 200:
                logger.info(f"Bucket {self.config.bucket_name} exists")
                return

            # Create bucket
            create_url = f"{self.storage_url}/bucket"

            response = await self.client.post(
                create_url,
                json={
                    "name": self.config.bucket_name,
                    "public": False,  # Private bucket
                    "fileSizeLimit": self.config.max_file_size,
                    "allowedMimeTypes": None,  # Allow all types
                },
            )

            if response.status_code in (200, 201):
                logger.info(f"Created bucket {self.config.bucket_name}")
            else:
                error_detail = response.text
                logger.error(f"Failed to create bucket: {error_detail}")
                raise RuntimeError(f"Failed to create bucket: {error_detail}")

        except Exception as e:
            logger.error(f"Failed to ensure bucket exists: {e}")
            raise
