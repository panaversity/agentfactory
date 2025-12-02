"""S3-based AttachmentStore implementation for ChatKit."""

import logging
from datetime import datetime, timedelta
from typing import Any

import boto3
from botocore.client import Config
from botocore.exceptions import ClientError

from chatkit.store import AttachmentStore
from chatkit.types import (
    Attachment,
    AttachmentCreateParams,
    FileAttachment,
    ImageAttachment,
)

from .config import AttachmentStoreConfig
from .context import RequestContext

logger = logging.getLogger(__name__)


class S3AttachmentStore(AttachmentStore[RequestContext]):
    """
    Production-ready S3-compatible AttachmentStore for ChatKit.
    
    Features:
    - Pre-signed URLs for secure uploads and downloads
    - Support for two-phase upload workflow
    - Image preview URL generation
    - User-based access control
    - Works with AWS S3, MinIO, LocalStack, etc.
    - Configurable expiration times
    - Storage class optimization
    """
    
    def __init__(
        self,
        config: AttachmentStoreConfig | None = None,
        s3_client: Any | None = None,
    ):
        """
        Initialize the S3 attachment store.
        
        Args:
            config: Attachment store configuration. If None, loads from environment.
            s3_client: Optional pre-configured boto3 S3 client.
                      If provided, config is ignored for client creation.
        """
        self.config = config or AttachmentStoreConfig()
        
        if s3_client:
            self.s3_client = s3_client
        else:
            self.s3_client = self._create_s3_client()
        
        logger.info(
            f"S3AttachmentStore initialized for bucket: {self.config.bucket_name}"
        )
    
    def _create_s3_client(self) -> Any:
        """Create boto3 S3 client with proper configuration."""
        session_kwargs: dict[str, Any] = {
            "region_name": self.config.region_name,
        }
        
        if self.config.aws_access_key_id:
            session_kwargs["aws_access_key_id"] = self.config.aws_access_key_id
        
        if self.config.aws_secret_access_key:
            session_kwargs["aws_secret_access_key"] = self.config.aws_secret_access_key
        
        # Create session
        session = boto3.Session(**session_kwargs)
        
        # Configure client with signature version for pre-signed URLs
        s3_config = Config(signature_version="s3v4")
        
        client_kwargs: dict[str, Any] = {"config": s3_config}
        
        if self.config.endpoint_url:
            client_kwargs["endpoint_url"] = self.config.endpoint_url
        
        return session.client("s3", **client_kwargs)
    
    def _get_s3_key(self, user_id: str, attachment_id: str) -> str:
        """
        Generate S3 key for an attachment.
        
        Format: {prefix}/users/{user_id}/attachments/{attachment_id}
        """
        return f"{self.config.key_prefix}/users/{user_id}/attachments/{attachment_id}"
    
    def _is_image(self, mime_type: str) -> bool:
        """Check if a MIME type represents an image."""
        return mime_type.startswith("image/")
    
    async def create_attachment(
        self, input: AttachmentCreateParams, context: RequestContext
    ) -> Attachment:
        """
        Create attachment metadata and generate pre-signed upload URL.
        
        This is phase 1 of the two-phase upload workflow.
        """
        # Generate attachment ID
        attachment_id = self.generate_attachment_id(input.mime_type, context)
        
        # Generate S3 key
        s3_key = self._get_s3_key(context.user_id, attachment_id)
        
        # Generate pre-signed upload URL
        try:
            upload_url = self.s3_client.generate_presigned_url(
                ClientMethod="put_object",
                Params={
                    "Bucket": self.config.bucket_name,
                    "Key": s3_key,
                    "ContentType": input.mime_type,
                    "ContentLength": input.size,
                    "StorageClass": self.config.storage_class,
                    "Metadata": {
                        "user_id": context.user_id,
                        "attachment_id": attachment_id,
                        "original_name": input.name,
                    },
                },
                ExpiresIn=self.config.presigned_url_expiration,
            )
        except ClientError as e:
            logger.error(f"Failed to generate pre-signed URL: {e}")
            raise RuntimeError("Failed to create attachment upload URL") from e
        
        # Create attachment object based on type
        base_params = {
            "id": attachment_id,
            "name": input.name,
            "mime_type": input.mime_type,
            "upload_url": upload_url,
        }
        
        if self._is_image(input.mime_type):
            # For images, we'll generate preview URL on load
            return ImageAttachment(
                **base_params,
                preview_url=None,  # Will be generated on load
            )
        else:
            return FileAttachment(**base_params)
    
    def _generate_preview_url(
        self, user_id: str, attachment_id: str
    ) -> str | None:
        """Generate a pre-signed URL for image preview."""
        if not self.config.generate_previews:
            return None
        
        s3_key = self._get_s3_key(user_id, attachment_id)
        
        try:
            preview_url = self.s3_client.generate_presigned_url(
                ClientMethod="get_object",
                Params={
                    "Bucket": self.config.bucket_name,
                    "Key": s3_key,
                },
                ExpiresIn=self.config.preview_url_expiration,
            )
            return preview_url
        except ClientError as e:
            logger.error(f"Failed to generate preview URL: {e}")
            return None
    
    async def get_download_url(
        self,
        attachment_id: str,
        context: RequestContext,
        expires_in: int | None = None,
    ) -> str:
        """
        Generate a pre-signed download URL for an attachment.
        
        Args:
            attachment_id: The attachment ID
            context: Request context for access control
            expires_in: Optional custom expiration time in seconds
        
        Returns:
            Pre-signed download URL
        """
        s3_key = self._get_s3_key(context.user_id, attachment_id)
        
        expiration = expires_in or self.config.presigned_url_expiration
        
        try:
            download_url = self.s3_client.generate_presigned_url(
                ClientMethod="get_object",
                Params={
                    "Bucket": self.config.bucket_name,
                    "Key": s3_key,
                },
                ExpiresIn=expiration,
            )
            return download_url
        except ClientError as e:
            logger.error(f"Failed to generate download URL: {e}")
            raise RuntimeError("Failed to create attachment download URL") from e
    
    async def delete_attachment(
        self, attachment_id: str, context: RequestContext
    ) -> None:
        """
        Delete attachment file from S3.
        
        Note: This only deletes the file. The metadata in the Store
        should be deleted separately.
        """
        s3_key = self._get_s3_key(context.user_id, attachment_id)
        
        try:
            self.s3_client.delete_object(
                Bucket=self.config.bucket_name,
                Key=s3_key,
            )
            logger.info(f"Deleted attachment {attachment_id} from S3")
        except ClientError as e:
            logger.error(f"Failed to delete attachment from S3: {e}")
            # Don't raise - deletion should be idempotent
    
    async def check_attachment_exists(
        self, attachment_id: str, context: RequestContext
    ) -> bool:
        """
        Check if an attachment exists in S3.
        
        Useful for validating that an upload completed successfully.
        """
        s3_key = self._get_s3_key(context.user_id, attachment_id)
        
        try:
            self.s3_client.head_object(
                Bucket=self.config.bucket_name,
                Key=s3_key,
            )
            return True
        except ClientError as e:
            if e.response["Error"]["Code"] == "404":
                return False
            logger.error(f"Failed to check attachment existence: {e}")
            return False
    
    async def get_attachment_metadata(
        self, attachment_id: str, context: RequestContext
    ) -> dict[str, Any]:
        """
        Get S3 object metadata for an attachment.
        
        Returns metadata like size, content type, last modified, etc.
        """
        s3_key = self._get_s3_key(context.user_id, attachment_id)
        
        try:
            response = self.s3_client.head_object(
                Bucket=self.config.bucket_name,
                Key=s3_key,
            )
            return {
                "content_type": response.get("ContentType"),
                "content_length": response.get("ContentLength"),
                "last_modified": response.get("LastModified"),
                "etag": response.get("ETag"),
                "metadata": response.get("Metadata", {}),
            }
        except ClientError as e:
            logger.error(f"Failed to get attachment metadata: {e}")
            raise RuntimeError("Failed to retrieve attachment metadata") from e
    
    def generate_attachment_id(
        self, mime_type: str, context: RequestContext
    ) -> str:
        """
        Generate a unique attachment ID.
        
        Override this method to customize ID generation.
        Default uses the parent class implementation.
        """
        return super().generate_attachment_id(mime_type, context)


class DirectUploadHandler:
    """
    Handler for direct upload endpoint.
    
    Use this in your FastAPI/Flask route to handle direct file uploads.
    """
    
    def __init__(
        self,
        store: S3AttachmentStore,
        max_file_size: int | None = None,
    ):
        """
        Initialize direct upload handler.
        
        Args:
            store: S3AttachmentStore instance
            max_file_size: Optional max file size override
        """
        self.store = store
        self.max_file_size = max_file_size or store.config.max_file_size
    
    async def handle_upload(
        self,
        file_content: bytes,
        filename: str,
        mime_type: str,
        context: RequestContext,
    ) -> Attachment:
        """
        Handle a direct file upload.
        
        Args:
            file_content: Raw file bytes
            filename: Original filename
            mime_type: File MIME type
            context: Request context for access control
        
        Returns:
            Created attachment object
        
        Raises:
            ValueError: If file size exceeds maximum
            RuntimeError: If upload fails
        """
        file_size = len(file_content)
        
        if file_size > self.max_file_size:
            raise ValueError(
                f"File size {file_size} exceeds maximum {self.max_file_size}"
            )
        
        # Generate attachment ID
        attachment_id = self.store.generate_attachment_id(mime_type, context)
        
        # Generate S3 key
        s3_key = self.store._get_s3_key(context.user_id, attachment_id)
        
        # Upload to S3
        try:
            self.store.s3_client.put_object(
                Bucket=self.store.config.bucket_name,
                Key=s3_key,
                Body=file_content,
                ContentType=mime_type,
                StorageClass=self.store.config.storage_class,
                Metadata={
                    "user_id": context.user_id,
                    "attachment_id": attachment_id,
                    "original_name": filename,
                },
            )
        except ClientError as e:
            logger.error(f"Failed to upload file to S3: {e}")
            raise RuntimeError("Failed to upload file") from e
        
        # Create attachment object
        base_params = {
            "id": attachment_id,
            "name": filename,
            "mime_type": mime_type,
            "upload_url": None,  # Already uploaded
        }
        
        if self.store._is_image(mime_type):
            preview_url = self.store._generate_preview_url(
                context.user_id, attachment_id
            )
            return ImageAttachment(**base_params, preview_url=preview_url)
        else:
            return FileAttachment(**base_params)

