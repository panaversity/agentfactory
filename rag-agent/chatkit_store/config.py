"""Configuration for ChatKit stores."""

from typing import Literal
from pydantic import Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class StoreConfig(BaseSettings):
    """
    PostgreSQL Store configuration.

    All settings can be overridden via environment variables with the
    CHATKIT_STORE_ prefix (e.g., CHATKIT_STORE_DATABASE_URL).
    """

    model_config = SettingsConfigDict(
        env_prefix="CHATKIT_STORE_",
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore",
    )

    # Database connection
    database_url: str = Field(
        ...,
        description="PostgreSQL connection URL (postgresql+asyncpg://user:pass@host:port/db)",
    )

    # Connection pool settings
    pool_size: int = Field(
        default=20,  # Production-ready pool size for concurrent requests
        description="Maximum number of connections in the pool",
        ge=1,
        le=100,
    )

    max_overflow: int = Field(
        default=10,  # Additional connections for traffic spikes
        description="Maximum overflow connections beyond pool_size",
        ge=0,
        le=50,
    )

    pool_timeout: float = Field(
        default=30.0,  # Allow sufficient time for connection under load
        description="Seconds to wait before timing out on getting a connection",
        gt=0,
    )

    pool_recycle: int = Field(
        default=3600, description="Seconds after which connections are recycled", ge=300
    )

    # Query settings
    statement_timeout: int = Field(
        default=30000, description="Statement timeout in milliseconds", ge=1000
    )

    # Schema
    schema_name: str = Field(
        default="chatkit", description="Database schema name for tables"
    )

    @field_validator("database_url")
    @classmethod
    def validate_database_url(cls, v: str) -> str:
        """Ensure database URL uses asyncpg driver and fix SSL parameters."""
        if not v.startswith("postgresql://") and not v.startswith(
            "postgresql+asyncpg://"
        ):
            raise ValueError(
                "database_url must start with postgresql:// or postgresql+asyncpg://"
            )

        # Convert to asyncpg if needed
        if v.startswith("postgresql://"):
            v = v.replace("postgresql://", "postgresql+asyncpg://", 1)

        # Fix SSL parameters for asyncpg
        # asyncpg uses 'ssl=require' not 'sslmode=require'
        if "sslmode=require" in v:
            v = v.replace("sslmode=require", "ssl=require")
        elif "sslmode=prefer" in v:
            v = v.replace("sslmode=prefer", "ssl=prefer")
        elif "sslmode=allow" in v:
            v = v.replace("sslmode=allow", "ssl=allow")
        elif "sslmode=disable" in v:
            v = v.replace("sslmode=disable", "ssl=disable")

        return v


class AttachmentStoreConfig(BaseSettings):
    """
    S3-compatible AttachmentStore configuration.

    All settings can be overridden via environment variables with the
    CHATKIT_ATTACHMENT_ prefix.
    """

    model_config = SettingsConfigDict(
        env_prefix="CHATKIT_ATTACHMENT_",
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore",
    )

    # S3 Configuration
    bucket_name: str = Field(..., description="S3 bucket name for attachments")

    region_name: str = Field(default="us-east-1", description="AWS region name")

    endpoint_url: str | None = Field(
        default=None, description="Custom S3 endpoint (for MinIO, LocalStack, etc.)"
    )

    # AWS Credentials (optional - can use IAM roles)
    aws_access_key_id: str | None = Field(
        default=None, description="AWS access key ID (optional if using IAM)"
    )

    aws_secret_access_key: str | None = Field(
        default=None, description="AWS secret access key (optional if using IAM)"
    )

    # URL signing
    presigned_url_expiration: int = Field(
        default=3600,
        description="Pre-signed URL expiration in seconds",
        ge=60,
        le=604800,  # Max 7 days
    )

    # Preview settings
    generate_previews: bool = Field(
        default=True, description="Whether to generate preview URLs for images"
    )

    preview_url_expiration: int = Field(
        default=86400,
        description="Preview URL expiration in seconds (24 hours default)",
        ge=300,
        le=604800,
    )

    # Storage settings
    storage_class: Literal["STANDARD", "INTELLIGENT_TIERING", "GLACIER"] = Field(
        default="STANDARD", description="S3 storage class"
    )

    # Key prefix for organization
    key_prefix: str = Field(
        default="chatkit-attachments", description="Prefix for all S3 keys"
    )

    # Upload constraints
    max_file_size: int = Field(
        default=100 * 1024 * 1024,  # 100MB
        description="Maximum file size in bytes",
        ge=1024,  # 1KB minimum
        le=5 * 1024 * 1024 * 1024,  # 5GB maximum
    )
