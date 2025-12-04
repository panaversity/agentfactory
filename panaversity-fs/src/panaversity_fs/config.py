"""Configuration management for PanaversityFS.

Uses pydantic-settings for environment variable loading with validation.
"""

from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import field_validator
from typing import Literal, List


class Config(BaseSettings):
    """PanaversityFS configuration loaded from environment variables.

    Environment variables are prefixed with PANAVERSITY_ by default.
    Example: PANAVERSITY_STORAGE_BACKEND=s3
    """

    model_config = SettingsConfigDict(
        env_prefix="PANAVERSITY_",
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
        case_sensitive=False
    )

    # Storage Configuration
    storage_backend: Literal["fs", "s3", "supabase"] = "fs"
    storage_root: str = "/tmp/panaversity-fs-data"

    # S3/R2 Configuration (for s3 backend)
    s3_bucket: str | None = None
    s3_region: str = "auto"
    s3_access_key_id: str | None = None
    s3_secret_access_key: str | None = None
    s3_endpoint: str | None = None  # For R2: https://[account-id].r2.cloudflarestorage.com

    # Supabase Configuration (for supabase backend)
    supabase_bucket: str | None = None
    supabase_url: str | None = None
    supabase_service_role_key: str | None = None

    # Authentication (Legacy API Key - DEPRECATED and non-functional, use JWT instead)
    # This field exists only for backwards compatibility and does nothing
    api_key: str | None = None

    # JWT Authentication (OAuth 2.1 compliant)
    # If jwt_secret is set, JWT auth is enabled; otherwise server runs in dev mode
    jwt_secret: str | None = None  # Secret key for HS256 JWT verification
    jwt_algorithm: str = "HS256"  # JWT algorithm (HS256, HS384, HS512)
    auth_issuer: str | None = None  # JWT issuer URL (iss claim validation)
    auth_audience: str | None = None  # JWT audience (aud claim validation)
    required_scopes_str: str = "read,write"  # Scopes as comma-separated string
    resource_server_url: str | None = None  # This server's public URL (RFC 9728)

    @property
    def required_scopes(self) -> List[str]:
        """Get required scopes as a list."""
        return [s.strip() for s in self.required_scopes_str.split(",") if s.strip()]

    @property
    def auth_enabled(self) -> bool:
        """Check if authentication is enabled.

        Auth is enabled only if jwt_secret is set to a non-empty value.
        Empty string is treated as disabled (for easier test isolation).
        """
        return bool(self.jwt_secret)

    # Server Configuration
    server_host: str = "0.0.0.0"
    server_port: int = 8000

    # Observability
    sentry_dsn: str | None = None
    log_level: Literal["DEBUG", "INFO", "WARNING", "ERROR"] = "INFO"

    # Asset Upload Configuration
    cdn_base_url: str = "https://cdn.panaversity.com"  # CDN base URL for asset access (FR-013)
    max_direct_upload_mb: int = 10  # Assets <10MB use direct upload, ≥10MB use presigned URLs
    max_asset_size_mb: int = 100  # Maximum asset size (FR-010)

    # Archive Generation Configuration
    archive_timeout_seconds: int = 60  # FR-030: <60s for 500 files / 200MB
    presign_expiry_seconds: int = 3600  # Presigned URL validity (1 hour default)

    def validate_backend_config(self) -> None:
        """Validate that required configuration exists for selected backend.

        Raises:
            ValueError: If required configuration is missing for backend.
        """
        if self.storage_backend == "s3":
            if not self.s3_bucket:
                raise ValueError("S3_BUCKET required when STORAGE_BACKEND=s3")
            if not self.s3_access_key_id or not self.s3_secret_access_key:
                raise ValueError("S3_ACCESS_KEY_ID and S3_SECRET_ACCESS_KEY required for s3 backend")

        elif self.storage_backend == "supabase":
            if not self.supabase_bucket:
                raise ValueError("SUPABASE_BUCKET required when STORAGE_BACKEND=supabase")
            if not self.supabase_url or not self.supabase_service_role_key:
                raise ValueError("SUPABASE_URL and SUPABASE_SERVICE_ROLE_KEY required for supabase backend")


# Global config instance (singleton pattern)
_config: Config | None = None


def get_config() -> Config:
    """Get or create the global configuration instance.

    Returns:
        Config: Validated configuration instance.

    Raises:
        ValueError: If backend configuration is invalid.
    """
    global _config
    if _config is None:
        _config = Config()
        _config.validate_backend_config()
    return _config
