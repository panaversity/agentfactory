"""Configuration management for PanaversityFS.

Uses pydantic-settings for environment variable loading with validation.
"""

from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import Literal


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

    # =========================================================================
    # Storage Configuration
    # =========================================================================
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

    # =========================================================================
    # Authentication
    # =========================================================================
    #
    # PanaversityFS acts as an OAuth 2.1 Resource Server, validating:
    # 1. Bearer tokens (JWT) - for user sessions via OIDC
    # 2. API keys - for M2M (GitHub Actions, agents, services)
    #
    # Both use the Authorization header:
    # - Authorization: Bearer <jwt-token>
    # - Authorization: ApiKey <api-key>
    #
    # Auth is DISABLED by default. Set auth_server_url to enable.
    # =========================================================================

    # SSO server URL (issuer for JWTs, validator for API keys)
    auth_server_url: str | None = None

    # JWKS endpoint path for JWT verification
    auth_jwks_path: str = "/api/auth/jwks"

    # API key verification endpoint path
    auth_api_key_path: str = "/api/api-key/verify"

    # JWKS cache TTL in seconds (keys rotate every 90 days)
    jwks_cache_ttl: int = 3600  # 1 hour

    # Token validation cache TTL in seconds
    token_cache_ttl: int = 300  # 5 minutes

    # This server's public URL for RFC 9728 Protected Resource Metadata
    resource_server_url: str | None = None

    @property
    def auth_enabled(self) -> bool:
        """Check if authentication is enabled."""
        return bool(self.auth_server_url)

    @property
    def jwks_url(self) -> str | None:
        """Get full JWKS endpoint URL."""
        if not self.auth_server_url:
            return None
        return f"{self.auth_server_url.rstrip('/')}{self.auth_jwks_path}"

    @property
    def api_key_verify_url(self) -> str | None:
        """Get full API key verification endpoint URL."""
        if not self.auth_server_url:
            return None
        return f"{self.auth_server_url.rstrip('/')}{self.auth_api_key_path}"

    # =========================================================================
    # Server Configuration
    # =========================================================================
    server_host: str = "0.0.0.0"
    server_port: int = 8000

    # =========================================================================
    # Observability
    # =========================================================================
    sentry_dsn: str | None = None
    log_level: Literal["DEBUG", "INFO", "WARNING", "ERROR"] = "INFO"

    # =========================================================================
    # Asset Upload Configuration
    # =========================================================================
    cdn_base_url: str = "https://cdn.panaversity.com"
    max_direct_upload_mb: int = 10  # <10MB direct, >=10MB presigned
    max_asset_size_mb: int = 100

    # =========================================================================
    # Archive Generation Configuration
    # =========================================================================
    archive_timeout_seconds: int = 60  # 500 files / 200MB within 60s
    presign_expiry_seconds: int = 3600  # 1 hour

    # =========================================================================
    # Database Configuration
    # =========================================================================
    # PostgreSQL: postgresql+asyncpg://user:pass@host/db
    # SQLite: sqlite+aiosqlite:///./panaversity_fs.db
    database_url: str | None = None

    @property
    def effective_database_url(self) -> str:
        """Get database URL, defaulting to SQLite for development."""
        return self.database_url or "sqlite+aiosqlite:///./panaversity_fs.db"

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
