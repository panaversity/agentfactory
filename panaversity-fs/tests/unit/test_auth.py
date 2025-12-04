"""Unit tests for JWT authentication module.

Tests cover:
1. JWTTokenVerifier - token validation
2. Config auth settings
3. Token generation utility
"""

import pytest
import jwt
import os
from datetime import datetime, timezone, timedelta


# Test fixtures
@pytest.fixture
def jwt_secret():
    """Test JWT secret."""
    return "test-secret-key-for-testing-only"


@pytest.fixture
def setup_auth_env(jwt_secret, temp_storage_root):
    """Setup environment with auth configuration."""
    os.environ['PANAVERSITY_STORAGE_BACKEND'] = 'fs'
    os.environ['PANAVERSITY_STORAGE_ROOT'] = temp_storage_root
    os.environ['PANAVERSITY_JWT_SECRET'] = jwt_secret
    os.environ['PANAVERSITY_JWT_ALGORITHM'] = 'HS256'
    os.environ['PANAVERSITY_REQUIRED_SCOPES_STR'] = 'read,write'

    # Clear cached config
    from panaversity_fs import config
    config._config = None

    yield

    # Cleanup
    del os.environ['PANAVERSITY_JWT_SECRET']
    del os.environ['PANAVERSITY_JWT_ALGORITHM']
    del os.environ['PANAVERSITY_REQUIRED_SCOPES_STR']
    if 'PANAVERSITY_AUTH_ISSUER' in os.environ:
        del os.environ['PANAVERSITY_AUTH_ISSUER']
    if 'PANAVERSITY_RESOURCE_SERVER_URL' in os.environ:
        del os.environ['PANAVERSITY_RESOURCE_SERVER_URL']
    config._config = None


@pytest.fixture
def no_auth_env(temp_storage_root):
    """Setup environment without auth configuration."""
    os.environ['PANAVERSITY_STORAGE_BACKEND'] = 'fs'
    os.environ['PANAVERSITY_STORAGE_ROOT'] = temp_storage_root

    # Explicitly set JWT_SECRET to empty to override .env file
    # pydantic-settings will treat empty string as None for Optional[str]
    os.environ['PANAVERSITY_JWT_SECRET'] = ''

    # Clear cached config
    from panaversity_fs import config
    config._config = None

    yield

    # Cleanup - restore to allow .env to work again
    del os.environ['PANAVERSITY_JWT_SECRET']
    config._config = None


def create_token(secret: str, scopes: list = None, expires_in: int = 3600,
                 issuer: str = None, algorithm: str = "HS256") -> str:
    """Helper to create test JWT tokens."""
    now = datetime.now(timezone.utc)
    payload = {
        "sub": "test-agent",
        "scopes": scopes or ["read", "write"],
        "iat": int(now.timestamp()),
        "exp": int(now.timestamp()) + expires_in
    }
    if issuer:
        payload["iss"] = issuer

    return jwt.encode(payload, secret, algorithm=algorithm)


# =============================================================================
# JWTTokenVerifier Tests
# =============================================================================

class TestJWTTokenVerifier:
    """Test JWTTokenVerifier class."""

    @pytest.mark.asyncio
    async def test_valid_token_accepted(self, jwt_secret):
        """Test that valid token is accepted."""
        from panaversity_fs.auth import JWTTokenVerifier

        verifier = JWTTokenVerifier(secret=jwt_secret)
        token = create_token(jwt_secret)

        result = await verifier.verify_token(token)

        assert result is not None
        assert result.token == token
        assert result.client_id == "test-agent"  # from 'sub' claim
        assert "read" in result.scopes
        assert "write" in result.scopes

    @pytest.mark.asyncio
    async def test_invalid_signature_rejected(self, jwt_secret):
        """Test that token with wrong signature is rejected."""
        from panaversity_fs.auth import JWTTokenVerifier

        verifier = JWTTokenVerifier(secret=jwt_secret)
        token = create_token("wrong-secret")

        result = await verifier.verify_token(token)

        assert result is None

    @pytest.mark.asyncio
    async def test_expired_token_rejected(self, jwt_secret):
        """Test that expired token is rejected."""
        from panaversity_fs.auth import JWTTokenVerifier

        verifier = JWTTokenVerifier(secret=jwt_secret)
        token = create_token(jwt_secret, expires_in=-3600)  # Expired 1 hour ago

        result = await verifier.verify_token(token)

        assert result is None

    @pytest.mark.asyncio
    async def test_malformed_token_rejected(self, jwt_secret):
        """Test that malformed token is rejected."""
        from panaversity_fs.auth import JWTTokenVerifier

        verifier = JWTTokenVerifier(secret=jwt_secret)

        result = await verifier.verify_token("not-a-valid-jwt")

        assert result is None

    @pytest.mark.asyncio
    async def test_empty_token_rejected(self, jwt_secret):
        """Test that empty token is rejected."""
        from panaversity_fs.auth import JWTTokenVerifier

        verifier = JWTTokenVerifier(secret=jwt_secret)

        result = await verifier.verify_token("")

        assert result is None

    @pytest.mark.asyncio
    async def test_issuer_validation(self, jwt_secret):
        """Test that issuer claim is validated when configured."""
        from panaversity_fs.auth import JWTTokenVerifier

        verifier = JWTTokenVerifier(
            secret=jwt_secret,
            issuer="https://auth.example.com"
        )

        # Token with correct issuer
        valid_token = create_token(jwt_secret, issuer="https://auth.example.com")
        result = await verifier.verify_token(valid_token)
        assert result is not None

        # Token with wrong issuer
        invalid_token = create_token(jwt_secret, issuer="https://wrong.com")
        result = await verifier.verify_token(invalid_token)
        assert result is None

    @pytest.mark.asyncio
    async def test_required_scopes_validation(self, jwt_secret):
        """Test that required scopes are validated."""
        from panaversity_fs.auth import JWTTokenVerifier

        verifier = JWTTokenVerifier(
            secret=jwt_secret,
            required_scopes=["admin"]
        )

        # Token without admin scope
        token_no_admin = create_token(jwt_secret, scopes=["read", "write"])
        result = await verifier.verify_token(token_no_admin)
        assert result is None

        # Token with admin scope
        token_with_admin = create_token(jwt_secret, scopes=["read", "write", "admin"])
        result = await verifier.verify_token(token_with_admin)
        assert result is not None

    @pytest.mark.asyncio
    async def test_scopes_as_string(self, jwt_secret):
        """Test that scopes can be comma-separated string in token."""
        from panaversity_fs.auth import JWTTokenVerifier

        verifier = JWTTokenVerifier(secret=jwt_secret)

        # Create token with scopes as string
        now = datetime.now(timezone.utc)
        payload = {
            "sub": "test",
            "scopes": "read,write",  # String instead of list
            "iat": int(now.timestamp()),
            "exp": int(now.timestamp()) + 3600
        }
        token = jwt.encode(payload, jwt_secret, algorithm="HS256")

        result = await verifier.verify_token(token)

        assert result is not None
        assert "read" in result.scopes
        assert "write" in result.scopes

    @pytest.mark.asyncio
    async def test_expiration_returned(self, jwt_secret):
        """Test that expiration is returned in AccessToken."""
        from panaversity_fs.auth import JWTTokenVerifier

        verifier = JWTTokenVerifier(secret=jwt_secret)
        token = create_token(jwt_secret, expires_in=7200)

        result = await verifier.verify_token(token)

        assert result is not None
        assert result.expires_at is not None
        # Should expire in approximately 2 hours
        expected_exp = int(datetime.now(timezone.utc).timestamp()) + 7200
        assert abs(result.expires_at - expected_exp) < 10


# =============================================================================
# Config Auth Settings Tests
# =============================================================================

class TestConfigAuthSettings:
    """Test config auth settings."""

    def test_auth_disabled_without_jwt_secret(self, no_auth_env):
        """Test that auth is disabled when JWT_SECRET not set or empty."""
        from panaversity_fs.config import get_config

        config = get_config()

        assert config.auth_enabled is False
        assert not config.jwt_secret  # Empty string or None both work

    def test_auth_enabled_with_jwt_secret(self, setup_auth_env, jwt_secret):
        """Test that auth is enabled when JWT_SECRET is set."""
        from panaversity_fs.config import get_config

        config = get_config()

        assert config.auth_enabled is True
        assert config.jwt_secret == jwt_secret

    def test_scopes_parsed_from_string(self, temp_storage_root):
        """Test that REQUIRED_SCOPES_STR string is parsed to list."""
        os.environ['PANAVERSITY_STORAGE_BACKEND'] = 'fs'
        os.environ['PANAVERSITY_STORAGE_ROOT'] = temp_storage_root
        os.environ['PANAVERSITY_JWT_SECRET'] = 'test'
        os.environ['PANAVERSITY_REQUIRED_SCOPES_STR'] = 'read, write, admin'

        from panaversity_fs import config
        config._config = None

        cfg = config.get_config()

        assert cfg.required_scopes == ["read", "write", "admin"]

        # Cleanup
        del os.environ['PANAVERSITY_JWT_SECRET']
        del os.environ['PANAVERSITY_REQUIRED_SCOPES_STR']
        config._config = None

    def test_default_scopes(self, setup_auth_env):
        """Test default required scopes."""
        from panaversity_fs.config import get_config

        config = get_config()

        assert "read" in config.required_scopes
        assert "write" in config.required_scopes


# =============================================================================
# get_auth_settings Tests
# =============================================================================

class TestGetAuthSettings:
    """Test get_auth_settings function."""

    def test_returns_none_when_disabled(self, no_auth_env):
        """Test that get_auth_settings returns None when auth disabled."""
        from panaversity_fs.auth import get_auth_settings

        verifier, settings = get_auth_settings()

        assert verifier is None
        assert settings is None

    def test_returns_verifier_when_enabled(self, setup_auth_env):
        """Test that get_auth_settings returns verifier and settings when enabled."""
        from panaversity_fs.auth import get_auth_settings, JWTTokenVerifier
        from mcp.server.auth.settings import AuthSettings

        verifier, settings = get_auth_settings()

        assert verifier is not None
        assert isinstance(verifier, JWTTokenVerifier)
        # MCP SDK requires auth settings when token_verifier is provided
        assert settings is not None
        assert isinstance(settings, AuthSettings)

    def test_returns_settings_with_issuer(self, setup_auth_env):
        """Test that AuthSettings returned when issuer configured."""
        os.environ['PANAVERSITY_AUTH_ISSUER'] = 'https://auth.example.com'
        os.environ['PANAVERSITY_RESOURCE_SERVER_URL'] = 'http://localhost:8000'

        # Clear cached config
        from panaversity_fs import config
        config._config = None

        from panaversity_fs.auth import get_auth_settings

        verifier, settings = get_auth_settings()

        assert settings is not None
        assert str(settings.issuer_url) == "https://auth.example.com/"


# =============================================================================
# create_test_token Tests
# =============================================================================

class TestCreateTestToken:
    """Test create_test_token utility."""

    def test_create_token_without_secret_raises(self, no_auth_env):
        """Test that creating token without secret raises error."""
        from panaversity_fs.auth import create_test_token

        with pytest.raises(ValueError) as exc_info:
            create_test_token()

        assert "JWT_SECRET" in str(exc_info.value)

    def test_create_token_with_default_scopes(self, setup_auth_env, jwt_secret):
        """Test that created token has default scopes."""
        from panaversity_fs.auth import create_test_token, JWTTokenVerifier

        token = create_test_token()
        verifier = JWTTokenVerifier(secret=jwt_secret)

        # Verify token is valid
        import asyncio
        result = asyncio.get_event_loop().run_until_complete(
            verifier.verify_token(token)
        )

        assert result is not None
        assert "read" in result.scopes
        assert "write" in result.scopes

    def test_create_token_with_custom_scopes(self, setup_auth_env, jwt_secret):
        """Test creating token with custom scopes."""
        from panaversity_fs.auth import create_test_token, JWTTokenVerifier

        token = create_test_token(scopes=["admin", "superuser"])
        verifier = JWTTokenVerifier(secret=jwt_secret)

        import asyncio
        result = asyncio.get_event_loop().run_until_complete(
            verifier.verify_token(token)
        )

        assert result is not None
        assert "admin" in result.scopes
        assert "superuser" in result.scopes

    def test_create_token_expiration(self, setup_auth_env, jwt_secret):
        """Test that created token has correct expiration."""
        from panaversity_fs.auth import create_test_token

        token = create_test_token(expires_in_seconds=60)

        # Decode without verification to check exp
        decoded = jwt.decode(token, options={"verify_signature": False})

        now = int(datetime.now(timezone.utc).timestamp())
        assert decoded["exp"] - now <= 65  # Allow 5s buffer
        assert decoded["exp"] - now >= 55
