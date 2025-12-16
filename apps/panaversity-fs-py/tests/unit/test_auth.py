"""Unit tests for authentication module.

Tests cover:
1. JWKSTokenVerifier - RS256 JWT validation via JWKS
2. APIKeyVerifier - M2M API key validation
3. DualAuthValidator - Combined bearer + API key validation
4. Config auth settings
"""

import pytest
import os
from unittest.mock import AsyncMock, MagicMock, patch


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def temp_storage_root(tmp_path):
    """Provide a temporary storage root."""
    return str(tmp_path)


@pytest.fixture
def setup_auth_env(temp_storage_root):
    """Setup environment with auth configuration."""
    os.environ['PANAVERSITY_STORAGE_BACKEND'] = 'fs'
    os.environ['PANAVERSITY_STORAGE_ROOT'] = temp_storage_root
    os.environ['PANAVERSITY_AUTH_SERVER_URL'] = 'https://auth.example.com'

    # Clear cached config
    from panaversity_fs import config
    config._config = None

    yield

    # Cleanup
    del os.environ['PANAVERSITY_AUTH_SERVER_URL']
    config._config = None


@pytest.fixture
def no_auth_env(temp_storage_root):
    """Setup environment without auth configuration."""
    os.environ['PANAVERSITY_STORAGE_BACKEND'] = 'fs'
    os.environ['PANAVERSITY_STORAGE_ROOT'] = temp_storage_root

    # Ensure AUTH_SERVER_URL is not set
    if 'PANAVERSITY_AUTH_SERVER_URL' in os.environ:
        del os.environ['PANAVERSITY_AUTH_SERVER_URL']

    # Clear cached config
    from panaversity_fs import config
    config._config = None

    yield

    config._config = None


# =============================================================================
# Config Auth Settings Tests
# =============================================================================

class TestConfigAuthSettings:
    """Test config auth settings."""

    def test_auth_disabled_without_auth_server_url(self, no_auth_env):
        """Test that auth is disabled when AUTH_SERVER_URL not set."""
        from panaversity_fs.config import get_config

        config = get_config()

        assert config.auth_enabled is False
        assert config.auth_server_url is None

    def test_auth_enabled_with_auth_server_url(self, setup_auth_env):
        """Test that auth is enabled when AUTH_SERVER_URL is set."""
        from panaversity_fs.config import get_config

        config = get_config()

        assert config.auth_enabled is True
        assert config.auth_server_url == 'https://auth.example.com'

    def test_jwks_url_computed(self, setup_auth_env):
        """Test that JWKS URL is correctly computed."""
        from panaversity_fs.config import get_config

        config = get_config()

        assert config.jwks_url == 'https://auth.example.com/api/auth/jwks'

    def test_api_key_verify_url_computed(self, setup_auth_env):
        """Test that API key verify URL is correctly computed."""
        from panaversity_fs.config import get_config

        config = get_config()

        assert config.api_key_verify_url == 'https://auth.example.com/api/api-key/verify'

    def test_default_cache_ttls(self, setup_auth_env):
        """Test default cache TTL values."""
        from panaversity_fs.config import get_config

        config = get_config()

        assert config.jwks_cache_ttl == 3600  # 1 hour
        assert config.token_cache_ttl == 300  # 5 minutes


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
        from panaversity_fs.auth import get_auth_settings, CombinedTokenVerifier
        from mcp.server.auth.settings import AuthSettings

        verifier, settings = get_auth_settings()

        assert verifier is not None
        # CombinedTokenVerifier handles both JWT and API key auth
        assert isinstance(verifier, CombinedTokenVerifier)
        assert settings is not None
        assert isinstance(settings, AuthSettings)

    def test_auth_settings_issuer_url(self, setup_auth_env):
        """Test that AuthSettings has correct issuer URL."""
        from panaversity_fs.auth import get_auth_settings

        verifier, settings = get_auth_settings()

        assert settings is not None
        assert str(settings.issuer_url) == "https://auth.example.com/"


# =============================================================================
# get_dual_auth_validator Tests
# =============================================================================

class TestGetDualAuthValidator:
    """Test get_dual_auth_validator function."""

    def test_returns_none_when_disabled(self, no_auth_env):
        """Test that get_dual_auth_validator returns None when auth disabled."""
        from panaversity_fs.auth import get_dual_auth_validator

        validator = get_dual_auth_validator()

        assert validator is None

    def test_returns_validator_when_enabled(self, setup_auth_env):
        """Test that get_dual_auth_validator returns validator when enabled."""
        from panaversity_fs.auth import get_dual_auth_validator, DualAuthValidator

        validator = get_dual_auth_validator()

        assert validator is not None
        assert isinstance(validator, DualAuthValidator)


# =============================================================================
# JWKSTokenVerifier Tests
# =============================================================================

class TestJWKSTokenVerifier:
    """Test JWKSTokenVerifier class."""

    @pytest.mark.asyncio
    async def test_malformed_token_rejected(self):
        """Test that malformed token is rejected."""
        from panaversity_fs.auth import JWKSTokenVerifier

        verifier = JWKSTokenVerifier(
            jwks_url="https://auth.example.com/api/auth/jwks",
            issuer="https://auth.example.com"
        )

        result = await verifier.verify_token("not-a-valid-jwt")

        assert result is None

    @pytest.mark.asyncio
    async def test_empty_token_rejected(self):
        """Test that empty token is rejected."""
        from panaversity_fs.auth import JWKSTokenVerifier

        verifier = JWKSTokenVerifier(
            jwks_url="https://auth.example.com/api/auth/jwks",
            issuer="https://auth.example.com"
        )

        result = await verifier.verify_token("")

        assert result is None

    @pytest.mark.asyncio
    async def test_token_caching(self):
        """Test that tokens are cached after validation."""
        from panaversity_fs.auth import JWKSTokenVerifier

        verifier = JWKSTokenVerifier(
            jwks_url="https://auth.example.com/api/auth/jwks",
            issuer="https://auth.example.com",
            token_cache_ttl=300
        )

        # Cache should be empty initially
        assert len(verifier._token_cache) == 0


# =============================================================================
# APIKeyVerifier Tests
# =============================================================================

class TestAPIKeyVerifier:
    """Test APIKeyVerifier class."""

    @pytest.mark.asyncio
    async def test_invalid_api_key_returns_none(self):
        """Test that invalid API key returns None."""
        from panaversity_fs.auth import APIKeyVerifier

        verifier = APIKeyVerifier(
            verify_url="https://auth.example.com/api/auth/api-key/verify"
        )

        # Mock the HTTP client to return 401
        with patch.object(verifier, '_get_client') as mock_get_client:
            mock_client = AsyncMock()
            mock_response = MagicMock()
            mock_response.status_code = 401
            mock_client.post = AsyncMock(return_value=mock_response)
            mock_get_client.return_value = mock_client

            result = await verifier.verify_api_key("sk_live_invalid")

            assert result is None

    @pytest.mark.asyncio
    async def test_valid_api_key_returns_context(self):
        """Test that valid API key returns AuthContext."""
        from panaversity_fs.auth import APIKeyVerifier, AuthContext

        verifier = APIKeyVerifier(
            verify_url="https://auth.example.com/api/auth/api-key/verify"
        )

        # Mock the HTTP client to return valid response
        with patch.object(verifier, '_get_client') as mock_get_client:
            mock_client = AsyncMock()
            mock_response = MagicMock()
            mock_response.status_code = 200
            mock_response.json.return_value = {
                "valid": True,
                "key": {
                    "userId": "user-123",
                    "permissions": {"fs:read": True, "fs:write": True},
                    "metadata": {"service": "github-actions"}
                }
            }
            mock_client.post = AsyncMock(return_value=mock_response)
            mock_get_client.return_value = mock_client

            result = await verifier.verify_api_key("sk_live_xxxxxxxxxxxxx")

            assert result is not None
            assert isinstance(result, AuthContext)
            assert result.user_id == "user-123"
            assert result.auth_type == "api_key"
            assert result.permissions == {"fs:read": True, "fs:write": True}
            assert result.metadata == {"service": "github-actions"}

    @pytest.mark.asyncio
    async def test_network_error_returns_none(self):
        """Test that network error returns None."""
        from panaversity_fs.auth import APIKeyVerifier

        verifier = APIKeyVerifier(
            verify_url="https://auth.example.com/api/auth/api-key/verify"
        )

        # Mock the HTTP client to raise an exception
        with patch.object(verifier, '_get_client') as mock_get_client:
            mock_client = AsyncMock()
            mock_client.post = AsyncMock(side_effect=Exception("Network error"))
            mock_get_client.return_value = mock_client

            result = await verifier.verify_api_key("sk_live_xxxxxxxxxxxxx")

            assert result is None


# =============================================================================
# DualAuthValidator Tests
# =============================================================================

class TestDualAuthValidator:
    """Test DualAuthValidator class."""

    @pytest.mark.asyncio
    async def test_no_auth_returns_none(self):
        """Test that no authentication returns None."""
        from panaversity_fs.auth import DualAuthValidator, JWKSTokenVerifier, APIKeyVerifier

        jwks_verifier = JWKSTokenVerifier(
            jwks_url="https://auth.example.com/api/auth/jwks",
            issuer="https://auth.example.com"
        )
        api_key_verifier = APIKeyVerifier(
            verify_url="https://auth.example.com/api/auth/api-key/verify"
        )
        validator = DualAuthValidator(jwks_verifier, api_key_verifier)

        result = await validator.validate(authorization=None, x_api_key=None)

        assert result is None

    @pytest.mark.asyncio
    async def test_bearer_token_validates_jwt(self):
        """Test that Bearer token triggers JWT validation."""
        from panaversity_fs.auth import DualAuthValidator, JWKSTokenVerifier, APIKeyVerifier, AuthContext
        from mcp.server.auth.provider import AccessToken

        jwks_verifier = JWKSTokenVerifier(
            jwks_url="https://auth.example.com/api/auth/jwks",
            issuer="https://auth.example.com"
        )
        api_key_verifier = APIKeyVerifier(
            verify_url="https://auth.example.com/api/auth/api-key/verify"
        )
        validator = DualAuthValidator(jwks_verifier, api_key_verifier)

        # Mock the JWKS verifier to succeed
        with patch.object(jwks_verifier, 'verify_token') as mock_jwks:
            mock_jwks.return_value = AccessToken(
                token="valid_jwt",
                client_id="user-123",
                scopes=["fs:read"],
                expires_at=None
            )

            result = await validator.validate(authorization="Bearer valid_jwt")

            mock_jwks.assert_called_once_with("valid_jwt")
            assert result is not None
            assert result.user_id == "user-123"
            assert result.auth_type == "bearer"

    @pytest.mark.asyncio
    async def test_invalid_bearer_returns_none(self):
        """Test that invalid Bearer token returns None."""
        from panaversity_fs.auth import DualAuthValidator, JWKSTokenVerifier, APIKeyVerifier

        jwks_verifier = JWKSTokenVerifier(
            jwks_url="https://auth.example.com/api/auth/jwks",
            issuer="https://auth.example.com"
        )
        api_key_verifier = APIKeyVerifier(
            verify_url="https://auth.example.com/api/auth/api-key/verify"
        )
        validator = DualAuthValidator(jwks_verifier, api_key_verifier)

        # Mock the JWKS verifier to fail
        with patch.object(jwks_verifier, 'verify_token', return_value=None) as mock_jwks:
            result = await validator.validate(authorization="Bearer invalid_token")

            mock_jwks.assert_called_once_with("invalid_token")
            assert result is None

    @pytest.mark.asyncio
    async def test_api_key_validates_via_x_api_key_header(self):
        """Test that x-api-key header triggers API key validation."""
        from panaversity_fs.auth import DualAuthValidator, JWKSTokenVerifier, APIKeyVerifier, AuthContext

        jwks_verifier = JWKSTokenVerifier(
            jwks_url="https://auth.example.com/api/auth/jwks",
            issuer="https://auth.example.com"
        )
        api_key_verifier = APIKeyVerifier(
            verify_url="https://auth.example.com/api/auth/api-key/verify"
        )
        validator = DualAuthValidator(jwks_verifier, api_key_verifier)

        with patch.object(api_key_verifier, 'verify_api_key') as mock_api_key:
            mock_api_key.return_value = AuthContext(
                user_id="service-account",
                auth_type="api_key",
                permissions={"fs:read": True, "fs:write": True},
                metadata={"service": "github-actions"}
            )

            # Use x-api-key header (Better Auth default)
            result = await validator.validate(x_api_key="sk_live_xxxxxxxxxxxxx")

            mock_api_key.assert_called_once_with("sk_live_xxxxxxxxxxxxx")
            assert result is not None
            assert result.user_id == "service-account"
            assert result.auth_type == "api_key"
            assert result.metadata == {"service": "github-actions"}

    @pytest.mark.asyncio
    async def test_unknown_auth_scheme_returns_none(self):
        """Test that unknown authorization scheme returns None."""
        from panaversity_fs.auth import DualAuthValidator, JWKSTokenVerifier, APIKeyVerifier

        jwks_verifier = JWKSTokenVerifier(
            jwks_url="https://auth.example.com/api/auth/jwks",
            issuer="https://auth.example.com"
        )
        api_key_verifier = APIKeyVerifier(
            verify_url="https://auth.example.com/api/auth/api-key/verify"
        )
        validator = DualAuthValidator(jwks_verifier, api_key_verifier)

        # Unknown scheme (not Bearer, no x-api-key)
        result = await validator.validate(authorization="Basic dXNlcjpwYXNz")

        assert result is None

    @pytest.mark.asyncio
    async def test_x_api_key_takes_precedence_over_bearer(self):
        """Test that x-api-key header takes precedence when both are provided."""
        from panaversity_fs.auth import DualAuthValidator, JWKSTokenVerifier, APIKeyVerifier, AuthContext

        jwks_verifier = JWKSTokenVerifier(
            jwks_url="https://auth.example.com/api/auth/jwks",
            issuer="https://auth.example.com"
        )
        api_key_verifier = APIKeyVerifier(
            verify_url="https://auth.example.com/api/auth/api-key/verify"
        )
        validator = DualAuthValidator(jwks_verifier, api_key_verifier)

        with patch.object(api_key_verifier, 'verify_api_key') as mock_api_key:
            mock_api_key.return_value = AuthContext(
                user_id="service-account",
                auth_type="api_key"
            )

            # Both headers provided - x-api-key should win
            result = await validator.validate(
                authorization="Bearer some_jwt_token",
                x_api_key="sk_live_xxxxxxxxxxxxx"
            )

            mock_api_key.assert_called_once_with("sk_live_xxxxxxxxxxxxx")
            assert result is not None
            assert result.auth_type == "api_key"


# =============================================================================
# AuthContext Tests
# =============================================================================

class TestAuthContext:
    """Test AuthContext dataclass."""

    def test_auth_context_creation(self):
        """Test creating AuthContext with all fields."""
        from panaversity_fs.auth import AuthContext

        context = AuthContext(
            user_id="user-123",
            auth_type="bearer",
            role="admin",
            tenant_id="org-456",
            org_role="owner",
            permissions={"read": True},
            metadata={"source": "web"}
        )

        assert context.user_id == "user-123"
        assert context.auth_type == "bearer"
        assert context.role == "admin"
        assert context.tenant_id == "org-456"
        assert context.org_role == "owner"
        assert context.permissions == {"read": True}
        assert context.metadata == {"source": "web"}

    def test_auth_context_minimal(self):
        """Test creating AuthContext with minimal fields."""
        from panaversity_fs.auth import AuthContext

        context = AuthContext(
            user_id="user-123",
            auth_type="api_key"
        )

        assert context.user_id == "user-123"
        assert context.auth_type == "api_key"
        assert context.role is None
        assert context.tenant_id is None
        assert context.permissions is None
