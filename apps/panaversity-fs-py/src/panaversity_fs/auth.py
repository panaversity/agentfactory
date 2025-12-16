"""Authentication for PanaversityFS MCP Server.

Implements dual authentication:
1. Bearer tokens (JWT) - for user sessions via OIDC, validated via JWKS
2. API keys - for M2M (GitHub Actions, agents), validated via API call

Authentication methods:
- Authorization: Bearer <jwt-token> - for user sessions
- x-api-key: <api-key> - for M2M authentication

Both authentication methods extract user_id for audit logging.

Environment Variables:
    PANAVERSITY_AUTH_SERVER_URL: SSO server URL (enables auth when set)

Token Claims:
    - sub: User ID (primary identifier for audit logging)
    - iss: Issuer URL
    - role: User role (admin or user)
    - tenant_id: Primary organization ID
    - organization_ids: List of organization IDs

API Key Response:
    - userId: User ID for audit logging
    - permissions: Resource-based permissions (fs:read, fs:write, fs:admin)
    - metadata: Service metadata (e.g., repository, service name)
"""

import asyncio
import hashlib
import time
import jwt
import httpx
from jwt import PyJWKClient, PyJWKClientError
from dataclasses import dataclass
from typing import Optional

from mcp.server.auth.provider import AccessToken, TokenVerifier
from mcp.server.auth.settings import AuthSettings
from pydantic import AnyHttpUrl

from panaversity_fs.config import get_config


# =============================================================================
# Authentication Context
# =============================================================================

@dataclass
class AuthContext:
    """Authentication context extracted from token or API key."""
    user_id: str
    auth_type: str  # "bearer" or "api_key"
    role: str | None = None
    tenant_id: str | None = None
    org_role: str | None = None
    permissions: dict | None = None
    metadata: dict | None = None


# =============================================================================
# Token Cache
# =============================================================================

@dataclass
class CachedToken:
    """Cached validated token with expiry."""
    access_token: AccessToken
    cached_at: float
    ttl: int

    def is_expired(self) -> bool:
        """Check if cache entry has expired."""
        return time.time() > (self.cached_at + self.ttl)


# =============================================================================
# JWKS Token Verifier (Bearer Tokens)
# =============================================================================

class JWKSTokenVerifier(TokenVerifier):
    """Verify JWT tokens using JWKS from Better Auth.

    Features:
    - RS256 asymmetric verification (no shared secrets)
    - JWKS auto-refresh with caching (1 hour default)
    - Token validation caching (5 minutes default)
    - Key rotation handling via `kid` header

    Token Validation:
    1. Verify signature using JWKS
    2. Check exp claim hasn't passed
    3. Verify iss matches expected issuer
    4. Extract sub for audit logging
    """

    def __init__(
        self,
        jwks_url: str,
        issuer: str,
        jwks_cache_ttl: int = 3600,
        token_cache_ttl: int = 300
    ):
        """Initialize JWKS token verifier.

        Args:
            jwks_url: JWKS endpoint URL
            issuer: Expected issuer claim
            jwks_cache_ttl: JWKS cache TTL in seconds (default: 1 hour)
            token_cache_ttl: Token cache TTL in seconds (default: 5 minutes)
        """
        self.jwks_url = jwks_url
        self.issuer = issuer
        self.jwks_cache_ttl = jwks_cache_ttl
        self.token_cache_ttl = token_cache_ttl

        # JWKS client (lazy initialized)
        self._jwks_client: PyJWKClient | None = None
        self._jwks_client_lock = asyncio.Lock()

        # Token validation cache
        self._token_cache: dict[str, CachedToken] = {}
        self._cache_lock = asyncio.Lock()

    async def _get_jwks_client(self) -> PyJWKClient:
        """Get or create JWKS client with caching."""
        if self._jwks_client is None:
            async with self._jwks_client_lock:
                if self._jwks_client is None:
                    self._jwks_client = PyJWKClient(
                        self.jwks_url,
                        cache_jwk_set=True,
                        lifespan=self.jwks_cache_ttl
                    )
        return self._jwks_client

    async def _get_cached_token(self, token: str) -> AccessToken | None:
        """Get token from cache if valid."""
        async with self._cache_lock:
            cached = self._token_cache.get(token)
            if cached and not cached.is_expired():
                if cached.access_token.expires_at:
                    if time.time() < cached.access_token.expires_at:
                        return cached.access_token
                else:
                    return cached.access_token
            if cached:
                del self._token_cache[token]
        return None

    async def _cache_token(self, token: str, access_token: AccessToken) -> None:
        """Cache validated token."""
        async with self._cache_lock:
            self._token_cache[token] = CachedToken(
                access_token=access_token,
                cached_at=time.time(),
                ttl=self.token_cache_ttl
            )
            # Prune cache if too large
            if len(self._token_cache) > 1000:
                entries = sorted(
                    self._token_cache.items(),
                    key=lambda x: x[1].cached_at
                )
                for key, _ in entries[:100]:
                    del self._token_cache[key]

    async def verify_token(self, token: str) -> AccessToken | None:
        """Verify JWT using JWKS and return AccessToken if valid.

        Args:
            token: JWT string from Authorization header

        Returns:
            AccessToken if valid, None if invalid
        """
        # Check cache first
        cached = await self._get_cached_token(token)
        if cached:
            return cached

        try:
            jwks_client = await self._get_jwks_client()
            signing_key = jwks_client.get_signing_key_from_jwt(token)

            payload = jwt.decode(
                token,
                signing_key.key,
                algorithms=["RS256"],
                issuer=self.issuer
            )

            # Extract user_id from 'sub' claim
            client_id = payload.get("sub", "unknown")

            # Extract scopes (Better Auth uses 'scope' space-separated)
            token_scopes = payload.get("scope", "")
            if isinstance(token_scopes, str):
                token_scopes = token_scopes.split() if token_scopes else []

            exp = payload.get("exp")
            expires_at = int(exp) if exp else None

            access_token = AccessToken(
                token=token,
                client_id=client_id,
                scopes=token_scopes,
                expires_at=expires_at
            )

            await self._cache_token(token, access_token)
            return access_token

        except jwt.ExpiredSignatureError:
            return None
        except jwt.InvalidTokenError:
            return None
        except PyJWKClientError:
            return None
        except Exception:
            return None


# =============================================================================
# API Key Verifier (M2M)
# =============================================================================

class APIKeyVerifier:
    """Verify API keys against SSO API key endpoint.

    For M2M authentication (GitHub Actions, agents, services).

    API keys use prefix format: sk_live_xxx (production), sk_test_xxx (development)

    Response includes:
    - userId: User ID for audit logging
    - permissions: Resource-based access control (fs:read, fs:write, fs:admin)
    - metadata: Service metadata

    Caching:
    - Successful verifications are cached for 5 minutes to reduce auth server load
    - Cache key is a hash of the API key (not the key itself for security)
    - Failed verifications are NOT cached (allow immediate retry)
    """

    # Cache TTL in seconds (5 minutes)
    CACHE_TTL = 300

    def __init__(self, verify_url: str):
        """Initialize API key verifier.

        Args:
            verify_url: API key verification endpoint URL
        """
        self.verify_url = verify_url
        self._client: httpx.AsyncClient | None = None
        # Cache: key_hash -> (AuthContext, expiry_time)
        self._cache: dict[str, tuple[AuthContext, float]] = {}

    def _get_key_hash(self, api_key: str) -> str:
        """Get a hash of the API key for cache lookup (security: don't store raw key)."""
        return hashlib.sha256(api_key.encode()).hexdigest()[:16]

    def _get_cached(self, api_key: str) -> Optional[AuthContext]:
        """Get cached auth context if valid and not expired."""
        key_hash = self._get_key_hash(api_key)
        if key_hash in self._cache:
            auth_context, expiry = self._cache[key_hash]
            if time.time() < expiry:
                return auth_context
            # Expired, remove from cache
            del self._cache[key_hash]
        return None

    def _set_cached(self, api_key: str, auth_context: AuthContext) -> None:
        """Cache a successful auth context."""
        key_hash = self._get_key_hash(api_key)
        expiry = time.time() + self.CACHE_TTL
        self._cache[key_hash] = (auth_context, expiry)

    async def _get_client(self) -> httpx.AsyncClient:
        """Get or create HTTP client."""
        if self._client is None:
            self._client = httpx.AsyncClient(timeout=10.0)
        return self._client

    async def verify_api_key(self, api_key: str) -> AuthContext | None:
        """Verify API key against SSO.

        Uses a short-lived cache (5 min) to reduce auth server load during
        bulk operations like syncing hundreds of files.

        Args:
            api_key: API key string

        Returns:
            AuthContext if valid, None if invalid
        """
        # Check cache first
        cached = self._get_cached(api_key)
        if cached is not None:
            return cached

        try:
            client = await self._get_client()
            response = await client.post(
                self.verify_url,
                json={"key": api_key}
            )

            if response.status_code != 200:
                return None

            result = response.json()
            if not result.get("valid"):
                return None

            key_data = result.get("key", {})
            auth_context = AuthContext(
                user_id=key_data.get("userId", "unknown"),
                auth_type="api_key",
                permissions=key_data.get("permissions"),
                metadata=key_data.get("metadata")
            )

            # Cache successful verification
            self._set_cached(api_key, auth_context)

            return auth_context

        except Exception:
            return None

    async def close(self) -> None:
        """Close HTTP client."""
        if self._client:
            await self._client.aclose()
            self._client = None


# =============================================================================
# Dual Auth Validator
# =============================================================================

class DualAuthValidator:
    """Validates both Bearer tokens and API keys.

    Supports two authentication methods:
    - Authorization: Bearer <jwt-token> -> JWKS verification
    - x-api-key: <api-key> -> API key verification

    Both methods extract user_id for audit logging.
    """

    def __init__(
        self,
        jwks_verifier: JWKSTokenVerifier,
        api_key_verifier: APIKeyVerifier
    ):
        self.jwks_verifier = jwks_verifier
        self.api_key_verifier = api_key_verifier

    async def validate(
        self,
        authorization: str | None = None,
        x_api_key: str | None = None
    ) -> AuthContext | None:
        """Validate request authentication.

        Checks headers in order:
        1. x-api-key header (M2M authentication)
        2. Authorization: Bearer (user sessions)

        Args:
            authorization: Authorization header value
            x_api_key: x-api-key header value

        Returns:
            AuthContext if authenticated, None otherwise
        """
        # Try x-api-key header first (M2M)
        if x_api_key:
            return await self.api_key_verifier.verify_api_key(x_api_key)

        # Try Bearer token (JWT)
        if authorization and authorization.startswith("Bearer "):
            token = authorization[7:]
            access_token = await self.jwks_verifier.verify_token(token)
            if access_token:
                return AuthContext(
                    user_id=access_token.client_id,
                    auth_type="bearer"
                )

        return None


# =============================================================================
# Combined Token Verifier (JWT + API Key via Bearer)
# =============================================================================

class CombinedTokenVerifier(TokenVerifier):
    """Verifies both JWT tokens and API keys passed via Authorization: Bearer.

    The MCP SDK only passes the token from 'Authorization: Bearer <token>'.
    This verifier detects the token type and routes to the appropriate verifier:
    - Tokens starting with 'pana_' or 'sk_' -> API key verification
    - Other tokens -> JWT/JWKS verification

    This allows clients to use: Authorization: Bearer pana_xxx for API keys.
    """

    def __init__(
        self,
        jwks_verifier: JWKSTokenVerifier,
        api_key_verifier: APIKeyVerifier,
        api_key_prefixes: tuple[str, ...] = ("pana_", "sk_live_", "sk_test_")
    ):
        self.jwks_verifier = jwks_verifier
        self.api_key_verifier = api_key_verifier
        self.api_key_prefixes = api_key_prefixes

    async def verify_token(self, token: str) -> AccessToken | None:
        """Verify token - routes to API key or JWT verifier based on prefix.

        Args:
            token: Token string from Authorization: Bearer header

        Returns:
            AccessToken if valid, None if invalid
        """
        if not token:
            return None

        # Check if it's an API key by prefix
        if any(token.startswith(prefix) for prefix in self.api_key_prefixes):
            # Verify as API key
            auth_context = await self.api_key_verifier.verify_api_key(token)
            if auth_context:
                # Convert AuthContext to AccessToken for MCP SDK compatibility
                return AccessToken(
                    token=token,
                    client_id=auth_context.user_id,
                    scopes=list(auth_context.permissions.keys()) if auth_context.permissions else [],
                    expires_at=None
                )
            return None

        # Otherwise verify as JWT
        return await self.jwks_verifier.verify_token(token)


# =============================================================================
# Auth Settings Factory
# =============================================================================

def get_auth_settings() -> tuple[CombinedTokenVerifier | None, AuthSettings | None]:
    """Get authentication settings from configuration.

    Returns:
        Tuple of (token_verifier, auth_settings) if auth is enabled,
        (None, None) if auth is disabled (dev mode).
    """
    config = get_config()

    if not config.auth_enabled:
        return None, None

    jwks_verifier = JWKSTokenVerifier(
        jwks_url=config.jwks_url,
        issuer=config.auth_server_url,
        jwks_cache_ttl=config.jwks_cache_ttl,
        token_cache_ttl=config.token_cache_ttl
    )

    api_key_verifier = APIKeyVerifier(
        verify_url=config.api_key_verify_url
    )

    # Combined verifier handles both JWT and API keys via Bearer token
    token_verifier = CombinedTokenVerifier(jwks_verifier, api_key_verifier)

    resource_url = config.resource_server_url or f"http://{config.server_host}:{config.server_port}"

    auth_settings = AuthSettings(
        issuer_url=AnyHttpUrl(config.auth_server_url),
        resource_server_url=AnyHttpUrl(resource_url),
        required_scopes=[]
    )

    return token_verifier, auth_settings


def get_dual_auth_validator() -> DualAuthValidator | None:
    """Get dual authentication validator.

    Returns:
        DualAuthValidator if auth is enabled, None otherwise.
    """
    config = get_config()

    if not config.auth_enabled:
        return None

    jwks_verifier = JWKSTokenVerifier(
        jwks_url=config.jwks_url,
        issuer=config.auth_server_url,
        jwks_cache_ttl=config.jwks_cache_ttl,
        token_cache_ttl=config.token_cache_ttl
    )

    api_key_verifier = APIKeyVerifier(
        verify_url=config.api_key_verify_url
    )

    return DualAuthValidator(jwks_verifier, api_key_verifier)
