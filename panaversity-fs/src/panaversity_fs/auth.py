"""JWT Authentication for PanaversityFS MCP Server.

Implements OAuth 2.1 token verification for MCP servers using the
built-in MCP SDK authentication framework.

Authentication is OPTIONAL:
- If PANAVERSITY_JWT_SECRET is set: JWT auth is enabled
- If not set: Server runs in dev mode without auth

Environment Variables:
    PANAVERSITY_JWT_SECRET: Secret key for HS256 JWT verification (required for auth)
    PANAVERSITY_JWT_ALGORITHM: JWT algorithm (default: HS256)
    PANAVERSITY_AUTH_ISSUER: JWT issuer URL for validation (optional)
    PANAVERSITY_AUTH_AUDIENCE: JWT audience for validation (optional)
    PANAVERSITY_REQUIRED_SCOPES: Comma-separated required scopes (default: read,write)
    PANAVERSITY_RESOURCE_SERVER_URL: This server's public URL (for RFC 9728 metadata)
"""

import jwt
from datetime import datetime, timezone
from typing import List

from mcp.server.auth.provider import AccessToken, TokenVerifier
from mcp.server.auth.settings import AuthSettings
from pydantic import AnyHttpUrl

from panaversity_fs.config import get_config


class JWTTokenVerifier(TokenVerifier):
    """Verify JWT tokens from Authorization header.

    Supports HS256 (symmetric) algorithm with shared secret.
    For production with external auth servers, extend to support RS256/JWKS.
    """

    def __init__(
        self,
        secret: str,
        algorithm: str = "HS256",
        issuer: str | None = None,
        audience: str | None = None,
        required_scopes: List[str] | None = None
    ):
        """Initialize JWT verifier.

        Args:
            secret: Secret key for JWT verification
            algorithm: JWT algorithm (default: HS256)
            issuer: Expected issuer claim (optional)
            audience: Expected audience claim (optional)
            required_scopes: Scopes required for access (optional)
        """
        self.secret = secret
        self.algorithm = algorithm
        self.issuer = issuer
        self.audience = audience
        self.required_scopes = required_scopes or []

    async def verify_token(self, token: str) -> AccessToken | None:
        """Verify JWT and return AccessToken if valid.

        Args:
            token: JWT string from Authorization header

        Returns:
            AccessToken if valid, None if invalid
        """
        try:
            # Build verification options
            options = {}
            verify_kwargs = {
                "algorithms": [self.algorithm]
            }

            if self.issuer:
                verify_kwargs["issuer"] = self.issuer
            if self.audience:
                verify_kwargs["audience"] = self.audience

            # Decode and verify JWT
            payload = jwt.decode(
                token,
                self.secret,
                **verify_kwargs
            )

            # Extract client_id from 'sub' claim (required by MCP AccessToken)
            client_id = payload.get("sub", "unknown")

            # Extract scopes from token
            token_scopes = payload.get("scopes", [])
            if isinstance(token_scopes, str):
                token_scopes = [s.strip() for s in token_scopes.split(",")]

            # Check required scopes
            if self.required_scopes:
                missing_scopes = set(self.required_scopes) - set(token_scopes)
                if missing_scopes:
                    return None  # Missing required scopes

            # Extract expiration
            exp = payload.get("exp")
            expires_at = None
            if exp:
                expires_at = int(exp)

            return AccessToken(
                token=token,
                client_id=client_id,
                scopes=token_scopes,
                expires_at=expires_at
            )

        except jwt.ExpiredSignatureError:
            # Token has expired
            return None
        except jwt.InvalidTokenError:
            # Token is invalid (bad signature, malformed, etc.)
            return None
        except Exception:
            # Any other error - reject token
            return None


def get_auth_settings() -> tuple[JWTTokenVerifier | None, AuthSettings | None]:
    """Get authentication settings from configuration.

    Returns:
        Tuple of (token_verifier, auth_settings) if auth is enabled,
        (None, None) if auth is disabled (dev mode).
    """
    config = get_config()

    # Check if auth is enabled
    if not config.jwt_secret:
        return None, None

    # Create token verifier
    token_verifier = JWTTokenVerifier(
        secret=config.jwt_secret,
        algorithm=config.jwt_algorithm,
        issuer=config.auth_issuer,
        audience=config.auth_audience,
        required_scopes=config.required_scopes
    )

    # Create auth settings for RFC 9728 Protected Resource Metadata
    # MCP SDK requires auth settings when token_verifier is provided
    # Use configured values or sensible defaults
    issuer_url = config.auth_issuer or "https://auth.panaversity.com"
    resource_url = config.resource_server_url or f"http://{config.server_host}:{config.server_port}"

    auth_settings = AuthSettings(
        issuer_url=AnyHttpUrl(issuer_url),
        resource_server_url=AnyHttpUrl(resource_url),
        required_scopes=config.required_scopes
    )

    return token_verifier, auth_settings


def create_test_token(
    scopes: List[str] | None = None,
    expires_in_seconds: int = 3600,
    subject: str = "test-agent"
) -> str:
    """Create a test JWT token for development/testing.

    Args:
        scopes: List of scopes to include (default: ["read", "write"])
        expires_in_seconds: Token validity period (default: 1 hour)
        subject: Token subject claim

    Returns:
        Signed JWT token string

    Raises:
        ValueError: If JWT_SECRET is not configured
    """
    config = get_config()

    if not config.jwt_secret:
        raise ValueError("PANAVERSITY_JWT_SECRET must be set to create tokens")

    if scopes is None:
        scopes = ["read", "write"]

    now = datetime.now(timezone.utc)
    payload = {
        "sub": subject,
        "scopes": scopes,
        "iat": int(now.timestamp()),
        "exp": int(now.timestamp()) + expires_in_seconds
    }

    if config.auth_issuer:
        payload["iss"] = config.auth_issuer
    if config.auth_audience:
        payload["aud"] = config.auth_audience

    return jwt.encode(payload, config.jwt_secret, algorithm=config.jwt_algorithm)
