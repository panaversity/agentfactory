# Backend Service Integration Guide

This guide explains how to integrate FastAPI, MCP, and other backend services with the RoboLearn Auth Server using OAuth 2.0/OIDC.

## Table of Contents

1. [Registering as Confidential Client](#1-registering-as-confidential-client)
2. [OAuth Authorization Code Flow](#2-oauth-authorization-code-flow)
3. [Verifying Tokens (JWKS)](#3-verifying-tokens-jwks)
4. [Enforcing Authorization](#4-enforcing-authorization)
5. [Client Credentials Grant (Coming Soon)](#5-client-credentials-grant-coming-soon)
6. [Production Considerations](#6-production-considerations)

---

## 1. Registering as Confidential Client

Backend services should register as **confidential clients** (services that can securely store a client secret).

### Registration Process

**Endpoint**: `POST /api/admin/clients/register`

**Authentication**: Requires admin session (admin user must be logged in)

**Request**:
```bash
curl -X POST http://localhost:3001/api/admin/clients/register \
  -H "Content-Type: application/json" \
  -H "Cookie: robolearn_session=<admin-session-cookie>" \
  -d '{
    "name": "My Backend Service",
    "redirectUrls": ["https://api.example.com/auth/callback"],
    "scope": "openid profile email",
    "clientType": "confidential",
    "skipConsent": false
  }'
```

**Response**:
```json
{
  "success": true,
  "client_id": "generated-client-id",
  "client_secret": "generated-client-secret",
  "client_type": "confidential",
  "name": "My Backend Service",
  "redirect_uris": ["https://api.example.com/auth/callback"]
}
```

**⚠️ Important Security Notes**:
- Store `client_secret` securely (environment variable, secrets manager, never in source code)
- The client secret is only shown once during registration
- Admin authentication is required to prevent unauthorized client registration

### Environment Variables

After registration, configure your backend service:

```bash
# OAuth Configuration
OAUTH_CLIENT_ID=generated-client-id
OAUTH_CLIENT_SECRET=generated-client-secret
OAUTH_AUTH_URL=http://localhost:3001
OAUTH_REDIRECT_URI=https://api.example.com/auth/callback
OAUTH_ISSUER=http://localhost:3001
```

### Client Types

| Type | Client Secret | PKCE | Use Case |
|------|---------------|------|----------|
| **Public** | ❌ No secret | ✅ Required | SPAs, mobile apps, CLI tools |
| **Confidential** | ✅ Has secret | ⚠️ Optional | Backend services, server-side apps |

---

## 2. OAuth Authorization Code Flow

### Overview

The Authorization Code Flow is the standard OAuth 2.0 flow for applications that can securely store credentials.

```
┌─────────────┐     ┌──────────────┐     ┌─────────────────┐
│   User      │────▶│ Auth Server  │────▶│  Your Backend   │
│  (Browser)  │     │  (Better Auth)│     │   (FastAPI)     │
└─────────────┘     └──────────────┘     └─────────────────┘
      │                    │                      │
      │  1. Redirect       │                      │
      │  /authorize        │                      │
      │───────────────────▶│                      │
      │                    │ 2. User Login        │
      │                    │    (if needed)       │
      │                    │                      │
      │  3. Redirect back  │                      │
      │     with code      │                      │
      │◀───────────────────│                      │
      │                    │                      │
      │  4. Send code to   │                      │
      │     backend        │                      │
      │────────────────────────────────────────▶│
      │                    │                      │
      │                    │  5. Exchange code    │
      │                    │     + client_secret  │
      │                    │◀─────────────────────│
      │                    │                      │
      │                    │  6. Return tokens    │
      │                    │─────────────────────▶│
      │                    │                      │
```

### Step 1: Redirect User to Authorization Endpoint

When a user needs to authenticate, redirect them to:

```
GET /api/auth/oauth2/authorize?
  client_id={CLIENT_ID}&
  redirect_uri={REDIRECT_URI}&
  response_type=code&
  scope=openid%20profile%20email&
  state={RANDOM_STATE}&
  code_challenge={CODE_CHALLENGE}&
  code_challenge_method=S256
```

**Required Parameters**:
- `client_id`: Your registered client ID
- `redirect_uri`: Must match registered redirect URI exactly
- `response_type`: Always `code` for authorization code flow
- `scope`: Requested scopes (e.g., `openid profile email`)
- `state`: Random string for CSRF protection

**PKCE Parameters** (optional for confidential clients, required for public):
- `code_challenge`: SHA256 hash of code_verifier
- `code_challenge_method`: `S256` (SHA-256)

**Python/FastAPI Example**:
```python
import secrets
import hashlib
import base64
from fastapi import FastAPI, Request
from fastapi.responses import RedirectResponse

app = FastAPI()

OAUTH_AUTH_URL = "http://localhost:3001"
CLIENT_ID = "your-client-id"
REDIRECT_URI = "https://api.example.com/auth/callback"

@app.get("/login")
async def login(request: Request):
    """Initiate OAuth login flow"""

    # Generate state for CSRF protection
    state = secrets.token_urlsafe(32)
    request.session['oauth_state'] = state

    # Optional: Generate PKCE values for enhanced security
    code_verifier = base64.urlsafe_b64encode(
        secrets.token_bytes(32)
    ).decode('utf-8').rstrip('=')

    code_challenge = base64.urlsafe_b64encode(
        hashlib.sha256(code_verifier.encode()).digest()
    ).decode('utf-8').rstrip('=')

    # Store verifier for later
    request.session['code_verifier'] = code_verifier

    # Build authorization URL
    params = {
        "client_id": CLIENT_ID,
        "redirect_uri": REDIRECT_URI,
        "response_type": "code",
        "scope": "openid profile email",
        "state": state,
        "code_challenge": code_challenge,
        "code_challenge_method": "S256"
    }

    auth_url = f"{OAUTH_AUTH_URL}/api/auth/oauth2/authorize?" + "&".join(
        f"{k}={v}" for k, v in params.items()
    )

    return RedirectResponse(url=auth_url)
```

### Step 2: Handle Callback

After user authenticates, they're redirected back with an authorization code:

```
GET /auth/callback?code={AUTHORIZATION_CODE}&state={STATE}
```

**FastAPI Example**:
```python
from fastapi import HTTPException

@app.get("/auth/callback")
async def callback(request: Request, code: str, state: str):
    """Handle OAuth callback"""

    # 1. Verify state matches (CSRF protection)
    stored_state = request.session.get('oauth_state')
    if state != stored_state:
        raise HTTPException(status_code=400, detail="Invalid state - CSRF detected")

    # 2. Exchange code for tokens
    code_verifier = request.session.get('code_verifier')

    token_response = await exchange_code_for_tokens(
        code=code,
        code_verifier=code_verifier  # Optional for confidential clients
    )

    # 3. Store tokens securely
    request.session['access_token'] = token_response['access_token']
    request.session['refresh_token'] = token_response['refresh_token']
    request.session['id_token'] = token_response['id_token']

    # 4. Clean up
    request.session.pop('oauth_state', None)
    request.session.pop('code_verifier', None)

    # 5. Redirect to app
    return RedirectResponse(url="/dashboard")
```

### Step 3: Exchange Code for Tokens

**For Confidential Clients** (with client secret):

Use **HTTP Basic Authentication** with client credentials:

```python
import httpx
import base64

async def exchange_code_for_tokens(
    code: str,
    code_verifier: str = None
) -> dict:
    """
    Exchange authorization code for tokens.

    For confidential clients, uses HTTP Basic Auth.
    For public clients, includes PKCE code_verifier.
    """
    token_url = f"{OAUTH_AUTH_URL}/api/auth/oauth2/token"

    # Build request data
    data = {
        "grant_type": "authorization_code",
        "code": code,
        "redirect_uri": REDIRECT_URI,
        "client_id": CLIENT_ID,
    }

    # Add PKCE verifier if using PKCE
    if code_verifier:
        data["code_verifier"] = code_verifier

    # For confidential clients: HTTP Basic Auth
    # Format: Authorization: Basic base64(client_id:client_secret)
    auth = None
    if CLIENT_SECRET:  # Confidential client
        auth = (CLIENT_ID, CLIENT_SECRET)  # httpx handles Basic Auth encoding

    async with httpx.AsyncClient() as client:
        response = await client.post(
            token_url,
            data=data,
            auth=auth  # HTTP Basic Auth for confidential clients
        )
        response.raise_for_status()
        return response.json()
```

**Token Response**:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 21600,
  "refresh_token": "eyJhbGciOiJSUzI1NiIs...",
  "id_token": "eyJhbGciOiJSUzI1NiIs...",
  "scope": "openid profile email"
}
```

### Alternative: Direct Token Exchange (Without Browser)

For backend-to-backend flows where you already have user credentials:

```python
# NOT RECOMMENDED: Only for trusted first-party backends
# Use Authorization Code Flow with user consent instead

async def direct_token_exchange(username: str, password: str):
    """
    Direct password grant (if enabled).
    WARNING: Only use for highly trusted first-party apps.
    """
    token_url = f"{OAUTH_AUTH_URL}/api/auth/oauth2/token"

    response = await httpx.AsyncClient().post(
        token_url,
        data={
            "grant_type": "password",  # Requires server configuration
            "username": username,
            "password": password,
            "client_id": CLIENT_ID,
            "client_secret": CLIENT_SECRET,
            "scope": "openid profile email"
        },
        auth=(CLIENT_ID, CLIENT_SECRET)
    )
    return response.json()
```

---

## 3. Verifying Tokens (JWKS)

### Overview

The auth server signs tokens with **RS256** (RSA with SHA-256). Your backend can verify tokens using the public keys from the JWKS endpoint.

**Benefits**:
- ✅ No shared secrets needed
- ✅ Verify tokens without database calls
- ✅ Standard OIDC compliance
- ✅ Scales horizontally

### Step 1: Fetch JWKS

**JWKS Endpoint**: `/api/auth/jwks`

```python
import httpx
import time
from jose import jwt, jwk
from jose.backends import RSAKey

# Configuration
JWKS_URL = f"{OAUTH_AUTH_URL}/api/auth/jwks"
ISSUER = OAUTH_AUTH_URL

# Cache JWKS for performance
_jwks_cache: dict = {}
_jwks_cache_time: float = 0
JWKS_CACHE_TTL = 3600  # 1 hour

async def get_jwks() -> dict:
    """Fetch and cache JWKS from auth server"""
    global _jwks_cache, _jwks_cache_time

    current_time = time.time()
    if _jwks_cache and (current_time - _jwks_cache_time) < JWKS_CACHE_TTL:
        return _jwks_cache

    async with httpx.AsyncClient() as client:
        response = await client.get(JWKS_URL)
        response.raise_for_status()
        _jwks_cache = response.json()
        _jwks_cache_time = current_time
        return _jwks_cache
```

**JWKS Response**:
```json
{
  "keys": [
    {
      "kty": "RSA",
      "kid": "key-id-12345",
      "alg": "RS256",
      "use": "sig",
      "n": "base64url-encoded-modulus",
      "e": "AQAB"
    }
  ]
}
```

### Step 2: Verify Token

```python
from jose import jwt, JWTError
from pydantic import BaseModel
from typing import Optional

class TokenPayload(BaseModel):
    """Token claims from the auth server"""
    sub: str  # User ID
    email: Optional[str] = None
    name: Optional[str] = None
    role: Optional[str] = "user"
    tenant_id: Optional[str] = None
    organization_ids: Optional[list[str]] = []
    org_role: Optional[str] = None
    software_background: Optional[str] = None
    hardware_tier: Optional[str] = None
    exp: int
    iat: int
    iss: str
    aud: Optional[str] = None

async def verify_token(token: str) -> TokenPayload:
    """
    Verify JWT token signature and claims.

    Steps:
    1. Get key ID (kid) from token header
    2. Fetch matching public key from JWKS
    3. Verify signature
    4. Validate claims (exp, iss, aud)
    """
    try:
        # Get token header to find key ID
        header = jwt.get_unverified_header(token)
        kid = header.get('kid')

        # Get JWKS
        jwks = await get_jwks()

        # Find matching key
        public_key = None
        for key in jwks.get('keys', []):
            if key.get('kid') == kid:
                public_key = key
                break

        if not public_key:
            # Key not found - refresh cache and retry once
            _jwks_cache.clear()
            jwks = await get_jwks()
            for key in jwks.get('keys', []):
                if key.get('kid') == kid:
                    public_key = key
                    break

            if not public_key:
                raise JWTError("No matching key found")

        # Verify token
        payload = jwt.decode(
            token,
            public_key,
            algorithms=['RS256'],
            issuer=ISSUER,
            options={"verify_aud": False}  # Set to True if audience is configured
        )

        return TokenPayload(**payload)

    except JWTError as e:
        raise HTTPException(
            status_code=401,
            detail=f"Token verification failed: {str(e)}"
        )
```

### Step 3: Create FastAPI Dependency

```python
from fastapi import Depends, HTTPException, Security
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

security = HTTPBearer()

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Security(security)
) -> TokenPayload:
    """
    Dependency to get the current authenticated user.

    Usage:
        @app.get("/api/me")
        async def get_me(user: TokenPayload = Depends(get_current_user)):
            return {"user_id": user.sub}
    """
    token = credentials.credentials
    return await verify_token(token)
```

### Step 4: Use in Routes

```python
@app.get("/api/me")
async def get_me(user: TokenPayload = Depends(get_current_user)):
    """Get current user info - requires authentication"""
    return {
        "user_id": user.sub,
        "email": user.email,
        "name": user.name,
        "role": user.role,
        "tenant_id": user.tenant_id,
        "software_background": user.software_background,
        "hardware_tier": user.hardware_tier,
    }

@app.get("/api/protected")
async def protected_endpoint(user: TokenPayload = Depends(get_current_user)):
    """Any authenticated user can access"""
    return {"message": f"Hello {user.name}"}
```

### Token Claims Reference

| Claim | Type | Description |
|-------|------|-------------|
| `sub` | string | User ID (unique identifier) |
| `email` | string | User's email address |
| `name` | string | User's display name |
| `role` | string | User role: "user" or "admin" |
| `tenant_id` | string\|null | Primary organization ID |
| `organization_ids` | string[] | All organizations user belongs to |
| `org_role` | string\|null | Role in primary organization |
| `software_background` | string\|null | "beginner", "intermediate", "advanced" |
| `hardware_tier` | string\|null | "tier1", "tier2", "tier3", "tier4" |
| `email_verified` | boolean | Whether email is verified |
| `exp` | int | Token expiration timestamp |
| `iat` | int | Token issued at timestamp |
| `iss` | string | Token issuer (auth server URL) |

---

## 4. Enforcing Authorization

### Role-Based Access Control (RBAC)

**System Roles**:
- `user`: Default role for all users
- `admin`: Administrative access

```python
async def require_role(required_roles: list[str]):
    """Factory for role-based access control"""
    async def role_checker(
        user: TokenPayload = Depends(get_current_user)
    ) -> TokenPayload:
        if user.role not in required_roles:
            raise HTTPException(
                status_code=403,
                detail=f"Role '{user.role}' not authorized. Required: {required_roles}"
            )
        return user
    return role_checker

# Usage
@app.get("/api/admin/users")
async def list_users(user: TokenPayload = Depends(require_role(["admin"]))):
    """Admin-only endpoint"""
    return {"message": "You are an admin", "admin_id": user.sub}
```

### Tenant-Based Access Control

Multi-tenant data isolation:

```python
async def require_tenant(
    user: TokenPayload = Depends(get_current_user)
) -> TokenPayload:
    """Require user to belong to a tenant/organization"""
    if not user.tenant_id:
        raise HTTPException(
            status_code=403,
            detail="User must belong to an organization"
        )
    return user

# Usage
@app.get("/api/tenant/resources")
async def get_tenant_resources(
    user: TokenPayload = Depends(require_tenant),
    db: Session = Depends(get_db)
):
    """Get resources scoped to user's tenant"""
    resources = db.query(Resource).filter(
        Resource.tenant_id == user.tenant_id
    ).all()
    return resources

@app.post("/api/tenant/resources")
async def create_resource(
    data: ResourceCreate,
    user: TokenPayload = Depends(require_tenant),
    db: Session = Depends(get_db)
):
    """Create resource scoped to user's tenant"""
    resource = Resource(
        **data.dict(),
        tenant_id=user.tenant_id,  # Automatic tenant isolation
        created_by=user.sub
    )
    db.add(resource)
    db.commit()
    return resource
```

### Organization Role-Based Access

```python
async def require_org_admin(
    user: TokenPayload = Depends(get_current_user)
) -> TokenPayload:
    """Require organization admin or owner role"""
    if user.org_role not in ["owner", "admin"]:
        raise HTTPException(
            status_code=403,
            detail="Organization admin access required"
        )
    return user

@app.post("/api/org/settings")
async def update_org_settings(
    settings: OrgSettings,
    user: TokenPayload = Depends(require_org_admin)
):
    """Only org admins/owners can update settings"""
    # Update organization settings...
    return {"success": True}
```

### Scope-Based Access Control

```python
async def require_scope(required_scope: str):
    """Decorator to require specific OAuth scope"""
    async def scope_checker(
        user: TokenPayload = Depends(get_current_user)
    ) -> TokenPayload:
        # Note: Scopes are in the token but not in our TokenPayload model
        # You may need to add a 'scope' field to TokenPayload
        scopes = getattr(user, 'scope', '').split(' ')

        if required_scope not in scopes:
            raise HTTPException(
                status_code=403,
                detail=f"Requires scope: {required_scope}"
            )
        return user
    return scope_checker

@app.get("/users/profile")
async def get_profile(user: TokenPayload = Depends(require_scope("profile"))):
    """Requires 'profile' scope"""
    return {"profile": {...}}
```

### Hardware Tier-Based Access Control

Control access based on user's hardware capabilities:

```python
TIER_LEVELS = {"tier1": 1, "tier2": 2, "tier3": 3, "tier4": 4}

async def require_hardware_tier(min_tier: str):
    """Require minimum hardware tier"""
    async def tier_checker(
        user: TokenPayload = Depends(get_current_user)
    ) -> TokenPayload:
        user_tier = TIER_LEVELS.get(user.hardware_tier or "tier1", 1)
        required_tier = TIER_LEVELS.get(min_tier, 1)

        if user_tier < required_tier:
            raise HTTPException(
                status_code=403,
                detail=f"This content requires {min_tier}. You have {user.hardware_tier or 'tier1'}."
            )
        return user
    return tier_checker

@app.get("/api/content/{content_id}")
async def get_content(
    content_id: str,
    user: TokenPayload = Depends(require_hardware_tier("tier2"))
):
    """Content requiring at least tier2 (RTX GPU)"""
    return {"content": "Advanced GPU-accelerated content"}
```

### Combining Multiple Checks

```python
from functools import wraps

async def require_admin_with_tenant(
    user: TokenPayload = Depends(get_current_user)
) -> TokenPayload:
    """Require both admin role AND tenant membership"""
    if user.role != "admin":
        raise HTTPException(status_code=403, detail="Admin role required")
    if not user.tenant_id:
        raise HTTPException(status_code=403, detail="Organization membership required")
    return user

@app.get("/api/admin/tenant/analytics")
async def get_tenant_analytics(
    user: TokenPayload = Depends(require_admin_with_tenant)
):
    """Admin users viewing their organization's analytics"""
    return {"analytics": "data for tenant", "tenant_id": user.tenant_id}
```

---

## 5. Client Credentials Grant (Coming Soon)

For **headless/backend-to-backend** authentication (machine-to-machine), we plan to support the **Client Credentials** grant type.

**Status**: 🚧 Not yet implemented. Track issue [#XX] for updates.

**Planned Usage**:
```python
# Future implementation
async def get_service_token():
    """
    Machine-to-machine authentication.
    No user context - service acts on its own behalf.
    """
    token_url = f"{OAUTH_AUTH_URL}/api/auth/oauth2/token"

    response = await httpx.AsyncClient().post(
        token_url,
        data={
            "grant_type": "client_credentials",
            "scope": "api:read api:write"
        },
        auth=(CLIENT_ID, CLIENT_SECRET)  # HTTP Basic Auth
    )

    return response.json()
```

**Use Cases**:
- Background jobs accessing APIs
- Service-to-service communication
- Cron jobs that don't act on behalf of a user
- Internal microservices

---

## 6. Production Considerations

### Security Best Practices

#### 1. Secure Token Storage

**DO**:
- Store access tokens in memory only (short-lived)
- Use httpOnly cookies for refresh tokens
- Clear tokens on logout

**DON'T**:
- Store tokens in localStorage (XSS vulnerable)
- Log tokens in application logs
- Send tokens in URL parameters

#### 2. JWKS Caching

```python
# Recommended cache TTL
JWKS_CACHE_TTL = 3600  # 1 hour in production

# Refresh on kid mismatch
if kid not in cached_keys:
    # Force refresh
    _jwks_cache.clear()
    jwks = await get_jwks()
```

#### 3. HTTPS Only

```python
# Enforce HTTPS in production
import os

if os.getenv("ENV") == "production":
    if not OAUTH_AUTH_URL.startswith("https://"):
        raise ValueError("AUTH_URL must use HTTPS in production")
```

#### 4. Rate Limiting

Add rate limiting to prevent token verification abuse:

```python
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)

@app.get("/api/protected")
@limiter.limit("100/minute")
async def protected_endpoint(
    request: Request,
    user: TokenPayload = Depends(get_current_user)
):
    return {"data": "protected"}
```

#### 5. Token Refresh

Implement token refresh before expiration:

```python
async def refresh_access_token(refresh_token: str) -> dict:
    """Refresh access token using refresh token"""
    token_url = f"{OAUTH_AUTH_URL}/api/auth/oauth2/token"

    response = await httpx.AsyncClient().post(
        token_url,
        data={
            "grant_type": "refresh_token",
            "refresh_token": refresh_token,
            "client_id": CLIENT_ID,
        },
        auth=(CLIENT_ID, CLIENT_SECRET)
    )

    return response.json()
```

### Error Handling

```python
from fastapi import Request
from fastapi.responses import JSONResponse

@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    """Standardized error responses"""
    if exc.status_code == 401:
        return JSONResponse(
            status_code=401,
            content={
                "error": "unauthorized",
                "message": "Authentication required",
                "detail": str(exc.detail)
            }
        )

    if exc.status_code == 403:
        return JSONResponse(
            status_code=403,
            content={
                "error": "forbidden",
                "message": "Insufficient permissions",
                "detail": str(exc.detail)
            }
        )

    return JSONResponse(
        status_code=exc.status_code,
        content={"error": "error", "detail": str(exc.detail)}
    )
```

### Logging & Monitoring

```python
import logging

logger = logging.getLogger(__name__)

async def get_current_user_with_logging(
    credentials: HTTPAuthorizationCredentials = Security(security)
) -> TokenPayload:
    """Enhanced version with audit logging"""
    try:
        user = await verify_token(credentials.credentials)
        logger.info(f"User authenticated: {user.sub} ({user.email})")
        return user
    except HTTPException as e:
        logger.warning(f"Authentication failed: {e.detail}")
        raise
```

### Environment Configuration

```python
# config.py
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    # OAuth Configuration
    oauth_client_id: str
    oauth_client_secret: str
    oauth_auth_url: str
    oauth_redirect_uri: str
    oauth_issuer: str

    # JWKS Configuration
    jwks_cache_ttl: int = 3600

    # Security
    enforce_https: bool = True

    class Config:
        env_file = ".env"

settings = Settings()
```

### Health Checks

```python
@app.get("/health")
async def health_check():
    """Health check endpoint"""
    try:
        # Verify JWKS is accessible
        jwks = await get_jwks()
        jwks_healthy = bool(jwks.get("keys"))
    except:
        jwks_healthy = False

    return {
        "status": "healthy" if jwks_healthy else "degraded",
        "checks": {
            "jwks": "ok" if jwks_healthy else "error"
        }
    }
```

---

## Additional Resources

- [FastAPI Integration Guide](./fastapi-integration.md) - Detailed FastAPI examples
- [JWT & JWKS Documentation](./jwt-jwks.md) - Token verification details
- [RBAC & Scopes](./rbac-and-scopes.md) - Authorization patterns
- [PKCE Flow](./pkce-flow.md) - Public client authentication
- [Multi-Tenancy](./multi-tenancy.md) - Tenant isolation patterns
- [Troubleshooting](./troubleshooting.md) - Common issues and solutions

---

## Quick Reference

### Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/admin/clients/register` | POST | Register confidential client (admin auth) |
| `/api/auth/oauth2/authorize` | GET | Start OAuth flow (browser redirect) |
| `/api/auth/oauth2/token` | POST | Exchange code for tokens |
| `/api/auth/oauth2/userinfo` | GET | Get user info (requires access token) |
| `/api/auth/jwks` | GET | Public keys for token verification |
| `/.well-known/openid-configuration` | GET | OIDC discovery endpoint |

### Environment Variables

```bash
# Required
OAUTH_CLIENT_ID=your-client-id
OAUTH_CLIENT_SECRET=your-client-secret
OAUTH_AUTH_URL=https://auth.yourdomain.com
OAUTH_REDIRECT_URI=https://api.yourdomain.com/auth/callback

# Optional
JWKS_CACHE_TTL=3600
```

### Token Lifetimes

| Token Type | Lifetime | Renewable |
|------------|----------|-----------|
| Authorization Code | 10 minutes | No (one-time use) |
| Access Token | 6 hours | Via refresh token |
| Refresh Token | 7 days | No (request new access token) |
| ID Token | 6 hours | N/A (informational only) |

---

## Support

For issues or questions:
- Check [Troubleshooting Guide](./troubleshooting.md)
- Review [Better Auth Documentation](https://www.better-auth.com/docs)
- File an issue on GitHub
