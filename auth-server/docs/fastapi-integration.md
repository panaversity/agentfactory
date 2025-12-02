# FastAPI Backend Integration Guide

This guide explains how to integrate the RoboLearn Auth Server with a FastAPI backend using JWKS-based token verification.

## Overview

The auth server issues RS256-signed JWTs that can be verified using the public keys from the JWKS endpoint. This approach:

- **No shared secrets**: Backend verifies tokens using public keys only
- **Standard OIDC**: Compatible with any OAuth2/OIDC client
- **Scalable**: Each microservice can verify tokens independently

## Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Frontend      │────▶│   Auth Server    │────▶│   FastAPI       │
│   (React/Next)  │     │   (Better Auth)  │     │   Backend       │
└─────────────────┘     └──────────────────┘     └─────────────────┘
                              │                         │
                              │ JWKS (public keys)      │ Verify JWT
                              ▼                         ▼
                        /api/auth/jwks          RS256 verification
```

## Quick Start

### 1. Install Dependencies

```bash
pip install python-jose[cryptography] httpx pydantic
```

### 2. Create Auth Module

```python
# app/auth.py
from typing import Optional
from jose import jwt, JWTError
from jose.backends import RSAKey
import httpx
from pydantic import BaseModel
from fastapi import HTTPException, Security, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import time

# Configuration
AUTH_SERVER_URL = "http://localhost:3001"  # Your auth server URL
JWKS_URL = f"{AUTH_SERVER_URL}/api/auth/jwks"
ISSUER = AUTH_SERVER_URL

# Cache JWKS for performance
_jwks_cache: dict = {}
_jwks_cache_time: float = 0
JWKS_CACHE_TTL = 3600  # 1 hour


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


async def get_public_key(token: str) -> RSAKey:
    """Get the public key for token verification"""
    jwks = await get_jwks()

    # Get the key ID from token header
    unverified_header = jwt.get_unverified_header(token)
    kid = unverified_header.get("kid")

    for key in jwks.get("keys", []):
        if key.get("kid") == kid:
            return key

    raise HTTPException(status_code=401, detail="Unable to find matching key")


security = HTTPBearer()


async def verify_token(
    credentials: HTTPAuthorizationCredentials = Security(security)
) -> TokenPayload:
    """Verify JWT and return token payload"""
    token = credentials.credentials

    try:
        public_key = await get_public_key(token)

        payload = jwt.decode(
            token,
            public_key,
            algorithms=["RS256"],
            issuer=ISSUER,
            options={"verify_aud": False}  # Skip audience check if not set
        )

        return TokenPayload(**payload)

    except JWTError as e:
        raise HTTPException(
            status_code=401,
            detail=f"Invalid token: {str(e)}"
        )


async def get_current_user(
    token: TokenPayload = Depends(verify_token)
) -> TokenPayload:
    """Dependency to get the current authenticated user"""
    return token


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
```

### 3. Use in FastAPI Routes

```python
# app/main.py
from fastapi import FastAPI, Depends
from app.auth import get_current_user, require_role, require_tenant, TokenPayload

app = FastAPI()


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


@app.get("/api/admin/users")
async def list_users(user: TokenPayload = Depends(require_role(["admin"]))):
    """Admin-only endpoint"""
    return {"message": "You are an admin", "admin_id": user.sub}


@app.get("/api/tenant/data")
async def get_tenant_data(user: TokenPayload = Depends(require_tenant)):
    """Requires user to belong to an organization"""
    return {
        "tenant_id": user.tenant_id,
        "org_role": user.org_role,
        "organizations": user.organization_ids,
    }


@app.get("/api/personalized")
async def get_personalized_content(user: TokenPayload = Depends(get_current_user)):
    """Content personalized by user profile"""
    # Use software_background and hardware_tier for personalization
    tier = user.hardware_tier or "tier1"
    background = user.software_background or "beginner"

    return {
        "content_level": background,
        "hardware_requirements": tier,
        "message": f"Content optimized for {background} with {tier} hardware"
    }
```

## Token Claims Reference

The auth server includes these claims in the ID token and userinfo:

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

## Multi-Tenant Pattern

For multi-tenant applications, use `tenant_id` to scope data:

```python
from sqlalchemy.orm import Session
from app.auth import TokenPayload, require_tenant


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
        tenant_id=user.tenant_id,
        created_by=user.sub
    )
    db.add(resource)
    db.commit()
    return resource
```

## MCP Server Integration

For MCP (Model Context Protocol) servers, verify tokens in the transport layer:

```python
# mcp_server.py
from mcp.server import Server
from mcp.server.stdio import stdio_server
from app.auth import verify_token_sync  # Sync version for MCP


class AuthenticatedMCPServer:
    def __init__(self):
        self.server = Server("robolearn-mcp")
        self.current_user = None

    async def authenticate(self, token: str):
        """Authenticate user from token"""
        self.current_user = await verify_token(token)
        return self.current_user

    @self.server.list_tools()
    async def list_tools(self):
        """List available tools based on user permissions"""
        tools = []

        # Base tools for all authenticated users
        tools.append({"name": "get_profile", "description": "Get user profile"})

        # Admin-only tools
        if self.current_user and self.current_user.role == "admin":
            tools.append({"name": "admin_stats", "description": "Get admin statistics"})

        # Tenant-specific tools
        if self.current_user and self.current_user.tenant_id:
            tools.append({"name": "tenant_data", "description": "Get tenant data"})

        return tools
```

## Testing

### Get a Token for Testing

```bash
# Run the test script to get a valid token
node tests/test-pkce-oauth.js
# Copy the access_token from output

# Or use curl to sign in and get a token
# (This requires session cookie handling)
```

### Test Your FastAPI Endpoint

```bash
# Replace ACCESS_TOKEN with actual token
curl http://localhost:8000/api/me \
  -H "Authorization: Bearer ACCESS_TOKEN"
```

## Production Considerations

1. **JWKS Caching**: The example caches JWKS for 1 hour. Adjust based on key rotation frequency.

2. **Audience Validation**: Set `AUTH_SERVER_AUDIENCE` if you configure specific audiences.

3. **Error Handling**: Add proper logging and error tracking in production.

4. **Rate Limiting**: Add rate limiting to prevent token verification abuse.

5. **HTTPS**: Always use HTTPS in production for both auth server and API.

## Environment Variables

```bash
# FastAPI Backend
AUTH_SERVER_URL=https://auth.yourdomain.com
JWKS_CACHE_TTL=3600
```

## Troubleshooting

### "Unable to find matching key"
- JWKS cache may be stale. Clear cache or reduce TTL.
- Token may be from different issuer. Check `AUTH_SERVER_URL`.

### "Invalid token"
- Token may be expired. Check `exp` claim.
- Token may be malformed. Verify token format.

### "Role not authorized"
- User's role doesn't match required roles.
- Check user's role in auth server admin panel.
