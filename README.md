# Auth Server

OAuth 2.1 / OIDC authentication server using Better Auth with PKCE, JWKS, and multi-tenancy support.

## Documentation

### Getting Started
- [Integration Guide](docs/integration-guide.md) - **Complete guide for backend services (FastAPI, MCP, etc.)**
- [Environment Variables](docs/environment-variables.md) - Complete reference for all config options

### Authentication & Authorization
- [PKCE OAuth Flow](docs/pkce-flow.md) - Public client authentication
- [JWT & JWKS](docs/jwt-jwks.md) - Token signing and verification
- [RBAC & Scopes](docs/rbac-and-scopes.md) - Roles and permissions
- [FastAPI Integration](docs/fastapi-integration.md) - Backend integration examples

### Additional Resources
- [CI/CD Pipeline](docs/ci-cd.md) - Test suite documentation and troubleshooting (51 automated tests)
- [Redis Setup](docs/redis-setup.md) - Distributed rate limiting for multi-instance deployments
- [Flow Diagrams](docs/flow-diagrams.md) - Visual authentication flows
- [Troubleshooting](docs/troubleshooting.md) - Common issues and solutions

## Setup

```bash
cd auth-server
npm install
cp .env.example .env.local
# Edit .env.local with your values
npm run db:push
# Seed the trusted public client (see below)
npm run dev  # http://localhost:3001
```

### Seed OAuth Clients

After running `db:push`, you need to register OAuth clients for applications to authenticate through the SSO server.

**🚀 Unified Setup (Recommended)**

The auth server uses a single setup script that imports from `src/lib/trusted-clients.ts`:

```bash
# Development setup (all 3 trusted clients + test organization)
pnpm run seed:setup

# Production setup (only Panaversity SSO + AI Native, skips RoboLearn)
pnpm run seed:prod
```

**What gets seeded:**

| Mode | Clients | Organization | Use Case |
|------|---------|--------------|----------|
| **Development** (`seed:setup`) | • RoboLearn<br>• Panaversity SSO<br>• AI Native | ✅ Test org | Local development & testing |
| **Production** (`seed:prod`) | • Panaversity SSO<br>• AI Native | ❌ None | Production deployment |

**Features:**
- ✅ Single source of truth (`src/lib/trusted-clients.ts`)
- ✅ Automatic localhost filtering in production (via `getRedirectUrls()`)
- ✅ Idempotent (safe to run multiple times)
- ✅ Optional multi-tenancy setup (organizations)

**Option 2: Manual SQL (Advanced)**

Execute this SQL in your database (via psql, pgAdmin, or your database tool):

```sql
INSERT INTO oauth_application (
  id,
  client_id,
  client_secret,
  name,
  redirect_urls,
  type,
  disabled,
  metadata,
  created_at,
  updated_at
) VALUES (
  'robolearn-public-client-id',
  'robolearn-public-client',
  NULL, -- No secret for public client (PKCE only)
  'RoboLearn Public Client',
  'http://localhost:3000/auth/callback,http://localhost:3000/robolearn/auth/callback,https://mjunaidca.github.io/robolearn/auth/callback', -- Comma-separated: dev (both variants) + prod URLs
  'public',
  false,
  '{"token_endpoint_auth_method":"none","grant_types":["authorization_code","refresh_token"]}',
  NOW(),
  NOW()
)
ON CONFLICT (client_id) DO UPDATE SET
  name = EXCLUDED.name,
  redirect_urls = EXCLUDED.redirect_urls,
  type = EXCLUDED.type,
  disabled = EXCLUDED.disabled,
  metadata = EXCLUDED.metadata,
  updated_at = NOW();
```


## Environment Variables

```env
# Required
DATABASE_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
BETTER_AUTH_SECRET=your-32-char-secret  # openssl rand -base64 32
BETTER_AUTH_URL=http://localhost:3001
# Comma-separated list of allowed origins for CORS
# Production: Include your production domain(s)
ALLOWED_ORIGINS=http://localhost:3000,https://mjunaidca.github.io
NEXT_PUBLIC_BETTER_AUTH_URL=http://localhost:3001

# Optional: Email verification
# Option 1: Resend (free tier: 100/day)
# RESEND_API_KEY=re_xxxxxxxxx
# RESEND_FROM_EMAIL=onboarding@resend.dev

# Option 2: SMTP (Gmail, custom SMTP, etc.)
# SMTP_HOST=smtp.gmail.com
# SMTP_PORT=587
# SMTP_USER=your@gmail.com
# SMTP_PASS=app-password
# SMTP_FROM=your@gmail.com
# EMAIL_FROM=your@gmail.com
```

## Features

- OAuth 2.1 with PKCE (no client secrets in browser)
- JWKS (JSON Web Key Set) for client-side token verification (RS256)
- Secure client registration (admin-only)
- Multi-tenancy with organizations (tenant_id in tokens)
- Role-based access control (admin/user + organization roles)
- Custom claims: software_background, hardware_tier, tenant_id
- Optional email verification via Resend or SMTP
- 7-day sessions with auto-refresh

## Testing

The test suite is organized into 3 tiers for fast feedback and comprehensive coverage.

### Quick Start

```bash
# 1. Start the auth server
pnpm dev  # http://localhost:3001

# 2. Seed OAuth clients and default organization
pnpm run seed:setup

# 3. Run tests (in another terminal)
pnpm test-api   # Fast API tests (~60s)
pnpm test-e2e   # Playwright visual tests (~30s)
pnpm test-all   # Everything (~90s)
```

### Test Suites

#### 🚀 **Suite 1: `test-api`** - Fast API Tests (7 tests, ~60s)

**Requirements**: Auth server running

**Tests**:
- OAuth 2.1 flows (PKCE + Confidential client)
- JWT tenant claims
- Edge case handling
- Default organization auto-join
- OAuth 2.1/OIDC compliance validation

```bash
pnpm test-api
```

**What it covers**:
- ✅ OIDC Discovery
- ✅ PKCE Flow (public client)
- ✅ Confidential Client Flow
- ✅ Authorization & Token Exchange
- ✅ Userinfo endpoint
- ✅ Multi-tenancy (tenant_id, organization_ids)
- ✅ Security edge cases
- ✅ Default org auto-join

---

#### 🎭 **Suite 2: `test-e2e`** - Playwright Visual Tests (3 tests, ~30s)

**Requirements**: Auth server + Book Interface (localhost:3000) + Playwright

**Tests**:
- Complete SSO flow (sign-up → OAuth → callback)
- PKCE flow with browser automation

```bash
pnpm test-e2e
```

**What it covers**:
- ✅ Book Interface integration
- ✅ Visual sign-up flow
- ✅ OAuth authorization UI
- ✅ Callback handling
- ✅ Profile access
- ✅ Screenshots for debugging

**Note**: Playwright tests gracefully fail if Book Interface isn't running.

---

#### ✅ **Suite 3: `test-all`** - Complete Test Suite (10 tests, ~90s)

**Requirements**: Everything

Runs both API and E2E tests sequentially.

```bash
pnpm test-all
```

---

### Special Tests

#### Admin Client Management (`test-client-admin`)

**Tests OAuth client CRUD operations**. Now with auto-login!

```bash
pnpm test-client-admin
```

**Auto-login**: No manual cookie required - signs in automatically using `admin@robolearn.io` credentials.

**Override credentials** (optional):
```bash
ADMIN_EMAIL=custom@example.com ADMIN_PASSWORD=CustomPass pnpm test-client-admin
```

#### Playwright Test Spec (`test-playwright-spec`)

**Runs e2e-auth-test.spec.ts using Playwright Test framework**.

```bash
pnpm test-playwright-spec
```

#### Manual PKCE Generator (`test-manual-pkce`)

**Utility to generate PKCE pairs for manual testing**.

```bash
pnpm test-manual-pkce
```

---

### Test Files

**Kept (11 tests)**:
- ✅ `test-oauth-flows.js` - OAuth PKCE + Confidential
- ✅ `test-tenant-claims.js` - JWT tenant_id claims
- ✅ `test-edge-cases.js` - Security edge cases
- ✅ `test-tenant-edge-cases.js` - Tenant edge cases
- ✅ `test-confidential-client.js` - Confidential flow
- ✅ `test-default-organization.js` - Default org auto-join
- ✅ `oauth-validation.test.ts` - OAuth 2.1 compliance
- ✅ `test-complete-sso.js` - Full SSO flow (Playwright)
- ✅ `test-pkce-playwright.mjs` - PKCE browser test
- ✅ `test-client-edit.js` - Admin API (with auto-login)
- ✅ `e2e-auth-test.spec.ts` - Playwright Test spec

**Removed (5 duplicates)**:
- 🗑️ `test-oauth-api.mjs` - Covered by test-oauth-flows.js
- 🗑️ `test-pkce-oauth.js` - Covered by test-oauth-flows.js
- 🗑️ `test-full-oauth.js` - Duplicate of test-complete-sso.js
- 🗑️ `test-visual-flow.js` - Covered by test-complete-sso.js
- 🗑️ `test-oauth-flow.js` - Covered by test-complete-sso.js

---

### Troubleshooting

**Tests timing out?**
- ✅ Ensure auth server is running (`pnpm dev`)
- ✅ Check http://localhost:3001 returns a page

**Playwright tests failing?**
- ✅ Playwright may not be installed: `npx playwright install`
- ✅ Book Interface not running: Tests will gracefully fail (expected)

**Admin tests failing?**
- ✅ Ensure default admin exists: `pnpm run seed:setup`
- ✅ Check credentials: `admin@robolearn.io` / `admin@robolearn.io`

## Endpoints

| Endpoint                            | Description                             |
| ----------------------------------- | --------------------------------------- |
| `/.well-known/openid-configuration` | OIDC discovery document                 |
| `/api/auth/jwks`                    | JWKS public keys for token verification |
| `/api/auth/oauth2/authorize`        | Start OAuth flow                        |
| `/api/auth/oauth2/token`            | Exchange code for tokens                |
| `/api/auth/oauth2/userinfo`         | Get user info                           |
| `/api/admin/clients/register`       | Register OAuth client (admin only)      |

## Register New OAuth Client

**SECURITY**: Open client registration is disabled. Use one of these secure methods:

### Option 1: Admin Dashboard (Recommended for UI)

1. Sign in as admin at `http://localhost:3001/auth/sign-in`
2. Navigate to `/admin/clients`
3. Create client via UI

### Option 2: Admin API Endpoint

```bash
curl -X POST http://localhost:3001/api/admin/clients/register \
  -H "Content-Type: application/json" \
  -H "Cookie: your-admin-session-cookie" \
  -d '{
    "name": "My App",
    "redirectUrls": ["http://localhost:4000/callback"],
    "clientType": "public"
  }'
```

Returns `client_id` and `client_secret` (if confidential) to use in your OAuth flow.

## Create Admin User

```sql
-- After signing up via UI:
UPDATE "user" SET role = 'admin' WHERE email = 'you@example.com';
```

## Integration Guide

### Quick Start: Integrate Your Application

All RoboLearn applications authenticate using this OAuth 2.1/OIDC server.

#### Step 1: Register Your Application

Use the admin dashboard at `/admin/clients` to register your application:

```json
{
  "name": "My RoboLearn App",
  "redirectUrls": ["https://myapp.com/auth/callback"],
  "clientType": "public"  // or "confidential" for backend apps
}
```

You'll receive:
- **Public clients**: `client_id` only (PKCE required)
- **Confidential clients**: `client_id` + `client_secret` (save it - shown once!)

#### Step 2: Configure Your Application

**Environment Variables**:
```env
OAUTH_ISSUER=https://auth.robolearn.org
OAUTH_CLIENT_ID=your-client-id-here
OAUTH_CALLBACK_URL=https://myapp.com/auth/callback

# Only for confidential clients:
# OAUTH_CLIENT_SECRET=your-secret-here
```

---

### Frontend Integration (Public Clients)

**Example**: React/Next.js SPA using PKCE

```typescript
// lib/auth-client.ts
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.NEXT_PUBLIC_AUTH_URL,

  // OAuth 2.1 configuration
  oauth: {
    clientId: process.env.NEXT_PUBLIC_OAUTH_CLIENT_ID!,
    // No clientSecret for public clients (PKCE handles security)
  },
});

// components/LoginButton.tsx
export function LoginButton() {
  const handleLogin = async () => {
    // Redirects to auth server with PKCE challenge
    await authClient.signIn.social({
      provider: "oauth2",
      callbackURL: "/auth/callback",
    });
  };

  return <button onClick={handleLogin}>Sign In</button>;
}

// app/auth/callback/page.tsx
export default async function CallbackPage() {
  // Better Auth handles PKCE verification automatically
  const session = await authClient.getSession();

  if (session) {
    redirect("/dashboard");
  }

  return <div>Signing you in...</div>;
}
```

**Manual OAuth Flow** (if not using Better Auth client):

```typescript
// 1. Generate PKCE verifier & challenge
const verifier = generateRandomString(128);
const challenge = await sha256(verifier).then(base64url);

// 2. Redirect to authorization endpoint
const authUrl = new URL("https://auth.robolearn.org/api/auth/oauth2/authorize");
authUrl.searchParams.set("client_id", CLIENT_ID);
authUrl.searchParams.set("redirect_uri", CALLBACK_URL);
authUrl.searchParams.set("response_type", "code");
authUrl.searchParams.set("scope", "openid profile email");
authUrl.searchParams.set("code_challenge", challenge);
authUrl.searchParams.set("code_challenge_method", "S256");
window.location.href = authUrl.toString();

// 3. Exchange code for tokens (in callback)
const tokenResponse = await fetch("https://auth.robolearn.org/api/auth/oauth2/token", {
  method: "POST",
  headers: { "Content-Type": "application/x-www-form-urlencoded" },
  body: new URLSearchParams({
    grant_type: "authorization_code",
    code: authorizationCode,
    redirect_uri: CALLBACK_URL,
    client_id: CLIENT_ID,
    code_verifier: verifier,  // PKCE verification
  }),
});

const { access_token, id_token, refresh_token } = await tokenResponse.json();
```

---

### Backend Integration (Confidential Clients)

**Example**: FastAPI backend with client secret

```python
# config.py
OAUTH_ISSUER = "https://auth.robolearn.org"
OAUTH_CLIENT_ID = "your-confidential-client-id"
OAUTH_CLIENT_SECRET = "your-client-secret"
OAUTH_CALLBACK_URL = "https://api.myapp.com/auth/callback"

# auth.py
import httpx
from jose import jwt, jwk
from jose.utils import base64url_decode

# 1. Fetch JWKS for token verification
async def get_jwks():
    async with httpx.AsyncClient() as client:
        response = await client.get(f"{OAUTH_ISSUER}/api/auth/jwks")
        return response.json()

# 2. Verify ID token
async def verify_id_token(id_token: str) -> dict:
    jwks = await get_jwks()

    # Decode header to get key ID
    header = jwt.get_unverified_header(id_token)
    key = next((k for k in jwks["keys"] if k["kid"] == header["kid"]), None)

    if not key:
        raise ValueError("Key not found in JWKS")

    # Verify signature using RS256
    public_key = jwk.construct(key)
    claims = jwt.decode(
        id_token,
        public_key,
        algorithms=["RS256"],
        audience=OAUTH_CLIENT_ID,
        issuer=OAUTH_ISSUER,
    )

    return claims  # Contains user info + custom claims

# 3. Exchange authorization code for tokens
async def exchange_code(code: str) -> dict:
    async with httpx.AsyncClient() as client:
        response = await client.post(
            f"{OAUTH_ISSUER}/api/auth/oauth2/token",
            data={
                "grant_type": "authorization_code",
                "code": code,
                "redirect_uri": OAUTH_CALLBACK_URL,
                "client_id": OAUTH_CLIENT_ID,
                "client_secret": OAUTH_CLIENT_SECRET,  # Confidential client
            },
        )
        return response.json()

# 4. Get user info
async def get_user_info(access_token: str) -> dict:
    async with httpx.AsyncClient() as client:
        response = await client.get(
            f"{OAUTH_ISSUER}/api/auth/oauth2/userinfo",
            headers={"Authorization": f"Bearer {access_token}"},
        )
        return response.json()
```

---

### Token Claims Reference

**ID Token Claims** (JWT signed with RS256):

```json
{
  "iss": "https://auth.robolearn.org",
  "aud": "your-client-id",
  "sub": "user-uuid",
  "exp": 1234567890,
  "iat": 1234567890,
  "auth_time": 1234567890,

  // Standard OIDC claims
  "email": "user@example.com",
  "email_verified": true,
  "name": "John Doe",
  "given_name": "John",
  "family_name": "Doe",
  "picture": "https://...",

  // RoboLearn custom claims
  "role": "admin" | "user",
  "tenant_id": "org-uuid",
  "organization_ids": ["org-1", "org-2"],
  "org_role": "owner" | "admin" | "member",
  "software_background": "beginner" | "intermediate" | "advanced",
  "hardware_tier": "tier1" | "tier2" | "tier3" | "tier4"
}
```

**Access Token**: Opaque bearer token for API access (validate via UserInfo endpoint)

**Refresh Token**: Used to get new access tokens when expired

---

### OIDC Discovery

Discover all endpoints automatically:

```bash
curl https://auth.robolearn.org/api/auth/.well-known/openid-configuration
```

Returns:
```json
{
  "issuer": "https://auth.robolearn.org",
  "authorization_endpoint": "https://auth.robolearn.org/api/auth/oauth2/authorize",
  "token_endpoint": "https://auth.robolearn.org/api/auth/oauth2/token",
  "userinfo_endpoint": "https://auth.robolearn.org/api/auth/oauth2/userinfo",
  "jwks_uri": "https://auth.robolearn.org/api/auth/jwks",
  "scopes_supported": ["openid", "profile", "email", "offline_access"],
  "response_types_supported": ["code"],
  "grant_types_supported": ["authorization_code", "refresh_token"],
  "code_challenge_methods_supported": ["S256", "plain"]
}
```

---

### Multi-Tenant Applications

Use `tenant_id` and `organization_ids` claims to scope data:

```typescript
// Example: Filter lessons by organization
const session = await authClient.getSession();
const tenantId = session.user.tenant_id;

const lessons = await db.select()
  .from(lessons)
  .where(eq(lessons.organizationId, tenantId));
```

**Switching Organizations**:

```typescript
// Update active organization (affects tenant_id in next token)
await authClient.organization.setActive({ organizationId: "new-org-id" });
```

---

### Security Best Practices

1. **Always use HTTPS** in production
2. **Validate `iss` and `aud`** in ID tokens
3. **Verify signatures** using JWKS keys
4. **Use PKCE** for public clients (SPAs, mobile)
5. **Rotate refresh tokens** periodically
6. **Set short access token expiry** (6 hours)
7. **Store tokens securely**:
   - Frontend: HttpOnly cookies (Better Auth handles this)
   - Backend: Encrypted database or secure key vault

---

### Testing Your Integration

**1. Authorization Flow**:
```bash
# Start OAuth flow
open "https://auth.robolearn.org/api/auth/oauth2/authorize?client_id=YOUR_CLIENT_ID&redirect_uri=YOUR_CALLBACK&response_type=code&scope=openid profile email&code_challenge=CHALLENGE&code_challenge_method=S256"
```

**2. Verify JWKS**:
```bash
curl https://auth.robolearn.org/api/auth/jwks | jq
```

**3. Decode ID Token**:
```bash
# Copy ID token from response, paste at https://jwt.io
# Verify:
# - iss matches auth server URL
# - aud matches your client_id
# - Signature verifies using JWKS
```

**4. Call UserInfo**:
```bash
curl -H "Authorization: Bearer YOUR_ACCESS_TOKEN" \
  https://auth.robolearn.org/api/auth/oauth2/userinfo
```

---

### Example Applications

- **robolearn-interface**: Public SPA using PKCE ([View Source](https://github.com/mjunaidca/robolearn-interface))
- **robolearn-api**: FastAPI backend with JWT verification (Coming Soon)
- **robolearn-mobile**: React Native app with OAuth (Coming Soon)

---

## Deploy to Vercel

1. Push to GitHub
2. Import in Vercel, set root to `auth-server`
3. Add environment variables
4. Deploy
