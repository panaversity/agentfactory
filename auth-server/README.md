# RoboLearn Auth Server

OAuth 2.1 / OIDC authentication server using Better Auth.

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

### Seed Trusted Public Client

After running `db:push`, you **must** seed the trusted public client (`robolearn-public-client`) in your database. This client is pre-configured in the code but needs to exist in the database for OAuth flows to work.

**Option 1: Run SQL directly (recommended)**

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

**Option 2: Use the seed script**

If you have `DATABASE_URL` set in `.env.local`:

```bash
npx tsx scripts/seed-public-client.ts
```

**Option 3: Use the API endpoint (admin only)**

If you're logged in as admin:

```bash
curl -X POST http://localhost:3001/api/admin/seed-public-client \
  -H "Cookie: your-session-cookie"
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

# Optional: Production OAuth client callback URL
# ROBOLEARN_INTERFACE_CALLBACK_URL=https://yourdomain.com/auth/callback
```

## Features

- OAuth 2.1 with PKCE (no client secrets in browser)
- JWKS (JSON Web Key Set) for client-side token verification
- Dynamic client registration (`POST /api/auth/oauth2/register`)
- Optional email verification via Resend or SMTP
- Role-based access (admin/user)
- 7-day sessions with auto-refresh

## Endpoints

| Endpoint | Description |
|----------|-------------|
| `/.well-known/openid-configuration` | OIDC discovery document |
| `/api/auth/jwks` | JWKS public keys for token verification |
| `/api/auth/oauth2/authorize` | Start OAuth flow |
| `/api/auth/oauth2/token` | Exchange code for tokens |
| `/api/auth/oauth2/userinfo` | Get user info |
| `/api/auth/oauth2/register` | Register new OAuth client |

## Register New OAuth Client

```bash
curl -X POST http://localhost:3001/api/auth/oauth2/register \
  -H "Content-Type: application/json" \
  -d '{
    "client_name": "My App",
    "redirect_uris": ["http://localhost:4000/callback"],
    "token_endpoint_auth_method": "none"
  }'
```

Returns `client_id` to use in your OAuth flow.

## Create Admin User

```sql
-- After signing up via UI:
UPDATE "user" SET role = 'admin' WHERE email = 'you@example.com';
```

## Deploy to Vercel

1. Push to GitHub
2. Import in Vercel, set root to `auth-server`
3. Add environment variables
4. Deploy
