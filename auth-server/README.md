# RoboLearn Auth Server

OAuth 2.1 / OIDC authentication server using Better Auth.

## Setup

```bash
cd auth-server
npm install
cp .env.example .env.local
# Edit .env.local with your values
npm run db:push
npm run dev  # http://localhost:3001
```

## Environment Variables

```env
# Required
DATABASE_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
BETTER_AUTH_SECRET=your-32-char-secret  # openssl rand -base64 32
BETTER_AUTH_URL=http://localhost:3001
ALLOWED_ORIGINS=http://localhost:3000
NEXT_PUBLIC_BETTER_AUTH_URL=http://localhost:3001

# Optional: Email verification (Resend free tier: 100/day)
# RESEND_API_KEY=re_xxxxxxxxx
# RESEND_FROM_EMAIL=onboarding@resend.dev
```

## Features

- OAuth 2.1 with PKCE (no client secrets in browser)
- Dynamic client registration (`POST /api/auth/oauth2/register`)
- Optional email verification via Resend
- Role-based access (admin/user)
- 7-day sessions with auto-refresh

## Endpoints

| Endpoint | Description |
|----------|-------------|
| `/.well-known/openid-configuration` | OIDC discovery |
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
