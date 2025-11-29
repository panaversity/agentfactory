# RoboLearn Auth Server

OAuth 2.1 / OIDC authentication server for the RoboLearn platform using Better Auth.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      Auth Server (Port 3001)                     │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │  OIDC Provider  │  │  Admin Plugin   │  │   User Mgmt     │  │
│  │  - /authorize   │  │  - Role mgmt    │  │  - Profiles     │  │
│  │  - /token       │  │  - User admin   │  │  - Preferences  │  │
│  │  - /userinfo    │  │  - Ban/unban    │  │                 │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
│                              │                                   │
│                    ┌─────────┴─────────┐                        │
│                    │  Neon Postgres    │                        │
│                    │  (User data)      │                        │
│                    └───────────────────┘                        │
└─────────────────────────────────────────────────────────────────┘
                              │
                    OAuth 2.1 Flow
                              │
┌─────────────────────────────────────────────────────────────────┐
│                   Book Interface (Port 3000)                     │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │  OAuth Client   │  │  Token Storage  │  │   NavbarAuth    │  │
│  │  - Auth flow    │  │  - localStorage │  │  - User menu    │  │
│  │  - Callback     │  │  - Auto-refresh │  │  - Sign out     │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Features

- **OAuth 2.1 Compliant**: Authorization Code Flow with PKCE support
- **OIDC Discovery**: Standard `.well-known/openid-configuration` endpoint
- **Role-Based Access**: Admin, user, and custom roles
- **Multi-App SSO**: Single sign-on across all RoboLearn applications
- **User Profiles**: Software background level tracking
- **Rate Limiting**: 5 attempts/minute/IP
- **Session Management**: 7-day sessions with 24-hour refresh

## Quick Start

### Local Development

```bash
# 1. Install dependencies
cd auth-server
npm install

# 2. Copy environment file
cp .env.example .env.local

# 3. Edit .env.local with your Neon database URL

# 4. Push database schema
npm run db:push

# 5. Start development server
npm run dev  # Runs on http://localhost:3001
```

### Environment Variables

```env
# Database (Neon Postgres) - REQUIRED
DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require

# Better Auth - REQUIRED
# Secret key for signing tokens - minimum 32 characters
# Generate with: openssl rand -base64 32
BETTER_AUTH_SECRET=your-secret-key-minimum-32-characters-long
BETTER_AUTH_URL=http://localhost:3001

# CORS - REQUIRED for production
# Allowed origins (comma-separated list of frontend URLs)
ALLOWED_ORIGINS=http://localhost:3000,https://robolearn.github.io

# OAuth Callback URL - REQUIRED for production
# The callback URL for the robolearn-interface OAuth client
ROBOLEARN_INTERFACE_CALLBACK_URL=http://localhost:3000/auth/callback

# Client-side URLs
NEXT_PUBLIC_BETTER_AUTH_URL=http://localhost:3001
NEXT_PUBLIC_BOOK_URL=http://localhost:3000

# Node Environment
NODE_ENV=development
```

**Note**: No client secret is needed for robolearn-interface - it uses PKCE (Proof Key for Code Exchange) for secure authentication without exposing secrets in the browser.

---

## Admin Setup & Role Management

### Creating the First Admin

**Option 1: Using the script (recommended)**

```bash
# Create admin user directly in database
node scripts/set-admin-role.js
```

**Option 2: Manual SQL**

```sql
-- First, sign up a user through the UI, then run:
UPDATE "user" SET role = 'admin' WHERE email = 'your-email@example.com';
```

**Option 3: Via code in auth.ts**

```typescript
// In src/lib/auth.ts, add to admin plugin config:
admin({
  defaultRole: "user",
  adminRoles: ["admin"],
  adminUserIds: ["your-user-id-here"],  // Gets admin on first login
}),
```

### Role Hierarchy

| Role | Permissions |
|------|-------------|
| `admin` | Full access: manage users, ban/unban, view all data, manage OAuth clients |
| `user` | Default: access own profile, use book interface |
| Custom | Define in `adminRoles` array for custom permissions |

### Admin API Endpoints

```bash
# List all users (admin only)
GET /api/auth/admin/list-users

# Ban a user
POST /api/auth/admin/ban-user
{ "userId": "user-id", "banReason": "Spam" }

# Unban a user
POST /api/auth/admin/unban-user
{ "userId": "user-id" }

# Set user role
POST /api/auth/admin/set-role
{ "userId": "user-id", "role": "admin" }
```

### Admin Dashboard

Access the admin dashboard at: `http://localhost:3001/admin`

Features:
- User management (list, search, ban/unban)
- OAuth client management
- Session monitoring

---

## Data Ownership

### Who Owns the User Data?

**You own all user data.** The auth server stores everything in YOUR Neon Postgres database.

```
Your Neon Database
├── user              # Core user data (email, name, role)
├── session           # Active sessions
├── account           # Auth provider accounts (credentials, OAuth)
├── user_profile      # RoboLearn-specific preferences
├── oauth_application # Registered OAuth clients
├── oauth_access_token # Issued tokens
└── oauth_consent     # User consent records
```

### Data Portability

Export all user data:

```sql
-- Export users
SELECT * FROM "user";

-- Export with profiles
SELECT u.*, p.software_background
FROM "user" u
LEFT JOIN user_profile p ON u.id = p.user_id;
```

### Data Deletion (GDPR Compliance)

```sql
-- Delete user and all related data (cascades automatically)
DELETE FROM "user" WHERE id = 'user-id';
```

---

## OAuth 2.1 Endpoints

### OIDC Discovery

```bash
GET /.well-known/openid-configuration
# Returns all available endpoints
```

### Authorization Endpoint

```bash
GET /api/auth/oauth2/authorize
  ?client_id=robolearn-interface
  &redirect_uri=http://localhost:3000/auth/callback
  &response_type=code
  &scope=openid profile email
  &state=random-state-string
  &code_challenge=SHA256-hash-of-code-verifier
  &code_challenge_method=S256
```

### Token Endpoint

```bash
POST /api/auth/oauth2/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code
&code=authorization-code
&redirect_uri=http://localhost:3000/auth/callback
&client_id=robolearn-interface
&code_verifier=your-pkce-code-verifier
```

**Note**: Uses PKCE `code_verifier` instead of `client_secret` for public clients (SPAs).

### UserInfo Endpoint

```bash
GET /api/auth/oauth2/userinfo
Authorization: Bearer access-token
```

---

## Deployment

### Vercel (Recommended)

1. **Push to GitHub**

2. **Create Vercel Project**
   - Import repository
   - Set root directory to `auth-server`

3. **Set Environment Variables**
   ```
   DATABASE_URL=your-neon-connection-string
   BETTER_AUTH_SECRET=random-32-char-secret
   BETTER_AUTH_URL=https://your-auth-domain.vercel.app
   ALLOWED_ORIGINS=https://robolearn.github.io,https://your-book-domain.com
   ROBOLEARN_INTERFACE_CALLBACK_URL=https://your-book-domain.com/auth/callback
   NEXT_PUBLIC_BOOK_URL=https://your-book-domain.com
   ```

4. **Deploy**

### Cloud Run / Railway / Render

Same process - set environment variables and deploy the Next.js app.

### Production Checklist

- [ ] Set strong `BETTER_AUTH_SECRET` (32+ random chars via `openssl rand -base64 32`)
- [ ] Set `DATABASE_URL` to your Neon connection string
- [ ] Update `ALLOWED_ORIGINS` to production domains (comma-separated)
- [ ] Update `BETTER_AUTH_URL` to production URL
- [ ] Set `ROBOLEARN_INTERFACE_CALLBACK_URL` to production callback URL
- [ ] Set `NEXT_PUBLIC_BOOK_URL` to production book interface URL
- [ ] Enable HTTPS (automatic on Vercel/Railway)
- [ ] Create admin user and set role
- [ ] Test OAuth flow end-to-end

---

## Registering New OAuth Clients

For future apps that want to use this auth server:

### Option 1: Pre-register in Code

```typescript
// In src/lib/auth.ts, add to trustedClients:
{
  clientId: "new-app",
  clientSecret: process.env.NEW_APP_CLIENT_SECRET,
  name: "New Application",
  redirectUrls: ["https://new-app.com/callback"],
  skipConsent: false,  // Show consent screen
}
```

### Option 2: Dynamic Registration

The OIDC provider supports dynamic client registration:

```bash
POST /api/auth/oauth2/register
{
  "client_name": "New App",
  "redirect_uris": ["https://new-app.com/callback"]
}
```

---

## Security Considerations

### Token Lifetimes

| Token | Lifetime |
|-------|----------|
| Access Token | 1 hour |
| Refresh Token | 30 days |
| Session | 7 days |

### Security Features

- **PKCE (RFC 7636)**: Required for public clients (SPAs, mobile apps)
- **Public Client Support**: No client secret exposed in browser - uses PKCE instead
- **Automatic Token Refresh**: Client automatically refreshes expired access tokens
- **Rate Limiting**: 5 attempts/minute per IP
- **CORS**: Strict origin checking via `trustedOrigins`
- **Secure Cookies**: HTTPOnly, SameSite in production
- **HTTPS**: Required in production

### PKCE Flow (How It Works)

```
1. Client generates code_verifier (random 32 bytes, base64url)
2. Client computes code_challenge = SHA256(code_verifier)
3. Authorization request includes code_challenge
4. Token exchange includes code_verifier (not client_secret)
5. Server verifies SHA256(code_verifier) == stored code_challenge
```

This prevents authorization code interception attacks without requiring a client secret.

### Custom Claims in UserInfo

The `/api/auth/oauth2/userinfo` endpoint returns standard OIDC claims plus:
- `software_background` - User's self-reported skill level (beginner/intermediate/advanced)
- `role` - User's role (user/admin)

### Logout Options

| Type | Behavior |
|------|----------|
| Local Logout | Clears tokens from client, user stays logged in at auth server (SSO) |
| Global Logout | Clears tokens AND ends auth server session (logs out everywhere) |

### Secrets Management

Never commit secrets. Use:
- Vercel Environment Variables
- Railway/Render secrets
- `.env.local` (gitignored)

**Note**: Public clients (like robolearn-interface) no longer need `ROBOLEARN_CLIENT_SECRET` - they use PKCE instead.

---

## Troubleshooting

### "Cannot read properties of undefined (reading 'find')"

Wrong property name in trustedClients. Use `redirectUrls` (lowercase), not `redirectURLs`.

### CORS Errors

Add your origin to `ALLOWED_ORIGINS` environment variable.

### Session Not Persisting

Check cookies are being set with correct domain. In development, both apps must be on localhost.

### OAuth Callback 404

Ensure the callback URL exactly matches what's registered in `trustedClients.redirectUrls`.

---

## API Reference

### Authentication

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/auth/sign-up/email` | POST | Register new user |
| `/api/auth/sign-in/email` | POST | Sign in |
| `/api/auth/sign-out` | POST | Sign out |
| `/api/auth/get-session` | GET | Get current session |

### OAuth/OIDC

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/.well-known/openid-configuration` | GET | OIDC discovery |
| `/api/auth/oauth2/authorize` | GET | Authorization |
| `/api/auth/oauth2/token` | POST | Token exchange |
| `/api/auth/oauth2/userinfo` | GET | User info |

### Profile

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/profile` | GET | Get profile |
| `/api/profile` | POST | Create profile |
| `/api/profile` | PUT | Update profile |

### Admin

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/auth/admin/list-users` | GET | List all users |
| `/api/auth/admin/ban-user` | POST | Ban user |
| `/api/auth/admin/unban-user` | POST | Unban user |
| `/api/auth/admin/set-role` | POST | Set user role |
