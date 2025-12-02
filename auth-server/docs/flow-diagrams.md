# Authentication Flow Diagrams

Visual diagrams of all authentication flows supported by the RoboLearn Auth Server.

## 1. Email/Password Sign-Up Flow

```
┌────────────┐          ┌──────────────┐          ┌──────────────┐
│   User     │          │  Auth Server │          │    Email     │
│  Browser   │          │              │          │   Service    │
└─────┬──────┘          └──────┬───────┘          └──────┬───────┘
      │                        │                         │
      │ POST /sign-up/email    │                         │
      │ {email, password,      │                         │
      │  name, ...}            │                         │
      │───────────────────────▶│                         │
      │                        │                         │
      │                        │ Create user (unverified)│
      │                        │──────┐                  │
      │                        │      │                  │
      │                        │◀─────┘                  │
      │                        │                         │
      │                        │ Send verification email │
      │                        │────────────────────────▶│
      │                        │                         │
      │  Return user info      │                         │
      │◀───────────────────────│                         │
      │                        │                         │
      │                        │                         │
      │  User clicks link      │                         │
      │  in email              │                         │
      │                        │                         │
      │ GET /verify-email      │                         │
      │ ?token=xxx             │                         │
      │───────────────────────▶│                         │
      │                        │                         │
      │                        │ Mark email verified     │
      │                        │ Create session          │
      │                        │──────┐                  │
      │                        │      │                  │
      │                        │◀─────┘                  │
      │                        │                         │
      │  Set session cookie    │                         │
      │  Redirect to app       │                         │
      │◀───────────────────────│                         │
      │                        │                         │
```

## 2. Email/Password Sign-In Flow

```
┌────────────┐          ┌──────────────┐
│   User     │          │  Auth Server │
│  Browser   │          │              │
└─────┬──────┘          └──────┬───────┘
      │                        │
      │ POST /sign-in/email    │
      │ {email, password}      │
      │───────────────────────▶│
      │                        │
      │                        │ Verify credentials
      │                        │ Check email verified
      │                        │ Check not banned
      │                        │──────┐
      │                        │      │
      │                        │◀─────┘
      │                        │
      │                        │ Create session
      │                        │──────┐
      │                        │      │
      │                        │◀─────┘
      │                        │
      │  Set session cookie    │
      │  Return user + session │
      │◀───────────────────────│
      │                        │
```

## 3. OAuth 2.1 + PKCE Flow (Public Client)

```
┌────────────┐          ┌──────────────┐          ┌──────────────┐
│   Client   │          │  Auth Server │          │   Resource   │
│   (SPA)    │          │              │          │    Server    │
└─────┬──────┘          └──────┬───────┘          └──────┬───────┘
      │                        │                         │
      │ Generate PKCE          │                         │
      │ verifier + challenge   │                         │
      │──────┐                 │                         │
      │      │                 │                         │
      │◀─────┘                 │                         │
      │                        │                         │
      │ GET /oauth2/authorize  │                         │
      │ ?client_id=...         │                         │
      │ &code_challenge=...    │                         │
      │ &state=...             │                         │
      │───────────────────────▶│                         │
      │                        │                         │
      │                        │ Check session           │
      │                        │ (redirect to login      │
      │                        │  if not logged in)      │
      │                        │──────┐                  │
      │                        │      │                  │
      │                        │◀─────┘                  │
      │                        │                         │
      │  JSON: {redirect: true,│                         │
      │   url: "...?code=xxx"} │                         │
      │◀───────────────────────│                         │
      │                        │                         │
      │ Client redirects to    │                         │
      │ callback URL           │                         │
      │──────┐                 │                         │
      │      │                 │                         │
      │◀─────┘                 │                         │
      │                        │                         │
      │ POST /oauth2/token     │                         │
      │ grant_type=authz_code  │                         │
      │ code=xxx               │                         │
      │ code_verifier=...      │                         │
      │───────────────────────▶│                         │
      │                        │                         │
      │                        │ Verify PKCE             │
      │                        │ SHA256(verifier)        │
      │                        │ == challenge?           │
      │                        │──────┐                  │
      │                        │      │                  │
      │                        │◀─────┘                  │
      │                        │                         │
      │  {access_token,        │                         │
      │   id_token,            │                         │
      │   token_type: Bearer}  │                         │
      │◀───────────────────────│                         │
      │                        │                         │
      │                        │                         │
      │ GET /api/resource      │                         │
      │ Authorization: Bearer  │                         │
      │ <access_token>         │                         │
      │────────────────────────│────────────────────────▶│
      │                        │                         │
      │                        │           Verify token  │
      │                        │           via JWKS      │
      │                        │           ──────┐       │
      │                        │                 │       │
      │                        │           ◀─────┘       │
      │                        │                         │
      │                        │         Return resource │
      │◀───────────────────────│─────────────────────────│
      │                        │                         │
```

## 4. OAuth 2.1 Confidential Client Flow

```
┌────────────┐          ┌──────────────┐
│ Backend    │          │  Auth Server │
│ Server     │          │              │
└─────┬──────┘          └──────┬───────┘
      │                        │
      │ POST /oauth2/token     │
      │ Authorization: Basic   │
      │ base64(client_id:      │
      │        client_secret)  │
      │                        │
      │ grant_type=authz_code  │
      │ code=xxx               │
      │ redirect_uri=...       │
      │───────────────────────▶│
      │                        │
      │                        │ Verify client
      │                        │ credentials
      │                        │──────┐
      │                        │      │
      │                        │◀─────┘
      │                        │
      │  {access_token,        │
      │   refresh_token,       │
      │   id_token}            │
      │◀───────────────────────│
      │                        │
```

## 5. Token Refresh Flow

```
┌────────────┐          ┌──────────────┐
│   Client   │          │  Auth Server │
└─────┬──────┘          └──────┬───────┘
      │                        │
      │ POST /oauth2/token     │
      │ grant_type=refresh     │
      │ refresh_token=xxx      │
      │ client_id=...          │
      │───────────────────────▶│
      │                        │
      │                        │ Verify refresh token
      │                        │ Check not expired
      │                        │ Check not revoked
      │                        │──────┐
      │                        │      │
      │                        │◀─────┘
      │                        │
      │  {access_token,        │
      │   token_type: Bearer,  │
      │   expires_in: 21600}   │
      │◀───────────────────────│
      │                        │
```

## 6. Token Verification Flow (Backend)

```
┌────────────┐          ┌──────────────┐          ┌──────────────┐
│  Backend   │          │  Auth Server │          │    Client    │
│   API      │          │   (JWKS)     │          │              │
└─────┬──────┘          └──────┬───────┘          └──────┬───────┘
      │                        │                         │
      │ GET /api/auth/jwks     │                         │
      │───────────────────────▶│                         │
      │                        │                         │
      │  {keys: [{kty, n, e,   │                         │
      │    kid, alg, use}]}    │                         │
      │◀───────────────────────│                         │
      │                        │                         │
      │ Cache JWKS             │                         │
      │──────┐                 │                         │
      │      │                 │                         │
      │◀─────┘                 │                         │
      │                        │                         │
      │                        │        API Request with │
      │                        │        Authorization:   │
      │                        │        Bearer <token>   │
      │◀───────────────────────│─────────────────────────│
      │                        │                         │
      │ Extract token from     │                         │
      │ Authorization header   │                         │
      │──────┐                 │                         │
      │      │                 │                         │
      │◀─────┘                 │                         │
      │                        │                         │
      │ Verify signature       │                         │
      │ using cached JWKS      │                         │
      │──────┐                 │                         │
      │      │                 │                         │
      │◀─────┘                 │                         │
      │                        │                         │
      │ Check exp, iss, aud    │                         │
      │──────┐                 │                         │
      │      │                 │                         │
      │◀─────┘                 │                         │
      │                        │                         │
      │                        │           API Response  │
      │────────────────────────│────────────────────────▶│
      │                        │                         │
```

## 7. Dynamic Client Registration

```
┌────────────┐          ┌──────────────┐
│ New Client │          │  Auth Server │
└─────┬──────┘          └──────┬───────┘
      │                        │
      │ POST /oauth2/register  │
      │ {client_name,          │
      │  redirect_uris,        │
      │  grant_types,          │
      │  token_endpoint_       │
      │  auth_method}          │
      │───────────────────────▶│
      │                        │
      │                        │ Generate client_id
      │                        │ Generate client_secret
      │                        │ (if not public)
      │                        │──────┐
      │                        │      │
      │                        │◀─────┘
      │                        │
      │  {client_id,           │
      │   client_secret,       │
      │   redirect_uris, ...}  │
      │◀───────────────────────│
      │                        │
```

## 8. Organization/Tenant Flow

```
┌────────────┐          ┌──────────────┐
│   Admin    │          │  Auth Server │
│   User     │          │              │
└─────┬──────┘          └──────┬───────┘
      │                        │
      │ POST /organization/    │
      │ create                 │
      │ {name, slug}           │
      │───────────────────────▶│
      │                        │
      │                        │ Create organization
      │                        │ Add user as owner
      │                        │──────┐
      │                        │      │
      │                        │◀─────┘
      │                        │
      │  {id, name, slug}      │
      │◀───────────────────────│
      │                        │
      │                        │
      │ POST /organization/    │
      │ invite-member          │
      │ {email, role}          │
      │───────────────────────▶│
      │                        │
      │                        │ Create invitation
      │                        │ Send email
      │                        │──────┐
      │                        │      │
      │                        │◀─────┘
      │                        │
      │  {invitation_id}       │
      │◀───────────────────────│
      │                        │
```

## 9. Password Reset Flow

```
┌────────────┐          ┌──────────────┐          ┌──────────────┐
│   User     │          │  Auth Server │          │    Email     │
└─────┬──────┘          └──────┬───────┘          └──────┬───────┘
      │                        │                         │
      │ POST /forgot-password  │                         │
      │ {email}                │                         │
      │───────────────────────▶│                         │
      │                        │                         │
      │                        │ Generate reset token    │
      │                        │──────┐                  │
      │                        │      │                  │
      │                        │◀─────┘                  │
      │                        │                         │
      │                        │ Send reset email        │
      │                        │────────────────────────▶│
      │                        │                         │
      │  {success: true}       │                         │
      │◀───────────────────────│                         │
      │                        │                         │
      │                        │                         │
      │ User clicks link       │                         │
      │                        │                         │
      │ POST /reset-password   │                         │
      │ {token, newPassword}   │                         │
      │───────────────────────▶│                         │
      │                        │                         │
      │                        │ Verify token            │
      │                        │ Update password         │
      │                        │──────┐                  │
      │                        │      │                  │
      │                        │◀─────┘                  │
      │                        │                         │
      │  {success: true}       │                         │
      │◀───────────────────────│                         │
      │                        │                         │
```

## Endpoint Summary

| Flow | Endpoints |
|------|-----------|
| Sign-up | `POST /api/auth/sign-up/email` |
| Sign-in | `POST /api/auth/sign-in/email` |
| Verify email | `GET /api/auth/verify-email?token=` |
| OAuth authorize | `GET /api/auth/oauth2/authorize` |
| OAuth token | `POST /api/auth/oauth2/token` |
| OAuth userinfo | `GET /api/auth/oauth2/userinfo` |
| OIDC discovery | `GET /api/auth/.well-known/openid-configuration` |
| JWKS | `GET /api/auth/jwks` |
| Client registration | `POST /api/auth/oauth2/register` |
| Organization create | `POST /api/auth/organization/create` |
| Password reset | `POST /api/auth/forgot-password` |
