# ADR-0003: OAuth/OIDC Provider Strategy

> **Scope**: OAuth 2.1 / OIDC Provider implementation strategy including PKCE, JWKS, public/confidential client types, token signing, and multi-app SSO architecture. These components work together to enable the auth server to act as an identity provider for multiple client applications.

- **Status:** Accepted
- **Date:** 2025-11-30
- **Feature:** 003-auth-server
- **Context:** RoboLearn platform initially planned simple email/password auth but requirements evolved to support multiple client applications (robolearn-interface, future author dashboard, potential institutional white-label deployments). Rather than each app implementing its own auth, a centralized OAuth/OIDC Provider enables Single Sign-On (SSO) across all apps while maintaining security through modern standards (PKCE, JWKS). System must support both public clients (SPAs, mobile apps - no secrets) and confidential clients (server-side apps with secrets), handle 10+ apps with 10,000+ users each, and enable offline token verification to reduce server load.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: ✅ Defines authentication architecture for entire multi-app platform
     2) Alternatives: ✅ Full OIDC vs simple auth, PKCE vs secrets, JWKS vs shared secrets
     3) Scope: ✅ Cross-cutting security decision affecting all client apps, scalability
-->

## Decision

Implement **Full OAuth 2.1 / OIDC Provider with PKCE + JWKS (RS256)** using Better Auth plugins:

| Component | Implementation | Rationale |
|-----------|----------------|-----------|
| **OAuth Standard** | OAuth 2.1 + OIDC | Industry standard, maximum compatibility with future apps |
| **Public Client Security** | PKCE (Proof Key for Code Exchange) | No client secrets in browser code, prevents auth code interception |
| **Token Signing** | JWKS with RS256 (asymmetric) | Enables offline token verification, reduces server load |
| **Client Types** | Public (PKCE) + Confidential (secrets) | Supports SPAs, mobile apps (public) + server-side apps (confidential) |
| **Client Registration** | Dynamic + Admin-only endpoint | Trusted clients (robolearn-interface) pre-registered, future apps via admin approval |
| **Token Expiry** | Access 6h, Refresh 7d, Code 10m | Balances security (short access) with UX (long refresh) |
| **Email Verification** | Resend/SMTP fallback | Required for security, multiple providers for reliability |
| **Consent Screen** | Skipped for first-party apps | Trusted apps (robolearn-interface) bypass consent, third-party would show consent |

**Key Integration Points:**
- Better Auth OIDC Provider plugin handles `/oauth2/authorize`, `/oauth2/token`, `/oauth2/userinfo`
- Better Auth JWT plugin generates JWKS keys, exposes `/api/auth/jwks` endpoint
- Public client configuration: `clientSecret = null`, `type = "public"`, requires PKCE
- Confidential client configuration: `clientSecret` set, `type = "confidential"`, optional PKCE

**Architecture Flow:**
```
Client App → /oauth2/authorize (+ PKCE challenge)
         ← redirect to sign-in if not authenticated
         ← authorization code after sign-in
Client App → /oauth2/token (+ PKCE verifier, no secret for public clients)
         ← access_token (JWT signed with RS256)
         ← id_token (user identity, signed with RS256)
         ← refresh_token (for future token renewal)
Client App → Verify ID token offline using /api/auth/jwks (no server call!)
```

## Consequences

### Positive

- **Multi-app SSO**: Single login works across all RoboLearn apps (book interface, author dashboard, admin panel)
- **Scalability**: JWKS enables offline token verification → clients don't hit auth server per request → handles 10,000+ users per app
- **Security**: PKCE prevents auth code interception attacks (standard for public clients since 2019)
- **Future-proof**: OIDC standard ensures compatibility with any OAuth-compliant app
- **Reduced server load**: Client-side token verification via JWKS (no `/userinfo` calls per request)
- **Flexibility**: Supports both SPAs (public) and server-side apps (confidential) with same provider
- **Admin control**: Dynamic client registration restricted to admins prevents unauthorized apps
- **Email verification**: Ensures real email addresses, reduces spam accounts

### Negative

- **Complexity**: Full OAuth 2.1 + OIDC significantly more complex than simple email/password auth
- **Token management**: Clients must handle access/refresh token rotation, expiry, storage
- **PKCE implementation**: Clients must implement PKCE correctly (code verifier/challenge generation)
- **JWKS key rotation**: Must handle key rotation without breaking existing tokens (Better Auth handles this, but adds operational overhead)
- **Multi-window scenarios**: Refresh token rotation can conflict if user opens app in multiple tabs
- **Email provider dependency**: Email verification requires Resend/SMTP configuration (adds external dependency)
- **Plugin coupling**: Tightly coupled to Better Auth OIDC Provider + JWT plugins (migration would be significant)
- **Debugging difficulty**: OAuth flows harder to debug than simple session-based auth

## Alternatives Considered

### Alternative A: Simple Email/Password (Session-Based)
- **Implementation**: Email/password → server creates session cookie
- **Clients**: All apps share session cookie (same domain or subdomains)
- **Token verification**: Every request validates session server-side
- **Rejected Reason**: Doesn't scale to 10+ apps with 10,000+ users (server bottleneck). Cookie domain restrictions limit deployment flexibility. No standard for third-party integrations.

### Alternative B: OAuth with Shared Secrets (No PKCE, No JWKS)
- **Implementation**: OAuth 2.0 with client secrets for all clients
- **Token signing**: HS256 (symmetric) with shared secret
- **Token verification**: Server-side validation required
- **Rejected Reason**: Secrets cannot be safely stored in browser code (SPAs). HS256 requires sharing secret with all clients (security risk). No offline token verification (server load).

### Alternative C: Managed Auth Providers (Auth0, Clerk, Supabase)
- **Implementation**: Delegate OAuth provider to third-party service
- **Client integration**: Use provider's SDKs
- **Token verification**: Provider handles JWKS
- **Rejected Reason**: Costs scale with MAUs (Auth0: $23/month per 1000 users, Clerk: $25/month + $0.02/MAU). Vendor lock-in. Loss of control over auth flows. Reduced learning opportunity.

### Alternative D: Custom JWT Implementation (No OAuth)
- **Implementation**: Roll own JWT generation + verification
- **Token signing**: Custom key management
- **Client integration**: Custom client libraries
- **Rejected Reason**: Reinventing the wheel, security risk (easy to make mistakes). No standard for multi-app SSO. No third-party app support. Significant maintenance burden.

## References

- Feature Spec: `specs/003-auth-server/spec.md` (see FR-013 to FR-018)
- Implementation Plan: `specs/003-auth-server/plan.md` (see Phase 2.5: OAuth/OIDC Provider Setup)
- Better Auth OIDC Provider Plugin: https://www.better-auth.com/docs/plugins/oidc-provider
- Better Auth JWT Plugin: https://www.better-auth.com/docs/plugins/jwt
- OAuth 2.1 Standard: https://oauth.net/2.1/
- PKCE RFC 7636: https://datatracker.ietf.org/doc/html/rfc7636
- JWKS RFC 7517: https://datatracker.ietf.org/doc/html/rfc7517
- Related ADRs: ADR-0002 (Authentication Technology Stack), ADR-0004 (Standalone Deployment Architecture)
- Implementation Skill: `.claude/skills/engineering/better-auth-setup/SKILL.md` (see Section 4-6: Admin registration, seeding, email, JWKS)
