# ADR 001: Standalone Auth Server Architecture

**Date**: 2025-12-02
**Status**: ✅ Accepted
**Deciders**: @mjunaidca
**Related Issues**: #16 - Production-ready OAuth 2.1/OIDC server

---

## Context

The RoboLearn platform requires a centralized authentication and authorization server that supports:
1. OAuth 2.1 / OIDC for third-party integrations
2. Multi-tenant organization management
3. User profile management with educational metadata
4. Session management across multiple applications
5. Admin dashboard for client and user management

We needed to decide between:
- **Monorepo**: Auth server as a package within the robolearn-interface monorepo
- **Standalone**: Separate repository/service for auth-server

---

## Decision

We will implement **auth-server as a standalone service** in its own repository at `robolearn-auth/auth-server`.

---

## Rationale

### Advantages of Standalone Architecture

#### 1. **Independent Deployment Lifecycle**
- Auth server changes don't require redeploying robolearn-interface
- Security patches can be applied immediately without coordinating releases
- Zero downtime updates using blue-green deployment
- Different scaling requirements (auth server scales horizontally, interface scales differently)

**Example**: Security vulnerability in Better Auth → patch auth-server in minutes without touching interface.

#### 2. **Clear Security Boundary**
- Separate codebase reduces attack surface
- Easier to audit and penetration test
- Secrets (JWT signing keys, database credentials) isolated
- Different access controls for auth vs interface repositories

**Security Principle**: Minimize blast radius of potential security breaches.

#### 3. **Technology Independence**
- Auth server uses Next.js 15 + Better Auth + Neon Postgres
- Interface can use different framework (React, Vue, Svelte)
- Future apps (mobile, desktop) consume same auth server
- No dependency coupling between services

**Example**: Migrate interface to Astro without touching auth server.

#### 4. **Team Scalability**
- Security-focused developers work on auth-server
- Frontend developers work on interface
- Clear ownership and responsibility boundaries
- Simpler CI/CD pipelines (each service has own pipeline)

#### 5. **Reusability Across RoboLearn Ecosystem**
Current consumers:
- `robolearn-interface` (book reading interface)

Future consumers (planned):
- `robolearn-author-dashboard` (book authoring tools)
- `robolearn-admin` (institution management)
- `robolearn-mobile` (iOS/Android apps)
- Third-party integrations (LMS, school portals)

**Architectural Principle**: One auth server serves ALL RoboLearn applications.

#### 6. **Compliance and Audit**
- Centralized audit logs for all authentication events
- Easier to demonstrate OAuth 2.1/OIDC compliance
- Single source of truth for user data
- Simplified GDPR/privacy compliance

---

### Disadvantages (and Mitigations)

| Disadvantage | Mitigation |
|--------------|------------|
| **Extra network hop** | Negligible latency (~5-10ms), mitigated by session caching |
| **Additional infrastructure** | Neon free tier covers 200+ users, minimal cost |
| **Coordination for schema changes** | Versioned API, backward compatibility guarantees |
| **Local development complexity** | Docker Compose for single-command setup |

---

## Alternative Considered: Monorepo

### Why We Rejected It

1. **Coupling**: Interface changes could break auth unintentionally
2. **Deploy Overhead**: Small auth fix requires full interface rebuild
3. **Scaling Complexity**: Can't scale auth independently
4. **Repository Size**: Monorepo grows large over time
5. **Permission Management**: Harder to restrict access to auth code

**Verdict**: Monorepo benefits (shared code, atomic commits) don't outweigh standalone benefits for this use case.

---

## Implementation Plan

### Phase 1: Core Auth Server ✅ (Current)
- [x] Better Auth OAuth 2.1/OIDC provider
- [x] User authentication (email/password)
- [x] Email verification (SMTP/Resend)
- [x] Admin panel for client management
- [x] Multi-tenant organizations
- [x] JWKS endpoint (RS256 asymmetric signing)
- [x] Password strength validation (HIBP)

### Phase 2: Production Deployment (Week 1)
- [ ] Deploy to Cloudflare Pages / Vercel
- [ ] Configure custom domain (`auth.robolearn.org`)
- [ ] Set up monitoring (Sentry, Uptime Robot)
- [ ] Enable Neon backups (point-in-time recovery)
- [ ] Document integration guide in README

### Phase 3: Advanced Features (Month 1)
- [ ] Social OAuth (Google, GitHub, Microsoft)
- [ ] Refresh token rotation
- [ ] Device flow for CLI apps
- [ ] Admin audit logs
- [ ] Rate limiting per client

---

## Integration Architecture

### Current Setup

```
┌─────────────────────┐
│ robolearn-interface │
│   (localhost:3000)  │
└──────────┬──────────┘
           │
           │ OAuth 2.1 / OIDC
           │ (PKCE flow)
           │
           ▼
┌─────────────────────┐
│   auth-server       │
│   (localhost:3001)  │
│                     │
│  Better Auth        │
│  + Neon Postgres    │
└─────────────────────┘
```

### Production Setup

```
┌─────────────────────────┐
│ book.robolearn.org      │
│ (robolearn-interface)   │
│ Cloudflare Pages        │
└───────────┬─────────────┘
            │
            │ HTTPS OAuth Flow
            │ + Session Cookies
            │
            ▼
┌─────────────────────────┐
│ auth.robolearn.org      │
│ (auth-server)           │
│ Vercel / Cloudflare     │
│                         │
│ Better Auth + Neon      │
└───────────┬─────────────┘
            │
            │ Postgres
            │
            ▼
┌─────────────────────────┐
│ Neon Cloud Database     │
│ (Frankfurt region)      │
└─────────────────────────┘
```

---

## Environment Variables

### Auth Server
```bash
# Core
BETTER_AUTH_URL=https://auth.robolearn.org
BETTER_AUTH_SECRET=<256-bit secret>

# Database
DATABASE_URL=postgresql://...neon.tech/robolearn

# Email (SMTP or Resend)
SMTP_HOST=smtp.gmail.com
SMTP_USER=hello@robolearn.org
SMTP_PASS=<app-password>
EMAIL_FROM=hello@robolearn.org

# CORS
ALLOWED_ORIGINS=https://book.robolearn.org

# Note: OAuth client redirect URLs are configured in src/lib/trusted-clients.ts
# Production localhost URLs are automatically filtered via environment-based logic
```

### Robolearn Interface
```bash
# Auth Server
OAUTH_ISSUER=https://auth.robolearn.org
OAUTH_CLIENT_ID=robolearn-public-client
OAUTH_CALLBACK_URL=https://book.robolearn.org/auth/callback
```

---

## Success Metrics

### Technical Metrics
- **Uptime**: 99.9% availability (monitored via Uptime Robot)
- **Latency**: < 100ms for token validation (P95)
- **Security**: Zero unauthorized access incidents
- **Compliance**: Pass external OAuth 2.1 conformance tests

### Business Metrics
- **Scalability**: Support 200+ daily active users on free tier
- **Cost**: $0-19/month for first 500 users
- **Integration Speed**: New app integration < 30 minutes
- **Developer Experience**: Clear docs, working examples

---

## Risks and Mitigations

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| **Service Outage** | Low | High | Auto-scaling, health checks, failover |
| **Data Breach** | Low | Critical | Encryption at rest/transit, audit logs |
| **Breaking Changes** | Medium | Medium | Semantic versioning, deprecation notices |
| **Vendor Lock-in** | Low | Medium | Better Auth is open source, DB portable |

---

## Future Considerations

### When to Reconsider This Decision

We should re-evaluate standalone architecture if:
1. **Team < 3 people**: Overhead of managing separate service becomes burden
2. **Single App Only**: No plans for additional RoboLearn applications
3. **Local-First**: Shift to local-only deployment (no cloud)
4. **Cost Constraints**: Running separate service exceeds budget

**Trigger**: Review this ADR every 6 months or when adding new application to ecosystem.

---

## References

- [OAuth 2.1 Spec](https://datatracker.ietf.org/doc/html/draft-ietf-oauth-v2-1-07)
- [OIDC Core Spec](https://openid.net/specs/openid-connect-core-1_0.html)
- [Better Auth Documentation](https://www.better-auth.com/docs)
- [ADR Template](https://github.com/joelparkerhenderson/architecture-decision-record)

---

## Changelog

- **2025-12-02**: Initial decision - Standalone architecture accepted
