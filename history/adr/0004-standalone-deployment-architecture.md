# ADR-0004: Standalone Deployment Architecture

> **Scope**: Deployment strategy for auth server including service isolation, CORS configuration, environment management, and deployment platform choices. These decisions affect operational complexity, scalability, and integration patterns.

- **Status:** Accepted
- **Date:** 2025-11-30
- **Feature:** 003-auth-server
- **Context:** RoboLearn platform requires authentication for multiple client applications (robolearn-interface, future author dashboard, institutional deployments). Two architectural options emerged: (1) embed auth in each client app (monolithic), or (2) deploy auth as standalone service (microservice). Standalone approach chosen to enable independent deployment, clear security boundaries, and shared auth across multiple apps. System must handle CORS for cross-origin requests, support serverless deployment (Vercel/Cloud Run), and enable environment-specific configuration without code changes.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: ✅ Defines deployment model for entire platform authentication
     2) Alternatives: ✅ Standalone vs embedded, serverless vs traditional hosting
     3) Scope: ✅ Cross-cutting decision affecting all client apps, operations, scalability
-->

## Decision

Deploy auth server as **Standalone Next.js Application** with the following architecture:

| Component | Implementation | Rationale |
|-----------|----------------|-----------|
| **Service Isolation** | Separate Next.js app in `auth-server/` directory | Independent deployment, clear security boundary, reusable across apps |
| **Deployment Platform** | Vercel (primary) / Cloud Run (alternative) | Serverless, zero-config, auto-scaling, global edge network |
| **CORS Configuration** | Environment-based `trustedOrigins` allowlist | Supports multiple client domains without code changes |
| **Session Management** | HTTP-only cookies with domain configuration | Secure, browser-managed, supports subdomains |
| **Environment Variables** | `.env.local` (dev) + Vercel dashboard (prod) | Secret management, environment-specific config |
| **Database Connection** | Neon serverless driver with connection pooling | Handles serverless cold starts, auto-scaling |
| **API Endpoints** | `/api/auth/*` (Better Auth) + `/api/profile` (custom) | Standardized auth endpoints + domain-specific extensions |

**Deployment Flow:**
```
Development:
auth-server/ → npm run dev → http://localhost:3001
robolearn-interface/ → npm run dev → http://localhost:3000
CORS: localhost:3000 in ALLOWED_ORIGINS

Production:
auth-server/ → Vercel → https://auth.robolearn.com
robolearn-interface/ → GitHub Pages → https://robolearn.com
CORS: https://robolearn.com in ALLOWED_ORIGINS
```

**Key Configuration Points:**
- **CORS**: `trustedOrigins: process.env.ALLOWED_ORIGINS?.split(",")`
- **Cookies**: `domain` NOT set → cookies scoped to auth server only (OAuth flow)
- **Environment**: `BETTER_AUTH_URL` must match deployed auth server URL
- **Database**: `DATABASE_URL` connects to Neon serverless (shared across environments)

## Consequences

### Positive

- **Independent deployment**: Auth server updates don't require redeploying client apps
- **Clear security boundary**: Auth logic isolated from book content, admin tools, etc.
- **Multi-client reuse**: Single auth server serves robolearn-interface, author dashboard, admin panel
- **Simplified scaling**: Vercel auto-scales auth server based on traffic (no manual server management)
- **Zero downtime updates**: Vercel atomic deploys → new version live instantly, old version terminates gracefully
- **Global performance**: Vercel edge network → auth endpoints served from nearest region
- **Environment parity**: Same Next.js app runs locally and in production (fewer environment bugs)
- **Secret isolation**: Each deployment environment has separate environment variables (dev vs prod secrets isolated)

### Negative

- **CORS complexity**: Must configure and test CORS for every new client app
- **Cookie domain limitations**: Auth cookies don't work across different root domains (auth.robolearn.com ≠ robolearn.com)
- **Operational overhead**: Two services to monitor, deploy, and maintain instead of one
- **Network latency**: Cross-origin requests add ~50-100ms latency (mitigated by JWKS offline verification)
- **Deployment coordination**: Changes affecting both auth server AND client require coordinated deploys
- **Local development setup**: Developers must run both auth-server and client app locally
- **Error debugging**: Distributed system → harder to trace errors across services
- **Vercel vendor lock-in**: Tightly coupled to Vercel platform features (though Cloud Run alternative exists)

## Alternatives Considered

### Alternative A: Embedded Auth in Each Client App
- **Architecture**: Each app (robolearn-interface, author dashboard) has its own `/api/auth` routes
- **Database**: Shared Neon database, each app connects independently
- **Session sharing**: Shared session table, apps read each other's sessions
- **Rejected Reason**: Code duplication across apps, no single source of truth for auth logic, harder to maintain consistency, session sharing fragile (table schema changes break all apps).

### Alternative B: Monolithic Next.js App (All-in-One)
- **Architecture**: Single Next.js app with `/book`, `/admin`, `/auth` routes
- **Deployment**: One Vercel deployment for entire platform
- **Session sharing**: Internal routing, no CORS needed
- **Rejected Reason**: Violates separation of concerns, admin tools and auth in same codebase as public book content (security risk), single point of failure, slower deploys (entire platform redeploys for any change).

### Alternative C: Traditional Server Deployment (EC2, DigitalOcean)
- **Architecture**: Standalone auth server on traditional VPS
- **Deployment**: Manual deployment with PM2/systemd
- **Scaling**: Manual server provisioning
- **Rejected Reason**: Operational complexity (server management, load balancers, uptime monitoring), slower deploys, no auto-scaling, higher costs at low traffic (paying for idle capacity).

### Alternative D: Auth Server + API Gateway (Kong, Traefik)
- **Architecture**: Auth server behind API gateway
- **Gateway**: Handles CORS, rate limiting, routing
- **Deployment**: Separate gateway + auth server deployments
- **Rejected Reason**: Over-engineered for current scale (~1000 users), API gateway adds complexity without clear benefit, Vercel edge network already provides global routing and rate limiting.

## References

- Feature Spec: `specs/003-auth-server/spec.md` (see NFR-001: Independent deployment)
- Implementation Plan: `specs/003-auth-server/plan.md` (see "Structure Decision" section, line 86)
- Next.js Deployment Docs: https://nextjs.org/docs/deployment
- Vercel Platform: https://vercel.com/docs
- Cloud Run Alternative: https://cloud.google.com/run/docs
- CORS Best Practices: https://developer.mozilla.org/en-US/docs/Web/HTTP/CORS
- Related ADRs: ADR-0002 (Authentication Technology Stack), ADR-0003 (OAuth/OIDC Provider Strategy)
- Environment Config: `auth-server/.env.example` (see ALLOWED_ORIGINS, BETTER_AUTH_URL)
