# ADR-0002: Authentication Technology Stack

> **Scope**: Core authentication infrastructure including framework, auth library, ORM, and database. These components work together as an integrated solution for handling user authentication, session management, and data persistence.

- **Status:** Accepted
- **Date:** 2025-11-30
- **Feature:** 003-auth-server
- **Context:** RoboLearn platform needs a production-ready authentication system to support multiple client applications (starting with robolearn-interface). Requirements include email/password auth, OAuth 2.1 / OIDC provider capabilities, session management, user profiles, and future SSO expansion. System must handle ~1000 initial users with <100ms session validation, support serverless deployment, and integrate seamlessly with Next.js-based client applications.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: ✅ Long-term authentication architecture for entire platform
     2) Alternatives: ✅ Multiple viable stacks considered (NextAuth, Prisma, traditional databases)
     3) Scope: ✅ Cross-cutting concern affecting all client apps, deployment, security
-->

## Decision

Adopt **Better Auth + Next.js + Drizzle + Neon Postgres** as the integrated authentication stack:

| Component | Choice | Version/Config |
|-----------|--------|----------------|
| **Framework** | Next.js | 14+ with App Router |
| **Auth Library** | Better Auth | Latest with plugins (OIDC Provider, JWT, Admin) |
| **ORM** | Drizzle | Latest with Neon serverless adapter |
| **Database** | Neon Postgres | Serverless, autoscaling |
| **Deployment** | Vercel / Cloud Run | Serverless, zero-config |
| **Session Storage** | HTTP-only cookies | 7-day duration, secure, sameSite |

**Key Integration Points:**
- Better Auth uses Drizzle adapter for database operations
- Next.js App Router provides API routes for Better Auth endpoints
- Neon's serverless driver enables edge deployment compatibility
- HTTP-only cookies managed by Better Auth with Next.js middleware

## Consequences

### Positive

- **Integrated tooling**: Better Auth designed specifically for Next.js, minimal configuration needed
- **Type safety**: End-to-end TypeScript from API routes → ORM → database schema
- **Serverless-ready**: Neon's connection pooling + Drizzle's edge support enable Vercel/Cloud Run deployment
- **OAuth/OIDC built-in**: Better Auth plugins provide full OIDC Provider without custom implementation
- **Schema evolution**: Drizzle migrations give full control over database changes (vs magic ORM migrations)
- **Developer experience**: Better Auth MCP server provides AI-assisted configuration guidance
- **Performance**: <100ms session validation achieved through Neon's edge caching + Better Auth's optimized queries
- **Future-proof**: OIDC Provider architecture supports SSO for multiple client apps

### Negative

- **Library maturity**: Better Auth newer than NextAuth (~1 year old vs 4+ years), smaller ecosystem
- **Migration complexity**: Moving away from Better Auth in future would require significant refactoring
- **Drizzle learning curve**: Less magic than Prisma = more manual query building
- **Neon vendor lock-in**: Serverless Postgres layer is Neon-specific (though standard Postgres underneath)
- **Plugin dependency**: OIDC Provider + JWT + Admin plugins must stay compatible across Better Auth updates
- **Documentation gaps**: Better Auth docs less comprehensive than NextAuth, relies on MCP server assistance
- **Edge limitations**: Some Neon features (full-text search, extensions) unavailable in edge runtime

## Alternatives Considered

### Alternative Stack A: NextAuth + Prisma + Neon
- **Framework**: Next.js 14+
- **Auth**: NextAuth (now Auth.js)
- **ORM**: Prisma
- **Database**: Neon Postgres
- **Rejected Reason**: NextAuth doesn't provide built-in OIDC Provider capabilities (would need custom implementation). Prisma's magic migrations harder to debug in production. Better Auth's plugin architecture better fits our OAuth provider requirements.

### Alternative Stack B: Auth0 + tRPC + PlanetScale
- **Auth**: Auth0 (managed service)
- **API**: tRPC for type-safe endpoints
- **Database**: PlanetScale (MySQL-based)
- **Rejected Reason**: Auth0 costs scale with MAUs ($23/month per 1000 users). Managed service reduces control over auth flows. PlanetScale's MySQL not as feature-rich as Postgres for future needs (JSONB, full-text search). Loss of self-hosted option.

### Alternative Stack C: Supabase Auth + Supabase Database
- **Auth**: Supabase Auth (managed)
- **Database**: Supabase Postgres
- **Rejected Reason**: Couples both auth AND database to single vendor. Supabase's auth primarily designed for their client libraries, harder to integrate as pure OAuth provider. Less flexibility for custom auth flows. Vendor lock-in for critical infrastructure.

### Alternative Stack D: Clerk + Prisma + Railway
- **Auth**: Clerk (managed)
- **ORM**: Prisma
- **Database**: Railway Postgres
- **Rejected Reason**: Clerk pricing ($25/month + $0.02/MAU after 10k users) expensive at scale. Proprietary SDK limits OAuth provider customization. Railway less mature than Vercel for Next.js deployment. Managed auth reduces learning opportunity for team.

## References

- Feature Spec: `specs/003-auth-server/spec.md`
- Implementation Plan: `specs/003-auth-server/plan.md`
- Better Auth Docs: https://www.better-auth.com/docs
- Better Auth MCP Server: Used for configuration guidance
- Drizzle Docs: https://orm.drizzle.team/docs/overview
- Neon Serverless: https://neon.tech/docs/serverless/serverless-driver
- Related ADRs: ADR-0003 (OAuth/OIDC Provider Strategy), ADR-0004 (Standalone Deployment Architecture)
