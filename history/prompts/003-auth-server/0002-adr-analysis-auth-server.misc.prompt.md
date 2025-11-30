---
id: 0002
title: adr-analysis-auth-server
stage: misc
date: 2025-11-30T00:00:00Z
surface: Claude Code
model: claude-sonnet-4-5-20250929
feature: 003-auth-server
branch: auth-server
user: mjs
command: /sp.adr
labels: [adr, architecture, auth-server, better-auth, oauth]
links:
  spec: specs/003-auth-server/spec.md
  plan: specs/003-auth-server/plan.md
  adr: history/adr/0002-authentication-technology-stack.md, history/adr/0003-oauth-oidc-provider-strategy.md, history/adr/0004-standalone-deployment-architecture.md
  ticket: N/A
  pr: N/A
files:
  - path: history/adr/0002-authentication-technology-stack.md
    action: created
    lines: 109 lines
  - path: history/adr/0003-oauth-oidc-provider-strategy.md
    action: created
    lines: 110 lines
  - path: history/adr/0004-standalone-deployment-architecture.md
    action: created
    lines: 109 lines
tests: []
---

## Prompt

/sp.adr

## Response snapshot

**Decision Extraction** (Analyze):
Reviewed `specs/003-auth-server/plan.md` and identified 3 architecturally significant decision clusters:

1. **Authentication Technology Stack**: Better Auth + Next.js + Drizzle + Neon Postgres (integrated solution for auth infrastructure)
2. **OAuth/OIDC Provider Strategy**: Full OAuth 2.1 / OIDC with PKCE + JWKS (RS256) for multi-app SSO
3. **Standalone Deployment Architecture**: Separate Next.js app with Vercel/Cloud Run deployment

**Significance Test** (Measure):
Applied checklist grader to each cluster - all 3 PASSED:
- ✅ Impacts how engineers write/structure software (auth patterns, token flows, deployment)
- ✅ Notable tradeoffs (Better Auth vs NextAuth, Full OIDC vs simple auth, Standalone vs monolithic)
- ✅ Will be questioned later (library maturity, complexity trade-offs, operational overhead)

**ADR Creation** (Improve):
Created 3 ADRs with comprehensive documentation:

**ADR-0002: Authentication Technology Stack**
- **Decision**: Better Auth + Next.js 14+ + Drizzle + Neon Postgres
- **Alternatives**: 4 stacks considered (NextAuth + Prisma, Auth0, Supabase, Clerk)
- **Key Tradeoff**: Library maturity vs built-in OIDC Provider capabilities
- **Consequence**: Type-safe, serverless-ready, but tightly coupled to Better Auth plugins

**ADR-0003: OAuth/OIDC Provider Strategy**
- **Decision**: Full OAuth 2.1 / OIDC with PKCE (public clients) + JWKS (RS256 signing)
- **Alternatives**: 4 approaches considered (Simple session-based, Shared secrets, Managed providers, Custom JWT)
- **Key Tradeoff**: Complexity vs scalability (JWKS enables 10,000+ users per app via offline verification)
- **Consequence**: Multi-app SSO enabled, but OAuth flows harder to debug than simple auth

**ADR-0004: Standalone Deployment Architecture**
- **Decision**: Separate Next.js app deployed to Vercel/Cloud Run
- **Alternatives**: 4 architectures considered (Embedded auth, Monolithic app, Traditional VPS, API gateway)
- **Key Tradeoff**: Independent deployment vs operational overhead (2 services to maintain)
- **Consequence**: Clear security boundaries, but CORS complexity and distributed debugging

**Grader Results**:
- ✅ All ADRs document decision clusters (not atomic choices)
- ✅ All ADRs list explicit alternatives with rejection reasons
- ✅ All ADRs include pros AND cons for chosen approach
- ✅ All ADRs concise but detailed (109-110 lines each)

## Outcome

- ✅ Impact: Permanent record of architectural decisions, enables future team members to understand "why" behind auth architecture
- 🧪 Tests: No tests added (documentation task)
- 📁 Files: 3 ADRs created (history/adr/0002-0004)
- 🔁 Next prompts: /sp.tasks to break down implementation into actionable tasks
- 🧠 Reflection: Decision clustering worked well - grouping Better Auth + Drizzle + Neon as single "stack" ADR avoided over-granular documentation. OAuth/OIDC ADR captures complexity trade-off that might be questioned later ("why not use Auth0?"). Standalone deployment ADR documents operational implications upfront.

## Evaluation notes (flywheel)

- Failure modes observed: None - all 3 ADRs passed significance test and grader checklist
- Graders run and results (PASS/FAIL):
  - Decision clustering: PASS (grouped related technologies, not atomic)
  - Alternatives documented: PASS (4 alternatives per ADR with rejection reasons)
  - Consequences balanced: PASS (pros AND cons listed for each decision)
  - Conciseness: PASS (109-110 lines, detailed but not verbose)
- Prompt variant (if applicable): Standard /sp.adr workflow
- Next experiment (smallest change to try): Consider adding "Migration Path" section to ADRs documenting how to move away from decision if needed (e.g., "How to migrate from Better Auth to Auth0 if library becomes unmaintained")
