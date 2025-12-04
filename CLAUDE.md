# CLAUDE.md — Panaversity SSO Platform

This file provides guidance to Claude Code (claude.ai/code) when working with this codebase.

## Project Overview

**Panaversity SSO** is a centralized OAuth 2.1 / OIDC authentication server serving multiple Panaversity applications. Built with Better Auth, Next.js 15, Drizzle ORM, and Neon Postgres.

### Key Capabilities
- OAuth 2.1 Authorization Code Flow with PKCE
- OIDC Provider with JWKS (RS256 signing)
- Multi-tenancy with organizations
- Role-based access control (admin/user + org roles)
- Custom JWT claims (software_background, hardware_tier, tenant_id)
- Rate limiting and security hardening

### Platform Clients
- **RoboLearn** - Educational platform (public client, PKCE)
- **AI Native** - AI development platform (public client, PKCE)
- **Panaversity SSO Dashboard** - Admin interface (first-party)

## Common Commands

```bash
# Development
pnpm dev                    # Start dev server on port 3001

# Database
pnpm db:push               # Push schema to database
pnpm db:generate           # Generate migration files
pnpm db:studio             # Open Drizzle Studio

# Seeding
pnpm seed:setup            # Development: all clients + test org
pnpm seed:prod             # Production: Panaversity + AI Native only

# Testing
pnpm test-api              # API tests (~60s)
pnpm test-e2e              # Playwright visual tests (~30s)
pnpm test-all              # Complete suite (~90s)
pnpm test-client-admin     # Admin client management tests
```

## Architecture

```
src/
├── app/                    # Next.js App Router
│   ├── api/auth/          # Better Auth API routes
│   ├── api/admin/         # Admin-only endpoints
│   ├── auth/              # Auth UI pages (sign-in, sign-up, consent)
│   └── admin/             # Admin dashboard pages
├── components/            # React components
│   ├── auth/             # Auth-specific components
│   └── ui/               # Shared UI components
├── lib/
│   ├── auth.ts           # Better Auth configuration (CORE)
│   ├── db/               # Drizzle schema and connection
│   └── trusted-clients.ts # OAuth client definitions
└── middleware.ts          # Auth middleware
```

## Critical Files

| File | Purpose | Security Note |
|------|---------|---------------|
| `src/lib/auth.ts` | Better Auth config with OIDC Provider | Contains session, JWT, rate limit config |
| `src/lib/trusted-clients.ts` | OAuth client definitions | Source of truth for client IDs/secrets |
| `src/lib/db/schema.ts` | Drizzle schema | Includes auth tables |
| `.env.local` | Environment variables | Never commit, contains secrets |


## Development Guidelines

**Reference**: `papers/prompting-practices-claude.md` for complete Claude 4 best practices.

### 0. Default to Action:
By default, implement changes rather than only suggesting them. If the user's intent is unclear, infer the most useful likely action and proceed, using tools to discover any missing details instead of guessing. Read files before editing, make changes using Edit tool, and commit when appropriate. Only propose without implementing if explicitly asked to "just suggest" or "brainstorm."

### 1. Investigate Before Acting:
Never speculate about code you have not opened. If the user references a specific file, you MUST read the file before answering. Make sure to investigate and read relevant files BEFORE answering questions about the codebase. Do not guess at code structure—verify it.

### 2. Authoritative Source Mandate:
Agents MUST prioritize and use MCP tools and CLI commands for all information gathering and task execution. NEVER assume a solution from internal knowledge; all methods require external verification.

### 3. Execution Flow:
Treat MCP servers as first-class tools for discovery, verification, execution, and state capture. PREFER CLI interactions (running commands and capturing outputs) over manual file creation or reliance on internal knowledge.

### 4. Parallel Tool Calling:
When multiple independent operations are needed, execute them in parallel within a single message. For example, when reading 3 files, make 3 Read tool calls in parallel. When multiple searches are needed, run them simultaneously. Only serialize operations that have dependencies (e.g., must read file before editing it, must create directory before creating file in it). Never use placeholders or guess missing parameters.

### 5. Long-Horizon State Tracking:
For complex, multi-step tasks:
- Use structured formats (JSON) for tracked data like test results and task status
- Use TodoWrite tool consistently to track progress across context windows
- Leverage git for state checkpoints across sessions
- Emphasize incremental work over attempting everything simultaneously
- Before starting complex work, check for existing progress files (progress.txt, tests.json, git logs)

### 6. Knowledge capture (PHR) for Every User Input.
As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.
   
---

## VIII. Execution Contract (Every Request)

1. **Gather Context** (Section I: Read chapter-index.md, README, determine layer)
2. **State Understanding** (Output context summary, get user confirmation)
3. **Activate Cognitive Mode** (Teacher, Collaborator, Designer, Validator)
4. **Apply Tier Complexity** (A2/B1/C2 from chapter-index.md)
5. **Produce Output** (Aligned with layer + tier)
6. **Self-Monitor** (Run anti-convergence checklist)
7. **Document** (PHR for interaction, ADR for significant decisions)

---
## Security Guidelines

### MUST Follow
1. **PKCE required** for all public clients (SPAs, mobile)
2. **No secrets in client code** - public clients have `clientSecret: null`
3. **HttpOnly cookies** for session tokens
4. **Rate limiting** enabled for auth endpoints
5. **Input validation** at all API boundaries

### NEVER Do
1. Log passwords, tokens, or secrets (even in debug)
2. Include secrets in URLs (use POST body)
3. Allow HTTP in production
4. Trust client-provided tenant_id without verification
5. Skip PKCE validation for any public client

## OAuth Flow Reference

### Public Client (PKCE)
```
1. Client generates code_verifier + code_challenge
2. GET /api/auth/oauth2/authorize?...&code_challenge=X
3. User authenticates at SSO
4. SSO redirects with ?code=Y
5. POST /api/auth/oauth2/token with code + code_verifier
6. SSO returns access_token, id_token, refresh_token
```

### Confidential Client
```
1. GET /api/auth/oauth2/authorize?...
2. User authenticates at SSO
3. SSO redirects with ?code=Y
4. POST /api/auth/oauth2/token with code + client_secret
5. SSO returns access_token, id_token, refresh_token
```

## Key Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/.well-known/openid-configuration` | GET | OIDC Discovery |
| `/api/auth/jwks` | GET | Public keys for JWT verification |
| `/api/auth/oauth2/authorize` | GET | Start OAuth flow |
| `/api/auth/oauth2/token` | POST | Exchange code for tokens |
| `/api/auth/oauth2/userinfo` | GET | Get user info (Bearer token) |
| `/api/admin/clients/register` | POST | Register OAuth client (admin) |

## Testing Approach

### Test Suites
1. **API Tests** (`test-api`): OAuth flows, JWT claims, edge cases
2. **E2E Tests** (`test-e2e`): Browser-based flows with Playwright
3. **Admin Tests** (`test-client-admin`): Client CRUD operations

### Before Committing
```bash
pnpm test-api              # Must pass
pnpm lint                  # Must pass
```

## Environment Variables

Required in `.env.local`:
```env
DATABASE_URL=              # Neon Postgres connection string
BETTER_AUTH_SECRET=        # 32+ char secret (openssl rand -base64 32)
BETTER_AUTH_URL=           # http://localhost:3001 (dev)
ALLOWED_ORIGINS=           # Comma-separated allowed CORS origins
```

Optional:
```env
RESEND_API_KEY=           # For email verification
SMTP_HOST=                # Alternative to Resend
```

## SDD-RI Integration

This project uses Spec-Driven Development with Reusable Intelligence.

### Key Artifacts
- **Constitution**: `.specify/memory/constitution.md` - Platform governance
- **Agents**: `.claude/agents/` - Specialized AI agents
- **Skills**: `.claude/skills/engineering/` - Reusable knowledge
- **Specs**: `specs/` - Feature specifications
- **ADRs**: `history/adr/` - Architecture Decision Records

### Workflow Commands
- `/sp.specify` - Create feature specification
- `/sp.plan` - Create implementation plan
- `/sp.tasks` - Generate task breakdown
- `/sp.implement` - Execute implementation
- `/sp.analyze` - Cross-artifact analysis

### Available Skills
| Skill | When to Use |
|-------|-------------|
| `better-auth-setup` | OAuth/OIDC implementation guidance |
| `frontend-design` | UI component design |
| `skill-creator` | Creating new skills |
| `session-intelligence-harvester` | Capturing session learnings |

## Common Issues & Solutions

### "code verification failed"
- PKCE params lost during sign-in redirect
- Fix: Ensure sign-in form preserves code_challenge in redirect

### "Cannot read properties of undefined (reading 'find')"
- Wrong property name: `redirectURLs` vs `redirectUrls`
- Fix: Use lowercase `redirectUrls` in Better Auth config

### CORS errors
- Missing origin in `ALLOWED_ORIGINS`
- Fix: Add client origin to comma-separated list

### "Unauthorized" on admin endpoints
- User lacks admin role
- Fix: `UPDATE "user" SET role = 'admin' WHERE email = '...'`

## Documentation

| Doc | Purpose |
|-----|---------|
| `docs/integration-guide.md` | Complete backend integration guide |
| `docs/pkce-flow.md` | PKCE OAuth flow details |
| `docs/jwt-jwks.md` | Token signing and verification |
| `docs/multi-tenancy.md` | Organization architecture |
| `docs/troubleshooting.md` | Common issues and solutions |

## Contributing

1. Read the constitution (`.specify/memory/constitution.md`)
2. Create feature spec with `/sp.specify`
3. Get spec reviewed before implementation
4. Follow security guidelines above
5. Ensure all tests pass before PR
