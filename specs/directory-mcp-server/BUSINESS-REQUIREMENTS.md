# Directory MCP Server: Complete Business Requirements Analysis

**Status**: Comprehensive Business Analysis
**Created**: 2025-11-21
**Purpose**: Define the COMPLETE end-to-end business requirements from content creation → versioning → publishing → consumption

---

## Executive Summary

**The Core Business Problem**:
We have 84 chapters of educational content currently locked in a Git filesystem. We need to make this Reusable Intelligence (RI) truly extensible and reusable by abstracting storage behind an MCP server with pluggable backends.

**Current Pain Points**:

1. ❌ Content is tightly coupled to local Git filesystem
2. ❌ AI agents have friction accessing/modifying content (clone, parse, push)
3. ❌ Cannot switch storage backends (local → R2 → S3) without code changes
4. ❌ RI (skills, agents, specs) cannot be reused across different books/projects
5. ❌ No structured content API - everything is file-based

**Business Goals**:

1. ✅ Decouple content from storage layer
2. ✅ Enable agents to access content via standardized MCP protocol
3. ✅ Support multiple storage backends (local, R2, S3, Azure)
4. ✅ Make RI reusable across projects (books, docs, courses)
5. ✅ Maintain existing Docusaurus publishing workflow
6. ✅ **Deploy TODAY** - simplest viable implementation

---

## Part 1: The Complete Book Lifecycle

### Current Workflow (As-Is)

```
┌─────────────────────────────────────────────────────────────┐
│  Content Creation (AI Agents + Humans)                      │
│  ├─ /sp.loopflow.v2 → spec.md                              │
│  ├─ /sp.plan → plan.md                                     │
│  ├─ /sp.tasks → tasks.md                                   │
│  └─ /sp.implement → lesson files (.md)                     │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  Storage (Local Git Filesystem)                             │
│  apps/learn-app/docs/                                          │
│  ├── 01-Introducing-AI-Driven-Development/                 │
│  │   ├── README.md                                          │
│  │   └── 01-chapter-name/                                   │
│  │       ├── README.md                                      │
│  │       ├── 01-lesson-1.md                                 │
│  │       └── 02-lesson-2.md                                 │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  Versioning (Git)                                           │
│  ├─ git add .                                               │
│  ├─ git commit                                              │
│  └─ git push                                                │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  Build (Docusaurus)                                         │
│  ├─ Reads apps/learn-app/docs/ filesystem                     │
│  ├─ Generates static site                                  │
│  └─ Outputs to build/                                       │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  Publishing (GitHub Pages)                                  │
│  ├─ GitHub Actions (.github/workflows/deploy.yml)          │
│  ├─ Runs npm run build                                     │
│  └─ Deploys to GitHub Pages                                │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  Consumption (Students)                                     │
│  ├─ Website: https://panaversity.com                       │
│  └─ Students read lessons                                  │
└─────────────────────────────────────────────────────────────┘
```

### Problems in Current Workflow

**1. Agent Access (Content Creation)**

- Agents must manipulate filesystem directly
- No semantic API (agents see files, not "lessons" or "chapters")
- No transactional writes (race conditions possible)
- No audit trail (who changed what, when, why)

**2. Storage Layer**

- Tightly coupled to local filesystem
- Cannot store in R2/S3 without complete rewrite
- No abstraction layer
- No content indexing (must scan filesystem for queries)

**3. Versioning**

- Git is heavyweight for content changes
- Binary assets (images) bloat repository
- Merge conflicts on markdown files
- Git history becomes noise (every agent write = commit)

**4. Build Process**

- Docusaurus assumes local filesystem
- Cannot fetch content from remote storage
- Must clone entire repo for CI builds
- No incremental builds (rebuilds everything)

**5. Publishing**

- Coupled to GitHub Pages via filesystem
- Cannot publish from R2/S3 directly
- No content preview before deployment
- No rollback mechanism

**6. Consumption**

- Static site only (no dynamic content)
- No personalized views
- No content search beyond static index
- No analytics on content usage

---

## Part 2: End-to-End Business Requirements

### Requirement 1: Agent-Native Content Access

**User Story**: As an AI agent, I need to read/write book content through a semantic API, not filesystem manipulation.

**Acceptance Criteria**:

- [ ] Agent can read lesson by semantic ID (e.g., `chapter-5/lesson-2`)
- [ ] Agent can write lesson content with metadata (author, timestamp)
- [ ] Agent can query: "Get all lessons in Chapter 5"
- [ ] Agent can query: "Find lessons mentioning 'async/await'"
- [ ] All operations logged with agent_id, timestamp, operation type

**Non-Functional Requirements**:

- Response time: < 100ms for reads
- Concurrent writes: Support 5 agents writing simultaneously
- Audit trail: Every operation logged, queryable

---

### Requirement 2: Storage Backend Abstraction

**User Story**: As a platform operator, I need to switch storage backends (local → R2 → S3) without code changes.

**Acceptance Criteria**:

- [ ] Support local filesystem (development)
- [ ] Support Cloudflare R2 (production)
- [ ] Support AWS S3 (alternative)
- [ ] Configuration-driven (change env vars, not code)
- [ ] Zero downtime migration (old → new backend)

**Non-Functional Requirements**:

- Backend change: 0 code changes, only config
- Migration: < 1 hour for 84 chapters
- Cost: Minimize egress fees (prefer R2)

---

### Requirement 3: Reusable Intelligence Across Projects

**User Story**: As a content architect, I need to reuse the same RI (skills, agents, specs) across multiple books/projects.

**Acceptance Criteria**:

- [ ] "Book" is abstracted to "Directory" (generic organized content)
- [ ] Same MCP server works for: Books, Docs, Courses, Knowledge Bases
- [ ] Schema-driven (define structure per directory type)
- [ ] Multi-tenancy (serve multiple directories simultaneously)

**Examples**:

- Book: 84 chapters (Parts → Chapters → Lessons)
- Docs: API documentation (Sections → Pages)
- Course: Learning path (Modules → Units → Activities)
- Knowledge: Internal wiki (Topics → Articles)

---

### Requirement 4: Versioning & Change Management

**User Story**: As a content manager, I need to track changes, revert mistakes, and audit who changed what.

**Acceptance Criteria**:

- [ ] Every content write creates audit log entry
- [ ] Audit log includes: agent_id, timestamp, operation, content_id, diff
- [ ] Query audit log: "Show all changes by agent-X in last 24h"
- [ ] Rollback capability: Restore lesson to previous version
- [ ] Diff view: Compare current vs previous version

**Non-Functional Requirements**:

- Audit retention: 90 days minimum
- Rollback: < 5 seconds to restore
- Query performance: < 200ms for audit queries

---

### Requirement 5: Docusaurus Integration (Zero Breaking Changes)

**User Story**: As a developer, I need Docusaurus to continue working without breaking changes.

**Acceptance Criteria**:

- [ ] Docusaurus build process unchanged
- [ ] `npm run build` still works
- [ ] GitHub Actions deployment still works
- [ ] File structure appears identical to Docusaurus
- [ ] No changes to existing lesson .md files

**Implementation Approach**:

- Hydration layer: Fetch content from MCP → Write to local temp dir → Docusaurus reads temp dir
- OR: Custom Docusaurus plugin that reads from MCP directly

---

### Requirement 6: Publishing & Deployment

**User Story**: As a publisher, I need to deploy updated content with preview, rollback, and zero downtime.

**Acceptance Criteria**:

- [ ] Preview builds: View changes before production deployment
- [ ] One-click deployment from any storage backend
- [ ] Rollback: Restore to previous deployment
- [ ] Zero downtime: New version goes live without outage
- [ ] Cache invalidation: CDN clears cache on deployment

---

### Requirement 7: Content Search & Discovery

**User Story**: As a student, I need to search across all lessons and find relevant content quickly.

**Acceptance Criteria**:

- [ ] Full-text search across all lessons
- [ ] Metadata search (filter by chapter, proficiency level, skills)
- [ ] Vector search (semantic similarity)
- [ ] Search results ranked by relevance

---

### Requirement 8: Performance & Scale

**User Story**: As a platform operator, I need the system to handle 1000+ lessons and 50+ concurrent agents.

**Acceptance Criteria**:

- [ ] Read latency: < 100ms (p95)
- [ ] Write latency: < 500ms (p95)
- [ ] Concurrent reads: 100/sec
- [ ] Concurrent writes: 10/sec
- [ ] Storage: Support 10GB+ content

---

### Requirement 9: Binary Asset Management

**User Story**: As a content creator, I need to upload images/videos and reference them in lessons.

**Acceptance Criteria**:

- [ ] Upload binary assets (images, PDFs, videos)
- [ ] Auto-generate CDN URLs
- [ ] Image optimization (resize, compress, WebP conversion)
- [ ] Asset versioning (track changes to images)
- [ ] Orphan detection (find unused images)

---

### Requirement 10: Multi-Consumer Support

**User Story**: As a platform architect, I need to serve content to multiple consumers (Docusaurus, mobile app, LMS).

**Acceptance Criteria**:

- [ ] MCP interface (for AI agents)
- [ ] REST API (for web apps)
- [ ] CLI (for developers)
- [ ] GraphQL API (for complex queries) - Future
- [ ] WebSocket (for real-time updates) - Future

---

## Part 3: Edge Cases & Failure Scenarios

### Edge Case 1: Concurrent Writes (Race Conditions)

**Scenario**: Agent A and Agent B both write to `chapter-5/lesson-2.md` simultaneously.

**Current Behavior**: Last write wins, one agent's changes lost.

**Required Behavior**:

- Option 1: Optimistic locking (version check, fail if stale)
- Option 2: Last-write-wins with conflict log
- Option 3: Merge strategies (auto-merge non-conflicting sections)

**Decision**: Option 1 (Optimistic locking) - Simplest, safest.

---

### Edge Case 2: Partial Write Failure

**Scenario**: Agent writes lesson content, but metadata update fails.

**Current Behavior**: Inconsistent state (content updated, metadata stale).

**Required Behavior**: Transactional writes - either both succeed or both fail.

**Implementation**: Atomic writes with rollback on failure.

---

### Edge Case 3: Storage Backend Failure

**Scenario**: R2 is down, cannot read content.

**Current Behavior**: Build fails, site goes down.

**Required Behavior**:

- Fallback to cached version
- Serve stale content with warning banner
- Alert operator of backend failure

**Implementation**: Local cache + health checks + fallback logic.

---

### Edge Case 4: Large Binary Assets (Videos)

**Scenario**: Lesson includes 500MB video file.

**Current Behavior**: Git repository bloats, slow clones.

**Required Behavior**:

- Store videos in R2, not Git
- Serve via CDN
- Streaming support

**Implementation**: Separate asset storage from content storage.

---

### Edge Case 5: Content Migration (Existing 84 Chapters)

**Scenario**: We have 84 existing chapters in Git filesystem.

**Required Behavior**:

- Migrate all 84 chapters to new storage without data loss
- Preserve Git history (keep as archive)
- Zero downtime during migration

**Implementation**: Migration script + dual-write period + cutover.

---

### Edge Case 6: Schema Evolution

**Scenario**: We add new field to lesson schema (e.g., `estimated_time`).

**Current Behavior**: Must update all 84 chapters manually.

**Required Behavior**:

- Schema versioning
- Default values for new fields
- Backward compatibility (old lessons still work)

**Implementation**: Schema version in metadata + migration scripts.

---

## Part 4: Existing Research Summary

We have 3 architectural proposals from AI researchers:

### Proposal 1: Headless Book Architecture (headless-book-arch.md)

**Key Ideas**:

- Spec-Driven Storage System (book.yaml defines structure)
- OpenDAL for storage abstraction
- Vertical Intelligence (watcher agents, auto-summaries)
- LanceDB for vector storage
- Docusaurus hydration from R2

**Strengths**:

- Comprehensive, production-grade architecture
- Addresses vertical intelligence (auto-summaries)
- Strong separation of concerns

**Weaknesses**:

- Complex (watcher agents, LanceDB, RAG)
- Long implementation timeline (months)
- Over-engineered for MVP

---

### Proposal 2: PanaversityFS (opendal+agentfs.md)

**Key Ideas**:

- OpenDAL + AgentFS patterns
- Python implementation
- Audit logging with agent tracking
- MCP server with educational tools
- CLI for Docusaurus sync

**Strengths**:

- Pragmatic, focused on actual use case
- AgentFS audit patterns (every operation logged)
- Python-first (matches existing agents)

**Weaknesses**:

- Python may be slower than TypeScript for MCP server
- Less emphasis on multi-consumer support
- Tightly coupled to book domain

---

### Proposal 3: Build vs Buy Analysis (compass_artifact)

**Key Ideas**:

- Evaluate existing solutions (AgentFS, MCP servers, storage libs)
- Recommendation: Buy storage abstraction, build thin MCP wrapper
- Flystorage vs @tweedegolf vs OpenDAL comparison

**Strengths**:

- Practical evaluation of ecosystem
- Identifies reusable components
- Cost-benefit analysis

**Weaknesses**:

- Less prescriptive on final architecture
- No implementation details
- Missing deployment strategy

---

## Part 5: Simplest Viable Implementation (Deploy TODAY)

**Principle**: Start with the simplest thing that solves the core problem, iterate later.

### MVP Scope (Deploy in 1 Day)

**In Scope**:

1. ✅ Storage abstraction (local + R2)
2. ✅ MCP server with basic tools (read, write, list)
3. ✅ Docusaurus hydration script (fetch from MCP → temp dir)
4. ✅ Audit logging (simple append-only log)
5. ✅ Migration script (Git → R2)

**Out of Scope (Phase 2)**:

1. ❌ Vertical Intelligence (watcher agents, summaries)
2. ❌ LanceDB vector storage
3. ❌ Full-text search
4. ❌ GraphQL API
5. ❌ Image optimization
6. ❌ Versioning beyond audit log

### Architecture (MVP)

```
┌─────────────────────────────────────────────────────────────┐
│  Consumers                                                   │
│  ├─ AI Agents (via MCP Client)                              │
│  └─ Docusaurus (via Hydration Script)                       │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  MCP Server (TypeScript)                                    │
│  Tools:                                                      │
│  ├─ read_content(path)                                      │
│  ├─ write_content(path, content, agent_id)                 │
│  ├─ list_contents(directory)                               │
│  ├─ search_content(query) - simple grep                    │
│  └─ get_audit_log(filters)                                 │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  Storage Abstraction Layer                                  │
│  Interface: read(), write(), list(), exists()               │
│  Implementations:                                            │
│  ├─ LocalStorage (development)                              │
│  └─ R2Storage (production)                                  │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  Physical Storage                                           │
│  ├─ Local: ./content/                                       │
│  └─ R2: s3://panaversity-book/                             │
└─────────────────────────────────────────────────────────────┘
```

### File Structure (MVP)

```
directory-mcp-server/
├── package.json
├── tsconfig.json
├── src/
│   ├── index.ts                  # MCP server entry point
│   ├── storage/
│   │   ├── interface.ts          # StorageBackend interface
│   │   ├── local.ts              # LocalStorage implementation
│   │   └── r2.ts                 # R2Storage implementation
│   ├── mcp/
│   │   ├── server.ts             # MCP protocol handler
│   │   └── tools.ts              # Tool definitions
│   └── audit/
│       └── logger.ts             # Simple append-only audit log
├── scripts/
│   ├── migrate-to-r2.ts          # Migration: Git → R2
│   └── hydrate-docusaurus.ts     # Fetch content → temp dir
└── config.json                   # Backend configuration
```

### Configuration (config.json)

```json
{
  "storage": {
    "backend": "r2",
    "local": {
      "path": "./content"
    },
    "r2": {
      "accountId": "xxx",
      "accessKeyId": "xxx",
      "secretAccessKey": "xxx",
      "bucket": "panaversity-book"
    }
  },
  "audit": {
    "enabled": true,
    "logFile": "audit.jsonl"
  }
}
```

### Implementation Steps (1 Day)

**Hour 1-2: Setup**

- [ ] Initialize TypeScript project
- [ ] Install dependencies (@modelcontextprotocol/sdk, @aws-sdk/client-s3)
- [ ] Define StorageBackend interface

**Hour 3-4: Storage Layer**

- [ ] Implement LocalStorage
- [ ] Implement R2Storage
- [ ] Write unit tests

**Hour 5-6: MCP Server**

- [ ] Implement MCP server with tools
- [ ] Test with MCP Inspector

**Hour 7-8: Migration & Hydration**

- [ ] Write migration script (Git → R2)
- [ ] Write Docusaurus hydration script
- [ ] Test end-to-end

**Hour 9: Deployment**

- [ ] Run migration for 84 chapters
- [ ] Update GitHub Actions to use hydration script
- [ ] Deploy

---

## Part 6: Future Phases (Post-MVP)

### Phase 2: Intelligence Layer (Week 2-3)

- [ ] Directory schemas (book, docs, course)
- [ ] Content indexing (in-memory search)
- [ ] Metadata extraction
- [ ] Relationship mapping (chapter → lessons)

### Phase 3: Advanced Features (Month 2)

- [ ] LanceDB vector storage
- [ ] Semantic search
- [ ] Watcher agents (auto-summaries)
- [ ] Image optimization

### Phase 4: Multi-Consumer (Month 3)

- [ ] REST API
- [ ] GraphQL API
- [ ] Mobile SDK
- [ ] LMS integration

---

## Part 7: Decision Framework

### Key Decisions Required

**Decision 1: Storage Abstraction Library**

- Option A: Build minimal wrapper around AWS SDK
- Option B: Use OpenDAL (Rust-based, multi-language)
- Option C: Use @tweedegolf/storage-abstraction (TypeScript-native)

**Recommendation**: Option A for MVP (minimal dependencies), migrate to OpenDAL in Phase 2.

---

**Decision 2: MCP Server Language**

- Option A: TypeScript (aligns with existing tooling)
- Option B: Python (matches agent code)

**Recommendation**: TypeScript (better MCP SDK support, faster execution).

---

**Decision 3: Audit Logging**

- Option A: Append-only JSONL file
- Option B: SQLite database (AgentFS pattern)
- Option C: External service (Cloudflare D1)

**Recommendation**: Option A for MVP, Option B for Phase 2.

---

**Decision 4: Docusaurus Integration**

- Option A: Hydration script (pre-build fetch)
- Option B: Custom Docusaurus plugin (runtime fetch)

**Recommendation**: Option A (simpler, no Docusaurus code changes).

---

**Decision 5: Migration Strategy**

- Option A: Big bang (migrate all 84 chapters at once)
- Option B: Gradual (dual-write, chapter-by-chapter)

**Recommendation**: Option A (84 chapters is small enough for atomic migration).

---

## Part 8: Success Criteria

**MVP Success (Day 1)**:

- [ ] 84 chapters migrated to R2
- [ ] MCP server running and accessible
- [ ] Agents can read/write via MCP
- [ ] Docusaurus build works with hydration
- [ ] Website deploys successfully
- [ ] Audit log captures all operations

**Phase 2 Success (Week 3)**:

- [ ] Directory schemas operational
- [ ] Content search working
- [ ] Multiple directory types supported
- [ ] Performance targets met (< 100ms reads)

**Phase 3 Success (Month 2)**:

- [ ] Vector search operational
- [ ] Watcher agents running
- [ ] Auto-summaries generated
- [ ] Zero manual intervention required

---

## Part 9: Risks & Mitigations

| Risk                | Impact | Probability | Mitigation                  |
| ------------------- | ------ | ----------- | --------------------------- |
| R2 downtime         | High   | Low         | Local cache + fallback      |
| Migration data loss | High   | Medium      | Dry-run + backup            |
| Performance issues  | Medium | Medium      | Load testing + caching      |
| Docusaurus breaks   | High   | Low         | Thorough testing + rollback |
| Agent conflicts     | Medium | High        | Optimistic locking          |

---

## Conclusion

**The simplest viable implementation is:**

1. TypeScript MCP server
2. Minimal storage abstraction (AWS SDK wrapper)
3. Two backends: Local + R2
4. Append-only audit log
5. Docusaurus hydration script
6. One-day migration

**Defer to Phase 2:**

- Vertical Intelligence
- LanceDB
- Advanced search
- Schema system

**This gets us live TODAY while building foundation for future phases.**
