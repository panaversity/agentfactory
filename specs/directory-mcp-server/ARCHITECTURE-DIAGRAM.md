# Directory MCP Server: Visual Architecture

**Status**: Ready for Implementation
**Approach**: MVP-First (1-Day Deployment)

---

## System Overview

```
┌───────────────────────────────────────────────────────────────────────┐
│                          CONSUMERS                                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────────────┐│
│  │ Claude Code  │  │ Python Agent │  │ Docusaurus Build (GitHub CI) ││
│  │ (MCP Client) │  │ (MCP Client) │  │ (Hydration Script)           ││
│  └──────────────┘  └──────────────┘  └──────────────────────────────┘│
└───────────────────────────────────────────────────────────────────────┘
                                │
                                ↓
┌───────────────────────────────────────────────────────────────────────┐
│                    DIRECTORY MCP SERVER                                │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │                          MCP Protocol Layer                     │ │
│  │  ┌────────────────────────────────────────────────────────────┐│ │
│  │  │ Tools (Agent-Callable Functions)                           ││ │
│  │  │                                                             ││ │
│  │  │  1. read_content(path)                                     ││ │
│  │  │     → Read lesson file from storage                        ││ │
│  │  │                                                             ││ │
│  │  │  2. write_content(path, content, agent_id)                 ││ │
│  │  │     → Write lesson file to storage + audit log             ││ │
│  │  │                                                             ││ │
│  │  │  3. list_contents(prefix)                                  ││ │
│  │  │     → List all files in directory                          ││ │
│  │  │                                                             ││ │
│  │  │  4. delete_content(path, agent_id)                         ││ │
│  │  │     → Delete file from storage + audit log                 ││ │
│  │  │                                                             ││ │
│  │  │  5. get_audit_log(filters)                                 ││ │
│  │  │     → Query operation history                              ││ │
│  │  └────────────────────────────────────────────────────────────┘│ │
│  └─────────────────────────────────────────────────────────────────┘ │
│                                                                        │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │                      Audit Logger                               │ │
│  │  Append-only JSONL file: audit.jsonl                           │ │
│  │  Format: { timestamp, operation, path, agentId, metadata }     │ │
│  └─────────────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────────┘
                                │
                                ↓
┌───────────────────────────────────────────────────────────────────────┐
│                   STORAGE ABSTRACTION LAYER                            │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │              StorageBackend Interface                           │ │
│  │  ┌────────────────────────────────────────────────────────────┐│ │
│  │  │  read(path: string): Promise<string>                       ││ │
│  │  │  write(path: string, content: string): Promise<void>       ││ │
│  │  │  list(prefix: string): Promise<string[]>                   ││ │
│  │  │  exists(path: string): Promise<boolean>                    ││ │
│  │  │  delete(path: string): Promise<void>                       ││ │
│  │  └────────────────────────────────────────────────────────────┘│ │
│  └─────────────────────────────────────────────────────────────────┘ │
│                                                                        │
│  ┌─────────────────────────────┐   ┌──────────────────────────────┐  │
│  │   LocalStorage              │   │   R2Storage                  │  │
│  │   (Development)             │   │   (Production)               │  │
│  │                             │   │                              │  │
│  │  - Uses Node.js fs module   │   │  - Uses AWS SDK S3 client    │  │
│  │  - Reads from ./content/    │   │  - Reads from R2 bucket      │  │
│  │  - Fast for local dev       │   │  - CDN-backed, scalable      │  │
│  └─────────────────────────────┘   └──────────────────────────────┘  │
└───────────────────────────────────────────────────────────────────────┘
                  │                                 │
                  ↓                                 ↓
┌──────────────────────────────┐  ┌──────────────────────────────────┐
│    Local Filesystem          │  │    Cloudflare R2                 │
│                              │  │                                  │
│  ./content/                  │  │  s3://panaversity-book/          │
│  ├── 01-Part/                │  │  ├── 01-Part/                    │
│  │   └── 01-chapter/         │  │  │   └── 01-chapter/             │
│  │       └── 01-lesson.md    │  │  │       └── 01-lesson.md        │
│  └── 02-Part/                │  │  └── 02-Part/                    │
│      └── ...                 │  │      └── ...                     │
└──────────────────────────────┘  └──────────────────────────────────┘
```

---

## Data Flow Diagrams

### Flow 1: Agent Writes Lesson (via Claude Code)

```
┌──────────────┐
│ Claude Code  │  User: "Update Chapter 5, Lesson 2"
│ (MCP Client) │
└──────┬───────┘
       │
       │ MCP Request:
       │ {
       │   tool: "write_content",
       │   args: {
       │     path: "01-Part/05-chapter/02-lesson.md",
       │     content: "# Updated Lesson...",
       │     agent_id: "claude-code-123"
       │   }
       │ }
       ↓
┌─────────────────────┐
│  Directory MCP      │  1. Validate request
│  Server             │  2. Call storage.write()
│                     │  3. Log to audit.jsonl
└─────────┬───────────┘
          │
          │ storage.write("01-Part/05-chapter/02-lesson.md", content)
          ↓
┌─────────────────────┐
│  R2Storage          │  AWS SDK PutObjectCommand
│                     │  → Cloudflare R2
└─────────┬───────────┘
          │
          ↓
┌─────────────────────┐
│  Cloudflare R2      │  File stored:
│  Bucket             │  s3://panaversity-book/01-Part/05-chapter/02-lesson.md
└─────────────────────┘

Simultaneously:
┌─────────────────────┐
│  Audit Logger       │  Append to audit.jsonl:
│                     │  {
│                     │    timestamp: "2025-11-21T10:30:00Z",
│                     │    operation: "write",
│                     │    path: "01-Part/05-chapter/02-lesson.md",
│                     │    agentId: "claude-code-123"
│                     │  }
└─────────────────────┘
```

---

### Flow 2: Docusaurus Build (via GitHub Actions)

```
┌─────────────────────┐
│  GitHub Actions     │  Trigger: Push to main
│  CI Pipeline        │
└─────────┬───────────┘
          │
          │ Step 1: Run hydration script
          ↓
┌─────────────────────┐
│  Hydration Script   │  1. Connect to MCP server
│  (TypeScript)       │  2. Call list_contents("")
│                     │  3. For each file: call read_content(path)
│                     │  4. Write to ./book-source/docs-temp/
└─────────┬───────────┘
          │
          │ MCP Request: list_contents("")
          ↓
┌─────────────────────┐
│  Directory MCP      │  Return: [
│  Server             │    "01-Part/01-chapter/01-lesson.md",
│                     │    "01-Part/01-chapter/02-lesson.md",
│                     │    ...
│                     │  ]
└─────────┬───────────┘
          │
          │ storage.list("")
          ↓
┌─────────────────────┐
│  R2Storage          │  AWS SDK ListObjectsV2Command
│                     │  → Fetch all object keys
└─────────┬───────────┘
          │
          ↓
┌─────────────────────┐
│  Cloudflare R2      │  Return list of all .md files
└─────────────────────┘

Then, for each file:
┌─────────────────────┐
│  Hydration Script   │  For "01-Part/01-chapter/01-lesson.md":
│                     │  1. Call read_content(path)
│                     │  2. Write to ./book-source/docs-temp/01-Part/01-chapter/01-lesson.md
└─────────┬───────────┘
          │
          ↓
┌─────────────────────┐
│  Local Filesystem   │  ./book-source/docs-temp/
│  (Build Server)     │  ├── 01-Part/
│                     │  │   └── 01-chapter/
│                     │  │       └── 01-lesson.md  ← Fetched from R2
│                     │  └── 02-Part/
│                     │      └── ...
└─────────┬───────────┘
          │
          │ Step 2: Run Docusaurus build
          ↓
┌─────────────────────┐
│  Docusaurus Build   │  1. Read ./book-source/docs-temp/
│                     │  2. Generate static site
│                     │  3. Output to ./build/
└─────────┬───────────┘
          │
          │ Step 3: Deploy to GitHub Pages
          ↓
┌─────────────────────┐
│  GitHub Pages       │  ✅ Site live at panaversity.com
└─────────────────────┘
```

---

## Component Architecture

### MCP Server Component Diagram

```
┌───────────────────────────────────────────────────────────────┐
│  src/index.ts                                                 │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  main()                                                  │ │
│  │  ├─ Load config from config.json                        │ │
│  │  ├─ Initialize storage backend (Local or R2)            │ │
│  │  ├─ Initialize audit logger                             │ │
│  │  └─ Start MCP server                                    │ │
│  └─────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        ↓                   ↓                   ↓
┌──────────────┐  ┌───────────────────┐  ┌──────────────┐
│ src/mcp/     │  │ src/storage/      │  │ src/audit/   │
│ server.ts    │  │ interface.ts      │  │ logger.ts    │
│              │  │ local.ts          │  │              │
│ - Server     │  │ r2.ts             │  │ - AuditEntry │
│ - Tools      │  │                   │  │ - log()      │
│ - Handlers   │  │ - StorageBackend  │  │ - query()    │
└──────────────┘  │ - LocalStorage    │  └──────────────┘
                  │ - R2Storage       │
                  └───────────────────┘
```

---

## File Size Breakdown

```
Total: ~600 LOC

src/
├── index.ts                 40 LOC   │ Entry point
├── config.ts                20 LOC   │ Config loader
├── storage/
│   ├── interface.ts         20 LOC   │ Interface definition
│   ├── local.ts             80 LOC   │ Filesystem implementation
│   └── r2.ts                80 LOC   │ R2 implementation
├── mcp/
│   └── server.ts           150 LOC   │ MCP protocol + tools
└── audit/
    └── logger.ts            50 LOC   │ Audit logging

scripts/
├── migrate-to-r2.ts         50 LOC   │ Git → R2 migration
└── hydrate-docusaurus.ts    70 LOC   │ R2 → Docusaurus hydration

tests/
├── storage.test.ts          30 LOC   │ Storage tests
├── mcp.test.ts              30 LOC   │ MCP server tests
└── audit.test.ts            30 LOC   │ Audit logger tests
```

---

## Deployment Architecture

### Current State (Before MVP)

```
┌────────────────────┐
│  Local Development │
│  Machine           │
│  ├─ Git repo       │
│  └─ book-source/   │
│      └─ docs/      │
└────────┬───────────┘
         │
         │ git push
         ↓
┌────────────────────┐
│  GitHub            │
│  Repository        │
│  ├─ .md files      │
│  └─ Images         │
└────────┬───────────┘
         │
         │ GitHub Actions
         ↓
┌────────────────────┐
│  GitHub Actions    │
│  Build Server      │
│  ├─ Clone repo     │
│  ├─ npm run build  │
│  └─ Deploy         │
└────────┬───────────┘
         │
         ↓
┌────────────────────┐
│  GitHub Pages      │
│  (Static Site)     │
└────────────────────┘
```

### Future State (After MVP)

```
┌────────────────────┐
│  Local Development │
│  Machine           │
│  ├─ MCP Client     │
│  └─ Agents         │
└────────┬───────────┘
         │
         │ MCP Protocol
         ↓
┌────────────────────┐
│  Directory MCP     │
│  Server            │
│  (Running locally  │
│   or as service)   │
└────────┬───────────┘
         │
         │ AWS SDK
         ↓
┌────────────────────┐
│  Cloudflare R2     │
│  ├─ 84 chapters    │
│  └─ Assets         │
└────────┬───────────┘
         │
         │ Hydration (GitHub Actions)
         ↓
┌────────────────────┐
│  GitHub Actions    │
│  Build Server      │
│  ├─ Run hydrate    │
│  ├─ npm run build  │
│  └─ Deploy         │
└────────┬───────────┘
         │
         ↓
┌────────────────────┐
│  GitHub Pages      │
│  (Static Site)     │
└────────────────────┘
```

---

## Security & Access Control

### Authentication Flow

```
┌──────────────┐
│ MCP Client   │  Environment Variables:
│ (Agent)      │  - R2_ACCOUNT_ID
│              │  - R2_ACCESS_KEY_ID
└──────┬───────┘  - R2_SECRET_ACCESS_KEY
       │
       │ Reads from .env or CI secrets
       ↓
┌─────────────────────┐
│  MCP Server         │  Uses credentials to initialize R2Storage
│  (Trusted Process)  │
└─────────┬───────────┘
          │
          │ AWS SDK with credentials
          ↓
┌─────────────────────┐
│  Cloudflare R2      │  Validates credentials
│  (IAM Policy:       │  Allows: GetObject, PutObject, ListBucket
│   read/write only)  │  Denies: DeleteBucket, etc.
└─────────────────────┘
```

**Security Principles**:
1. ✅ Credentials never exposed to end users
2. ✅ MCP server is trusted intermediary
3. ✅ R2 IAM policy restricts permissions (read/write only, no admin)
4. ✅ Audit log tracks all operations with agent_id

---

## Configuration Flow

```
┌──────────────────────┐
│  config.json         │
│  {                   │
│    storage: {        │
│      backend: "r2",  │ ← Environment-specific
│      r2: { ... }     │
│    },                │
│    audit: {          │
│      enabled: true,  │
│      logPath: "..."  │
│    }                 │
│  }                   │
└──────────┬───────────┘
           │
           │ Loaded by src/config.ts
           ↓
┌──────────────────────┐
│  main() function     │
│                      │
│  if (backend == "r2")│
│    storage = R2Storage(...)
│  else                │
│    storage = LocalStorage(...)
│                      │
└──────────────────────┘
```

**Environments**:
- **Development**: `backend: "local"` → Uses `./content/`
- **Production**: `backend: "r2"` → Uses R2 bucket
- **Switching**: Change `config.json`, restart server (zero code changes)

---

## Migration Strategy

### Phase 1: Preparation (No Disruption)

```
┌────────────────────┐
│  Git Repository    │  ← Still authoritative source
│  (Current)         │    No changes yet
└────────────────────┘

┌────────────────────┐
│  R2 Bucket         │  ← Created, empty
│  (New)             │
└────────────────────┘
```

### Phase 2: Migration (Dry Run)

```
┌────────────────────┐
│  Git Repository    │
│  (84 chapters)     │
└────────┬───────────┘
         │
         │ migration script (dry-run)
         ↓
┌────────────────────┐
│  R2 Bucket         │  Test upload, verify integrity
│  (Copy)            │
└────────────────────┘

Verify:
- [ ] File count matches (84 chapters)
- [ ] Content hashes match
- [ ] Folder structure preserved
```

### Phase 3: Cutover (Controlled)

```
┌────────────────────┐
│  Git Repository    │  ← Read-only (backup)
│  (Archived)        │
└────────────────────┘

┌────────────────────┐
│  R2 Bucket         │  ← New authoritative source
│  (Primary)         │    Agents write here
└────────────────────┘

GitHub Actions updated:
- Old: Build from Git
- New: Hydrate from R2 → Build
```

### Phase 4: Monitoring (24 Hours)

```
Monitor:
- [ ] Audit log shows operations
- [ ] Docusaurus builds succeed
- [ ] Website updates correctly
- [ ] No errors in MCP server logs

If issues:
→ Rollback to Git (revert GitHub Actions)
```

---

## Rollback Procedures

### Rollback Plan A: Revert GitHub Actions

```yaml
# Revert .github/workflows/deploy.yml to previous version
# Remove hydration step, build directly from Git

git revert <commit-hash>
git push
```

**Time to rollback**: 2 minutes

---

### Rollback Plan B: Switch to Local Backend

```json
// Change config.json
{
  "storage": {
    "backend": "local",  // ← Changed from "r2"
    "local": {
      "rootPath": "./content"  // ← Points to Git clone
    }
  }
}
```

**Time to rollback**: 30 seconds (config change + restart)

---

### Rollback Plan C: Re-migrate from Git

```bash
# If R2 data corrupted, re-upload from Git
npm run migrate
```

**Time to rollback**: 10 minutes (re-upload 84 chapters)

---

## Performance Characteristics

### Read Performance

```
Agent Request: read_content("01-Part/05-chapter/02-lesson.md")
    ↓
MCP Server (< 5ms processing)
    ↓
R2Storage.read()
    ↓
AWS SDK GetObjectCommand
    ↓
Cloudflare R2 (CDN-backed, < 50ms)
    ↓
Return content

Total: < 100ms (p95)
```

### Write Performance

```
Agent Request: write_content(path, content, agent_id)
    ↓
MCP Server (validate)
    ↓
R2Storage.write()  +  AuditLogger.log()  (parallel)
    ↓                     ↓
AWS SDK PutObject    Append to JSONL
    ↓                     ↓
R2 upload            Local disk write
(~200ms)             (~10ms)
    ↓
Return success

Total: < 500ms (p95)
```

---

## Monitoring & Observability

### Audit Log Query Examples

**Query 1: All writes in last 24 hours**
```typescript
await audit.query({
  operation: 'write',
  since: new Date(Date.now() - 24*60*60*1000).toISOString()
});
```

**Query 2: All operations by specific agent**
```typescript
await audit.query({
  agentId: 'claude-code-123',
  limit: 100
});
```

**Query 3: Recent errors** (if metadata.error exists)
```typescript
const entries = await audit.query({ limit: 1000 });
const errors = entries.filter(e => e.metadata?.error);
```

---

## Conclusion

**This architecture**:
- ✅ Solves the core problem (storage abstraction + agent access)
- ✅ Deploys in 1 day (~600 LOC)
- ✅ Low risk (proven tech, reversible)
- ✅ Scalable (R2 CDN, concurrent access)
- ✅ Cost-effective (~$0.02/month)
- ✅ Foundation for future features (Phase 2-3)

**Ready for implementation.**
