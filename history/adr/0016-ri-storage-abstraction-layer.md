# ADR-0016: RI Storage Abstraction Layer with MCP Server Architecture

**Status**: Proposed
**Date**: 2025-11-21
**Deciders**: Project maintainers + AI collaboration
**Consulted**: AgentFS (Turso), MCP filesystem servers, File-Store MCP patterns

---

## Context

### Problem Statement

The current RI (Reusable Intelligence) architecture (`docs/ri-architecture.md`) is tightly coupled to **local filesystem storage**:

```
.claude/          # Agent architectures (local files)
.specify/         # SDD-RI templates (local files)
specs/            # Feature specifications (local files, 50+)
history/          # Organizational memory (local files, ADRs/PHRs/audits)
```

**Critical Limitations**:
1. ❌ **No cloud deployment**: Cannot store RI in Cloudflare R2, AWS S3, or other scalable storage
2. ❌ **Tight coupling**: All agents assume `fs.readFile()` / `fs.writeFile()` (Node.js filesystem)
3. ❌ **No versioning**: Entire RI state distributed across thousands of files (hard to snapshot)
4. ❌ **No audit trail**: Cannot track which agent modified which intelligence artifact, when, or why
5. ❌ **Limited portability**: Filesystem paths hard-coded throughout codebase

### Strategic Requirements

**From Constitution** (`.specify/memory/constitution.md` v6.0.1):

> **Principle 5: Intelligence Accumulation** — Before creating content, reason about: What accumulated intelligence exists that informs this work?

**RI as Strategic Asset** (`docs/ri-architecture.md`):
- Specifications → Executable contracts (primary artifact)
- Agent architectures → Reasoning frameworks (not code)
- Skills → Bundled expertise (P+Q+P pattern)
- Organizational memory → Decision provenance (ADRs, PHRs, audits)

**Implications**:
- Intelligence must be **queryable** (not just files in directories)
- Intelligence must be **versionable** (track evolution over time)
- Intelligence must be **auditable** (who changed what, when, why)
- Intelligence must be **portable** (local ↔ cloud seamlessly)

### Research Findings

**Key Insights from External Systems**:

1. **AgentFS (Turso)**:
   - Entire agent runtime in **single SQLite file** (portable, versionable)
   - **Audit trail**: Every file operation + tool call logged
   - **Reproducibility**: Snapshot via file copy, restore via replacement
   - **Queryable**: SQL interface to agent history

2. **File-Store MCP**:
   - **Multi-backend abstraction**: AWS S3, R2, MinIO, OSS, COS, Qiniu, GitHub
   - **Single interface** for multiple cloud backends
   - Critical for scalable agentic systems

3. **MCP Official Filesystem Server**:
   - **Proven pattern**: MCP protocol for filesystem operations
   - **Security**: Directory access control via CLI args or Roots
   - **Dynamic updates**: `roots/list_changed` notifications

**Gap Identified**: No existing solution combines:
- AgentFS-style audit trail + versioning
- File-Store MCP's multi-backend abstraction
- MCP-native protocol (standardized, interoperable)

---

## Decision

We will implement a **custom RI Storage MCP Server** with multi-backend abstraction, following this architecture:

```
┌──────────────────────────────────────────────────────────────┐
│ AI Agents (Claude, super-orchestra, content-implementer)     │
│ • Use MCP client to access storage                           │
│ • Unaware of underlying storage backend                      │
└──────────────────────────────────────────────────────────────┘
                          ↓ MCP Protocol
┌──────────────────────────────────────────────────────────────┐
│ RI Storage MCP Server (Node.js / TypeScript)                 │
│ • Implements MCP specification 2025-06-18                    │
│ • Provides unified storage API (resources + tools)           │
│ • Handles audit logging                                      │
│ • Manages versioning (optional, backend-specific)            │
└──────────────────────────────────────────────────────────────┘
                          ↓
┌──────────────────────────────────────────────────────────────┐
│ Storage Abstraction Layer                                    │
│ • Backend-agnostic interface (RIStorageBackend)              │
│ • Adapter pattern for each backend                           │
└──────────────────────────────────────────────────────────────┘
        ↓               ↓               ↓               ↓
┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│ Local FS     │ │ Cloudflare   │ │ AWS S3       │ │ SQLite       │
│ Adapter      │ │ R2 Adapter   │ │ Adapter      │ │ Adapter      │
│              │ │              │ │              │ │ (AgentFS     │
│              │ │              │ │              │ │ portability) │
└──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘
```

### Core Components

#### 1. Storage Abstraction Interface

```typescript
interface RIStorageBackend {
  // Core operations
  read(path: string): Promise<Buffer | string>;
  write(path: string, content: Buffer | string): Promise<void>;
  delete(path: string): Promise<void>;
  list(path: string): Promise<string[]>;
  exists(path: string): Promise<boolean>;

  // Metadata operations
  stat(path: string): Promise<FileStat>;

  // Batch operations (optimization)
  readBatch(paths: string[]): Promise<Map<string, Buffer | string>>;
  writeBatch(files: Map<string, Buffer | string>): Promise<void>;

  // Audit operations (optional, backend-specific)
  getAuditLog?(path: string, options?: AuditOptions): Promise<AuditEntry[]>;
}
```

#### 2. Backend Adapters

- **LocalFilesystemAdapter**: Development baseline (Node.js `fs`)
- **CloudflareR2Adapter**: Production cloud storage (S3-compatible)
- **AWSS3Adapter**: Enterprise deployments
- **SQLiteStorageAdapter**: AgentFS-inspired portability (entire RI in one file)

#### 3. MCP Server Implementation

**Resources**:
- `resources/list` → List all RI artifacts
- `resources/read` → Read RI content (specs, plans, ADRs, etc.)
- `resources/subscribe` → Real-time updates when RI changes

**Tools**:
- `write_file` → Write content to RI storage
- `delete_file` → Delete RI artifact
- `get_audit_log` → Retrieve operation history for artifact

### Implementation Approach: Hybrid (Prototype → Extract)

**Phase 1: Prototype Integrated** (2-3 weeks)
- Build inside `tutorsgpt/md/ri-storage-mcp/`
- Validate design with real RI workloads
- Iterate rapidly on adapter interface

**Phase 2: Extract to Separate Repo** (Week 4+, if successful)
- Extract to `ri-storage-mcp` repository
- Publish to npm (`@panaversity/ri-storage-mcp`)
- tutorsgpt becomes consumer (install from npm)

**Rationale for Hybrid**:
- ✅ **Fast prototyping**: Co-located with RI for rapid iteration
- ✅ **Low risk**: Can stay integrated if extraction isn't worth it
- ✅ **Proven design**: Don't extract until battle-tested with super-orchestra workflows

---

## Rationale

### Why MCP Server (Not Direct Library)?

**MCP Advantages**:
1. **Standardized protocol**: Works with ANY MCP-compatible client (Claude Desktop, Claude Code, OpenAI Agents SDK)
2. **Process isolation**: Storage server runs separately from agents (security, fault tolerance)
3. **Interoperability**: Other projects can use same storage server
4. **Future-proof**: MCP is open standard, not proprietary API

**Alternative Rejected**: Direct TypeScript library
- ❌ Tight coupling to Node.js
- ❌ No interoperability with Python agents, .NET agents, etc.
- ❌ Harder to swap storage backends at runtime

### Why Multi-Backend Abstraction?

**Use Cases**:
- **Local development**: Filesystem (fast iteration, no cloud costs)
- **Production**: Cloudflare R2 (scalable, cost-effective)
- **Enterprise**: AWS S3 (compliance, existing infrastructure)
- **Portability**: SQLite (entire RI in one file, git-trackable)

**Flexibility**: Same RI code works across all backends (agents don't change)

### Why Audit Trail?

**Critical for Educational Content**:
- **Compliance**: Track which agent modified constitution (v6.0.0 → v6.0.1)
- **Debugging**: "Why did this spec change between yesterday and today?"
- **Provenance**: Trace decision from ADR → Constitution → Implementation → Lesson
- **Rollback**: Revert to previous version if validation fails

**Example Audit Query**:
```typescript
// Check who modified constitution recently
const auditLog = await mcp.callTool('get_audit_log', {
  path: '.specify/memory/constitution.md',
  limit: 10
});
// Returns: [{ timestamp, operation: 'write', agent: 'super-orchestra', success: true, metadata }]
```

### Why AgentFS-Inspired SQLite Adapter?

**Portability Benefits**:
- **Single-file RI**: Entire intelligence (specs, agents, skills, ADRs, PHRs) in one `.db` file
- **Git-trackable**: Can version entire RI state (export as SQL dump)
- **Snapshot-restore**: Copy file = instant backup/restore
- **Queryable**: SQL interface to intelligence (`SELECT * FROM specs WHERE status = 'active'`)

**Trade-offs Accepted**:
- ⚠️ Binary file (not human-readable like markdown)
- ⚠️ Additional adapter complexity
- ✅ **But**: Optional — can use filesystem for dev, SQLite for production snapshots

---

## Consequences

### Positive

1. **🎯 Backend Flexibility**: Same RI works on local filesystem, R2, S3, SQLite
2. **🔍 Audit Trail**: Complete operation history for all intelligence artifacts
3. **📦 Portability**: SQLite adapter enables single-file RI distribution
4. **🔌 Interoperability**: MCP protocol works with ANY MCP-compatible client
5. **🛡️ Security**: Process isolation between storage server and agents
6. **📈 Scalability**: Cloud storage (R2/S3) handles unlimited RI growth
7. **🔄 Versioning**: Entire RI state can be snapshotted and restored
8. **🧪 Testability**: Mock storage backends for agent testing

### Negative

1. **⏱️ Development Time**: 2-3 weeks to implement + test adapters
2. **🔧 Complexity**: Additional abstraction layer (interface + 4 adapters)
3. **🐛 Debugging**: Storage issues now span MCP protocol + backend adapters
4. **📚 Documentation**: Need comprehensive guide for configuration (local vs R2 vs S3)
5. **🚀 Deployment**: Extra configuration step (MCP server setup)

### Neutral

1. **🔄 Migration**: Existing RI (local filesystem) continues to work via LocalFilesystemAdapter
2. **🧰 Agent Changes**: Minimal — swap `fs.readFile()` with `mcp.readResource('ri://path')`
3. **📊 Performance**: Network latency for R2/S3 (vs instant for local FS) — acceptable for RI workloads

---

## Compliance

### Constitutional Alignment

**Principle 5: Intelligence Accumulation** (Constitution v6.0.1, Section III):

> Before creating content, reason about: What accumulated intelligence exists that informs this work?

**How Storage Abstraction Supports This**:
- ✅ **Context-Rich Workflows**: Agents query RI across ANY storage backend (vertical intelligence)
- ✅ **Reusable Intelligence**: Specifications, agents, skills are storage-agnostic (portable)
- ✅ **Organizational Memory**: Audit trail preserves decision provenance (ADRs → Constitution → Lessons)
- ✅ **Never Zero Context**: Storage abstraction preserves intelligence access patterns

**Agent Workflow Transparency**:

**Before** (filesystem-bound):
```typescript
const constitution = await fs.readFile('.specify/memory/constitution.md', 'utf-8');
```

**After** (storage-agnostic):
```typescript
const constitution = await mcp.readResource('ri://.specify/memory/constitution.md');
```

**Impact**: Agents don't change logic, just API calls. Intelligence accumulation pattern preserved.

---

## Alternatives Considered

### Alternative 1: Use Official MCP Filesystem Server

**Pros**: Already exists, maintained by Anthropic
**Cons**:
- ❌ Local filesystem only (no R2/S3)
- ❌ No audit trail
- ❌ No versioning
- ❌ No abstraction layer

**Rejected**: Doesn't meet RI portability requirements

---

### Alternative 2: Use File-Store MCP (Existing)

**Pros**: Multi-backend support (R2, S3, OSS, COS, etc.)
**Cons**:
- ❌ No audit trail
- ❌ No SQLite portability option
- ❌ Generic file storage (not RI-specific)
- ⚠️ May lack RI-specific optimizations (batch operations, intelligence querying)

**Rejected**: Close, but missing audit trail and RI-specific features

---

### Alternative 3: Direct Cloud SDK Integration (No Abstraction)

**Approach**: Directly use `@aws-sdk/client-s3` or R2 API in agents

**Pros**: Simpler (no abstraction layer)
**Cons**:
- ❌ **Tight coupling**: Every agent needs cloud storage code
- ❌ **No flexibility**: Hard to switch backends later
- ❌ **No local development**: Must use cloud even for dev
- ❌ **No MCP standard**: Proprietary API (not interoperable)

**Rejected**: Violates Constitution Principle 5 (Intelligence Accumulation requires context-rich, reusable patterns)

---

### Alternative 4: AgentFS (Turso) Directly

**Approach**: Use AgentFS as-is

**Pros**: Audit trail, versioning, portability (single SQLite file)
**Cons**:
- ❌ **Not MCP-native**: Proprietary API (not standardized)
- ❌ **SQLite-only**: Can't use R2/S3 directly
- ❌ **Single-backend**: Locked to Turso ecosystem

**Rejected**: Too restrictive, but **patterns adopted** (audit trail, single-file portability)

---

## Implementation Plan

### Phase 1: Research Validation (COMPLETED)

- [x] Research AgentFS architecture (docs/storage-abstraction-research.md)
- [x] Identify existing MCP filesystem servers
- [x] Analyze MCP specification for storage patterns
- [x] Create ADR documenting decision (this document)

### Phase 2: Prototype (Week 1-2)

- [ ] Create `tutorsgpt/md/ri-storage-mcp/` directory
- [ ] Implement `RIStorageBackend` interface (`src/storage/interface.ts`)
- [ ] Create Local Filesystem Adapter (baseline, `src/storage/local-adapter.ts`)
- [ ] Create Cloudflare R2 Adapter (primary cloud, `src/storage/r2-adapter.ts`)
- [ ] Implement basic MCP server (`src/server.ts`)
  - `resources/list`, `resources/read`, `resources/subscribe`
  - `tools/call`: `write_file`, `delete_file`
- [ ] Test with tutorsgpt RI workloads (read constitution, specs, ADRs)

### Phase 3: Audit & Versioning (Week 3)

- [ ] Add audit logging to all adapters
- [ ] Create SQLite adapter (`src/storage/sqlite-adapter.ts`) — AgentFS pattern
- [ ] Implement `get_audit_log` tool
- [ ] Test audit queries:
  - "Which agent modified constitution v6.0.0 → v6.0.1?"
  - "Show all spec changes for Chapter 5"

### Phase 4: Integration (Week 4)

- [ ] Update `.claude/agents/super-orchestra.md` to use RI storage MCP
- [ ] Migrate critical workflows:
  - chapter-planner (read specs, write plans)
  - content-implementer (read plans, write lessons)
  - validation-auditor (read lessons, write audit reports)
- [ ] Update `docs/ri-architecture.md` with storage abstraction section
- [ ] Document configuration:
  - Local development (filesystem)
  - Production (Cloudflare R2)
  - Enterprise (AWS S3)
  - Portable (SQLite)

### Phase 5: Production Deployment (Week 5-6)

- [ ] Deploy RI to Cloudflare R2 bucket
- [ ] Configure CI/CD for RI versioning (snapshot on release)
- [ ] Create backup/restore workflows (SQLite export)
- [ ] Monitor audit logs for anomalies

### Phase 6: Open Source (Week 7+, Optional)

- [ ] Evaluate extraction feasibility
- [ ] If successful: Extract to `ri-storage-mcp` repository
- [ ] Publish to npm (`@panaversity/ri-storage-mcp`)
- [ ] Write comprehensive documentation
- [ ] Create examples for other projects

---

## Monitoring & Success Criteria

### Success Metrics

**Technical**:
- ✅ All 4 adapters implemented (Local, R2, S3, SQLite)
- ✅ MCP server passes compliance tests (resources + tools)
- ✅ Audit trail captures 100% of write/delete operations
- ✅ Performance: <100ms latency for local FS, <500ms for R2/S3

**Workflow Validation**:
- ✅ super-orchestra workflow uses RI storage MCP (read constitution, specs)
- ✅ chapter-planner reads spec.md from RI storage, writes plan.md
- ✅ validation-auditor queries audit log for compliance checks

**Constitutional Compliance**:
- ✅ Principle 5 (Intelligence Accumulation) preserved across all backends
- ✅ Agent workflows remain context-rich (not horizontally distributed)
- ✅ Organizational memory (ADRs, PHRs, audits) queryable via storage API

### Rollback Criteria

**If any of these occur, revert to filesystem-only**:
- ❌ Storage abstraction adds >200ms latency to critical workflows
- ❌ Cloud backends (R2/S3) prove unreliable (>5% failure rate)
- ❌ Audit trail overhead causes performance degradation
- ❌ Migration complexity blocks RI evolution

---

## References

### External Resources

1. **AgentFS (Turso)**: https://github.com/tursodatabase/agentfs
   - Patterns: Single-file portability, audit trail, SQLite storage

2. **File-Store MCP**: https://skywork.ai/skypage/en/Deep-Dive-into-file-store-mcp-The-Definitive-Guide-for-AI-Engineers/1971023230360678400
   - Patterns: Multi-backend abstraction, S3/R2 compatibility

3. **MCP Official Filesystem Server**: https://github.com/modelcontextprotocol/servers/tree/main/src/filesystem
   - Patterns: MCP resources + tools, directory access control

4. **MCP Specification 2025-06-18**: https://modelcontextprotocol.io/specification/2025-06-18/
   - Standards: Resource URIs, tool schemas, notification protocols

### Internal Documents

1. **RI Architecture**: `docs/ri-architecture.md`
   - Current: 5-layer vertical intelligence stack
   - Impact: Add storage abstraction as Layer 0 (foundation)

2. **Constitution v6.0.1**: `.specify/memory/constitution.md`
   - Principle 5: Intelligence Accumulation (context-rich workflows)
   - Compliance: Storage abstraction preserves intelligence access patterns

3. **Storage Abstraction Research**: `docs/storage-abstraction-research.md`
   - Complete analysis of AgentFS, MCP servers, design proposals

---

## Approval

**Proposed By**: AI Collaboration + Project Maintainers
**Date Proposed**: 2025-11-21

**Awaiting Approval From**: User (@mjs)

**Decision Options**:
1. **Approve**: Proceed with Phase 2 (Prototype)
2. **Modify**: Request changes to design (specify which aspects)
3. **Reject**: Revert to filesystem-only, defer storage abstraction

---

**Document Status**: Proposed
**Next Review**: After Phase 2 completion (Week 2)
**Impact**: High (foundational infrastructure change)

---

**END OF ADR-0016**
