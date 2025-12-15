# Storage Abstraction Layer Research & Design
**Version**: 1.0
**Date**: 2025-11-21
**Status**: Research & Architecture Design

---

## Executive Summary

This document analyzes the requirement for an **extensible, storage-agnostic MCP server** that abstracts filesystem operations across local, cloud (R2/S3), and any compatible storage backend. This infrastructure will enable the RI (Reusable Intelligence) architecture to scale beyond local filesystems while maintaining a unified API for AI agents.

**Strategic Goal**: Transform the RI architecture from filesystem-bound to storage-agnostic, enabling:
- **Local development** (filesystem)
- **Cloud deployment** (R2, S3, Azure Blob, GCS)
- **Agent portability** (same RI works anywhere)
- **Versioned intelligence** (git-trackable via AgentFS patterns)
- **Audit trails** (complete operation history)

---

## Table of Contents

1. [Problem Statement](#problem-statement)
2. [Research Findings](#research-findings)
3. [Architectural Analysis](#architectural-analysis)
4. [Design Proposal](#design-proposal)
5. [Implementation Approaches](#implementation-approaches)
6. [Integration with RI Architecture](#integration-with-ri-architecture)
7. [Next Steps](#next-steps)

---

## Problem Statement

### Current State

The RI architecture (`docs/ri-architecture.md`) currently assumes **local filesystem** for all intelligence storage:

```
.claude/                          # Agent architectures & output styles
.specify/                         # SDD-RI workflow templates
specs/                            # Feature specifications (50+ features)
history/                          # Organizational memory (ADRs, PHRs, audits)
papers/                           # Research foundation
apps/learn-app/docs/                 # Generated outputs
```

**Limitations**:
- ❌ Cannot deploy RI to cloud storage (R2, S3)
- ❌ No storage abstraction layer (tightly coupled to Node.js `fs`)
- ❌ Difficult to version entire RI state (distributed across many files)
- ❌ No audit trail for intelligence operations
- ❌ Limited portability (filesystem paths hard-coded)

### Required State

**Vision**: Unified Storage API that works across:
- **Local Filesystem** (development, testing)
- **Cloudflare R2** (production, scalable, cost-effective)
- **AWS S3** (enterprise deployments)
- **Any S3-compatible storage** (MinIO, Wasabi, Backblaze B2)

**Design Goals**:
1. **Abstraction**: Same API regardless of backend
2. **Portability**: RI portable across storage backends
3. **Versioning**: Entire RI state versionable (like AgentFS single-file approach)
4. **Auditability**: Complete operation history (read, write, delete)
5. **MCP Native**: Built as MCP server (not proprietary API)

---

## Research Findings

### 1. AgentFS Architecture (Turso)

**Source**: https://github.com/tursodatabase/agentfs

**Key Insights**:

#### Three-Layer Architecture
```
┌─────────────────────────────────────────┐
│ Execution Environment (Linux sandbox)   │
│ • Mounts /agent directory               │
│ • Isolates execution                    │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│ SDK Layer (TypeScript, Rust)            │
│ • Programmatic filesystem access        │
│ • Key-value store                       │
│ • Toolcall ledger (audit trail)         │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│ Storage Foundation (SQLite/Turso)       │
│ • Single database file                  │
│ • All state + files + operations        │
│ • Portable, versionable, queryable      │
└─────────────────────────────────────────┘
```

**Critical Design Patterns**:

1. **Portability Through Encapsulation**
   - Entire runtime in one SQLite file
   - Movable across machines
   - Versionable in git (binary or exported as SQL)
   - Deployable anywhere

2. **Auditability-First**
   - Every file operation recorded in SQLite
   - Tool calls logged with timestamps
   - Query agent history with SQL
   - Enables debugging and compliance

3. **Reproducibility**
   - Snapshot via file copy
   - Restore via file replacement
   - What-if testing through snapshots
   - State rollback for experimentation

**RI Architecture Implications**:
- ✅ **Adopt**: Single-file portability (package entire RI)
- ✅ **Adopt**: Audit trail (all intelligence operations logged)
- ⚠️ **Modify**: SQLite → Storage abstraction layer (not locked to SQLite)
- ⚠️ **Extend**: Add cloud storage backends (R2/S3)

---

### 2. Existing MCP Filesystem Servers

**Source**: Web research on MCP server implementations

#### Official Filesystem MCP Server (`@modelcontextprotocol/server-filesystem`)

**Installation**:
```json
{
  "mcpServers": {
    "filesystem": {
      "command": "npx",
      "args": [
        "-y",
        "@modelcontextprotocol/server-filesystem",
        "/path/to/allowed/files"
      ]
    }
  }
}
```

**Features**:
- POSIX-like file operations (read, write, list, delete)
- Directory access control (configurable via CLI args or Roots)
- Dynamic directory updates (via `roots/list_changed` notifications)
- Security: No access outside specified directories

**Limitations for RI**:
- ❌ Local filesystem only (no cloud storage)
- ❌ No versioning capabilities
- ❌ No audit trail
- ❌ No abstraction layer (tightly coupled to Node.js `fs`)

#### S3 MCP Servers

**Multiple implementations found**:

1. **@sanggggg/record-mcp**
   - Storage abstraction: local files OR Cloudflare R2
   - S3-compatible API for R2
   - Easy backend switching

2. **File-Store MCP**
   - Multi-backend: AWS S3, R2, MinIO, OSS, COS, Qiniu, GitHub
   - **Single interface for multiple cloud backends**
   - Critical for scalable agentic systems

3. **Geun-Oh/s3-mcp-server**
   - AWS S3 access via MCP
   - Streaming capabilities (large files, PDFs)
   - Efficient handling

**RI Architecture Implications**:
- ✅ **Proven**: MCP servers can abstract cloud storage
- ✅ **Validated**: R2 + S3 compatibility exists
- ⚠️ **Gap**: None provide AgentFS-style audit trail + versioning
- ✅ **Opportunity**: Combine best of AgentFS + File-Store MCP patterns

---

### 3. Model Context Protocol Specifications

**Source**: Context7 `/llmstxt/modelcontextprotocol_io_llms_txt`

**Key MCP Concepts for Storage**:

#### Resources
```typescript
interface Root {
  uri: string;           // file:///path/to/directory
  name?: string;        // "Travel Planning Workspace"
  _meta?: object;       // Custom metadata
}
```

**URI Template Pattern**:
```json
{
  "uriTemplate": "storage://{backend}/{bucket}/{path}",
  "name": "ri-storage",
  "title": "Reusable Intelligence Storage",
  "description": "Unified storage API for RI artifacts",
  "mimeType": "application/json"
}
```

#### Resources Capability
```typescript
resources?: {
  listChanged?: boolean;    // Notify when resources change
  subscribe?: boolean;      // Subscribe to resource updates
}
```

**RI Architecture Implications**:
- ✅ **Use**: URI templates for backend abstraction
  - `storage://local/specs/chapter-5/spec.md`
  - `storage://r2/panaversity-ri/specs/chapter-5/spec.md`
  - `storage://s3/my-bucket/specs/chapter-5/spec.md`
- ✅ **Use**: `listChanged` notifications for RI updates
- ✅ **Use**: Resource subscriptions for real-time intelligence sync

---

## Architectural Analysis

### Design Constraints

**From Constitution** (`.specify/memory/constitution.md`):

1. **Principle 5: Intelligence Accumulation** (v6.0.1)
   - Context-rich workflows (vertical accumulation)
   - Never start from zero context
   - Intelligence is primary artifact (code is regenerable)

2. **RI as Strategic Asset** (`docs/ri-architecture.md`)
   - Specifications → Executable contracts
   - Agent architectures → Reasoning frameworks
   - Skills → Bundled expertise
   - Organizational memory → Decision provenance

**Implications for Storage**:
- Intelligence must be **queryable** (not just files in directories)
- Intelligence must be **versionable** (track evolution over time)
- Intelligence must be **auditable** (who changed what, when, why)
- Intelligence must be **portable** (local ↔ cloud seamlessly)

---

### Comparison Matrix

| Feature | Local Filesystem | Official MCP FS | AgentFS | File-Store MCP | **Ideal RI Storage** |
|---------|------------------|-----------------|---------|----------------|----------------------|
| **Local storage** | ✅ | ✅ | ✅ | ✅ | ✅ |
| **Cloud storage (R2/S3)** | ❌ | ❌ | ❌ | ✅ | ✅ |
| **Storage abstraction** | ❌ | ❌ | ⚠️ (SQLite only) | ✅ | ✅ |
| **Audit trail** | ❌ | ❌ | ✅ | ❌ | ✅ |
| **Versioning** | ⚠️ (git only) | ❌ | ✅ | ❌ | ✅ |
| **Single-file portability** | ❌ | ❌ | ✅ | ❌ | ⚠️ (optional) |
| **MCP native** | ⚠️ (via official server) | ✅ | ❌ | ✅ | ✅ |
| **Queryable intelligence** | ❌ | ❌ | ✅ (SQL) | ❌ | ✅ |
| **Multi-backend** | ❌ | ❌ | ❌ | ✅ | ✅ |

**Conclusion**: No existing solution meets all requirements. Need custom implementation.

---

## Design Proposal

### Architecture Overview

```
┌──────────────────────────────────────────────────────────────┐
│ AI Agents (Claude, super-orchestra, content-implementer)     │
│ • Use MCP client to access storage                           │
│ • Unaware of underlying storage backend                      │
└──────────────────────────────────────────────────────────────┘
                          ↓ MCP Protocol
┌──────────────────────────────────────────────────────────────┐
│ RI Storage MCP Server (Node.js / TypeScript)                 │
│ • Implements MCP specification                               │
│ • Provides unified storage API                               │
│ • Handles audit logging                                      │
│ • Manages versioning (optional)                              │
└──────────────────────────────────────────────────────────────┘
                          ↓
┌──────────────────────────────────────────────────────────────┐
│ Storage Abstraction Layer                                    │
│ • Backend-agnostic interface                                 │
│ • Adapter pattern for each backend                           │
└──────────────────────────────────────────────────────────────┘
        ↓               ↓               ↓               ↓
┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│ Local FS     │ │ Cloudflare   │ │ AWS S3       │ │ Any S3-      │
│ Adapter      │ │ R2 Adapter   │ │ Adapter      │ │ compatible   │
└──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘
```

---

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

  // Batch operations
  readBatch(paths: string[]): Promise<Map<string, Buffer | string>>;
  writeBatch(files: Map<string, Buffer | string>): Promise<void>;

  // Audit operations (optional, backend-specific)
  getAuditLog?(path: string, options?: AuditOptions): Promise<AuditEntry[]>;
}

interface FileStat {
  path: string;
  size: number;
  modified: Date;
  isDirectory: boolean;
  contentType?: string;
  metadata?: Record<string, string>;
}

interface AuditEntry {
  timestamp: Date;
  operation: 'read' | 'write' | 'delete' | 'list';
  path: string;
  agent?: string;        // Which AI agent performed operation
  success: boolean;
  error?: string;
  metadata?: Record<string, unknown>;
}
```

---

#### 2. Backend Adapters

**Local Filesystem Adapter**:
```typescript
class LocalFilesystemAdapter implements RIStorageBackend {
  constructor(private rootPath: string) {}

  async read(path: string): Promise<string> {
    const fullPath = join(this.rootPath, path);
    return await fs.readFile(fullPath, 'utf-8');
  }

  async write(path: string, content: string): Promise<void> {
    const fullPath = join(this.rootPath, path);
    await fs.mkdir(dirname(fullPath), { recursive: true });
    await fs.writeFile(fullPath, content, 'utf-8');
  }

  // ... other methods
}
```

**Cloudflare R2 Adapter**:
```typescript
class CloudflareR2Adapter implements RIStorageBackend {
  constructor(
    private bucketName: string,
    private accountId: string,
    private accessKeyId: string,
    private secretAccessKey: string
  ) {
    // Initialize R2 client (S3-compatible)
    this.s3Client = new S3Client({
      region: 'auto',
      endpoint: `https://${accountId}.r2.cloudflarestorage.com`,
      credentials: {
        accessKeyId,
        secretAccessKey
      }
    });
  }

  async read(path: string): Promise<string> {
    const response = await this.s3Client.send(new GetObjectCommand({
      Bucket: this.bucketName,
      Key: path
    }));
    return await response.Body.transformToString();
  }

  async write(path: string, content: string): Promise<void> {
    await this.s3Client.send(new PutObjectCommand({
      Bucket: this.bucketName,
      Key: path,
      Body: content,
      ContentType: this.inferContentType(path)
    }));
  }

  // ... other methods
}
```

**AWS S3 Adapter**: Similar to R2 but with `region: 'us-east-1'` (or configurable)

**AgentFS-Inspired SQLite Adapter** (optional, for portability):
```typescript
class SQLiteStorageAdapter implements RIStorageBackend {
  constructor(private dbPath: string) {
    // Initialize SQLite database
    this.db = new Database(dbPath);
    this.initSchema();
  }

  private initSchema() {
    this.db.exec(`
      CREATE TABLE IF NOT EXISTS files (
        path TEXT PRIMARY KEY,
        content BLOB,
        content_type TEXT,
        modified TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        metadata JSON
      );

      CREATE TABLE IF NOT EXISTS audit_log (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        operation TEXT,
        path TEXT,
        agent TEXT,
        success BOOLEAN,
        error TEXT,
        metadata JSON
      );
    `);
  }

  async read(path: string): Promise<string> {
    const row = this.db.prepare('SELECT content FROM files WHERE path = ?').get(path);
    if (!row) throw new Error(`File not found: ${path}`);
    return row.content.toString('utf-8');
  }

  async write(path: string, content: string): Promise<void> {
    this.db.prepare(`
      INSERT INTO files (path, content, content_type, modified)
      VALUES (?, ?, ?, CURRENT_TIMESTAMP)
      ON CONFLICT(path) DO UPDATE SET
        content = excluded.content,
        modified = excluded.modified
    `).run(path, Buffer.from(content, 'utf-8'), this.inferContentType(path));

    // Log audit entry
    this.logAudit('write', path, true);
  }

  async getAuditLog(path: string): Promise<AuditEntry[]> {
    return this.db.prepare('SELECT * FROM audit_log WHERE path = ? ORDER BY timestamp DESC').all(path);
  }

  // Export entire database as single file for portability
  async export(outputPath: string): Promise<void> {
    await fs.copyFile(this.dbPath, outputPath);
  }

  // ... other methods
}
```

---

#### 3. MCP Server Implementation

```typescript
import { Server } from "@modelcontextprotocol/sdk/server/index.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";

class RIStorageMCPServer {
  private server: Server;
  private storage: RIStorageBackend;

  constructor(storage: RIStorageBackend) {
    this.storage = storage;
    this.server = new Server({
      name: "ri-storage",
      version: "1.0.0"
    }, {
      capabilities: {
        resources: {
          listChanged: true,    // Notify when RI changes
          subscribe: true       // Real-time updates
        },
        tools: {}
      }
    });

    this.setupHandlers();
  }

  private setupHandlers() {
    // Resource listing
    this.server.setRequestHandler("resources/list", async () => {
      const paths = await this.storage.list("/");
      return {
        resources: paths.map(path => ({
          uri: `ri://${path}`,
          name: path,
          mimeType: this.inferMimeType(path)
        }))
      };
    });

    // Resource reading
    this.server.setRequestHandler("resources/read", async (request) => {
      const uri = request.params.uri as string;
      const path = uri.replace("ri://", "");
      const content = await this.storage.read(path);

      return {
        contents: [{
          uri,
          mimeType: this.inferMimeType(path),
          text: content
        }]
      };
    });

    // Tools for writing, deleting, etc.
    this.server.setRequestHandler("tools/list", async () => ({
      tools: [
        {
          name: "write_file",
          description: "Write content to RI storage",
          inputSchema: {
            type: "object",
            properties: {
              path: { type: "string" },
              content: { type: "string" }
            },
            required: ["path", "content"]
          }
        },
        {
          name: "delete_file",
          description: "Delete file from RI storage",
          inputSchema: {
            type: "object",
            properties: {
              path: { type: "string" }
            },
            required: ["path"]
          }
        },
        {
          name: "get_audit_log",
          description: "Retrieve audit log for a file",
          inputSchema: {
            type: "object",
            properties: {
              path: { type: "string" },
              limit: { type: "number" }
            },
            required: ["path"]
          }
        }
      ]
    }));

    this.server.setRequestHandler("tools/call", async (request) => {
      const { name, arguments: args } = request.params;

      switch (name) {
        case "write_file":
          await this.storage.write(args.path, args.content);
          return { content: [{ type: "text", text: "File written successfully" }] };

        case "delete_file":
          await this.storage.delete(args.path);
          return { content: [{ type: "text", text: "File deleted successfully" }] };

        case "get_audit_log":
          if (!this.storage.getAuditLog) {
            throw new Error("Audit log not supported by this backend");
          }
          const log = await this.storage.getAuditLog(args.path, { limit: args.limit });
          return { content: [{ type: "text", text: JSON.stringify(log, null, 2) }] };

        default:
          throw new Error(`Unknown tool: ${name}`);
      }
    });
  }

  async run() {
    const transport = new StdioServerTransport();
    await this.server.connect(transport);
  }
}

// Usage
const backend = process.env.STORAGE_BACKEND || 'local';
let storage: RIStorageBackend;

switch (backend) {
  case 'local':
    storage = new LocalFilesystemAdapter(process.env.ROOT_PATH || process.cwd());
    break;
  case 'r2':
    storage = new CloudflareR2Adapter(
      process.env.R2_BUCKET!,
      process.env.R2_ACCOUNT_ID!,
      process.env.R2_ACCESS_KEY_ID!,
      process.env.R2_SECRET_ACCESS_KEY!
    );
    break;
  case 's3':
    storage = new AWSS3Adapter(/* ... */);
    break;
  case 'sqlite':
    storage = new SQLiteStorageAdapter(process.env.SQLITE_DB_PATH || './ri-storage.db');
    break;
  default:
    throw new Error(`Unknown storage backend: ${backend}`);
}

const server = new RIStorageMCPServer(storage);
server.run();
```

---

### Configuration

**Claude Desktop / Claude Code**:
```json
{
  "mcpServers": {
    "ri-storage": {
      "command": "node",
      "args": ["/path/to/ri-storage-server/dist/index.js"],
      "env": {
        "STORAGE_BACKEND": "r2",
        "R2_BUCKET": "panaversity-ri",
        "R2_ACCOUNT_ID": "your-account-id",
        "R2_ACCESS_KEY_ID": "your-access-key",
        "R2_SECRET_ACCESS_KEY": "your-secret-key"
      }
    }
  }
}
```

**For local development**:
```json
{
  "mcpServers": {
    "ri-storage": {
      "command": "node",
      "args": ["/path/to/ri-storage-server/dist/index.js"],
      "env": {
        "STORAGE_BACKEND": "local",
        "ROOT_PATH": "/Users/username/tutorsgpt/md"
      }
    }
  }
}
```

**For SQLite portability**:
```json
{
  "mcpServers": {
    "ri-storage": {
      "command": "node",
      "args": ["/path/to/ri-storage-server/dist/index.js"],
      "env": {
        "STORAGE_BACKEND": "sqlite",
        "SQLITE_DB_PATH": "./ri-intelligence.db"
      }
    }
  }
}
```

---

## Implementation Approaches

### Option 1: Separate Repository (Recommended)

**Repository Structure**:
```
ri-storage-mcp/
├── src/
│   ├── index.ts                    # MCP server entry point
│   ├── server.ts                   # MCP server implementation
│   ├── storage/
│   │   ├── interface.ts            # RIStorageBackend interface
│   │   ├── local-adapter.ts        # Local filesystem
│   │   ├── r2-adapter.ts           # Cloudflare R2
│   │   ├── s3-adapter.ts           # AWS S3
│   │   ├── sqlite-adapter.ts       # SQLite (AgentFS pattern)
│   │   └── adapter-factory.ts      # Factory for creating adapters
│   ├── audit/
│   │   ├── logger.ts               # Audit logging
│   │   └── query.ts                # Audit log querying
│   └── utils/
│       ├── mime-types.ts           # Content-Type inference
│       └── validation.ts           # Path validation
├── tests/
│   ├── local-adapter.test.ts
│   ├── r2-adapter.test.ts
│   └── mcp-server.test.ts
├── package.json
├── tsconfig.json
└── README.md
```

**Advantages**:
- ✅ **Reusable**: Can be used by ANY project needing storage abstraction
- ✅ **Modular**: Easy to add new backends (Azure Blob, GCS, etc.)
- ✅ **Testable**: Isolated testing of adapters
- ✅ **Publishable**: Can publish to npm as `@panaversity/ri-storage-mcp`
- ✅ **Versioned independently**: Storage API evolves separately from book content

**Disadvantages**:
- ⚠️ Adds dependency management overhead
- ⚠️ Requires coordination between repos

---

### Option 2: Integrated in Current Repository

**Directory Structure**:
```
tutorsgpt/md/
├── .claude/
├── .specify/
├── specs/
├── history/
├── ri-storage-mcp/          # New directory
│   ├── src/                 # (same as Option 1)
│   ├── tests/
│   └── package.json
└── package.json             # Root package.json (workspace)
```

**Advantages**:
- ✅ **Simpler**: Single repo, no external dependencies
- ✅ **Faster iteration**: Changes to storage + RI together
- ✅ **Co-located**: Documentation + implementation + usage in one place

**Disadvantages**:
- ❌ **Less reusable**: Harder for other projects to adopt
- ❌ **Coupling**: Storage tied to tutorsgpt project
- ❌ **Repo bloat**: Adds TypeScript build to book content repo

---

### Option 3: Hybrid Approach

**Phase 1: Prototype Integrated** (Option 2)
- Build quickly inside tutorsgpt repo
- Validate design with real RI workloads
- Iterate rapidly

**Phase 2: Extract to Separate Repo** (Option 1)
- Once design stabilizes, extract to `ri-storage-mcp` repo
- Publish to npm
- tutorsgpt becomes consumer (install from npm)

**Advantages**:
- ✅ **Best of both**: Fast prototyping + eventual reusability
- ✅ **Low risk**: Can stay integrated if extraction isn't worth it
- ✅ **Proven design**: Don't extract until it's battle-tested

---

## Integration with RI Architecture

### Constitutional Alignment

**From `.specify/memory/constitution.md` v6.0.1**:

#### Principle 5: Intelligence Accumulation (Section III)

**Current**:
> Before creating content, reason about: What accumulated intelligence exists that informs this work?
> - Constitutional governance (this document)
> - Domain structure (chapter-index.md, part-level progression)
> - Existing specifications (patterns from similar chapters)
> - Skills library (pedagogical and technical patterns)
> - Research foundation (library documentation, official sources)

**With Storage Abstraction**:
> Intelligence sources remain the same, but **storage backend is transparent**:
> - `ri://constitution.md` (could be local OR R2 OR S3)
> - `ri://chapter-index.md` (backend-agnostic)
> - `ri://specs/chapter-5/spec.md` (portable)
> - Agents unaware of underlying storage

---

### Agent Workflow Changes

**Before** (filesystem-bound):
```typescript
// super-orchestra agent
const constitutionPath = '.specify/memory/constitution.md';
const constitution = await fs.readFile(constitutionPath, 'utf-8');
```

**After** (storage-agnostic):
```typescript
// super-orchestra agent
const constitution = await mcp.readResource('ri://.specify/memory/constitution.md');
```

**Transparency**: Agents don't change logic, just API calls

---

### Audit Trail Integration

**Example Audit Query** (for educational-validator):
```typescript
// Check who modified constitution recently
const auditLog = await mcp.callTool('get_audit_log', {
  path: '.specify/memory/constitution.md',
  limit: 10
});

console.log(auditLog);
// [
//   {
//     timestamp: '2025-11-18T10:30:00Z',
//     operation: 'write',
//     path: '.specify/memory/constitution.md',
//     agent: 'super-orchestra',
//     success: true,
//     metadata: { version: '6.0.1', reason: 'Meta-commentary prohibition' }
//   },
//   ...
// ]
```

**Use Cases**:
- **Compliance**: Track which agent modified which RI artifact
- **Debugging**: "Why did this spec change between yesterday and today?"
- **Provenance**: Trace decision from ADR → Constitution → Implementation
- **Rollback**: Revert to previous version if needed (via versioning)

---

## Next Steps

### Phase 1: Research Validation (CURRENT)

- [x] Research AgentFS architecture
- [x] Identify existing MCP filesystem servers
- [x] Analyze MCP specification for storage patterns
- [ ] **Create ADR**: Document architectural decision (separate repo vs integrated)
- [ ] **Create specification**: `specs/ri-storage-mcp/spec.md`

### Phase 2: Prototype (Week 1-2)

- [ ] Implement `RIStorageBackend` interface
- [ ] Create Local Filesystem Adapter (baseline)
- [ ] Create Cloudflare R2 Adapter (primary cloud backend)
- [ ] Implement basic MCP server (resources + tools)
- [ ] Test with tutorsgpt RI workloads

### Phase 3: Audit & Versioning (Week 3)

- [ ] Add audit logging to all adapters
- [ ] Create SQLite adapter (AgentFS-inspired portability)
- [ ] Implement `get_audit_log` tool
- [ ] Test audit queries with super-orchestra workflow

### Phase 4: Integration (Week 4)

- [ ] Update `.claude/agents/` to use RI storage MCP
- [ ] Migrate critical workflows (chapter-planner, content-implementer)
- [ ] Update `docs/ri-architecture.md` with storage abstraction
- [ ] Document configuration for local, R2, S3 backends

### Phase 5: Production Deployment (Week 5-6)

- [ ] Deploy to Cloudflare R2 (production RI storage)
- [ ] Configure CI/CD for RI versioning
- [ ] Create backup/restore workflows
- [ ] Monitor audit logs for anomalies

### Phase 6: Open Source (Week 7+)

- [ ] Extract to `ri-storage-mcp` repository
- [ ] Publish to npm (`@panaversity/ri-storage-mcp`)
- [ ] Write comprehensive documentation
- [ ] Create examples for other projects

---

## Decision Required

**Question for User**:

Should we implement this as:

### A) Separate Repository (Recommended)
**Pros**: Reusable, modular, publishable to npm
**Cons**: Dependency management overhead

### B) Integrated in tutorsgpt/md
**Pros**: Faster iteration, simpler setup
**Cons**: Less reusable, repo bloat

### C) Hybrid (Prototype Integrated → Extract Later)
**Pros**: Best of both, low risk
**Cons**: Extraction work later

---

## Conclusion

This research demonstrates:

1. ✅ **Feasibility**: MCP filesystem servers with cloud storage backends exist and work
2. ✅ **Gap Identified**: No solution combines AgentFS audit trail + multi-backend abstraction
3. ✅ **Design Validated**: Three-layer architecture (MCP Server → Abstraction Layer → Adapters) is proven pattern
4. ✅ **Constitutional Alignment**: Storage abstraction preserves RI principles (Principle 5: Intelligence Accumulation)
5. ✅ **Agent Compatibility**: Minimal changes to existing agents (swap fs calls with MCP resource reads)

**Recommendation**: Proceed with **Option C (Hybrid)** — prototype integrated, extract if successful.

**Next Immediate Action**: Create ADR documenting this architectural decision.

---

**Document Metadata**:
- **Filename**: `docs/storage-abstraction-research.md`
- **Purpose**: Research findings + architectural design for RI storage abstraction
- **Audience**: Project maintainers, architects, future implementers
- **Related Documents**:
  - `docs/ri-architecture.md` (current RI architecture)
  - `.specify/memory/constitution.md` (governance framework)
  - AgentFS: https://github.com/tursodatabase/agentfs
  - MCP Specification: https://modelcontextprotocol.io

---

**END OF DOCUMENT**
