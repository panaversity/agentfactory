# MVP Implementation Specification: Directory MCP Server

**Status**: Ready for Implementation
**Timeline**: 1 Day (9 hours)
**Risk Level**: LOW
**Reversibility**: HIGH (Git backup preserved)

---

## Executive Summary

This spec defines the **simplest possible implementation** that solves the core business problem:

**Problem**: 84 chapters locked in Git filesystem, agents cannot access efficiently
**Solution**: MCP server with pluggable storage (local + R2)
**Timeline**: 1 day to production
**LOC**: ~600 lines of TypeScript

---

## Architecture

### System Diagram

```
┌─────────────────────────────────────────────────────────────┐
│  Consumers                                                   │
│  ├─ Claude Code (MCP Client) ────────┐                      │
│  ├─ Python Agents (MCP Client) ──────┤                      │
│  └─ Docusaurus (Hydration Script) ───┘                      │
└──────────────────────────────────────────────────────────────┘
                           │
                           ↓
┌──────────────────────────────────────────────────────────────┐
│  Directory MCP Server (TypeScript)                           │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ MCP Tools                                              │ │
│  │ ├─ read_content(path)                                  │ │
│  │ ├─ write_content(path, content, agent_id)             │ │
│  │ ├─ list_contents(directory)                           │ │
│  │ ├─ delete_content(path, agent_id)                     │ │
│  │ └─ get_audit_log(filters)                             │ │
│  └────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ Audit Logger (Append-only JSONL)                      │ │
│  └────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────┘
                           │
                           ↓
┌──────────────────────────────────────────────────────────────┐
│  Storage Abstraction Interface                               │
│  ┌──────────────────┐         ┌──────────────────┐          │
│  │  LocalStorage    │         │   R2Storage      │          │
│  │  (Development)   │         │  (Production)    │          │
│  └──────────────────┘         └──────────────────┘          │
└──────────────────────────────────────────────────────────────┘
                           │
                  ┌────────┴────────┐
                  ↓                 ↓
┌─────────────────────────┐  ┌─────────────────────────┐
│  Local Filesystem       │  │  Cloudflare R2          │
│  ./content/             │  │  s3://panaversity-book/ │
└─────────────────────────┘  └─────────────────────────┘
```

---

## File Structure

```
directory-mcp-server/
├── package.json
├── tsconfig.json
├── .env.example
├── README.md
│
├── src/
│   ├── index.ts                    # MCP server entry point
│   │
│   ├── storage/
│   │   ├── interface.ts            # StorageBackend interface
│   │   ├── local.ts                # LocalStorage implementation
│   │   └── r2.ts                   # R2Storage implementation
│   │
│   ├── mcp/
│   │   ├── server.ts               # MCP protocol handler
│   │   └── tools.ts                # Tool definitions
│   │
│   ├── audit/
│   │   └── logger.ts               # Append-only audit log
│   │
│   └── config.ts                   # Configuration loader
│
├── scripts/
│   ├── migrate-to-r2.ts            # Migration: Git → R2
│   ├── hydrate-docusaurus.ts       # Fetch from MCP → temp dir
│   └── test-connection.ts          # Verify MCP server works
│
├── tests/
│   ├── storage.test.ts
│   ├── mcp.test.ts
│   └── audit.test.ts
│
└── config.json                     # Runtime configuration
```

---

## Implementation Details

### 1. Storage Interface (`src/storage/interface.ts`)

**Purpose**: Define the contract all storage backends must implement

```typescript
/**
 * Storage backend interface
 * All implementations must support these operations
 */
export interface StorageBackend {
  /**
   * Read file content from storage
   * @throws {Error} if file does not exist
   */
  read(path: string): Promise<string>;

  /**
   * Write file content to storage
   * Creates parent directories if needed
   */
  write(path: string, content: string): Promise<void>;

  /**
   * List all files under a directory prefix
   * Returns full paths, not just filenames
   */
  list(prefix: string): Promise<string[]>;

  /**
   * Check if a file exists
   */
  exists(path: string): Promise<boolean>;

  /**
   * Delete a file
   * @throws {Error} if file does not exist
   */
  delete(path: string): Promise<void>;
}

/**
 * Storage configuration
 */
export interface StorageConfig {
  backend: 'local' | 'r2';
  local?: {
    rootPath: string;
  };
  r2?: {
    accountId: string;
    accessKeyId: string;
    secretAccessKey: string;
    bucket: string;
  };
}
```

**LOC**: 20

---

### 2. Local Storage (`src/storage/local.ts`)

**Purpose**: Filesystem-based storage for development

```typescript
import * as fs from 'fs/promises';
import * as path from 'path';
import { StorageBackend } from './interface';

export class LocalStorage implements StorageBackend {
  constructor(private rootPath: string) {}

  async read(filePath: string): Promise<string> {
    const fullPath = path.join(this.rootPath, filePath);
    return await fs.readFile(fullPath, 'utf-8');
  }

  async write(filePath: string, content: string): Promise<void> {
    const fullPath = path.join(this.rootPath, filePath);
    await fs.mkdir(path.dirname(fullPath), { recursive: true });
    await fs.writeFile(fullPath, content, 'utf-8');
  }

  async list(prefix: string): Promise<string[]> {
    const fullPath = path.join(this.rootPath, prefix);
    const files: string[] = [];

    async function walk(dir: string, base: string) {
      const entries = await fs.readdir(dir, { withFileTypes: true });
      for (const entry of entries) {
        const fullEntryPath = path.join(dir, entry.name);
        const relativePath = path.join(base, entry.name);

        if (entry.isDirectory()) {
          await walk(fullEntryPath, relativePath);
        } else {
          files.push(relativePath);
        }
      }
    }

    await walk(fullPath, '');
    return files;
  }

  async exists(filePath: string): Promise<boolean> {
    const fullPath = path.join(this.rootPath, filePath);
    try {
      await fs.access(fullPath);
      return true;
    } catch {
      return false;
    }
  }

  async delete(filePath: string): Promise<void> {
    const fullPath = path.join(this.rootPath, filePath);
    await fs.unlink(fullPath);
  }
}
```

**LOC**: 80

---

### 3. R2 Storage (`src/storage/r2.ts`)

**Purpose**: Cloudflare R2 (S3-compatible) storage for production

```typescript
import { S3Client, GetObjectCommand, PutObjectCommand, ListObjectsV2Command, HeadObjectCommand, DeleteObjectCommand } from '@aws-sdk/client-s3';
import { StorageBackend } from './interface';

export class R2Storage implements StorageBackend {
  private client: S3Client;

  constructor(
    private bucket: string,
    accountId: string,
    accessKeyId: string,
    secretAccessKey: string
  ) {
    this.client = new S3Client({
      region: 'auto',
      endpoint: `https://${accountId}.r2.cloudflarestorage.com`,
      credentials: {
        accessKeyId,
        secretAccessKey,
      },
    });
  }

  async read(path: string): Promise<string> {
    const command = new GetObjectCommand({
      Bucket: this.bucket,
      Key: path,
    });

    const response = await this.client.send(command);
    return await response.Body!.transformToString();
  }

  async write(path: string, content: string): Promise<void> {
    const command = new PutObjectCommand({
      Bucket: this.bucket,
      Key: path,
      Body: content,
      ContentType: 'text/markdown',
    });

    await this.client.send(command);
  }

  async list(prefix: string): Promise<string[]> {
    const command = new ListObjectsV2Command({
      Bucket: this.bucket,
      Prefix: prefix,
    });

    const response = await this.client.send(command);
    return (response.Contents || []).map(obj => obj.Key!);
  }

  async exists(path: string): Promise<boolean> {
    try {
      const command = new HeadObjectCommand({
        Bucket: this.bucket,
        Key: path,
      });
      await this.client.send(command);
      return true;
    } catch {
      return false;
    }
  }

  async delete(path: string): Promise<void> {
    const command = new DeleteObjectCommand({
      Bucket: this.bucket,
      Key: path,
    });

    await this.client.send(command);
  }
}
```

**LOC**: 80

---

### 4. Audit Logger (`src/audit/logger.ts`)

**Purpose**: Log all storage operations for tracking and debugging

```typescript
import * as fs from 'fs/promises';
import * as path from 'path';

export interface AuditEntry {
  timestamp: string;
  operation: 'read' | 'write' | 'delete' | 'list';
  path: string;
  agentId: string;
  metadata?: Record<string, unknown>;
}

export class AuditLogger {
  constructor(private logPath: string) {}

  async log(entry: Omit<AuditEntry, 'timestamp'>): Promise<void> {
    const fullEntry: AuditEntry = {
      ...entry,
      timestamp: new Date().toISOString(),
    };

    // Append to JSONL file (one JSON object per line)
    const line = JSON.stringify(fullEntry) + '\n';
    await fs.appendFile(this.logPath, line, 'utf-8');
  }

  async query(filters: {
    operation?: AuditEntry['operation'];
    agentId?: string;
    since?: string;
    limit?: number;
  }): Promise<AuditEntry[]> {
    const content = await fs.readFile(this.logPath, 'utf-8');
    const lines = content.split('\n').filter(l => l.trim());

    let entries: AuditEntry[] = lines.map(line => JSON.parse(line));

    // Apply filters
    if (filters.operation) {
      entries = entries.filter(e => e.operation === filters.operation);
    }
    if (filters.agentId) {
      entries = entries.filter(e => e.agentId === filters.agentId);
    }
    if (filters.since) {
      entries = entries.filter(e => e.timestamp >= filters.since);
    }
    if (filters.limit) {
      entries = entries.slice(-filters.limit);
    }

    return entries;
  }
}
```

**LOC**: 50

---

### 5. MCP Server (`src/mcp/server.ts`)

**Purpose**: Expose storage operations as MCP tools

```typescript
import { Server } from '@modelcontextprotocol/sdk/server/index.js';
import { StdioServerTransport } from '@modelcontextprotocol/sdk/server/stdio.js';
import { CallToolRequestSchema, ListToolsRequestSchema } from '@modelcontextprotocol/sdk/types.js';
import { StorageBackend } from '../storage/interface';
import { AuditLogger } from '../audit/logger';

export class DirectoryMCPServer {
  private server: Server;

  constructor(
    private storage: StorageBackend,
    private audit: AuditLogger
  ) {
    this.server = new Server(
      {
        name: 'directory-mcp-server',
        version: '1.0.0',
      },
      {
        capabilities: {
          tools: {},
        },
      }
    );

    this.setupHandlers();
  }

  private setupHandlers() {
    // List available tools
    this.server.setRequestHandler(ListToolsRequestSchema, async () => ({
      tools: [
        {
          name: 'read_content',
          description: 'Read file content from storage',
          inputSchema: {
            type: 'object',
            properties: {
              path: { type: 'string', description: 'File path (e.g., "01-Part/01-chapter/01-lesson.md")' },
            },
            required: ['path'],
          },
        },
        {
          name: 'write_content',
          description: 'Write file content to storage',
          inputSchema: {
            type: 'object',
            properties: {
              path: { type: 'string', description: 'File path' },
              content: { type: 'string', description: 'File content' },
              agent_id: { type: 'string', description: 'ID of the agent making the change' },
            },
            required: ['path', 'content', 'agent_id'],
          },
        },
        {
          name: 'list_contents',
          description: 'List all files under a directory',
          inputSchema: {
            type: 'object',
            properties: {
              prefix: { type: 'string', description: 'Directory prefix (e.g., "01-Part/")' },
            },
            required: ['prefix'],
          },
        },
        {
          name: 'delete_content',
          description: 'Delete a file from storage',
          inputSchema: {
            type: 'object',
            properties: {
              path: { type: 'string', description: 'File path' },
              agent_id: { type: 'string', description: 'ID of the agent making the change' },
            },
            required: ['path', 'agent_id'],
          },
        },
        {
          name: 'get_audit_log',
          description: 'Query audit log for operations',
          inputSchema: {
            type: 'object',
            properties: {
              operation: { type: 'string', enum: ['read', 'write', 'delete', 'list'] },
              agent_id: { type: 'string' },
              since: { type: 'string', description: 'ISO timestamp' },
              limit: { type: 'number', description: 'Max entries to return' },
            },
          },
        },
      ],
    }));

    // Handle tool calls
    this.server.setRequestHandler(CallToolRequestSchema, async (request) => {
      const { name, arguments: args } = request.params;

      switch (name) {
        case 'read_content': {
          const content = await this.storage.read(args.path);
          await this.audit.log({ operation: 'read', path: args.path, agentId: 'system' });
          return {
            content: [{ type: 'text', text: content }],
          };
        }

        case 'write_content': {
          await this.storage.write(args.path, args.content);
          await this.audit.log({ operation: 'write', path: args.path, agentId: args.agent_id });
          return {
            content: [{ type: 'text', text: `✓ Content written to ${args.path}` }],
          };
        }

        case 'list_contents': {
          const files = await this.storage.list(args.prefix);
          await this.audit.log({ operation: 'list', path: args.prefix, agentId: 'system' });
          return {
            content: [{ type: 'text', text: JSON.stringify(files, null, 2) }],
          };
        }

        case 'delete_content': {
          await this.storage.delete(args.path);
          await this.audit.log({ operation: 'delete', path: args.path, agentId: args.agent_id });
          return {
            content: [{ type: 'text', text: `✓ Deleted ${args.path}` }],
          };
        }

        case 'get_audit_log': {
          const entries = await this.audit.query(args);
          return {
            content: [{ type: 'text', text: JSON.stringify(entries, null, 2) }],
          };
        }

        default:
          throw new Error(`Unknown tool: ${name}`);
      }
    });
  }

  async start() {
    const transport = new StdioServerTransport();
    await this.server.connect(transport);
    console.error('Directory MCP Server running on stdio');
  }
}
```

**LOC**: 150

---

### 6. Configuration Loader (`src/config.ts`)

```typescript
import * as fs from 'fs/promises';
import { StorageConfig } from './storage/interface';

export interface Config {
  storage: StorageConfig;
  audit: {
    enabled: boolean;
    logPath: string;
  };
}

export async function loadConfig(path: string): Promise<Config> {
  const content = await fs.readFile(path, 'utf-8');
  return JSON.parse(content);
}
```

**LOC**: 20

---

### 7. Entry Point (`src/index.ts`)

```typescript
import { DirectoryMCPServer } from './mcp/server';
import { LocalStorage } from './storage/local';
import { R2Storage } from './storage/r2';
import { AuditLogger } from './audit/logger';
import { loadConfig } from './config';

async function main() {
  const config = await loadConfig('./config.json');

  // Initialize storage backend
  let storage;
  if (config.storage.backend === 'local') {
    storage = new LocalStorage(config.storage.local!.rootPath);
  } else if (config.storage.backend === 'r2') {
    const r2Config = config.storage.r2!;
    storage = new R2Storage(
      r2Config.bucket,
      r2Config.accountId,
      r2Config.accessKeyId,
      r2Config.secretAccessKey
    );
  } else {
    throw new Error(`Unknown storage backend: ${config.storage.backend}`);
  }

  // Initialize audit logger
  const audit = new AuditLogger(config.audit.logPath);

  // Start MCP server
  const server = new DirectoryMCPServer(storage, audit);
  await server.start();
}

main().catch(console.error);
```

**LOC**: 40

---

### 8. Migration Script (`scripts/migrate-to-r2.ts`)

**Purpose**: Copy all 84 chapters from Git to R2

```typescript
import * as fs from 'fs/promises';
import * as path from 'path';
import { R2Storage } from '../src/storage/r2';
import { loadConfig } from '../src/config';

async function migrate() {
  const config = await loadConfig('./config.json');
  const storage = new R2Storage(
    config.storage.r2!.bucket,
    config.storage.r2!.accountId,
    config.storage.r2!.accessKeyId,
    config.storage.r2!.secretAccessKey
  );

  const sourceDir = './book-source/docs';
  let fileCount = 0;

  async function uploadDirectory(dir: string, prefix: string) {
    const entries = await fs.readdir(dir, { withFileTypes: true });

    for (const entry of entries) {
      const sourcePath = path.join(dir, entry.name);
      const remotePath = path.join(prefix, entry.name);

      if (entry.isDirectory()) {
        await uploadDirectory(sourcePath, remotePath);
      } else if (entry.name.endsWith('.md')) {
        const content = await fs.readFile(sourcePath, 'utf-8');
        await storage.write(remotePath, content);
        fileCount++;
        console.log(`✓ Uploaded ${remotePath}`);
      }
    }
  }

  console.log('Starting migration...');
  await uploadDirectory(sourceDir, '');
  console.log(`✓ Migration complete: ${fileCount} files uploaded`);
}

migrate().catch(console.error);
```

**LOC**: 50

---

### 9. Docusaurus Hydration Script (`scripts/hydrate-docusaurus.ts`)

**Purpose**: Fetch content from MCP server and write to temp directory for Docusaurus build

```typescript
import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { StdioClientTransport } from '@modelcontextprotocol/sdk/client/stdio.js';
import * as fs from 'fs/promises';
import * as path from 'path';

async function hydrate() {
  // Connect to MCP server
  const transport = new StdioClientTransport({
    command: 'node',
    args: ['dist/index.js'],
  });

  const client = new Client({
    name: 'hydration-script',
    version: '1.0.0',
  }, {
    capabilities: {},
  });

  await client.connect(transport);

  // List all content
  const listResult = await client.callTool({
    name: 'list_contents',
    arguments: { prefix: '' },
  });

  const files = JSON.parse(listResult.content[0].text);

  // Fetch and write each file
  const outputDir = './book-source/docs-temp';
  await fs.mkdir(outputDir, { recursive: true });

  for (const filePath of files) {
    const readResult = await client.callTool({
      name: 'read_content',
      arguments: { path: filePath },
    });

    const content = readResult.content[0].text;
    const localPath = path.join(outputDir, filePath);

    await fs.mkdir(path.dirname(localPath), { recursive: true });
    await fs.writeFile(localPath, content, 'utf-8');

    console.log(`✓ Hydrated ${filePath}`);
  }

  console.log(`✓ Hydration complete: ${files.length} files`);
  await client.close();
}

hydrate().catch(console.error);
```

**LOC**: 70

---

## Configuration File

### `config.json`

```json
{
  "storage": {
    "backend": "r2",
    "local": {
      "rootPath": "./content"
    },
    "r2": {
      "accountId": "${R2_ACCOUNT_ID}",
      "accessKeyId": "${R2_ACCESS_KEY_ID}",
      "secretAccessKey": "${R2_SECRET_ACCESS_KEY}",
      "bucket": "panaversity-book"
    }
  },
  "audit": {
    "enabled": true,
    "logPath": "./audit.jsonl"
  }
}
```

### `.env.example`

```bash
R2_ACCOUNT_ID=your_account_id
R2_ACCESS_KEY_ID=your_access_key
R2_SECRET_ACCESS_KEY=your_secret_key
```

---

## Dependencies (`package.json`)

```json
{
  "name": "directory-mcp-server",
  "version": "1.0.0",
  "type": "module",
  "scripts": {
    "build": "tsc",
    "start": "node dist/index.js",
    "migrate": "node dist/scripts/migrate-to-r2.js",
    "hydrate": "node dist/scripts/hydrate-docusaurus.js",
    "test": "vitest"
  },
  "dependencies": {
    "@modelcontextprotocol/sdk": "^1.0.0",
    "@aws-sdk/client-s3": "^3.0.0"
  },
  "devDependencies": {
    "typescript": "^5.0.0",
    "@types/node": "^20.0.0",
    "vitest": "^1.0.0"
  }
}
```

---

## Implementation Timeline (1 Day)

### Phase 1: Setup (Hour 1-2)

**Tasks**:
- [ ] Create `directory-mcp-server/` repository
- [ ] Initialize TypeScript project (`npm init`, `tsc --init`)
- [ ] Install dependencies (`@modelcontextprotocol/sdk`, `@aws-sdk/client-s3`)
- [ ] Set up project structure (create directories)

**Deliverable**: Project scaffold with dependencies installed

---

### Phase 2: Storage Layer (Hour 3-4)

**Tasks**:
- [ ] Implement `src/storage/interface.ts` (20 LOC)
- [ ] Implement `src/storage/local.ts` (80 LOC)
- [ ] Implement `src/storage/r2.ts` (80 LOC)
- [ ] Write unit tests (`tests/storage.test.ts`)

**Deliverable**: Storage abstraction with local + R2 backends working

---

### Phase 3: Audit Logger (Hour 4)

**Tasks**:
- [ ] Implement `src/audit/logger.ts` (50 LOC)
- [ ] Write unit tests (`tests/audit.test.ts`)

**Deliverable**: Audit logging functional

---

### Phase 4: MCP Server (Hour 5-6)

**Tasks**:
- [ ] Implement `src/mcp/server.ts` (150 LOC)
- [ ] Implement `src/config.ts` (20 LOC)
- [ ] Implement `src/index.ts` (40 LOC)
- [ ] Test with MCP Inspector

**Deliverable**: MCP server running, tools callable

---

### Phase 5: Migration & Hydration (Hour 7-8)

**Tasks**:
- [ ] Implement `scripts/migrate-to-r2.ts` (50 LOC)
- [ ] Implement `scripts/hydrate-docusaurus.ts` (70 LOC)
- [ ] Test migration (dry-run)
- [ ] Test hydration (temp directory created)

**Deliverable**: Migration and hydration scripts working

---

### Phase 6: Integration & Deployment (Hour 9)

**Tasks**:
- [ ] Update `.github/workflows/deploy.yml` to run hydration before build
- [ ] Run migration for all 84 chapters
- [ ] Test Docusaurus build with hydrated content
- [ ] Deploy to production
- [ ] Monitor audit log for first operations

**Deliverable**: **LIVE IN PRODUCTION** ✅

---

## GitHub Actions Integration

### Update `.github/workflows/deploy.yml`

```yaml
name: Deploy Docusaurus

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      # NEW: Hydrate content from R2
      - name: Hydrate content
        working-directory: directory-mcp-server
        env:
          R2_ACCOUNT_ID: ${{ secrets.R2_ACCOUNT_ID }}
          R2_ACCESS_KEY_ID: ${{ secrets.R2_ACCESS_KEY_ID }}
          R2_SECRET_ACCESS_KEY: ${{ secrets.R2_SECRET_ACCESS_KEY }}
        run: |
          npm install
          npm run build
          npm run hydrate

      # Existing build process
      - name: Build Docusaurus
        working-directory: book-source
        run: |
          # Point Docusaurus to hydrated content
          rm -rf docs
          ln -s docs-temp docs
          npm install
          npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./book-source/build
```

---

## Testing Strategy

### Unit Tests

**Storage**:
```typescript
import { describe, it, expect } from 'vitest';
import { LocalStorage } from '../src/storage/local';

describe('LocalStorage', () => {
  it('should write and read content', async () => {
    const storage = new LocalStorage('./test-content');
    await storage.write('test.md', '# Hello');
    const content = await storage.read('test.md');
    expect(content).toBe('# Hello');
  });
});
```

**Audit**:
```typescript
import { AuditLogger } from '../src/audit/logger';

describe('AuditLogger', () => {
  it('should log operations', async () => {
    const logger = new AuditLogger('./test-audit.jsonl');
    await logger.log({ operation: 'read', path: 'test.md', agentId: 'test-agent' });
    const entries = await logger.query({ agentId: 'test-agent' });
    expect(entries.length).toBe(1);
  });
});
```

### Integration Test

```typescript
import { DirectoryMCPServer } from '../src/mcp/server';
import { LocalStorage } from '../src/storage/local';
import { AuditLogger } from '../src/audit/logger';

describe('MCP Server', () => {
  it('should handle read_content tool', async () => {
    const storage = new LocalStorage('./test-content');
    const audit = new AuditLogger('./test-audit.jsonl');
    const server = new DirectoryMCPServer(storage, audit);

    // Test tool execution
    // (requires MCP SDK test utilities)
  });
});
```

---

## Rollback Plan

**If something goes wrong**:

1. **Docusaurus build fails**:
   - Revert `.github/workflows/deploy.yml` to previous version
   - Redeploy from Git (old workflow)

2. **Migration corrupts data**:
   - Git is preserved as backup
   - Re-run migration from Git source

3. **R2 access fails**:
   - Switch `config.json` to `"backend": "local"`
   - Use local filesystem temporarily

4. **MCP server has bugs**:
   - Agents fall back to direct filesystem access (existing workflow)
   - Fix bugs, redeploy

**Key**: Git repository is NEVER deleted. It remains as authoritative backup.

---

## Success Metrics

**MVP Success Criteria**:
- [ ] MCP server responds to tool calls in < 100ms
- [ ] All 84 chapters migrated to R2 without data loss
- [ ] Docusaurus build completes successfully
- [ ] Website deploys and renders correctly
- [ ] Audit log captures all operations
- [ ] Agents can read/write via MCP

**Performance Targets**:
- Read latency: < 100ms (p95)
- Write latency: < 500ms (p95)
- Migration time: < 10 minutes for 84 chapters

---

## Next Steps (Post-MVP)

**Week 2-3 (Phase 2)**:
- Add directory schemas (book.yaml)
- Add content indexing (in-memory search)
- Add `search_content` tool (keyword search)

**Month 2 (Phase 3)**:
- Add LanceDB vector storage
- Add semantic search
- Add watcher agents (auto-summaries)

**Month 3 (Phase 4)**:
- Add REST API (for web apps)
- Add GraphQL API (for complex queries)
- Add WebSocket (for real-time updates)

---

## Approval Checklist

Before proceeding, confirm:
- [ ] Architecture approved
- [ ] Timeline acceptable (1 day)
- [ ] R2 credentials available
- [ ] Docusaurus integration approach acceptable
- [ ] Rollback plan satisfactory
- [ ] LOC estimate acceptable (~600 lines)

**Status**: ✅ **READY FOR IMPLEMENTATION**

---

**Total LOC Estimate**:
- Storage layer: 180 LOC
- Audit logger: 50 LOC
- MCP server: 150 LOC
- Config + Entry: 60 LOC
- Migration script: 50 LOC
- Hydration script: 70 LOC
- **TOTAL: ~560 LOC**

**Complexity**: LOW
**Risk**: LOW
**Reversibility**: HIGH
**Time to Production**: 1 DAY

**Let's ship it.** 🚀
