# MVP v2 Implementation Specification: Directory MCP Server

**Status**: Ready for Implementation (Updated with Quick Wins)
**Timeline**: 1 Day + 2 Hours (11 hours total)
**Risk Level**: LOW
**Reversibility**: HIGH (Git backup preserved)
**Version**: 2.0 (includes glob/grep search from LangChain inspiration)

---

## Executive Summary

This spec defines the **simplest possible implementation with essential search capabilities**:

**Problem**: 84 chapters locked in Git filesystem, agents cannot access efficiently
**Solution**: MCP server with pluggable storage (local + R2) + intelligent search
**Timeline**: 11 hours to production (was 9 hours, added 2 for search)
**LOC**: ~700 lines of TypeScript (was 600, added 100 for search)

**What's New in v2**:
- ✅ Added `glob_search` tool (find files by pattern)
- ✅ Added `grep_search` tool (search file contents)
- ✅ **7 tools** instead of 5 (better context engineering)

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
│  │ MCP Tools (7 total)                                    │ │
│  │ ├─ read_content(path)                                  │ │
│  │ ├─ write_content(path, content, agent_id)             │ │
│  │ ├─ list_contents(directory)                           │ │
│  │ ├─ delete_content(path, agent_id)                     │ │
│  │ ├─ glob_search(pattern) ← NEW                         │ │
│  │ ├─ grep_search(pattern, path, context) ← NEW          │ │
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
│   ├── search/                     # NEW
│   │   ├── glob.ts                 # Glob pattern matching
│   │   └── grep.ts                 # Content search
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
│   ├── search.test.ts              # NEW
│   └── audit.test.ts
│
└── config.json                     # Runtime configuration
```

---

## Implementation Details

### NEW: Search Tools

#### Tool 6: `glob_search` (`src/search/glob.ts`)

**Purpose**: Find files matching glob patterns (inspired by LangChain Deep Agents)

```typescript
import { minimatch } from 'minimatch';
import { StorageBackend } from '../storage/interface';

export class GlobSearcher {
  constructor(private storage: StorageBackend) {}

  /**
   * Find files matching glob pattern
   * Examples:
   *   "**/*.md" - All markdown files
   *   "01-Part/**" - All files in Part 1
   *   "**/chapter-5/**" - All files in any chapter-5
   */
  async search(pattern: string): Promise<string[]> {
    // Get all files from storage
    const allFiles = await this.storage.list('');

    // Filter using minimatch
    const matched = allFiles.filter(file => minimatch(file, pattern));

    return matched.sort();
  }
}
```

**MCP Tool Definition**:
```typescript
{
  name: 'glob_search',
  description: 'Find files matching a glob pattern (e.g., "**/*.md", "**/chapter-5/**")',
  inputSchema: {
    type: 'object',
    properties: {
      pattern: {
        type: 'string',
        description: 'Glob pattern to match files against'
      }
    },
    required: ['pattern']
  }
}
```

**Example Usage**:
```typescript
// Find all lessons in Chapter 5
await mcp.callTool({
  name: 'glob_search',
  arguments: { pattern: '**/05-chapter/**/*.md' }
});

// Returns: [
//   "01-Part/05-chapter/01-lesson.md",
//   "01-Part/05-chapter/02-lesson.md",
//   ...
// ]
```

**LOC**: 30

---

#### Tool 7: `grep_search` (`src/search/grep.ts`)

**Purpose**: Search file contents for text/regex patterns with context lines

```typescript
import { StorageBackend } from '../storage/interface';

export interface GrepMatch {
  file: string;
  lineNumber: number;
  line: string;
  contextBefore: string[];
  contextAfter: string[];
}

export class GrepSearcher {
  constructor(private storage: StorageBackend) {}

  /**
   * Search file contents for pattern
   * @param pattern - Text or regex to search for
   * @param path - Directory to search in (default: all files)
   * @param contextLines - Lines before/after match to include (default: 2)
   * @param isRegex - Treat pattern as regex (default: false)
   */
  async search(
    pattern: string,
    path: string = '',
    contextLines: number = 2,
    isRegex: boolean = false
  ): Promise<GrepMatch[]> {
    const results: GrepMatch[] = [];

    // Get files to search
    const files = await this.storage.list(path);
    const mdFiles = files.filter(f => f.endsWith('.md'));

    // Compile regex if needed
    const regex = isRegex
      ? new RegExp(pattern, 'gi')
      : new RegExp(pattern.replace(/[.*+?^${}()|[\]\\]/g, '\\$&'), 'gi');

    // Search each file
    for (const file of mdFiles) {
      const content = await this.storage.read(file);
      const lines = content.split('\n');

      lines.forEach((line, index) => {
        if (regex.test(line)) {
          results.push({
            file,
            lineNumber: index + 1,
            line: line.trim(),
            contextBefore: lines.slice(
              Math.max(0, index - contextLines),
              index
            ),
            contextAfter: lines.slice(
              index + 1,
              index + 1 + contextLines
            )
          });
        }
      });
    }

    return results;
  }
}
```

**MCP Tool Definition**:
```typescript
{
  name: 'grep_search',
  description: 'Search file contents for text or regex pattern with context',
  inputSchema: {
    type: 'object',
    properties: {
      pattern: {
        type: 'string',
        description: 'Text or regex pattern to search for'
      },
      path: {
        type: 'string',
        description: 'Directory to search in (default: all files)',
        default: ''
      },
      context_lines: {
        type: 'number',
        description: 'Lines of context before/after match (default: 2)',
        default: 2
      },
      is_regex: {
        type: 'boolean',
        description: 'Treat pattern as regex (default: false)',
        default: false
      }
    },
    required: ['pattern']
  }
}
```

**Example Usage**:
```typescript
// Find all mentions of "async/await"
await mcp.callTool({
  name: 'grep_search',
  arguments: {
    pattern: 'async/await',
    path: '04-Python-Fundamentals/',
    context_lines: 3
  }
});

// Returns:
// [
//   {
//     file: "04-Python-Fundamentals/29-asyncio/02-lesson.md",
//     lineNumber: 45,
//     line: "The `async/await` syntax provides...",
//     contextBefore: [
//       "",
//       "## Understanding Async/Await",
//       ""
//     ],
//     contextAfter: [
//       "a cleaner way to write asynchronous code.",
//       "",
//       "### Basic Example"
//     ]
//   }
// ]
```

**LOC**: 70

---

### Updated MCP Server (`src/mcp/server.ts`)

**Add search tool handlers**:

```typescript
import { GlobSearcher } from '../search/glob';
import { GrepSearcher } from '../search/grep';

export class DirectoryMCPServer {
  private server: Server;
  private globSearcher: GlobSearcher;
  private grepSearcher: GrepSearcher;

  constructor(
    private storage: StorageBackend,
    private audit: AuditLogger
  ) {
    this.server = new Server(/*...*/);
    this.globSearcher = new GlobSearcher(storage);
    this.grepSearcher = new GrepSearcher(storage);
    this.setupHandlers();
  }

  private setupHandlers() {
    // ... existing handlers ...

    this.server.setRequestHandler(CallToolRequestSchema, async (request) => {
      const { name, arguments: args } = request.params;

      switch (name) {
        // ... existing cases ...

        case 'glob_search': {
          const files = await this.globSearcher.search(args.pattern);
          await this.audit.log({
            operation: 'glob_search',
            path: args.pattern,
            agentId: 'system',
            metadata: { resultCount: files.length }
          });
          return {
            content: [{
              type: 'text',
              text: JSON.stringify(files, null, 2)
            }]
          };
        }

        case 'grep_search': {
          const matches = await this.grepSearcher.search(
            args.pattern,
            args.path || '',
            args.context_lines || 2,
            args.is_regex || false
          );
          await this.audit.log({
            operation: 'grep_search',
            path: args.path || '',
            agentId: 'system',
            metadata: {
              pattern: args.pattern,
              matchCount: matches.length
            }
          });
          return {
            content: [{
              type: 'text',
              text: JSON.stringify(matches, null, 2)
            }]
          };
        }

        default:
          throw new Error(`Unknown tool: ${name}`);
      }
    });
  }
}
```

**Additional LOC**: +30 (handler code)

---

## Dependencies (`package.json`)

**Updated with search library**:

```json
{
  "name": "directory-mcp-server",
  "version": "2.0.0",
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
    "@aws-sdk/client-s3": "^3.0.0",
    "minimatch": "^9.0.0"
  },
  "devDependencies": {
    "typescript": "^5.0.0",
    "@types/node": "^20.0.0",
    "@types/minimatch": "^5.0.0",
    "vitest": "^1.0.0"
  }
}
```

**New dependency**: `minimatch` (glob pattern matching, 0 dependencies, 50KB)

---

## Implementation Timeline (Updated)

### Phase 1: Setup (Hour 1-2) - UNCHANGED

**Tasks**:
- [ ] Create `directory-mcp-server/` repository
- [ ] Initialize TypeScript project (`npm init`, `tsc --init`)
- [ ] Install dependencies (added `minimatch`)
- [ ] Set up project structure

**Deliverable**: Project scaffold with dependencies installed

---

### Phase 2: Storage Layer (Hour 3-4) - UNCHANGED

**Tasks**:
- [ ] Implement `src/storage/interface.ts` (20 LOC)
- [ ] Implement `src/storage/local.ts` (80 LOC)
- [ ] Implement `src/storage/r2.ts` (80 LOC)
- [ ] Write unit tests

**Deliverable**: Storage abstraction with local + R2 backends working

---

### Phase 3: Audit Logger (Hour 4) - UNCHANGED

**Tasks**:
- [ ] Implement `src/audit/logger.ts` (50 LOC)
- [ ] Write unit tests

**Deliverable**: Audit logging functional

---

### Phase 4: Search Tools (Hour 5-6) - NEW

**Tasks**:
- [ ] Implement `src/search/glob.ts` (30 LOC)
- [ ] Implement `src/search/grep.ts` (70 LOC)
- [ ] Write unit tests for glob search
- [ ] Write unit tests for grep search

**Deliverable**: Glob and grep search working

**Test cases**:
```typescript
// Glob tests
expect(await glob.search('**/*.md')).toHaveLength(84);
expect(await glob.search('**/05-chapter/**')).toContain('01-Part/05-chapter/01-lesson.md');

// Grep tests
const results = await grep.search('async/await', '04-Python-Fundamentals/');
expect(results).toHaveLength(5);
expect(results[0].file).toContain('29-asyncio');
```

---

### Phase 5: MCP Server (Hour 7-8) - UPDATED

**Tasks**:
- [ ] Implement `src/mcp/server.ts` (180 LOC, was 150)
- [ ] Implement `src/config.ts` (20 LOC)
- [ ] Implement `src/index.ts` (40 LOC)
- [ ] Test with MCP Inspector (7 tools now, was 5)

**Deliverable**: MCP server running, all 7 tools callable

---

### Phase 6: Migration & Hydration (Hour 9-10) - UNCHANGED

**Tasks**:
- [ ] Implement `scripts/migrate-to-r2.ts` (50 LOC)
- [ ] Implement `scripts/hydrate-docusaurus.ts` (70 LOC)
- [ ] Test migration (dry-run)
- [ ] Test hydration (temp directory created)

**Deliverable**: Migration and hydration scripts working

---

### Phase 7: Integration & Deployment (Hour 11) - UPDATED

**Tasks**:
- [ ] Update `.github/workflows/deploy.yml` to run hydration before build
- [ ] Run migration for all 84 chapters
- [ ] Test Docusaurus build with hydrated content
- [ ] **Test search tools** with real queries
- [ ] Deploy to production
- [ ] Monitor audit log for first operations

**Deliverable**: **LIVE IN PRODUCTION** ✅

---

## Use Cases: Why Search Tools Matter

### Use Case 1: Finding Related Content

**Without search**:
```typescript
// Agent has to list ALL files, read ALL files
const files = await list_contents('');
// Returns 84 files
for (const file of files) {
  const content = await read_content(file);
  if (content.includes('async')) {
    // Found it!
  }
}
// 84 read operations, expensive!
```

**With grep_search**:
```typescript
// Direct search
const matches = await grep_search({
  pattern: 'async',
  path: '04-Python-Fundamentals/'
});
// Returns only matching files with context
// 1 operation, cheap!
```

---

### Use Case 2: Understanding Chapter Structure

**Without search**:
```typescript
// Agent has to manually navigate
const part1 = await list_contents('01-Part/');
const chapter5 = await list_contents('01-Part/05-chapter/');
// Multiple operations
```

**With glob_search**:
```typescript
// Get entire structure at once
const chapter5Files = await glob_search({
  pattern: '**/05-chapter/**'
});
// Returns all files in chapter 5, any part
// 1 operation
```

---

### Use Case 3: Finding Proficiency Levels

**Scenario**: Agent needs to find all A2-level lessons

**With grep_search**:
```typescript
const a2Lessons = await grep_search({
  pattern: 'proficiency: A2',
  context_lines: 1
});
// Returns:
// [
//   {
//     file: "01-Part/02-chapter/01-lesson.md",
//     lineNumber: 5,
//     line: "proficiency: A2",
//     contextBefore: ["---", "title: Understanding AI Tools"],
//     contextAfter: ["description: Learn about AI coding assistants", "---"]
//   }
// ]
```

---

## File Size Breakdown (Updated)

```
Total: ~700 LOC (was 600)

src/
├── index.ts                 40 LOC   │ Entry point
├── config.ts                20 LOC   │ Config loader
├── storage/
│   ├── interface.ts         20 LOC   │ Interface definition
│   ├── local.ts             80 LOC   │ Filesystem implementation
│   └── r2.ts                80 LOC   │ R2 implementation
├── mcp/
│   └── server.ts           180 LOC   │ MCP protocol + tools (was 150)
├── search/                            │ NEW
│   ├── glob.ts              30 LOC   │ Glob pattern matching
│   └── grep.ts              70 LOC   │ Content search
└── audit/
    └── logger.ts            50 LOC   │ Audit logging

scripts/
├── migrate-to-r2.ts         50 LOC   │ Git → R2 migration
└── hydrate-docusaurus.ts    70 LOC   │ R2 → Docusaurus hydration

tests/
├── storage.test.ts          30 LOC   │ Storage tests
├── search.test.ts           30 LOC   │ Search tests (NEW)
├── mcp.test.ts              30 LOC   │ MCP server tests
└── audit.test.ts            30 LOC   │ Audit logger tests
```

**Added**: 100 LOC for search functionality

---

## Testing Strategy (Updated)

### Search Tool Tests

**Glob Search**:
```typescript
import { describe, it, expect } from 'vitest';
import { GlobSearcher } from '../src/search/glob';
import { LocalStorage } from '../src/storage/local';

describe('GlobSearcher', () => {
  const storage = new LocalStorage('./test-content');
  const searcher = new GlobSearcher(storage);

  it('should find all markdown files', async () => {
    const files = await searcher.search('**/*.md');
    expect(files.length).toBeGreaterThan(0);
    expect(files.every(f => f.endsWith('.md'))).toBe(true);
  });

  it('should find files in specific chapter', async () => {
    const files = await searcher.search('**/05-chapter/**');
    expect(files.every(f => f.includes('05-chapter'))).toBe(true);
  });

  it('should handle pattern with no matches', async () => {
    const files = await searcher.search('**/nonexistent/**');
    expect(files).toHaveLength(0);
  });
});
```

**Grep Search**:
```typescript
describe('GrepSearcher', () => {
  const storage = new LocalStorage('./test-content');
  const searcher = new GrepSearcher(storage);

  it('should find text in files', async () => {
    const matches = await searcher.search('async/await');
    expect(matches.length).toBeGreaterThan(0);
    expect(matches[0].line).toContain('async/await');
  });

  it('should include context lines', async () => {
    const matches = await searcher.search('async/await', '', 2);
    expect(matches[0].contextBefore).toHaveLength(2);
    expect(matches[0].contextAfter).toHaveLength(2);
  });

  it('should support regex patterns', async () => {
    const matches = await searcher.search('async.*await', '', 1, true);
    expect(matches.length).toBeGreaterThan(0);
  });
});
```

---

## Performance Characteristics (Updated)

### Glob Search Performance

```
Pattern: "**/*.md" (find all markdown files)
    ↓
List all files from storage (R2: ~100ms)
    ↓
Filter with minimatch (~10ms for 84 files)
    ↓
Return sorted results

Total: ~110ms
```

### Grep Search Performance

```
Pattern: "async/await" in "04-Python-Fundamentals/"
    ↓
List files in directory (R2: ~50ms)
    ↓
Read each file in parallel (R2: ~200ms for 18 chapters)
    ↓
Search content with regex (~20ms)
    ↓
Return matches with context

Total: ~270ms (worst case)
Optimization: Cache file contents locally after first read
```

---

## Success Criteria (Updated)

**MVP Success (End of Hour 11)**:
- [ ] MCP server responds to tool calls in < 100ms
- [ ] All 84 chapters migrated to R2 without data loss
- [ ] **Glob search finds files correctly** (new)
- [ ] **Grep search finds content correctly** (new)
- [ ] Docusaurus build completes successfully
- [ ] Website deploys and renders correctly
- [ ] Audit log captures all operations (including searches)
- [ ] Agents can read/write via MCP

---

## What Changed from v1

| Aspect | v1 (Original MVP) | v2 (with Quick Wins) |
|--------|-------------------|----------------------|
| **Tools** | 5 | **7** (+glob, +grep) |
| **LOC** | ~600 | **~700** (+100 for search) |
| **Timeline** | 9 hours | **11 hours** (+2 for search) |
| **Dependencies** | 2 | **3** (+minimatch) |
| **Search Capability** | list_contents only | **glob + grep patterns** |
| **Context Engineering** | Basic | **Advanced (LangChain-inspired)** |

---

## External Dependencies (AgentFS, OpenDAL)

### Do We Need Them?

**AgentFS**: ❌ NO
- We're using their audit pattern (agent_id, timestamp)
- But implementing in TypeScript with JSONL (simpler)
- Don't need their SQLite dependency

**OpenDAL**: ❌ NO (for MVP)
- We're using their concept (unified storage)
- But implementing directly with AWS SDK (simpler)
- Can add later if we need 10+ backends

**minimatch**: ✅ YES
- Lightweight (50KB, 0 dependencies)
- Industry-standard glob matching
- Used by npm, webpack, many others

---

## Why Quick Wins Are Worth It

### 1. **Better Agent Experience**

**Before**:
```typescript
// Agent: "Find all lessons about Python loops"
// Has to:
1. list_contents('04-Python-Fundamentals/')
2. For each file: read_content(file)
3. Search manually in conversation context
// Result: 18 read operations, slow
```

**After**:
```typescript
// Agent: "Find all lessons about Python loops"
grep_search({
  pattern: 'for loop|while loop',
  path: '04-Python-Fundamentals/',
  is_regex: true
})
// Result: 1 operation, fast, with context
```

### 2. **Aligns with LangChain Best Practices**

From their post:
> "Filesystems provide an alternative to allow agents to intelligently search for context with ls, glob, and grep tools... there are situations where using filesystems (and the search capabilities you gain by using a filesystem) can create better results."

We're implementing exactly what they recommend.

### 3. **Only +2 Hours**

- Total time: 11 hours (still "deploy today")
- Added value: Significantly better agent UX
- Risk: Still LOW (minimatch is battle-tested)

---

## Approval Checklist (Updated)

Before proceeding, confirm:
- [ ] Architecture approved (7 tools instead of 5)
- [ ] Timeline acceptable (11 hours instead of 9)
- [ ] R2 credentials available
- [ ] Docusaurus integration approach acceptable
- [ ] Search tools (glob/grep) valuable
- [ ] Rollback plan satisfactory
- [ ] LOC estimate acceptable (~700 lines)

**Status**: ✅ **READY FOR IMPLEMENTATION**

---

**Total LOC Estimate (v2)**:
- Storage layer: 180 LOC
- Search layer: 100 LOC (NEW)
- Audit logger: 50 LOC
- MCP server: 180 LOC (updated)
- Config + Entry: 60 LOC
- Migration script: 50 LOC
- Hydration script: 70 LOC
- **TOTAL: ~690 LOC**

**Complexity**: LOW
**Risk**: LOW
**Reversibility**: HIGH
**Time to Production**: **11 HOURS** (same day)

**Let's ship it.** 🚀
