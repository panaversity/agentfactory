# PanaversityFS: Audit Trail & Versioning Architecture

**Date**: 2025-12-09
**Status**: ✅ **AUDIT ENABLED** | ⚠️ **R2 VERSIONING OPTIONAL**

---

## Executive Summary

Your system has **3 layers of tracking** for assets and content:

1. **FileJournal**: Current state snapshot (what's deployed NOW)
2. **AuditLog**: Complete history with hash chains (every change ever made)
3. **ManifestSnapshot**: Point-in-time book states (rollback capability)

**Bonus**: Cloudflare R2 can store old versions if you enable versioning.

---

## Layer 1: FileJournal (Current State)

### Purpose
**Single source of truth** for "what files exist RIGHT NOW"

### Schema
```sql
CREATE TABLE file_journal (
    book_id VARCHAR(255),
    path VARCHAR(1024),           -- e.g. "static/images/logo.png"
    user_id VARCHAR(255),          -- "__base__" for canonical content
    sha256 VARCHAR(64),            -- Current content hash
    last_written_at TIMESTAMP,
    storage_backend VARCHAR(50),   -- "s3" for Cloudflare R2
    PRIMARY KEY (book_id, path, user_id)
);
```

### Behavior
| Operation | FileJournal Action | Result |
|-----------|-------------------|--------|
| **Add asset** | INSERT new row | ✅ Tracked |
| **Update asset** | UPDATE sha256 hash | ✅ Hash updated (old hash lost) |
| **Rename asset** | INSERT new row | ⚠️ Old entry remains (residual) |
| **Delete asset** | No action | ⚠️ Entry remains (residual) |

### Key Point
**FileJournal tracks CURRENT state ONLY**. No history preserved here.

---

## Layer 2: AuditLog (Complete History)

### Purpose
**Append-only audit trail** with cryptographic hash chains

### Schema
```sql
CREATE TABLE audit_log (
    id SERIAL PRIMARY KEY,
    timestamp TIMESTAMP,
    agent_id VARCHAR(255),         -- Who made the change
    operation VARCHAR(50),         -- create, update, delete, read
    book_id VARCHAR(255),
    path VARCHAR(1024),
    user_id VARCHAR(255),
    prev_hash VARCHAR(64),         -- Hash BEFORE this operation
    new_hash VARCHAR(64),          -- Hash AFTER this operation
    status VARCHAR(50),            -- success, error
    error_message TEXT,
    execution_time_ms INTEGER
);
```

### Hash Chain Integrity
```
Entry 1: logo.png  | prev: NULL    → new: abc123... (create)
Entry 2: logo.png  | prev: abc123  → new: def456... (update)
Entry 3: logo.png  | prev: def456  → new: NULL     (delete)
```

**Invariant R6**: `entry[n].new_hash == entry[n+1].prev_hash`
- Enables tamper detection
- Proves operation sequence

### What Gets Logged
✅ **Every operation** on assets and markdown:
- `upload_asset` → logs "create" or "update"
- `write_content` → logs "create" or "update"
- `delete_content` → logs "delete"
- `read_content` → logs "read"

### Audit Trail for Rename
**Scenario**: Rename `logo.png` → `banner.png`

AuditLog will show:
```sql
-- Operation 1: Upload banner.png (treated as NEW file)
timestamp: 2025-12-09 14:05:00
operation: upload_asset
path: static/images/banner.png
prev_hash: NULL
new_hash: e96ff0188ec31e51...
status: success
```

**Result**: You get audit trail showing banner.png was created, but NO explicit link showing it's a rename of logo.png.

**Why**: Rename is filesystem operation. MCP sees it as:
1. Upload new file (`banner.png`)
2. Old file (`logo.png`) no longer synced

### Audit Trail for Update
**Scenario**: Update `diagram.png` content

AuditLog will show:
```sql
-- Operation 1: Initial upload
timestamp: 2025-12-09 14:01:00
operation: upload_asset
path: static/images/diagram.png
prev_hash: NULL
new_hash: ebf4f635a17d10d6...

-- Operation 2: Content update
timestamp: 2025-12-09 14:06:00
operation: upload_asset
path: static/images/diagram.png
prev_hash: ebf4f635a17d10d6...  ← Links to previous version
new_hash: be5f157f18ada8fc...
```

**Result**: Complete history showing content evolved from hash1 → hash2.

---

## Layer 3: ManifestSnapshot (Point-in-Time States)

### Purpose
**Capture complete book state** at intervals for:
- Delta build computation
- Rollback capability
- Compliance audits

### Schema
```sql
CREATE TABLE manifest_snapshot (
    manifest_hash VARCHAR(64) PRIMARY KEY,  -- Hash of entire manifest
    book_id VARCHAR(255),
    created_at TIMESTAMP,
    file_count INTEGER,
    content_json TEXT  -- JSON: {"path": "sha256", ...}
);
```

### Example Snapshot
```json
{
  "content/01-Part/01-Chapter/intro.md": "abc123...",
  "static/images/logo.png": "def456...",
  "static/images/diagram.png": "ghi789..."
}
```

### Use Cases
1. **Delta Detection**: Compare current FileJournal to last snapshot → find changes
2. **Rollback**: Restore FileJournal to match historical snapshot
3. **Compliance**: Prove book state at specific date/time

---

## R2 Storage Versioning (Optional)

### Default Behavior
**Uploads OVERWRITE previous versions**:
```
Upload logo.png (v1) → R2 stores: logo.png
Upload logo.png (v2) → R2 stores: logo.png (v1 deleted!)
```

### Enable Versioning (Cloudflare Dashboard)
1. Go to R2 dashboard → Your bucket
2. Settings → Enable Versioning
3. Now ALL versions preserved:
   ```
   Upload logo.png (v1) → R2 stores: logo.png (versionId: abc)
   Upload logo.png (v2) → R2 stores: logo.png (versionId: def)
   ```

### Benefits
✅ Recover deleted assets
✅ Rollback to previous versions
✅ Full history in storage layer

### Costs
⚠️ Storage costs increase (all versions stored)
⚠️ Requires lifecycle policies to prune old versions

---

## Answering Your Questions

### Q1: "Does rename give us audit trail?"

**Answer**: ✅ **Partial audit trail**

**What you get**:
- AuditLog shows `banner.png` created
- FileJournal has both `logo.png` (residual) and `banner.png`
- ManifestSnapshot (before) shows `logo.png`, (after) shows `banner.png`

**What you DON'T get**:
- No explicit "renamed from" link in database
- Must infer rename by comparing snapshots or FileJournal residuals

**Workaround**: Compare manifests to detect renames:
```python
old_files = {"static/images/logo.png": "abc123"}
new_files = {"static/images/banner.png": "abc123"}
# Same hash → likely rename
```

---

### Q2: "Do we keep audit trail and old versions in R2?"

**Answer**: ✅ **YES for audit trail** | ⚠️ **OPTIONAL for R2 versions**

#### Audit Trail (Database)
✅ **Always enabled** - every operation logged forever:
- AuditLog is append-only (no deletes)
- Hash chains prevent tampering
- Complete provenance for compliance

#### Old Versions (R2 Storage)
⚠️ **Disabled by default** - must enable versioning:
- Current: Uploads overwrite previous versions
- With versioning: All versions preserved in R2
- Trade-off: Storage costs vs recovery capability

---

## Current System Behavior

### Scenario: Upload → Rename → Update → Delete

| Step | FileJournal | AuditLog | R2 Storage (no versioning) | R2 Storage (with versioning) |
|------|-------------|----------|----------------------------|------------------------------|
| Upload `logo.png` | `logo.png`: hash1 | Entry 1: create, hash1 | `logo.png` (v1) | `logo.png` (versionId: a) |
| Rename to `banner.png` | `logo.png`: hash1<br>`banner.png`: hash1 | Entry 2: create banner, hash1 | `logo.png` (v1)<br>`banner.png` (v1) | `logo.png` (versionId: a)<br>`banner.png` (versionId: b) |
| Update `banner.png` | `logo.png`: hash1<br>`banner.png`: hash2 | Entry 3: update banner, hash1→hash2 | `logo.png` (v1)<br>`banner.png` (v2, v1 deleted) | `logo.png` (versionId: a)<br>`banner.png` (versionId: b, c) |
| Delete `banner.png` | `logo.png`: hash1<br>`banner.png`: hash2 | Entry 4: delete banner | `logo.png` (v1)<br>`banner.png` (deleted) | `logo.png` (versionId: a)<br>`banner.png` (versionId: b, c, d-deleted) |

---

## Residual Data Management

### Problem
Renames and deletes leave **residual entries** in FileJournal:
- `logo.png` still listed after rename to `banner.png`
- `banner.png` still listed after deletion

### Solutions

#### Option 1: Keep Residuals (Current)
**Pros**:
- Simple implementation
- Audit trail shows "what was here before"
- Can detect renames by comparing hashes

**Cons**:
- FileJournal grows over time
- "Active files" count includes deleted files

#### Option 2: Garbage Collection (Future)
**After each sync**:
1. Query FileJournal for book_id
2. Compare DB paths vs source directory paths
3. Delete FileJournal entries not in source
4. Optionally delete from R2 (storage cleanup)

**Trade-off**: Removes residuals but loses "deleted file" visibility in FileJournal.

---

## Recommendations

### For Your Use Case (Educational Content)

**Current setup is GOOD**:
1. ✅ Assets tracked in FileJournal (incremental builds work)
2. ✅ Full audit trail in AuditLog (compliance)
3. ✅ ManifestSnapshot for rollback capability
4. ⚠️ R2 versioning OFF (save storage costs)
5. ⚠️ Residuals accumulate (low impact for books)

**When assets rarely rename/delete**:
- Residual impact: Minimal
- Audit trail: Complete via AuditLog
- Storage costs: Low (no R2 versioning)

**Future enhancements** (if needed):
1. Add garbage collection for residuals
2. Enable R2 versioning for critical assets
3. Add rename detection (hash-based comparison)

---

## SQL Queries for Audit Analysis

### Query 1: Asset History (All Changes)
```sql
SELECT
    timestamp,
    operation,
    LEFT(prev_hash, 12) || '...' as prev_hash,
    LEFT(new_hash, 12) || '...' as new_hash,
    agent_id
FROM audit_log
WHERE book_id = 'your-book-id'
  AND path = 'static/images/logo.png'
ORDER BY timestamp;
```

### Query 2: Detect Renames (Same Hash, Different Path)
```sql
-- Find assets with same hash but different paths
SELECT
    f1.path as old_path,
    f2.path as new_path,
    f1.sha256,
    f1.last_written_at as old_time,
    f2.last_written_at as new_time
FROM file_journal f1
JOIN file_journal f2
  ON f1.book_id = f2.book_id
  AND f1.sha256 = f2.sha256
  AND f1.path != f2.path
WHERE f1.book_id = 'your-book-id'
  AND f1.path LIKE 'static/%'
  AND f2.path LIKE 'static/%'
  AND f2.last_written_at > f1.last_written_at;
```

### Query 3: Residual Assets (In DB, Not in Manifest)
```sql
-- Assets in FileJournal but not in latest ManifestSnapshot
SELECT fj.path, fj.sha256
FROM file_journal fj
LEFT JOIN manifest_snapshot ms ON ms.book_id = fj.book_id
WHERE fj.book_id = 'your-book-id'
  AND fj.path LIKE 'static/%'
  AND ms.content_json NOT LIKE '%' || fj.path || '%'
ORDER BY fj.last_written_at DESC;
```

---

## Summary Table

| Feature | FileJournal | AuditLog | ManifestSnapshot | R2 Versioning |
|---------|-------------|----------|------------------|---------------|
| **Purpose** | Current state | Complete history | Point-in-time snapshots | Storage-level versions |
| **Add** | ✅ Tracked | ✅ Logged | ✅ Next snapshot | ✅ Stored |
| **Update** | ✅ Hash updated | ✅ Logged (prev→new) | ✅ Next snapshot | ✅/⚠️ Version kept/overwritten |
| **Rename** | ⚠️ New entry (residual) | ✅ Logged as new file | ✅ Shows change | ✅/⚠️ Both versions/overwrite |
| **Delete** | ⚠️ Entry remains | ✅ Logged | ✅ Shows removal | ✅/❌ Marked deleted/gone |
| **Audit Trail** | ❌ Current only | ✅ Full history | ⚠️ Snapshots only | ⚠️ If versioning ON |
| **Residual Data** | ⚠️ Accumulates | ✅ Append-only (intentional) | ❌ Clean snapshots | ⚠️ Lifecycle policies |
| **Storage Cost** | Low (metadata) | Low (metadata) | Low (JSON) | High (if versioning ON) |

---

## Conclusion

Your question revealed a **critical architectural insight**:

1. **Audit trail**: ✅ **FULLY IMPLEMENTED** via AuditLog
   - Every operation logged with hash chains
   - Tamper-proof append-only design
   - Compliance-ready

2. **Old versions in R2**: ⚠️ **OPTIONAL** (disabled by default)
   - Currently: Uploads overwrite
   - Enable in Cloudflare dashboard if needed
   - Trade-off: Recovery vs storage costs

3. **Residual data**: ⚠️ **CURRENT LIMITATION**
   - Renames/deletes leave entries in FileJournal
   - Low impact for book content (rare renames/deletes)
   - Can add garbage collection later if needed

**Bottom line**: Your system has excellent audit capabilities. The asset cache fix you implemented ensures **reliable incremental builds** with **full audit trail** and **optional R2 versioning** for recovery scenarios.
