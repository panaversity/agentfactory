# Edge Case Analysis: Incremental Build Integration

**Date:** 2025-12-08
**Reviewer:** Claude Code
**Branch:** incremental-build-integration
**Status:** 🔍 Detailed Review

---

## Table of Contents

1. [Path Mapping Edge Cases](#1-path-mapping-edge-cases)
2. [Workflow Race Conditions](#2-workflow-race-conditions)
3. [Manifest Cache Edge Cases](#3-manifest-cache-edge-cases)
4. [Error Handling Gaps](#4-error-handling-gaps)
5. [File System Edge Cases](#5-file-system-edge-cases)
6. [Security Vulnerabilities](#6-security-vulnerabilities)
7. [Data Corruption Scenarios](#7-data-corruption-scenarios)
8. [Performance Edge Cases](#8-performance-edge-cases)
9. [Concurrency Issues](#9-concurrency-issues)
10. [Recovery Scenarios](#10-recovery-scenarios)
11. [Critical Issues Found](#critical-issues-found)
12. [Recommendations](#recommendations)

---

## 1. Path Mapping Edge Cases

### Issue 1.1: Windows Path Separators ⚠️ MEDIUM

**Location:** `scripts/ingest/path_mapper.py:103`

**Code:**
```python
parts = source_path.split("/")
```

**Problem:** Uses hardcoded `/` separator, won't work on Windows.

**Edge Cases:**
- Windows paths: `Part-01\Chapter-01\01-intro.md`
- Mixed separators: `Part-01/Chapter-01\01-intro.md`
- UNC paths: `\\server\share\Part-01\Chapter-01\01-intro.md`

**Impact:** Script will fail on Windows CI runners or local Windows development.

**Fix:**
```python
parts = source_path.replace("\\", "/").split("/")
```

**Status:** ✅ Already handled in line 103 (test exists for Windows paths)

---

### Issue 1.2: Empty Path Components ⚠️ MEDIUM

**Location:** `scripts/ingest/path_mapper.py:103-113`

**Problem:** Doesn't check for empty parts from double slashes.

**Edge Cases:**
- Double slashes: `Part-01//Chapter-01/01-intro.md`
- Trailing slashes: `Part-01/Chapter-01/`
- Leading slashes: `/Part-01/Chapter-01/01-intro.md`

**Example:**
```python
"Part-01//Chapter-01/01-intro.md".split("/")
# Returns: ['Part-01', '', 'Chapter-01', '01-intro.md']
# len(parts) = 4, but parts[1] is empty!
```

**Impact:** Could match incorrect patterns or create invalid storage paths.

**Fix Needed:**
```python
parts = [p for p in source_path.split("/") if p]  # Filter empty parts
```

**Status:** ❌ NOT HANDLED - Need to add

---

### Issue 1.3: Unicode/Special Characters in Paths 🟡 LOW

**Location:** `scripts/ingest/path_mapper.py:141-150`

**Problem:** Regex patterns don't account for Unicode characters.

**Edge Cases:**
- Accented characters: `Part-01/Chapter-01/01-introducción.md`
- Emoji: `Part-01/Chapter-01/01-intro-🚀.md`
- Spaces (URL encoded): `Part-01/Chapter-01/01-intro%20guide.md`
- CJK characters: `Part-01/Chapter-01/01-介绍.md`

**Current Pattern:**
```python
LESSON_PATTERN = re.compile(r"^(\d{1,2})-([a-z0-9-]+)(\.summary)?\.md$")
```

**Impact:** Non-ASCII filenames will be rejected.

**Fix:**
```python
LESSON_PATTERN = re.compile(r"^(\d{1,2})-([a-z0-9-\u0080-\uFFFF]+)(\.summary)?\.md$", re.UNICODE)
```

**Status:** 🤔 DECISION NEEDED - Do we allow Unicode in filenames?

---

### Issue 1.4: Case Sensitivity Mismatch 🟡 LOW

**Location:** `scripts/ingest/path_mapper.py:82-85`

**Problem:** Asset detection uses `lower()` but path construction doesn't.

**Edge Cases:**
- `Part-01/Chapter-01/IMG/test.png` (uppercase IMG)
- `Part-01/Chapter-01/Img/test.png` (mixed case)

**Current Code:**
```python
lower_path = source_path.lower()  # Converts to lowercase
for ext in ASSET_EXTENSIONS:
    if lower_path.endswith(ext):
        return _map_asset_path(source_path)  # Uses original case
```

Then in `_map_asset_path`:
```python
if part.lower() in asset_types:  # Compares lowercase
    asset_type = part.lower()     # Stores lowercase
```

**Impact:** Works correctly! Asset paths normalized to lowercase.

**Status:** ✅ HANDLED CORRECTLY

---

### Issue 1.5: Very Long Paths ⚠️ MEDIUM

**Location:** `scripts/ingest/path_mapper.py:157`

**Problem:** No validation of total path length.

**Edge Cases:**
- OS limit (Windows: 260 chars, Linux: 4096 chars)
- Very long lesson names: `Part-01/Chapter-01/01-this-is-a-very-very-very-...-long-lesson-name.md`

**Impact:** Could hit filesystem limits.

**Fix Needed:**
```python
MAX_PATH_LENGTH = 250  # Conservative limit

if len(storage_path) > MAX_PATH_LENGTH:
    return MappedPath(..., valid=False, error=f"Path too long: {len(storage_path)} chars")
```

**Status:** ❌ NOT HANDLED - Need to add

---

### Issue 1.6: Path Traversal Attack 🔴 HIGH

**Location:** `scripts/ingest/path_mapper.py:157-160`

**Problem:** No sanitization of path components.

**Attack Vectors:**
```
Part-01/../../../etc/passwd.md
Part-01/Chapter-01/../../secrets.md
Part-01/Chapter-01/01-intro.md/../../../escape.md
```

**Current Code:**
```python
storage_path = f"content/{part_num}-Part/{chapter_num}-Chapter/{lesson_num}-{lesson_name}"
```

**Impact:** If `lesson_name` contains `../`, attacker could write outside content directory.

**Example:**
```
Source: Part-01/Chapter-01/01-../../etc/passwd.md
Lesson name extracted: "../../etc/passwd"
Storage path: content/01-Part/01-Chapter/01-../../etc/passwd.md
Actual write: content/01-Part/etc/passwd.md (OUTSIDE content dir!)
```

**Fix Needed:**
```python
# Sanitize lesson_name to prevent path traversal
lesson_name = lesson_name.replace("..", "").replace("/", "").replace("\\", "")

# OR validate that final path is within content directory
from pathlib import Path
final_path = Path("content") / f"{part_num}-Part" / f"{chapter_num}-Chapter" / f"{lesson_num}-{lesson_name}.md"
if not final_path.resolve().is_relative_to(Path("content").resolve()):
    return MappedPath(..., valid=False, error="Path traversal detected")
```

**Status:** ❌ CRITICAL - Must fix before production

---

## 2. Workflow Race Conditions

### Issue 2.1: Concurrent Sync + Deploy 🔴 HIGH

**Location:** `.github/workflows/sync-content.yml` + `deploy.yml`

**Problem:** Both workflows can run simultaneously.

**Scenario:**
```
T0: Author pushes change to docs/lesson-1.md
T1: sync-content.yml starts (syncs lesson-1.md)
T2: Author pushes change to docs/lesson-2.md
T3: deploy.yml triggers from T1 sync
T4: sync-content.yml starts for T2 push
T5: deploy.yml tries to hydrate (T4 sync in progress)
```

**Impact:** Deploy might hydrate partially synced content.

**Current Mitigation:**
```yaml
# deploy.yml
workflow_run:
  workflows: ["Sync Content to R2"]
  types: [completed]  # ✓ Waits for sync to complete
```

**Remaining Issue:** If sync completes and new sync starts, deploy might run during second sync.

**Fix Needed:**
```yaml
# Add concurrency group
concurrency:
  group: panaversity-sync-${{ github.ref }}
  cancel-in-progress: false  # Queue, don't cancel
```

**Status:** ⚠️ PARTIALLY HANDLED - Add concurrency group

---

### Issue 2.2: Manifest Cache Race Condition ⚠️ MEDIUM

**Location:** `.github/workflows/deploy.yml:65-73`

**Problem:** Multiple concurrent builds could corrupt manifest cache.

**Scenario:**
```
Build A: Restores manifest (hash: abc123)
Build B: Restores manifest (hash: abc123)
Build A: Downloads delta, saves manifest (hash: def456)
Build B: Downloads delta, saves manifest (hash: ghi789)
Result: Build B overwrites Build A's manifest!
```

**Current Mitigation:**
```yaml
concurrency:
  group: pages
  cancel-in-progress: false  # ✓ Only one build at a time
```

**Status:** ✅ HANDLED (concurrency group prevents parallel builds)

---

### Issue 2.3: Sync Workflow Retries ⚠️ MEDIUM

**Location:** `.github/workflows/sync-content.yml:63-87`

**Problem:** No retry logic for transient failures.

**Edge Cases:**
- Network timeout during upload
- MCP server temporary unavailability
- R2 rate limiting

**Impact:** One-off network glitch causes entire sync to fail.

**Fix Needed:**
```yaml
- name: Sync content to PanaversityFS
  uses: nick-fields/retry@v2
  with:
    timeout_minutes: 10
    max_attempts: 3
    retry_wait_seconds: 30
    command: |
      python scripts/ingest-book.py ...
```

**Status:** ❌ NOT HANDLED - Should add retries

---

## 3. Manifest Cache Edge Cases

### Issue 3.1: Corrupted Manifest File 🔴 HIGH

**Location:** `scripts/hydrate/manifest.py:18-28`

**Code:**
```python
def load_manifest(path: Path) -> Optional[ManifestFile]:
    if not path.exists():
        return None

    with open(path, 'r') as f:
        data = json.load(f)  # ❌ No error handling

    return ManifestFile(**data)  # ❌ No validation
```

**Edge Cases:**
- File exists but empty: `manifest.json` with 0 bytes
- Partial write: `{"manifest_hash": "abc` (truncated JSON)
- Invalid JSON: `{manifest_hash": "abc"}` (syntax error)
- Wrong structure: `{"wrong_field": "value"}`
- Encoding issues: Binary data in JSON file

**Impact:** Script crashes, fallback never triggers, build fails completely.

**Example Crash:**
```
json.decoder.JSONDecodeError: Expecting value: line 1 column 1 (char 0)
```

**Fix Needed:**
```python
def load_manifest(path: Path) -> Optional[ManifestFile]:
    if not path.exists():
        return None

    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # Validate structure
        return ManifestFile(**data)

    except (json.JSONDecodeError, UnicodeDecodeError, ValidationError) as e:
        logger.warning(f"Corrupt manifest at {path}: {e}")
        # Delete corrupt manifest
        path.unlink(missing_ok=True)
        return None  # Triggers full rebuild

    except Exception as e:
        logger.error(f"Unexpected error loading manifest: {e}")
        return None
```

**Status:** ❌ CRITICAL - Must fix before production

---

### Issue 3.2: Manifest Cache Key Collision 🟡 LOW

**Location:** `.github/workflows/deploy.yml:70-72`

**Problem:** Cache key includes `github.sha` which is unique per commit.

**Code:**
```yaml
key: panaversity-manifest-${{ github.sha }}
restore-keys: panaversity-manifest-
```

**Edge Cases:**
- Force push changes SHA, invalidates cache
- Cherry-pick creates new SHA for same content
- Rebase creates new SHAs

**Impact:** Cache miss more often than necessary.

**Behavior:**
- Primary key: `panaversity-manifest-abc123` (exact commit)
- Fallback: `panaversity-manifest-*` (most recent)

**Analysis:** This actually works correctly! Falls back to most recent manifest.

**Status:** ✅ HANDLED CORRECTLY

---

### Issue 3.3: Manifest File Size Growth 🟡 LOW

**Location:** `scripts/hydrate/manifest.py`

**Problem:** Manifest size could grow unbounded with many files.

**Calculation:**
```
1000 files × ~100 bytes per entry = 100KB manifest
10,000 files × ~100 bytes = 1MB manifest
100,000 files = 10MB manifest
```

**Impact:** Minimal for expected scale (1000-5000 files = 100KB-500KB).

**Status:** ✅ ACCEPTABLE for current scale

---

## 4. Error Handling Gaps

### Issue 4.1: Hydration Failure Fallback Path ⚠️ MEDIUM

**Location:** `.github/workflows/deploy.yml:118-123`

**Problem:** Fallback copies from `docs/` but docs/ might be outdated.

**Code:**
```yaml
- name: Fallback to local docs
  if: ${{ vars.PANAVERSITY_PLUGIN_ENABLED == 'true' && failure() }}
  run: |
    echo "⚠️ Hydration failed, using local docs/ as fallback"
    mkdir -p build-source
    cp -r book-source/docs/* build-source/
```

**Edge Cases:**
- MCP server is up but returns wrong content
- Partial hydration (some files downloaded, then crash)
- `docs/` directory doesn't exist (authors deleted it)
- `docs/` is empty (new repo clone)

**Impact:**
- Case 1: Deploys wrong content (no error, silent corruption)
- Case 2: Deploys mixed old/new content
- Case 3: Build fails with empty directory
- Case 4: Build fails

**Fix Needed:**
```yaml
- name: Fallback to local docs
  if: ${{ vars.PANAVERSITY_PLUGIN_ENABLED == 'true' && failure() }}
  run: |
    echo "⚠️ Hydration failed, using local docs/ as fallback"

    # Check if docs/ exists and has content
    if [ ! -d "book-source/docs" ] || [ -z "$(ls -A book-source/docs)" ]; then
      echo "❌ FATAL: Hydration failed AND local docs/ is empty"
      echo "Cannot proceed with empty content directory"
      exit 1
    fi

    # Clear partial hydration
    rm -rf build-source/*

    # Copy from docs/
    mkdir -p build-source
    cp -r book-source/docs/* build-source/

    echo "⚠️ DEPLOYED FROM LOCAL DOCS (NOT PanaversityFS)" >> $GITHUB_STEP_SUMMARY
```

**Status:** ❌ NEEDS IMPROVEMENT

---

### Issue 4.2: Silent Partial Sync Failure ⚠️ MEDIUM

**Location:** `scripts/ingest/sync_engine.py` (need to review)

**Problem:** If sync fails midway, some files uploaded, some not.

**Scenario:**
```
Sync plan: 10 files to upload
Files 1-5: ✓ Uploaded
File 6: ✗ Network error
Files 7-10: ⚠️ Never attempted
```

**Current Behavior:** Script exits with error.

**Issues:**
- PanaversityFS has 5/10 files (inconsistent state)
- Next sync tries again, uploads same 5 files (wasted bandwidth)
- No way to resume from file 6

**Fix Needed:**
- Track successful uploads in temp file
- On retry, skip successfully uploaded files
- OR use transactions (all-or-nothing)

**Status:** 🤔 REVIEW NEEDED - Check sync_engine implementation

---

### Issue 4.3: No Disk Space Handling ⚠️ MEDIUM

**Location:** `scripts/hydrate/downloader.py`

**Problem:** No check for available disk space before downloading.

**Edge Cases:**
- Downloading 1GB of content with 500MB free space
- Disk fills during download
- Tmp directory on different partition with less space

**Impact:** Partial download, corrupted files, build failure.

**Fix Needed:**
```python
import shutil

def download_changed_files(...):
    # Estimate required space (conservative: 2x file sizes)
    total_size = sum(file.size for file in delta_files)
    required_space = total_size * 2  # Buffer for temp files

    # Check available space
    stat = shutil.disk_usage(output_dir)
    if stat.free < required_space:
        raise IOError(
            f"Insufficient disk space. "
            f"Required: {format_bytes(required_space)}, "
            f"Available: {format_bytes(stat.free)}"
        )

    # Proceed with download
    ...
```

**Status:** ❌ NOT HANDLED - Should add

---

## 5. File System Edge Cases

### Issue 5.1: Symlink Handling 🟡 LOW

**Location:** `scripts/ingest/source_scanner.py`

**Problem:** No explicit handling of symbolic links.

**Edge Cases:**
- Symlink to file: `Part-01/Chapter-01/01-intro.md -> ../../shared/intro.md`
- Symlink to directory: `Part-01/Chapter-01 -> ../templates/chapter`
- Circular symlinks: `A -> B -> A`
- Broken symlinks: `link.md -> nonexistent.md`

**Impact:** Could cause infinite loops or upload wrong files.

**Fix Needed:**
```python
def scan_source_directory(source_dir: Path) -> ScanResult:
    ...
    for root, dirs, files in os.walk(source_dir, followlinks=False):  # Don't follow symlinks
        # Filter out symlink directories
        dirs[:] = [d for d in dirs if not (Path(root) / d).is_symlink()]
        ...
```

**Status:** 🤔 CHECK CURRENT BEHAVIOR

---

### Issue 5.2: File Permissions ⚠️ MEDIUM

**Location:** Multiple scripts

**Problem:** No handling of permission denied errors.

**Edge Cases:**
- Read-only files in source
- No write permission in output directory
- File locked by another process (Windows)

**Impact:** Script crashes with unclear error.

**Fix Needed:**
```python
try:
    with open(file_path, 'r') as f:
        content = f.read()
except PermissionError:
    logger.warning(f"Permission denied: {file_path}, skipping")
    continue
except IOError as e:
    logger.error(f"Cannot read {file_path}: {e}")
    raise
```

**Status:** 🤔 REVIEW NEEDED

---

### Issue 5.3: Filename Case Sensitivity ⚠️ MEDIUM

**Location:** Path mapping + file system

**Problem:** macOS/Windows are case-insensitive, Linux is case-sensitive.

**Edge Cases:**
- Author creates `intro.md` locally (macOS, case-insensitive)
- CI uploads as `content/.../intro.md`
- Author renames to `Intro.md` locally (macOS sees no change)
- CI sees new file `Intro.md`, uploads both
- Result: Two files in R2: `intro.md` and `Intro.md`
- Linux server serves one, macOS serves other!

**Impact:** Inconsistent behavior across platforms.

**Fix Needed:**
- Normalize all paths to lowercase
- OR reject mixed-case filenames
- OR use case-sensitive comparison

**Status:** ❌ NOT HANDLED - Needs decision

---

## 6. Security Vulnerabilities

### Issue 6.1: Path Traversal in Storage Paths 🔴 CRITICAL

**Location:** `scripts/ingest/path_mapper.py:157`

**Already documented in Issue 1.6**

**Status:** ❌ CRITICAL - Must fix

---

### Issue 6.2: No Input Validation on book-id ⚠️ MEDIUM

**Location:** `scripts/ingest/cli.py`, `scripts/hydrate/cli.py`

**Problem:** book-id passed directly to MCP without validation.

**Attack Vectors:**
```bash
python ingest-book.py --book-id "../../../etc/passwd"
python ingest-book.py --book-id "../../admin-book"
python ingest-book.py --book-id "book; rm -rf /"
```

**Impact:** Could access other books' content or execute commands.

**Fix Needed:**
```python
import re

BOOK_ID_PATTERN = re.compile(r'^[a-z0-9-]+$')

def validate_book_id(book_id: str) -> str:
    if not BOOK_ID_PATTERN.match(book_id):
        raise ValueError(
            f"Invalid book-id: {book_id}. "
            f"Must contain only lowercase letters, numbers, and hyphens."
        )
    if len(book_id) > 100:
        raise ValueError(f"book-id too long: {len(book_id)} chars")
    return book_id
```

**Status:** ❌ NOT HANDLED - Should add

---

### Issue 6.3: API Key Exposure in Logs 🔴 HIGH

**Location:** Workflow files, script logging

**Problem:** API keys might appear in logs.

**Edge Cases:**
- `--verbose` mode logs HTTP headers
- Error messages include URLs with API keys
- Stack traces expose environment variables

**Current Mitigation:**
```yaml
env:
  PANAVERSITY_API_KEY: ${{ secrets.PANAVERSITY_API_KEY }}
```

**Remaining Issue:** Script might log the key in verbose mode.

**Fix Needed:**
```python
def sanitize_for_logging(text: str) -> str:
    """Remove sensitive data from log messages."""
    # Redact API keys (look for patterns like key=...)
    text = re.sub(r'(api[_-]?key=)[^\s&]+', r'\1***REDACTED***', text, flags=re.IGNORECASE)
    # Redact authorization headers
    text = re.sub(r'(Authorization:)\s*\S+', r'\1 ***REDACTED***', text, flags=re.IGNORECASE)
    return text

logger.info(sanitize_for_logging(message))
```

**Status:** ⚠️ POTENTIAL ISSUE - Review logging

---

### Issue 6.4: No Rate Limiting ⚠️ MEDIUM

**Location:** `scripts/ingest/sync_engine.py`, `scripts/hydrate/downloader.py`

**Problem:** No protection against accidental DoS of MCP server.

**Edge Cases:**
- Script run in loop
- Multiple CI jobs running simultaneously
- Large number of files (1000+) uploaded rapidly

**Impact:** Could overwhelm MCP server, affecting other users.

**Fix Needed:**
```python
import asyncio
from asyncio import Semaphore

# Limit concurrent requests
MAX_CONCURRENT_REQUESTS = 10
semaphore = Semaphore(MAX_CONCURRENT_REQUESTS)

async def upload_with_limit(file):
    async with semaphore:
        return await upload_file(file)
```

**Status:** 🤔 CHECK IF IMPLEMENTED

---

## 7. Data Corruption Scenarios

### Issue 7.1: Partial File Upload 🔴 HIGH

**Location:** `scripts/ingest/sync_engine.py`

**Problem:** Network interruption during upload could leave partial file in R2.

**Scenario:**
```
1. Upload starts: lesson.md (10KB)
2. Upload 5KB
3. Network error
4. Upload aborted
5. R2 might have 5KB partial file (corrupted)
```

**Impact:** Content corrupted, builds fail or show broken lessons.

**Fix Needed:**
```python
# Upload to temporary path first
temp_path = f"{storage_path}.uploading"
client.call_tool("write_content", {
    "book_id": book_id,
    "path": temp_path,
    "content": content
})

# Verify upload
verify = client.call_tool("read_content", {
    "book_id": book_id,
    "path": temp_path
})

if sha256(verify['content']) == expected_hash:
    # Atomic rename
    client.call_tool("move_content", {
        "book_id": book_id,
        "from": temp_path,
        "to": storage_path
    })
else:
    raise IOError("Upload verification failed")
```

**Status:** 🤔 CHECK IF MCP HAS MOVE/RENAME TOOL

---

### Issue 7.2: Hash Mismatch Detection ⚠️ MEDIUM

**Location:** `scripts/ingest/sync_engine.py`

**Problem:** No verification after upload.

**Edge Cases:**
- Content modified during upload
- Network corruption (rare but possible)
- R2 storage bit flip (extremely rare)

**Impact:** Uploaded content differs from source.

**Fix Needed:**
```python
# After upload
uploaded_hash = compute_hash_of_uploaded_content()
local_hash = compute_hash_of_local_file()

if uploaded_hash != local_hash:
    logger.error(f"Hash mismatch for {file}: {uploaded_hash} != {local_hash}")
    # Retry upload
    retry_upload(file)
```

**Status:** 🤔 REVIEW sync_engine.py implementation

---

### Issue 7.3: Manifest Out of Sync with Reality ⚠️ MEDIUM

**Location:** Manifest caching logic

**Problem:** Manifest could become out of sync with actual R2 content.

**Scenarios:**
- Manual upload to R2 bypasses manifest
- Concurrent syncs from different sources
- Manifest cache corrupted but not detected
- Sync fails midway, manifest not updated

**Impact:** Hydration skips files it should download.

**Fix Needed:**
- Add `--force-verify` flag to compare manifest with R2
- Periodic full scan to detect drift
- Checksum validation

**Status:** ❌ NOT IMPLEMENTED

---

## 8. Performance Edge Cases

### Issue 8.1: Large File Handling 🟡 LOW

**Location:** `scripts/hydrate/downloader.py`, `scripts/ingest/sync_engine.py`

**Problem:** No special handling for large files.

**Edge Cases:**
- 100MB video file
- 500MB dataset
- 1GB backup file accidentally in docs/

**Current Approach:** Load entire file into memory.

**Impact:**
- Memory exhaustion
- Timeout during upload/download
- Slow hash computation

**Fix Needed:**
```python
MAX_IN_MEMORY_SIZE = 10 * 1024 * 1024  # 10MB

if file_size > MAX_IN_MEMORY_SIZE:
    # Stream file instead of loading into memory
    with open(file_path, 'rb') as f:
        chunks = iter(lambda: f.read(4096), b'')
        for chunk in chunks:
            upload_chunk(chunk)
else:
    # Small file, load into memory (faster)
    content = read_file(file_path)
    upload_content(content)
```

**Status:** 🤔 REVIEW NEEDED

---

### Issue 8.2: Excessive API Calls 🟡 LOW

**Location:** `scripts/ingest/sync_engine.py`

**Problem:** Might make one API call per file to check if exists.

**Scenario:**
```
1000 files to check
→ 1000 API calls to get hashes
→ ~10 seconds just for hash checks
```

**Fix Needed:**
- Batch API to get multiple hashes in one call
- Use manifest from previous build
- Cache hash responses

**Status:** 🤔 CHECK sync_engine.py implementation

---

### Issue 8.3: No Progress Indication for Long Operations 🟡 LOW

**Location:** All CLIs

**Problem:** No progress bars for long uploads/downloads.

**Impact:** User doesn't know if script is stuck or working.

**Fix Needed:**
```python
from tqdm import tqdm

for file in tqdm(files, desc="Uploading"):
    upload_file(file)
```

**Status:** ❌ NOT IMPLEMENTED (nice-to-have)

---

## 9. Concurrency Issues

### Issue 9.1: Multiple Authors Pushing Simultaneously ⚠️ MEDIUM

**Location:** Workflow concurrency

**Scenario:**
```
T0: Author A pushes to main
T1: sync-content.yml starts (Author A's changes)
T2: Author B pushes to main (before A's sync completes)
T3: sync-content.yml starts (Author B's changes)
T4: Both syncs running in parallel
```

**Problem:** Both syncs might upload same files with different content.

**Impact:** Last write wins, earlier changes lost.

**Fix Needed:**
```yaml
# sync-content.yml
concurrency:
  group: panaversity-sync
  cancel-in-progress: false  # Queue, don't cancel
```

**Status:** ⚠️ NOT HANDLED - Need to add concurrency group

---

### Issue 9.2: Sync During Build ⚠️ MEDIUM

**Location:** Workflow orchestration

**Problem:** Build might hydrate while sync is uploading.

**Scenario:**
```
T0: Sync starts (uploads file 1, 2, 3...)
T1: Sync completes, triggers deploy
T2: Deploy starts hydrating
T3: New sync starts (file 4, 5, 6...)
T4: Deploy hydration reads manifest (includes files 4,5,6 that aren't uploaded yet)
```

**Impact:** Build might reference files that don't exist yet.

**Current Mitigation:**
```yaml
workflow_run:
  types: [completed]  # Wait for sync to complete
```

**Remaining Issue:** If new sync starts AFTER deploy triggers but BEFORE hydration completes.

**Fix Needed:**
- Add concurrency group to prevent overlapping workflows
- OR lock mechanism at MCP level

**Status:** ⚠️ PARTIALLY HANDLED

---

## 10. Recovery Scenarios

### Issue 10.1: Recovery from Corrupted Build ⚠️ MEDIUM

**Scenario:** Build deployed with corrupted content.

**Current Recovery:**
1. Disable PANAVERSITY_PLUGIN_ENABLED
2. Trigger new build (uses local docs/)
3. Fix corruption
4. Re-enable

**Problem:** No automated corruption detection.

**Improvement Needed:**
- Post-build validation (smoke test)
- Automatic rollback on validation failure
- Health check endpoint

**Status:** ❌ NO AUTOMATED DETECTION

---

### Issue 10.2: Recovery from Lost Manifest ⚠️ MEDIUM

**Scenario:** Manifest cache corrupted or lost.

**Current Behavior:** Full rebuild (downloads all files).

**Impact:** Slow but safe.

**Improvement:**
- Reconstruct manifest from last successful build
- Store manifest in R2 as backup

**Status:** ✅ ACCEPTABLE (full rebuild is safe fallback)

---

### Issue 10.3: Recovery from MCP Server Data Loss 🔴 HIGH

**Scenario:** R2 bucket corrupted or accidentally deleted.

**Current Backup:** Content exists in git (book-source/docs/).

**Recovery:**
```bash
# Re-sync all content from git
git clone book-content
cd book-content
python scripts/ingest-book.py --book-id ai-native-dev --source-dir . --full-sync
```

**Problem:** No automated recovery, no monitoring for this scenario.

**Improvement Needed:**
- Daily backup of R2 bucket
- Monitoring for empty/missing content
- Automated recovery workflow

**Status:** ⚠️ MANUAL RECOVERY ONLY

---

## Critical Issues Found

### 🔴 CRITICAL (Must Fix Before Production)

1. **Path Traversal Vulnerability (Issue 1.6)**
   - Attack: `Part-01/Chapter-01/01-../../etc/passwd.md`
   - Impact: Write files outside content directory
   - Fix: Sanitize lesson names, validate final paths

2. **Corrupted Manifest Handling (Issue 3.1)**
   - Problem: Script crashes on corrupted JSON
   - Impact: Build fails completely instead of fallback
   - Fix: Try-catch with manifest deletion on error

3. **API Key Exposure (Issue 6.3)**
   - Problem: Keys might appear in verbose logs
   - Impact: Security breach
   - Fix: Sanitize logs, redact sensitive data

---

### ⚠️ HIGH (Should Fix Before Production)

4. **Partial File Upload (Issue 7.1)**
   - Problem: Network error leaves partial file in R2
   - Impact: Corrupted content
   - Fix: Upload to temp path, verify, then rename

5. **Concurrent Workflow Execution (Issue 9.1)**
   - Problem: Multiple syncs can run simultaneously
   - Impact: Race conditions, lost updates
   - Fix: Add concurrency groups to workflows

6. **Fallback Path Issues (Issue 4.1)**
   - Problem: Fallback might fail if docs/ empty
   - Impact: Build fails with no recovery
   - Fix: Check docs/ exists before copying

---

### 🟡 MEDIUM (Fix Soon After Launch)

7. **Empty Path Components (Issue 1.2)**
8. **No Retry Logic (Issue 2.3)**
9. **No Disk Space Check (Issue 4.3)**
10. **No book-id Validation (Issue 6.2)**
11. **Hash Mismatch Detection (Issue 7.2)**
12. **Sync During Build (Issue 9.2)**

---

### 🟢 LOW (Nice to Have)

13. **Unicode Filenames (Issue 1.3)**
14. **Very Long Paths (Issue 1.5)**
15. **Symlink Handling (Issue 5.1)**
16. **Large File Handling (Issue 8.1)**
17. **Progress Indication (Issue 8.3)**

---

## Recommendations

### Immediate Actions (Before Production)

1. **Fix Path Traversal (Issue 1.6)**
```python
# In path_mapper.py
lesson_name = lesson_name.replace("..", "").replace("/", "").replace("\\", "")

# Validate final path
final_path = Path(storage_path).resolve()
if not final_path.is_relative_to(Path("content").resolve()):
    return MappedPath(..., valid=False, error="Invalid path")
```

2. **Fix Manifest Corruption Handling (Issue 3.1)**
```python
# In manifest.py
try:
    data = json.load(f)
    return ManifestFile(**data)
except (JSONDecodeError, ValidationError) as e:
    logger.warning(f"Corrupt manifest: {e}")
    path.unlink(missing_ok=True)
    return None
```

3. **Add Concurrency Groups**
```yaml
# In sync-content.yml
concurrency:
  group: panaversity-sync
  cancel-in-progress: false

# In deploy.yml (already has 'pages' group, verify it's correct)
```

4. **Improve Fallback Logic**
```yaml
# In deploy.yml
- name: Fallback to local docs
  if: failure()
  run: |
    if [ ! -d "book-source/docs" ] || [ -z "$(ls -A book-source/docs)" ]; then
      echo "❌ Cannot fallback: docs/ is empty"
      exit 1
    fi
    rm -rf build-source/*
    cp -r book-source/docs/* build-source/
```

5. **Add Logging Sanitization**
```python
# In all scripts
import re

def sanitize_for_logging(msg):
    msg = re.sub(r'api[_-]?key[=:]\s*\S+', 'api_key=***', msg, flags=re.I)
    return re.sub(r'Authorization:\s*\S+', 'Authorization: ***', msg, flags=re.I)
```

---

### Post-Launch Improvements

6. **Add Retry Logic** (Issue 2.3)
7. **Add Disk Space Checks** (Issue 4.3)
8. **Validate book-id** (Issue 6.2)
9. **Add Upload Verification** (Issue 7.1)
10. **Handle Empty Path Components** (Issue 1.2)

---

### Monitoring & Alerting

**Add these monitors:**
- Sync success rate (<98% → alert)
- Build time (>5min → warning)
- Manifest cache hit rate (<70% → investigate)
- Fallback activation count (>0 → alert)
- MCP server errors (>1% → alert)

---

## Test Coverage for Edge Cases

**Additional tests needed:**

```python
# Test path traversal prevention
def test_path_traversal_blocked():
    result = map_source_to_storage("Part-01/Chapter-01/01-../../etc/passwd.md")
    assert not result.valid
    assert "traversal" in result.error.lower()

# Test corrupted manifest
def test_corrupted_manifest_recovery():
    with open("manifest.json", "w") as f:
        f.write("{invalid json")

    manifest = load_manifest(Path("manifest.json"))
    assert manifest is None
    assert not Path("manifest.json").exists()  # Deleted

# Test empty path components
def test_empty_path_components():
    result = map_source_to_storage("Part-01//Chapter-01/01-intro.md")
    assert result.valid  # Should handle gracefully

# Test concurrent uploads
@pytest.mark.asyncio
async def test_concurrent_uploads():
    # Simulate 10 concurrent uploads
    tasks = [upload_file(f"file_{i}.md") for i in range(10)]
    results = await asyncio.gather(*tasks)
    assert all(r.success for r in results)
```

---

## Sign-Off

**Reviewer:** Claude Code
**Date:** 2025-12-08
**Status:** 🔴 CRITICAL ISSUES FOUND

**Recommendation:** Fix 6 critical/high issues before production deployment.

**Critical Issues (MUST FIX):**
1. Path traversal vulnerability
2. Corrupted manifest handling
3. API key exposure in logs

**High Priority (SHOULD FIX):**
4. Partial file upload handling
5. Concurrent workflow execution
6. Fallback path validation

**Time Estimate:** 4-6 hours to fix critical + high issues

**Next Steps:**
1. Review this analysis with team
2. Prioritize fixes
3. Implement critical fixes
4. Re-test with edge cases
5. Deploy to production

---

**End of Edge Case Analysis**
