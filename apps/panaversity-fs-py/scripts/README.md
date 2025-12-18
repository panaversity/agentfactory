# PanaversityFS Scripts

Migration and utility scripts for PanaversityFS.

## migrate_book_source.py

Migrate content from `book-source/` to any storage backend via OpenDAL.

### Prerequisites

```bash
cd panaversity-fs
uv sync
```

## Preview

```bash
mjs@Muhammads-MacBook-Pro-3 panaversity-fs-py % uv run alembic upgrade head
INFO  [alembic.runtime.migration] Context impl PostgresqlImpl.
INFO  [alembic.runtime.migration] Will assume transactional DDL.
INFO  [alembic.runtime.migration] Running upgrade  -> 147ff406b539, initial FileJournal and AuditLog schema
INFO  [alembic.runtime.migration] Running upgrade 147ff406b539 -> 853c7e2d6e2a, use timezone aware timestamps
mjs@Muhammads-MacBook-Pro-3 panaversity-fs-py %   PANAVERSITY_LOG_LEVEL=WARNING uv run python scripts/migrate_book_source.py \
    --source ../learn-app \
    --rewrite-urls \
    --write-local
```


  cd apps/panaversity-fs-py

  # Upload your image

uv run python scripts/upload_image.py \
    --image /Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/panaversity-fs-py/skills-mcp-complementarity.png \
    --part 2 \
    --chapter 05 \
    --name skills-mcp-complementarity

uv run python scripts/upload_image.py \
--image /Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/skills-in-stack.png \
--part 2 \
--chapter 05 \
--name lesson-4-skills-in-stack

### Usage

```bash
# Dry run - preview what will be uploaded
uv run python scripts/migrate_book_source.py --dry-run

# Full migration with URL rewriting (recommended for cloud backends)
uv run python scripts/migrate_book_source.py --rewrite-urls

# Content only (skip assets)
uv run python scripts/migrate_book_source.py --content-only --rewrite-urls

# Assets only (skip content)
uv run python scripts/migrate_book_source.py --assets-only

# Verbose output
uv run python scripts/migrate_book_source.py --rewrite-urls -v

# Resume from a specific path (for large migrations or retries)
uv run python scripts/migrate_book_source.py --resume-from "chapter-14" --rewrite-urls
```

### Storage Backend Configuration

Configure via `.env` or environment variables:

#### Local Filesystem

```bash
export PANAVERSITY_STORAGE_BACKEND=fs
export PANAVERSITY_STORAGE_ROOT=/tmp/panaversity-data
```

#### Supabase Storage

```bash
export PANAVERSITY_STORAGE_BACKEND=supabase
export PANAVERSITY_SUPABASE_URL=https://xxx.supabase.co
export PANAVERSITY_SUPABASE_SERVICE_ROLE_KEY=eyJ...
export PANAVERSITY_SUPABASE_BUCKET=panaversity-books
```

**Note**: Toggle bucket to **Public** in Supabase Dashboard for CDN URLs to work.

#### Cloudflare R2 / AWS S3

```bash
export PANAVERSITY_STORAGE_BACKEND=s3
export PANAVERSITY_S3_BUCKET=your-bucket
export PANAVERSITY_S3_ENDPOINT=https://xxx.r2.cloudflarestorage.com
export PANAVERSITY_S3_ACCESS_KEY_ID=your-key
export PANAVERSITY_S3_SECRET_ACCESS_KEY=your-secret
export PANAVERSITY_S3_REGION=auto
export PANAVERSITY_CDN_BASE_URL=https://your-bucket.r2.dev  # or custom domain
```

### URL Rewriting

With `--rewrite-urls`, the script transforms local asset paths to CDN URLs:

| Pattern            | Before                   | After                                              |
| ------------------ | ------------------------ | -------------------------------------------------- |
| Markdown images    | `![alt](/img/foo.png)`   | `![alt]({cdn}/books/{book}/static/images/foo.png)` |
| Markdown slides    | `![alt](/slides/x.pdf)`  | `![alt]({cdn}/books/{book}/static/slides/x.pdf)`   |
| Frontmatter slides | `source: "slides/x.pdf"` | `source: "{cdn}/books/{book}/static/slides/x.pdf"` |

### Directory Mapping (ADR-0018)

| Source                       | Target                           |
| ---------------------------- | -------------------------------- |
| `apps/learn-app/docs/`       | `books/{book-id}/content/`       |
| `book-source/static/img/`    | `books/{book-id}/static/images/` |
| `book-source/static/slides/` | `books/{book-id}/static/slides/` |

### Example Output

```
============================================================
OpenDAL Migration (ADR-0018)
============================================================
Source:       ../book-source
Backend:      supabase
Book ID:      ai-native-dev
Mode:         LIVE
Content:      Yes
Assets:       Yes
Rewrite URLs: Yes
Resume:       From beginning
============================================================

=== Migrating Content ===
  URL rewriting: ENABLED
  Found 299 markdown files
  Uploading: 299/299
  ✓ Uploaded: 299

=== Migrating Assets ===
  Found 167 asset files
  Uploading: 167/167
  ✓ Uploaded: 165
  ✗ Failed: 2

Migration Summary
=================
Content:
  Uploaded: 299
  Skipped:  0
  Failed:   0

Assets:
  Uploaded: 165
  Skipped:  0
  Failed:   2

URLs Rewritten: 115
Total Size: 89.32 MB
Errors: 2
```
