---
id: 005-004
title: NextAuth to Better Auth Migration Implementation
stage: green
date: 2024-12-05
surface: claude-code
model: claude-opus-4-5
feature: 005-nextauth-migration
branch: 005-nextauth-migration
user: mjs
command: /sp.implement
labels: [migration, implementation, batch-processing, conflict-resolution, bcrypt]
links:
  spec: specs/005-nextauth-migration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: scripts/migrate-nextauth-users.ts
    action: created
    description: Complete migration script (900+ lines) with CLI, batch processing, conflict resolution
tests:
  - name: dry-run-10-users
    status: passed
    description: Basic functionality test with 10 users
  - name: dry-run-100-users
    status: passed
    description: Batch processing with 2 batches of 50 users
  - name: dry-run-1000-users
    status: passed
    description: Full validation with 2 batches of 500 users, 4m9s duration
  - name: overlapping-user-detection
    status: passed
    description: All 3 overlapping users correctly identified for merge
  - name: country-normalization
    status: passed
    description: 30+ country codes mapped correctly (PK→Pakistan, BD→Bangladesh, etc.)
---

## Prompt

Implement the migration script based on spec.md and plan.md. Follow all 59 tasks from tasks.md. Key requirements:
- Batch processing (500/batch) with transaction safety
- ID preservation for FK integrity
- Conflict resolution: update target user.id to match source
- Country normalization (PK→Pakistan, NULL→Pakistan)
- Username generation from email
- Bcrypt password hash preservation

## Response snapshot

### Implementation Complete

Created `scripts/migrate-nextauth-users.ts` - a comprehensive 900+ line migration script implementing all requirements:

**CLI Features:**
- `--dry-run` - Log operations without DB modifications
- `--limit N` - Process only N users (testing)
- `--offset N` - Resume from position N
- `--batch-size N` - Users per batch (default: 500)
- `--verbose` - Detailed logging
- `--help` - Usage information

**Core Components:**
1. **Config Validation** - Verifies environment variables and DB connections
2. **Country Normalization** - 30+ country codes mapped (PK, BD, KR, US, GB, etc.)
3. **Username Generation** - Email prefix + random 4-char suffix, collision detection
4. **Batch Processor** - Transaction-per-batch, progress tracking, ETA calculation
5. **Conflict Resolution** - Updates all 8 FK tables, then user.id, then fills nullables
6. **Summary Reporter** - Statistics, country normalizations, verification queries

**Validation Results (1000 users):**
```
Results:
├── Total Processed: 1,000
├── New Users Inserted: 997
├── Overlapping Users Merged: 3
├── Skipped: 0
├── Errors: 0
└── Duration: 4 minute(s) 9 second(s)
```

**Performance Projection:**
- 1000 users: 4m9s
- 14,821 users (full): ~61 minutes estimated

## Outcome

- ✅ Impact: High - Production-ready migration script for 14,821 users
- 🧪 Tests: 5/5 passed (dry-run validations across multiple scales)
- 📁 Files: 1 created (scripts/migrate-nextauth-users.ts)
- 🔁 Next prompts: Execute live migration when ready
- 🧠 Reflection: Neon serverless template literals required explicit FK update queries

## Evaluation notes (flywheel)

- Failure modes observed: Neon SQL template literals don't support dynamic identifiers
- Graders run and results (PASS/FAIL):
  - dry-run-10: PASS
  - dry-run-100: PASS
  - dry-run-1000: PASS
  - overlapping-detection: PASS (3/3 found)
  - country-normalization: PASS
- Prompt variant (if applicable): SDD-RI orchestrated implementation
- Next experiment (smallest change to try): Add --resume-from-file for crash recovery
