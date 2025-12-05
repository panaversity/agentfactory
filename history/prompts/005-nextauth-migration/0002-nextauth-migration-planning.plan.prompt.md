---
id: 005-002
title: NextAuth to Better Auth Migration Planning
stage: plan
date: 2024-12-05
surface: claude-code
model: claude-opus-4-5
feature: 005-nextauth-migration
branch: 005-nextauth-migration
user: mjs
command: /sp.plan
labels: [migration, planning, architecture, batch-processing]
links:
  spec: specs/005-nextauth-migration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/005-nextauth-migration/plan.md
    action: created
    description: Complete implementation plan with architecture
tests:
  - name: constitution-check
    status: passed
    description: All 7 applicable security principles verified
---

## Prompt

Create implementation plan for NextAuth to Better Auth user migration. Work type: PLATFORM (database migration). Route to general-purpose planning for technical architecture.

## Response snapshot

### Implementation Plan Created

Created comprehensive plan at `specs/005-nextauth-migration/plan.md` with:

**Architecture Components**:
- CLI Parser (--dry-run, --limit, --offset flags)
- Config Loader (environment variables)
- Logger (progress tracking, ETA)
- DB Connections (Neon serverless)
- Batch Processor (500 users/batch)
- Summary Reporter

**Implementation Sequence (4 phases)**:
1. Core Infrastructure: CLI, config, logging, connections
2. Data Transformation: Country normalizer, username generator, hash validator
3. Migration Logic: Source query, conflict detector, inserter, resolver
4. Execution: Main loop, summary, exit handler

**Key Algorithms**:
- Batch Processing: Transaction per batch, rollback on error
- Conflict Resolution: Update user.id, all FK references, fill nulls
- Country Normalization: 18 codes + default + fallback
- Username Generation: email prefix + random suffix

**Constitution Check**: PASS (7/7 applicable principles)

### Plan Highlights

- Single CLI script (no new directories)
- Transaction safety per batch
- Comprehensive logging
- Dry-run mode for verification
- Resume capability with --offset

## Outcome

- ✅ Impact: High - Complete architecture for 14,821 user migration
- 🧪 Tests: 1/1 passed (constitution check)
- 📁 Files: 1 created (plan.md)
- 🔁 Next prompts: /sp.tasks 005-nextauth-migration
- 🧠 Reflection: Conflict resolution algorithm is the critical path - FK updates before user.id change

## Evaluation notes (flywheel)

- Failure modes observed: None - plan follows established patterns
- Graders run and results (PASS/FAIL): Constitution check PASS
- Prompt variant (if applicable): SDD-RI orchestrated workflow
- Next experiment (smallest change to try): Proceed to task breakdown
