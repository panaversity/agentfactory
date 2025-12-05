---
id: 005-001
title: NextAuth to Better Auth Migration Specification
stage: spec
date: 2024-12-05
surface: claude-code
model: claude-opus-4-5
feature: 005-nextauth-migration
branch: 005-nextauth-migration
user: mjs
command: /sp.specify
labels: [migration, nextauth, better-auth, database, batch-processing]
links:
  spec: specs/005-nextauth-migration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/005-nextauth-migration/spec.md
    action: created
    description: Full migration specification with edge cases
  - path: specs/005-nextauth-migration/checklists/requirements.md
    action: created
    description: Quality checklist from spec-architect validation
tests:
  - name: spec-architect-validation
    status: passed
    description: Specification completeness and quality validation
---

## Prompt

Create comprehensive specification for NextAuth to Better Auth user migration with:
- Batch processing (500 users/batch)
- Conflict resolution for overlapping users
- Country normalization (PK → Pakistan, NULL → Pakistan)
- Username recreation from email
- All edge cases documented
- Dry-run and resume capabilities

## Response snapshot

### Specification Created

Created comprehensive spec at `specs/005-nextauth-migration/spec.md` with:

**User Stories (6)**:
1. P1: Dry Run Migration
2. P1: Batch Migration with Progress
3. P2: Country Normalization
4. P2: Username Recreation
5. P2: Conflict Resolution
6. P3: Resume Capability

**Edge Cases (10)**: NULL name, empty password, missing profile, connection drops, invalid UUID, unknown country code, unusual email, non-standard role, long username, NULL timestamps

**Functional Requirements (18)**: FR-001 to FR-018 covering batch processing, ID preservation, password migration, country normalization, username generation, conflict handling, CLI flags, progress tracking, rollback

**Success Evals (5)**: Data Integrity (P0), Login Functionality (P0), Performance (P1), Data Transformation Accuracy (P1), Dry-Run Fidelity (P2)

**Data Transformation Rules**: Complete mapping table from NextAuth schema to Better Auth schema

**Country Normalization Mapping**: 18 country codes with full names + NULL default + fallback behavior

### Spec-Architect Validation

- Testability: 9.5/10
- Completeness: 10/10
- Ambiguity: 9/10
- Traceability: 8/10
- Formal Verification: 10/10
- **Overall Score**: 9.4/10

**Verdict**: READY FOR PLANNING

## Outcome

- ✅ Impact: High - Comprehensive specification for 14,821 user migration with all edge cases
- 🧪 Tests: 1/1 passed (spec-architect validation)
- 📁 Files: 2 created (spec.md, requirements.md checklist)
- 🔁 Next prompts: /sp.plan 005-nextauth-migration
- 🧠 Reflection: Evals-First pattern critical - added Success Evals section before user stories

## Evaluation notes (flywheel)

- Failure modes observed: Initial spec missing Success Evals section (fixed)
- Graders run and results (PASS/FAIL): spec-architect PASS (9.4/10)
- Prompt variant (if applicable): SDD-RI orchestrated workflow
- Next experiment (smallest change to try): Proceed to planning phase
