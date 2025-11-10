---
id: 0003
title: licensing-and-local-first-clarification
stage: plan
date: 2025-01-10
surface: conversational
model: claude-sonnet-4-5-20250929
feature: 017-usage-data-collection
branch: 017-usage-data-collection
user: mjs
command: N/A (follow-up clarifications)
labels: [licensing, cc-by-nc-nd, local-first, telemetry-workflow, data-sharing]
links:
  spec: specs/017-usage-data-collection/spec.md
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: LICENSE
    status: created
    lines: 177
  - path: NOTICE
    status: modified
    lines: 119
  - path: specs/017-usage-data-collection/TELEMETRY-WORKFLOW.md
    status: created
    lines: 543
  - path: specs/017-usage-data-collection/plan.md
    status: modified
    lines: 437
tests:
  - name: licensing-research
    status: passed
    notes: Verified CC BY-NC-ND 4.0 prevents commercial PDF distribution while allowing free reading
---

## Prompt

User provided two critical clarifications to the planning phase:

**Clarification 1: Book Content Licensing**
> "No making PDF shall not be allowed so anyone can read this book and audit for free that's all no reselling og book or something??"

**Intent**: User wants to prevent commercial distribution (reselling PDFs, creating paid courses) while allowing free access for reading, learning, and auditing.

**Clarification 2: Telemetry Architecture (Local-First)**
> "Also I want to collect all data but how shall each memember run locallyy and then share files or how? Maybe for now we start locally and once satisifed we then setup cloud URL"

**Intent**: Phased deployment approach:
1. Start with local collection (each team member runs own telemetry stack)
2. Later migrate to centralized cloud endpoint when satisfied with stability
3. Need workflow for data sharing (local → optional cloud)

## Response snapshot

**Licensing Strategy Implemented**:

Created comprehensive dual licensing structure:

1. **LICENSE file** (177 lines):
   - **Part 1**: Book content under CC BY-NC-ND 4.0 (free to read, cannot resell)
   - **Part 2**: Source code under MIT (fully open source)
   - **Part 3**: Telemetry data (project-private, not included in distribution)
   - Includes 12 FAQs addressing common questions (can I translate? can I teach paid course? etc.)

2. **NOTICE file** (119 lines):
   - Clarifies what cloners receive (code + docs) vs what they don't (collected data)
   - Explains separation: book content ≠ infrastructure code ≠ telemetry data
   - Summary table showing license for each component

3. **Key License Terms (CC BY-NC-ND 4.0)**:
   - ✅ **Allowed**: Free reading, sharing links, personal PDFs, auditing, free courses
   - ❌ **Prohibited**: Selling PDFs, commercial courses, translations without permission, derivative works

**Telemetry Workflow Documented**:

Created `TELEMETRY-WORKFLOW.md` (543 lines) describing phased deployment:

**Phase 1 (Local-First)**:
- Each team member runs docker-compose locally (localhost:8123)
- Data stays on individual machines (Docker volumes, not in git)
- No shared infrastructure needed
- Advantages: Privacy, offline-capable, fast iteration, zero cost

**Phase 2 (Optional Centralization)**:
- Migrate to shared cloud VM when team needs cross-member analytics
- Decision tree helps determine when to centralize
- Migration checklist (provision VM, configure TLS, update env vars, export local data)
- Rollback plan if centralization fails

**Phase 3 (Analytics)**:
- Automated reports for project coordinator
- Cross-team analytics (cost by chapter, error patterns, token efficiency)
- Example queries for common use cases

**Data Sharing Options**:
- Export local data to CSV for manual sharing
- Import to centralized database when migrating
- Role-based access control (contributors see own data, coordinators see aggregates)

**Architecture Comparison**:
```
LOCAL (Phase 1):
Each member → localhost:4317 → Local ClickHouse → Query own data

CLOUD (Phase 2):
All members → cloud:4317 → Shared ClickHouse → Role-based access
```

**Updated plan.md Summary**:
- Emphasized "phased telemetry collection infrastructure"
- Added deployment phases (NOW: local, LATER: cloud, OPTIONAL: analytics)
- Added licensing strategy section explaining dual licensing
- Clarified "local-first architecture" as key innovation

## Outcome

- ✅ Impact: Resolved critical ambiguity about book content protection (CC BY-NC-ND prevents reselling while allowing free reading) and telemetry deployment (local-first with optional cloud migration). This unblocks implementation by clarifying licensing requirements and eliminating premature centralization complexity.
- 🧪 Tests: Licensing research validated CC BY-NC-ND 4.0 meets requirements; no technical tests needed (documentation-only changes)
- 📁 Files: 4 files created/modified (LICENSE: 177 lines, NOTICE: 119 lines, TELEMETRY-WORKFLOW.md: 543 lines, plan.md: summary updated)
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks based on revised local-first architecture, OR user review of licensing terms and telemetry workflow
- 🧠 Reflection: User's clarifications fundamentally improved the feature design. Initial plan assumed immediate centralization (complex, expensive, premature). Revised local-first approach eliminates infrastructure dependencies, reduces cost to $0 during development, and respects team member privacy. Licensing clarification prevents common open-source misunderstanding where "open source repo" is conflated with "all content is commercially exploitable"—dual licensing allows code to be MIT (fully open) while book content is CC BY-NC-ND (free to read, not free to resell).

## Evaluation notes (flywheel)

- Failure modes observed: Initial plan failed to address book content commercial protection and prematurely assumed centralized infrastructure. User clarifications surfaced these critical oversights.
- Graders run and results (PASS/FAIL): Licensing research — PASS (CC BY-NC-ND correctly prevents reselling while allowing free reading)
- Prompt variant (if applicable): N/A (clarification-driven refinement, not prompt optimization)
- Next experiment (smallest change to try): Test local-first setup with one team member following TELEMETRY-WORKFLOW.md Phase 1 instructions to validate <15 minute setup claim (SC-001). If successful, validates architecture decision; if fails, identifies documentation gaps.
