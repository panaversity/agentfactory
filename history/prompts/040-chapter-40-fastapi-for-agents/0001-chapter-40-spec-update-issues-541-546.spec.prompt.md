---
id: "0001"
title: "chapter-40-spec-update-issues-541-546"
stage: spec
date: 2025-12-27
surface: cli
model: claude-opus-4-5
feature: 040-chapter-40-fastapi-for-agents
branch: main
user: mjs
command: /sp.chapter
labels: [spec, chapter-40, fastapi, issues-541-546]
links:
  spec: specs/040-chapter-40-fastapi-for-agents/spec.md
  ticket: "#541, #542, #543, #544, #545, #546"
  adr: null
  pr: null
files:
  - specs/040-chapter-40-fastapi-for-agents/spec.md (updated)
  - specs/040-chapter-40-fastapi-for-agents/checklists/requirements.md (created)
tests:
  - null
---

## Prompt

User requested via /sp.chapter to review issues #541-#546 and update Chapter 40: FastAPI for Agents specification.

Key requirements from user:
1. "One concept per lesson" - password hashing separate from JWT, env vars separate from SQLModel
2. "The same APIs can become tools of agent at end"
3. "SQLModel fast track will be taught in such a way that this one lesson gets basics of relation and postgres in simple way with no complexity using Neon"
4. "This chapter is bare base for all aspects and learning pathways"
5. "Students can learn Claude Code then this chapter and move to Part 7"

Issues to address:
- #541: Add Fast Track SQLModel Setup lesson (before DI)
- #542: Add Environment Variables lesson (after Error Handling)
- #543: Add JWT Authentication lesson (tokens only)
- #544: Add Pytest Fundamentals as Lesson 2 (L1 Manual First)
- #545: Simplify Agent Integration lesson (APIs → Tools → Streaming)
- #546: Add Password Hashing + Rate Limiting lesson

## Response snapshot

Updated spec.md from v2.0.0 to v3.0.0 with:

1. **Restructured to 13 lessons** (from 8):
   - Tier A (1-5): FastAPI Fundamentals with Testing
   - Tier B (6-9): Production Essentials (env vars, SQLModel, JWT, password/rate limit)
   - Tier C (10-11): DI & Streaming
   - Tier D (12-13): Agent Integration & Capstone

2. **Added "Assumed Knowledge" section** per educational content requirements

3. **Detailed lesson specifications** for each new lesson:
   - Lesson 2: Pytest Fundamentals (L1 Manual First)
   - Lesson 6: Environment Variables
   - Lesson 7: SQLModel + Neon Setup (Fast Track)
   - Lesson 8: JWT Authentication (tokens only)
   - Lesson 9: Password Hashing + Rate Limiting

4. **Simplified Lesson 12** (Agent Integration) per Issue #545

5. **Added Issue Traceability** section mapping each issue to lesson

6. **Created validation checklist** at checklists/requirements.md

## Outcome

- ✅ Impact: Spec updated with 13-lesson structure addressing all 6 issues
- 🧪 Tests: N/A (spec phase)
- 📁 Files: spec.md updated, requirements.md created
- 🔁 Next prompts: /sp.plan, /sp.tasks, /sp.implement
- 🧠 Reflection: User's emphasis on "one concept per lesson" and "bare base for all pathways" drove the restructuring. The separation of JWT from password hashing exemplifies this principle.

## Evaluation notes (flywheel)

- Failure modes observed: None - followed GATE protocol correctly
- Graders run and results (PASS/FAIL): N/A (spec phase)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to /sp.plan phase
