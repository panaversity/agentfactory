---
id: 0004
title: Chapter 3 Plan Revised for 8-Lesson Structure
stage: plan
date: 2025-10-30
surface: agent
model: claude-haiku-4-5-20251001
feature: 004-chapter-3-redesign
branch: 004-chapter-3-redesign
user: (user feedback: lessons need redesign not consolidation)
command: /sp.plan
labels: ["chapter-3", "plan", "lesson-structure", "revision"]
links:
  spec: /specs/004-chapter-3-redesign/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-chapter-3-redesign/plan.md (revised with 8-lesson approach)
tests:
  - Plan review: all design decisions documented
  - Lesson architecture: 8 files with clear scope/content
  - Constitution check: passed
---

## Prompt

User feedback: "The lessons are not updated? We need to revise our plan as the lessons needs to be redesigned instead of getting consolidated in readme"

**Context**: Initial implementation consolidated all Chapter 3 content into README.md, but the correct approach is to create 8 separate lesson files (01-opening-hook.md through 08-closing.md) maintaining consistency with existing lesson structure and better pacing.

**Task**: Revise the implementation plan to specify:
1. 8-lesson structure with clear scoping
2. Individual YAML frontmatter for each lesson
3. Distribution of engagement elements (diagrams, videos, tables)
4. Content reuse strategy from existing 5 files
5. Lesson-by-lesson validation approach
6. Docosaurus integration (_category_.json, image paths)

## Response snapshot

**Planning Status**: ✅ COMPLETE

**Revised Approach**:
- Changed from: Single consolidated README.md (4,728 words)
- Changed to: 8 separate lesson files (300-500 words each = 2,500-3,250 total)

**Lesson Architecture**:

| Lesson | File | Title | Words | Duration |
|--------|------|-------|-------|----------|
| 1 | 01-opening-hook.md | Opening Hook & Introduction | 300-400 | 2 min |
| 2 | 02-snakes-and-ladders.md | Snakes and Ladders Framework | 400-500 | 3 min |
| 3 | 03-super-orchestrators.md | Super Orchestrators Economics | 400-500 | 3 min |
| 4 | 04-vertical-intelligence.md | Code → Vertical Intelligence | 350-450 | 2.5 min |
| 5 | 05-ppp-strategy.md | Piggyback Protocol Pivot | 400-500 | 3 min |
| 6 | 06-three-requirements.md | Three Requirements | 250-350 | 2 min |
| 7 | 07-pause-and-reflect.md | Pause and Reflect | 100-150 | 1 min |
| 8 | 08-closing.md | Closing & Chapter 4 Bridge | 300-400 | 2.5 min |

**Target State Structure**:
```
book-source/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/
├── _category_.json                    [NEW]
├── 01-opening-hook.md                 [NEW]
├── 02-snakes-and-ladders.md           [NEW]
├── 03-super-orchestrators.md          [NEW]
├── 04-vertical-intelligence.md        [NEW]
├── 05-ppp-strategy.md                 [REPLACE existing]
├── 06-three-requirements.md           [NEW]
├── 07-pause-and-reflect.md            [NEW]
└── 08-closing.md                      [NEW]
```

**Key Design Decisions**:
1. **8-lesson modular structure** (vs. consolidated README) → better pacing, easier navigation, modular updates
2. **Individual YAML frontmatter** → Docosaurus sidebar metadata per lesson
3. **Content reuse** → salvage best content from existing 5 files, reorganize into new structure
4. **Engagement distribution** → spread diagrams, videos, tables across lessons to prevent overload
5. **Video links** → Lesson 1 or 2 (opening)
6. **Diagram** → Lesson 2 (Snakes & Ladders framework)
7. **Tables** → Lessons 3, 4, 5 (economics, paradigm shift, strategy comparison)

**Constitution Check**: ✅ PASSED
- All AI-First Teaching principles confirmed
- Concept scaffolding verified (simple → complex progression)
- Book scaffolding alignment confirmed (Ch2 → Ch3 → Ch4)
- Grade 7-9 reading level maintained
- No code required (Part 1 strategic focus)

**Validation Strategy**:
- Per-lesson validation: reading level, engagement, content elements, natural flow to next lesson
- Chapter-level validation: all frameworks covered, all evidence present, Ch2 callback, Ch4 bridge

**Implementation Approach**:
- Phase 1: Review existing files for salvageable content
- Phase 2: Create 8 new lesson files with content remapped/reorganized
- Phase 3: Cross-lesson validation and Docosaurus integration
- Phase 4: Technical review and publication

## Outcome

- ✅ Impact: Plan completely revised to reflect 8-lesson modular approach instead of consolidated README
- 🧪 Tests: Design decisions documented, lesson architecture specified, validation strategy clear
- 📁 Files: specs/004-chapter-3-redesign/plan.md (revised and saved)
- 🔁 Next prompts: 1) `/sp.tasks` to generate implementation task checklist, 2) Lesson-writer subagent to create 8 files
- 🧠 Reflection: The 8-lesson structure is more aligned with Docosaurus conventions, provides better reader pacing, and enables modular validation. It's the correct architectural choice despite requiring more individual files.

## Evaluation notes (flywheel)

- Failure modes observed: None—plan revision clarifies direction
- Graders run and results: Plan PASS (architectural decisions sound, constitution compliant, feasible)
- Prompt variant: User feedback driven (not a variant, genuine course correction)
- Next experiment: Implement per plan, validate per-lesson quality, then chapter coherence

---

**Summary**: Chapter 3 plan revised to specify 8-lesson modular structure. Constitution check passed. Ready for `/sp.tasks` to generate implementation checklist. Estimated effort: 12-16 hours (8 lessons × 1.5-2 hours each including validation).

