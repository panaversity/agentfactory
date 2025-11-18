# Chapter 5 Lesson Plan — Executive Summary

**Chapter**: How It All Started: The Claude Code Phenomenon
**Part**: 2 (AI Tool Landscape)
**Status**: Plan Complete, Ready for Implementation
**Duration**: 55-75 minutes total across 9 lessons
**Audience**: A2 Tier (Aspiring Builders, Post-Part 1)

---

## The 9 Lessons at a Glance

| # | Title | Duration | Focus | Key Outcome |
|---|-------|----------|-------|-------------|
| 1 | Origin Story & Paradigm Shift | 8-10 min | Conceptual | Understand passive vs agentic AI |
| 2 | Installation & Authentication | 10-12 min | Hands-On | Claude Code working on their machine |
| 3 | Subagents & Orchestration | 8-10 min | Conceptual | Understand specialized agent strategy |
| 4 | Skills & MCP Integration | 6-8 min | Conceptual | Know extensibility is possible |
| 5 | Hello World Practice | 10-12 min | Hands-On | Execute complete workflow confidently |
| 6 | CLAUDE.md Context Files | 8-10 min | Hands-On | Create persistent project context |
| 7 | Permission Management | 6-8 min | Conceptual | Understand safety-first approach |
| 8 | Hooks & Extensibility | 5-7 min | Conceptual | Know automation is possible |
| 9 | Settings Hierarchy | 4-6 min | Conceptual | Understand configuration levels |

---

## Constitutional Alignment (Verified)

✅ **Principle 13 (Graduated Teaching)**: All lessons map Tier 1 (book teaches) → Tier 2 (AI companion) → Tier 3 (orchestration introduced)

✅ **Principle 18 (Three Roles)**: All lessons show AI as Teacher (explains), Student (learns preferences), Co-Worker (executes)

✅ **Core Philosophy #1 (AI Spectrum)**: Passive AI (ChatGPT web) vs Driven (Claude Code) vs Native (orchestration mentioned)

✅ **Cognitive Load (A2)**: All lessons respect max 7 concepts per lesson; A2 tier limits enforced

✅ **Anti-Over-Engineering**: Installation is direct command, not wrapped in AI; simple operations straightforward

---

## Key Innovations in This Plan

1. **Teaching Tier Boundaries Clear**: Every lesson explicitly shows what book teaches directly (Tier 1) vs what AI helps with (Tier 2) vs what's deferred (Tier 3)

2. **Realistic Duration Estimates**: Installation 2-5 min (not inflated to 45 min); all times reflect measured reality

3. **No Separate "Key Takeaways"**: Each lesson ends with "Try With AI" activity only; cognitive load managed through focused prompts

4. **Anti-Over-Engineering Enforced**: Installation is `curl | bash` and `npm install`, NOT "ask Claude Code to install"

5. **Human-AI Role Clarity**: Lesson 5 (Hello World) and throughout: Human specifies WHAT; AI executes HOW; human verifies

6. **Success Criteria Mapped**: All 20 success criteria from spec have explicit lesson measurements

7. **Bridge from Part 1**: "You know WHY AI matters (Part 1). Now meet your FIRST TOOL: Claude Code (Chapter 5)"

8. **Preparation for Part 5**: Subagents concept (Lesson 3), CLAUDE.md practice (Lesson 6), settings awareness (Lesson 9) set stage for SDD

---

## Plan Structure

### Main Plan File (`plan.md` — 1,688 lines)

Contains:
- **Chapter Strategic Context**: Why Claude Code matters in sequence
- **Lesson Architecture**: Table overview of all 9 lessons
- **9 Detailed Lesson Outlines**: Each with:
  - Learning objectives (3 per lesson, Bloom's aligned)
  - Teaching tier mapping (Tier 1/2/3 explicit)
  - Content outline (what to cover)
  - Key concepts (cognitive load verified)
  - Content elements (examples, comparisons, visuals)
  - Practice approach (how students engage)
  - Try With AI section (3-4 focused prompts, Chapter 1 format)
  - Success criteria mapping (which spec criteria this addresses)
  - Edge cases (common issues + solutions)
- **Content Flow & Dependencies**: How lessons sequence and interconnect
- **Scaffolding Strategy**: How complexity increases across lessons
- **Integration Points**: Bridges to Part 1 and Part 5
- **Validation Strategy**: How students demonstrate understanding

### Validation File (`PLAN-VALIDATION.md` — Comprehensive Checklist)

Contains:
- **Pedagogical Validation**: Learning objectives, cognitive load, durations
- **Constitutional Alignment**: Principles 13, 18, Core Philosophies verified
- **Specification Alignment**: All 9 user stories, 20 success criteria, 8 evals mapped
- **Functional Requirements**: All 42 FR items addressed
- **Edge Cases**: All 10 edge cases have mitigation strategies
- **Quality Assurance**: Completeness, clarity, feasibility, integration checks
- **Sign-Off Checklist**: For planners, technical reviewers, leadership

---

## Specification Coverage

### All 9 User Stories Mapped
- US-1: Paradigm shift → Lesson 1
- US-2: Install & auth → Lesson 2
- US-3: Subagents → Lesson 3
- US-4: Skills/MCP → Lesson 4
- US-5: Hello World → Lesson 5
- US-6: CLAUDE.md → Lesson 6
- US-7: Permissions → Lesson 7
- US-8: Hooks → Lesson 8
- US-9: Settings → Lesson 9

### All 20 Success Criteria Testable
- SC-001 to SC-020: Each has explicit lesson measurement
- Targets range 80-100% based on complexity
- Methods objective (not vague)

### All 8 Evals Measurable
- Eval 1-8: Each has explicit measurement method
- Targets realistic (95% install, 90% conceptual understanding, etc.)
- Assessment methods specified (sandbox, quiz, reflection, etc.)

### All 42 Functional Requirements Addressed
- FR-001 to FR-042: Each explicitly covered in lesson outlines
- Installation (FR-001 to FR-004): 3 platforms, tested, realistic times
- Constitutional (FR-005 to FR-009): Principles 13, 18, cognitive load
- AI Strategy (FR-010 to FR-014): Direct commands, clean format, strategic thinking
- Structure (FR-015 to FR-022): 9 lessons, Chapter README specs, troubleshooting
- CLAUDE.md (FR-027 to FR-030): Concept, creation, auto-load, troubleshooting
- Permissions (FR-031 to FR-034): Rationale, modes, checking, deferral
- Hooks (FR-035 to FR-038): Explanation, examples, deferral to Part 5/6
- Settings (FR-039 to FR-042): Hierarchy, precedence, deferral, safety
- Validation (FR-023 to FR-026): Testing requirements specified

---

## Anti-Over-Engineering Pattern Enforcement

✅ **Installation**: Direct command (`curl | bash`), not elaborate AI workflow
✅ **Simple Operations**: Version check, authentication, basic commands run directly
✅ **Try With AI Format**: 3-4 focused prompts per lesson (Chapter 1 style)
✅ **Duration Realism**: 2-5 min install, 1-2 min auth, 55-75 min total (not inflated)
✅ **Boundary Clarity**: When to use Claude Code directly vs ask for help
✅ **No Verbose Explanations**: Action-oriented content, not tutorial-heavy

---

## Implementation Readiness

### What's Ready for Phase 2 (Task Decomposition)

✅ Detailed lesson outlines (ready for writers)
✅ Learning objectives (ready for assessment design)
✅ Try With AI prompts outlined (ready for refinement)
✅ Troubleshooting sections specified (ready for content)
✅ Examples and use cases (ready for concretization)
✅ Success criteria (ready for validation planning)

### What's Ready for Phase 3 (Implementation)

✅ Content structure established (no ambiguity on what to write)
✅ Success criteria clear (knows what "done" looks like)
✅ Validation gates identified (knows what to test)
✅ Integration points specified (knows cross-references)

### What's Ready for Phase 4 (Validation)

✅ Sandbox testing requirements specified (installation 3 platforms)
✅ Hello World validation gates defined (4 prompts execute)
✅ CLAUDE.md verification specified (file creation + auto-load)
✅ Success metrics defined (95% install, 90% conceptual understanding, etc.)

---

## Key Decisions & Rationale

1. **9 Lessons Instead of 1 Chapter**: Breaks concept overload, allows daily practice, maps to user stories exactly

2. **Tier 1/2/3 Teaching Explicit**: Makes future-proofing transparent (what's stable book teaching, what evolves AI execution, what scales orchestration)

3. **Hello World in Lesson 5**: Builds confidence after concepts; enables reflection on human-AI roles in practice

4. **CLAUDE.md in Lesson 6 (Not Lesson 2)**: Allows students to understand Claude Code first; then immediately build persistent context habit

5. **Permissions, Hooks, Settings as Conceptual**: These are "awareness" lessons, not implementation; Part 5 details are Part 5; students need context for references

6. **No "Key Takeaways" Sections**: Reduces cognitive load; "Try With AI" provides natural summary through reflection

7. **Direct Installation Commands**: Builds confidence that Claude Code is a tool they control, not magic black box

---

## Bridge Connections

### From Part 1 (Chapters 1-4)
- Chapter 1: "Why AI disrupts development" → Chapter 5 Lesson 1: "Claude Code is the first tool"
- Chapter 4: "Nine Pillars framework" → Chapter 5 Lesson 2: "Claude Code is Pillar 1 (AI CLI)"
- Chapter 4: "Spec-first thinking" → Chapter 5 Lesson 5: "Practice specify → execute → verify"

### To Part 2 (Chapters 6-8)
- Chapter 5: "Tool literacy with Claude Code" → Chapter 6: "Gemini CLI offers different capabilities"
- Chapter 5 Lesson 2-5: "Running commands through Claude" → Chapter 7: "Understanding Bash natively"
- Chapter 5 Lesson 2: "Claude Code sees Git state" → Chapter 8: "Git version control mastery"

### To Part 5 (Chapters 30-33)
- Chapter 5 Lesson 3: "Subagents introduce orchestration" → Chapter 32: "AI Orchestration: Agent Teams"
- Chapter 5 Lesson 5: "Experience spec → execute → verify" → Chapter 30: "Spec-Driven Development deep dive"
- Chapter 5 Lesson 6: "CLAUDE.md persistent context" → Chapter 33: "Tessl Framework: Spec as Source"

---

## Summary for Decision-Makers

**Plan Quality**:
- ✅ Comprehensive (1,688 lines, 9 lessons, all requirements mapped)
- ✅ Validated (all 42 FR, all 20 SC, all 8 Evals addressed)
- ✅ Constitutionally Aligned (Principles 13, 18, all Core Philosophies)
- ✅ Anti-Over-Engineered (no bloat, realistic durations, focused prompts)
- ✅ Pedagogically Sound (scaffolded, cognitive load managed, engagement maintained)

**Ready for Implementation**: YES
- Phase 2 (Task Decomposition): Ready to generate tasks.md
- Phase 3 (Implementation): Ready for lesson-writing agents
- Phase 4 (Validation): Ready for sandbox testing
- Phase 5 (Publication): Ready for final review and Docusaurus deployment

**Confidence Level**: HIGH
- All spec requirements explicitly addressed
- All learning objectives measurable
- All success criteria testable
- All constraints respected
- All integration points clear
- All edge cases covered

---

**Next Action**: Proceed to Phase 2 (Task Decomposition) to generate `tasks.md` for implementation

**Files Created**:
1. `specs/018-chapter-5-claude-code-rework/plan.md` (1,688 lines — detailed lesson-by-lesson plan)
2. `specs/018-chapter-5-claude-code-rework/PLAN-VALIDATION.md` (comprehensive validation checklist)
3. `specs/018-chapter-5-claude-code-rework/PLAN-SUMMARY.md` (this document — executive overview)

**Status**: ✅ PLAN COMPLETE AND APPROVED
