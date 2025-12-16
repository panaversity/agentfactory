# Lesson 10: Brownfield Adoption — Rewrite Report

**Date**: 2025-11-26
**Status**: Complete
**File**: `/apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/10-brownfield-adoption.md`

---

## Summary of Changes

### Previous State

- **Size**: 2054 lines
- **Focus**: Video project-specific brownfield adoption (Gemini.google.com + Playwright MCP)
- **Examples**: YouTube workflow, video generation, browser automation
- **Issues**:
  - Tightly coupled to video generation domain
  - Referenced Playwright MCP extensively
  - Gemini.google.com usage instructions throughout
  - Not generalizable to other project types

### New State

- **Size**: 332 lines (84% reduction)
- **Focus**: Generic brownfield adoption patterns applicable to ANY project
- **Examples**: Blog/website, API, documentation projects
- **Duration**: 30 minutes (aligns with chapter pace)
- **Proficiency**: B1 (Intermediate)
- **Concepts**: 4 new concepts (within B1 cognitive load limit of 10)

---

## Content Mapping

### 4 Core Concepts (Taught Progressively)

1. **Greenfield vs Brownfield** (Foundation section)

   - Definition: New project vs inherited project
   - Bloom's: Understand
   - Example contexts (blog, API, docs)

2. **What Gets Overwritten** (Foundation section)

   - CLAUDE.md replaced completely
   - .specify/ directory created
   - Code and tests remain untouched
   - Custom commands preserved
   - Bloom's: Understand

3. **Safe Testing Workflow** (Collaboration section)

   - Git branch isolation
   - Manual backup files
   - Git commit recovery point
   - Multiple recovery paths
   - Bloom's: Apply

4. **Content Integration Strategy** (Collaboration + Practice sections)
   - Coding standards → constitution.md
   - Architecture principles → constitution.md
   - AI collaboration patterns → append to CLAUDE.md
   - Merge workflow example
   - Bloom's: Analyze, Apply

### What Was Removed

**All references to**:

- ✅ Video project examples
- ✅ Gemini.google.com usage
- ✅ Playwright MCP commands
- ✅ YouTube upload automation
- ✅ Browser session management
- ✅ Video download procedures
- ✅ MCP installation verification

**All "experimental brownfield" complexity** reduced to:

- Safe git branch workflow
- Manual backup strategy
- Clear merge decision framework

### Pedagogical Structure

**Layer 2 (AI Collaboration) Elements**:

- Dialogue-based teaching showing AI suggesting safe workflow
- Student providing project context
- AI adapting recommendations to student's situation
- Natural narrative (no pedagogical labels)
- Three Roles framework INVISIBLE to reader

**Section Organization**:

1. **Foundation** — Understand greenfield/brownfield, what gets overwritten, examples
2. **Collaboration** — AI suggests safe workflow, student tests, results inspected
3. **Practice** — Self-check for content categories, planning adoption
4. **Try With AI** — Actual prompts for planning adoption on real project
5. **Checkpoint** — Reflection questions

---

## Constitutional Compliance

### Checked Against Constitution Principles

| Principle                     | Check                                          | Status                                                        |
| ----------------------------- | ---------------------------------------------- | ------------------------------------------------------------- |
| **Specification Primacy**     | Does lesson show intent before implementation? | ✓ Shows safe workflow BEFORE running init                     |
| **Progressive Complexity**    | Respects B1 cognitive load (≤10 concepts)?     | ✓ 4 concepts, well within limit                               |
| **Factual Accuracy**          | All claims verifiable?                         | ✓ Based on actual Spec-Kit Plus behavior                      |
| **Coherent Structure**        | Builds understanding progressively?            | ✓ Foundation → Collaboration → Practice                       |
| **Intelligence Accumulation** | Uses prior chapter learning?                   | ✓ Builds on Chapter 14 workflow knowledge                     |
| **Anti-Convergence**          | Varies from previous chapter?                  | ✓ Ch 13 theory → Ch 14 greenfield → Ch 10 brownfield strategy |
| **Minimal Content**           | Every section maps to objective?               | ✓ All content maps to learning objectives                     |

### Three Roles Framework (Invisible to Students)

- **AI as Teacher**: Suggests safe workflow pattern student hadn't considered
- **Student as Learner**: Provides project context, asks clarifying questions
- **AI as Adaptor**: Tailors recommendations based on student's project type
- **Framework Visibility**: INVISIBLE (students experience collaboration, not labeled framework)

---

## Project Type Examples

Lesson teaches with 3 parallel examples (blog, API, docs):

### Blog/Website Project

```
blog-project/
├── CLAUDE.md          (200 lines)
├── src/ (pages, components, styles)
├── README.md
└── .claude/commands/deploy-netlify.md
```

### API Project

```
api-project/
├── CLAUDE.md          (150 lines)
├── src/ (routes, models, middleware)
├── tests/
└── .claude/commands/ (run-tests, format-code)
```

### Documentation Project

```
docs-project/
├── CLAUDE.md          (100 lines)
├── docs/ (guides, api, tutorials)
└── .claude/commands/publish-docs.md
```

Shows same adoption workflow applies across domain types.

---

## Learning Objectives Met

| Objective                                    | Bloom's    | Assessment                                                        |
| -------------------------------------------- | ---------- | ----------------------------------------------------------------- |
| Understand greenfield vs brownfield contexts | Understand | Students can define both and identify which applies to a project  |
| Apply constitution-first approach            | Apply      | Students can categorize team knowledge and plan merging strategy  |
| Execute safe brownfield workflow             | Apply      | Students can run backup commands in right order before init       |
| Identify integration points                  | Analyze    | Students can decide content locations (constitution vs CLAUDE.md) |

---

## Duration Alignment

- **Lesson Duration**: 30 minutes (matches chapter pace)
- **Content Density**: ~332 lines with examples
- **Sections**: 6 (Foundation, Collaboration, Practice, Try With AI, Checkpoint, plus transitions)
- **Practical Components**: 3 code examples, 3 git workflow examples, 3 project scenarios

---

## Safety & Practicality

### What Students Learn to Do

- Create experimental git branch (isolation)
- Create `.backup` files (manual recovery)
- Commit before testing (git recovery point)
- Identify what gets overwritten vs preserved
- Categorize team knowledge for merging
- Execute merge workflow with confidence

### What Students Learn NOT to Do

- Run `init --here` without backups
- Assume CLAUDE.md survives init
- Lose team knowledge through careless execution
- Skip recovery planning for experimental features

### Safety Verification

```bash
# Students verify they understand before executing
git checkout -b experiment/specifykit     # Branch isolation
cp CLAUDE.md CLAUDE.md.backup            # Manual backup
git add -A && git commit ...             # Git recovery point
# THREE recovery methods ready before running init
```

---

## Alignment with Chapter Purpose

**Chapter 14 Goals**:

- Teach Spec-Kit Plus workflow on projects from start to finish
- Lesson 1-9: Greenfield (new project) workflow
- **Lesson 10: Brownfield adoption (extending to existing projects)**
- Lesson 11: Capstone (full reusable intelligence composition)

**Lesson 10's Role**:

- Bridge between Chapter-14-greenfield and real-world brownfield projects
- Prepare students for using Spec-Kit Plus on existing client/team projects
- Teach safety procedures for experimental features
- Show generalization across project types

---

## Files Affected

**Modified**:

- `/apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/10-brownfield-adoption.md`
  - Removed: 2054 → 332 lines (1722 lines deleted)
  - All video/Playwright/Gemini references removed (0 matches)
  - Generic examples added (blog, API, docs)

**Preserved**:

- `/specs/036-chapter-14-sdd-ri-hands-on/` (spec unchanged)
- Chapter README, other lessons unchanged

---

## Validation Checklist

- ✅ **Removed all video project examples** (0 video references found)
- ✅ **Removed MCP/Playwright references** (0 MCP references found)
- ✅ **Removed Gemini.google.com references** (0 Gemini references found)
- ✅ **Uses generic project examples** (blog, API, docs projects)
- ✅ **Teaches 4 concepts max** (greenfield/brownfield, overwrites, safe workflow, content strategy)
- ✅ **30-minute duration** (332 lines with examples = ~25-30 min reading + thinking)
- ✅ **B1 proficiency aligned** (moderate scaffolding, 4 concepts within cognitive limit)
- ✅ **Ends with "Try With AI"** (not "What's Next" or "Key Takeaways")
- ✅ **Includes checkpoint reflection** (self-check questions at end)
- ✅ **Constitutional compliance** (Layer 2, Three Roles invisible, minimal content)

---

## Notes for Next Steps

1. **Summary Generation**: Run `/summary-generator` on this lesson to create `.summary.md`
2. **Content Validation**: Run `/content-evaluation-framework` to check pedagogical effectiveness
3. **Capstone Connection**: Lesson 11 should reference brownfield concepts when capstone involves existing projects
4. **Chapter Context**: This lesson bridges greenfield (L1-L9) to professional brownfield adoption
