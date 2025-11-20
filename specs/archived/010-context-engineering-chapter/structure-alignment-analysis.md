# Chapter 10 Structure Alignment Analysis

**Date**: 2025-01-24  
**Task**: Align all 9 Chapter 10 lessons with Chapter 1 Lesson 1 template structure  
**Reference Template**: `book-source/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/01-moment_that_changed_everything.md`

---

## Template Structure Requirements

### Required Frontmatter Metadata

```yaml
---
title: "Lesson Title"
chapter: N
lesson: N
duration_minutes: N

# Skills metadata (hidden from students)
skills:
  - name: "Skill name"
    proficiency_level: "A1/A2/B1/B2/C1"
    category: "Conceptual/Technical/Soft"
    bloom_level: "Remember/Understand/Apply/Analyze/Evaluate/Create"
    digcomp_area: "Information Literacy / Communication & Collaboration / Content Creation / Safety / Problem-Solving"
    measurable_at_this_level: "Specific measurable outcome at this proficiency level"

# Learning objectives (visible, assessment-focused)
learning_objectives:
  - objective: "Specific measurable objective"
    proficiency_level: "A1/A2/B1"
    bloom_level: "Remember/Understand/Apply"
    assessment_method: "How achievement is measured"

# Cognitive load tracking
cognitive_load:
  new_concepts: N
  assessment: "N new concepts (...list...) within [A1: 5 / A2: 7 / B1: 10] limit ✓"

# Differentiation strategies
differentiation:
  extension_for_advanced: "What advanced learners can explore"
  remedial_for_struggling: "Simplified approach for struggling learners"
---
```

### Required Content Structure

1. **Title** (`# Lesson Title`)
2. **Learning Objectives Section** (visible, after title)
   ```markdown
   ## Learning Objectives
   
   By the end of this lesson, you will be able to:
   - **Action Verb** description (Proficiency Level - Bloom Level)
   - **Action Verb** description (Proficiency Level - Bloom Level)
   ```
3. **Main Content Sections** (lesson-specific)
4. **Try With AI Section** (MUST be last section)
   ```markdown
   ## Try With AI
   
   ### Prompt 1: ...
   ### Prompt 2: ...
   ### Prompt 3: ...
   ```

---

## Current State Analysis

### Lesson 1: `01-what-is-context-engineering.md`

**Current Frontmatter**:
```yaml
---
title: "Lesson 1: What is Context Engineering?"
sidebar_position: 1
description: "..."
keywords: [...]
---
```

**Missing**:
- ❌ `chapter: 10`
- ❌ `lesson: 1`
- ❌ `duration_minutes: 15`
- ❌ `skills` array
- ❌ `learning_objectives` array (has visible section but not metadata)
- ❌ `cognitive_load` object
- ❌ `differentiation` object

**Has**:
- ✅ Visible "Learning Objectives" section (lines 14-18)
- ❌ "Try With AI" section exists but mentions ChatGPT (should be Claude Code/Gemini CLI)

**Action Required**:
1. Add all missing frontmatter metadata
2. Convert visible learning objectives to frontmatter format
3. Define 3 skills taught
4. Count new concepts and assess cognitive load
5. Add differentiation strategies
6. Update "Try With AI" to use Claude Code instead of ChatGPT

---

### Lesson 2: `02-understanding-context-windows.md`

**Status**: Need to read and analyze (likely similar issues to Lesson 1)

---

### Lesson 3: `03-six-components-aidd-context.md`

**Current Frontmatter** (partial):
```yaml
---
title: "Lesson 3: The Six Components of AIDD Context"
sidebar_position: 3
description: "..."
keywords: [...]
---
```

**Has**:
- ✅ Visible "Learning Objectives" section (lines 15-18)

**Missing**: Same as Lesson 1 (all metadata except title)

**Action Required**: Same as Lesson 1

---

### Lessons 4-9

**Status**: Need to read and analyze each

**Expected Pattern**: All lessons likely missing:
- `chapter`, `lesson`, `duration_minutes` fields
- `skills` array
- `learning_objectives` array (has visible but not metadata)
- `cognitive_load` object
- `differentiation` object

---

## Proficiency Level Mapping (Constitution v3.0.0)

**Chapter 10 Complexity Tier**: Beginner → Intermediate (A1 → B1)

| Lessons | Tier | Proficiency | New Concepts Limit | Bloom Levels |
|---------|------|-------------|-------------------|--------------|
| 1-2 | Beginner | A1-A2 | Max 5-7 | Remember, Understand |
| 3-9 | Intermediate | A2-B1 | Max 7-10 | Understand, Apply, Analyze |

---

## Skills Categories (DigComp 2.1 Framework)

**Available Areas**:
1. **Information Literacy**: Finding, evaluating, organizing information
2. **Communication & Collaboration**: Interacting with AI agents
3. **Content Creation**: Building specifications, code, documentation
4. **Safety**: Security, validation, error handling
5. **Problem-Solving**: Debugging, architectural decisions

**Chapter 10 Primary Areas**:
- Information Literacy (understanding context)
- Communication & Collaboration (working with AI)
- Problem-Solving (debugging context issues)
- Safety (validation, guardrails)

---

## Bloom's Taxonomy Alignment

| Bloom Level | Action Verbs | Chapter 10 Use |
|-------------|--------------|----------------|
| **Remember** (A1) | Define, Identify, List, Recognize | Lessons 1-2: Basic concepts |
| **Understand** (A1-A2) | Explain, Compare, Classify, Summarize | All lessons: Core understanding |
| **Apply** (B1) | Use, Demonstrate, Apply, Execute | Lessons 3-9: Practical application |
| **Analyze** (B1+) | Analyze, Differentiate, Examine | Lessons 7-9: Advanced topics |
| **Evaluate** (B2+) | Evaluate, Justify, Critique | Not in Chapter 10 (beginner) |
| **Create** (C1+) | Design, Construct, Develop | Not in Chapter 10 (beginner) |

---

## Update Priority

**Phase 1: Critical Metadata** (All Lessons)
1. Add `chapter`, `lesson`, `duration_minutes`
2. Add `cognitive_load` (count concepts, assess against limit)

**Phase 2: Skills & Learning Objectives** (All Lessons)
3. Extract visible learning objectives
4. Convert to frontmatter `learning_objectives` array
5. Define `skills` array (2-3 skills per lesson)

**Phase 3: Differentiation** (All Lessons)
6. Add `extension_for_advanced`
7. Add `remedial_for_struggling`

**Phase 4: Tool Updates** (Lessons 1-2, possibly others)
8. Replace ChatGPT examples with Claude Code/Gemini CLI

---

## Next Steps

1. ✅ Read Lesson 1 (DONE)
2. ✅ Read Lesson 3 (DONE)
3. ⏳ Read remaining lessons 2, 4-9 (identify patterns)
4. ⏳ Create update plan for each lesson
5. ⏳ Execute updates systematically
6. ⏳ Validate all changes
7. ⏳ Update `index.md` if needed

---

## Notes

- **Tool Constraint**: All AI examples must use Claude Code or Gemini CLI (NO ChatGPT Web)
- **Consistency Goal**: All 9 lessons must have identical frontmatter structure
- **Hidden Metadata**: Skills/proficiency metadata is for institutional use, not visible to students
- **Visible Objectives**: Learning objectives appear both in frontmatter AND as visible section
- **Constitution Alignment**: All proficiency levels must align with constitution v3.0.0 graduated complexity guidelines
