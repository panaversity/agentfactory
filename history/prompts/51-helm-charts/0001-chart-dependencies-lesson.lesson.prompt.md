---
phr_id: 0001
stage: lesson
feature: 51-helm-charts
title: "Chart Dependencies Lesson"
author: claude-haiku-4-5
date: 2025-12-23
---

# Lesson 4: Chart Dependencies — Chapter 51 (Helm Charts)

## PROMPT_TEXT

Write Lesson 4: Chart Dependencies for Chapter 51: Helm Charts for AI Services

**CRITICAL EXECUTION RULES**:
- Output file: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/04-chart-dependencies.md`
- Execute autonomously - DO NOT ask "Should I proceed?" or wait for confirmation
- DO NOT create new directories - write directly to the path specified
- Write the complete file after gathering context

**Context**:
- Chapter 51 in Part 7: AI Cloud Native Development
- Students completed Lessons 1-3 (templating, helpers, values)
- This lesson explains FROM SCRATCH: Chart.yaml dependencies section, subchart resolution, conditions for optional dependencies, tags for dependency groups, import-values for exposing subchart values, helm dependency update command, the charts/ directory, version constraints (semver), aliases for multiple instances

**Frontmatter**:
```yaml
---
sidebar_position: 4
chapter: 51
lesson: 4
duration_minutes: 50
title: "Chart Dependencies"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Compose charts with community subcharts like PostgreSQL and Redis"
cognitive_load:
  concepts_count: 9
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Add dependencies to Chart.yaml with version constraints"
    bloom_level: "Apply"
  - id: LO2
    description: "Use conditions to enable/disable optional dependencies"
    bloom_level: "Apply"
  - id: LO3
    description: "Configure subcharts through parent values"
    bloom_level: "Apply"
  - id: LO4
    description: "Use import-values to expose subchart configuration"
    bloom_level: "Apply"
  - id: LO5
    description: "Run helm dependency update to fetch subcharts"
    bloom_level: "Apply"
---
```

**Content Structure**:
1. Introduction: Why compose instead of build from scratch? (PostgreSQL, Redis as examples)
2. 9 Key Concepts with code and Output sections:
   - Chart.yaml dependencies section structure
   - Subchart resolution (how Helm finds dependencies)
   - Conditions: `condition: redis.enabled`
   - Tags: Grouping dependencies for bulk enable/disable
   - import-values: Exposing subchart values to parent
   - helm dependency update command
   - The charts/ directory structure
   - Version constraints (semver: ^, ~, >=, <)
   - Aliases: Multiple instances of same chart

3. Exercises (6 total):
   - 4.1: Add Bitnami PostgreSQL as dependency with version constraint
   - 4.2: Add Redis dependency with condition (redis.enabled)
   - 4.3: Configure PostgreSQL via parent values: `postgresql.auth.database: mydb`
   - 4.4: Use import-values to expose PostgreSQL host
   - 4.5: Run `helm dependency update`, verify charts/ populated
   - 4.6: Create values-nodedb.yaml disabling Redis but enabling PostgreSQL

4. "Try With AI" section (5-part structure)

**Constitutional Requirements**:
- Every code block MUST have an **Output:** section
- End with "## Try With AI" as the ONLY final section
- NO framework labels, NO meta-commentary, NO MDX-breaking angle brackets

## RESPONSE_TEXT

Created `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/07-AI-Cloud-Native-Development/51-helm-charts/04-chart-dependencies.md`

**File Statistics:**
- 693 total lines
- 9 concepts with complete explanations and Output sections
- 6 practical exercises with progressive complexity
- Try With AI section with 5-part collaborative structure
- Ends with "## Try With AI" as only final section

**Content Validation:**
- Zero framework labels or meta-commentary (grep check)
- Every concept includes **Output:** demonstrating expected behavior
- Code examples show realistic Helm patterns: semantic versioning, conditions, aliases
- Exercises map directly to learning objectives
- Try With AI section activates bidirectional learning (Part 1: challenge description, Part 2: initial setup, Part 3: review, Part 4: configuration, Part 5: verification)

**Layer Alignment:**
- Layer 1 (Manual Foundation): Students manually create Chart.yaml, values.yaml, run helm dependency update
- B1 Proficiency: 9 concepts appropriate for intermediate students who've completed Lessons 1-3
- Cognitive Load: Moderate scaffolding with examples for each concept

**Constitutional Compliance:**
- Specification clarity: "Chart dependencies let you compose community charts"
- Progressive complexity: Semver constraints introduced after basic dependencies
- Minimal content: Every section serves learning objective
- Anti-convergence: Different teaching approach from Lessons 1-3 (focused on composition vs templating syntax)

