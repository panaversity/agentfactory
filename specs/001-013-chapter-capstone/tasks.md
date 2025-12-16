# Implementation Tasks: Chapter 15 - AI Sales Assistant Capstone

**Plan Reference**: `plan.md`
**Created**: 2025-11-25
**Status**: Ready for Implementation

---

## Task Overview

| ID  | Task                                             | Priority | Dependencies | Status |
| --- | ------------------------------------------------ | -------- | ------------ | ------ |
| T01 | Update README.md                                 | P1       | None         | ⬜     |
| T02 | Create Lesson 01: Project Setup + Constitution   | P1       | T01          | ⬜     |
| T03 | Create Lesson 02: Feature 1 - Lead Profiler      | P1       | T02          | ⬜     |
| T04 | Create Lesson 03: Feature 2 - ICP Scorer         | P1       | T03          | ⬜     |
| T05 | Create Lesson 04: Feature 3 - Outreach Generator | P1       | T04          | ⬜     |
| T06 | Create Lesson 05: Feature 4 - Campaign Dashboard | P1       | T05          | ⬜     |
| T07 | Create Lesson 06: Skill Creation + Polish        | P1       | T06          | ⬜     |
| T08 | Create Lesson 07: Ship + Retrospective           | P1       | T07          | ⬜     |
| T09 | Create Quiz                                      | P1       | T08          | ⬜     |
| T10 | Validate constitutional compliance               | P1       | T09          | ⬜     |
| T11 | Build and verify                                 | P1       | T10          | ⬜     |

---

## Detailed Tasks

### T01: Update README.md

**File**: `apps/learn-app/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/README.md`

**Action**: Replace existing theory-heavy README with hands-on capstone overview

**Content Requirements**:

- [ ] Title: "Chapter 15: AI Sales Assistant Capstone"
- [ ] Brief overview (3-4 paragraphs max)
- [ ] What you'll build: AI Sales Assistant with 4 features
- [ ] Intelligence acceleration preview
- [ ] Prerequisites: Chapters 13-14
- [ ] No "Week 1, Week 2" timeline language
- [ ] No forbidden sections (Key Takeaways, Summary, etc.)

**Frontmatter**:

```yaml
---
sidebar_position: 15
title: "Chapter 15: AI Sales Assistant Capstone"
---
```

---

### T02: Create Lesson 01 - Project Setup + Constitution

**File**: `apps/learn-app/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/01-project-setup-constitution.md`

**Sections**:

- [ ] Opening context (2-3 paragraphs)
- [ ] Hands-On: Project Creation (create directory, init Spec-Kit Plus)
- [ ] Hands-On: Write Constitution (project vision, quality constraints)
- [ ] Hands-On: Create Time Tracker (template for recording times)
- [ ] Try With AI (2 prompts)

**Validation**:

- [ ] No meta-commentary
- [ ] Ends with Try With AI
- [ ] 90% hands-on
- [ ] No `<number` patterns

---

### T03: Create Lesson 02 - Feature 1: Lead Profiler

**File**: `apps/learn-app/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/02-feature-1-lead-profiler.md`

**Sections**:

- [ ] Opening (2 paragraphs, explicit timer start)
- [ ] Hands-On: Write Specification (`/sp.specify`)
- [ ] Hands-On: Generate Plan and Tasks (`/sp.plan`, `/sp.tasks`)
- [ ] Hands-On: Implement (`/sp.implement`)
- [ ] Hands-On: Test and Validate (sample URL, record time)
- [ ] Try With AI (2 prompts about patterns and testing)

**Key Elements**:

- [ ] Time tracking explicit
- [ ] Full SDD-RI cycle shown
- [ ] Sample spec content provided
- [ ] Test validation included

---

### T04: Create Lesson 03 - Feature 2: ICP Scorer

**File**: `apps/learn-app/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/03-feature-2-icp-scorer.md`

**Sections**:

- [ ] Opening (2 paragraphs, timer + F1 reference)
- [ ] Hands-On: Define ICP Criteria (criteria + weights)
- [ ] Hands-On: Write Specification (input: Lead Profile JSON)
- [ ] Hands-On: Plan, Task, Implement (note: feels faster)
- [ ] Hands-On: Test with Real Data (pipe F1 → F2, record time)
- [ ] Try With AI (2 prompts about acceleration and criteria)

**Key Elements**:

- [ ] Data flow from F1 established
- [ ] Time comparison to F1 explicit
- [ ] ICP criteria example provided

---

### T05: Create Lesson 04 - Feature 3: Outreach Generator

**File**: `apps/learn-app/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/04-feature-3-outreach-generator.md`

**Sections**:

- [ ] Opening (2 paragraphs, timer + F1/F2 reference)
- [ ] Hands-On: Define Outreach Templates (high/medium/low score responses)
- [ ] Hands-On: Specification and Implementation
- [ ] Hands-On: Test End-to-End Flow (URL → F1 → F2 → F3)
- [ ] Try With AI (2 prompts about personalization)

**Key Elements**:

- [ ] Three-feature pipeline tested
- [ ] Template examples provided
- [ ] Continued acceleration noted

---

### T06: Create Lesson 05 - Feature 4: Campaign Dashboard

**File**: `apps/learn-app/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/05-feature-4-campaign-dashboard.md`

**Sections**:

- [ ] Opening (2 paragraphs, target: less than 50% of F1)
- [ ] Hands-On: Define Dashboard Scope
- [ ] Hands-On: Specification and Implementation
- [ ] Hands-On: Complete Integration Test (3 URLs, full pipeline)
- [ ] Try With AI (2 prompts about acceleration analysis)

**Key Elements**:

- [ ] Acceleration target explicit (50% of F1)
- [ ] Full pipeline integration
- [ ] All 4 features compose

---

### T07: Create Lesson 06 - Skill Creation + Polish

**File**: `apps/learn-app/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/06-skill-creation-polish.md`

**Sections**:

- [ ] Opening (2 paragraphs, P+Q+P intro)
- [ ] Hands-On: Identify Recurring Patterns
- [ ] Hands-On: Create First Skill (full P+Q+P)
- [ ] Hands-On: Create Second Skill
- [ ] Hands-On: Test Skills on Hypothetical Feature
- [ ] Try With AI (2 prompts about skill improvement)

**Key Elements**:

- [ ] P+Q+P framework applied (not just explained)
- [ ] Actual skill file created
- [ ] Reusability tested mentally

---

### T08: Create Lesson 07 - Ship + Retrospective

**File**: `apps/learn-app/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/07-ship-retrospective.md`

**Sections**:

- [ ] Opening (2 paragraphs)
- [ ] Hands-On: Final Polish
- [ ] Hands-On: Calculate Acceleration Metrics (time table)
- [ ] Hands-On: Write Retrospective (guiding questions)
- [ ] Hands-On: Ship Declaration (commit, README)
- [ ] Try With AI (2 prompts about insights and next project)

**Key Elements**:

- [ ] Quantitative data (time comparison table)
- [ ] Qualitative reflection template
- [ ] Ship commitment ritual

---

### T09: Create Quiz

**File**: `apps/learn-app/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/08_chapter_15_quiz.md`

**Requirements**:

- [ ] 10-12 questions
- [ ] Mix: Multiple choice, True/False, Short answer
- [ ] Topics: SDD-RI workflow, intelligence accumulation, P+Q+P, spec writing
- [ ] Balanced answer distribution (A/B/C/D)
- [ ] Hidden answers section

**Frontmatter**:

```yaml
---
sidebar_position: 8
title: "Chapter 15 Quiz"
---
```

---

### T10: Validate Constitutional Compliance

**Actions**:

- [ ] Run grep for meta-commentary patterns:
  ```bash
  grep -i "What to notice\|AI.*teach\|AI.*learn\|AI as\|AI now knows" *.md
  ```
- [ ] Verify no forbidden sections (Key Takeaways, Summary, What's Next)
- [ ] Verify all lessons end with "Try With AI"
- [ ] Check for `<number` MDX patterns
- [ ] Verify 90% hands-on ratio

---

### T11: Build and Verify

**Actions**:

- [ ] Run Docusaurus build:
  ```bash
  cd book-source && npm run build
  ```
- [ ] Fix any build errors
- [ ] Verify sidebar displays correctly
- [ ] Spot-check rendered content

---

## Implementation Order

Execute tasks in order T01 → T11. Each task depends on previous completion.

**Estimated Total**: 8 lesson files + 1 quiz + README update + validation

---

## Quality Gates

Before marking any lesson complete:

1. ✅ File created with correct path
2. ✅ Frontmatter correct (sidebar_position, title)
3. ✅ Ends with "Try With AI" section
4. ✅ No forbidden patterns (grep check)
5. ✅ No forbidden sections
6. ✅ 90% hands-on content
