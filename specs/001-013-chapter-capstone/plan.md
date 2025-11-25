# Implementation Plan: Chapter 15 - AI Sales Assistant Capstone

**Spec Reference**: `spec.md`
**Created**: 2025-11-25
**Status**: Draft

---

## Architecture Overview

Chapter 15 consists of **7 lessons + 1 quiz** that guide students through building a complete AI Sales Assistant with 4 interconnected features. The pedagogical architecture emphasizes **intelligence acceleration measurement**—students time each feature and observe decreasing implementation time.

### Lesson-to-User-Story Mapping

| Lesson | User Story | Primary Deliverable |
|--------|------------|---------------------|
| 01 | Setup | Project configured, constitution created |
| 02 | US1 | Lead Profiler working, baseline time recorded |
| 03 | US2 | ICP Scorer working, acceleration observed |
| 04 | US3 | Outreach Generator working, continued acceleration |
| 05 | US4 | Campaign Dashboard working, target: less than 50% of F1 time |
| 06 | US5 | 2-3 formalized skills using P+Q+P |
| 07 | US6 | Shipped product, retrospective document |
| Quiz | All | Knowledge assessment |

---

## Lesson Architectures

### Lesson 01: Project Setup + Constitution

**Objective**: Student creates new project and establishes constitution for AI Sales Assistant

**Structure** (Target: 15 min read, immediate action):

1. **Opening** (2-3 paragraphs max)
   - What we're building: AI Sales Assistant with 4 features
   - Why sales domain: practical business value, transferable skills
   - Intelligence acceleration preview: Feature 4 will be faster than Feature 1

2. **Hands-On: Project Creation**
   - Create project directory
   - Initialize Spec-Kit Plus (already installed from Chapter 14)
   - Verify configuration

3. **Hands-On: Write Constitution**
   - Define project vision (AI Sales Assistant)
   - Set quality constraints (output formats, validation requirements)
   - Establish non-goals (not building production SaaS)

4. **Hands-On: Create Time Tracker**
   - Simple mechanism to track feature build times
   - Template for recording start/end times per feature

5. **Try With AI**
   - Prompt: "Review my constitution and suggest any missing constraints for a sales assistant"
   - Prompt: "What questions should I answer before building a Lead Profiler?"

**Constitutional Compliance**:
- ✅ 90% hands-on (4 hands-on sections, 1 brief context)
- ✅ Ends with "Try With AI"
- ✅ No meta-commentary
- ✅ No forbidden sections

---

### Lesson 02: Feature 1 - Lead Profiler

**Objective**: Build complete Lead Profiler using full SDD-RI cycle, record baseline time

**Structure** (Target: 45-60 min execution):

1. **Opening** (2 paragraphs max)
   - Lead Profiler purpose: Analyze company website → structured profile
   - Start your timer now (explicit instruction)

2. **Hands-On: Write Specification**
   - Run `/sp.specify` for Lead Profiler feature
   - Define: Input (company URL), Output (structured JSON profile)
   - Success criteria: Profile contains company name, industry, size estimate, tech stack hints, pain points

3. **Hands-On: Generate Plan and Tasks**
   - Run `/sp.plan` to create implementation plan
   - Run `/sp.tasks` to generate task checklist
   - Review generated artifacts

4. **Hands-On: Implement**
   - Run `/sp.implement` to build Lead Profiler
   - Work through tasks with AI assistance
   - Handle any issues that arise

5. **Hands-On: Test and Validate**
   - Test with sample company URL
   - Verify output matches specification
   - Record end time
   - Calculate total time: _____ minutes

6. **Try With AI**
   - Prompt: "My Lead Profiler took [X] minutes. What patterns from this build could I reuse?"
   - Prompt: "Generate 3 more test URLs to validate my Lead Profiler"

**Constitutional Compliance**:
- ✅ 90% hands-on (5 hands-on sections)
- ✅ Explicit time tracking
- ✅ Ends with "Try With AI"
- ✅ No framework labels

---

### Lesson 03: Feature 2 - ICP Scorer

**Objective**: Build ICP Scorer consuming Lead Profiler output, observe first acceleration

**Structure** (Target: 30-40 min execution):

1. **Opening** (2 paragraphs max)
   - ICP Scorer purpose: Score leads against criteria
   - Start timer, reference F1 time as baseline

2. **Hands-On: Define ICP Criteria**
   - Create Ideal Customer Profile definition
   - Criteria: company size, industry fit, tech stack compatibility, budget indicators
   - Weights for each criterion

3. **Hands-On: Write Specification**
   - Run `/sp.specify` for ICP Scorer
   - Input: Lead Profile JSON from F1
   - Output: Score (0-100) + reasoning
   - Notice: Spec writing feels faster (patterns from F1 apply)

4. **Hands-On: Plan, Task, Implement**
   - Run `/sp.plan` and `/sp.tasks`
   - Run `/sp.implement`
   - Notice: Less friction than F1

5. **Hands-On: Test with Real Data**
   - Run Lead Profiler → pipe output to ICP Scorer
   - Verify score and reasoning make sense
   - Record end time
   - Calculate: F2 time = _____ minutes (compare to F1: _____ minutes)

6. **Try With AI**
   - Prompt: "Compare my F1 and F2 times. What specific patterns transferred?"
   - Prompt: "How could I make ICP criteria more sophisticated for my industry?"

**Constitutional Compliance**:
- ✅ 90% hands-on
- ✅ Data flow established (F1 → F2)
- ✅ Time comparison explicit
- ✅ Ends with "Try With AI"

---

### Lesson 04: Feature 3 - Outreach Generator

**Objective**: Build Outreach Generator using ICP score for personalization, continue acceleration

**Structure** (Target: 25-35 min execution):

1. **Opening** (2 paragraphs max)
   - Outreach Generator purpose: Personalized messages based on ICP score
   - Timer start, note F1 and F2 times

2. **Hands-On: Define Outreach Templates**
   - High-score leads: Direct value proposition
   - Medium-score leads: Educational approach
   - Low-score leads: Nurture sequence or skip

3. **Hands-On: Specification and Implementation**
   - Run `/sp.specify` for Outreach Generator
   - Input: Lead Profile + ICP Score
   - Output: Personalized email/message
   - Run `/sp.plan`, `/sp.tasks`, `/sp.implement`

4. **Hands-On: Test End-to-End Flow**
   - URL → Lead Profiler → ICP Scorer → Outreach Generator
   - Verify personalization matches ICP score
   - Record end time
   - Calculate: F3 time = _____ minutes

5. **Try With AI**
   - Prompt: "Review my outreach output. How could the personalization be improved?"
   - Prompt: "Generate 5 variations of my outreach template for A/B testing"

**Constitutional Compliance**:
- ✅ 90% hands-on
- ✅ Three-feature data flow
- ✅ Time tracking continues
- ✅ Ends with "Try With AI"

---

### Lesson 05: Feature 4 - Campaign Dashboard

**Objective**: Build Campaign Dashboard aggregating all outputs, achieve target acceleration (less than 50% of F1 time)

**Structure** (Target: 20-30 min execution):

1. **Opening** (2 paragraphs max)
   - Dashboard purpose: Unified view of all campaign data
   - Target: Complete this in less than half your F1 time
   - Timer start

2. **Hands-On: Define Dashboard Scope**
   - Display: Lead profiles list
   - Display: ICP scores with visual indicators
   - Display: Outreach drafts with edit capability
   - Simple CLI or web output (student choice)

3. **Hands-On: Specification and Implementation**
   - Run `/sp.specify` for Campaign Dashboard
   - Run `/sp.plan`, `/sp.tasks`, `/sp.implement`
   - This should feel significantly faster

4. **Hands-On: Complete Integration Test**
   - Process 3 company URLs through full pipeline
   - View all data in dashboard
   - Record end time
   - Calculate: F4 time = _____ minutes
   - Compare: F4 vs F1 (target: F4 less than 50% of F1)

5. **Try With AI**
   - Prompt: "Analyze my feature times: F1=[X], F2=[X], F3=[X], F4=[X]. What drove the acceleration?"
   - Prompt: "What would a Feature 5 look like, and how long should it take?"

**Constitutional Compliance**:
- ✅ 90% hands-on
- ✅ Acceleration target explicit
- ✅ Full pipeline test
- ✅ Ends with "Try With AI"

---

### Lesson 06: Skill Creation + Polish

**Objective**: Formalize 2-3 reusable skills from accumulated patterns using P+Q+P framework

**Structure** (Target: 20 min):

1. **Opening** (2 paragraphs max)
   - Patterns you used repeatedly are skills waiting to be formalized
   - P+Q+P framework: Persona + Questions + Principles

2. **Hands-On: Identify Recurring Patterns**
   - Review your 4 feature specs
   - What patterns appeared in 2+ features?
   - Examples: "spec-for-data-transformer", "cli-output-formatter", "test-with-sample-data"

3. **Hands-On: Create First Skill**
   - Choose highest-value recurring pattern
   - Define Persona (who uses this skill)
   - Define Questions (what decisions does it guide)
   - Define Principles (what constraints apply)
   - Save as `.claude/skills/[skill-name].md`

4. **Hands-On: Create Second Skill**
   - Choose second pattern
   - Apply P+Q+P framework
   - Save skill file

5. **Hands-On: Test Skills on Hypothetical Feature**
   - Imagine Feature 5: "Lead Nurture Sequencer"
   - Apply your new skills mentally
   - Do they provide useful guidance?

6. **Try With AI**
   - Prompt: "Review my skill definition. Is the P+Q+P structure complete?"
   - Prompt: "Suggest improvements to make this skill more reusable across projects"

**Constitutional Compliance**:
- ✅ 90% hands-on
- ✅ P+Q+P framework applied (not explained abstractly)
- ✅ Tangible skill artifacts created
- ✅ Ends with "Try With AI"

---

### Lesson 07: Ship + Retrospective

**Objective**: Ship complete AI Sales Assistant, document acceleration achieved, identify transferable learnings

**Structure** (Target: 15 min):

1. **Opening** (2 paragraphs max)
   - Shipping means declaring "done" and reflecting
   - Your acceleration data tells the intelligence accumulation story

2. **Hands-On: Final Polish**
   - Run full pipeline one more time
   - Fix any remaining issues
   - Ensure all 4 features work end-to-end

3. **Hands-On: Calculate Acceleration Metrics**
   - Fill in time comparison table:
     | Feature | Time (min) | % of F1 |
     |---------|------------|---------|
     | F1 | _____ | 100% |
     | F2 | _____ | _____% |
     | F3 | _____ | _____% |
     | F4 | _____ | _____% |
   - Did F4 achieve less than 50% of F1?

4. **Hands-On: Write Retrospective**
   - What specific skills caused acceleration?
   - What would you do differently?
   - What skills transfer to your next project?
   - Save as `RETROSPECTIVE.md` in project root

5. **Hands-On: Ship Declaration**
   - Commit all code with message: "Ship: AI Sales Assistant v1.0"
   - Create brief README documenting what you built

6. **Try With AI**
   - Prompt: "Review my retrospective. What insights am I missing?"
   - Prompt: "Based on my acceleration data, what type of project should I build next?"

**Constitutional Compliance**:
- ✅ 90% hands-on
- ✅ Quantitative reflection (time data)
- ✅ Qualitative reflection (retrospective)
- ✅ Ends with "Try With AI"

---

### Quiz: Chapter 15 Assessment

**Objective**: Assess understanding of SDD-RI workflow and business intelligence

**Structure**:
- 10-12 questions
- Mix of: Multiple choice, True/False, Short answer
- Topics covered:
  - SDD-RI workflow steps
  - Intelligence accumulation concept
  - P+Q+P framework components
  - Specification writing principles
  - Business value of reusable skills

**Answer Distribution**: Balanced across A/B/C/D options (per quiz-generator skill)

---

## File Structure

```
book-source/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/
├── README.md (update with new overview)
├── 01-project-setup-constitution.md
├── 02-feature-1-lead-profiler.md
├── 03-feature-2-icp-scorer.md
├── 04-feature-3-outreach-generator.md
├── 05-feature-4-campaign-dashboard.md
├── 06-skill-creation-polish.md
├── 07-ship-retrospective.md
└── 08_chapter_15_quiz.md
```

---

## Constitutional Compliance Checklist

Before implementation, verify each lesson:

- [ ] Ends with "Try With AI" section only
- [ ] No "Key Takeaways", "Summary", "What's Next" sections
- [ ] No meta-commentary ("What to notice", "AI is teaching you")
- [ ] No Three Roles framework labels
- [ ] No `<number` MDX patterns
- [ ] 90% hands-on content, 10% context
- [ ] Proficiency B1 (7-10 concepts per section, moderate scaffolding)

---

## Dependencies

- **Chapter 14**: Students must have Spec-Kit Plus installed and configured
- **Chapter 13**: Students understand SDD-RI theory
- **Skills used**: skill-creator (referenced in Lesson 06), quiz-generator (for quiz)

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Student doesn't achieve 50% acceleration | Retrospective addresses "why not" as valid learning |
| Features too complex for time targets | Keep features minimal viable—prototype quality |
| ICP/Sales concepts unfamiliar | Provide minimal necessary context, focus on building |
| Quiz too difficult | Balance questions across difficulty levels |
