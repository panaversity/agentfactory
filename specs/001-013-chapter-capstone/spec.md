# Feature Specification: Chapter 15 - AI Sales Assistant Capstone

**Feature Branch**: `001-013-chapter-capstone`
**Created**: 2025-11-25
**Status**: Draft
**Input**: Multi-feature capstone where students build an AI Sales Assistant with 4 features using Spec-Kit Plus

---

## Overview

Chapter 15 is the Part 4 capstone. Students apply SDD-RI skills from Chapters 13-14 to build a **complete AI Sales Assistant** with 4 interconnected features. The key outcome is **intelligence acceleration**: Feature 4 should take less than half the time of Feature 1, demonstrating accumulated reusable skills.

**Ratio**: 90% hands-on building, 10% context
**Domain**: Marketing/Sales (aligned with book's business focus)
**Layer**: L4 (Spec-Driven Integration) with L2-L3 embedded

---

## User Scenarios & Testing

### User Story 1 - Complete Feature 1: Lead Profiler (Priority: P1)

Student builds the Lead Profiler feature end-to-end using full SDD-RI workflow (spec → plan → tasks → implement). This is the foundational feature that establishes patterns for all subsequent features.

**Why this priority**: First feature teaches complete workflow. Student times this to baseline acceleration measurement.

**Independent Test**: Student has working Lead Profiler that takes a company URL and generates a structured lead profile. Student records time spent.

**Acceptance Scenarios**:

1. **Given** student has Spec-Kit Plus configured (from Chapter 14), **When** student writes spec for Lead Profiler, **Then** spec contains clear intent, success criteria, and constraints
2. **Given** spec exists, **When** student runs `/sp.plan` and `/sp.tasks`, **Then** implementation plan and task checklist are generated
3. **Given** tasks exist, **When** student runs `/sp.implement`, **Then** working Lead Profiler is created
4. **Given** implementation complete, **When** student tests with sample URL, **Then** Lead Profiler generates structured profile output

---

### User Story 2 - Complete Feature 2: ICP Scorer (Priority: P1)

Student builds ICP Scorer that consumes Lead Profiler output. Student notices acceleration—this feature builds faster because skills from F1 apply.

**Why this priority**: Demonstrates first intelligence reuse. Introduces data flow between features.

**Independent Test**: Student has working ICP Scorer that takes lead profile and outputs numerical score with reasoning. Time recorded shows reduction from F1.

**Acceptance Scenarios**:

1. **Given** Lead Profiler works, **When** student specs ICP Scorer, **Then** spec references lead profile as input
2. **Given** F1 spec/plan patterns exist, **When** student writes F2 spec, **Then** student reuses patterns (reduced effort)
3. **Given** implementation complete, **When** student tests with Lead Profiler output, **Then** ICP Scorer returns score + reasoning

---

### User Story 3 - Complete Feature 3: Outreach Generator (Priority: P1)

Student builds Outreach Generator that uses ICP score to personalize outreach. Acceleration continues—reusable patterns compound.

**Why this priority**: Demonstrates compounding acceleration. Highest value-add feature for sales domain.

**Independent Test**: Student has working Outreach Generator that produces personalized email/message. Time recorded shows continued reduction.

**Acceptance Scenarios**:

1. **Given** ICP Scorer works, **When** student specs Outreach Generator, **Then** spec uses ICP score + lead profile as inputs
2. **Given** F1-F2 patterns exist, **When** student implements F3, **Then** implementation time is noticeably faster
3. **Given** implementation complete, **When** tested with sample data, **Then** generates contextually relevant outreach

---

### User Story 4 - Complete Feature 4: Campaign Dashboard (Priority: P1)

Student builds Campaign Dashboard that aggregates all feature outputs. This feature should take less than half the time of Feature 1.

**Why this priority**: Demonstrates maximum intelligence accumulation. Measurable acceleration target.

**Independent Test**: Student has working dashboard. Time recorded is less than half of F1 time.

**Acceptance Scenarios**:

1. **Given** all features work, **When** student specs Dashboard, **Then** spec composes outputs from F1-F3
2. **Given** accumulated skills, **When** student implements Dashboard, **Then** time is less than 50% of F1 time
3. **Given** implementation complete, **When** tested, **Then** dashboard displays lead profiles, scores, outreach drafts

---

### User Story 5 - Formalize Reusable Skills (Priority: P2)

Student extracts recurring patterns into formal skills using P+Q+P framework. Creates 2-3 skills from accumulated patterns.

**Why this priority**: Explicit intelligence extraction. Creates reusable artifacts for future projects.

**Independent Test**: Student has 2-3 documented skills that can be applied to new features.

**Acceptance Scenarios**:

1. **Given** 4 features complete, **When** student analyzes patterns, **Then** identifies 2-3 recurring patterns
2. **Given** patterns identified, **When** student applies P+Q+P framework, **Then** creates formal skill definitions
3. **Given** skills created, **When** tested on hypothetical Feature 5, **Then** skills provide relevant guidance

---

### User Story 6 - Ship and Retrospective (Priority: P2)

Student ships complete AI Sales Assistant and reflects on acceleration achieved. Compares feature times and documents lessons learned.

**Why this priority**: Synthesis and reflection. Validates intelligence accumulation hypothesis.

**Independent Test**: Student has shipped product + reflection document showing F4 time vs F1 time.

**Acceptance Scenarios**:

1. **Given** all features complete, **When** student reviews time data, **Then** F4 time is less than 50% of F1 time
2. **Given** acceleration documented, **When** student writes retrospective, **Then** articulates specific skills that caused acceleration
3. **Given** retrospective complete, **When** student considers next project, **Then** has transferable skills identified

---

### Edge Cases

- **What happens when** Lead Profiler gets a malformed URL? → Lesson addresses input validation in spec
- **What happens when** ICP criteria aren't well-defined? → Lesson shows how to refine spec iteratively
- **What happens when** student doesn't achieve 50% acceleration? → Retrospective analyzes why, identifies improvement areas
- **What happens when** AI generates incorrect outreach tone? → Shows iteration through AI collaboration

---

## Requirements

### Functional Requirements (Educational Content)

- **FR-001**: Chapter MUST contain 7-8 lessons plus quiz
- **FR-002**: Each feature lesson MUST include timed execution tracking
- **FR-003**: Lessons MUST be 90% hands-on building, 10% context
- **FR-004**: Each lesson MUST end with "Try With AI" section only (no Key Takeaways, Summary, etc.)
- **FR-005**: Content MUST NOT contain meta-commentary ("What to notice", "AI is teaching you", role labels)
- **FR-006**: Content MUST avoid `<number` MDX patterns that break builds
- **FR-007**: Features MUST have data flow (F1 → F2 → F3 → F4)
- **FR-008**: Final lesson MUST compare feature times and validate acceleration
- **FR-009**: Skill creation lesson MUST use P+Q+P framework
- **FR-010**: Quiz MUST assess both technical and business understanding

### Key Entities

- **Lead Profile**: Company name, industry, size, tech stack, pain points, contact info
- **ICP Score**: Numerical score (0-100), scoring criteria, reasoning
- **Outreach Draft**: Personalized message, tone, call-to-action
- **Campaign Dashboard**: Aggregated view of all leads, scores, outreach status

### Data Flow Architecture

```
[Company URL] → F1: Lead Profiler → [Lead Profile JSON]
                                          ↓
                     F2: ICP Scorer ← [ICP Criteria]
                                          ↓
                                    [ICP Score + Reasoning]
                                          ↓
              F3: Outreach Generator ← [Outreach Templates]
                                          ↓
                                    [Personalized Outreach]
                                          ↓
                    F4: Campaign Dashboard
                              ↓
                    [Unified View of All Data]
```

---

## Success Criteria

### Measurable Outcomes

- **SC-001**: Student completes all 4 features using SDD-RI workflow
- **SC-002**: Feature 4 implementation time is less than 50% of Feature 1 time
- **SC-003**: Student creates 2-3 reusable skills using P+Q+P framework
- **SC-004**: Student can articulate which skills caused acceleration in retrospective
- **SC-005**: Quiz pass rate demonstrates understanding of SDD-RI + business intelligence
- **SC-006**: Zero build errors from MDX patterns
- **SC-007**: All lessons pass constitutional compliance (no meta-commentary, proper endings)

### Anti-Convergence Validation

- **AC-001**: Chapter 15 uses multi-feature approach vs Chapter 14's single-feature approach
- **AC-002**: Focus on acceleration measurement vs Chapter 14's workflow learning
- **AC-003**: Domain is sales/marketing vs Chapter 14's content creation (video)

---

## Lesson Structure

| Lesson | Title | Focus | Time Target |
|--------|-------|-------|-------------|
| 01 | Project Setup + Constitution | Configure project, set constitution | 15 min |
| 02 | Feature 1: Lead Profiler | Full SDD-RI cycle, baseline timing | 45-60 min |
| 03 | Feature 2: ICP Scorer | First reuse, measure acceleration | 30-40 min |
| 04 | Feature 3: Outreach Generator | Compounding reuse | 25-35 min |
| 05 | Feature 4: Campaign Dashboard | Maximum reuse, target: 50% of F1 | 20-30 min |
| 06 | Skill Creation + Polish | Formalize skills with P+Q+P | 20 min |
| 07 | Ship + Retrospective | Compare times, document learnings | 15 min |
| Quiz | Chapter 15 Quiz | Assess understanding | 10-15 min |

---

## Constraints

- **No theory-heavy content**: Maximum 2-3 paragraphs of context per lesson
- **No framework exposure**: Three Roles demonstrated but never labeled
- **No forbidden sections**: Key Takeaways, Summary, What's Next are prohibited
- **MDX safety**: Avoid patterns like `<10`, use `&lt;10` or "fewer than 10"
- **Proficiency**: B1 (7-10 concepts per section, moderate scaffolding)

---

## Non-Goals

- Teaching Python programming (that's Part 5)
- Deep marketing/sales theory (only what's needed for feature context)
- Building production-ready SaaS (prototype quality is sufficient)
- Teaching Claude Code/Gemini CLI setup (covered in Chapter 14)
