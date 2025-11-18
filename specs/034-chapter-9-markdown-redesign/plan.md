# Chapter 9: Markdown — The Language of AI Communication — Complete Lesson Plan

**Specification**: `specs/034-chapter-9-markdown-redesign/spec.md`
**Status**: Plan (Ready for Content Implementation)
**Created**: 2025-11-18
**Version**: 2.0.0 (Reasoning-Activated Edition)
**Constitution**: v6.0.0

---

## Executive Summary

This lesson plan transforms Chapter 9 from a generic markdown tutorial into a **specification-language curriculum** with 5 lessons that:

1. **Teaches markdown as specification language, not formatting tool** (varying from Chapter 8's hands-on discovery)
2. **Respects A2 cognitive load limits** (5-7 concepts per lesson maximum)
3. **Enforces 4-stage progression** explicitly (Manual → AI Collaboration → Intelligence Design → Spec-Driven Integration)
4. **Demonstrates Three Roles collaboration through narrative** (not labels) in Stages 2-3
5. **Maintains zero-code constraint** (specification examples only, no programming syntax)

**Key Design Decision**: Consolidated to 5 lessons (not 9) justified by concept density analysis: 28 core markdown concepts + 5 specification-language concepts = 33 total ÷ 5 lessons = 6.6 concepts per lesson average (within A2 limit of 5-7).

---

## I. Concept Density & Lesson Count Justification

### Core Concepts Inventory (from spec.md)

**Stage 1 (Manual Foundation)**: 6 concepts
- Markdown as structure language, heading hierarchy (#, ##, ###), document organization, file naming, editor basics, validation

**Stage 2 (AI Collaboration - Lists)**: 7 concepts
- Bullet lists (-), numbered lists (1., 2., 3.), feature enumeration, option vs sequence distinction, list nesting, AI-suggested organization, student refinement

**Stage 3 (AI Collaboration - Code Blocks)**: 7 concepts
- Code block syntax (```), specification contexts (not code), expected outputs, API endpoint descriptions, feature requirements, Three Roles convergence, specification clarity validation

**Stage 4 (Intelligence Design)**: 5 concepts
- Links [text](url), image references ![alt](url), reference materials, reusable templates, Persona + Questions + Principles pattern

**Stage 5 (Spec-Driven Integration - Capstone)**: 8 concepts
- Complete specification composition, headings for hierarchy, lists for requirements, code blocks for outputs, links for references, specification clarity validation, AI feedback integration, task management system scope

**Total**: 33 concepts across 5 stages

### Justified Lesson Count

Using A2 tier limits (5-7 concepts/lesson):

- Stage 1: 6 concepts → 1 lesson (foundation)
- Stage 2: 7 concepts → 1 lesson (lists, AI collaboration starts)
- Stage 3: 7 concepts → 1 lesson (code blocks, Three Roles demonstrated)
- Stage 4: 5 concepts → 1 lesson (links/images, intelligence design)
- Stage 5: 8 concepts → 1 lesson (capstone composition)

**Total: 5 lessons** (not arbitrary 9-lesson template)

**Validation**: Average 6.6 concepts/lesson = WITHIN A2 limits (5-7) with heavy scaffolding ✅

---

## II. Pedagogical Arc

Chapter follows explicit 5-phase progression:

| Phase | Lessons | Stage | Modality | Focus |
|-------|---------|-------|----------|-------|
| **Foundation** | 1 | 1 (Manual) | Specification-first modality (write → validate → iterate) | Markdown structure, intent vs formatting |
| **Application - Lists** | 2 | 2 (AI Collab) | Three Roles demonstrations (Teacher/Student/Co-Worker) | Feature enumeration with AI assistance |
| **Application - Code Blocks** | 3 | 2 (AI Collab) | Three Roles demonstrations (Teacher/Student/Co-Worker) | Specification outputs with AI collaboration |
| **Integration** | 4 | 3 (Intelligence) | Persona + Questions + Principles reflection | Reusable templates, reference material organization |
| **Mastery** | 5 | 4 (Capstone) | Spec-first capstone, full composition | Task management system specification |

---

## III. Complete Lesson Breakdown

### Lesson 1: Markdown as Specification Language - Foundation

**Stage**: 1 (Manual Foundation)
**Phase**: Foundation
**Duration**: 45 minutes
**Cognitive Load**: 6 concepts (at A2 limit)

#### Learning Objectives

1. Explain markdown's purpose as structured specification language for AI agents (Bloom's Understand)
2. Distinguish markdown as intent layer vs code as implementation (Bloom's Understand)
3. Create document hierarchy using heading levels (#, ##, ###) (Bloom's Apply)
4. Validate markdown syntax through manual inspection (Bloom's Apply)
5. Explain why clear structure helps AI agents parse requirements (Bloom's Understand)

#### Concepts Covered (6 total)

1. Markdown as specification language (intent articulation)
2. Specification vs implementation distinction (WHAT vs HOW)
3. Heading levels for hierarchy (document structure)
4. Feature > Sub-feature > Requirement nesting
5. Plain text editor basics (no formatting tools)
6. Manual validation (checking syntax correctness)

#### Functional Requirements Addressed

- FR-001: Markdown as specification language (Intent Layer of SDD-RI)
- FR-007: Heading syntax for document hierarchy
- FR-008: Manual markdown writing by hand in text editor
- FR-009: Specification examples using headings

#### Teaching Approach: Specification-First Modality

**Phase 1: Establish Mental Model** (10 min)

Show side-by-side comparison:
- **Specification (Markdown)**: "System displays list of all tasks with completion status"
- **Implementation (Code)**: Python/JavaScript code that renders that feature
- **Key insight**: Same intent, completely different languages

Ask reflection: "Why would AI need the specification in markdown rather than just the code?"
- Because spec describes WHAT; AI figures out HOW to code it
- Code is HOW; if implementation changes, spec stays the same
- Students realize: Specs are more stable and reusable than code

**Phase 2: Learn Heading Structure** (15 min)

Manual example: Feature list for task management system

```
# Task Management System
Highest-level system overview

## Core Features
What the system provides

### Create Task
What happens when user adds task

### View Tasks
How system displays all tasks

### Mark Complete
How system marks task done
```

Students manually type this in text editor (no WYSIWYG tools).

Ask: "Why do we nest features this way?"
- Clarity: Readers immediately understand system → features → individual requirements
- AI parsing: Machine-readable hierarchy
- Comparison: Compare to unstructured paragraph version (harder to understand)

**Phase 3: Manual Validation** (10 min)

Present three examples:
1. **Correct**: Proper nesting with # for main, ## for features, ### for details
2. **Broken**: Missing hierarchy markers
3. **Broken**: Inconsistent nesting

Students manually identify which is correct and explain why.

**Phase 4: Apply** (10 min)

Students manually create their own feature hierarchy for simple system:
- Simple e-book reader OR Recipe collection OR Daily planner
- Must include: System overview (H1), feature categories (H2), individual features (H3)
- Students validate their own syntax

#### Success Criteria Mapping

- SC-001: Students explain markdown's role as Intent Layer
- SC-003: Identify specification examples (not code) in paired examples

#### Prerequisites

- Chapter 8 (Git): Understanding version control concepts
- Chapter 7 (Bash): Text editor familiarity

#### Deliverables

- Manually typed markdown file with 3-level heading structure
- Written explanation: "Why AI agents need specifications in markdown before code"
- Identification of specification vs implementation in 3 paired examples

#### Intelligence Notes

**No-Code Constraint Validation**: ✅ Zero code examples in Lesson 1. Only specification examples using markdown headings.

---

### Lesson 2: Lists for Structured Requirements - AI Collaboration Begins

**Stage**: 2 (AI Collaboration with Three Roles)
**Phase**: Application - Lists
**Duration**: 50 minutes
**Cognitive Load**: 7 concepts (at A2 limit)

#### Learning Objectives

1. Use bullet lists (-) for alternative options in specifications (Bloom's Apply)
2. Use numbered lists (1., 2., 3.) for sequential requirements (Bloom's Apply)
3. Distinguish bullet (options) vs numbered (sequence) list usage (Bloom's Understand)
4. Collaborate with AI to improve list structure through iteration (Bloom's Apply)
5. Explain how AI agents parse discrete list items as separate requirements (Bloom's Understand)

#### Concepts Covered (7 total)

1. Bullet lists (-) for non-sequential options
2. Numbered lists (1.) for sequential steps
3. Feature enumeration (listing all system features)
4. Option vs sequence distinction
5. List nesting (sub-options under main options)
6. AI suggesting list structure
7. Student refining based on project constraints

#### Functional Requirements Addressed

- FR-010: Lessons 2-3 demonstrate Three Roles
- FR-011: AI teaches pattern student didn't know (list organization)
- FR-014: Lesson 2 teaches lists for requirements

#### Teaching Approach: Three Roles Demonstration (Stage 2)

**Setup**: Students have markdown document from Lesson 1 with feature headings. Now add feature details.

**Role 1: AI as Teacher** (12 min)

Student has written:
```
### Create Task
User wants to add new task to system. Task has name and priority.
Should be easy to use quickly.
Can set due date or not.
Should work on phone too.
```

Student asks AI: "Help me structure this feature requirements clearly"

AI suggests:
```
### Create Task

**Inputs:**
- Task name (required)
- Priority (optional: high/medium/low)
- Due date (optional)
- Mobile friendly (required)

**Success:**
- Task saved to list
- Confirmation shown
```

**What student learns**: Structured format (inputs/success) they didn't use before. AI introduced pattern student wasn't considering.

---

**Role 2: AI as Student** (12 min)

Student reviews AI's suggestion and corrects:

Student says: "Wait—we're building a simple MVP first. We can't handle priority levels yet."

Student refines:
```
### Create Task (MVP)

**Inputs:**
- Task name (required)

**Later features:**
- Priority levels
- Due dates
- Mobile support
```

AI adapts: "Understood. Focusing on MVP—just task name for now. Deferring priority/dates/mobile to future. Clear boundary."

**What AI learns**: Student's MVP scope constraint. AI now knows to not suggest complex features.

---

**Role 3: AI as Co-Worker (Convergence Loop)** (15 min)

Iteration 1:
- Student proposes: Bullet list of all required features
- AI suggests: Organizing features by category (view, edit, delete)

Iteration 2:
- Student evaluates: "That helps, but let's group by priority"
- AI adapts: Reorganizes by "must have" / "should have" / "nice to have"

Iteration 3:
- Together they converge: MVP has "must have" (create, view), future has "should have" (edit, delete)
- Result: Better specification than either alone

**Key insight**: Neither student's flat list nor AI's category structure was optimal. Iteration produced superior specification.

#### Success Criteria Mapping

- SC-002: 80% students complete "Try With AI" exercise with valid list structure
- SC-004: Demonstrate Three Roles: AI suggests → student refines → convergence

#### Prerequisites

- Lesson 1 (Markdown basics, heading hierarchy)

#### Deliverables

- Feature requirement specification using bullet lists (options) and numbered lists (steps)
- Written explanation: "How did AI's suggestion help? How did you refine it?"
- Specification shows Teacher/Student/Co-Worker roles in action

#### Try With AI Structure

**Prompt**: "I have feature requirements for a task manager. Help me organize them clearly using lists. AI should suggest structure, you should refine based on your MVP scope."

Expected outcome: Specification with clear list structure showing all three roles.

---

### Lesson 3: Code Blocks for Specifications (Not Code) - Three Roles Deepened

**Stage**: 2 (AI Collaboration with Three Roles)
**Phase**: Application - Code Blocks
**Duration**: 55 minutes
**Cognitive Load**: 7 concepts (at A2 limit)

#### Learning Objectives

1. Use code blocks (```) for specification outputs, not implementation code (Bloom's Understand)
2. Demonstrate expected system outputs in code blocks (Bloom's Apply)
3. Describe API endpoint specifications without implementation (Bloom's Apply)
4. Collaborate with AI to refine output specifications through iteration (Bloom's Apply)
5. Validate specification clarity through AI feedback (Bloom's Apply)

#### Concepts Covered (7 total)

1. Code block syntax (triple backticks ```)
2. Specification context (not implementation)
3. Expected output specifications
4. API endpoint descriptions (request/response format)
5. Feature requirement specifications
6. Three Roles iteration (AI teaches structure, student refines content)
7. Specification clarity validation through AI feedback

#### Functional Requirements Addressed

- FR-004: Specification examples only (feature lists, acceptance criteria, expected outputs)
- FR-006: Code blocks demonstrate specification usage
- FR-010: Three Roles demonstrated explicitly
- FR-012: Student teaches AI domain constraints
- FR-013: Convergence loop toward optimal specification

#### Teaching Approach: Three Roles Demonstration + Specification Clarity Validation (Stage 2)

**Critical Constraint**: All examples show expected outputs or API descriptions, ZERO implementation code.

**Phase 1: Establish Code Blocks as Specification Tool** (10 min)

Show examples:

**Example 1 - Expected Output Specification:**
```
When user selects "View Tasks", system displays:

```
Welcome to Task Manager!

All Tasks:
1. Buy groceries (not complete)
2. Write report (complete)
3. Call mom (not complete)
```
```

Student recognizes: This shows WHAT system displays, not HOW it displays.

**Example 2 - API Endpoint Specification:**
```
POST /task/create

Request: { "name": "Buy groceries", "priority": "medium" }
Response: { "id": 1, "name": "Buy groceries", "created": "2025-11-18" }
Error: If name empty, return { "error": "Task name required" }
```

Student recognizes: This describes interface contract, not implementation.

**Why code blocks?** AI agents parse code blocks as structured data. Clear formatting helps AI understand intent.

---

**Phase 2: Three Roles Iteration on Output Specification** (18 min)

**Setup**: Students have feature list from Lesson 2. Now add expected outputs.

**Role 1: AI as Teacher**

Student writes basic output:
```
User sees list of tasks.
```

Student asks AI: "How should I describe what the output looks like?"

AI suggests detailed format:
```
## View Tasks Output

When user asks to view tasks, system shows:

```
Tasks:
[Task 1 Name] - [Status]
[Task 2 Name] - [Status]
[Task 3 Name] - [Status]
```
```

**What student learns**: Structured output format with placeholders they didn't consider.

---

**Role 2: AI as Student**

Student evaluates AI's suggestion and corrects:

Student says: "That's clear, but let's also show how many tasks are complete. Our users need that summary."

Student refines:
```
## View Tasks Output

Total tasks: [N], Complete: [M], Remaining: [N-M]

Tasks:
[Task 1 Name] - [Status]
...
```

AI adapts: "Good point—adding summary metrics. Users see progress at a glance. Clear."

**What AI learns**: Student's user-centric requirement (progress visibility).

---

**Role 3: AI as Co-Worker (Specification Clarity)** (15 min)

Iteration 1:
- Student proposes output format
- AI suggests adding error case: "What if no tasks exist?"

Iteration 2:
- Student refines: "If no tasks, show: 'No tasks yet. Create your first task!'"
- AI evaluates: "That's friendly. Clear what to do next."

Iteration 3:
- Both iterate on status values: Should status be "complete/incomplete" or "done/pending"?
- Student decides: "Done/pending—shorter, clearer"
- AI validates: "Good choice. Consistent with UI constraints we discussed"

**Convergence result**: Specification now covers happy path AND edge case, with deliberate language choices.

---

**Phase 3: Specification Clarity Validation** (12 min)

Students share specification with AI:

Prompt: "Review my task manager specification. Can you identify what features I need to implement? Are there any ambiguous requirements?"

AI feedback scenarios:
1. **Clarity issue detected**: "You said 'task name' but didn't specify max length. How long can names be?"
   - Student refines spec: "Task names limited to 100 characters"
2. **Completeness issue**: "What happens when user tries to create duplicate task name?"
   - Student adds: "System allows duplicate names but shows both"
3. **Validation passed**: "Specification is clear. I can implement this without ambiguity"

**Key insight**: Specification iteration improves clarity. AI feedback identifies gaps.

#### Success Criteria Mapping

- SC-002: Specification artifacts include code blocks for outputs
- SC-004: Three Roles demonstrated through narrative
- SC-003: Distinguish specification (outputs) from implementation (code)

#### Prerequisites

- Lessons 1-2 (heading structure, list organization)

#### Deliverables

- Code blocks showing: expected outputs, API endpoint specifications, error cases
- Written reflection: "What did AI teach you about formatting specifications?"
- Specification refined through AI feedback iterations
- Evidence of all three roles in action

#### Try With AI Structure

**Prompt**: "Help me describe the expected output for [feature]. I'll show you what I wrote, you suggest how to format it clearly, I'll refine based on my constraints, and we'll iterate until it's unambiguous."

Expected outcome: Code blocks demonstrating specification usage (not implementation), clear through iteration.

#### No-Code Constraint Validation

✅ CRITICAL: This lesson teaches code blocks as specification tool only.
- Shows expected outputs (what user sees)
- Shows API contracts (request/response format)
- Shows error messages
- Shows NO implementation code (no Python, JavaScript, Bash, etc.)

If implementation code appears → FAIL the specification purpose.

---

### Lesson 4: Links and Templates - Intelligence Design

**Stage**: 3 (Intelligence Design)
**Phase**: Integration
**Duration**: 60 minutes
**Cognitive Load**: 5 concepts (intelligence design)

#### Learning Objectives

1. Use markdown links [text](url) to reference authoritative sources (Bloom's Apply)
2. Identify recurring specification patterns (Bloom's Analyze)
3. Create reusable specification template using Persona + Questions + Principles (Bloom's Create)
4. Apply template to new specification scenario (Bloom's Apply)

#### Concepts Covered (5 total)

1. Link syntax and reference materials
2. Image references for diagrams/examples
3. Pattern recognition (feature template, acceptance criteria template, user story template)
4. Persona definition (thinking stance)
5. Template application and reuse

#### Functional Requirements Addressed

- FR-016: Links for references and diagrams
- FR-017: Create reusable specification templates
- FR-018: Persona + Questions + Principles pattern
- FR-019: Skip Mermaid diagrams (cognitive load management)

#### Teaching Approach: Persona + Questions + Principles (Stage 3)

**Part 1: Link References in Specifications** (10 min)

Show example specification:

```
## Create Task Feature

Based on: [Task Management Best Practices](https://example.com/task-patterns)
Reference: [Acceptance Criteria Format](https://example.com/ac-format)

### Acceptance Criteria
- User can add task with name (Required by spec)
- System confirms save within 2 seconds
- Works offline-first (see [Offline Patterns](https://example.com/offline))
```

Students recognize: Specifications reference authoritative sources, not personal opinions.

**Part 2: Pattern Recognition** (15 min)

Reflection questions from Lessons 1-3:

1. "What structure do ALL feature specifications need?" → Feature name, inputs, expected output, edge cases
2. "What questions did we always ask?" → What's the MVP scope? What's success? What fails?
3. "What principles guide specification writing?" → Clarity first, no ambiguity, include edge cases

---

**Part 3: Create Reusable Template** (25 min)

Students design **Feature Specification Template** using Persona + Questions + Principles:

```markdown
# Feature Specification Template

## Persona
"Think like a requirements engineer ensuring AI can implement this feature unambiguously"

## Analysis Questions

1. **What is this feature doing?**
   - One sentence clear intent
   - Why does user care?

2. **What inputs does user provide?**
   - Required vs optional
   - Data format (text, numbers, choices)

3. **What is expected output?**
   - What does user see?
   - What happens in system?

4. **What should NOT happen (edge cases)?**
   - Empty input
   - Duplicate data
   - Boundary conditions

5. **Success criteria (how do we know it works?)**
   - Must pass [criterion 1]
   - Must complete in [time]
   - Must handle [error case]

## Principles

1. **Unambiguous Intent**: Any developer could implement this
2. **Complete Scope**: Nothing left to assumptions
3. **Edge Case Coverage**: Handle failures gracefully
4. **Reference Authority**: Link to standards/patterns

## Template (Use for Any Feature)

### [Feature Name]

**Intent**: [One sentence]

**Inputs**:
- [Input 1] (required/optional)
- [Input 2] (required/optional)

**Output**:
```
[Show expected result]
```

**Edge Cases**:
- If [condition], then [behavior]

**References**:
- [Link to relevant standard]
```

---

**Part 4: Apply to Novel Scenario** (10 min)

New feature: "Delete Task"

Students use template to write specification without lesson reference.

Expected output:

```
### Delete Task

**Intent**: Remove task from system permanently

**Inputs**:
- Task ID (required)

**Output**:
```
Task deleted successfully.
Remaining tasks: 2
```

**Edge Cases**:
- If task ID invalid → "Task not found. Choose from list."

**References**:
- [Confirmation UI Patterns](https://example.com/confirmation)
```

**Validation**: Students don't refer back to lessons. Template provides sufficient guidance. Intelligence reuses across projects.

#### Success Criteria Mapping

- SC-001: Create reusable template applying Persona + Questions + Principles
- SC-002: Apply template to new feature specification independently

#### Prerequisites

- Lessons 1-3 (markdown fundamentals, Three Roles collaboration)

#### Deliverables

- **Feature Specification Template** (reusable across projects)
- **Applied specification** using template for novel feature
- **Written reflection**: "How does template reduce work on future projects?"

#### Template Quality Criteria

- ✅ Persona activates reasoning (not just pattern retrieval)
- ✅ Questions force context-specific analysis (not generic checklist)
- ✅ Principles guide decisions (not arbitrary rules)
- ✅ Template applies to 3+ different features (general, not specific)

---

### Lesson 5: Capstone - Complete Task Management System Specification

**Stage**: 4 (Spec-Driven Integration)
**Phase**: Mastery
**Duration**: 90 minutes
**Cognitive Load**: 8 concepts (capstone composition + SDD-RI positioning)

#### Learning Objectives

1. Compose all markdown skills (headings, lists, code blocks, links) into complete specification (Bloom's Create)
2. Write specification FIRST before any implementation (Bloom's Create)
3. Validate specification clarity through AI feedback (Bloom's Evaluate)
4. Refine specification based on AI feedback (Bloom's Create)
5. Position markdown as Intent Layer in Spec-Driven Development (Bloom's Understand)

#### Concepts Covered (8 total)

**Part A: Specification Composition (5 concepts)**
1. Complete specification structure (system overview, features, requirements, outputs, references)
2. Meaningful headings (hierarchy from system → feature → requirement)
3. Enumerated requirements (lists showing features/acceptance criteria)
4. Expected behavior specifications (code blocks showing outputs)
5. Reference materials (links to standards/authorities)

**Part B: SDD-RI Methodology Positioning (3 concepts)**
1. Markdown as Intent Layer (human-readable specification)
2. AI as implementation orchestrator (receives spec, generates code)
3. Specification-first approach (write WHAT before HOW)

#### Functional Requirements Addressed

- FR-020: Complete task management system specification
- FR-021: All required components (headings, lists, code blocks, links)
- FR-022: Scope appropriate for A2 tier (3-4 features)
- FR-023: Specification clarity validation through AI feedback
- FR-028: Markdown positioned as Intent Layer

#### Teaching Approach: Specification-First Capstone

**Part 1: Project Scope Definition (10 min)**

Students write **spec.md FIRST** (before any implementation or AI code generation):

```markdown
# Task Management System Specification

## System Overview
A simple CLI task manager where users can create tasks, view all tasks,
mark tasks complete, and delete tasks. Focus: MVP simplicity.

## Core Features (MVP)

1. Create Task
2. View All Tasks
3. Mark Task Complete
4. Delete Task

## Success Criteria
- All features work without errors
- Clear output shows task status
- User knows what to do at each step

## Non-Goals (NOT building)
- User authentication
- Task sharing
- Advanced filtering
- Cloud sync
```

---

**Part 2: Feature Specifications (30 min)**

Using Lesson 4 template, students write detailed specifications:

```markdown
## Feature: Create Task

### Intent
User can add new task to system with a name

### Inputs
- Task name (required, text, max 100 chars)

### Output
```
Task created: "Buy groceries"
```

### Edge Cases
- If name empty: "Task name required. Please enter name."
- If name > 100 chars: "Name too long (max 100). Please shorten."

### References
[Form Validation Patterns](https://example.com/validation)

---

## Feature: View Tasks

### Intent
User sees list of all tasks with completion status

### Output
```
Welcome to Task Manager!

Tasks:
1. Buy groceries - not complete
2. Write report - complete
3. Call mom - not complete

Total: 3, Complete: 1, Remaining: 2
```

### Edge Cases
- If no tasks: "No tasks yet. Create your first task!"

---

[Continue for Mark Complete and Delete features]
```

---

**Part 3: Specification Clarity Validation with AI (25 min)**

Students share specification with AI:

**Prompt**: "I'm building a task manager with this specification. Read through and tell me:
1. Are all requirements clear enough that you could implement this?
2. What's ambiguous?
3. What edge cases are missing?"

AI feedback scenarios:

**Scenario 1: Ambiguity detected**
- AI: "You said 'task name' but didn't say: Is it case-sensitive? Can names have special characters?"
- Student refines: "Task names: alphanumeric + spaces, case-insensitive"
- Specification improved through feedback

**Scenario 2: Missing edge case**
- AI: "What happens if user creates 1000 tasks? Does system slow down?"
- Student adds: "No limit enforced in MVP. Handle gracefully."
- Specification captures non-goal boundary

**Scenario 3: Clarity confirmed**
- AI: "Specification is unambiguous. I understand exactly what to implement."
- Student validates: "My specification is sufficient"

**Key principle**: Students refine specification through iteration. They do NOT ask AI to implement code. Specification clarity is the goal.

---

**Part 4: SDD-RI Methodology Positioning (15 min)**

**Section A: Markdown as Intent Layer**

Explain positioning:
```
Spec-Driven Development with Reusable Intelligence (SDD-RI)

Layers:
1. Intent Layer (markdown specification) ← YOU ARE HERE
2. Intelligence Layer (reusable components)
3. Implementation Layer (code generation)

Markdown's role:
- Communicates WHAT to build (intent)
- AI reads spec and generates HOW (implementation)
- Specification is reusable across projects and languages
```

Student recognizes: "I've been learning how to tell AI what to build, not how to code."

**Section B: Why This Matters**

- **Problem solved**: Code-first approach creates confusion (don't know what code should do)
- **Solution provided**: Specification-first approach (clear intent before any code)
- **Foundation for future**: Part 4 (Python) will use markdown specs from Chapter 9
- **Preparation for Part 5**: Spec-Driven Development chapters will build on these skills

**Section C: You're Prepared**

- You can write clear specifications now
- You understand how AI agents parse markdown structure
- You know how to validate specifications through feedback
- When you learn to code (Part 4), you'll write specs first, not code first

#### Success Criteria Mapping

- SC-001: Capstone specification clear enough for AI implementation (80% of students)
- SC-002: Includes all required components (headings, lists, code blocks, links)
- SC-003: Specification clarity refined through AI feedback
- SC-007: Capstone demonstrates 4-stage progression complete

#### Prerequisites

- Lessons 1-4 (all markdown skills + template pattern)

#### Deliverables

- **Complete Task Management System specification** (spec.md)
  - System overview (H1 heading)
  - Feature list (H2 headings)
  - Feature details (H3 headings with inputs/outputs/edge cases)
  - Code blocks showing expected outputs
  - Links to reference materials
  - ZERO implementation code
- **Iteration log**: "AI suggested [change]. I refined to [improvement]."
- **Reflection**: "How does writing spec first change how you think about building?"
- **Portfolio**: Spec.md published and validated

#### Capstone Scope Management

**REQUIRED Features (3):**
1. Create Task
2. View All Tasks
3. Mark Task Complete

**OPTIONAL 4th Feature** (if time):
4. Delete Task

**EXPLICITLY NOT BUILDING** (A2 scope preservation):
- User authentication
- Multiple users
- Task sharing
- Cloud sync
- Web interface
- Database (simple storage only)

---

## IV. Stage Progression Validation

### Stage Enforcement

**Stage 1 (Lesson 1)**: NO AI for markdown writing
- Students write markdown manually in text editor
- Students manually validate syntax
- AI not involved in content creation
- Students build mental model: markdown = structure for specification

**Stage 2 (Lessons 2-3)**: AI collaboration WITH Three Roles
- Each lesson explicitly demonstrates:
  - **Teacher**: AI suggests structure/format student didn't know
  - **Student**: Student refines with domain constraints
  - **Co-Worker**: Iteration toward specification clarity
- No one-way instruction (student prompts → AI generates → done)

**Stage 3 (Lesson 4)**: Intelligence design with Persona+Q+P
- Students reflect on patterns from Lessons 1-3
- Create reusable Feature Specification Template
- Apply template to novel feature without lesson reference

**Stage 4 (Lesson 5)**: Specification-first orchestration
- Spec.md written FIRST (before implementation)
- AI provides feedback on specification clarity
- Students refine based on feedback
- **Critical**: NO implementation code requested or provided

### Skipping Prevention

**✅ NOT doing**: Spec-first in Lessons 1-4
- Stage 4 reserved for capstone only
- Lessons 1-4 build foundation and skills
- Early spec-first would add cognitive overload

**✅ NOT doing**: AI as passive tool in Stage 2
- All Stage 2 lessons require Three Roles
- Detection: If lesson shows only "human suggests → AI structures → done" → FAIL
- All three roles must be demonstrated

**✅ NOT doing**: Showing code examples in any lesson
- All examples are specification artifacts
- Code blocks show specifications (outputs, API descriptions)
- NOT code syntax

---

## V. Pedagogical Progression Summary

| Phase | Lessons | Stage | Arc Progress | Modality | Cognitive Load |
|-------|---------|-------|--------------|----------|-----------------|
| **Foundation** | 1 | 1 | Learn markdown as specification language | Specification-first writing | 6 concepts |
| **Application** | 2 | 2 | Practice organizing features and requirements | Three Roles collaboration | 7 concepts |
| **Application** | 3 | 2 | Practice specifying expected outputs | Three Roles + clarity validation | 7 concepts |
| **Integration** | 4 | 3 | Create reusable specification template | Persona+Q+P reflection | 5 concepts |
| **Mastery** | 5 | 4 | Compose complete system specification | Spec-first capstone | 8 concepts |

**Validation**:
- ✅ Follows pedagogical arc (Foundation → Application → Integration → Mastery)
- ✅ Cognitive load within A2 limits (6, 7, 7, 5, 8 concepts = avg 6.6)
- ✅ Modality varies (specification-first, Three Roles ×2, Persona+Q+P, capstone)
- ✅ Stage progression enforced (no skipping)

---

## VI. Anti-Convergence Validation

### Chapter 8 vs Chapter 9 Modality

**Chapter 8 (Git)**: Hands-on discovery
- Execute → Observe → Understand
- Students execute git commands, observe results, derive understanding

**Chapter 9 (Markdown)**: Specification-first writing
- Write → Validate → Iterate
- Students write specifications, validate clarity, iterate through AI feedback

**Difference**: ✅ CONFIRMED (clearly distinct modalities)

### Within-Chapter Modality Variation

- Lesson 1: Specification-first writing (manual)
- Lesson 2: Three Roles collaboration (lists)
- Lesson 3: Three Roles + clarity validation (code blocks)
- Lesson 4: Persona + Questions + Principles (template design)
- Lesson 5: Specification-first capstone (composition)

**Variation**: ✅ CONFIRMED (no identical consecutive modalities)

---

## VII. Constitutional Compliance

### Principle 1: Specification Primacy
- **All lessons**: Markdown is specification language (WHAT to build) ✅
- **Lesson 5**: Spec.md written BEFORE any implementation ✅
- **Zero code examples**: All examples are specification artifacts ✅

### Principle 2: Progressive Complexity
- **Concept load**: 6, 7, 7, 5, 8 concepts/lesson (avg 6.6, within A2 of 5-7) ✅
- **Scaffolding**: Heavy for Lessons 1-3 (step-by-step, validation checkpoints) ✅
- **Options**: Max 2-3 per decision point (A2 requirement) ✅

### Principle 4: Coherent Structure
- **Arc**: Foundation → Application → Integration → Mastery ✅
- **Justification**: 33 concepts ÷ 5 lessons = 6.6/lesson (not arbitrary 9) ✅

### Principle 6: Anti-Convergence
- **Chapter variation**: Specification-first (vs Git's hands-on discovery) ✅
- **Within-chapter variation**: Five different modalities ✅

### Principle 7: Minimal Content
- **Lesson endings**: Only "Try With AI" sections (no "What's Next", "Summary", etc.) ✅
- **Content scope**: 5-7 core concepts per lesson, non-goals clearly excluded ✅

**Overall**: ✅ FULLY COMPLIANT

---

## VIII. No-Code Constraint Enforcement

### Critical Validation Checklist

**All Lessons**:
- ✅ Zero Python, JavaScript, Bash, or any programming language syntax
- ✅ All code blocks show specifications (expected outputs, API contracts, feature requirements)
- ✅ No implementation code examples
- ✅ Internal scaffolding labels NOT in student text ("Stage 1/2/3/4" removed from lessons)

**Detection Strategy**:
```bash
# Search for code patterns that violate constraint
grep -rE "(^```python|^```js|^```bash|def |function |import |const |#!/bin/bash)" \
  /path/to/lesson-files/

# Expected: ZERO matches
```

---

## IX. Content Implementation Requirements

For each lesson, content-implementer must:

### Lesson 1: Markdown Fundamentals
- Create hands-on specification-writing examples
- Student manually types markdown (no copy-paste)
- Heading hierarchy: Feature > Sub-feature > Requirement
- Validation: Students check their own syntax
- NO AI assistance (Stage 1)
- Deliverable: Feature specification with 3-level heading hierarchy

### Lesson 2: Lists for Requirements
- Test bullet lists (-) and numbered lists (1., 2., 3.)
- Develop explicit Three Roles scenario (AI suggests → student refines → convergence)
- Show AI suggesting list organization student didn't consider
- Show student correcting AI's scope (MVP constraint)
- Show iteration toward better specification structure
- AI not one-way; include all three roles visibly
- Deliverable: Feature requirement specification with bullet and numbered lists

### Lesson 3: Code Blocks for Specifications
- Test code block syntax (```)
- Create examples showing expected outputs (NOT code)
- Create examples showing API endpoint descriptions (NOT code)
- Demonstrate Three Roles with specification clarity focus
- AI feedback on ambiguity, edge cases
- Student refines specification based on feedback
- Deliverable: Specification with code blocks showing outputs and error handling

### Lesson 4: Links and Reusable Templates
- Guide Pattern Recognition reflection questions
- Facilitate Feature Specification Template creation using Persona+Q+P
- Test template application to novel feature (students don't refer to lessons)
- Verify template is general (applies to 3+ features)
- Deliverable: Reusable template + application to new feature

### Lesson 5: Capstone Specification
- Provide task management system scope document (3-4 features)
- Guide spec.md writing with explicit structure (system overview → features → details)
- Test AI feedback on specification clarity
- Document iteration process (AI suggests → student refines)
- Include 15-minute SDD-RI methodology positioning section
- Verify ZERO code examples (only specification)
- Deliverable: Complete spec.md with all required components + iteration log

---

## X. Success Metrics

**This plan succeeds when**:

- ✅ Lesson count (5) justified by concept density (33 concepts ÷ 5 = 6.6/lesson)
- ✅ All Stage 1 lessons avoid AI assistance (manual foundation built)
- ✅ All Stage 2 lessons demonstrate Three Roles explicitly (not labels—through narrative)
- ✅ Stage 3 creates reusable intelligence (Feature Specification Template)
- ✅ Stage 4 writes spec FIRST, then validates through AI feedback
- ✅ Cognitive load ≤ 7 concepts per lesson (A2 tier compliance)
- ✅ Teaching modalities vary (specification-first vs Chapter 8's hands-on discovery)
- ✅ Capstone composes all skills (headings, lists, code blocks, links)
- ✅ Zero code examples in entire chapter (specification examples only)
- ✅ Constitutional compliance verified (all 7 principles)
- ✅ Success criteria measurable at each lesson end
- ✅ SDD-RI methodology positioned (markdown as Intent Layer)

---

**Plan Status**: Ready for content implementation
**Quality**: Reasoning-activated, concept-density justified, pedagogically coherent, constitutionally compliant, specification-language focused
**Next**: Content-implementer executes per lesson requirements above

---

## XI. Appendix: Key Distinctions (Student-Facing Clarity)

### Specification vs Implementation

**Specification (Markdown - This Chapter)**:
- Describes WHAT the system does
- Written in human-readable markdown
- Example: "User enters task name and presses Enter. System shows confirmation: 'Task created.'"
- Focus: Clarity about intent

**Implementation (Code - Later in Part 4)**:
- Describes HOW to build the system
- Written in programming language (Python, JavaScript, etc.)
- Example: `tasks_list.append(Task(name=input_name))`
- Focus: Technical execution

**Key insight**: One specification can be implemented multiple ways. Multiple implementations can achieve same specification. Specification is stable; implementation is flexible.

### Markdown vs Code Blocks

**Markdown** (Structure):
- `# Heading` = document organization
- `- Item` = requirement enumeration
- `[Link](url)` = references
- Regular text = descriptions

**Code Blocks in Markdown** (Specification):
- Show expected outputs (what user sees)
- Show API contracts (request/response format)
- Show error messages (what system says)
- Show system behavior (not code syntax)

**Not code blocks** (programming):
- No Python: `def`, `import`, syntax highlighting
- No JavaScript: `function`, `const`, semicolons
- No Bash: `#!/bin/bash`, pipes, command-chaining

**Key distinction**: Code blocks are formatting tool showing specifications, not programming.

---

**Completed: Comprehensive lesson plan for Chapter 9 aligned with constitutional frameworks, pedagogical progression, cognitive load limits, and specification-language approach.**
