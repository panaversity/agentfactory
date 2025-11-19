# Feature Specification: Chapter 13 Redesign - Introduction to Python (Story-Based Learning)

**Feature Branch**: `014-chapter-13-redesign`
**Created**: 2025-01-18
**Status**: Draft
**Input**: Redesign Chapter 13 of Python for absolute beginners who have never programmed before. Use story-based learning (narrative journey), "WHAT then WHY" method, and progressive depth (start simple with "For Curious Learners" sections for advanced topics like bytecode/OOP).

**Context**: This is Part 4, Chapter 13 (first Python programming chapter). Students have completed Chapter 12 (UV package manager) but have ZERO programming experience. Chapter 14 (Data Types) already redesigned with excellent "Concept Before Syntax" pedagogy.

## Evals Section (Success Criteria First)

### What Does Success Look Like?

**For Students (Learning Outcomes)**:
- **EV-001**: 85%+ of absolute beginners can explain "what Python is" in their own words without technical jargon
- **EV-002**: 90%+ can write and execute their first "Hello, World!" program independently within 15 minutes of lesson completion
- **EV-003**: 80%+ can create a variable with type hint, understand what it does, and modify the value successfully
- **EV-004**: 75%+ can explain WHY variables need names and types (not just HOW to write syntax)
- **EV-005**: 70%+ complete capstone project (personal introduction program) demonstrating understanding of print(), variables, type hints, and basic input

**For Educational Quality (Constitutional Compliance)**:
- **EV-006**: All code examples execute successfully in Python 3.14+ (verified with test logs)
- **EV-007**: All Python claims verified against official Python.org documentation (with citations)
- **EV-008**: Cognitive load managed at A2 tier: max 5-7 concepts per lesson, heavy scaffolding
- **EV-009**: Story-based narrative maintained throughout chapter (not reverting to dry lecture)
- **EV-010**: "For Curious Learners" sections present for advanced topics (bytecode, OOP preview) without overwhelming beginners

**For Pedagogy (Teaching Effectiveness)**:
- **EV-011**: WHAT-then-WHY pattern applied consistently: concepts explained BEFORE syntax in every lesson
- **EV-012**: Story elements engage students: chapter unfolds like a narrative journey, not topic list
- **EV-013**: Progressive depth evident: Lessons 1-3 simple foundation, Lesson 4-5 build complexity, optional advanced sections don't disrupt flow
- **EV-014**: Zero "Stage 1/2/3/4" or "Three Roles Framework" labels in student-facing text (internal scaffolding hidden)
- **EV-015**: Each lesson ends with ONLY "Try With AI" section (no "What's Next", "Key Takeaways", "Summary")

### How We'll Measure Success

**Immediate Validation (During Implementation)**:
1. Code execution logs attached to each lesson (all examples tested)
2. Citation list present (Python docs, official sources for every claim)
3. Concept count audit per lesson (5-7 max for A2 compliance)
4. Grep validation: zero matches for forbidden patterns ("Stage", "Layer", "Three Roles Framework" as headers)
5. Section audit: lessons end with "Try With AI" only

**Post-Implementation Validation** (validation-auditor):
1. Pedagogical effectiveness check (story cohesion, WHAT-WHY pattern, progressive depth)
2. Constitutional compliance audit (Principles 1-7 applied)
3. Factual accuracy verification (all code, all claims validated)
4. Cognitive load assessment (A2 tier limits respected)
5. Anti-pattern detection (convergence on lecture-style, scaffolding exposure, meta-commentary)

**Student-Facing Validation** (After Deployment):
1. Completion rate tracking (target: 75%+ finish all lessons)
2. Capstone submission quality (demonstrate variables, print, type hints mastery)
3. Confusion indicators: forum questions reveal gaps (address in revisions)

---

## User Scenarios & Testing (Prioritized Learning Journeys)

### User Story 1 - First Programmer's Welcome (Priority: P1)

**Narrative**: Alex has never written a line of code. They open Chapter 13 excited but nervous. The chapter begins with a story: "Imagine teaching a robot to make your favorite sandwich..." This story shows that programming is just giving clear instructions. Alex understands Python is a language for talking to computers.

**Why This Priority**: Absolute beginners need motivation and demystification FIRST. If they feel overwhelmed in Lesson 1, they quit. Success here = retention.

**Independent Test**: Student can explain "what Python is" to a friend using analogies from the story (not memorized definitions).

**Acceptance Scenarios**:

1. **Given** Alex has never programmed, **When** they read Lesson 1's story-based introduction, **Then** they understand programming as "giving instructions to computers" and feel encouraged (not intimidated)

2. **Given** Alex finishes Lesson 1, **When** asked "What is Python?", **Then** they can explain in own words: "A language that lets me tell computers what to do, like recipes for cooking"

3. **Given** Alex completes "Hello, World!" example, **When** they see output in terminal, **Then** they experience first "aha moment" and want to continue

---

### User Story 2 - Variable Discovery Journey (Priority: P1)

**Narrative**: After first program success, Alex encounters the story: "Remember naming your pet? Variables are like labels for information your program needs to remember." The chapter shows WHY variables exist (computers need to remember things) BEFORE showing HOW to write them.

**Why This Priority**: Variables are foundational. Students must understand PURPOSE (what/why) before syntax (how). This journey prevents "memorize syntax without understanding" trap.

**Independent Test**: Student can create variables with type hints, modify values, and explain WHY variables need both names and types.

**Acceptance Scenarios**:

1. **Given** Alex understands variables conceptually (labels for memory), **When** they write `name: str = "Alex"`, **Then** they understand this creates a labeled box in memory holding text

2. **Given** Alex sees `age: int = 25`, **When** asked "why `:int`?", **Then** they explain type hints tell Python what KIND of data (not just syntax requirement)

3. **Given** Alex completes variable practice, **When** they try `age = "twenty-five"`, **Then** they understand type mismatch (even though Python allows it, it violates intent)

---

### User Story 3 - Building Confidence Through Capstone (Priority: P1)

**Narrative**: Alex has learned print(), variables, type hints, and basic input. The chapter concludes with a capstone story: "You're creating a digital introduction card, like those name tags at conferences." This integrates all concepts into ONE small program.

**Why This Priority**: Capstone validates understanding. If students can't build this independently, foundation is weak. This is the proof of P1/P2 success.

**Independent Test**: Student creates "introduction card" program using print(), variables with type hints, and user input—without copy/paste, with light AI guidance.

**Acceptance Scenarios**:

1. **Given** Alex has completed Lessons 1-4, **When** they start capstone project, **Then** they can plan the program (pseudocode): "Ask name, ask age, print greeting"

2. **Given** Alex writes capstone code, **When** they run program, **Then** it executes successfully and displays personalized introduction

3. **Given** Alex finishes capstone, **When** they reflect, **Then** they recognize how variables, type hints, and print() work together to solve a problem

---

### User Story 4 - Curious Learner Exploration (Priority: P2)

**Narrative**: Some students like Alex think "I wonder HOW Python actually works inside the computer..." The chapter includes "For Curious Learners" sections: bytecode, Python execution, indentation importance. These DON'T interrupt main story but reward curiosity.

**Why This Priority**: Advanced students need enrichment without main path becoming overwhelming. P2 because optional, but valuable for engagement.

**Independent Test**: Student CAN skip these sections and still master chapter. OR student CAN explore and gain deeper understanding (both paths valid).

**Acceptance Scenarios**:

1. **Given** Alex is curious about bytecode, **When** they read "For Curious Learners: How Python Runs Your Code", **Then** they understand compilation → bytecode → execution (but this isn't required for Chapter 13 mastery)

2. **Given** Alex skips advanced sections, **When** they complete main lessons + capstone, **Then** they still demonstrate full Chapter 13 competency

3. **Given** Alex explores OOP preview section, **When** they encounter classes in Chapter 24, **Then** they recognize concepts from Chapter 13 "sneak peek"

---

### User Story 5 - AI Collaboration Introduction (Priority: P2)

**Narrative**: Each lesson ends with "Try With AI" where Alex practices concepts with their AI companion. The story frames AI as a "coding buddy" who helps practice, not a tool that does work for them. Alex learns to ask good questions and validate AI suggestions.

**Why This Priority**: P2 because this is Stage 1 foundation chapter (manual first, AI second). AI collaboration is introduced gently, not central yet.

**Independent Test**: Student uses "Try With AI" sections to practice without becoming dependent (they CAN complete exercises manually).

**Acceptance Scenarios**:

1. **Given** Alex completes Lesson 2, **When** they reach "Try With AI", **Then** they understand AI helps PRACTICE (not replace understanding)

2. **Given** Alex asks AI to generate variable examples, **When** AI responds, **Then** Alex validates examples against lesson concepts (type hints correct? Names meaningful?)

3. **Given** Alex gets confused, **When** they ask AI for clarification, **Then** AI's explanation uses same story-based analogies from chapter (reinforcement)

---

### Edge Cases

**Absolute Beginner Variations**:
- **What if student has never used a terminal?** → Lesson 1 includes basic terminal primer ("black screen where you type commands")
- **What if student is intimidated by "coding"?** → Story-based approach demystifies: programming = instructions, like recipes
- **What if student makes syntax errors?** → Each lesson includes "Common Mistakes" section with friendly explanations (not cryptic error messages)

**Cognitive Load Boundaries**:
- **What if student finds story "too childish"?** → Balance narrative with real-world relevance (Python powers YouTube, Instagram)
- **What if student wants MORE advanced content?** → "For Curious Learners" sections satisfy without disrupting main path
- **What if student falls behind?** → Lessons build sequentially but each is self-contained (can return to Lesson 2 without re-reading 1)

**Technical Edge Cases**:
- **What if student's Python version differs?** → Chapter specifies "Python 3.14+" with note that 3.10+ mostly compatible (version checks in setup)
- **What if code examples fail?** → All examples tested with execution logs; if environment issue, troubleshooting guide in Chapter 12 reference
- **What if student uses Windows/Mac/Linux?** → Code examples are cross-platform; UV setup in Chapter 12 handles environment consistency

---

## Requirements

### Functional Requirements (Educational Content)

- **FR-001**: Chapter MUST begin with story-based introduction that explains programming as "giving instructions to computers" before any code syntax
- **FR-002**: Every concept MUST follow WHAT-then-WHY pattern: explain WHAT it is and WHY it matters BEFORE showing HOW syntax works
- **FR-003**: Chapter MUST include 5-6 core lessons establishing foundation (Lesson 1: What is Python, Lesson 2: First Program, Lesson 3: Variables, Lesson 4: Type Hints, Lesson 5: Capstone)
- **FR-004**: Each lesson MUST respect A2 cognitive load limits: 5-7 new concepts maximum per lesson, heavy scaffolding, max 2 options when choices presented
- **FR-005**: Advanced topics (bytecode, OOP preview, indentation deep dive) MUST be in "For Curious Learners" sections that don't interrupt main narrative
- **FR-006**: All code examples MUST execute in Python 3.14+ and include execution logs as verification
- **FR-007**: All Python technical claims MUST cite Python.org official documentation or authoritative sources
- **FR-008**: Chapter MUST include capstone project (personal introduction program) integrating print(), variables, type hints, and input()
- **FR-009**: Story-based narrative MUST maintain cohesion across lessons (not reverting to disconnected topic coverage)
- **FR-010**: Each lesson MUST end with ONLY "Try With AI" section (NO "What's Next", "Key Takeaways", "Summary", or standalone "Safety Notes")

### Pedagogical Requirements (Constitutional Compliance)

- **FR-011**: Chapter MUST apply Stage 1 teaching approach: manual foundation building BEFORE AI collaboration (AI introduced gently in "Try With AI" sections, not central)
- **FR-012**: Chapter MUST vary teaching modality from Chapter 14's "Concept Before Syntax" → Use story-based learning with narrative journey
- **FR-013**: Chapter MUST NOT expose internal scaffolding: zero "Stage 1/2/3/4", "Layer", "Three Roles Framework" as section headers in student-facing text
- **FR-014**: Chapter MUST include only concepts essential to Chapter 13 objectives (defer operators to Ch 15, control flow to Ch 17, etc. per non-goals)
- **FR-015**: Chapter README MUST explain learning journey, prerequisites, connection to AI-native development (as Chapter 14 README does)

### Quality Assurance Requirements

- **FR-016**: Chapter MUST pass validation-auditor checks: pedagogical effectiveness, constitutional compliance, factual accuracy, cognitive load assessment
- **FR-017**: Chapter MUST pass grep validation: zero forbidden patterns ("Stage [0-9]", "Layer [0-9]", "Three Roles (Framework|in Action)" in student text)
- **FR-018**: Chapter MUST pass section audit: all lessons end with "Try With AI" as final section (no sections after it)
- **FR-019**: Chapter MUST include test execution logs for ALL code examples (no untested code published)
- **FR-020**: Chapter MUST include citation list for all Python technical claims (version features, behavior, standard library docs)

### Key Entities (Educational Content Structure)

- **Chapter 13**: Introduction to Python for absolute beginners (Part 4, first programming chapter)
  - **Lessons 1-5**: Core progression (What is Python → First Program → Variables → Type Hints → Capstone)
  - **"For Curious Learners" Sections**: Advanced topics (bytecode, OOP preview, Python internals) optional for enrichment
  - **"Try With AI" Sections**: End each lesson, introduce gentle AI collaboration practice
  - **README**: Chapter overview, prerequisites, learning journey map, connection to AIDD

- **Story-Based Narrative**: Cohesive narrative thread across all lessons
  - **Opening Hook**: Programming as "teaching robot to make sandwich" (instruction-giving)
  - **Variable Introduction**: Variables as "pet name labels" (memory labels for data)
  - **Capstone Story**: "Digital introduction card" (conference name tag analogy)

- **Prerequisites**: Chapter 12 (UV package manager, Python 3.14+ installed)
- **Next Chapter**: Chapter 14 (Data Types) already redesigned with complementary pedagogy

---

## Success Criteria (Measurable, Technology-Agnostic)

### Student Learning Outcomes

- **SC-001**: 85%+ of students can explain "what Python is" using story analogies (not memorized definitions) after Lesson 1
- **SC-002**: 90%+ can write and execute "Hello, World!" program independently within 15 minutes of Lesson 2 completion
- **SC-003**: 80%+ can create variables with type hints and explain WHY types matter (not just HOW to write them) after Lesson 4
- **SC-004**: 75%+ complete capstone project demonstrating integrated understanding of print(), variables, type hints, and input()
- **SC-005**: Completion rate: 75%+ of students who start Chapter 13 finish all core lessons (not counting optional "For Curious Learners")

### Educational Quality Metrics

- **SC-006**: Code verification: 100% of code examples execute successfully in Python 3.14+ (verified with test logs)
- **SC-007**: Factual accuracy: 100% of technical claims cited with authoritative sources (Python.org, PEPs)
- **SC-008**: Cognitive load compliance: 100% of lessons stay within 5-7 concept limit (A2 tier)
- **SC-009**: Story cohesion: Narrative thread evident across all lessons (validated by human review)
- **SC-010**: Anti-pattern elimination: Zero forbidden scaffolding exposures ("Stage", "Layer", "Three Roles" as headers)

### Pedagogical Effectiveness

- **SC-011**: WHAT-WHY-HOW pattern: 100% of concepts explained conceptually BEFORE syntax shown
- **SC-012**: Progressive depth: Lessons 1-3 foundational simplicity, Lessons 4-5 controlled complexity increase
- **SC-013**: AI collaboration introduction: "Try With AI" sections present in all lessons, framed as practice (not dependency)
- **SC-014**: Teaching modality variation: Story-based learning distinguishable from Chapter 14's analogy-based approach
- **SC-015**: Student engagement: Forum feedback indicates story elements made content "less intimidating" and "more memorable"

---

## Constraints

### Audience Constraints (A2 Tier Beginners)

- **CON-001**: Assume ZERO programming experience (define all terms: "terminal", "syntax", "execute", "variable")
- **CON-002**: Cognitive load maximum: 5-7 concepts per lesson (A2 tier limit from constitution)
- **CON-003**: Scaffolding intensity: Heavy handholding required (step-by-step with validation checkpoints)
- **CON-004**: Option presentation: Max 2 options when choices given (decision paralysis risk for beginners)
- **CON-005**: Reading level: Clear, simple language (avoid jargon; when jargon necessary, define immediately with analogy)

### Content Scope Constraints

- **CON-006**: Chapter 13 covers ONLY: Python introduction, first program (print), variables, type hints, basic input, capstone
- **CON-007**: Advanced topics (bytecode, OOP preview, indentation deep dive, duck typing) MUST be optional "For Curious Learners" sections
- **CON-008**: Defer to later chapters: operators (Ch 15), strings (Ch 16), control flow (Ch 17), collections (Ch 18)
- **CON-009**: Lesson count: 5-6 core lessons + 1 capstone (total ~6 lessons, not 9—justified by beginner simplicity)
- **CON-010**: Duration: Each lesson 30-45 minutes for absolute beginners (includes reading, practice, "Try With AI")

### Pedagogical Constraints (Constitutional)

- **CON-011**: Stage 1 focus: Manual foundation building (AI collaboration gentle introduction only, not central)
- **CON-012**: Teaching modality: Story-based narrative (must differ from Chapter 14's analogy-based approach)
- **CON-013**: WHAT-WHY-HOW sequence: Always explain WHAT and WHY before syntax HOW
- **CON-014**: Specification before implementation: Show program purpose BEFORE code in examples
- **CON-015**: Internal scaffolding hidden: NO "Stage 1/2/3/4", "Layer", "Three Roles Framework" in student-facing text

### Quality Constraints

- **CON-016**: All code tested: Execution logs required for every example (no untested code published)
- **CON-017**: All claims verified: Python.org citations required for technical statements
- **CON-018**: Version specificity: Code works in Python 3.14+ (note compatibility with 3.10+)
- **CON-019**: Cross-platform: Examples work on Windows/Mac/Linux (UV from Chapter 12 ensures consistency)
- **CON-020**: Validation gates: Must pass validation-auditor (pedagogical + factual + constitutional compliance)

---

## Non-Goals (Explicitly Out of Scope)

### What We're NOT Teaching in Chapter 13

- **NG-001**: Operators (`+`, `-`, `*`, `/`, `==`, `<`, `and`, `or`) → Chapter 15: Operators, Keywords, Variables
- **NG-002**: String methods (`.split()`, `.join()`, `.upper()`, `.format()`) → Chapter 16: Strings and Type Casting
- **NG-003**: Control flow (`if`/`else`, `for` loops, `while` loops) → Chapter 17: Control Flow and Loops
- **NG-004**: Collections deep dive (lists, tuples, dicts, sets methods) → Chapter 18: Lists, Tuples, Dictionary
- **NG-005**: Functions (`def`, parameters, `return` statements) → Chapter 20: Module and Functions
- **NG-006**: File I/O (reading/writing files) → Chapter 22: IO and File Handling
- **NG-007**: Exception handling (`try`/`except`, error management) → Chapter 21: Exception Handling
- **NG-008**: Object-oriented programming (classes, inheritance, polymorphism) → Chapters 24-25: OOP Part I & II (only preview in "For Curious Learners" if contextualized)

### What We're NOT Doing (Pedagogical Boundaries)

- **NG-009**: Stage 2 AI collaboration as central methodology (Chapter 13 is Stage 1: manual foundation)
- **NG-010**: Creating skills/subagents (Stage 3 intelligence design deferred to later chapters when patterns recur)
- **NG-011**: Spec-driven projects (Stage 4 capstone approach saved for Part 5 chapters after SDD introduction)
- **NG-012**: Lecture-style teaching (avoiding: "Here are 10 facts about Python" dry presentations)
- **NG-013**: Topic taxonomy organization (avoiding: "Python History", "Python Philosophy", "Python Community" as separate disconnected lessons)

### What We're NOT Changing (Preserving Good Elements)

- **NG-014**: Chapter 12 UV setup (prerequisite remains unchanged; students have working Python environment)
- **NG-015**: Chapter 14 structure (already excellent; we reference it but don't modify)
- **NG-016**: Later chapter content (operators, strings, control flow stay in their respective chapters)

---

## Assumptions

### Student Context Assumptions

- **AS-001**: Students completed Chapter 12 (UV package manager) and have Python 3.14+ installed and working
- **AS-002**: Students can open terminal/command prompt (basic OS navigation covered in Chapter 12 or Chapter 7: Bash Essentials)
- **AS-003**: Students are motivated to learn programming (enrolled in course, committed to time investment)
- **AS-004**: Students have text editor or IDE available (Zed, VS Code, Cursor mentioned in Chapter 12)
- **AS-005**: Students have internet access for "Try With AI" sections (Claude Code, Gemini CLI, or web-based AI)

### Content Design Assumptions

- **AS-006**: Story-based learning is more engaging than lecture-style for absolute beginners (pedagogical research-backed)
- **AS-007**: WHAT-WHY-HOW sequence prevents "memorize without understanding" trap (cognitive science principle)
- **AS-008**: Progressive depth with optional advanced sections accommodates diverse learning paces (differentiation best practice)
- **AS-009**: 5-6 lessons suffice for Chapter 13 scope (beginner simplicity justifies fewer lessons than typical 7-9 chapter pattern)
- **AS-010**: "For Curious Learners" sections satisfy advanced students without overwhelming beginners (engagement + accessibility balance)

### Technical Assumptions

- **AS-011**: Python 3.14+ behavior is backwards-compatible with 3.10+ for Chapter 13 concepts (variables, print, type hints stable)
- **AS-012**: Code examples are cross-platform (print, variables work identically on Windows/Mac/Linux)
- **AS-013**: UV package manager handles environment consistency (Chapter 12 setup ensures all students have same Python version)
- **AS-014**: Official Python documentation (python.org) is authoritative and accessible (citation standard)
- **AS-015**: Terminal/command prompt is available on all student systems (validated in Chapter 12 prerequisite)

---

## Acceptance Tests (How We Know We're Done)

### Specification Phase Complete When:

- **AT-001**: All sections of spec.md filled with concrete details (no placeholders like "[TODO]" remaining)
- **AT-002**: spec-architect validation passes: spec is testable, complete, unambiguous, traceable
- **AT-003**: Maximum 3 [NEEDS CLARIFICATION] markers remain (all lower-priority ambiguities resolved with informed guesses documented in Assumptions)
- **AT-004**: User scenarios prioritized and independently testable (each P1/P2/P3 story can be validated standalone)
- **AT-005**: Success criteria measurable and technology-agnostic (no "React renders fast", only "Students complete task in under 3 minutes")

### Planning Phase Complete When:

- **AT-006**: Lesson count justified by concept density (5-6 lessons for beginner simplicity, NOT arbitrary 9-lesson template)
- **AT-007**: Pedagogical arc evident: Foundation (Lessons 1-2) → Application (Lessons 3-4) → Integration (Lesson 5-6 capstone)
- **AT-008**: Story-based modality mapped to each lesson (narrative thread connects lessons, not isolated topics)
- **AT-009**: Cognitive load distributed: Early lessons 5-6 concepts, later lessons approach 7 concept limit
- **AT-010**: "For Curious Learners" sections identified per lesson (optional advanced content planned but not required for progression)

### Implementation Phase Complete When:

- **AT-011**: All core lessons written (5-6 main lessons) with story-based narrative maintained throughout
- **AT-012**: All code examples execute successfully (test logs attached, 100% execution verification)
- **AT-013**: All technical claims cited (Python.org references present for version features, behavior, standard library)
- **AT-014**: Grep validation passes: Zero matches for "Stage [0-9]", "Layer [0-9]", "Three Roles (Framework|in Action)" in student-facing text
- **AT-015**: Section audit passes: All lessons end with "Try With AI" as ONLY final section (no "What's Next", "Key Takeaways", "Summary" after it)

### Validation Phase Complete When:

- **AT-016**: validation-auditor passes: Pedagogical effectiveness, constitutional compliance, factual accuracy, cognitive load all verified
- **AT-017**: factual-verifier passes: All Python claims verified against Python.org official docs, version compatibility confirmed
- **AT-018**: Cognitive load assessment confirms: 5-7 concepts per lesson (A2 tier limit), heavy scaffolding present
- **AT-019**: Story cohesion validated: Narrative thread evident across lessons (human review confirms story-based learning success)
- **AT-020**: Anti-pattern checks pass: No lecture-style convergence, no scaffolding exposure, no meta-commentary in student text

### User-Facing Acceptance (Post-Deployment):

- **AT-021**: Student completion rate ≥75% (baseline tracking over first 50 students)
- **AT-022**: Capstone submission quality indicates understanding: 80%+ demonstrate variables, print, type hints correctly
- **AT-023**: Forum confusion indicators LOW: Fewer than 10% of students post "I don't understand variables" type questions
- **AT-024**: Engagement feedback positive: Story elements mentioned as helpful in student surveys/feedback
- **AT-025**: Progression success: 85%+ of students who complete Chapter 13 successfully begin Chapter 14 (smooth transition)

---

## Reasoning-Activated Intelligence Context

### Constitutional Frameworks Applied

**From Constitution v6.0.0**:
- **Section IIa (4-Stage Progression)**: Chapter 13 establishes Stage 1 foundation (manual practice) before any AI collaboration (gentle Stage 2 introduction in "Try With AI")
- **Principle 1 (Specification Primacy)**: Show WHAT programs do BEFORE code syntax (story explains purpose, then code follows)
- **Principle 2 (Progressive Complexity)**: A2 tier = 5-7 concepts max, heavy scaffolding, max 2 options
- **Principle 3 (Factual Accuracy)**: All code tested (execution logs), all claims cited (Python.org)
- **Principle 6 (Anti-Convergence)**: Chapter 14 used analogy-based "Concept Before Syntax" → Chapter 13 uses story-based narrative journey (teaching modality variation)
- **Principle 7 (Minimal Content)**: Only essential concepts (defer operators, strings, control flow to later chapters per non-goals)

**From Chapter-Index.md**:
- **Part 4, Chapter 13**: First Python programming chapter for absolute beginners
- **Prerequisites**: Chapter 12 (UV package manager, Python 3.14+ installed)
- **Audience**: A2 (beginner) tier with zero programming experience
- **Next Chapter**: Chapter 14 (Data Types) already redesigned with complementary pedagogy

### User-Specified Preferences (Phase 0 Clarifications)

1. **Teaching Modality**: Story-based learning (narrative journey where Python concepts unfold like a story)
   - **Rationale**: Demystifies programming for absolute beginners, more engaging than lecture-style
   - **Implementation**: Opening hook (robot sandwich story), variable introduction (pet name labels), capstone (digital introduction card)

2. **Content Approach**: WHAT-then-WHY method (describe topic first for absolute beginners, follow "what" and then "why" before showing "how")
   - **Rationale**: Prevents "memorize syntax without understanding" trap common with beginners
   - **Implementation**: Every concept explained conceptually (WHAT it is, WHY it matters) before code syntax (HOW to write it)

3. **Scope Strategy**: Progressive depth (start simple 3-4 lessons, add "For Curious Learners" sections for bytecode/OOP concepts)
   - **Rationale**: Balanced approach accommodates different learning paces, satisfies advanced students without overwhelming beginners
   - **Implementation**: Core 5-6 lessons stay simple, optional enrichment sections for bytecode, Python internals, OOP preview

### Intelligence Accumulation (Building on Existing Work)

**From Chapter 14 Redesign (Successful Pattern)**:
- **README Structure**: Clear chapter overview, prerequisites, learning philosophy, connection to AIDD → Apply to Chapter 13
- **Concept Before Syntax**: WHAT/WHY before HOW pattern proven effective → Adapt as story-based WHAT-WHY-HOW
- **"Try With AI" Endings**: Single closing section for AI practice → Replicate in Chapter 13
- **Skills Metadata**: CEFR/Bloom's proficiency mapping (hidden from students, enables institutional integration) → Include in Chapter 13 lessons
- **Anti-Pattern Elimination**: Chapter 14 successfully avoided scaffolding exposure → Apply same grep validation to Chapter 13

**Skills Library Available**:
- `learning-objectives`: Generate measurable learning objectives aligned with Bloom's taxonomy
- `concept-scaffolding`: Structure progressive concept introduction for cognitive load management
- `ai-collaborate-teaching`: Design "Try With AI" sections for gentle AI partnership introduction
- `code-example-generator`: Create tested, verified code examples with execution logs
- `technical-clarity`: Write clear explanations for absolute beginners (define jargon, use analogies)
- `assessment-builder`: Design capstone project assessing integrated understanding

### Anti-Convergence Strategy (Avoiding Generic Patterns)

**What Chapter 13 Must NOT Become**:
- ❌ Lecture-style: "Here are 10 facts about Python" disconnected information dump
- ❌ Topic taxonomy: "Lesson 1: History, Lesson 2: Philosophy, Lesson 3: Community" isolated topics
- ❌ Syntax-first: "Here's how to write a variable" without explaining WHAT/WHY variables exist
- ❌ Scaffolding exposure: "Stage 1 Focus", "Three Roles Framework" visible to students
- ❌ Meta-commentary: "What's Next", "Congratulations", "Key Takeaways" after "Try With AI"

**What Chapter 13 WILL Be**:
- ✅ Story-driven: Cohesive narrative where programming unfolds like teaching robot to make sandwich → variables as pet name labels → capstone as digital introduction card
- ✅ WHAT-WHY-HOW sequence: Every concept explained conceptually (WHAT it is, WHY it matters) before syntax (HOW to write)
- ✅ Progressive depth: Simple foundation (Lessons 1-2) → Building complexity (Lessons 3-4) → Integration (Lesson 5-6) with optional "For Curious Learners"
- ✅ Scaffolding hidden: Internal "Stage 1" planning NOT visible; students experience pedagogy, don't study it
- ✅ Minimal endings: Each lesson closes with ONLY "Try With AI" (no additional sections)

---

## Next Steps After Spec Approval

1. **`/sp.clarify`**: Identify any remaining underspecified areas (should be minimal—max 3 clarifications)
2. **`/sp.plan`**: Structure 5-6 lesson progression with story-based narrative mapped to pedagogical arc
3. **`/sp.tasks`**: Break down lesson implementation with validation checkpoints (code testing, citation verification, grep audits)
4. **`/sp.implement`**: Write lessons using content-implementer with complete intelligence context from Phases 0-3
5. **Validation**: Run validation-auditor + factual-verifier ensuring constitutional compliance, pedagogical effectiveness, factual accuracy

---

**This specification activates reasoning about Panaversity's distinctive pedagogy for absolute beginners. It's not generic "intro to Python"—it's a story-based, WHAT-WHY-HOW journey designed specifically for A2 learners who've never programmed, with constitutional grounding and anti-convergence safeguards built in.**
