# Implementation Plan: Chapter 39 - Agent Skills & MCP Code Execution

**Branch**: `047-chapter-39-skills-mcp` | **Date**: 2025-12-26 | **Spec**: `specs/047-chapter-39-skills-mcp/spec.md`
**Input**: Feature specification from `/specs/047-chapter-39-skills-mcp/spec.md`

---

## I. Summary

**Chapter Goal**: Build Skills that Implement the Code Execution Pattern

Students learn to create skills that don't just advise—they EXECUTE. The skill orchestrates code execution (MCP calls, scripts, data processing) to autonomously solve problems. Students progress from wrapping existing MCP servers with intelligence, to writing and executing scripts autonomously, to full workflow orchestration, culminating in a shippable execution skill that encapsulates domain expertise.

**Learning Arc**:
1. **Lessons 1-2** (Layer 1 → L2): Advanced skill patterns foundation (persona, principles, composition)
2. **Lessons 3-4** (Layer 1 → L2): Skills that wrap MCP — first example of code execution pattern
3. **Lessons 5-6** (Layer 1 → L2): Skills that write + execute scripts — generalize beyond MCP
4. **Lesson 7** (Layer 3): Full workflow orchestration (MCP + scripts + iteration + error recovery)
5. **Lesson 8** (Layer 4): Capstone — shippable execution skill integrating all patterns

**Why This Matters**: Chapter 39 operationalizes the code execution pattern. Students move from "skills that advise" (Chapter 5, Part 5) to "skills that execute." This is the difference between a documentation agent and a production automation system.

---

## II. Technical Context

**Language/Version**: Python 3.11+, Bash
**Primary Dependencies**: Claude SDK, Anthropic Models, MCP clients, FastMCP
**Storage**: In-memory state for skill demonstrations, persistent logging for error recovery
**Testing**: pytest for skill validation, MCP server testing patterns
**Target Platform**: CLI-first (students developing locally), cloud-deployable skills
**Project Type**: Educational content (lessons) + reference implementations (skills)
**Performance Goals**: Skills execute within 30-60 second timeframes; MCP calls optimized for token efficiency (build on Chapter 37-38 foundation)
**Constraints**: Skills must respect API rate limits, implement error recovery, and handle partial execution state
**Scale/Scope**: 8 lessons, each producing 1-2 standalone code examples, cumulative toward final shippable skill

**Educational Content Type**: Technical/Hybrid (builds on MCP foundations from Chapter 37-38, applies to script execution generalization)

**B2 Proficiency Context** (from chapter-index.md Part 6): Students are intermediate developers building Custom Agents. They understand Python fundamentals (Part 5), MCP architecture (Chapter 37), and MCP server development (Chapter 38). They're ready for production patterns that combine multiple SDKs.

---

## III. Constitution Check

**Governing Principles** (from Constitution v7.0.0):
- **Specification Primacy**: Each skill example must have explicit specification before implementation
- **Progressive Complexity**: B2 tier (7-10 concepts per lesson), build from manual to AI-assisted to reusable intelligence
- **AI Collaboration (Three Roles)**: Layer 2 lessons must demonstrate AI as Teacher (suggests execution patterns), Student (adapts to constraints), Co-Worker (iterates toward robust solution)
- **Intelligence Accumulation**: Skills created in this chapter reuse patterns from Chapters 5 (basic skills), 37 (MCP fundamentals), 38 (MCP servers)
- **Anti-Convergence Variation**: Chapter 38 used lecture-based MCP server development. Chapter 39 uses specification-first + hands-on skill orchestration (different modality)
- **Vibe Coding Prevention**: All code execution examples must follow spec-first approach (write specification of what skill should do BEFORE implementation)
- **Minimal Sufficient Content**: Only teach execution patterns that recur 2+ times across chapters (code execution is foundational; MCP wrapping generalizes to script execution)

**Layer 4 (Capstone) Requirement**: Lesson 8 must produce a deployable/sellable Digital FTE skill (execution skill addressing real domain problem)

**Cognitive Load Validation**:
- Lessons 1-2: 7-8 concepts (advanced skill patterns, persona design, principles)
- Lessons 3-4: 8-9 concepts (MCP execution pattern, result filtering, error handling)
- Lessons 5-6: 8-9 concepts (script generation, write-execute-analyze loop, recovery)
- Lesson 7: 9-10 concepts (orchestration, convergence criteria, state management)
- Lesson 8: 10+ (capstone integration, no limit for B2 capstone complexity)

**Validation**: ✅ All B2 proficiency requirements satisfied. ✅ Layer progression enforced. ✅ Anti-convergence addressed (specification-first modality differs from Chapter 38). ✅ Three Roles integration required in Layers 2, 3, 4.

---

## IV. Lesson-by-Lesson Breakdown

### Lesson 1: Advanced Skill Patterns — Persona, Principles, Composition (Layer 1: Manual Foundation)

**Pedagogical Layer**: L1 (Manual Foundation)

**Teaching Modality**: Direct teaching with comparative analysis

**Learning Objectives** (B2 proficiency):
- Identify core components of execution skills: persona (expert identity), questions (decision logic), principles (domain constraints)
- Design skill personas that activate domain expertise
- Write decision-making questions that encode autonomous behavior
- Articulate principles that guide execution skill behavior

**Bloom's Level**: Understand + Apply

**Key Concepts** (count: 8 — within B2 limit of 7-10):
1. Skill persona definition (expert identity in code execution domain)
2. Persona patterns for execution (vs advisory) skills
3. Questions framework for autonomous decision-making
4. Principles for constraining behavior (safety, efficiency, convergence)
5. Skill composition (dependencies between skills)
6. Skill references (how skills depend on other skills)
7. Reusability criteria for skill patterns
8. Layer 3 vs Layer 4 skill design decisions

**Content Structure**:
- **Opening Context**: Why advanced skill patterns matter for code execution (manual foundation)
- **Persona for Execution**: Show contrast between advisory skill persona ("You are a Python expert") and execution skill persona ("You are a Python execution orchestrator: see code → estimate complexity → execute → analyze → iterate")
- **Principles for Safe Execution**: Demonstrate explicit principles constraining behavior (e.g., "Fail safe: Always validate before execution", "Converge deliberately: Iterate 3 times max before declaring done")
- **Skill Composition Patterns**: Teach how to reference/depend on other skills (skill X wraps MCP server Y; skill Z depends on skill X)
- **Manual Exercise**: Students design a skill persona for their own domain, write 5 questions, articulate 3 principles

**Teaching Pattern**: Specification-driven (show spec of desired skill FIRST, then analyze how persona/questions/principles implement it)

**Success Criteria**:
- Students can articulate why execution skill persona differs from advisory skill persona
- Students write testable questions that encode decision logic (not vague guidance)
- Students articulate principles that enable autonomous operation
- (Manual validation checkpoint before proceeding to Layer 2)

**Estimated Time**: 25 minutes

**Intelligence Creation Opportunity**: Reference document "Execution Skill Persona Patterns" (reusable framework for designing execution skills across any domain)

---

### Lesson 2: Skill Composition & Dependencies (Layer 2: AI Collaboration)

**Pedagogical Layer**: L2 (AI Collaboration with Three Roles)

**Teaching Modality**: Specification-first + collaborative refinement

**Learning Objectives** (B2 proficiency):
- Design skill dependency graphs that avoid circular dependencies
- Compose multiple skills into coordinated execution workflows
- Implement skill-to-skill communication patterns
- Test skill composition for integration issues

**Bloom's Level**: Apply + Analyze

**Key Concepts** (count: 8 — within B2 limit):
1. Skill dependency declaration (spec.md references)
2. Composition patterns (sequential, parallel, conditional)
3. Data flow between skills (output of skill A → input of skill B)
4. Error propagation in composed skills
5. Testing composed skills (mocking dependencies)
6. Circular dependency detection
7. Skill versioning for compatibility
8. Skill composition within capstone projects

**Content Structure**:
- **Review Layer 1**: Recap skill personas and principles from Lesson 1
- **Spec-First Example**: Show spec of "data-processing-pipeline-skill" that composes 3 sub-skills (data-fetch-skill, transform-skill, validate-skill)
- **Three Roles Demonstration**:
  - **AI as Teacher**: "Here's a pattern: dependency declaration in YAML. This prevents circular dependencies. Also enables others to understand what this skill needs."
  - **AI as Student**: "Your first attempt hardcoded skill references. I adapted it to accept skill references as parameters, making it more flexible."
  - **AI as Co-Worker**: Student and AI iteratively refine error handling between composed skills (Iteration 1: silent failure if sub-skill fails → Iteration 2: error logging → Iteration 3: rollback on partial failure → converge on strategy matching domain requirements)
- **Hands-On Exercise**: Students compose 2-3 existing skills (from earlier chapters/projects), test integration, debug data flow

**Teaching Pattern**: Collaborative debugging of skill composition issues (AI suggests patterns, student validates against requirements, iterate)

**Three Roles Validation** (REQUIRED for Layer 2):
- ✓ AI teaches composition patterns (e.g., dependency declaration YAML syntax)
- ✓ Student teaches AI domain constraints (e.g., "We can't fail mid-process without rollback")
- ✓ Convergence through iteration (error handling strategy emerges through collaboration)

**Success Criteria**:
- Students can design dependency graphs without cycles
- Students compose skills with testable data flow
- Students implement error propagation strategies
- (Integration checkpoint: Students run composed skill successfully)

**Estimated Time**: 30 minutes

**Intelligence Creation Opportunity**: "Skill Composition Testing Framework" (reusable patterns for validating skill interactions across any domain)

---

### Lesson 3: Anatomy of MCP-Wrapping Skills (Layer 1: Manual Foundation)

**Pedagogical Layer**: L1 (Manual Foundation)

**Teaching Modality**: Specification-driven deconstruction (analyze existing skills to understand pattern)

**Learning Objectives** (B2 proficiency):
- Analyze existing MCP-wrapping skills (like `fetching-library-docs`)
- Identify the intelligence layer (when/how to wrap MCP vs when to invoke directly)
- Understand result filtering for token efficiency
- Recognize error recovery patterns

**Bloom's Level**: Analyze

**Key Concepts** (count: 9 — within B2 limit of 7-10):
1. MCP server vs skill wrapper distinction
2. Intelligence layer (decision logic added by skill)
3. When to trigger skill (persona conditions)
4. Result filtering (token efficiency pattern)
5. Error recovery from MCP failures
6. MCP call batching/optimization
7. Skill-specific MCP client configuration
8. Fallback strategies when MCP unavailable
9. Example: `fetching-library-docs` pattern (Context7 MCP wrapping)

**Content Structure**:
- **Opening**: Why wrap MCP in skills? (MCP is "dumb tool"; skill adds "when/how/why" intelligence)
- **Deconstruction 1**: Walk through `fetching-library-docs` skill
  - Show spec first (what this skill should do)
  - Analyze persona (when does this skill activate? what decisions does it make?)
  - Analyze questions (how does skill filter documentation?)
  - Analyze principles (what makes result filtering effective?)
  - Show implementation (MCP call + filtering logic)
  - Highlight result reduction (934 tokens → 205 tokens via filtering)
- **Deconstruction 2**: Walk through `browsing-with-playwright` skill (similar pattern, different MCP server)
- **Pattern Recognition**: Show that both skills follow same structure (MCP wrapper + intelligence layer + result filtering)
- **Manual Exercise**: Students design (on paper, no coding) a skill that wraps an MCP server from Chapter 38, specifying persona, questions, principles

**Teaching Pattern**: Specification-driven analysis (show spec of existing skill, reverse-engineer pattern, understand why it works)

**Success Criteria**:
- Students can articulate the intelligence layer beyond MCP
- Students identify filtering/optimization patterns in existing skills
- Students recognize error recovery strategies
- Students design new MCP-wrapping skill (spec-only, no code yet)

**Estimated Time**: 30 minutes

**Intelligence Creation Opportunity**: None (analysis lesson, no new artifact created—but sets foundation for Lesson 4)

---

### Lesson 4: Build Your MCP-Wrapping Skill (Layer 2: AI Collaboration)

**Pedagogical Layer**: L2 (AI Collaboration with Three Roles)

**Teaching Modality**: Specification-first collaborative implementation

**Learning Objectives** (B2 proficiency):
- Write spec for MCP-wrapping skill (what MCP to wrap, what intelligence to add)
- Implement skill with MCP client integration
- Test skill with real MCP server (from Chapter 38)
- Validate result filtering effectiveness

**Bloom's Level**: Apply + Create

**Key Concepts** (count: 9 — within B2 limit):
1. Spec.md for skill (intent, MCP server reference, filtering logic)
2. Persona implementation (trigger conditions)
3. Questions as filtering criteria
4. MCP client initialization
5. Tool invocation patterns
6. Result filtering logic (token counting, relevance scoring)
7. Error handling for MCP failures
8. Skill testing patterns
9. Documentation for skill consumers

**Content Structure**:
- **Step 1: Specification (spec-FIRST)**
  - Students write spec.md for a skill that wraps an MCP server (e.g., "Python-API-Documentation-Fetcher" wrapping Context7)
  - Spec must include: intent, MCP server reference, filtering criteria, success criteria
- **Step 2: Persona & Questions Design**
  - Students design persona: "You are an API documentation specialist: decide which APIs are relevant to the user's question, fetch them via MCP, filter for examples and signatures only"
  - Students write questions: "What's the user's primary goal? What's their experience level? Which API modules are relevant?"
- **Step 3: Three Roles Collaboration**
  - **AI as Teacher**: "Here's the pattern for MCP client initialization. Also, here's how to count tokens before/after filtering to prove efficiency."
  - **AI as Student**: "Your first version filters too aggressively. Let me suggest returning more context but with better scoring so user can select relevance level."
  - **AI as Co-Worker**: Iterate on filtering heuristics (AI suggests approach → student tests with sample queries → AI refines scoring → converge on balanced filtering)
- **Step 4: Implementation & Testing**
  - Students implement skill with AI collaboration
  - Test against real MCP server from Chapter 38
  - Validate token efficiency (before/after metrics)
- **Step 5: Documentation**
  - Students write skill usage guide (when to activate, what parameters, expected results)

**Teaching Pattern**: Specification-first implementation with collaborative refinement on filtering logic (where efficiency vs accuracy tradeoff exists)

**Three Roles Validation** (REQUIRED for Layer 2):
- ✓ AI teaches MCP client patterns and token counting
- ✓ Student teaches AI domain-specific filtering priorities (what documentation matters)
- ✓ Convergence through iteration (filtering heuristics improved through collaboration)

**Success Criteria**:
- Skill spec is complete and clear
- Skill integrates MCP server successfully
- Result filtering demonstrates token efficiency (30%+ reduction)
- Skill tested with multiple queries from different domains
- Documentation enables other developers to use skill
- (Integration checkpoint: Skill runs end-to-end without errors)

**Estimated Time**: 45 minutes

**Intelligence Creation Opportunity**: "MCP-Wrapping Skill Template" (reusable pattern for wrapping any MCP server with intelligence layer; students will apply this template repeatedly in future chapters)

---

### Lesson 5: Script Execution Fundamentals — Write-Execute-Analyze Loop (Layer 1: Manual Foundation)

**Pedagogical Layer**: L1 (Manual Foundation)

**Teaching Modality**: Specification-driven walkthrough of iterative execution pattern

**Learning Objectives** (B2 proficiency):
- Understand write-execute-analyze loop as iterative pattern
- Recognize when script execution is appropriate (vs MCP)
- Identify error types (syntax vs runtime) and recovery strategies
- Design skill specification for script execution

**Bloom's Level**: Understand + Apply

**Key Concepts** (count: 9 — within B2 limit of 7-10):
1. Write-execute-analyze loop (iterative pattern)
2. Syntax error detection and fixing
3. Runtime error handling
4. Timeout and resource constraints
5. Partial execution state (resuming after failure)
6. Safety constraints (what scripts should/shouldn't do)
7. Output parsing and validation
8. Convergence criteria (when to stop iterating)
9. Comparison to MCP approach (execute vs invoke)

**Content Structure**:
- **Opening**: Why script execution? (Generalization beyond MCP: handle any computational task)
- **Manual Walkthrough 1**: Show spec and step-by-step manual execution of "data-analysis-skill"
  1. Spec: Process CSV, identify patterns, generate summary
  2. Write: Generate Python script from spec
  3. Execute: Run script, get syntax error (misspelled variable)
  4. Analyze: Error message says "NameError: undefined variable"
  5. Fix: Correct code, retry
  6. Execute: Script runs, generates partial output
  7. Analyze: Missing some edge cases (empty rows not handled)
  8. Fix: Add error handling
  9. Execute: Complete successful output
  10. Conclude: This is write-execute-analyze loop
- **Manual Walkthrough 2**: Show how AI would orchestrate same loop (AI does writing/analyzing, student does validating)
- **Safety Considerations**: What scripts should/shouldn't do (filesystem access constraints, API rate limits, resource limits)
- **Convergence Decision**: When to stop iterating (success metric reached, timeout, iteration limit)
- **Manual Exercise**: Students manually walk through 2-3 script execution iterations (no coding, design-focused)

**Teaching Pattern**: Specification-driven step-by-step walkthrough (observe pattern before generalizing)

**Success Criteria**:
- Students can articulate write-execute-analyze loop phases
- Students distinguish syntax vs runtime errors (and recovery strategies)
- Students identify safety constraints for their domain
- Students design convergence criteria for iteration
- (Foundation checkpoint: students understand loop pattern before implementing)

**Estimated Time**: 30 minutes

**Intelligence Creation Opportunity**: None (foundation lesson)

---

### Lesson 6: Build Script-Execution Skill (Layer 2: AI Collaboration)

**Pedagogical Layer**: L2 (AI Collaboration with Three Roles)

**Teaching Modality**: Specification-first collaborative skill building (write-execute-analyze loop implementation)

**Learning Objectives** (B2 proficiency):
- Write skill spec for script execution (data processing, analysis, transformation)
- Implement write-execute-analyze loop with error recovery
- Iterate toward robust solution using AI collaboration
- Test skill with diverse input scenarios

**Bloom's Level**: Apply + Create

**Key Concepts** (count: 9 — within B2 limit of 7-10):
1. Skill spec for script execution (intent, success criteria, constraints)
2. Script generation prompting (spec → code prompt)
3. Execution environment setup (sandboxing, timeouts)
4. Error detection and parsing
5. Iterative fix generation (AI writes fixed code)
6. Iteration limit and convergence detection
7. Output validation and parsing
8. State management (resuming after failure)
9. Logging and observability

**Content Structure**:
- **Step 1: Specification (spec-FIRST)**
  - Students write spec for "csv-analysis-skill" or "data-transformation-skill"
  - Spec must include: data inputs, transformations, success validation, edge cases
- **Step 2: Persona & Questions**
  - Persona: "You are a data orchestrator: write Python scripts that process data, validate outputs, and handle edge cases"
  - Questions: "What's the data structure? What transformations matter? What edge cases exist?"
- **Step 3: Three Roles Collaboration**
  - **AI as Teacher**: "Here's the pattern for writing scripts that validate outputs. Also, here's how to parse Python error messages to fix code automatically."
  - **AI as Student**: "Your first version assumes well-formed data. Let me suggest adding validation for edge cases (empty inputs, malformed rows, etc.)."
  - **AI as Co-Worker**: Iterate on error recovery strategy (AI writes code → student checks error message → AI fixes → student validates output → converge on robust solution that handles edge cases)
- **Step 4: Implementation & Testing**
  - Students implement skill with AI collaboration on write-execute-analyze loop
  - Test with clean data, malformed data, edge cases
  - Validate iteration/convergence behavior (system stops iterating when criteria met)
- **Step 5: Error Scenarios**
  - Students test with intentional errors: syntax errors, runtime errors, timeout scenarios
  - Observe and validate error recovery behavior

**Teaching Pattern**: Collaborative refinement of error recovery (AI suggests patterns, student validates against real data, iterate toward robust solution)

**Three Roles Validation** (REQUIRED for Layer 2):
- ✓ AI teaches error detection/parsing patterns and recovery strategies
- ✓ Student teaches AI data validation requirements and edge cases
- ✓ Convergence through iteration (error handling strategy emerges through testing multiple scenarios)

**Success Criteria**:
- Skill spec is complete and realistic
- Skill implements write-execute-analyze loop successfully
- Error recovery handles syntax, runtime, and timeout errors
- Skill tested with clean data, malformed data, and edge cases
- Iteration stops when success criteria met (no infinite loops)
- Output validation passes for diverse inputs
- (Integration checkpoint: Skill produces correct results across 5+ test scenarios)

**Estimated Time**: 50 minutes

**Intelligence Creation Opportunity**: "Script-Execution-Skill Template" (reusable pattern for any computational task; students will use this for data processing, analysis, transformation across future projects)

---

### Lesson 7: Full Workflow Orchestration — MCP + Scripts + Error Recovery (Layer 3: Intelligence Design)

**Pedagogical Layer**: L3 (Intelligence Design)

**Teaching Modality**: Skill composition with orchestration design

**Learning Objectives** (B2/C1 proficiency):
- Compose MCP-wrapping skills and script-execution skills into orchestrated workflow
- Design error recovery and state management for complex workflows
- Implement convergence criteria for multi-step processes
- Test workflow for robustness and efficiency

**Bloom's Level**: Analyze + Create

**Key Concepts** (count: 9 — within B2 upper range):
1. Workflow orchestration patterns (sequential, parallel, conditional)
2. State management for multi-step processes
3. Error propagation and recovery strategies
4. Convergence criteria across multiple steps
5. Logging and observability for debugging
6. Performance optimization (parallelization, caching)
7. Fallback and retry logic
8. Workflow specification (DAG-like definition)
9. Testing orchestrated workflows

**Content Structure**:
- **Step 1: Workflow Specification**
  - Students design workflow combining skills from Lessons 4 & 6 (e.g., "fetch-and-analyze-workflow" = MCP-fetch-skill → script-analysis-skill)
  - Spec defines step sequence, data flow, error recovery, convergence criteria
- **Step 2: Skill Composition**
  - Students compose MCP-wrapping skill from Lesson 4 with script-execution skill from Lesson 6
  - Design data contract between skills (output of skill A → input of skill B)
- **Step 3: Error Recovery Design**
  - Design error handling: if skill A fails, use fallback → if skill B fails, retry → if both fail, escalate
  - Implement state checkpoints (resume from step 2 if step 3 fails)
- **Step 4: Convergence Strategy**
  - Define when workflow succeeds (all steps complete, outputs valid)
  - Define when workflow fails (non-recoverable error, timeout)
  - Implement iteration/refinement loop within workflow (if results are partial, retry with adjusted parameters)
- **Step 5: Implementation & Testing**
  - Students implement orchestrated workflow as reusable skill
  - Test happy path (all steps succeed)
  - Test error paths (partial failures, timeouts, retries)
  - Validate state management (resumption after failure)
- **Step 6: Performance Analysis**
  - Measure execution time, token efficiency, error rates
  - Identify optimization opportunities

**Teaching Pattern**: Design-first (specification before implementation), then collaborative refinement of orchestration logic

**Layer 3 Validation** (REQUIRED for Layer 3):
- Reusable artifact created: Orchestrated workflow skill encapsulating MCP + script execution patterns
- Persona/questions/principles define autonomous behavior
- Skill composition demonstrates pattern recognized 2+ times (MCP wrapping + script execution combined)
- Artifact is generalizable (could apply to other domains with different skills/MCP servers)

**Success Criteria**:
- Workflow spec is complete (step sequence, data flow, error handling)
- Skill composition validated (data flow tests pass)
- Error recovery tested across multiple failure scenarios
- State management allows workflow resumption
- Workflow tested end-to-end with diverse inputs
- (Reusability checkpoint: Skill documented for reuse in other projects)

**Estimated Time**: 50 minutes

**Intelligence Creation Opportunity**: "Orchestration Skill" (reusable pattern for combining MCP + script execution; this becomes a component in Chapter 40 and beyond)

---

### Lesson 8: Capstone — Shippable Agent Skill (Layer 4: Spec-Driven Integration)

**Pedagogical Layer**: L4 (Spec-Driven Integration / Digital FTE Production)

**Teaching Modality**: Specification-first capstone project (compose all accumulated intelligence)

**Learning Objectives** (B2/C1 proficiency):
- Write specification for domain-specific execution skill
- Compose skills from Lessons 1-7 into integrated Digital FTE
- Validate skill against specification (acceptance tests)
- Package skill for deployment/sales

**Bloom's Level**: Create + Evaluate

**Key Concepts** (count: 10+ — no limit for Layer 4 capstone complexity):
1. Domain specification for execution skill (customer-facing intent)
2. Skill composition architecture (which skills to include)
3. User interface design (how customers invoke skill)
4. Safety and governance constraints
5. Acceptance testing and validation
6. Performance and cost metrics
7. Documentation for skill consumers/users
8. Deployment packaging and versioning
9. Monitoring and observability in production
10. Monetization strategy (subscription, success-fee, license)
11. Competitive differentiation (why this skill is better)

**Content Structure**:

**Phase 1: Domain-Specific Specification (Spec FIRST)**
- Students select domain or use provided template (e.g., "legal-document-analyzer-skill", "financial-data-processor-skill", "code-review-automation-skill")
- Write spec.md covering:
  - **Intent**: What problem does this Digital FTE solve? Why does it matter to customers?
  - **Success Criteria**: How do we measure if skill works? (accuracy, speed, cost)
  - **Constraints**: What's explicitly NOT included? (non-goals)
  - **Acceptance Tests**: Concrete test cases proving spec satisfaction
  - **Architecture**: Which skills from Lessons 1-7 compose into this system?

**Phase 2: Skill Composition**
- Students compose skills from Lessons 4 (MCP-wrapping), 6 (script-execution), 7 (orchestration)
- Design execution flow: fetch data (MCP) → analyze/transform (scripts) → validate/refine (orchestration)
- Integrate error recovery from Lesson 7

**Phase 3: Specification → Implementation Orchestration**
- Use spec.md to drive implementation without additional guidance
- AI implements composed skill based on spec ALONE
- No back-and-forth—if spec is insufficient, identify gaps and refine spec (don't communicate implementation needs)

**Phase 4: Validation Against Specification**
- Write acceptance tests based on success criteria from spec
- Validate skill produces results meeting spec requirements
- Measure performance against success metrics (accuracy, speed, cost)
- Document any spec gaps discovered during validation

**Phase 5: Production Packaging**
- Document skill for consumers (usage guide, API, examples)
- Package skill as deployable artifact
- Create installation/setup documentation
- Version skill appropriately (major.minor.patch based on changes)

**Phase 6: Digital FTE Positioning**
- Articulate business value (what customer problem does this solve?)
- Identify customer segments (who would buy/use this?)
- Design pricing model (subscription, success-fee, license)
- Plan go-to-market strategy

**Teaching Pattern**: Specification-first capstone (write spec, then orchestrate composition to implement spec, then validate)

**Layer 4 Validation** (REQUIRED for Layer 4):
- ✓ Specification written FIRST (before any implementation)
- ✓ Accumulated intelligence from Lessons 1-7 composed (not reinvented)
- ✓ Implementation driven by specification alone
- ✓ Validation proves spec ↔ implementation alignment
- ✓ Output is deployable Digital FTE (production-ready skill)

**Success Criteria**:
- Specification is clear, complete, and measurable
- All acceptance tests pass (spec requirements satisfied)
- Skill integrates 3+ components from earlier lessons
- Error recovery handles real-world failure scenarios
- Documentation enables external users to adopt skill
- Skill is deployable/shareable as standalone artifact
- Business case articulated (why customers would buy this)
- (Capstone checkpoint: Skill ready for sale/deployment as Digital FTE)

**Estimated Time**: 90 minutes (extended for complexity)

**Intelligence Creation Opportunity**: Final artifact is shippable execution skill addressing real domain problem—this is the culmination of Chapter 39, representing students' ability to encode expertise as sellable Digital FTE

---

## V. Skill Dependencies & Cross-Chapter Integration

### Skill Dependencies Within Chapter

```
Lesson 1: Persona/Principles Patterns (Layer 1)
  ↓ enables ↓
Lesson 2: Skill Composition (Layer 2)
  ↓ requires ↓
Lesson 3: MCP-Wrapping Analysis (Layer 1) → Lesson 4: MCP-Wrapping Implementation (Layer 2)
Lesson 5: Script Execution Fundamentals (Layer 1) → Lesson 6: Script Execution Skill (Layer 2)
  ↓ both enable ↓
Lesson 7: Orchestration (Layer 3)
  ↓ enables ↓
Lesson 8: Capstone (Layer 4)
```

### Cross-Chapter Dependencies

**Prerequisite Chapters** (must be completed before Chapter 39):
- **Chapter 5** (Basic Skills): Students understand SKILL.md structure, persona/questions/principles pattern, skill composition basics
- **Chapter 37** (MCP Fundamentals): Students understand MCP architecture, clients, tool invocation patterns
- **Chapter 38** (MCP Server Development): Students have implemented MCP servers they can wrap in Chapter 39 skills
- **Part 5** (Python Fundamentals): Students have Python skills for script writing/execution

**Validation**: ✅ All prerequisites are implemented and available. Chapter 39 builds directly on established patterns from Chapters 5, 37-38.

### Skill Artifacts Created in Chapter 39

| Lesson | Skill Name | Type | Reuse Pattern |
|--------|-----------|------|---------------|
| 1 | Execution Skill Persona Patterns | Reference Document | Framework for designing execution skills |
| 2 | Skill Composition Testing Framework | Reference Document | Testing patterns for skill interactions |
| 4 | MCP-Wrapping Skill Template | Reusable Skill | Template for wrapping any MCP server |
| 6 | Script-Execution-Skill Template | Reusable Skill | Template for any computational task execution |
| 7 | Orchestration Skill | Reusable Skill | Pattern for combining MCP + scripts |
| 8 | Domain-Specific Execution Skill | Digital FTE | Shippable product (varies by student) |

**Intelligence Accumulation**: Skills from Chapter 39 will be referenced and composed in Chapter 40 (FastAPI for Agents), Chapters 43+ (Agent evaluation, observability), and Part 7 (Deployment)

---

## VI. Assessment Strategy

### Formative Assessments (During Lessons)

**Lesson 1-2**:
- Can students design skill persona that differs from advisory skills?
- Can students write testable decision-making questions?

**Lesson 3-4**:
- Can students analyze MCP-wrapping skill and identify intelligence layer?
- Does student's implemented skill reduce token count by 30%+?

**Lesson 5-6**:
- Can students manually walk through write-execute-analyze loop?
- Does student's skill handle syntax + runtime errors correctly?

**Lesson 7**:
- Can students design workflow composition without circular dependencies?
- Does orchestrated skill handle multiple failure scenarios?

**Lesson 8**:
- Is capstone specification complete and measurable?
- Do all acceptance tests pass?
- Is skill documented and deployable?

### Summative Assessment (End of Chapter)

**Capstone Project Evaluation** (Lesson 8):
- Specification completeness (5 points)
- Skill composition integration (5 points)
- Error recovery robustness (5 points)
- Documentation quality (5 points)
- Business case articulation (5 points)
- **Total**: 25 points (60%+ = pass, 90%+ = mastery)

**Success Evals** (from spec.md):
- **SC-001**: Students analyze existing MCP-wrapping skill and explain pattern
- **SC-002**: Students build skill wrapping MCP server with proper triggering + filtering
- **SC-003**: Students build skill that writes/executes/iterates on Python scripts
- **SC-004**: Students implement error recovery in execution skills
- **SC-005**: Students complete capstone (shippable skill implementing full code execution pattern)
- **SC-006**: 90% of exercises completed using AI collaboration
- **SC-007**: Chapter maintains B2 proficiency level

**All evals must be met for chapter completion.**

---

## VII. Constitution & Quality Alignment

### Vibe Coding Prevention

**Pattern Enforced**: Every skill example must have explicit specification before implementation

**Validation Strategy**:
1. Lesson content shows spec.md FIRST
2. Shows how specification drives implementation decisions
3. Demonstrates specification validation (testing against spec)
4. Explicitly teaches that vague specs lead to uncertain execution

**Examples**:
- Lesson 3: Deconstruct skill starting with spec.md (what it should do)
- Lesson 4: Write spec before touching code
- Lesson 8: Capstone starts with specification-first requirement

### Three Roles Integration (Layer 2 & 4)

**Lessons requiring Three Roles** (Lessons 2, 4, 6, 8):
- AI teaches execution patterns (MCP integration, error recovery, script generation)
- Student teaches AI domain requirements (data validation needs, safety constraints, performance goals)
- Convergence through iteration (error handling, filtering heuristics, orchestration logic emerge through collaboration)

**Validation**: Each Layer 2/4 lesson must document all three roles with concrete examples (no meta-commentary exposed to students)

### Anti-Convergence Variation

**Chapter 38 Teaching Modality**: Lecture-style MCP server development (explain → implement)

**Chapter 39 Teaching Modality**: Specification-first skill orchestration (spec → analyze → compose → validate)

**Variation Within Chapter**:
- Lessons 1-2: Pattern analysis (deconstruction, design)
- Lessons 3-4: Specification-driven (write spec first)
- Lessons 5-6: Collaborative refinement (iterate on error recovery)
- Lesson 7: Architectural composition (multi-skill orchestration)
- Lesson 8: Capstone specification-first (full cycle)

**No repetition of teaching patterns—each lesson modality varies to maintain engagement and activate different reasoning types**

### Cognitive Load Compliance

**B2 Proficiency (7-10 concepts per lesson)**:
- Lesson 1: 8 concepts (persona, questions, principles, composition, references, reusability, design decisions) ✓
- Lesson 2: 8 concepts (dependencies, composition patterns, data flow, error propagation, testing, versioning) ✓
- Lesson 3: 9 concepts (wrapper distinction, intelligence layer, triggering, filtering, recovery, optimization, configuration, fallbacks) ✓
- Lesson 4: 9 concepts (spec, persona, questions, client init, invocation, filtering, error handling, testing, documentation) ✓
- Lesson 5: 9 concepts (loop phases, error types, recovery, constraints, partial state, safety, parsing, convergence, comparison) ✓
- Lesson 6: 9 concepts (spec, prompting, environment setup, error detection, iteration, limits, validation, state, logging) ✓
- Lesson 7: 9 concepts (orchestration patterns, state, error propagation, convergence, logging, optimization, fallback, specification, testing) ✓
- Lesson 8: 10+ concepts (specification, composition, testing, safety, observability, metrics, documentation, packaging, monitoring, monetization, differentiation) ✓

**All lessons within B2 limits or justified (capstone has no limit)**

---

## VIII. Teaching Modality Variation Summary

| Lesson | Modality | Why This Pattern |
|--------|----------|------------------|
| 1 | Direct teaching with comparative analysis | Establish pattern distinction (execution vs advisory) |
| 2 | Specification-first + collaborative refinement | Introduce composition with AI collaboration on integration issues |
| 3 | Specification-driven deconstruction | Reverse-engineer existing skills to understand pattern |
| 4 | Specification-first collaborative implementation | Build skills with specification driving implementation |
| 5 | Specification-driven step-by-step walkthrough | Understand iterative pattern before implementing |
| 6 | Collaborative refinement of error recovery | Iterate with AI on robustness across scenarios |
| 7 | Design-first orchestration composition | Architect multi-skill workflows before implementation |
| 8 | Specification-first capstone (full cycle) | Demonstrate complete specification-driven development |

**Anti-Convergence Check**: Each lesson varies modality. No two consecutive lessons use identical teaching patterns. ✓

---

## IX. Quality Benchmarking

**Reference Lesson Standard**: Chapter 1, Lesson 1 (`01-the-2025-inflection-point.md`)

**Quality Requirements Applied to Chapter 39**:
1. **Full YAML Frontmatter**: Skills metadata, learning objectives, cognitive load, differentiation ✓
2. **Compelling Narrative Opening**: Connect skill execution to Digital FTE production ✓
3. **Deep Evidence Throughout**: Architecture diagrams for orchestration, code examples with specifications ✓
4. **Three "Try With AI" Sections**: Each targets different skill (MCP wrapping, script execution, orchestration) ✓
5. **Fact-Checked Content**: All MCP patterns verified against Chapter 37-38, Python patterns verified ✓
6. **No Meta-Commentary**: Three Roles framework invisible to students (experience through action, not through framework labels) ✓

---

## X. Success Metrics & Validation Criteria

**Chapter Succeeds When**:

✅ **Pedagogical Structure**: 8 lessons follow Foundation → Application → Integration → Mastery progression
✅ **Layer Progression**: L1 → L2 → L3 → L4 enforced with no skipping
✅ **Three Roles Integration**: Lessons 2, 4, 6, 8 demonstrate bidirectional learning (no meta-commentary)
✅ **Vibe Coding Prevention**: All skills show specification before implementation
✅ **Cognitive Load**: B2 proficiency limits respected (7-10 concepts per lesson, 10+ only in capstone)
✅ **Skill Artifacts**: Chapter produces 6 reusable components (templates, frameworks, orchestration skill)
✅ **Capstone Digital FTE**: Lesson 8 produces deployable execution skill addressing real domain problem
✅ **Anti-Convergence**: Teaching modalities vary across lessons (no repetition)
✅ **Assessment Coverage**: All success evals from spec addressed by lessons
✅ **Cross-Chapter Integration**: Skills created enable future chapters (FastAPI, evals, deployment)

**Chapter Fails When**:
- Any lesson shows code without specification
- Three Roles framework exposed in student-facing content
- Cognitive load exceeds B2 limits without justification
- Capstone produces incomplete/non-deployable skill
- Skills don't demonstrate patterns that recur in future chapters

---

## XI. Estimated Implementation Timeline

**Total Duration**: ~8-10 hours for all 8 lessons + capstone

| Phase | Duration | Deliverable |
|-------|----------|-------------|
| Planning (this document) | 1 hour | Implementation plan ✓ |
| Lesson 1 | 1 hour | Persona/principles patterns lesson |
| Lesson 2 | 1.25 hours | Skill composition lesson with Three Roles |
| Lesson 3 | 1 hour | MCP-wrapping analysis lesson |
| Lesson 4 | 1.5 hours | MCP-wrapping implementation with Three Roles |
| Lesson 5 | 1 hour | Script execution fundamentals lesson |
| Lesson 6 | 1.5 hours | Script execution skill with Three Roles |
| Lesson 7 | 1.5 hours | Orchestration skill (Layer 3) |
| Lesson 8 | 2 hours | Capstone specification-first project (Layer 4) |
| Review & Validation | 1 hour | Quality assurance, constitutional alignment |
| **Total** | **~12 hours** | Complete chapter ready for publication |

---

## XII. Next Steps (For Implementation Agents)

1. **Invoke content-implementer for Lesson 1**: Use this plan + spec.md as input
2. **Reference existing skills**: Point content-implementer to fetching-library-docs, browsing-with-playwright as examples for Lessons 3-4
3. **Provide MCP server examples**: Share Chapter 38 MCP server implementation as template for Lesson 4 students to wrap
4. **Quality validation**: After each lesson, invoke educational-validator to check constitutional compliance
5. **Capstone grading rubric**: Define acceptance criteria from success evals for Lesson 8 validation

---

**This plan activates specification-driven pedagogy, code execution pattern teaching, and Digital FTE production as the culmination. Every lesson contributes to students' ability to encode domain expertise as shippable skills.**
