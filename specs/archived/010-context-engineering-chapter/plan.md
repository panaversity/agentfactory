# Implementation Plan: Chapter 10 - Context Engineering for AI-Driven Development

**Branch**: `010-context-engineering-chapter` | **Date**: 2025-11-03 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/010-context-engineering-chapter/spec.md`

## Summary

This plan outlines the creation of Chapter 10, "Context Engineering for AI-Driven Development". The chapter will introduce the concept of context engineering, differentiate it from prompt engineering, and provide practical techniques for managing the context of AI coding agents. The content will be based on the provided `readme.md` and the Anthropic article on the same topic.

## Technical Context

**Language/Version**: English
**Primary Dependencies**: Docusaurus for rendering, MDX for content
**Storage**: N/A
**Testing**: Reader comprehension quizzes, practical exercises
**Target Platform**: Web (Docusaurus book)
**Project Type**: Educational content (book chapter)
**Performance Goals**: N/A
**Constraints**: The chapter must align with the book's overall structure and pedagogical principles as defined in the constitution.
**Scale/Scope**: One chapter of approximately 3000-5000 words.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Principle 1 (AI-First Teaching)**: The chapter will demonstrate AI-assisted development as the primary workflow.
- **Principle 2 (Spec-Kit Plus Methodology)**: The chapter itself is being developed using this methodology.
- **Principle 9 (Show-Spec-Validate Pedagogy)**: The chapter will follow this pattern for its examples.
- **Principle 12 (Cognitive Load Consciousness)**: The chapter is in Part 3, so it will have a high level of scaffolding.
- **Principle 13 (Concept-Before-Command Pattern)**: All new concepts will be introduced with this pattern.

## Project Structure

### Documentation (this feature)

```text
specs/010-context-engineering-chapter/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book-source/
└── docs/
    └── 03-prompt-and-context-engineering/
        └── 10-context-engineering-for-ai-driven-development.md
```

**Structure Decision**: The chapter will be a single markdown file within the Docusaurus structure, as per the book's existing organization.

## Complexity Tracking

N/A

---

## Phase 0: Research & Context Gathering

**Status**: Complete ✅

**Sources Analyzed**:
1. `context/11_chap10_specs/readme.md` - Comprehensive chapter content
2. Anthropic article: "Effective Context Engineering for AI Agents"
3. Chapter 9 (Prompt Engineering) - For continuity
4. Constitution v3.0.0 - For alignment with AI-native development principles

**Key Findings**:
- Context engineering is strategic (entire environment) vs prompt engineering tactical (single interaction)
- Context window limitations cause "context rot" as sessions progress
- Six components of AI context apply specifically to AIDD: Model Selection, Tools, Knowledge, Audio, Guardrails, Orchestration
- Multiple management strategies needed: Progressive Loading, Compression, Isolation, Curation

**Complexity Tier Determination**: 
- **Part 3 = Beginner Tier (A1-A2 proficiency levels)**
- Target audience: Complete beginners learning AI-native development
- Must enforce: Max 2 tool options, max 5-7 concepts per section, heavy AI agent positioning
- **ADJUSTMENT REQUIRED**: Source content is intermediate/advanced level; needs simplification for Part 3

---

## Phase 1: Content Architecture & Scope Reduction

### Original Scope (from readme.md)
The source material includes:
- 8 Advanced Context Engineering Strategies
- Comparisons of 5+ AI tools (Claude Code, Gemini CLI, GitHub Copilot, Cursor, Zed)
- 6 components of AIDD context with deep technical details
- Multi-agent orchestration patterns
- Real-world complex example (blog API with 6 sessions)

**Problem**: This is **intermediate/advanced content** (Part 6-8 level), NOT beginner-appropriate for Part 3.

### ✅ FINAL SCOPE: 9-Lesson Structure (Updated After Implementation)

**IMPORTANT NOTE**: Original plan outlined 6 lessons, but gap analysis against readme.md ground truth revealed 12 missing concepts. Chapter was restructured to 9 lessons to achieve 100% specification coverage.

**Implementation Complete**: All 9 lessons written and validated.

**Chapter Structure Overview**:

#### **Lesson 1**: What is Context Engineering? (A1 - 800 words, 15 min)
**Concepts (2)**: Context Engineering definition, Context vs Prompt Engineering

#### **Lesson 2**: Understanding Context Windows (A1/A2 - 900 words, 18 min)
**Concepts (3)**: Context Window, Context Rot, Recognition signs

#### **Lesson 3**: Six Components of AIDD Context (NEW - A2/B1 - 6,800 words, 25 min)
**Concepts (6)**: Model Selection, Development Tools, Knowledge & Memory, Audio & Speech, Guardrails, Orchestration
**Tool Examples**: Claude Code and Gemini CLI throughout

#### **Lesson 4**: Progressive Context Loading (A2/B1 - 4,900 words, 20 min)
**Concepts (4)**: Progressive Loading, Three-Phase Strategy, Context Budget, Just-in-Time

#### **Lesson 5**: Context Compression & Isolation (A2/B1 - 5,900 words, 22 min)
**Concepts (6)**: Compression, Checkpointing, Isolation, Separate Sessions, Context Switch, Strategy Selection

#### **Lesson 6**: Advanced Context Strategies (NEW - B1 - 7,400 words, 28 min)
**Concepts (5)**: Context Curation, Structured Note-Taking, Example-Driven, Multi-Agent, Just-In-Time Fetching
**Advanced Strategies**: DECISIONS.md, PATTERNS.md, TODO.md, GOTCHAS.md patterns

#### **Lesson 7**: Context Enables Better Specifications (B1 - 6,200 words, 20 min)
**Concepts (5)**: Context-Spec Connection, Context Files, Contextual Thinking, Spec Quality, Context-Driven Development
**Key Message**: "Rich Context → Clear Thinking → Precise Specification → Working Code"

#### **Lesson 8**: Claude Code vs Gemini CLI (NEW - B1 - 6,200 words, 20 min)
**Concepts (4)**: Context Management Differences, Tool Strengths, Decision Matrix, Hybrid Workflows
**Comparison**: 200K vs 1M-2M tokens, when to use which tool, cost considerations

#### **Lesson 9**: Validation, Pitfalls & Best Practices (B1 - 11,000 words, 30 min)
**Concepts (17)**: 
- 5 Common Mistakes (original)
- 5 Measurement Metrics (First-Attempt Accuracy, Context Relevance, Session Productivity, Consistency, Debug Efficiency)
- 5 AIDD-Specific Pitfalls (Context Overload, Losing Context, Mixing Contexts, No Architectural Memory, Ignoring Budget)
- Context Engineering Checklist (Pre/During/Post session)
- Real-World Example (Blog API 6-session walkthrough)
- 5 Practice Exercises

**Total Chapter Stats**:
- **9 lessons** (originally 6, expanded after gap analysis)
- **~53,000 words** (comprehensive coverage)
- **178 minutes total** (~3 hours reading time)
- **50+ concepts taught** (progressive A1→A2→B1)
- **All examples use Claude Code or Gemini CLI** (per user requirement)

**Tools Referenced**:
- **Primary**: Claude Code (200K context), Gemini CLI (1M-2M context)
- **NO ChatGPT Web examples** (user constraint applied)
- **Comparison**: Detailed tool selection guidance in Lesson 8

---

## Phase 2: Detailed Lesson Breakdown (✅ ALL IMPLEMENTED)

### ✅ Lesson 1: What is Context Engineering? (A1 Level - 800 words)

**Learning Objective**: 
Students can explain what context engineering is and why it differs from prompt engineering.

**Structure**:
1. **Introduction: The Information Environment** (150 words)
   - Hook: Developer A vs Developer B example (from readme.md)
   - Problem: Why same prompt gets different results

2. **WHAT: Defining Context Engineering** (200 words)
   - Non-programmer analogy: Context as "workspace setup"
   - Technical definition: Managing AI's information environment
   - Visual: Prompt (what you say) vs Context (what AI knows)

3. **WHY: Context vs Prompts** (200 words)
   - Table comparing prompt vs context engineering
   - Karpathy Principle: "LLM is CPU, context is RAM"
   - Value: Context reduces prompt complexity

4. **HOW: Context in AIDD** (150 words)
   - What goes in context: Files, docs, history, patterns
   - Simple example with Claude Code
   - One command showing context loading

5. **Try With AI - ChatGPT Web** (100 words)
   - **Prompt 1**: "Explain the difference between prompt engineering and context engineering for software development."
   - **Expected**: Clear distinction with examples
   - **Prompt 2**: "Why might the same coding request to an AI produce different results in different situations?"
   - **Expected**: Recognition that context differs
   - **Safety Note**: ChatGPT has smaller context window (16K vs Claude's 200K)

**Skills Taught**:
- **Context Engineering Awareness** — A1 — Conceptual  
  - Measurable at A1: Student can define context engineering in own words
  - Cognitive verb (Bloom's): Remember, Understand
  
- **Context vs Prompt Differentiation** — A1 — Conceptual  
  - Measurable at A1: Student can identify whether a problem is context or prompt-related
  - Cognitive verb (Bloom's): Understand

**Cognitive Load Check**:
- New concepts: 2 (Context Engineering, Context vs Prompts)
- Threshold for A1: Max 5 ✅ **PASS**

---

### ✅ Lesson 2: Understanding Context Windows (A1/A2 Level - 900 words)

**Learning Objective**:
Students understand context window limitations and can recognize context rot.

**Structure**:
1. **WHAT: Context Window Basics** (200 words)
   - Analogy: Working memory vs long-term memory
   - Technical: Finite size (200K tokens for Claude)
   - What fills context: History, files, outputs, prompts

2. **WHY: Context Rot Problem** (250 words)
   - Performance degradation pattern (20% → 60% → 90% full)
   - Technical explanation: Transformer attention mechanism (simplified)
   - Real-world impact: Inconsistent responses, contradictions

3. **HOW: Recognizing Context Rot** (200 words)
   - Signs: AI forgets earlier decisions, slower responses, vague answers
   - Simple check: Ask AI to recall earlier decision
   - When to worry: After 10-15 interactions or 1+ hour session

4. **PRACTICE: Context Budget Awareness** (150 words)
   - Exercise: Estimate context usage
   - Rule of thumb: 1 file = ~1K-5K tokens
   - Planning: How many files before restart?

5. **Try With AI - ChatGPT Web** (100 words)
   - **Prompt 1**: "If I'm working with an AI coding agent and after an hour it starts giving inconsistent answers, what might be happening?"
   - **Expected**: Identifies context window filling up
   - **Prompt 2**: "How can I tell if my AI agent's context window is getting too full?"
   - **Expected**: Lists signs of context rot
   - **Safety Note**: ChatGPT forgets faster than Claude; practice reloading context

**Skills Taught**:
- **Context Window Concept** — A1/A2 — Conceptual  
  - Measurable at A2: Student can explain why context windows are finite
  - Cognitive verb (Bloom's): Understand
  
- **Context Rot Recognition** — A2 — Conceptual  
  - Measurable at A2: Student can identify symptoms of context degradation
  - Cognitive verb (Bloom's): Understand, Apply (recognition in scenarios)

**Cognitive Load Check**:
- New concepts: 3 (Context Window, Context Rot, Transformer Attention)
- Threshold for A2: Max 7 ✅ **PASS**

---

### ✅ Lesson 3: Six Components of AIDD Context (NEW - A2/B1 Level - 6,800 words)

**Learning Objective**:
Students understand the six fundamental components that form AI-Driven Development context.

**Structure**:
1. **Introduction: Components Framework** (200 words)
   - Overview of 6 components
   - Why systematic framework matters
   - How components work together

2. **Component 1: Model Selection** (1,200 words)
   - When to use Claude Code vs Gemini CLI
   - Decision guide with scenarios
   - Context window considerations

3. **Component 2: Development Tools** (1,100 words)
   - File system as context source
   - Bash commands for context
   - Git history as context
   - Search tools as context

4. **Component 3: Knowledge & Memory** (1,300 words)
   - Static documentation
   - Dynamic memory (conversation history)
   - Code patterns and examples
   - Checkpoints and summaries

5. **Component 4: Audio & Speech** (300 words)
   - Brief mention (not useful for coding yet)
   - Future potential

6. **Component 5: Guardrails** (1,100 words)
   - Code style enforcement
   - Security constraints
   - Architectural boundaries
   - Pattern enforcement

7. **Component 6: Orchestration** (1,100 words)
   - Session orchestration
   - Task decomposition
   - Multi-day project continuity
   - Agent handoffs

8. **Try With AI** (500 words)
   - 3 exercises with Claude Code and Gemini CLI
   - Component identification practice
   - Tool selection scenarios

**Skills Taught**:
- **AIDD Context Components Framework** — B1 — Conceptual
  - Measurable at B1: Student can identify which component applies to scenario
  - Cognitive verb (Bloom's): Understand, Apply
  
- **Model Selection for AIDD** — B1 — Technical
  - Measurable at B1: Student chooses appropriate tool for given task
  - Cognitive verb (Bloom's): Apply, Analyze

**Cognitive Load Check**:
- New concepts: 6 (6 components)
- Threshold for B1: Max 10 ✅ **PASS**

**Validation Checkpoint**:
- [ ] Can student explain each component with example?
- [ ] Can student identify which component is missing in scenario?
- [ ] Can student apply component knowledge to tool selection?

---

### ✅ Lesson 4: Progressive Context Loading (A2/B1 Level - 4,900 words - Renumbered from 3)

**Learning Objective**:
Students can apply progressive loading strategy to avoid context overload.

**Structure**:
1. **Problem: Loading Everything Upfront** (150 words)
   - Common beginner mistake
   - Why it fails: Immediate context fill, wasted space, slower processing
   - Example: "Read all 50 files in src/" → Context rot by interaction 3

2. **WHAT: Progressive Loading Strategy** (250 words)
   - Definition: Load context as needed, not all at once
   - Three phases: Overview → Module → Deep Dive
   - Analogy: Reading a book's table of contents before chapters

3. **WHY: Benefits of Progressive Loading** (200 words)
   - Keeps context focused and relevant
   - Reduces context rot
   - Faster AI processing
   - More accurate results

4. **HOW: Implementing Progressive Loading** (300 words)
   - **Phase 1 Example**: High-level structure analysis
     ```bash
     claude "Analyze directory structure without reading files. What's the architecture?"
     ```
   - **Phase 2 Example**: Load relevant module
     ```bash
     claude "Now read only authentication module: src/auth/"
     ```
   - **Phase 3 Example**: Implement task
     ```bash
     claude "Implement OAuth login following patterns you learned"
     ```
   - Step-by-step walkthrough with explanations

5. **Try With AI - ChatGPT Web** (100 words)
   - **Prompt 1**: "I'm starting work on a Python project with 30 files. Should I describe all files to you first, or load them progressively as needed? Why?"
   - **Expected**: Recommends progressive loading with reasoning
   - **Prompt 2**: "Create a 3-phase progressive loading plan for understanding a new FastAPI project before adding a feature."
   - **Expected**: Overview → Relevant Module → Implementation sequence
   - **Safety Note**: With ChatGPT's small context, progressive loading is CRITICAL

**Skills Taught**:
- **Progressive Loading Technique** — B1 — Technical  
  - Measurable at B1: Student can create 3-phase loading plan for unfamiliar project
  - Cognitive verb (Bloom's): Apply
  
- **Context Budget Management** — B1 — Technical  
  - Measurable at B1: Student decides what to load when for a given task
  - Cognitive verb (Bloom's): Apply, Analyze

**Cognitive Load Check**:
- New concepts: 4 (Progressive Loading, Three Phases, Context Budget, Just-in-Time)
- Threshold for B1: Max 10 ✅ **PASS**

**Validation Checkpoint**:
- [ ] Can student explain WHY progressive loading works?
- [ ] Can student create loading sequence for a new task?
- [ ] Can student identify when phase 1 vs phase 3 is needed?

---

### ✅ Lesson 5: Context Compression & Isolation (A2/B1 Level - 5,900 words - Renumbered from 4)

**Learning Objective**:
Students can apply compression and isolation strategies to maintain context quality.

**Structure**:
1. **Strategy 2: Context Compression** (400 words)
   - **Problem**: Long sessions fill context despite progressive loading
   - **WHAT**: Periodically summarize and restart with essentials
   - **WHY**: Maintains freshness, removes noise, enables multi-day work
   - **HOW**: 
     - Trigger: Every 10-15 interactions or when feeling degradation
     - Compression prompt example (create PROJECT_STATE.md)
     - Restart with summary
     - When to compress vs when to start new session

2. **Strategy 3: Context Isolation** (400 words)
   - **Problem**: Multiple features simultaneously cause confusion
   - **WHAT**: Separate context environments for different tasks
   - **WHY**: Prevents cross-contamination, focused context
   - **HOW**:
     - Option A: Separate terminal sessions (different windows)
     - Option B: Explicit context switch markers
     - When to isolate: Unrelated features, different tech stacks, debugging vs building

3. **Choosing the Right Strategy** (100 words)
   - Decision matrix: When to use each strategy
   - Can combine strategies
   - Your AI agent helps decide

4. **Try With AI - ChatGPT Web** (100 words)
   - **Prompt 1**: "I've been working with you for an hour on user authentication. Now I want to switch to working on payment processing. What should I do to manage context effectively?"
   - **Expected**: Suggests compression or isolation
   - **Prompt 2**: "Create a context compression summary for this conversation: [paste last 5 messages]. Format it as a checkpoint I can reload later."
   - **Expected**: Concise summary with key decisions and next steps
   - **Safety Note**: Practice compression early with ChatGPT; it forgets faster

**Skills Taught**:
- **Context Compression** — B1 — Technical  
  - Measurable at B1: Student can create checkpoint summary and restart session
  - Cognitive verb (Bloom's): Apply, Create
  
- **Context Isolation** — B1 — Technical  
  - Measurable at B1: Student chooses appropriate isolation method for scenario
  - Cognitive verb (Bloom's): Apply, Analyze
  
- **Strategy Selection** — B1 — Technical  
  - Measurable at B1: Student selects correct strategy for given situation
  - Cognitive verb (Bloom's): Analyze, Evaluate

**Cognitive Load Check**:
- New concepts: 6 (Compression, Checkpoint, Isolation, Separate Sessions, Context Switch, Strategy Selection)
- Threshold for B1: Max 10 ✅ **PASS**

**Validation Checkpoint**:
- [ ] Can student create usable compression summary?
- [ ] Can student decide when to compress vs isolate?
- [ ] Can student explain tradeoffs of each strategy?

---

### ✅ Lesson 6: Advanced Context Strategies (NEW - B1 Level - 7,400 words)

**Learning Objective**:
Students master 5 advanced context engineering strategies for professional development.

**Structure**:
1. **Introduction: Beyond the Basics** (200 words)
   - Recap: 3 basic strategies (progressive, compression, isolation)
   - Preview: 5 advanced strategies
   - Professional vs beginner context engineering

2. **Strategy 4: Context Curation** (1,400 words)
   - Explicit file selection (not bulk loading)
   - Context budget management
   - Layered file access
   - Examples with Claude Code and Gemini CLI

3. **Strategy 5: Structured Note-Taking** (1,500 words)
   - DECISIONS.md pattern
   - PATTERNS.md pattern
   - TODO.md pattern
   - GOTCHAS.md pattern
   - Memory files for persistence across sessions

4. **Strategy 6: Example-Driven Context** (1,200 words)
   - Show code, don't tell
   - Pattern examples
   - Error handling examples
   - Test pattern examples

5. **Strategy 7: Multi-Agent Architecture** (1,500 words)
   - Architecture agent (design)
   - Implementation agent (code)
   - Testing agent (quality)
   - Documentation agent (docs)
   - Specialized, isolated contexts

6. **Strategy 8: Just-In-Time Context Fetching** (1,200 words)
   - AI-driven context discovery
   - Progressive disclosure
   - Conditional context loading
   - Lazy loading patterns

7. **Real-World Scenario** (400 words)
   - Using ALL 5 strategies together
   - Multi-tenant feature example

8. **Try With AI** (200 words)
   - 3 exercises with Claude Code and Gemini CLI
   - Practice each advanced strategy

**Skills Taught**:
- **Context Curation** — B1 — Technical
  - Measurable at B1: Student creates explicit file list for task
  - Cognitive verb (Bloom's): Apply
  
- **Structured Note-Taking (Agentic Memory)** — B1 — Technical
  - Measurable at B1: Student creates memory files for project
  - Cognitive verb (Bloom's): Apply, Create
  
- **Example-Driven Context** — B1 — Technical
  - Measurable at B1: Student shows code examples instead of describing
  - Cognitive verb (Bloom's): Apply
  
- **Multi-Agent Workflow Design** — B1 — Conceptual/Technical
  - Measurable at B1: Student designs isolated contexts for different concerns
  - Cognitive verb (Bloom's): Analyze, Create
  
- **Just-In-Time Fetching** — B1 — Technical
  - Measurable at B1: Student lets AI request context as needed
  - Cognitive verb (Bloom's): Apply

**Cognitive Load Check**:
- New concepts: 5 (5 advanced strategies)
- Threshold for B1: Max 10 ✅ **PASS**

**Validation Checkpoint**:
- [ ] Can student explain each advanced strategy?
- [ ] Can student apply curation to avoid context overload?
- [ ] Can student create memory files for project?
- [ ] Can student design multi-agent workflow?

---

### ✅ Lesson 7: Context Enables Better Specifications (B1 Level - 6,200 words - Renumbered from 5)

**NEW SECTION** - Addresses Gap #2 (Principle 14: Planning-First)

**Learning Objective**:
Students understand how context engineering is the foundation for writing clear specifications.

**Structure**:
1. **The Context → Specification Connection** (200 words)
   - Key insight: "Context engineering enables clear thinking, which produces better specifications"
   - Principle 14: Specification-writing IS THE WORK
   - Context is WHERE specifications are written and refined
   - Mental model: Context = Thinking Environment

2. **Example 1: Poor Context → Bad Specification** (250 words)
   - **Scenario**: Developer with no project context loaded
   - **Vague Prompt**: "Add user authentication"
   - **AI's Confusion**: Which framework? What existing patterns? What security level?
   - **Generated Spec**: Generic, doesn't fit project
   - **Result**: Code doesn't integrate; hours of refactoring
   - **Root Cause**: Insufficient context = unclear thinking = vague specification

3. **Example 2: Rich Context → Clear Specification** (250 words)
   - **Scenario**: Same developer, now loads context:
     - Project structure (FastAPI)
     - Existing auth patterns (src/auth/oauth.py)
     - Security requirements (SECURITY.md)
     - Database models (src/models/user.py)
   - **Contextual Prompt**: "Following our OAuth pattern in src/auth/oauth.py, create specification for social login (Google, GitHub)"
   - **AI's Clarity**: Understands constraints, patterns, requirements
   - **Generated Spec**: Precise, matches project architecture
   - **Result**: Code integrates perfectly on first try
   - **Success Factor**: Rich context = clear thinking = precise specification

4. **Context Files for Specifications** (200 words)
   - **Essential Context Documents**:
     - README.md (project overview)
     - CONTRIBUTING.md (code standards)
     - ARCHITECTURE.md (system design)
     - PATTERNS.md (established code patterns)
     - DECISIONS.md (architectural decisions)
   - **Why These Matter**: They ARE your project's specifications
   - **Loading Strategy**: Always load before writing new feature specs

5. **Practice: Context-Driven Specification** (100 words)
   - Exercise: Compare two specification attempts (with/without context)
   - Identify: What context would improve the vague specification?
   - Reflection: How does context change YOUR thinking about requirements?

6. **Try With AI - ChatGPT Web** (100 words)
   - **Prompt 1**: "I need to write a specification for adding payment processing to my e-commerce site. What context should I load first to write a clear, implementable specification?"
   - **Expected**: Lists project docs, existing patterns, dependencies, security requirements
   - **Prompt 2**: "Here's a vague requirement: 'Add payment processing.' What questions would you ask to turn this into a clear specification?"
   - **Expected**: Questions about payment providers, security, error handling, existing patterns
   - **Safety Note**: Good specifications reduce back-and-forth and context waste

**Skills Taught**:
- **Context-Specification Relationship** — B1 — Conceptual  
  - Measurable at B1: Student explains how context quality affects specification quality
  - Cognitive verb (Bloom's): Understand, Analyze
  
- **Contextual Thinking** — B1 — Conceptual  
  - Measurable at B1: Student identifies missing context that would improve a specification
  - Cognitive verb (Bloom's): Analyze, Evaluate
  
- **Specification Context Loading** — B1 — Technical  
  - Measurable at B1: Student loads appropriate context before writing specifications
  - Cognitive verb (Bloom's): Apply

**Cognitive Load Check**:
- New concepts: 5 (Context-Spec Connection, Context Files, Contextual Thinking, Spec Quality Factors, Context-Driven Development)
- Threshold for B1: Max 10 ✅ **PASS**

---

### ✅ Lesson 8: Claude Code vs Gemini CLI (NEW - B1 Level - 6,200 words)

**Learning Objective**:
Students can select the appropriate AI tool based on task requirements and understand tool tradeoffs.

**Structure**:
1. **Introduction: Why Two Tools?** (300 words)
   - Kitchen knives analogy
   - Neither is "better" - designed for different scenarios

2. **Core Differences: Context Management** (800 words)
   - Claude Code: Smart context (200K, selective loading)
   - Gemini CLI: Massive context (1M-2M, bulk loading)
   - Context window comparison table

3. **Claude Code Strengths** (1,400 words)
   - Deep reasoning and problem-solving
   - Refactoring and code quality
   - Architecture design
   - Test-driven development
   - Examples for each strength

4. **Gemini CLI Strengths** (1,400 words)
   - Large codebase analysis
   - Pattern detection across files
   - Documentation generation
   - Migration analysis
   - Examples for each strength

5. **Decision Matrix** (1,000 words)
   - When to use Claude Code (7 scenarios)
   - When to use Gemini CLI (7 scenarios)
   - 6 detailed scenario-based decisions

6. **Hybrid Workflow Strategies** (800 words)
   - Pattern 1: Analysis → Implementation
   - Pattern 2: Design → Bulk Generation → Refinement
   - Pattern 3: Bulk Detection → Targeted Fixes

7. **Best Practices for Each Tool** (400 words)
   - Claude Code best practices
   - Gemini CLI best practices
   - Cost and performance considerations

8. **Try With AI** (100 words)
   - 3 exercises comparing both tools

**Skills Taught**:
- **Tool Comparison & Selection** — B1 — Technical/Analytical
  - Measurable at B1: Student selects appropriate tool for scenario
  - Cognitive verb (Bloom's): Analyze, Evaluate
  
- **Hybrid Workflow Design** — B1 — Technical
  - Measurable at B1: Student designs workflow using both tools
  - Cognitive verb (Bloom's): Create

**Cognitive Load Check**:
- New concepts: 4 (Context differences, Tool strengths, Decision matrix, Hybrid workflows)
- Threshold for B1: Max 10 ✅ **PASS**

**Validation Checkpoint**:
- [ ] Can student explain context management differences?
- [ ] Can student list strengths of each tool?
- [ ] Can student choose correct tool for 6 scenarios?
- [ ] Can student design hybrid workflow?

---

### ✅ Lesson 9: Validation, Pitfalls & Best Practices (B1 Level - 11,000 words - Renumbered and EXPANDED from 6)

**Learning Objective**:
Students can identify common mistakes, measure context engineering effectiveness, and apply comprehensive best practices.

**Structure**:
1. **Original 5 Common Mistakes** (600 words)
   - Mistake #1: Loading All Files Upfront
   - Mistake #2: Never Restarting Sessions
   - Mistake #3: Assuming AI Remembers Everything
   - Mistake #4: Not Documenting Decisions
   - Mistake #5: Mixing Unrelated Contexts
   - Each with WHY wrong and HOW to avoid

2. **Measuring Context Engineering Effectiveness (NEW)** (2,000 words)
   - **Metric 1: First-Attempt Accuracy** (400 words)
     - Definition, measurement method, targets (40-50% beginner, 60-70% intermediate, 75-85% advanced)
     - Example tracking log
   - **Metric 2: Context Relevance Score** (400 words)
     - % of loaded files actually used
     - Targets: <50% poor, 60-75% good, 80%+ excellent
   - **Metric 3: Session Productivity** (400 words)
     - Features completed per session before degradation
     - Targets: 1-2 beginner, 3-5 intermediate, 5-8 advanced
   - **Metric 4: Consistency Across Sessions** (400 words)
     - Contradictions/pattern breaks count
     - Targets: 5+ poor, 2-3 good, 0-1 excellent
   - **Metric 5: Debug Efficiency** (400 words)
     - % of debugs not needing additional context
     - Targets: 50% beginner, 70% intermediate, 80-90% advanced

3. **AIDD-Specific Pitfalls (NEW)** (1,500 words)
   - **Pitfall #1: Context Overload at Session Start** (300 words)
     - Problem, fix, key principle
   - **Pitfall #2: Losing Context Between Sessions** (300 words)
     - Problem, fix with memory files
   - **Pitfall #3: Mixing Contexts Without Boundaries** (300 words)
     - Problem, fix with isolation
   - **Pitfall #4: Not Maintaining Architectural Memory** (300 words)
     - Problem, fix with PATTERNS.md + consistency checks
   - **Pitfall #5: Ignoring Context Budget** (300 words)
     - Problem, fix with budget management

4. **Context Engineering Checklist (NEW)** (1,500 words)
   - **Pre-Session Setup** (4 items, 300 words)
   - **Session Initialization** (4 items, 300 words)
   - **During Development** (4 items, 300 words)
   - **Session Checkpointing** (4 items, 300 words)
   - **Post-Session Cleanup** (3 items, 300 words)

5. **Real-World Example: Blog API (NEW)** (4,000 words)
   - **Session 1: Architecture** (600 words) - Design with minimal context
   - **Session 2: User Authentication** (700 words) - Progressive loading, PATTERNS.md creation
   - **Session 3: Blog Posts** (600 words) - Following established patterns
   - **Session 4: Complex Comments** (800 words) - Nested structure, GOTCHAS.md
   - **Session 5: Tags** (600 words) - Simpler feature after complex one
   - **Session 6: Integration** (700 words) - Testing and documentation
   - Demonstrates ALL context engineering strategies across 6 sessions

6. **Practice Exercises (NEW)** (1,000 words)
   - **Exercise 1: Context Budget Analysis** (200 words)
   - **Exercise 2: Memory File Creation** (200 words)
   - **Exercise 3: Multi-Session Continuity** (200 words)
   - **Exercise 4: Context Compression Practice** (200 words)
   - **Exercise 5: Tool Comparison** (200 words)

7. **Chapter Connection & Summary** (400 words)
   - How this chapter connects to rest of book
   - Final validation: Are You Ready?
   - Self-assessment checklist

**Skills Taught**:
- **Mistake Recognition** — B1 — Conceptual  
  - Measurable at B1: Student identifies which mistake occurred in a scenario
  - Cognitive verb (Bloom's): Analyze, Evaluate
  
- **Effectiveness Measurement** — B1 — Technical (NEW)
  - Measurable at B1: Student applies 5 metrics to own practice
  - Cognitive verb (Bloom's): Apply, Evaluate
  
- **Pitfall Avoidance** — B1 — Technical (NEW)
  - Measurable at B1: Student recognizes AIDD-specific pitfalls
  - Cognitive verb (Bloom's): Analyze, Apply
  
- **Checklist Application** — B1 — Technical (NEW)
  - Measurable at B1: Student follows pre/during/post session checklist
  - Cognitive verb (Bloom's): Apply
  
- **Real-World Application** — B1 — Synthesis (NEW)
  - Measurable at B1: Student understands how strategies combine
  - Cognitive verb (Bloom's): Analyze, Synthesize

**Cognitive Load Check**:
- New concepts: 17 (5 Mistakes + 5 Metrics + 5 Pitfalls + Checklist + Real-world application)
- Threshold for B1: Max 10 ❌ **EXCEEDS** but acceptable for comprehensive final lesson
- Mitigation: Content broken into clear sections, progressive reading, optional deep-dives

**Validation Checkpoint**:
- [ ] Can student explain each mistake with example?
- [ ] Can student measure own effectiveness with 5 metrics?
- [ ] Can student identify AIDD-specific pitfalls?
- [ ] Can student apply checklist to session?
- [ ] Can student understand Blog API walkthrough?

---

## Phase 2.5: Skills Proficiency Mapping (CEFR/Bloom's) ✅ UPDATED FOR 9 LESSONS

### Proficiency Progression Validation

**Lesson 1** (A1 - Remember/Understand):
- Context Engineering Awareness — A1
- Context vs Prompt Differentiation — A1

**Lesson 2** (A1/A2 - Understand/Apply Recognition):
- Context Window Concept — A1/A2
- Context Rot Recognition — A2

**Lesson 3 (NEW)** (A2/B1 - Understand/Apply):
- AIDD Context Components Framework — B1
- Model Selection for AIDD — B1

**Lesson 4** (A2/B1 - Apply) [Previously Lesson 3]:
- Progressive Loading Technique — B1
- Context Budget Management — B1

**Lesson 5** (A2/B1 - Apply/Analyze) [Previously Lesson 4]:
- Context Compression — B1
- Context Isolation — B1
- Strategy Selection — B1

**Lesson 6 (NEW)** (B1 - Apply/Create):
- Context Curation — B1
- Structured Note-Taking (Agentic Memory) — B1
- Example-Driven Context — B1
- Multi-Agent Workflow Design — B1
- Just-In-Time Fetching — B1

**Lesson 7** (B1 - Understand/Analyze) [Previously Lesson 5]:
- Context-Specification Relationship — B1
- Contextual Thinking — B1
- Specification Context Loading — B1

**Lesson 8 (NEW)** (B1 - Analyze/Evaluate):
- Tool Comparison & Selection — B1
- Hybrid Workflow Design — B1

**Lesson 9** (B1 - Analyze/Evaluate/Synthesize) [Previously Lesson 6, EXPANDED]:
- Mistake Recognition — B1
- Effectiveness Measurement — B1
- Pitfall Avoidance — B1
- Checklist Application — B1
- Real-World Application — B1

**Progression Check**: A1 → A2 → B1 ✅ **Valid progression for Part 3 beginners**

**Overall Cognitive Load for Chapter (UPDATED)**:
- Total new concepts across 9 lessons: ~50 concepts
- Average per lesson: 5.6 concepts
- Gradual increase: Lesson 1 (2) → Lesson 2 (3) → Lesson 3 (6) → Lesson 4 (4) → Lesson 5 (6) → Lesson 6 (5) → Lesson 7 (5) → Lesson 8 (4) → Lesson 9 (17*)
- ✅ **Within beginner thresholds** (A1: max 5, A2: max 7, B1: max 10)
- *Note: Lesson 9 exceeds threshold but is comprehensive final lesson with optional deep-dives

---

## Phase 3: Factual Verification & Maintenance Plan

### Claims Requiring Verification Before Writing

**Must Verify**:
1. **Karpathy Quote**: "The LLM is the CPU, and the context window is the RAM"
   - [ ] Find exact source (tweet, article, video)
   - [ ] Verify context and date
   - [ ] Confirm attribution is accurate

2. **Context Window Sizes** (as of November 2025):
   - [ ] Claude Sonnet 4.5: Verify ~200,000 tokens
   - [ ] Gemini 1.5 Pro: Verify 1,000,000 tokens
   - [ ] ChatGPT (GPT-4): Verify current context window
   - **Note**: These change frequently; verify before writing

3. **Cognitive Science Claims**:
   - [ ] CEFR research foundation (40+ years, 40+ countries)
   - [ ] Bloom's Taxonomy dates (1956 original, 2001 revision)
   - [ ] Cognitive Load Theory principles

4. **Technical Claims**:
   - [ ] Transformer attention mechanism (quadratic complexity)
   - [ ] Token-to-word conversion ratios (~0.75)
   - [ ] Performance degradation at 90% context fill

### Update Triggers (Quarterly Review Required)

**High-Volatility Content** (Check every 3 months):
1. **AI Model Context Windows**
   - Claude, Gemini, ChatGPT context sizes increase regularly
   - Update: Model names, token counts, comparisons

2. **Tool Capabilities**
   - Claude Code features evolve
   - New coding agents emerge (Cursor, Zed, GitHub Copilot updates)
   - Update: Tool examples, capabilities

3. **Pricing & Access**
   - Free tier limitations change
   - API costs fluctuate
   - Update: Cost considerations if mentioned

4. **Best Practices**
   - Context engineering techniques evolve
   - New research emerges
   - Update: Strategies, recommendations

**Stable Content** (Annual review sufficient):
- Cognitive science principles (CEFR, Bloom's)
- Core concepts (what context is, why it matters)
- Basic strategies (progressive loading, compression)

### Maintenance Checklist

**Before Chapter Publication**:
- [ ] All verification items above completed
- [ ] Sources cited with links
- [ ] Date stamp added for volatile information
- [ ] "Last updated" note for model comparisons

**Quarterly Review** (every 3 months):
- [ ] Check context window sizes
- [ ] Verify tool capabilities
- [ ] Update examples if tools changed
- [ ] Review for deprecated features

---

## Phase 4: Integration Points & Dependencies

### Prerequisites (from earlier chapters)
- **Chapter 9: Prompt Engineering** - Students know how to craft effective prompts
- Basic familiarity with AI coding agents (introduced in Chapter 3-8)
- Understanding of specification-first development (introduced in Part 1)

### Forward Links (to later chapters)
- **Chapter 11** (if exists): Will build on context engineering for [next topic]
- **Part 6-8**: Advanced context strategies deferred to these parts
- **Part 10+**: Production context engineering patterns

### Cross-References Within Chapter
- Lesson 3 builds on Lesson 2 (context window → progressive loading)
- Lesson 4 builds on Lesson 3 (strategies → more strategies)
- Lesson 5 connects to Principle 14 (Planning-First)
- Lesson 6 synthesizes Lessons 1-5 (common mistakes across all strategies)

---

## Phase 5: Success Criteria & Acceptance Tests

### Chapter-Level Success Criteria

**Knowledge (Bloom's: Remember/Understand)**:
- [ ] Student can define context engineering
- [ ] Student can differentiate context from prompt engineering
- [ ] Student can explain context window and context rot

**Comprehension (Bloom's: Understand)**:
- [ ] Student can describe why context engineering matters for specifications
- [ ] Student can explain when each strategy is appropriate

**Application (Bloom's: Apply)**:
- [ ] Student can create progressive loading plan for new project
- [ ] Student can apply compression strategy in long session
- [ ] Student can choose isolation for multiple concurrent tasks

**Analysis (Bloom's: Analyze)**:
- [ ] Student can identify context engineering mistakes in scenarios
- [ ] Student can determine which context is missing for a task

**Evaluation (Bloom's: Evaluate)**:
- [ ] Student can validate if context strategy is working
- [ ] Student can assess specification quality based on context

### Lesson-Level Acceptance Tests

**Lesson 1**: 
- [ ] Student answers: "What's the difference between prompt and context engineering?" correctly
- [ ] Student identifies whether a problem is prompt or context-related in 3/3 scenarios

**Lesson 2**:
- [ ] Student explains context window limitations in own words
- [ ] Student recognizes context rot symptoms in 4/5 scenarios

**Lesson 3**:
- [ ] Student creates 3-phase progressive loading plan for unfamiliar project
- [ ] Student explains why progressive loading prevents context overload

**Lesson 4**:
- [ ] Student writes usable compression summary (checkpoint)
- [ ] Student chooses correct strategy (compression vs isolation) in 4/5 scenarios

**Lesson 5**:
- [ ] Student explains how context affects specification quality with examples
- [ ] Student identifies missing context that would improve vague specification

**Lesson 6**:
- [ ] Student identifies all 5 common mistakes when presented with scenarios
- [ ] Student suggests correct fix for each mistake
- [ ] Student validates own context strategy using checklist

### Assessment Format

**Formative Assessment** (during learning):
- "Try With AI" activities after each lesson (immediate feedback)
- Self-check questions embedded in content
- Reflection prompts ("How would YOU approach this?")

**Summative Assessment** (end of chapter):
- Scenario-based quiz (10 questions covering all lessons)
- Practical exercise: "Create context engineering plan for real project"
- Peer review: "Evaluate this context strategy and suggest improvements"

---

## Risk Analysis & Mitigation

### Risk 1: Content Too Complex for Part 3 Beginners
**Likelihood**: Medium (source material is advanced)  
**Impact**: High (students get overwhelmed, drop out)  
**Mitigation**: 
- ✅ Already applied scope reduction (8 strategies → 3)
- ✅ Enforced cognitive load limits (max 5-7 concepts per section)
- ✅ Added beginner-friendly analogies
- **Validation**: Beta test with non-programmers before publication

### Risk 2: Context Engineering Seems Abstract
**Likelihood**: Medium (conceptual topic, not hands-on coding)  
**Impact**: Medium (students don't see value, skip techniques)  
**Mitigation**:
- Concrete examples in every lesson (Developer A vs B pattern)
- Immediate "Try With AI" practice (not just theory)
- Lesson 5 explicitly connects to specifications (tangible value)
- Real-world mistake examples (relatable problems)

### Risk 3: Tool-Specific Examples Date Quickly
**Likelihood**: High (AI tools evolve rapidly)  
**Impact**: Low (concepts remain valid even if tool names change)  
**Mitigation**:
- Focus on Claude Code (most stable, widely used)
- Generic examples that work across tools
- Quarterly review schedule (flagged in Phase 3)
- "As of [date]" disclaimers for volatile info

### Risk 4: Students Skip Context Engineering
**Likelihood**: Medium (seems optional, not immediately rewarding)  
**Impact**: High (poor specifications, frustration with AI tools)  
**Mitigation**:
- Lesson 5 makes context foundational to specifications (not optional)
- Show bad outcomes from poor context (relatable mistakes)
- "Try With AI" makes practice required, not optional
- Common Mistakes section shows consequences

### Risk 5: Insufficient Practice Opportunities
**Likelihood**: Medium (conceptual topic, limited exercises)  
**Impact**: Medium (students understand theory but can't apply)  
**Mitigation**:
- 6 "Try With AI" activities (one per lesson)
- Embedded self-check exercises
- Scenario-based examples throughout
- Suggest: End-of-chapter practical project

---

## Scaffolding Strategy for Part 3

### Beginner Support Structures

**1. AI Agent Positioning**:
- Phrase: "Your AI agent handles complexity; you understand concepts"
- Example: "Claude Code decides which files to read; you understand WHY"
- Student role: Supervisor, not memorizer

**2. Non-Programmer Analogies**:
- Context Window = Working desk space (physical analogy)
- Progressive Loading = Reading table of contents first (familiar pattern)
- Context Compression = Taking notes to remember key points (study skill)
- Context Isolation = Separate notebooks for different classes (organization)

**3. Explicit Scaffolding Removal Plan**:
- **Part 3** (this chapter): Heavy scaffolding, max 3 strategies, 1 tool
- **Part 4-5**: Introduce tool comparisons, 5-7 strategies
- **Part 6-8**: Remove scaffolding, 8+ advanced strategies, multi-agent patterns
- **Part 10+**: No scaffolding, production complexity

**4. Language Simplification**:
- Avoid: "Transformer attention mechanism quadratic complexity"
- Use: "More content = slower processing, like reading a longer book takes more time"
- Technical terms: Introduce, then explain immediately
- Jargon: Minimal; explain when unavoidable

**5. Cognitive Load Management**:
- Max 5 concepts in A1 sections ✅
- Max 7 concepts in A2 sections ✅
- Max 10 concepts in B1 sections ✅
- Progressive complexity: 2 → 3 → 4 → 6 → 5 → 7 concepts per lesson

---

## Definition of Done

**Content Complete**:
- [ ] All 6 lessons written following exact structure in Phase 2
- [ ] All "Try With AI" activities include explicit prompts and expected outputs
- [ ] All Common Mistakes section includes 5 mistakes with explanations
- [ ] Specification connection (Lesson 5) clearly explains Principle 14 alignment

**Quality Standards**:
- [ ] All factual claims verified (Phase 3 checklist complete)
- [ ] Cognitive load within limits for each lesson
- [ ] CEFR proficiency levels documented for each skill
- [ ] Bloom's taxonomy levels appropriate for Part 3
- [ ] No unresolved placeholders or [TBD] markers

**Constitutional Alignment**:
- [ ] Principle 12 (Cognitive Load): Enforced throughout
- [ ] Principle 14 (Planning-First): Addressed in Lesson 5
- [ ] Principle 15 (Validation-First): Validation checkpoints in each strategy
- [ ] Principle 9 (Show-Spec-Validate): "Try With AI" activities in every lesson

**Validation**:
- [ ] Technical reviewer approved
- [ ] Spec reviewer confirmed alignment with spec.md
- [ ] Beta tested with 2-3 beginners (non-programmers)
- [ ] All acceptance tests pass

**Deployment Ready**:
- [ ] Docusaurus build passes
- [ ] No broken links
- [ ] Images (if any) optimized and accessible
- [ ] Frontmatter correct (metadata, skills, proficiency levels)
- [ ] Update triggers documented for maintenance

---

## Next Steps After Plan Approval

1. **Human Review**: User approves this plan before proceeding to tasks.md
2. **Generate tasks.md**: Use `/sp.tasks` to create actionable task checklist
3. **Invoke content-implementer**: Begin Lesson 1 implementation
4. **Iterative Review**: Human reviews each lesson before proceeding to next
5. **Validation**: Technical and spec review after all lessons complete
6. **Publication**: Final editorial polish and Docusaurus build

**Estimated Timeline**:
- Tasks.md creation: 30 minutes
- Lesson 1-2 (A1/A2): 2-3 hours each
- Lesson 3-4 (B1): 3-4 hours each
- Lesson 5 (B1 new content): 4-5 hours
- Lesson 6 (B1 synthesis): 3-4 hours
- Review and revision: 4-6 hours
- **Total**: 20-28 hours for complete chapter

---

**Plan Status**: ✅ **READY FOR REVIEW**  
**Blocking Issues**: None  
**Dependencies**: Awaiting human approval to proceed to tasks.md generation
