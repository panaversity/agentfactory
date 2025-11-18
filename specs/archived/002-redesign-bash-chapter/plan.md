# Bash Chapter Redesign: Detailed Implementation Plan

**Specification**: `/specs/002-redesign-bash-chapter/spec.md`
**Chapter Type**: Technical + Dialogue-First (Part 2, Chapter 7)
**Branch**: `002-redesign-bash-chapter`
**Created**: 2025-11-02
**Status**: Ready for Implementation
**Model**: Claude Haiku 4.5

---

## Executive Summary

This plan transforms the current bash chapter from a command-focused tutorial into a **dialogue-first, AI-centered learning experience**. The core innovation is teaching bash through natural conversations between learner and AI companion (Claude Code, Gemini CLI) rather than isolated command syntax.

**Key Paradigm Shift**:
- FROM: Command lists with syntax → isolated terminal exercises
- TO: Natural dialogue flow → learner requests → AI executes → output → explanation → safety questions

**Chapter Scope**: 8 lessons, 270-340 minutes (~4.5-5.5 hours), progressive complexity from A1 (recognition) to A2 (simple application)

**Core Innovation**: Every bash concept appears **only** within authentic dialogue examples, never in isolation. Learner develops understanding through conversation patterns, not syntax memorization.

---

## Chapter Overview & Learning Path

### Target Audience
- **Prior Knowledge**: Completed Chapter 6 (Claude Code / Gemini CLI introduction) + basic computer literacy
- **Motivation**: "I want to understand what my AI is doing when it works in the terminal"
- **Key Insight**: AI companion is doing work on learner's behalf; learner's job is to understand, verify, and supervise

### Learning Progression (A1 → A2)

| Lesson | Focus | Duration | CEFR | Concepts | Safety |
|--------|-------|----------|------|----------|--------|
| 1 | Workspace (pwd, ls) | 35 min | A1 | 3 | None |
| 2 | Safety pattern | 35 min | A1→A2 | 4 | Intro |
| 3 | Navigation (cd, paths) | 40 min | A2 | 5 | Practiced |
| 4 | File ops (cp, mv, rm) | 45 min | A2 | 5 | Central |
| 5 | Configuration | 40 min | A2 | 5 | Critical |
| 6 | Packages (pip, npm) | 40 min | A2 | 4 | Verify |
| 7 | Pipes and chaining | 40 min | A2 | 6 | Understand |
| 8 | Real project + troubleshoot | 50 min | A2→B1 | 7 | Integrated |

**Total Duration**: 325 minutes (5 hours 25 minutes)

### Prerequisites & Knowledge Activation

**Required Before Chapter**:
1. Chapter 6 completion (Claude Code or AI CLI tool available)
2. Can open terminal on their system
3. Basic file/folder understanding
4. Growth mindset about learning through dialogue with AI

---

## Detailed Lesson Plans (Summary)

### Lesson 1: Introducing Your AI Companion's Workspace (35 min)

**Learning Objectives** (CEFR A1):
- LO-1-1: Identify current directory concept by observing pwd in dialogue
- LO-1-2: Read and interpret ls output (files, folders, permissions)
- LO-1-3: Ask AI "Where are you?" and understand the response

**Key Concepts** (3 new):
1. AI has a "current location" in filesystem (pwd shows this)
2. Files/folders visible to AI; learner supervises
3. ls shows files with meaningful columns

**Cognitive Load**: 3 new concepts (SAFE for A1)

**Content**: Opening hook → pwd in dialogue → ls interpretation → supervision concept → scaffolding questions

**Dialogue Examples** (3 real interactions):
- Example 1-1: Basic location question (pwd natural conversation)
- Example 1-2: Understanding file listing (ls output explained)
- Example 1-3: Confirming location before action (safety pattern intro)

**Exercises** (3):
- E1-1: Prediction exercise (predict ls output)
- E1-2: Dialogue continuation (fill in missing parts)
- E1-3: Safety pattern introduction (why confirm location?)

**Assessments**:
- Formative: Learner predicts pwd output, interprets ls
- Summative: 3 conversations with AI about location/files; explain outputs

**Try With AI** (5 min):
- Tool: ChatGPT Code Interpreter or Claude Code
- Prompt 1: Show dialogue asking where AI works + what files exist
- Prompt 2: Explain what pwd and ls are doing in the dialogue

---

### Lesson 2: The Safety-First Dialogue Pattern (35 min)

**Learning Objectives** (CEFR A1→A2):
- LO-2-1: Identify 5-step safety pattern (Ask → Explain → Understand → Verify → Execute)
- LO-2-2: Demonstrate asking "Is this safe?" before destructive operations
- LO-2-3: Verify operations worked through dialogue

**Key Concepts** (4 new):
1. Safety dialogue pattern (Ask → Understand → Verify → Execute)
2. Destructive operations need backup plans
3. Verification commands show "did it work?"
4. Learner's responsibility: understand before executing

**Cognitive Load**: 4 new concepts (SAFE for A1→A2)

**Content**: Why safety matters → pattern step-by-step → real dialogues → identifying destructive ops → learner responsibility

**Dialogue Examples** (3):
- Example 2-1: Good safety dialogue (copy/backup operation)
- Example 2-2: Bad dialogue (what NOT to do)
- Example 2-3: Verification in action

**Exercises** (3):
- E2-1: Safety dialogue evaluation
- E2-2: Asking clarifying questions
- E2-3: Verification challenge

**Assessments**:
- Formative: Identify safety pattern steps; point out gaps
- Summative: Write complete dialogue showing safety pattern with 5 steps

**Try With AI** (5 min):
- Prompt 1: Explain safety pattern for deletion operation
- Prompt 2: Why is understanding before executing most important?

---

### Lesson 3: Understanding File Navigation Through Dialogue (40 min)

**Learning Objectives** (CEFR A2):
- LO-3-1: Predict file structure from navigation dialogue
- LO-3-2: Understand absolute vs. relative paths through conversation
- LO-3-3: Draw or describe folder structure from navigation dialogue

**Key Concepts** (5 new):
1. Absolute paths (/Users/mjs/Projects)
2. Relative paths (./src, ../backup)
3. cd command changes location
4. .., . have conversation analogues
5. Path reading reduces need to memorize

**Cognitive Load**: 5 new concepts (MAX for A2)

**Content**: Paths as directions → absolute vs. relative → building mental maps → real project context → safety in navigation

**Dialogue Examples** (3):
- Example 3-1: Absolute path navigation
- Example 3-2: Relative path (going up)
- Example 3-3: Predicting path depth

**Exercises** (3):
- E3-1: Path structure mapping (draw from dialogue)
- E3-2: Predicting navigation output
- E3-3: Path explanation in plain language

**Assessments**:
- Formative: Draw structure; explain "why would .. take us here?"
- Summative: Navigate 3 folders; draw structure; describe how to get back

**Try With AI** (5 min):
- Prompt 1: Build navigation dialogue (absolute + relative paths)
- Prompt 2: Why is pwd after every cd important?

---

### Lesson 4: Understanding File Operations Safely (45 min)

**Learning Objectives** (CEFR A2):
- LO-4-1: Request file operations; understand resulting commands
- LO-4-2: Identify data loss risks; ask "should we backup first?"
- LO-4-3: Verify file operations succeeded

**Key Concepts** (5 new):
1. Destructive operations need planning (cp for backup first)
2. Commands appear only in dialogue
3. Flags modify behavior; agent explains purpose
4. Verification shows before/after snapshots
5. Learner's role: understand → ask → verify → decide

**Cognitive Load**: 5 new concepts (MAX for A2)

**Content**: Operations as conversation → copy operations → move/rename → delete safely → verification patterns

**Dialogue Examples** (2):
- Example 4-1: Full file operation with safety (backup → move → verify)
- Example 4-2: Asking for backups (comprehensive approach)

**Exercises** (3):
- E4-1: Safety dialogue evaluation
- E4-2: Asking clarifying questions
- E4-3: Verification challenge (suggest verification commands)

**Assessments**:
- Formative: Identify which ops need backup; suggest verification
- Summative: Complete dialogue showing backup → operation → verification

**Try With AI** (5 min):
- Prompt 1: Walk through safe deletion process with backup
- Prompt 2: What's the riskiest operation? How to make it safer?

---

### Lesson 5: Understanding Configuration and Secrets (40 min)

**Learning Objectives** (CEFR A2):
- LO-5-1: Understand temporary (export) vs. persistent (~/.bashrc) configuration
- LO-5-2: Request config help; identify security implications
- LO-5-3: Verify configuration worked

**Key Concepts** (5 new):
1. export = temporary (this session only)
2. ~/.bashrc = persistent (survives restarts)
3. Secrets never hardcoded; use environment variables
4. Verification means "does variable work?"
5. Configuration security is critical

**Cognitive Load**: 5 new concepts (MAX for A2)

**Content**: Why config matters → temporary with export → persistent ~/.bashrc → security best practices → verification patterns

**Dialogue Examples** (2):
- Example 5-1: Complete configuration workflow (temp + persistent)
- Example 5-2: The security conversation (why not hardcode?)

**Exercises** (3):
- E5-1: Temp vs. persistent identification
- E5-2: Security review (identify unsafe code)
- E5-3: Verification commands

**Assessments**:
- Formative: Identify temp vs. persistent; explain security concern
- Summative: Set variable both ways with dialogue; demonstrate reload

**Try With AI** (5 min):
- Prompt 1: Complete API key setup (temporary + permanent)
- Prompt 2: What to do if API key accidentally committed?

---

### Lesson 6: Understanding Dependencies and Packages (40 min)

**Learning Objectives** (CEFR A2):
- LO-6-1: Understand package installation concept through dialogue
- LO-6-2: Request package installation; understand what's added
- LO-6-3: Verify package installation using agent-guided verification

**Key Concepts** (4 new):
1. Package = reusable code library
2. Package managers (pip, npm) handle installation
3. Installation adds code you can import
4. Verification means "can I import it?"

**Cognitive Load**: 4 new concepts (SAFE for A2)

**Content**: What are packages? → installation through dialogue → verification → different package managers → installation safety

**Dialogue Examples** (2):
- Example 6-1: Complete package installation (pip install + verify)
- Example 6-2: Understanding dependencies

**Exercises** (3):
- E6-1: Package concept explanation
- E6-2: Choosing package managers (pip vs. npm)
- E6-3: Verification commands

**Assessments**:
- Formative: Explain what package is; choose package manager
- Summative: Install real package with dialogue; verify it works

**Try With AI** (5 min):
- Prompt 1: Is package installation safe? What to watch for?
- Prompt 2: Why do we need package managers?

---

### Lesson 7: Understanding Pipes and Complex Commands (40 min)

**Learning Objectives** (CEFR A2):
- LO-7-1: Trace data flowing through piped commands
- LO-7-2: Request filtering/searching; understand pipeline building
- LO-7-3: Modify pipeline requests based on output

**Key Concepts** (6 new):
1. Pipes (|) connect commands
2. Data flows left-to-right
3. grep for searching
4. wc for counting
5. head/tail for limiting
6. Iteration: modify request, rebuild pipeline

**Cognitive Load**: 6 new concepts (SAFE for A2)

**Content**: Data flow analogy → step-by-step pipe building → reading complex pipelines → iteration through dialogue → real examples

**Dialogue Examples** (2):
- Example 7-1: Building pipeline step-by-step
- Example 7-2: Understanding data flow (detailed tracing)

**Exercises** (3):
- E7-1: Tracing data flow (3-step pipeline)
- E7-2: Pipeline building (find 5 largest files)
- E7-3: Iterative refinement

**Assessments**:
- Formative: Trace data; explain what each command does
- Summative: Build pipeline answering "how many different file types?"; modify for src folder

**Try With AI** (5 min):
- Prompt 1: Build pipeline to find recently modified files
- Prompt 2: How to refine pipeline to show only top 10 results?

---

### Lesson 8: Working with Real AI Tools and Troubleshooting (50 min)

**Learning Objectives** (CEFR A2→B1):
- LO-8-1: Complete multi-step project setup with AI help
- LO-8-2: Encounter real errors; work with AI to understand and fix
- LO-8-3: Apply troubleshooting pattern (error → explanation → diagnosis → fix → verify)

**Key Concepts** (7 new):
1. Real project setup orchestrates all prior skills
2. Real error messages (authentic, not hypothetical)
3. Error diagnosis pattern (read → explain → diagnose)
4. Iterative fixes (error → fix → verify → success)
5. Context debugging (what changed, what broke)
6. Fallback strategies (if approach 1 fails, try 2)
7. Persistence (troubleshooting is normal)

**Cognitive Load**: 7 new concepts (ACCEPTABLE for capstone)

**Content**: Real project setup → when things go wrong → troubleshooting pattern → real errors → resilience and iteration

**Dialogue Examples** (2):
- Example 8-1: Complete real project setup
- Example 8-2: Real error and troubleshooting

**Exercises** (3):
- E8-1: Error interpretation
- E8-2: Troubleshooting sequence
- E8-3: Real project setup

**Assessments**:
- Formative: Interpret error; apply troubleshooting pattern
- Summative (CAPSTONE): Set up real project with error encountered + fixed

**Try With AI** (5 min):
- Prompt 1: Complete project setup with error handling strategy
- Prompt 2: Reflect on most important learning about working with AI

---

## Skills & Proficiency Mapping

### Domain Skills Applied

| Skill | Applied How | Lessons |
|-------|-------------|---------|
| **learning-objectives** | Measurable A1/A2 objectives, Bloom's aligned | All |
| **assessment-builder** | Dialogue-based formative + summative | All |
| **technical-clarity** | Explain bash in non-programmer language | All |
| **book-scaffolding** | Progressive complexity A1→A2, cognitive load management | 1→8 |
| **concept-scaffolding** | Break "workspace supervision" into components | 1,3 |
| **exercise-designer** | Dialogue prediction + verification exercises | All |
| **ai-collaborate-teaching** | Dialogue-first as core teaching pattern | All |

### CEFR Proficiency Progression

**Lessons 1-2**: A1 (Recognition & Identification)
- Measurable: Learner identifies pwd/ls; explains "this shows my location"
- Cognitive: Remember + Understand
- Constraint: Recognition only, no application

**Lessons 3-7**: A2 (Simple Application with Scaffolding)
- Measurable: Request operations, predict responses, verify results
- Cognitive: Understand + Apply
- Constraint: Scaffolded application to new scenarios

**Lesson 8**: A2→B1 Transition (Independent Problem-Solving)
- Measurable: Troubleshoot real errors through conversation
- Cognitive: Apply + Analyze
- Constraint: Use dialogue pattern to diagnose unfamiliar problems

### Bloom's Taxonomy Alignment

| Level | Lessons | Evidence |
|-------|---------|----------|
| Remember | 1-2 | Identify pwd/ls commands in dialogue |
| Understand | 1-8 | Explain what commands do, predict outputs |
| Apply | 3-8 | Use dialogue pattern for new scenarios |
| Analyze | 8 | Troubleshoot by breaking down problems |

---

## Chapter-Level Architecture

### Prerequisites
- Chapter 6: Claude Code / AI CLI introduction
- Basic computer literacy: files, folders, terminal basics
- Growth mindset: willing to learn through AI collaboration

### Learning Progression
```
Lesson 1: "Where are you?" (pwd, ls understanding)
  ↓
Lesson 2: "How do we stay safe?" (safety dialogue pattern)
  ↓
Lesson 3-4: "Navigate and operate" (cd, cp, mv, rm)
  ↓
Lesson 5-6: "Configure and verify" (export, pip/npm)
  ↓
Lesson 7: "Chain operations" (pipes, complex commands)
  ↓
Lesson 8: "Real project" (integration + troubleshooting)
```

### Cross-Chapter Connections

**Prerequisite**:
- Chapter 6: Claude Code / Gemini CLI basics

**Referenced By**:
- Chapter 8+: Python examples reference "your AI runs: `python -c ...`"
- Part 4+ (Python): Package installation examples use bash in dialogue
- Part 5 (SDD): References "bash in AI's workspace"

---

## Cognitive Load Management

| Lesson | Concepts | Status | Scaffolding |
|--------|----------|--------|-------------|
| 1 | 3 | SAFE | Maximum (foundational) |
| 2 | 4 | SAFE | High (pattern introduction) |
| 3 | 5 | SAFE | High (builds on 1-2) |
| 4 | 5 | SAFE | High (builds on 1-3) |
| 5 | 5 | SAFE | Moderate (new domain) |
| 6 | 4 | SAFE | Moderate (straightforward) |
| 7 | 6 | SAFE | Moderate (complex concept) |
| 8 | 7 | CHALLENGING | Low (capstone, learner-driven) |

**Target**: A1 max 5 concepts, A2 max 7 concepts. All lessons meet or exceed targets through prior lesson scaffolding.

---

## Assessment Strategy

### Formative Assessments (During Lessons)

Each lesson includes in-lesson checks:
- Lesson 1: Predict pwd output
- Lesson 2: Identify 5-step safety pattern
- Lesson 3: Draw folder structure from dialogue
- Lesson 4: Suggest backup before deletion
- Lesson 5: Explain temp vs. persistent
- Lesson 6: Verify package installation
- Lesson 7: Trace data flow through pipe
- Lesson 8: Apply troubleshooting pattern

### Summative Assessments (End of Lessons)

All assessments based on REAL AI interactions (not hypothetical):
- Lesson 1: 3 conversations with AI about location/files
- Lesson 2: Complete safety dialogue (5 steps)
- Lesson 3: Navigate 3 folders + draw structure
- Lesson 4: Backup + operation + verification dialogue
- Lesson 5: Configure variable (temp + permanent)
- Lesson 6: Install real package with verification
- Lesson 7: Build pipeline; modify for different scope
- Lesson 8 (CAPSTONE): Real project setup with error troubleshot

---

## Example Sourcing

**All 30+ Dialogue Examples Must Be**:
- ✅ Sourced from real Claude Code / Gemini CLI interactions OR
- ✅ Clearly marked "to be sourced during implementation"
- ✅ Never fabricated
- ✅ Documented with source information

**Sources Table** (to be completed during implementation):

| Example | Lesson | Type | Source Status |
|---------|--------|------|----------------|
| 1-1 through 1-3 | 1 | pwd/ls dialogue | To be sourced |
| 2-1 through 2-3 | 2 | Safety pattern | To be sourced |
| [Continue for all lessons...] | | | |

---

## Edge Cases & Learner Variations

### OS Differences (macOS vs. Linux vs. Windows)
- Strategy: AI companion handles platform differences in dialogue
- All examples annotate if OS-specific
- Encourage learners to ask: "Does this work on my system?"

### Missing Tools
- Strategy: Agent suggests alternatives or explains fallback
- Example: "find isn't available. Let's use ls -R instead"

### Platform-Specific Commands
- Always verify command exists before using
- Suggest alternatives if unavailable
- Never assume specific tools

---

## Safety Pattern (Recurring Theme)

**Ask → Explain → Understand → Verify → Execute**

Explicitly taught in Lesson 2; reinforced in every subsequent lesson through dialogue.

**Why This Matters**:
- Learners develop HABIT, not just knowledge
- Pattern repetition becomes automatic
- Verification before trust becomes instinct
- Dialogue is natural; not a "lecture"

---

## Validation Checklist

### Technical Validation
- [ ] All dialogue examples sourced or marked "to be sourced"
- [ ] Commands shown are correct for stated OS
- [ ] Output matches actual command output
- [ ] No syntax taught in isolation
- [ ] All examples read naturally (not robotic)

### Pedagogical Validation
- [ ] Learning objectives measurable
- [ ] Cognitive load within A1/A2 limits
- [ ] Safety pattern visible in dialogue
- [ ] Exercises have clear acceptance criteria
- [ ] Assessments based on real interactions

### Accessibility
- [ ] No gatekeeping language
- [ ] Jargon defined on first use
- [ ] Multiple explanation styles
- [ ] Appropriate pacing and visual breaks

---

## Next Steps for Implementation

### Phase 1: Content Creation (content-implementer subagent)
1. Create 8 lesson `.md` files
2. Source all dialogue examples (real interactions)
3. Write exercises with answer keys
4. Design formative + summative assessments
5. Apply output style templates

### Phase 2: Technical Review
1. Verify authenticity of dialogues
2. Test commands (ensure they work)
3. Validate learning objectives
4. Check cognitive load
5. Verify accessibility

### Phase 3: Pedagogical Review
1. Assess flow and progression
2. Evaluate scaffolding
3. Verify constitution alignment
4. Check cross-chapter coherence

### Phase 4: Publication
1. Merge to main branch
2. Build Docusaurus
3. Deploy to live site

---

## Success Criteria

✅ Plan includes:
- All 8 lessons detailed (objectives, concepts, content, examples, exercises, assessments)
- CEFR proficiency progression documented
- Bloom's taxonomy alignment verified
- Cognitive load within beginner tier
- Example sourcing requirements clear
- Edge cases addressed
- Skills mapping complete
- Ready for content-implementer implementation

✅ Implementation will produce:
- 8 authentic dialogue-first lesson files
- 30+ real AI-learner dialogue examples
- Real exercises (not syntax drills)
- Real assessments (not quizzes)
- Chapter README with dialogue-first approach
- No isolated command syntax teaching

---

**Plan Status**: READY FOR IMPLEMENTATION

This detailed plan provides content-implementer with everything needed to create authentic, dialogue-first bash content that teaches AI collaboration, safety-first thinking, and verification habits—not bash command syntax.
