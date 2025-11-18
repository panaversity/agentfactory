# Chapter 7 Specification: Understanding Your AI Companion's Bash Usage

**Chapter ID**: `007-ai-bash-companion`
**Title**: "Bash for Developers: How Your AI Companion Works With the Command Line"
**Branch**: `07-bash-revision`
**Created**: 2025-10-31
**Status**: Specification Ready for Planning

---

## Vision Statement

This chapter **does NOT teach bash** as a traditional skill. Instead, it teaches readers to **understand and collaborate with their AI companion** as it uses bash on their behalf.

**Core Learning Goal**: "When your AI companion suggests a bash command, you understand WHAT it's doing, WHY it matters, and whether it's safe to execute."

**Paradigm Shift**:
- **Not**: "Memorize 100 bash commands"
- **YES**: "Understand the 8-10 bash patterns your AI uses most frequently"
- **Not**: "Learn to write complex bash"
- **YES**: "Learn to ask clear questions and verify AI's responses"
- **Not**: "Bash is hard, technical knowledge"
- **YES**: "Bash is your AI companion's interface; you're the quality checker"

---

## User Scenarios & Testing

### User Story 1: Understanding File Navigation (Priority: P1)

**Scenario**: A learner receives AI suggestions that involve `cd`, `pwd`, `ls` commands. They need to understand what the AI is doing with the filesystem.

**Why P1**: Navigation is foundational—every bash interaction involves knowing where you are. If learners don't understand this, they'll fear breaking things.

**Independent Test**: Learner can read an AI-generated bash command involving file navigation, explain what it does and where on disk it operates, and predict the outcome before execution.

**Acceptance Scenarios**:
1. **Given** AI suggests `cd ~/project && ls -la`, **When** learner reads it, **Then** learner can explain: "Move to home/project folder and list files with details"
2. **Given** AI suggests `mkdir -p src/{tests,models,utils}`, **When** learner reads it, **Then** learner can draw the resulting folder structure
3. **Given** AI suggests `rm -rf old_project`, **When** learner considers it, **Then** learner can identify this is destructive and asks for confirmation

---

### User Story 2: Understanding Configuration and Environment (Priority: P1)

**Scenario**: Learner needs to set API keys, configure tools, or set environment variables. AI companion suggests commands like `export API_KEY="..."` or edits to `~/.bashrc`.

**Why P1**: Modern development requires configuration. Learners must understand how their system is configured and how to verify AI's configuration changes.

**Independent Test**: Learner can read AI-generated configuration commands, understand what system variable is being set, explain where it persists, and verify it was applied correctly.

**Acceptance Scenarios**:
1. **Given** AI suggests `export ANTHROPIC_API_KEY="sk-..."`, **When** learner executes it, **Then** learner can verify it's set with `echo $ANTHROPIC_API_KEY`
2. **Given** AI edits `~/.bashrc` to add a new export, **When** learner views the file, **Then** learner can identify the new line and understand it persists across sessions
3. **Given** configuration needs to be changed, **When** learner works with AI, **Then** learner can decide between temporary (export) vs. permanent (~/.bashrc) solution

---

### User Story 3: Understanding File Operations and Safety (Priority: P1)

**Scenario**: AI suggests operations like copy, move, delete. Learner must understand what data moves where and identify destructive operations before they happen.

**Why P1**: Data loss prevention is critical. Learners must develop safety instincts and understand backup patterns that AI suggests.

**Independent Test**: Learner can read copy/move/delete commands from AI, trace what data is affected, identify if backups are suggested, and decide if the operation is safe before executing.

**Acceptance Scenarios**:
1. **Given** AI suggests `cp old_file.py backup_file.py`, **When** learner reads it, **Then** learner understands: creates copy, original unchanged
2. **Given** AI suggests `mv src/ old_src/ && mkdir src/`, **When** learner analyzes it, **Then** learner recognizes this as "rename src to old_src, create new empty src"
3. **Given** AI suggests destructive operation, **When** learner reviews the plan, **Then** learner asks "Should we backup first?" and AI adjusts the plan

---

### User Story 4: Understanding Package Installation and Dependencies (Priority: P2)

**Scenario**: AI companion suggests installing packages with `pip install`, `npm install`, or `brew install`. Learner must understand what's being installed, where it goes, and how to verify success.

**Why P2**: Essential for real projects but slightly less critical than navigation/configuration/safety. Learners will encounter this frequently but with clear AI guidance.

**Independent Test**: Learner can read package installation commands, understand what dependency is being added, where it's installed, and verify success with appropriate check commands.

**Acceptance Scenarios**:
1. **Given** AI suggests `pip install requests`, **When** learner executes it, **Then** learner can verify with `python -m pip list | grep requests`
2. **Given** AI suggests `npm install --save express`, **When** learner reviews it, **Then** learner understands this adds to package.json and installs to node_modules
3. **Given** installation fails, **When** learner works with AI, **Then** learner understands the error message and AI's debugging approach

---

### User Story 5: Understanding Process Management and Monitoring (Priority: P2)

**Scenario**: AI suggests commands to check running processes, manage background tasks, or stop services. Learner must understand what's running and how to safely manage it.

**Why P2**: Important for real development (servers, services) but not needed until later chapters when learners build running applications.

**Independent Test**: Learner can read process management commands from AI, understand what's running or what will change, and decide if it's safe to execute.

**Acceptance Scenarios**:
1. **Given** AI suggests `ps aux | grep python`, **When** learner reviews it, **Then** learner understands: "show running processes, filter for python"
2. **Given** a server is running, **When** AI suggests `kill -9 <PID>`, **Then** learner understands this force-terminates that process
3. **Given** background process blocking work, **When** AI helps diagnose, **Then** learner learns the troubleshooting pattern

---

### User Story 6: Understanding Pipes and Command Chaining (Priority: P2)

**Scenario**: AI suggests complex commands combining multiple bash tools: `cat file.txt | grep "error" | wc -l`. Learner must understand the flow of data and purpose of each command.

**Why P2**: Essential for real problem-solving but easier to learn through guided examples than by memorizing.

**Independent Test**: Learner can read piped commands from AI, trace how output flows through each step, and explain the final result.

**Acceptance Scenarios**:
1. **Given** AI suggests `ls -la | grep ".py" | wc -l`, **When** learner reads it, **Then** learner can trace: "list → filter → count" and predict it shows Python file count
2. **Given** complex pipeline from AI, **When** learner reviews it with AI, **Then** learner asks "What does each part do?" and AI explains step-by-step
3. **Given** need to filter/search files, **When** learner works with AI, **Then** learner requests explanation of the pattern before executing

---

### User Story 7: Understanding Error Messages and Troubleshooting (Priority: P3)

**Scenario**: Command fails with error. AI interprets the error, suggests fixes. Learner must understand what went wrong and why the fix works.

**Why P3**: Critical for real work but learners will develop this through repeated exposure. Can be scaffolded from earlier stories.

**Independent Test**: Learner encounters command error from AI, reads AI's explanation, understands the root cause, and verifies the fix works.

**Acceptance Scenarios**:
1. **Given** "command not found" error, **When** AI suggests checking PATH or installing, **Then** learner understands why command failed
2. **Given** "permission denied" error, **When** AI suggests chmod, **Then** learner understands file permissions caused the issue
3. **Given** any bash error, **When** learner reads AI's explanation, **Then** learner can identify the root cause

---

### Edge Cases

- What if learner's AI suggests a command that's different from their OS (e.g., macOS vs. Linux)?
- How does learner verify AI's suggestions are following security best practices (no hardcoded secrets)?
- What if learner doesn't have admin privileges (sudo) for an operation AI suggests?
- How does learner recognize when AI's bash command might cause data loss?
- What happens when API key configuration is wrong and how does learner debug it?

---

## Requirements

### Functional Requirements (Learning Outcomes)

- **FR-001**: Learner MUST understand file system navigation by reading AI-generated navigation commands
- **FR-002**: Learner MUST be able to read and interpret `mkdir`, `touch`, `cp`, `mv`, `rm` commands from AI
- **FR-003**: Learner MUST understand environment variables and how to set/verify them
- **FR-004**: Learner MUST recognize safe vs. destructive operations and ask questions before executing risky commands
- **FR-005**: Learner MUST understand how to verify package installations and check system configuration
- **FR-006**: Learner MUST be able to read and trace piped commands to understand data flow
- **FR-007**: Learner MUST understand common bash error messages and work with AI to debug them
- **FR-008**: Learner MUST understand when to trust AI's bash suggestions vs. when to ask for explanation
- **FR-009**: Learner MUST practice the "AI Collaboration Pattern": Ask → Understand → Verify → Execute → Learn

### Content Requirements

- **CR-001**: Each lesson MUST show real bash commands generated BY AI, not just theoretical examples
- **CR-002**: Each lesson MUST follow the pattern: AI suggests → Learner analyzes → Learner questions → AI explains → Learner executes
- **CR-003**: Lessons MUST reference actual CLI tools (Claude Code, Gemini CLI) as concrete examples of AI companions
- **CR-004**: Every destructive command MUST be framed as a safety question: "Is it safe to execute?"
- **CR-005**: Lessons MUST avoid teaching bash syntax in isolation; all syntax explained in context of AI collaboration
- **CR-006**: Lessons MUST include screenshots/examples of real AI → bash interactions from Claude Code or Gemini CLI
- **CR-007**: Each lesson MUST have a "Questions to Ask Your AI" section to guide learner-AI dialogue

### Key Entities

- **AI Companion**: Claude Code, Gemini CLI, or other agentic AI tools that execute bash on the user's behalf
- **Bash Command**: A shell instruction that the AI may suggest or execute
- **Safety Pattern**: Learner's mental model for evaluating if a command is safe before execution
- **Configuration State**: Environment variables, shell config files, installed packages—what the learner's system looks like
- **Trust Markers**: Indicators that an AI-suggested command is safe (explanation provided, backup suggested, non-destructive, etc.)

---

## Chapter Structure: 8 Lessons, 4-5 Hours Total

### **Lesson 1: Why Bash? Your AI Companion's Native Language** (35 min)
- **Learning Outcome**: Understand why AI tools use bash and why you need to understand what they're doing
- **Key Concept**: Bash is the AI's "interface" to your computer; you're learning to supervise and collaborate, not command
- **Practice**: Open terminal, ask AI "where am I?" and "what's in this folder?", understand `pwd` and `ls` by watching AI use them
- **Assessment**: Can explain why bash matters without memorizing any commands

### **Lesson 2: The AI Collaboration Pattern** (35 min)
- **Learning Outcome**: Learn the repeatable pattern for working with AI on ANY bash task
- **Key Concept**: Ask → Understand → Verify → Execute → Learn (5-step framework)
- **Practice**: Work through 3 guided scenarios with AI, applying the pattern each time
- **Assessment**: Can apply the pattern to a new bash task with AI guidance

### **Lesson 3: Understanding File Navigation and Paths** (40 min)
- **Learning Outcome**: Understand file system concepts that underlie all bash work
- **Key Concept**: Absolute vs. relative paths, home directory, current directory, moving safely
- **Practice**: Ask AI to navigate to different folders, analyze the commands, predict outcomes
- **Assessment**: Can read AI-generated navigation commands and draw the resulting file structure

### **Lesson 4: Understanding File Operations (Copy, Move, Delete, Create)** (45 min)
- **Learning Outcome**: Understand what happens to data when AI suggests file operations
- **Key Concept**: Safety-first approach; understanding destructive operations; backup patterns
- **Practice**: Review AI-suggested file operations, identify safety concerns, ask questions
- **Assessment**: Can recognize risky operations and know when to ask "Should we backup first?"

### **Lesson 5: Understanding Configuration and Environment** (40 min)
- **Learning Outcome**: Understand how AI configures your system (API keys, variables, settings)
- **Key Concept**: Temporary vs. permanent configuration; security (no hardcoded secrets); verification
- **Practice**: Set an API key with AI, verify it's working, understand where it's stored
- **Assessment**: Can configure a new tool with AI and verify the configuration is correct

### **Lesson 6: Understanding Packages and Dependencies** (40 min)
- **Learning Outcome**: Understand what AI is doing when it installs packages
- **Key Concept**: Different package managers (pip, npm, brew, apt); dependency trees; verification
- **Practice**: Ask AI to install a package, watch it happen, verify success
- **Assessment**: Can read package installation commands and know what was added to their system

### **Lesson 7: Understanding Pipes and Complex Commands** (40 min)
- **Learning Outcome**: Understand how AI chains commands to accomplish complex tasks
- **Key Concept**: Pipes, data flow, filtering, reducing (grep, wc, sort)
- **Practice**: Read AI-suggested pipes, trace the data, understand each step
- **Assessment**: Can trace a piped command and predict its output before execution

### **Lesson 8: Try With AI - Real Project Setup and Troubleshooting** (50 min)
- **Learning Outcome**: Synthesize all lessons; work with actual AI companion on real task
- **Key Concept**: Complete project setup from scratch; handle real errors; learn from troubleshooting
- **Practice**: Set up a real Python project with Claude Code or Gemini CLI; troubleshoot real errors
- **Assessment**: Can complete a multi-step project setup, asking AI for explanation at each step; troubleshoot when errors occur

---

## Success Criteria

### Measurable Learning Outcomes

- **SC-001**: Learner can read 10 different bash commands suggested by AI and explain what each does WITHOUT memorizing commands
- **SC-002**: Learner can identify at least 3 safety concerns in AI-suggested commands and ask appropriate questions
- **SC-003**: Learner completes a full project setup (create folders, set environment variables, install packages) working with AI
- **SC-004**: Learner encounters a bash error, interprets it with AI help, and fixes it independently
- **SC-005**: Learner rates their confidence in "understanding what my AI companion is doing" at 4+/5 (start-of-chapter baseline ~1/5)
- **SC-006**: Learner can apply the AI Collaboration Pattern to a completely new bash task not covered in lessons

### Quality Metrics

- **QM-001**: All code examples are REAL AI outputs (from Claude Code or Gemini CLI), not hypothetical
- **QM-002**: Zero standalone bash syntax lessons; all syntax introduced IN CONTEXT of a task
- **QM-003**: Safety culture reinforced in every lesson (backup before destructive operations, understand before executing)
- **QM-004**: AI-first pedagogy: every lesson emphasizes collaboration, not memorization
- **QM-005**: AI tool examples updated quarterly to reflect current Claude Code / Gemini CLI behavior

---

## In-Scope vs. Out-of-Scope

### In Scope
- Understanding bash through AI collaboration
- Reading and verifying AI-suggested commands
- Safety patterns for file operations
- Configuration and environment setup
- Installing and verifying packages
- Troubleshooting common errors
- Working with Claude Code / Gemini CLI as concrete examples

### Out of Scope
- Writing bash scripts from scratch
- Advanced bash programming (functions, arrays, conditionals)
- Bash internals or theory
- System administration tasks
- Performance optimization
- Bash syntax memorization (opposite of the point!)
- Teaching bash as a standalone skill

---

## Prerequisites

- **Chapter 5**: Claude Code (or Chapter 6: Gemini CLI) — students should be familiar with their AI companion
- **Basic computer literacy**: Can open terminal, navigate menus, understand files/folders
- **Growth mindset**: Willingness to learn by doing with AI, not by memorization

---

## Constraints & Non-Goals

### Constraints
- Must use actual AI CLI tools (Claude Code, Gemini CLI) for all examples, not hypothetical
- All code examples must be VERIFIED to work with current tool versions
- Safety emphasis non-negotiable; never suggest risky commands without explanation
- Platform-specific guidance required (macOS vs. Linux vs. Windows)

### Non-Goals
- Teaching bash as a standalone programming language
- Replacing practical hands-on learning with video/passive content
- Creating a reference manual (learners can Google for that)
- Teaching every bash command (opposite of philosophy!)

---

## Domain Skills Application

This chapter MUST apply these domain skills from `.claude/skills/`:

1. **learning-objectives** — Define what learners can DO after each lesson (not what they know)
2. **concept-scaffolding** — Break understanding of bash patterns into progressive steps
3. **exercise-designer** — Exercises must involve reading/analyzing AI commands, not memorizing
4. **assessment-builder** — Quizzes test understanding of AI collaboration, not bash trivia
5. **technical-clarity** — Explain technical concepts clearly for beginners unfamiliar with terminals
6. **book-scaffolding** — Scaffold from "supervise AI" to "collaborate confidently" across 8 lessons
7. **ai-augmented-teaching** — Model learning WITH AI, not just using AI; show the thinking process
8. **code-example-generator** — All code examples are REAL AI outputs, not fabricated

---

## Definition of Done

- [ ] All 8 lesson files created and written to `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/`
- [ ] Each lesson follows the shared lesson structure from `.claude/output-styles/lesson.md`
- [ ] All learning objectives defined using learning-objectives skill
- [ ] All code examples are real outputs from Claude Code or Gemini CLI
- [ ] All exercises follow exercise-designer patterns (analyzing vs. memorizing)
- [ ] All assessments test understanding of AI collaboration, not bash syntax
- [ ] Safety warnings included in Lessons 3-4 and reinforced in Lesson 8
- [ ] Technical reviewer validates all content for accuracy and pedagogy
- [ ] README.md updated with new lesson structure and learning path
- [ ] No bash syntax taught in isolation; all syntax contextual to tasks
- [ ] All lessons demonstrate AI-first paradigm (learner supervises, AI executes)

---

## Deliverables

1. **08-intro.md** — Chapter introduction (rewritten for new paradigm)
2. **01-why-bash-ai-language.md** — Lesson 1
3. **02-ai-collaboration-pattern.md** — Lesson 2
4. **03-understanding-navigation.md** — Lesson 3
5. **04-understanding-file-operations.md** — Lesson 4
6. **05-understanding-configuration.md** — Lesson 5
7. **06-understanding-packages.md** — Lesson 6
8. **07-understanding-pipes.md** — Lesson 7
9. **08-capstone-real-workflow.md** — Lesson 8
10. **README.md** — Updated with new structure and learning path

---

## Next Steps

1. **Plan Phase** (`/sp.plan`): Use chapter-planner agent to create detailed lesson plans
2. **Write Phase**: Use content-implementer agent for each lesson; **I am responsible for writing outputs to files**
3. **Validate Phase**: Use validation-auditor agent to validate pedagogy and technical accuracy
4. **Integration**: Update book index and cross-references
5. **Publish**: Deploy to Docusaurus

