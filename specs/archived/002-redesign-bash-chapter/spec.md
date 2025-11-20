# Feature Specification: Redesign Bash Chapter with Interactive Dialogue Pattern

**Feature Branch**: `002-redesign-bash-chapter`
**Created**: 2025-11-02
**Status**: Draft
**Input**: Proofreader feedback indicated Chapter 7 (Bash Essentials) was causing confusion. Core redesign uses interactive dialogue pattern: `You: [Request] → Agent: Tool Execution → Tool Output → Agent: [Explanation]`

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### Vision Statement

The current Bash chapter teaches commands in isolation, which causes learners to fixate on syntax rather than understanding the **learning pattern** of working with AI. The redesign uses an **interactive dialogue-first approach** where every bash concept is taught through a conversation flow:

1. **You**: "Show me my current directory path"
2. **Agent**: Executes `$ pwd`
3. **Tool Output**: `/Users/mjs/Documents`
4. **Agent**: "You are in `/Users/mjs/Documents` directory"

This pattern teaches learners to:
- Request specific information from their AI companion
- Understand the tool execution happening on their behalf
- Interpret the output and the agent's explanation
- Build mental models of what bash does through natural conversation, not syntax memorization

---

### User Story 1 - Understand File Navigation Through Dialogue (Priority: P1)

**Scenario**: A learner needs to understand where their AI companion is working in the filesystem. Instead of learning `pwd`, `ls`, `cd` commands in isolation, they learn by requesting information in natural language and seeing the agent execute and explain.

**Why P1**: Navigation is foundational to all bash work. If learners don't understand "where am I?" and "what's here?" through natural conversation, they get lost in command syntax and miss the purpose of commands.

**Independent Test**: Learner can have a natural conversation with their AI companion about file location/navigation, understand what commands are being executed and why, and predict outcomes before execution.

**Acceptance Scenarios**:

1. **Given** learner asks "Show me my current directory path", **When** agent executes `pwd` and explains the output, **Then** learner understands their current location without memorizing the `pwd` command
2. **Given** learner asks "What files are in this project?", **When** agent executes `ls -la` and explains what each column means, **Then** learner can interpret the file listing and understand project structure
3. **Given** learner asks "Navigate me to the src folder", **When** agent executes `cd src` and confirms with a new `pwd`, **Then** learner understands the navigation happened and can verify the new location

---

### User Story 2 - Understand File Operations Through Dialogue (Priority: P1)

**Scenario**: Learner needs to copy, move, or delete files. Before executing any operation, they have a conversation with their AI that explains what will happen and why it's safe (or risky).

**Why P1**: File operations are destructive and critical to understand. Learners must develop safety awareness through conversation, not by memorizing flags.

**Independent Test**: Learner can request a file operation from AI, understand the full execution plan, identify safety concerns, and either proceed or ask for modifications.

**Acceptance Scenarios**:

1. **Given** learner asks "Back up my old code before I replace it", **When** agent suggests a series of commands and explains each step, **Then** learner can trace through the plan and verify it's safe
2. **Given** learner asks "Delete the build folder", **When** agent asks clarifying questions and suggests a backup first, **Then** learner learns the safety pattern without being forced
3. **Given** learner asks "Rename my project folder", **When** agent shows the exact commands and what happens, **Then** learner can predict the final state

---

### User Story 3 - Understand Configuration Through Dialogue (Priority: P1)

**Scenario**: Learner needs to set API keys or environment variables. They have a conversation with their AI about what will be set, where it will persist, and how to verify it worked.

**Why P1**: Configuration is where secrets live. Learners must understand the difference between temporary (`export`) and permanent (`~/.bashrc`) through conversation, not by memorizing syntax.

**Independent Test**: Learner can request configuration help from AI, understand the security implications, verify the configuration worked, and know how to troubleshoot if it doesn't.

**Acceptance Scenarios**:

1. **Given** learner asks "Set my API key", **When** agent explains the difference between session and persistent config, **Then** learner can choose the right approach
2. **Given** agent suggests `export API_KEY="..."`, **When** learner asks "Will this persist after I close the terminal?", **Then** agent clearly explains the temporary nature and suggests `~/.bashrc` if needed
3. **Given** configuration is complete, **When** learner asks "How do I verify this worked?", **Then** agent shows the verification command and explains how to read the output

---

### User Story 4 - Understand Package Installation Through Dialogue (Priority: P2)

**Scenario**: Learner asks their AI to install packages. They understand what's being installed, where it goes, and how to verify success through conversation.

**Why P2**: Important for real projects but less critical than navigation/safety/configuration. Learners encounter this frequently with clear AI guidance.

**Independent Test**: Learner can request a package installation, understand what will be installed and where, and verify success through AI-guided verification.

**Acceptance Scenarios**:

1. **Given** learner asks "Install the requests library", **When** agent explains it's a Python package and shows the install command, **Then** learner understands which package manager will be used and why
2. **Given** installation completes, **When** learner asks "Did it work?", **Then** agent shows how to verify with `python -c "import requests"` and explains what the output means
3. **Given** installation fails, **When** learner shows the error, **Then** agent explains the error message and helps troubleshoot

---

### User Story 5 - Understand Process Management Through Dialogue (Priority: P2)

**Scenario**: Learner asks their AI about running processes or background tasks. They understand what's running, how to stop it, and what the consequences are through conversation.

**Why P2**: Important for real development but not needed until learners build running services.

**Independent Test**: Learner can request information about running processes from AI, understand what's happening, and safely manage (stop/restart) processes through guided conversation.

**Acceptance Scenarios**:

1. **Given** learner asks "What's currently running in the background?", **When** agent executes `ps aux` and explains which processes matter, **Then** learner understands system state without analyzing raw output
2. **Given** learner asks "Stop my server", **When** agent identifies the process and explains the kill command, **Then** learner understands what "force kill" means and its implications
3. **Given** a stuck process, **When** learner works with AI to diagnose and fix, **Then** learner learns the troubleshooting pattern through conversation

---

### User Story 6 - Understand Pipes and Chaining Through Dialogue (Priority: P2)

**Scenario**: Learner asks their AI to find or filter data. They understand how pipes connect commands and data flows through the chain by discussing it with the agent, not by memorizing pipe syntax.

**Why P2**: Essential for real problem-solving, but learned easier through guided examples than memorization.

**Independent Test**: Learner can ask their AI to filter/search data, understand the piped commands being executed, and modify the request if needed.

**Acceptance Scenarios**:

1. **Given** learner asks "Count how many Python files are in this folder", **When** agent explains the pipe chain `ls | grep ".py" | wc -l`, **Then** learner understands data flowing through each step
2. **Given** complex filtering task, **When** learner works with AI to build the pipeline step-by-step, **Then** learner learns by doing, not by memorizing syntax
3. **Given** pipe doesn't work as expected, **When** learner discusses it with AI and tries modifications, **Then** learner develops intuition through experimentation

---

### User Story 7 - Learn Safety Culture Through Dialogue (Priority: P1)

**Scenario**: Learner develops a safety habit: before executing any command, they understand what it does and verify it's safe. This is learned through repeated conversations with their AI companion, not through lectures.

**Why P1**: Safety culture is more important than any individual command. Learners must develop instincts through practice, and the chapter must reinforce this in every lesson.

**Independent Test**: Learner demonstrates the safety pattern naturally: when their AI suggests a command, they ask clarifying questions, understand the operation, and decide if it's safe before execution.

**Acceptance Scenarios**:

1. **Given** agent suggests any potentially destructive command, **When** learner immediately asks "Is this safe? Do we need a backup?", **Then** the lesson has succeeded in building habit
2. **Given** command with unclear purpose, **When** learner asks "What does this line do?" instead of blindly executing, **Then** learner has internalized the verify-first pattern
3. **Given** unfamiliar command, **When** learner asks agent to explain before executing, **Then** learner is practicing appropriate skepticism

---

### Edge Cases

- What happens when the learner's system is different from the example (macOS vs. Linux vs. Windows)?
- How does the lesson handle when a command doesn't exist on the learner's system (e.g., missing tools)?
- What if the learner's AI companion suggests a platform-specific command and the learner is on a different OS?
- How does the lesson handle when a destructive operation is necessary and backups aren't feasible?
- What if a command takes a long time to execute (e.g., large file operations)?
- How does the lesson address security concerns when configuration requires secrets?

## Requirements *(mandatory)*

### Functional Requirements (Learning Outcomes)

- **FR-001**: Lessons MUST use the dialogue pattern: `You → Agent → Tool Execution → Tool Output → Agent Explanation` for every bash concept
- **FR-002**: Learners MUST understand file navigation by having conversations with AI about locations and structures, not by memorizing `pwd`, `ls`, `cd` commands in isolation
- **FR-003**: Learners MUST learn file operations (copy, move, delete) through safety-conscious dialogue where AI asks clarifying questions and suggests backups
- **FR-004**: Learners MUST understand configuration (environment variables, API keys) through conversations about temporary vs. persistent changes and verification
- **FR-005**: Learners MUST develop a natural habit of asking "Is this safe?" before executing any potentially destructive operation
- **FR-006**: Learners MUST understand package installation by discussing what will be installed, where, and how to verify, not by memorizing `pip install` syntax
- **FR-007**: Learners MUST understand process management through conversation about what's running and how to safely control processes
- **FR-008**: Learners MUST understand pipes and command chaining by tracing data flow through agent-guided explanations, not by memorizing pipe syntax
- **FR-009**: Learners MUST understand common bash errors through agents explaining root causes and fix approaches, enabling troubleshooting independence
- **FR-010**: Every lesson MUST include concrete examples of real AI-to-human dialogue (from Claude Code, Gemini CLI, or similar), not hypothetical scenarios

### Content Requirements

- **CR-001**: Dialogue MUST be authentic—sourced from real Claude Code or Gemini CLI interactions, not fabricated
- **CR-002**: Every bash command MUST appear in a natural conversation context, not in isolated code blocks teaching syntax
- **CR-003**: Safety concerns MUST be surfaced through agent questions/suggestions, not through separate warning boxes
- **CR-004**: Verification MUST be demonstrated through dialogue (agent shows verification commands and explains output)
- **CR-005**: Error handling MUST be taught through actual error messages and agent-guided explanation, not theoretical cases
- **CR-006**: Lessons MUST explicitly teach the verification habit: understand → verify → execute, not just execution

### Key Entities

- **Dialogue Flow**: The conversation pattern between learner and AI companion (You → Agent → Tool → Explanation)
- **Bash Command**: A shell instruction, meaningful only in the context of what the learner requested and why the agent is executing it
- **Safety Pattern**: The learner's developing instinct to understand and verify before executing, especially for destructive operations
- **Verification**: The steps (usually shown in dialogue) to confirm a command did what was intended
- **Tool Output**: The result of a bash command, interpreted and explained by the agent for the learner's understanding

## Success Criteria *(mandatory)*

### Measurable Learning Outcomes

- **SC-001**: Learner can request any file operation through natural language and understand the resulting commands without syntax training
- **SC-002**: When learner sees a new bash command, they can ask their AI "What does this do?" and understand the explanation in under 30 seconds without looking up documentation
- **SC-003**: Learner demonstrates safety habit consistently: asks for explanation before executing any unfamiliar command
- **SC-004**: Learner can complete a complex task (setup project, configure API, install packages) by requesting help from AI and understanding each step
- **SC-005**: Learner's confidence in "understanding what my AI is doing" increases from baseline (Chapter 6) to 4+/5 by chapter end
- **SC-006**: Learner can troubleshoot a bash error by reading the error message, asking their AI for explanation, and applying the fix independently
- **SC-007**: 90%+ of learners report that the dialogue-first approach helped them understand bash concepts better than traditional syntax-first teaching

### Quality Metrics

- **QM-001**: All dialogue examples are sourced from real Claude Code / Gemini CLI interactions, with source documented
- **QM-002**: Zero dialogue examples are fabricated or hypothetical—all are verified real interactions
- **QM-003**: Every bash command appears in at least one complete dialogue example, never in isolation
- **QM-004**: Safety concerns are surfaced naturally in dialogue (agent asks questions), not in separate warning blocks
- **QM-005**: Lesson progression builds confidence: early lessons use simple commands (pwd, ls), later lessons use complex pipelines
- **QM-006**: Each lesson includes verification examples showing how to confirm a command worked

---

## Assumptions

- **A1**: Learners have completed Chapter 6 (Claude Code or Gemini CLI) and understand they're working with an AI companion
- **A2**: Learners have a terminal open and one of the recommended AI tools (Claude Code, Gemini CLI, ChatGPT Code Interpreter) available
- **A3**: All dialogue examples will be gathered from real interactions with these tools; if not available, lessons will note "example only" clearly
- **A4**: Platform differences (macOS/Linux/Windows) will be addressed in dialogue—AI companion should handle these naturally
- **A5**: Learners will have basic computer literacy (open terminal, understand files/folders) from earlier chapters
- **A6**: The chapter focuses on understanding, not mastery; learners don't need to memorize any bash commands (can ask AI anytime)

---

## In-Scope vs. Out-of-Scope

### In Scope

- Teaching through AI-learner dialogue patterns
- Understanding bash concepts through conversation, not memorization
- Safety culture and verification habits
- Real-world examples from Claude Code / Gemini CLI
- Troubleshooting and error interpretation
- Working with actual AI companions as concrete examples

### Out of Scope

- Writing bash scripts from scratch
- Advanced bash programming (functions, arrays, loops, conditionals)
- Bash internals, performance optimization, or theory
- System administration advanced topics
- Shell scripting education (not dialogue-based)
- Teaching commands in isolation without AI context

---

## Complexity Tier

**Tier 1: Beginner** (Part 2, Chapter 7)

- Target audience: Learners with AI companion experience, no terminal experience
- Cognitive load: Max 2 options per lesson (AI handles complexity)
- Teaching approach: Dialogue-first, dialogue-only (never syntax-first)
- Safety: Every lesson reinforces safety habits through natural conversation
- Skills: Understanding bash through collaboration, not technical command syntax

---

## Prerequisites

- **Chapter 6**: Claude Code (or alternative AI CLI tools) — learners should understand they're working with an AI companion
- **Basic computer literacy**: Can open terminal, understand files and folders
- **Growth mindset**: Willing to learn through conversation with AI, not through memorization

---

## Chapter Structure: 8 Lessons, 4-5 Hours Total

### **Lesson 1: Introducing Your AI Companion's Workspace** (35 min)

- **Dialogue Focus**: "Where are you working?" and "What files can you see?"
- **AI Pattern**: Learner requests location info → Agent executes `pwd`, `ls` → Learner understands workspace
- **Key Concept**: AI has a "current location" and can see files; learner supervises from outside the terminal
- **Practice**: Have 3 natural conversations with AI about location/files
- **Assessment**: Learner can predict what an AI companion sees before it shows them

---

### **Lesson 2: The Safety-First Dialogue Pattern** (35 min)

- **Dialogue Focus**: Before executing anything, understand it completely
- **AI Pattern**: Learner requests operation → Agent explains step-by-step → Learner asks "Is this safe?" → Agent confirms or suggests backup
- **Key Concept**: Every bash interaction follows: Ask → Understand → Verify → Execute → Learn
- **Practice**: Work through 3 guided scenarios, practicing the safety pattern
- **Assessment**: Learner applies safety pattern to a new task without prompting

---

### **Lesson 3: Understanding File Navigation Through Dialogue** (40 min)

- **Dialogue Focus**: "Navigate to X" and "Show me what's there"
- **AI Pattern**: Learner makes location request → Agent navigates and explains → Learner understands paths
- **Key Concept**: Paths are just descriptions of where you are; both absolute and relative paths have natural conversation analogues
- **Practice**: Ask AI to navigate to different folders, predict what you'll see, verify
- **Assessment**: Learner can draw file structure from AI navigation dialogue without looking at raw output

---

### **Lesson 4: Understanding File Operations Safely** (45 min)

- **Dialogue Focus**: "Back up my code," "Copy this folder," "Delete old files" with safety questions
- **AI Pattern**: Learner requests operation → Agent asks clarifying questions → Learner and agent agree on backup strategy → Execute safely
- **Key Concept**: Safety is built into conversation, not an afterthought; good AI companions ask before destroying
- **Practice**: Review AI-suggested file operations, identify concerns, ask clarifying questions
- **Assessment**: Learner identifies potential data loss and knows to ask "Should we backup first?"

---

### **Lesson 5: Understanding Configuration and Secrets** (40 min)

- **Dialogue Focus**: "Set my API key" and "How do I know it worked?"
- **AI Pattern**: Learner requests config → Agent explains temp vs. permanent → Learner chooses approach → Learner verifies
- **Key Concept**: Configuration is temporary (`export`) or persistent (`~/.bashrc`); secrets must never be hardcoded; verification is always needed
- **Practice**: Set an API key with AI, verify it works, understand persistence
- **Assessment**: Learner configures a new tool with AI and can verify success without being told how

---

### **Lesson 6: Understanding Dependencies and Packages** (40 min)

- **Dialogue Focus**: "Install X library" with understanding of what's being installed and where
- **AI Pattern**: Learner requests package → Agent explains what will be installed → Learner verifies success
- **Key Concept**: Different package managers (pip, npm, brew, apt); AI knows which to use; verification matters
- **Practice**: Ask AI to install a package, verify it worked, understand what happened
- **Assessment**: Learner can read package installation dialogue and explain what will be added to their system

---

### **Lesson 7: Understanding Pipes and Complex Commands** (40 min)

- **Dialogue Focus**: "Count Python files," "Find files with 'error'" through natural conversation
- **AI Pattern**: Learner makes request → Agent builds pipeline → Learner traces data flow → Learner understands the result
- **Key Concept**: Pipes chain commands; data flows through each step; understanding data flow is the key, not memorizing pipe syntax
- **Practice**: Request filtering tasks, trace how AI builds the pipeline, modify requests based on output
- **Assessment**: Learner can trace a piped command and predict output before execution

---

### **Lesson 8: Working with Real AI Tools and Troubleshooting** (50 min)

- **Dialogue Focus**: Real project setup with real errors and troubleshooting dialogue
- **AI Pattern**: Real project setup task → Errors occur → Learner and AI troubleshoot together → Learner learns pattern
- **Key Concept**: Real work involves errors; AI explains error messages; troubleshooting is collaboration
- **Practice**: Set up a real project with Claude Code or Gemini CLI; encounter and fix real errors
- **Assessment**: Learner completes multi-step project setup, asking AI for help at each step; troubleshoots real errors

---

## Redesign Approach: Key Differences from Current Chapter

| Aspect | Current (Problematic) | Redesigned (Dialogue-First) |
|--------|----------------------|---------------------------|
| **Structure** | Commands → Explanation | Learner Request → Agent Execution → Output → Agent Explanation |
| **Focus** | Bash syntax and command options | Understanding what AI is doing on learner's behalf |
| **Safety** | Warning box at lesson end | Surfaced naturally through agent questions in dialogue |
| **Verification** | Separate "how to verify" section | Built into dialogue flow (agent shows and explains verification) |
| **Practice** | Hypothetical scenarios | Real AI-learner conversations from Claude Code / Gemini CLI |
| **Learning Pattern** | Command isolation + syntax rules | Natural conversation → Understanding → Verification → Execution |
| **Error Handling** | Theoretical error message examples | Real errors from actual AI interactions, explained through dialogue |
| **Assessment** | "Can you recall the -la flag?" | "Can you have a safety-conscious conversation about this operation?" |

---

## Definition of Done

- [ ] All 8 lesson files created using dialogue-first structure
- [ ] Every bash command appears in at least one complete dialogue example (not isolated code blocks)
- [ ] All dialogue examples sourced from real Claude Code / Gemini CLI interactions (documented)
- [ ] All lessons follow the pattern: `You → Agent → Tool → Explanation`
- [ ] Safety concerns surfaced through agent questions, not warning boxes
- [ ] Verification steps demonstrated through dialogue, not as separate sections
- [ ] No bash syntax taught in isolation; all syntax introduced in conversation context
- [ ] Lessons progress from simple (pwd, ls) to complex (pipes, troubleshooting)
- [ ] Chapter includes at least 3 complete worked examples from real AI interactions
- [ ] Assessment questions test dialogue skills, not command memorization
- [ ] Technical reviewer validates pedagogy and authenticity of dialogue examples

---

## Deliverables

1. **Lesson 1**: `01-introducing-ai-workspace.md` — "Where are you working?"
2. **Lesson 2**: `02-safety-first-pattern.md` — "Ask → Understand → Verify → Execute"
3. **Lesson 3**: `03-navigation-dialogue.md` — "Navigate and understand file structure"
4. **Lesson 4**: `04-file-operations-safely.md` — "Copy, move, delete with safety questions"
5. **Lesson 5**: `05-configuration-secrets.md` — "Set API keys and verify configuration"
6. **Lesson 6**: `06-packages-dependencies.md` — "Install and verify packages"
7. **Lesson 7**: `07-pipes-complex-commands.md` — "Chain commands through dialogue"
8. **Lesson 8**: `08-real-project-troubleshooting.md` — "Real project setup and error handling"
9. **Chapter README**: `README.md` — Updated with new dialogue-first approach

---

## Next Steps

1. **Specification Review & Approval** — Get feedback on dialogue-first approach
2. **Plan Phase** (`/sp.plan`) — Chapter-planner breaks spec into detailed lesson plans
3. **Implementation** (`content-implementer`) — Write each lesson following dialogue-first pattern
4. **Validation** (`validation-auditor`) — Verify authenticity of examples and pedagogical soundness
5. **Integration** — Replace existing Chapter 7 with redesigned version
6. **Publish** — Deploy to Docusaurus
