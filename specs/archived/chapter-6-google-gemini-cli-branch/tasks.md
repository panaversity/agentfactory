# Chapter 6: Google Gemini CLI — Implementation Tasks

**Chapter Type**: Technical/Intermediate (Part 2)
**Status**: Ready for Development
**Feature Branch**: feature/chapter-6-google-gemini-cli
**Owner**: [To be assigned]
**Estimated Total Effort**: 80-120 hours (16-24 story points @ 5h per point)

---

## Overview

This task checklist breaks Chapter 6 enhancement into specific, testable development tasks. The plan spans 5 lessons with progressive complexity, covering 35 functional requirements and ~170 minutes of content.

**Success Criteria**:
- All 5 lessons implemented with specified enhancements
- All 35 functional requirements covered
- 90%+ command recall (FR knowledge checks)
- 85%+ configuration proficiency (settings.json exercises)
- 80%+ memory system mastery (GEMINI.md creation)
- 75%+ MCP server installation success
- 70%+ custom command creation success
- All code examples tested and working
- All exercises with model solutions
- Total reading time: ~100 min (core), ~175 min (full chapter)

---

## Lesson 1: Why Gemini CLI Matters (Enhancement Phase)

### Task 1.1: MUST — Review & Enhance Lesson 1 Narrative

**Description**: Review existing 186-line lesson; identify enhancement points; add power user framing

**Acceptance Criteria**:
- [ ] Current lesson content preserved (100%)
- [ ] "Power User Advantage" section added (~200 words) explaining 10+ commands as decision factor
- [ ] "For the North Star Developer" section added (~150 words) reframing for sophisticated users
- [ ] New content emphasizes: speed, context management, integration, tool ownership
- [ ] All 5 new concepts listed and explained

**Reference**:
- Current file: `/book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/01-why-gemini-cli-matters.md`
- Plan section: LESSON 1 Enhancement

**Effort**: 2-3 hours

**Acceptance**: Narrative approved by human reviewer; reads naturally; ~250 words added; all new content integrated

---

### Task 1.2: MUST — Update "Try With AI" Prompts (Lesson 1)

**Description**: Keep existing 4 prompts, add NEW Prompt 4

**Acceptance Criteria**:
- [ ] Prompts 1-3: Unchanged
- [ ] Prompt 4 NEW: "What Features Would Make Me Choose Gemini?"
  - Personalized for developer role
  - Focuses on: custom commands, memory systems, MCP integration
  - Expected outcome: Personalized capability roadmap
- [ ] All prompts tested in Gemini CLI
- [ ] Outcomes match expected outputs

**Effort**: 1-2 hours

**Acceptance**: Prompt 4 added; tested; provides personalized recommendations

---

## Lesson 2: Core Commands, Syntax & Workflow Mastery (MAJOR EXPANSION)

### Task 2.1: MUST — Create Comprehensive Command Reference Section

**Description**: Create "The 13 Essential Commands" section with reference table, business scenarios, examples

**Acceptance Criteria**:
- [ ] Command reference table created with columns:
  - Command name
  - Purpose (1-2 sentence)
  - Example usage
  - Common business use
- [ ] 13+ commands documented:
  - /help, /clear, /tools, /mcp, /stats, /compress, /copy, /memory
  - /chat, /restore, /settings, /init, /theme, /ide
- [ ] Each command has realistic business scenario
- [ ] Table is readable and scannable
- [ ] ~800 word section matches plan specification

**Reference**:
- Plan section: LESSON 2, "The 13 Essential Commands"
- FRs: FR-001

**Effort**: 4-5 hours

**Acceptance**: Table complete; all 13 commands documented; examples tested; ~800 words

---

### Task 2.2: MUST — Document @ Syntax Section

**Description**: Create section explaining @ context reference syntax

**Acceptance Criteria**:
- [ ] What @ does explained clearly (references files/folders/URLs without copying)
- [ ] Format documented: @path/to/file, @folder/, @https://example.com
- [ ] When to use @ (large files, binary data, external URLs)
- [ ] Advantages over copy-paste explained (privacy, rate limits, dynamic updates)
- [ ] Real example: @./docs/ analyze API documentation
- [ ] Comparison: @ vs. copy-paste vs. /mcp servers
- [ ] Business workflow impact noted (~600 words)

**Reference**:
- Plan section: LESSON 2, "The @ Syntax"
- FR: FR-003

**Effort**: 3-4 hours

**Acceptance**: Section complete; syntax clear; examples working; advantages articulated

---

### Task 2.3: MUST — Enhance ! Shell Syntax Section

**Description**: Expand existing shell mode explanation with advanced usage

**Acceptance Criteria**:
- [ ] Current shell mode explanation kept
- [ ] Expanded format: Type !, exit with ESC
- [ ] Real workflows documented: running tests, git status, file operations
- [ ] When to use ! vs. asking Gemini to run commands
- [ ] Error handling: How Gemini interprets shell output
- [ ] Security considerations: destructive commands, permissions
- [ ] ~400 word section

**Reference**:
- Plan section: LESSON 2, "The ! Syntax"
- FR: FR-004

**Effort**: 2-3 hours

**Acceptance**: Shell syntax section enhanced; workflows realistic; security notes clear

---

### Task 2.4: MUST — Create Keyboard Shortcuts Section

**Description**: Document 4 essential keyboard shortcuts with practice scenarios

**Acceptance Criteria**:
- [ ] All 4 shortcuts documented:
  - Ctrl+L — Clear screen (keeps history)
  - Ctrl+V — Paste multi-line input
  - Ctrl+Y — Scroll history forward
  - Ctrl+X — Scroll history backward
- [ ] Each shortcut has business use case
- [ ] Time savings noted ("saves 10+ seconds per interaction")
- [ ] Section ~300 words
- [ ] Mnemonics or memory aids provided

**Reference**:
- Plan section: LESSON 2, "Keyboard Shortcuts"
- FR: FR-005

**Effort**: 1-2 hours

**Acceptance**: All 4 shortcuts documented; examples clear; business value articulated

---

### Task 2.5: MUST — Create Command-Line Invocation Modes Section

**Description**: Document all CLI invocation modes with business scenarios

**Acceptance Criteria**:
- [ ] 6 modes documented with examples:
  - Interactive: `gemini` (default)
  - Piped input: `echo "..." | gemini -p`
  - Message mode: `gemini -m "Fix this code"`
  - Sandbox mode: `gemini --sandbox`
  - Yolo mode: `gemini --yolo`
  - Checkpointing: `gemini --checkpointing`
- [ ] Business scenarios for each mode
- [ ] Decision framework for mode selection
- [ ] Automation and CI/CD integration examples
- [ ] ~500 word section

**Reference**:
- Plan section: LESSON 2, "Command-Line Invocation Modes"
- FR: FR-008

**Effort**: 3-4 hours

**Acceptance**: All 6 modes documented; examples tested; decision framework clear

---

### Task 2.6: MUST — Create GEMINI.md Hierarchical Memory Introduction

**Description**: First introduction to GEMINI.md; 5-level hierarchy concept; project setup

**Acceptance Criteria**:
- [ ] GEMINI.md concept explained (file-based context, not commands)
- [ ] 5-level hierarchy documented:
  - Global: ~/.gemini/GEMINI.md
  - Project: ./GEMINI.md
  - Subdirectory: ./src/GEMINI.md
  - Feature branch: ./.git/gemini/feature-x.md
  - Module-specific: ./modules/auth/GEMINI.md
- [ ] Creating project GEMINI.md: /init command walkthrough
- [ ] Example GEMINI.md structure (Tech stack, Standards, Goals)
- [ ] How Gemini uses it automatically
- [ ] Business value: "Consistent guidance, no repeating yourself"
- [ ] ~600 word section, bridges to Lesson 4

**Reference**:
- Plan section: LESSON 2, "Hierarchical Memory"
- FR: FR-012 (first introduction)

**Effort**: 3-4 hours

**Acceptance**: GEMINI.md concept clear; 5-level hierarchy documented; example provided

---

### Task 2.7: MUST — Create Basic Configuration Preview Section

**Description**: Preview settings.json; tool control basics (expanded in Lesson 3)

**Acceptance Criteria**:
- [ ] Where settings.json lives: ~/.config/gemini/settings.json
- [ ] Basic options explained:
  - disabled_tools (which tools to disable)
  - model (which Gemini model)
  - api_key_source (authentication)
- [ ] Real example JSON provided
- [ ] Business use: "Restrict tools in sensitive environments"
- [ ] Note that Lesson 3 covers settings in depth
- [ ] ~400 word preview section

**Reference**:
- Plan section: LESSON 2, "Basic Configuration"
- FR: FR-006 (preview)

**Effort**: 1-2 hours

**Acceptance**: Settings preview clear; example JSON valid; transitions to Lesson 3

---

### Task 2.8: MUST — Develop 4 Code Examples (Lesson 2)

**Description**: Create realistic, tested code examples

**Acceptance Criteria**:
- [ ] Example 1: Basic command execution workflow
  - Sequence: /tools → /memory list → /stats
  - Shows information gathering in session
  - Runnable in Gemini CLI

- [ ] Example 2: @ syntax reference example
  - Prompt: @sales.csv summarize this file
  - Shows @ usage with real data file
  - Expected output documented

- [ ] Example 3: ! shell mode example
  - Prompt: ! git status (exit, show output)
  - Shows shell integration
  - How Gemini interprets output

- [ ] Example 4: settings.json modification
  - Config: disable web_search
  - Show before/after
  - Verification via /settings

- [ ] All examples tested in actual Gemini CLI
- [ ] All examples produce expected outputs
- [ ] Code blocks properly formatted

**Reference**:
- Plan section: LESSON 2, Code Examples
- FRs: FR-001, FR-003, FR-004, FR-006

**Effort**: 5-6 hours (including testing)

**Acceptance**: All 4 examples working; tested; expected outputs match

---

### Task 2.9: MUST — Create 5 Hands-On Exercises with Solutions

**Description**: Create bite-sized exercises with model solutions

**Acceptance Criteria**:
- [ ] Exercise 1: Command Mastery Drill (10 min)
  - Scenario: Research 5 competitors
  - Tasks: Execute /tools, /memory list, /stats
  - Expected outcome documented
  - Model solution provided

- [ ] Exercise 2: @ Syntax Practice (8 min)
  - Prompt 1: @./README.md summarize
  - Prompt 2: @https://stripe.com/pricing analyze
  - Expected outcomes documented
  - Model solutions provided

- [ ] Exercise 3: Keyboard Shortcuts Speedrun (7 min)
  - Navigate history with Ctrl+Y, +X
  - Clear and continue with Ctrl+L
  - Time tracking
  - Model solution: Expected completion time

- [ ] Exercise 4: Invocation Mode Selection (10 min)
  - 3 scenarios, choose correct mode
  - Justify choices
  - Model solutions with explanations

- [ ] Exercise 5: GEMINI.md Setup (10 min)
  - Create GEMINI.md in project
  - Add 3 sections
  - Verify /init loads correctly
  - Model GEMINI.md template provided

- [ ] All exercises have clear instructions
- [ ] All exercises have model solutions
- [ ] Time estimates realistic
- [ ] Difficulty progression appropriate

**Reference**:
- Plan section: LESSON 2, Exercises
- Total time: 45 minutes exercises

**Effort**: 6-8 hours (including solution documentation)

**Acceptance**: All 5 exercises clear; model solutions complete; time estimates validated

---

### Task 2.10: MUST — Create "Try With AI" End-of-Lesson Activities

**Description**: Create 4 closing prompts for dialogue-based learning

**Acceptance Criteria**:
- [ ] Prompt 1: Command Reference Builder
  - Personalized command priorities
  - Based on workflow type
  - Expected outcome: 5 commands + justification

- [ ] Prompt 2: Keyboard Efficiency Audit
  - Personalized shortcut usage
  - Creates cheat sheet
  - Expected outcome: 3 shortcuts + usage patterns

- [ ] Prompt 3: Automation Design
  - Pipe input mode blueprint
  - Shell script implementation
  - Expected outcome: Automation code + integration steps

- [ ] Prompt 4: Memory System Introduction
  - Hierarchical setup strategy
  - Template creation
  - Expected outcome: Memory architecture blueprint

- [ ] All prompts tested in Gemini CLI
- [ ] All prompts produce expected outputs
- [ ] Prompts build on lesson content naturally

**Reference**:
- Plan section: LESSON 2, Try With AI

**Effort**: 4-5 hours (including testing)

**Acceptance**: All 4 prompts working; tested; produce personalized outputs

---

### Task 2.11: SHOULD — Create Quick Reference Card

**Description**: One-page cheat sheet for Lesson 2 concepts

**Acceptance Criteria**:
- [ ] All 13+ commands listed with single-line descriptions
- [ ] @ syntax examples (3 examples)
- [ ] ! syntax examples (2 examples)
- [ ] Keyboard shortcuts (all 4)
- [ ] CLI modes (quick decision table)
- [ ] Fits on one printed page (PDF-ready)
- [ ] Styled for readability (tables, icons, colors)

**Effort**: 2-3 hours

**Acceptance**: Reference card complete; PDF output clean; all key concepts included

---

**Lesson 2 Total Effort**: 35-45 hours
**Lesson 2 Status**: 11 tasks (10 MUST, 1 SHOULD)

---

## Lesson 3: Configuration & Tool Management (Enhancement Phase)

### Task 3.1: MUST — Create Settings Hierarchy Section

**Description**: Document 7-level configuration hierarchy

**Acceptance Criteria**:
- [ ] All 7 levels documented with priority order:
  1. Built-in defaults
  2. Global settings: ~/.config/gemini/settings.json
  3. Project settings: ./gemini.json
  4. Subdirectory overrides: .gemini.json
  5. GEMINI.md tool instructions
  6. Session flags
  7. Runtime commands (/settings)
- [ ] Each level explained with examples
- [ ] Priority/override behavior documented
- [ ] Real business scenarios for each level
- [ ] ~600 word section

**Reference**:
- Plan section: LESSON 3, "Understanding the Settings Hierarchy"
- FR: FR-006, FR-007, FR-009, FR-010

**Effort**: 3-4 hours

**Acceptance**: All 7 levels clear; examples tested; priority documented

---

### Task 3.2: MUST — Create settings.json Deep Dive Section

**Description**: Document key configuration options with examples

**Acceptance Criteria**:
- [ ] Key options documented:
  - disabled_tools (array)
  - allowed_files (whitelist)
  - web_fetch_timeout (seconds)
  - context_window_percent (int)
  - auto_save_memory (boolean)
  - model (string)
  - api_key_source (enum)

- [ ] 3 example configurations provided:
  - Researcher config (all tools enabled)
  - Implementation config (selective tools)
  - Security-sensitive config (minimal tools)

- [ ] Each option explained: what it does, when to use
- [ ] Validation via /settings documented
- [ ] Common mistakes noted
- [ ] ~700 word section

**Reference**:
- Plan section: LESSON 3, "Configuring settings.json"
- FR: FR-006

**Effort**: 4-5 hours

**Acceptance**: All options documented; 3 example configs; JSON valid; /settings validation shown

---

### Task 3.3: MUST — Create Tool Restrictions for Security Section

**Description**: Business scenarios for tool restriction strategies

**Acceptance Criteria**:
- [ ] 5 business scenarios documented:
  - Research phase (enable all)
  - Implementation phase (selective)
  - Testing phase (read-only)
  - Production debugging (shell only)
  - Team compliance (gemini.json in repo)

- [ ] Per-session tool control explained (/settings override)
- [ ] Per-project tool control explained (./gemini.json)
- [ ] Team patterns: shared config in git repo
- [ ] Enforcement: consistency across developers
- [ ] ~500 word section

**Reference**:
- Plan section: LESSON 3, "Tool Restrictions for Security"
- FR: FR-006, FR-007, FR-009, FR-010

**Effort**: 3-4 hours

**Acceptance**: 5 scenarios clear; tool control options documented; team patterns shown

---

### Task 3.4: MUST — Create Token Cost Understanding Section

**Description**: Explain token impact of tools; preview /compress and /memory

**Acceptance Criteria**:
- [ ] Token cost per tool explained:
  - File read: Tokens ≈ file size
  - Web fetch: Tokens ≈ page size
  - Web search: Tokens = query + all results
  - Shell: Tokens = command + output

- [ ] Token cost examples (real file sizes → token estimates)
- [ ] Strategy: @ for large files (instead of /copy)
- [ ] Strategy: /mcp for batch operations
- [ ] Preview of Lesson 4: /compress and /memory for efficiency
- [ ] ~400 word section, bridges to Lesson 4

**Reference**:
- Plan section: LESSON 3, "Understanding Token Cost"
- FR: FR-015 (preview)

**Effort**: 2-3 hours

**Acceptance**: Token costs clear; strategies practical; Lesson 4 bridge smooth

---

### Task 3.5: MUST — Enhance Error Handling Section

**Description**: Add "Tool disabled" error handling to existing section

**Acceptance Criteria**:
- [ ] Keep existing error handling section
- [ ] Add: "ERROR: Tool disabled - check settings.json"
  - What it means
  - How to fix
  - Prevention strategy

- [ ] Add: "Tool not available in this context"
  - Scope explanation
  - How to enable

- [ ] All error messages documented with solutions

**Reference**:
- Plan section: LESSON 3, Enhancement of existing section
- FR: FR-006, FR-007

**Effort**: 1-2 hours

**Acceptance**: Tool-related errors documented; solutions clear

---

### Task 3.6: MUST — Enhance Privacy Section

**Description**: Add settings.json controls to existing privacy guidance

**Acceptance Criteria**:
- [ ] Keep existing privacy best practices
- [ ] Add: How settings.json enforces privacy
  - disabled_tools: Prevent accidental web access
  - allowed_files: Whitelist access to sensitive directories

- [ ] Add: Team policy examples
  - "We disable web_search in production"
  - "We only allow reads, not writes"

- [ ] Integration with security workflow

**Reference**:
- Plan section: LESSON 3, Enhancement of existing section
- FR: FR-009, FR-010

**Effort**: 1-2 hours

**Acceptance**: Privacy controls documented; team policy examples shown

---

### Task 3.7: MUST — Develop 2 Code Examples (Lesson 3)

**Description**: Create realistic settings configuration examples

**Acceptance Criteria**:
- [ ] Example 1: Researcher settings.json
  - All tools enabled
  - Realistic for competitive analysis workflow
  - Documented assumptions

- [ ] Example 2: Security-sensitive settings.json
  - Minimal tools enabled
  - File access restricted
  - Write operations disabled
  - Production-ready

- [ ] All JSON valid and tested
- [ ] Expected behavior documented

**Reference**:
- Plan section: LESSON 3, Code Examples
- FR: FR-006, FR-007

**Effort**: 3-4 hours (including validation)

**Acceptance**: All examples valid; tested; expected behaviors documented

---

### Task 3.8: MUST — Create 2 Hands-On Exercises with Solutions

**Description**: Configuration exercises with model solutions

**Acceptance Criteria**:
- [ ] Exercise 1: settings.json Configuration (5 min)
  - Scenario: Sensitive customer data
  - Create settings with disabled tools
  - Test via /settings
  - Model solution provided

- [ ] Exercise 2: Tool Selection Strategy (5 min)
  - Scenario: Competitive pricing analysis (5 competitors)
  - Recommended tools: web_fetch, search, file_ops
  - Create gemini.json
  - Model solution with justifications

- [ ] Both exercises have clear instructions
- [ ] Both have model solutions with explanations
- [ ] Time estimates realistic

**Reference**:
- Plan section: LESSON 3, Exercises
- Total time: 10 minutes exercises

**Effort**: 4-5 hours

**Acceptance**: Exercises clear; model solutions complete; testing validated

---

### Task 3.9: MUST — Create "Try With AI" End-of-Lesson Activities

**Description**: Create 2 closing prompts for configuration learning

**Acceptance Criteria**:
- [ ] Prompt 1: Security Configuration Design
  - Role-specific configuration
  - Tool balance for use case
  - Explanation of enable/disable choices
  - Expected outcome: settings.json + rationale

- [ ] Prompt 2: Token Efficiency Audit
  - Workflow analysis
  - Token-heavy tools identified
  - Restructuring recommendations
  - Expected outcome: Optimized workflow strategy

- [ ] Both prompts tested in Gemini CLI
- [ ] Both produce expected outputs

**Reference**:
- Plan section: LESSON 3, Try With AI

**Effort**: 3-4 hours (including testing)

**Acceptance**: Both prompts working; tested; produce personalized outputs

---

### Task 3.10: SHOULD — Create Configuration Decision Tree

**Description**: Visual guide for selecting appropriate configuration

**Acceptance Criteria**:
- [ ] Decision tree for tool selection by use case
- [ ] Decision tree for settings hierarchy level
- [ ] Format: Flowchart or table-based
- [ ] Covers 5+ common scenarios
- [ ] Easy to reference

**Effort**: 2-3 hours

**Acceptance**: Decision tree complete; covers scenarios; visually clear

---

**Lesson 3 Total Effort**: 25-35 hours
**Lesson 3 Status**: 10 tasks (9 MUST, 1 SHOULD)

---

## Lesson 4: Memory Systems & Context Management (MAJOR REDESIGN)

### Task 4.1: MUST — Create Three Memory Types Foundation Section

**Description**: Foundational explanation of ephemeral, GEMINI.md, and save_memory

**Acceptance Criteria**:
- [ ] Ephemeral memory section:
  - What it is: In-RAM conversation history
  - Lifetime: Until /quit or crash
  - Access: Automatic (Ctrl+Y, Ctrl+X)
  - Use cases: Exploration, single-task sessions
  - Pros: No setup, private
  - Cons: Lost on exit, no sharing

- [ ] GEMINI.md hierarchical section:
  - What it is: Persistent markdown context files
  - 5-level hierarchy explained
  - Lifetime: Persists across sessions
  - Access: Automatic loading, @GEMINI.md reference
  - Content types: Tech context, standards, goals, issues
  - Use cases: Onboarding, consistency, large projects
  - Pros: Persistent, versioned, shared
  - Cons: Requires management, can become stale

- [ ] save_memory section:
  - What it is: Named conversation snapshots
  - Lifetime: Until /memory delete
  - Access: /memory commands
  - Use cases: Long-running projects, recurring tasks
  - Pros: Full context restoration, named, shareable
  - Cons: Token overhead, requires cleanup

- [ ] All 3 types clearly differentiated
- [ ] Business examples for each
- [ ] ~800 word comprehensive section

**Reference**:
- Plan section: LESSON 4, "Three Types of Memory"
- FR: FR-011, FR-012, FR-013

**Effort**: 5-6 hours

**Acceptance**: All 3 types clear; differentiation obvious; examples realistic

---

### Task 4.2: MUST — Create Organizing GEMINI.md Section

**Description**: Templates and strategies for hierarchical GEMINI.md

**Acceptance Criteria**:
- [ ] Template 1: Global Context (~/.gemini/GEMINI.md)
  - Standard libraries (Python, TypeScript)
  - Coding standards (type hints, testing, formatting)
  - Security standards (no hardcoding secrets, auth, input validation)
  - Full working template provided

- [ ] Template 2: Project Context (./GEMINI.md)
  - Tech stack (languages, frameworks, databases)
  - Project goals (deliverables, dates)
  - Critical files (@./docs/api.md references)
  - Known issues and workarounds
  - Full working template provided

- [ ] Template 3: Feature Branch Context (./.git/gemini/feature-auth.md)
  - Feature name
  - Acceptance criteria (checklist)
  - Architecture notes
  - Testing checklist
  - Full working template provided

- [ ] Strategy for maintaining hierarchy
- [ ] When to create subdirectory GEMINI.md
- [ ] Team practices for GEMINI.md sharing
- [ ] ~900 word section with templates

**Reference**:
- Plan section: LESSON 4, "Organizing GEMINI.md"
- FR: FR-012

**Effort**: 6-7 hours

**Acceptance**: All 3 templates complete; strategies clear; templates tested

---

### Task 4.3: MUST — Create /memory Command Reference Section

**Description**: Document /memory operations (list, save, load, delete, clear)

**Acceptance Criteria**:
- [ ] /memory list documented
  - Output format explained
  - Shows: name, created date, size, preview

- [ ] /memory save <name> documented
  - Syntax and examples
  - Naming convention recommendation (date-based, task-based)
  - Storage location
  - Typical use case

- [ ] /memory load <name> documented
  - Restores full conversation state
  - Use case: Resume previous research
  - Example workflow

- [ ] /memory delete <name> documented
  - Cleanup process
  - When to use

- [ ] /memory clear documented
  - Immediate session history clear
  - vs. /quit difference

- [ ] Practical workflow example
  - Session 1: Research, /memory save
  - Session 2: /memory load, continue
  - Session 3: /memory list

- [ ] ~600 word section

**Reference**:
- Plan section: LESSON 4, "The /memory Command"
- FR: FR-013, FR-014

**Effort**: 4-5 hours

**Acceptance**: All operations documented; workflow example clear; tested

---

### Task 4.4: MUST — Create Token Efficiency & Compression Section

**Description**: When and how to use /compress and /copy; token management

**Acceptance Criteria**:
- [ ] Context window usage explained
  - Every prompt costs tokens
  - Longer context = more tokens per interaction
  - Status bar shows remaining

- [ ] /compress operation documented
  - When to use (at 90% context, or 50+ messages)
  - What it does (summarizes conversation)
  - Trade-offs: nuance lost, core ideas preserved
  - Example before/after

- [ ] /copy operation documented
  - Copies last response to clipboard
  - Avoids context pollution
  - Use case: Save data separately

- [ ] Token minimization strategies:
  - Use @file instead of pasting
  - Use /mcp for bulk operations
  - Use /memory for archiving
  - Use /compress before hitting limit

- [ ] Advanced: Token remaining indicator
  - Status bar reading: "Tokens: 842K/1M"
  - Strategy at 80%+: plan /compress or /memory save

- [ ] ~500 word section

**Reference**:
- Plan section: LESSON 4, "Token Efficiency"
- FR: FR-015

**Effort**: 4-5 hours

**Acceptance**: /compress and /copy clear; strategies practical; token management articulated

---

### Task 4.5: MUST — Create Memory in Action: Real Workflows Section

**Description**: Three realistic workflows demonstrating all memory types

**Acceptance Criteria**:
- [ ] Workflow 1: Research Project (Save & Load)
  - Day 1: Research, analyze, /memory save
  - Day 8: /memory load, continue, /memory save v2
  - Shows: save_memory for persistence
  - Time span: Multiple sessions

- [ ] Workflow 2: Codebase Learning (GEMINI.md Hierarchy)
  - Global GEMINI.md: Developer standards
  - Project GEMINI.md: Tech stack and goals
  - Feature GEMINI.md: Feature-specific guidance
  - Shows: automatic context building
  - Team context: No repeating explanations

- [ ] Workflow 3: Token Efficiency (Monitor & Compress)
  - Session start: 1M tokens
  - After 30 min: 400K remaining
  - At 100K: /compress to summarize
  - Before quit: /memory save findings
  - Shows: lifetime management

- [ ] Each workflow: ~200 words
- [ ] All workflows realistic and tested
- [ ] Total: ~700 word section

**Reference**:
- Plan section: LESSON 4, "Memory in Action"
- FR: FR-011, FR-012, FR-013, FR-014, FR-015

**Effort**: 5-6 hours

**Acceptance**: All 3 workflows realistic; tested; demonstrate concepts clearly

---

### Task 4.6: MUST — Develop 2-3 Code Examples (Lesson 4)

**Description**: Create example GEMINI.md files and /memory workflows

**Acceptance Criteria**:
- [ ] Example 1: Hierarchical GEMINI.md structure
  - 3-level example (global, project, feature)
  - All files complete and valid
  - Shown in context of workflow

- [ ] Example 2: /memory workflow
  - Session 1: Commands shown
  - Session 2: Loading and continuing
  - Expected outputs

- [ ] Example 3: Context token tracking
  - Session start state
  - After interactions: remaining tokens
  - /compress shown

- [ ] All examples tested and working
- [ ] Expected outputs documented

**Reference**:
- Plan section: LESSON 4, Code Examples
- FR: FR-011, FR-012, FR-014

**Effort**: 5-6 hours (including testing)

**Acceptance**: All examples valid; tested; expected outputs match

---

### Task 4.7: MUST — Create 4 Hands-On Exercises with Solutions

**Description**: Memory system exercises with model solutions

**Acceptance Criteria**:
- [ ] Exercise 1: Create Hierarchical GEMINI.md (8 min)
  - Create 3 GEMINI.md files at different levels
  - Add specified sections to each
  - Verify with /init that they load
  - Model GEMINI.md templates provided

- [ ] Exercise 2: Memory Lifecycle (10 min)
  - Session 1: Research and /memory save
  - Session 2: /memory load and continue
  - Session 3: /memory list and view
  - Model solution: Commands and expected outputs

- [ ] Exercise 3: Token Monitoring & Compression (5 min)
  - Start session with verbose context
  - Watch token remaining
  - Use /compress at 60%
  - Observe context reduction
  - Model solution: Before/after comparison

- [ ] Exercise 4: Memory Types Decision (7 min)
  - 5 scenarios: choose memory type
  - Justify each choice
  - Model solutions with explanations

- [ ] All exercises clear and testable
- [ ] All have model solutions
- [ ] Time estimates realistic and validated

**Reference**:
- Plan section: LESSON 4, Exercises
- Total time: 30 minutes exercises

**Effort**: 6-8 hours

**Acceptance**: All 4 exercises clear; model solutions complete; tested

---

### Task 4.8: MUST — Create "Try With AI" End-of-Lesson Activities

**Description**: Create 3 closing prompts for memory system mastery

**Acceptance Criteria**:
- [ ] Prompt 1: GEMINI.md Design
  - Project-specific design based on type
  - Hierarchical structure template
  - Level-specific content
  - Expected outcome: Architecture blueprint

- [ ] Prompt 2: Memory Strategy
  - Workflow-to-memory mapping
  - When to use each type
  - /compress and /memory save points
  - Expected outcome: Memory strategy guide

- [ ] Prompt 3: Long-Running Project Management
  - Scope: 3-6 month project
  - /memory naming and organization
  - Retrieval strategy
  - Expected outcome: Memory management system

- [ ] All prompts tested in Gemini CLI
- [ ] All produce expected outputs

**Reference**:
- Plan section: LESSON 4, Try With AI

**Effort**: 4-5 hours (including testing)

**Acceptance**: All 3 prompts working; tested; personalized outputs

---

### Task 4.9: SHOULD — Create Memory Architecture Decision Tree

**Description**: Visual guide for selecting memory type for given scenario

**Acceptance Criteria**:
- [ ] Decision tree covers common scenarios
- [ ] Paths lead to: ephemeral, GEMINI.md, or save_memory
- [ ] Format: Flowchart or decision table
- [ ] Easy to reference

**Effort**: 2-3 hours

**Acceptance**: Decision tree complete; covers scenarios; clear

---

**Lesson 4 Total Effort**: 40-50 hours
**Lesson 4 Status**: 9 tasks (8 MUST, 1 SHOULD)

---

## Lesson 5: Extend with MCP, Custom Commands & Extensions (MAJOR EXPANSION)

### Task 5.1: MUST — Complete Playwright MCP Server Section

**Description**: Complete partial Playwright content; add installation, configuration, workflow

**Acceptance Criteria**:
- [ ] Installation steps documented (npm command)
- [ ] Configuration (mcp-servers.json) provided
- [ ] Workflow example: Competitive analysis (3 competitors)
- [ ] Advanced: Form filling with Playwright
- [ ] When to use: Playwright vs. built-in web fetch
- [ ] Decision framework documented
- [ ] ~1000 word complete section

**Reference**:
- Plan section: LESSON 5, "MCP Server 1: Playwright"
- Current file has partial content (100+ lines)
- FR: FR-016, FR-017, FR-020

**Effort**: 6-8 hours

**Acceptance**: Playwright section complete; configuration valid; workflow tested

---

### Task 5.2: MUST — Create Context7 MCP Server Section

**Description**: Documentation integration via Context7; practical API integration

**Acceptance Criteria**:
- [ ] What Context7 does (access to current docs)
- [ ] Installation documented
- [ ] Configuration (documentation sources) shown
- [ ] Real workflow: Stripe API integration
  - Current API docs access
  - Webhook setup
  - Signature verification code
- [ ] Advanced: Custom documentation sources (internal wiki)
- [ ] ~700 word section

**Reference**:
- Plan section: LESSON 5, "MCP Server 2: Context7"
- FR: FR-016, FR-018, FR-020

**Effort**: 5-6 hours

**Acceptance**: Context7 section complete; Stripe example working; tested

---

### Task 5.3: MUST — Create GitHub MCP Server Section

**Description**: Code collaboration via GitHub MCP

**Acceptance Criteria**:
- [ ] What GitHub MCP does (query repos, issues, PRs, search)
- [ ] Installation documented
- [ ] Setup: GitHub API token (GITHUB_TOKEN)
- [ ] Real workflows:
  - Query recent PRs
  - Find open issues
  - Search for deprecated API usage
  - Summarize PR changes
- [ ] Integration with Playwright (open PR in browser)
- [ ] ~600 word section

**Reference**:
- Plan section: LESSON 5, "MCP Server 3: GitHub"
- FR: FR-016, FR-019, FR-020

**Effort**: 5-6 hours

**Acceptance**: GitHub section complete; workflows realistic; tested

---

### Task 5.4: MUST — Create Custom Slash Commands (TOML) Section

**Description**: User-defined command automation via TOML files

**Acceptance Criteria**:
- [ ] What custom commands are (different from MCP servers)
- [ ] Format: .gemini-commands.toml in project root
- [ ] Basic syntax documented with examples
- [ ] 3+ example commands provided:
  - /analyze (code review)
  - /research (topic research)
  - /deploy (deployment checklist)
- [ ] {{args}} placeholder injection explained with examples
- [ ] {{service}}, {{environment}}, {{feature}} multi-variable examples
- [ ] Advanced: !{command} subprocess execution
  - Shell command injection
  - Output captured by Gemini
- [ ] User vs. project scope distinction:
  - User: ~/.gemini/.gemini-commands.toml (personal)
  - Project: ./.gemini-commands.toml (team, in git)
  - Loading priority explained
- [ ] Team example: /deploy-prod, /review-pr, /estimate-effort
- [ ] Real business impact demonstrated (time savings)
- [ ] ~1200 word section

**Reference**:
- Plan section: LESSON 5, "Custom Slash Commands"
- FR: FR-021, FR-022, FR-023, FR-024, FR-025

**Effort**: 8-10 hours

**Acceptance**: Custom commands fully explained; TOML syntax valid; team examples shown

---

### Task 5.5: MUST — Create Extensions Section

**Description**: Extension installation, evaluation, and security

**Acceptance Criteria**:
- [ ] What extensions are (pre-packaged capabilities)
- [ ] Difference: vs. MCP servers, vs. custom commands
- [ ] Installation process documented
  - `gemini extension install <name>`
  - `gemini extension list`
  - `gemini extension remove <name>`
- [ ] Example extensions listed (data science, ML workflow, DevOps)
- [ ] Security evaluation framework:
  1. Vendor reputation (official, popular, unknown)
  2. Code review (open source, auditable)
  3. Permissions (file access, shell, network)
  4. Dependencies (suspicious packages)
  5. Updates (actively maintained)
  6. Community (reviews, users)
- [ ] Risk assessment matrix:
  - Green: Official, popular, open source
  - Yellow: Community, requires scrutiny
  - Red: Closed source, unknown author, root access
- [ ] Real evaluation example: Data science toolkit
- [ ] ~800 word section

**Reference**:
- Plan section: LESSON 5, "Extensions"
- FR: FR-026, FR-027, FR-028, FR-029

**Effort**: 6-7 hours

**Acceptance**: Extensions explained; security framework clear; evaluation example realistic

---

### Task 5.6: MUST — Create Integration Architecture Section

**Description**: How systems work together (GEMINI.md + custom commands + MCP + extensions)

**Acceptance Criteria**:
- [ ] System 1: GEMINI.md (context/knowledge)
- [ ] System 2: Custom commands (automation/workflow)
- [ ] System 3: MCP servers (capabilities/integration)
- [ ] System 4: Extensions (pre-packaged solutions)
- [ ] Architecture diagram (ASCII or visual)
- [ ] User query flow through all systems
- [ ] Practical integrated workflow example:
  - Setup: GEMINI.md, custom command, MCP servers
  - Execution: /market-research "AI productivity"
  - Step-by-step: What happens at each layer
  - Result: 10-minute competitive analysis
- [ ] How systems avoid conflicts/duplication
- [ ] ~600 word section

**Reference**:
- Plan section: LESSON 5, "Integration Architecture"
- FR: FR-020, FR-025, FR-030

**Effort**: 5-6 hours

**Acceptance**: Architecture clear; diagram helpful; workflow example integrated

---

### Task 5.7: MUST — Develop 3+ Code Examples (Lesson 5)

**Description**: Create realistic MCP, custom command, and extension examples

**Acceptance Criteria**:
- [ ] Example 1: Playwright MCP configuration
  - mcp-servers.json valid
  - Capabilities specified

- [ ] Example 2: Custom TOML command with {{args}}
  - Realistic command definition
  - Multiple variables
  - Expected output documented

- [ ] Example 3: GitHub MCP query
  - API token setup
  - Example queries
  - Expected outputs

- [ ] Example 4: Custom command with !{command} execution
  - Shell integration
  - Output handling

- [ ] All examples tested and working
- [ ] Expected outputs documented

**Reference**:
- Plan section: LESSON 5, Code Examples
- FR: FR-017, FR-019, FR-021, FR-023

**Effort**: 8-10 hours (including testing and validation)

**Acceptance**: All examples valid; tested; working as documented

---

### Task 5.8: MUST — Create 4 Hands-On Exercises with Solutions

**Description**: MCP, custom commands, extensions exercises

**Acceptance Criteria**:
- [ ] Exercise 1: Install Playwright MCP (8 min)
  - Installation steps
  - Configuration in mcp-servers.json
  - Test: Browse and extract data
  - Model solution provided

- [ ] Exercise 2: Create Custom Command (12 min)
  - Create .gemini-commands.toml
  - Define /research command
  - Define /analyze command
  - Test both commands
  - Model solutions provided

- [ ] Exercise 3: Security Evaluation (5 min)
  - Evaluate 2 fictional extensions
  - Apply security framework
  - Make decision: safe/caution/skip
  - Model solutions with rationale

- [ ] Exercise 4: Integration Design (10 min)
  - Scenario: Analytics dashboard
  - Design: GEMINI.md, custom commands, MCP, extensions
  - Document architecture
  - Model solution provided

- [ ] All exercises clear and testable
- [ ] All have model solutions
- [ ] Time estimates realistic

**Reference**:
- Plan section: LESSON 5, Exercises
- Total time: 35 minutes exercises

**Effort**: 8-10 hours

**Acceptance**: All 4 exercises clear; model solutions complete; tested

---

### Task 5.9: MUST — Create "Try With AI" End-of-Lesson Activities

**Description**: Create 4 closing prompts for advanced integration

**Acceptance Criteria**:
- [ ] Prompt 1: MCP Server Selection
  - Workflow analysis
  - MCP selection for business impact
  - Configuration design
  - Time savings estimation
  - Expected outcome: MCP strategy

- [ ] Prompt 2: Custom Command Library Design
  - Team process analysis
  - 3-5 custom commands designed
  - .gemini-commands.toml file generated
  - Business impact calculated
  - Expected outcome: Team-ready command set

- [ ] Prompt 3: Extension Security Evaluation
  - Real/fictional extension provided
  - Security evaluation applied
  - Risk assessment
  - Recommendation
  - Expected outcome: Security decision

- [ ] Prompt 4: Full Integration Architecture
  - Team context provided
  - Complete architecture designed
  - File structures shown
  - Configurations provided
  - Productivity improvement estimated
  - Expected outcome: Complete blueprint

- [ ] All prompts tested in Gemini CLI
- [ ] All produce expected outputs

**Reference**:
- Plan section: LESSON 5, Try With AI

**Effort**: 6-8 hours (including testing)

**Acceptance**: All 4 prompts working; tested; personalized outputs

---

### Task 5.10: SHOULD — Create MCP Troubleshooting Guide

**Description**: Common MCP connectivity issues and solutions

**Acceptance Criteria**:
- [ ] 5+ common issues documented:
  - "MCP server not connecting"
  - "Command timeout"
  - "Permission denied"
  - "Extensions conflict"
  - "Version incompatibility"
- [ ] Each issue has: symptom, cause, solution
- [ ] Debugging steps provided
- [ ] Support resources linked

**Effort**: 3-4 hours

**Acceptance**: Guide complete; covers common issues; solutions practical

---

### Task 5.11: SHOULD — Create Advanced Patterns Guide

**Description**: Advanced MCP usage patterns (error handling, rate limiting, caching)

**Acceptance Criteria**:
- [ ] Error handling in MCP workflows
- [ ] Rate limiting strategies
- [ ] Caching for performance
- [ ] Multi-MCP server orchestration
- [ ] Fallback strategies

**Effort**: 4-5 hours

**Acceptance**: Patterns documented; examples provided; practical

---

**Lesson 5 Total Effort**: 65-85 hours
**Lesson 5 Status**: 11 tasks (9 MUST, 2 SHOULD)

---

## Cross-Lesson Tasks

### Task CR.1: MUST — Implement Lesson README.md

**Description**: Create chapter-level README with learning outcomes and structure

**Acceptance Criteria**:
- [ ] File: `book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/README.md`
- [ ] Sections:
  - Overview (what this chapter teaches)
  - Learning outcomes (all 5 lessons)
  - Structure (lessons 1-5 with time estimates)
  - Prerequisites (Chapter 5: Claude Code context)
  - What you'll build (integrated workflows)
  - Next chapters (Chapter 7: Bash, Chapter 8+: Python/TS)
- [ ] Follow `.claude/output-styles/chapters.md` template

**Effort**: 2-3 hours

**Acceptance**: README complete; follows style guide; outcomes clear

---

### Task CR.2: MUST — Comprehensive Review & Quality Assurance

**Description**: Full chapter review before deployment

**Acceptance Criteria**:
- [ ] All 5 lessons complete
- [ ] All code examples tested and working
- [ ] All exercises have model solutions
- [ ] All "Try With AI" prompts tested
- [ ] All references and links valid
- [ ] Consistent tone and style across lessons
- [ ] Markdown formatting validated
- [ ] No placeholder text remaining
- [ ] Grammar and spelling check passed
- [ ] Accessibility reviewed (clear language, definitions for jargon)

**Effort**: 4-6 hours

**Acceptance**: All quality checks passed; ready for publication

---

### Task CR.3: SHOULD — Create Interactive Checklist

**Description**: Student self-assessment checklist for chapter

**Acceptance Criteria**:
- [ ] "Can you..." format questions for each major topic
- [ ] Covers all 35 functional requirements
- [ ] Organized by lesson
- [ ] Answers key provided

**Effort**: 3-4 hours

**Acceptance**: Checklist complete; comprehensive; self-assessment ready

---

### Task CR.4: SHOULD — Create Teaching Notes for Instructors

**Description**: Guidance for instructors using this chapter

**Acceptance Criteria**:
- [ ] Common student misconceptions documented
- [ ] Time management notes
- [ ] Pacing recommendations
- [ ] Extension activities for advanced students
- [ ] Troubleshooting guide
- [ ] Assessment rubrics

**Effort**: 4-5 hours

**Acceptance**: Teaching notes complete; practical; actionable

---

**Cross-Lesson Total Effort**: 15-22 hours
**Cross-Lesson Status**: 4 tasks (2 MUST, 2 SHOULD)

---

## Summary: Task Breakdown by Category

### MUST Tasks (Critical Path)
- Lesson 1: 2 tasks
- Lesson 2: 9 tasks
- Lesson 3: 9 tasks
- Lesson 4: 8 tasks
- Lesson 5: 9 tasks
- Cross-lesson: 2 tasks
- **Total MUST**: 39 tasks

### SHOULD Tasks (High Priority)
- Lesson 2: 1 task
- Lesson 3: 1 task
- Lesson 4: 1 task
- Lesson 5: 2 tasks
- Cross-lesson: 2 tasks
- **Total SHOULD**: 7 tasks

### Total Tasks: 46

---

## Effort Estimation

| Phase | Min Hours | Max Hours | Story Points |
|-------|-----------|-----------|--------------|
| Lesson 1 | 3 | 5 | 1 |
| Lesson 2 | 35 | 45 | 8-9 |
| Lesson 3 | 25 | 35 | 6-7 |
| Lesson 4 | 40 | 50 | 8-10 |
| Lesson 5 | 65 | 85 | 13-17 |
| Cross-lesson | 15 | 22 | 3-4 |
| **TOTAL** | **183** | **242** | **39-52** |

**Estimated Project Duration**: 6-8 weeks (at 30 hrs/week development)

---

## Acceptance Criteria (Definition of Done)

**All MUST tasks completed**: ✓ No chapter publication without completing all 39 MUST tasks

**Quality Standards**:
- [ ] No placeholder text remaining
- [ ] All code examples tested and working (80%+ coverage)
- [ ] All exercises have clear instructions and model solutions
- [ ] All "Try With AI" prompts tested in actual Gemini CLI
- [ ] Reading time validated: ~100 min (core lessons 1-4), ~175 min (full chapter)
- [ ] All 35 functional requirements mapped to specific lesson content
- [ ] Consistent style across all lessons
- [ ] Accessibility requirements met (clear language, jargon defined)
- [ ] Links and references validated
- [ ] Grammar and spelling checked

**Functional Requirements Coverage**:
- [ ] FR-001 to FR-035 all addressed
- [ ] Each FR traceable to specific lesson section
- [ ] No FR left to interpretation

**Success Criteria Achievement**:
- [ ] Command recall: 90%+ (design prompts to test recall)
- [ ] Configuration proficiency: 85%+ (exercise validation)
- [ ] Memory system mastery: 80%+ (GEMINI.md creation)
- [ ] MCP server installation: 75%+ (exercise with testing)
- [ ] Custom command creation: 70%+ (TOML file exercise)
- [ ] Productivity improvement: 5-10x (documented in workflows)

**Chapter Integration**:
- [ ] Backward reference to Chapter 5 working
- [ ] Forward reference to Chapter 7 smooth
- [ ] Prerequisites clear
- [ ] Learning objectives articulated
- [ ] Skills progression appropriate for Part 2

---

## Risks & Mitigation Strategies

### Risk 1: Content Too Comprehensive (Lesson 5 especially)

**Probability**: High | **Impact**: High

**Mitigation**:
- Mark Lesson 5 as "Advanced: Optional for power users"
- Core mastery in Lessons 1-4 fits 60-90 min target
- Lesson 5 provides depth for those who want it
- Clear signposting: "Ready to go deeper?" before Lesson 5

---

### Risk 2: Outdated Tool Versions During Development

**Probability**: Medium | **Impact**: Medium

**Mitigation**:
- Flag all version numbers with dates
- Include "Check for latest" instructions
- Add "Try With AI: Check current Gemini CLI version" prompts
- Schedule quarterly content review

---

### Risk 3: MCP Server Stability/Changes

**Probability**: Medium | **Impact**: Medium

**Mitigation**:
- Provide 3 MCP examples (Playwright, Context7, GitHub)
- If one breaks, others remain valid
- Include troubleshooting guide
- Document "MCP servers may change; core principles remain"

---

### Risk 4: Student Overwhelm (5 lessons in 1 chapter)

**Probability**: Medium | **Impact**: Low

**Mitigation**:
- Progressive complexity: A2 → A2 → A2 → B1 → B1
- Each lesson standalone
- Clear prerequisites between lessons
- Exercises bite-sized (5-12 minutes each)
- "Try With AI" summarizes key concepts

---

### Risk 5: Time Estimation Inaccuracy

**Probability**: Medium | **Impact**: Medium

**Mitigation**:
- Build 30% contingency buffer (183-242 hours)
- Validate exercise times during implementation
- Adjust if lessons 2-3 take longer than estimated
- Document actual times for future reference

---

## Follow-Ups & Next Steps

After this planning phase completes, next actions are:

1. **Phase 2: Implementation** (content-implementer subagent)
   - Implement lessons in sequence (1-5)
   - Create content, examples, exercises
   - Submit each lesson for human review
   - Iterate on feedback

2. **Phase 3: Validation** (validation-auditor subagent)
   - Verify all 35 FRs covered
   - Code examples tested
   - Exercises validated
   - "Try With AI" prompts working

3. **Phase 4: Integration** (Human final review)
   - Narrative flow across lessons
   - Cross-chapter references working
   - Formatting and style consistent
   - Chapter ready for Docusaurus build

4. **Phase 5: Publication**
   - Merge to main branch
   - Deploy to production
   - Announce chapter availability

---

## Appendix: Functional Requirements Mapping

| FR | Title | Lesson | Section |
|-----|-------|--------|----------|
| FR-001 | Slash command reference | 2 | Essential Commands |
| FR-002 | Why Gemini CLI excels | 1 | Enhancement sections |
| FR-003 | @ context syntax | 2 | @ Syntax |
| FR-004 | ! shell syntax | 2 | ! Syntax |
| FR-005 | Keyboard shortcuts | 2 | Keyboard Shortcuts |
| FR-006 | settings.json configuration | 2, 3 | Configuration sections |
| FR-007 | Per-session tool restrictions | 3 | Settings Hierarchy |
| FR-008 | Command-line invocation modes | 2 | CLI Invocation Modes |
| FR-009 | allowed_files whitelist | 3 | settings.json Deep Dive |
| FR-010 | Disable tool categories | 3 | Tool Restrictions |
| FR-011 | Ephemeral session memory | 4 | Three Memory Types |
| FR-012 | GEMINI.md hierarchical | 2, 4 | GEMINI.md sections |
| FR-013 | save_memory persistent | 4 | Three Memory Types |
| FR-014 | /memory commands | 4 | /memory Command |
| FR-015 | /compress, token management | 4 | Token Efficiency |
| FR-016 | MCP server configuration | 5 | Playwright section |
| FR-017 | Playwright MCP | 5 | Playwright section |
| FR-018 | Context7 MCP | 5 | Context7 section |
| FR-019 | GitHub MCP | 5 | GitHub section |
| FR-020 | Practical MCP workflows | 5 | All MCP sections |
| FR-021 | Custom slash commands TOML | 5 | Custom Commands |
| FR-022 | {{args}} injection | 5 | Custom Commands |
| FR-023 | !{command} execution | 5 | Custom Commands |
| FR-024 | User vs. project scope | 5 | Custom Commands |
| FR-025 | Team command sharing | 5 | Custom Commands |
| FR-026 | Extension installation | 5 | Extensions |
| FR-027 | Security evaluation | 5 | Extensions |
| FR-028 | Troubleshooting | 5 | Troubleshooting |
| FR-029 | Extension management | 5 | Extensions |
| FR-030 | Advanced MCP patterns | 5 | Integration Architecture |
| FR-031 | Checkpointing workflows | 2 | CLI Invocation Modes |
| FR-032 | Conversation branching | 4 | Memory sections |
| FR-033 | IDE integration | 2 | /ide command |
| FR-034 | Sandboxing | 2 | --sandbox mode |
| FR-035 | Token compression strategy | 4 | Token Efficiency |

---

## Document Metadata

**Plan Version**: 1.0
**Created**: 2025-11-07
**Status**: Ready for Implementation
**Next Review**: After Phase 2 (Implementation) completion

**Prepared by**: Chapter Planner (Spec-Driven Development)
**Approved by**: [Pending user approval]

**References**:
- Specification: `specs/chapter-6-google-gemini-cli/spec.md` (referenced, not yet created)
- Plan: `specs/chapter-6-google-gemini-cli/plan.md` (this document)
- Constitution: `.specify/memory/constitution.md` v3.0.2
- Output Styles: `.claude/output-styles/lesson.md`
- Current Lessons: `book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/`

---

**End of Task Checklist**
