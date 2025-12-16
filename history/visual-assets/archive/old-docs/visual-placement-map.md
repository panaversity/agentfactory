# Visual Placement Mapping: Parts 2-4

**Date**: 2025-11-22
**Scope**: 38 visuals across Chapters 5-15
**Purpose**: Systematic mapping of visual assets to specific lesson locations for markdown embedding

---

## Placement Strategy

**Principles**:

1. **Conceptual visuals** → Early in lesson (after section header introducing concept)
2. **Workflow/process visuals** → Mid-lesson (where process is explained)
3. **Comparison visuals** → Where contrasting approaches discussed
4. **Summary visuals** → End of section (consolidating multiple concepts)

**Markdown Format**:

```markdown
![Alt Text](/img/part-N/chapter-NN/filename.png)
```

---

## Chapter 5: Claude Code Features and Workflows

**Chapter Directory**: `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/`

### Visual 1: Traditional Chat AI vs Agentic Claude Code

- **File**: `/img/part-2/chapter-05/traditional-chat-vs-claude-code-workflow.png`
- **Placement**: `01-origin-story.md` - After "## The Shift from Chat to Agent" section
- **Alt Text**: Split-screen workflow comparison showing traditional chat AI requiring manual copy-paste steps (left, red bottlenecks) versus Claude Code's autonomous workflow with file context and command execution (right, green flow)
- **Purpose**: Establish core distinction between passive chat and active agent

### Visual 2: Claude Code Context Architecture

- **File**: `/img/part-2/chapter-05/claude-code-context-architecture-four-layers.png`
- **Placement**: `04-claude-md-context-files.md` - After "## Context Architecture" section
- **Alt Text**: Four-layer context architecture showing Working Directory (base), .claude/context files (project knowledge), Message History (conversation state), and Current Tools (active capabilities), with information flow arrows
- **Purpose**: Explain how Claude Code builds comprehensive context

### Visual 3: Claude Code Installation & Authentication Flowchart

- **File**: `/img/part-2/chapter-05/claude-code-installation-authentication-flow.png`
- **Placement**: `02-installation-and-authentication.md` - After "## Installation Process" header
- **Alt Text**: Installation flowchart showing Node.js check, npx/npm install options, authentication with API key or web login, and verification steps with decision points and success/error paths
- **Purpose**: Guide students through installation decision points

### Visual 4: Skills & Subagents Hierarchy Tree

- **File**: `/img/part-2/chapter-05/skills-subagents-hierarchy-tree.png`
- **Placement**: `06-subagents-and-orchestration.md` - After "## Understanding the Hierarchy" section
- **Alt Text**: Three-tier hierarchy tree showing Claude Code (orchestrator) at top, Subagents (specialized agents) in middle tier, and Skills (reusable capabilities) at bottom, with delegation arrows and example instances
- **Purpose**: Visualize orchestration architecture

### Visual 5: Settings Hierarchy (Global → Project → File)

- **File**: `/img/part-2/chapter-05/settings-hierarchy-global-project-file.png`
- **Placement**: `09-settings-hierarchy.md` - After "## Three Levels of Configuration" header
- **Alt Text**: Pyramid showing three configuration levels—Global settings (base, system-wide defaults), Project settings (middle, .claude/config for repo), File settings (top, frontmatter overrides)—with precedence arrows showing file > project > global
- **Purpose**: Clarify configuration precedence rules

---

## Chapter 6: Gemini CLI Installation and Basics

**Chapter Directory**: `apps/learn-app/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/`

### Visual 6: Gemini 2.5 Pro Feature Comparison Matrix

- **File**: `/img/part-2/chapter-06/gemini-2-5-pro-feature-matrix.png`
- **Placement**: Chapter README.md - After "## What You'll Learn" section
- **Alt Text**: Feature comparison matrix showing Gemini 2.5 Pro capabilities (2M token context, native code execution, multimodal input, speed benchmarks) compared to previous versions and competitors
- **Purpose**: Establish Gemini's value proposition

### Visual 7: CLI vs Web Interface Workflow

- **File**: `/img/part-2/chapter-06/cli-vs-web-workflow-comparison.png`
- **Placement**: First lesson introducing CLI vs web (create if doesn't exist, or in README)
- **Alt Text**: Side-by-side comparison showing web interface workflow (browser, mouse clicks, copy-paste) versus CLI workflow (terminal, keyboard commands, direct file access) with efficiency indicators
- **Purpose**: Motivate CLI learning for developers

### Visual 8: Gemini CLI Installation & Setup Flowchart

- **File**: `/img/part-2/chapter-06/gemini-cli-installation-setup-flow.png`
- **Placement**: Installation lesson - After "## Installation Steps" header
- **Alt Text**: Installation flowchart showing Python check, pip/uv install options, API key configuration, authentication verification, and troubleshooting branches
- **Purpose**: Guide installation with decision points

### Visual 9: Tool Comparison Matrix (Claude Code, Gemini CLI, Cursor, Zed)

- **File**: `/img/part-2/chapter-06/tool-comparison-matrix-four-tools.png`
- **Placement**: Final lesson or README - "## Choosing the Right Tool" section
- **Alt Text**: Comparison matrix showing Claude Code, Gemini CLI, Cursor, and Zed across dimensions: autonomy level, context size, multimodal support, offline capability, learning curve, and ideal use cases
- **Purpose**: Help students select appropriate tools

---

## Chapter 7: Bash Essentials

**Chapter Directory**: `apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/`

### Visual 10: Terminal Anatomy (Prompt, Command, Arguments, Output)

- **File**: `/img/part-2/chapter-07/terminal-anatomy-prompt-command-output.png`
- **Placement**: First lesson - After "## Understanding the Terminal" header
- **Alt Text**: Annotated terminal screenshot showing prompt (username@host), command (ls), arguments (-la /home), and output (file listing), with labels explaining each component and their purposes
- **Purpose**: Demystify terminal interface components

### Visual 11: File System Navigation Tree

- **File**: `/img/part-2/chapter-07/file-system-navigation-tree.png`
- **Placement**: Navigation lesson - After "## File System Structure" section
- **Alt Text**: File system tree showing root (/), home directory, common paths (Documents, Downloads), with pwd current location indicator and cd navigation arrows
- **Purpose**: Visualize directory hierarchy and navigation

### Visual 12: Common Bash Commands Reference Card

- **File**: `/img/part-2/chapter-07/common-bash-commands-reference.png`
- **Placement**: Commands lesson - End of lesson as reference
- **Alt Text**: Reference card showing 12 essential commands (ls, cd, pwd, mkdir, rm, cp, mv, cat, grep, find) with syntax, common flags, and practical examples
- **Purpose**: Quick reference for common operations

### Visual 13: Environment Variables Flow

- **File**: `/img/part-2/chapter-07/environment-variables-flow.png`
- **Placement**: Environment variables lesson - After "## What Are Environment Variables" section
- **Alt Text**: Flow diagram showing shell startup reading .bashrc/.zshrc, loading environment variables (PATH, HOME, USER), and programs accessing these variables at runtime
- **Purpose**: Explain variable scope and lifecycle

### Visual 14: Bash Script Execution Workflow

- **File**: `/img/part-2/chapter-07/bash-script-execution-workflow.png`
- **Placement**: Scripting lesson - After "## Running Scripts" section
- **Alt Text**: Workflow showing script creation, chmod +x permission setting, shebang interpretation (#!/bin/bash), execution methods (./script.sh vs bash script.sh), and output capture
- **Purpose**: Clarify script execution mechanics

---

## Chapter 8: AI-Native IDEs

**Chapter Directory**: `apps/learn-app/docs/02-AI-Tool-Landscape/08-ai-native-ides/`

### Visual 15: IDE Comparison Matrix (VS Code, Cursor, Zed)

- **File**: `/img/part-2/chapter-08/ide-comparison-matrix-three-ides.png`
- **Placement**: README or first lesson - After "## Choosing an IDE" section
- **Alt Text**: Comparison matrix showing VS Code, Cursor, and Zed across: AI integration depth, performance, extensibility, learning curve, collaboration features, and ideal user profiles
- **Purpose**: Guide IDE selection based on needs

### Visual 16: AI Integration Architecture (How IDEs Connect to AI Models)

- **File**: `/img/part-2/chapter-08/ai-integration-architecture-ide-models.png`
- **Placement**: Architecture lesson - After "## How AI Integration Works" header
- **Alt Text**: Architecture diagram showing IDE sending code context to AI service, model processing with retrieval/reasoning, response generation, and integration back into editor with inline suggestions, chat, and refactoring
- **Purpose**: Demystify AI-IDE communication

### Visual 17: IDE Selection Decision Tree

- **File**: `/img/part-2/chapter-08/ide-selection-decision-tree.png`
- **Placement**: Final lesson - "## Making Your Choice" section
- **Alt Text**: Decision tree with questions: Need extensions? (Yes→VS Code, No→continue), Need max speed? (Yes→Zed, No→Cursor), Want AI-first experience? (Yes→Cursor, No→VS Code), leading to recommended IDE
- **Purpose**: Systematic IDE selection process

---

## Chapter 9: Git and GitHub

**Chapter Directory**: `apps/learn-app/docs/02-AI-Tool-Landscape/09-git-and-github/`

### Visual 18: Git Three-Stage Workflow (Working → Staging → Repository)

- **File**: `/img/part-2/chapter-09/git-three-stage-workflow.png`
- **Placement**: Git basics lesson - After "## Understanding Git's Three Stages" header
- **Alt Text**: Three-stage diagram showing Working Directory (modified files), Staging Area (git add, files ready to commit), and Repository (git commit, permanent snapshots), with command arrows and file state transitions
- **Purpose**: Core Git mental model

### Visual 19: Git Branching Strategy (Feature Branches → Main)

- **File**: `/img/part-2/chapter-09/git-branching-strategy-feature-main.png`
- **Placement**: Branching lesson - After "## Why Branch?" section
- **Alt Text**: Branch diagram showing main branch (stable) with feature branches (feature-auth, feature-ui) diverging for development, then merging back via pull requests after review
- **Purpose**: Visualize parallel development workflow

### Visual 20: Pull Request Lifecycle (Create → Review → Merge)

- **File**: `/img/part-2/chapter-09/pull-request-lifecycle.png`
- **Placement**: Pull request lesson - After "## Pull Request Process" header
- **Alt Text**: Lifecycle flowchart showing PR creation (branch → GitHub), review phase (comments, requested changes, approvals), merge decision (squash, rebase, or merge commit), and branch cleanup
- **Purpose**: Clarify PR workflow stages

### Visual 21: Merge Conflict Resolution Anatomy

- **File**: `/img/part-2/chapter-09/merge-conflict-resolution-anatomy.png`
- **Placement**: Conflict resolution lesson - After "## Understanding Conflicts" section
- **Alt Text**: Annotated conflict showing <<<<<<< HEAD (your changes), ======= (divider), >>>>>>> branch (their changes), with resolution steps: identify conflict markers, choose changes, remove markers, test, commit
- **Purpose**: Decode conflict syntax

### Visual 22: AI-Assisted Git Workflow (How AI Helps with Commits, PRs, Conflicts)

- **File**: `/img/part-2/chapter-09/ai-assisted-git-workflow.png`
- **Placement**: AI collaboration lesson - After "## AI as Git Partner" header
- **Alt Text**: Workflow showing AI helping at each stage: analyzing diffs for commit messages, reviewing PR changes for suggestions, resolving conflicts by understanding context, with Three Roles framework (Teacher/Student/Co-Worker) highlighted
- **Purpose**: Demonstrate AI collaboration points

---

## Chapter 10: Markdown Language of AI

**Chapter Directory**: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/10-markdown-language-of-ai/`

### Visual 23: Markdown Syntax Anatomy (Headers, Lists, Code, Links)

- **File**: `/img/part-3/chapter-10/markdown-syntax-anatomy-four-elements.png`
- **Placement**: Syntax lesson - After "## Core Syntax Elements" header
- **Alt Text**: Side-by-side showing markdown source (# Header, - list, `code`, [link](url)) and rendered output with annotations explaining syntax purpose and AI interpretation
- **Purpose**: Teach basic markdown syntax

### Visual 24: Plain Text vs Rendered Markdown Comparison

- **File**: `/img/part-3/chapter-10/plain-text-vs-rendered-markdown.png`
- **Placement**: Why Markdown lesson - After "## Why Markdown?" section
- **Alt Text**: Split comparison showing plain text document (cluttered, no structure) versus rendered markdown (clear hierarchy, formatted), emphasizing readability and AI parsing advantages
- **Purpose**: Motivate markdown learning

### Visual 25: Code Block Examples (Syntax Highlighting by Language)

- **File**: `/img/part-3/chapter-10/code-block-syntax-highlighting.png`
- **Placement**: Code blocks lesson - After "## Language-Specific Highlighting" section
- **Alt Text**: Four code blocks showing syntax highlighting for Python, JavaScript, Bash, and JSON, with language identifiers (`python, `js) and color-coded keywords, strings, and comments
- **Purpose**: Demonstrate language-specific formatting

### Visual 26: Markdown as Intent Layer (Structure Before AI Generation)

- **File**: `/img/part-3/chapter-10/markdown-intent-layer.png`
- **Placement**: AI collaboration lesson - After "## Markdown as Specification" header
- **Alt Text**: Two-layer diagram showing markdown structure layer (headers, sections, placeholders) defining intent, then AI generation layer filling content while preserving structure, demonstrating spec-driven content creation
- **Purpose**: Teach specification-first approach

---

## Chapter 11: Prompt Engineering for AIDD

**Chapter Directory**: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/11-prompt-engineering-for-aidd/`

### Visual 27: Vague vs Clear Specification Comparison

- **File**: `/img/part-3/chapter-11/vague-vs-clear-specification-comparison.png`
- **Placement**: Specification quality lesson - After "## Quality Matters" header
- **Alt Text**: Side-by-side prompt comparison showing vague prompt (left, red, "Create auth" with incomplete AI output missing security features) versus clear prompt (right, green, "JWT auth with bcrypt, 24h expiry, rate limiting" with complete AI implementation)
- **Purpose**: Demonstrate specification impact on output

### Visual 28: Prompt Pattern Templates (Persona, Task, Context, Format)

- **File**: `/img/part-3/chapter-11/prompt-pattern-template-four-components.png`
- **Placement**: Template lesson - After "## Reusable Patterns" section
- **Alt Text**: Four-component template showing Persona (purple, "Act as [expert]"), Task (blue, "Create [deliverable]"), Context (orange, "For [use case] with [constraints]"), Format (green, "Output as [structure]"), with filled example
- **Purpose**: Provide reusable prompt structure

### Visual 29: Iteration Loop (Initial Prompt → Review → Refine → Better Output)

- **File**: `/img/part-3/chapter-11/prompt-iteration-refinement-loop.png`
- **Placement**: Iteration lesson - After "## Iteration is Normal" header
- **Alt Text**: Circular workflow showing Initial Prompt (rough), AI Output (initial result), Review (identify gaps), Refine Prompt (add constraints), spiraling upward to Better Output, emphasizing learning through practice
- **Purpose**: Normalize iteration mindset

---

## Chapter 12: Context Engineering for AI-Driven Development

**Chapter Directory**: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/12-context-engineering-for-ai-driven-development/`

### Visual 30: Context Hierarchy (Always Relevant → Task-Specific → One-Time)

- **File**: `/img/part-3/chapter-12/context-hierarchy-pyramid-three-tiers.png`
- **Placement**: Hierarchy lesson - After "## Three Tiers of Context" header
- **Alt Text**: Pyramid showing Always Relevant (base, README, architecture), Task-Specific (middle, feature files), One-Time (top, rare references), with width representing frequency and importance
- **Purpose**: Teach context prioritization

### Visual 31: File Importance Matrix (High Impact × High Frequency)

- **File**: `/img/part-3/chapter-12/file-importance-matrix-impact-frequency.png`
- **Placement**: Priority lesson - After "## Prioritizing Context" section
- **Alt Text**: 2×2 matrix with quadrants—High Impact × High Frequency (green, always load), High Impact × Low Frequency (blue, load when critical), Low Impact × High Frequency (yellow, quick reference), Low Impact × Low Frequency (gray, rarely load)
- **Purpose**: Context selection framework

### Visual 32: Decision Preservation (Why We Made This Choice)

- **File**: `/img/part-3/chapter-12/decision-preservation-timeline-why-documentation.png`
- **Placement**: Documentation lesson - After "## Why Document Decisions?" header
- **Alt Text**: Timeline showing Present (code + decision documented), 6 Months Later with two paths—With Decision (green, future developer understands), Without Decision (red, confusion and rework)
- **Purpose**: Motivate decision documentation

---

## Chapter 13: Python UV Package Manager

**Chapter Directory**: `apps/learn-app/docs/04-Python-Fundamentals/13-python-uv-package-manager/`

### Visual 33: UV Workflow (Install → Create Venv → Install Packages → Run)

- **File**: `/img/part-4/chapter-13/uv-workflow-four-step-setup.png`
- **Placement**: `01-why-uv-understanding-modern-package-management.md` - After "## The UV Workflow" section
- **Alt Text**: Four-step workflow showing Install UV (curl/brew), Create venv (uv venv), Install packages (uv pip install), Run code (uv run app.py), with speed indicators and success checkpoints
- **Purpose**: Visualize UV's simplified workflow

### Visual 34: Virtual Environment Isolation Concept

- **File**: `/img/part-4/chapter-13/virtual-environment-isolation-concept.png`
- **Placement**: `03-creating-first-uv-project-with-ai.md` - After "## Why Virtual Environments?" section
- **Alt Text**: Three-tier diagram showing Global Python (gray, system-wide with conflict indicators) versus isolated Venv A (blue, Django 3) and Venv B (green, Django 4), with arrows demonstrating dependency isolation
- **Purpose**: Explain isolation benefits

---

## Chapter 14: Introduction to Python

**Chapter Directory**: `apps/learn-app/docs/04-Python-Fundamentals/14-introduction-to-python/`

### Visual 35: Python REPL Anatomy (>>> Prompt, Input, Output)

- **File**: `/img/part-4/chapter-14/python-repl-anatomy-interactive-loop.png`
- **Placement**: REPL lesson - After "## Using the Python REPL" header
- **Alt Text**: Annotated REPL showing >>> prompt (blue, waiting), user input 2+2 (green, expression), output 4 (gray, result), next >>> prompt, with labels explaining Read-Eval-Print-Loop cycle
- **Purpose**: Demystify REPL interface

### Visual 36: Variable Memory Model (Name → Value in Memory)

- **File**: `/img/part-4/chapter-14/variable-memory-model-name-value-pointers.png`
- **Placement**: Variables lesson - After "## How Variables Work" section
- **Alt Text**: Memory diagram showing variable names (blue, x, y, z) as separate from memory values (green, 5, 10, "hello"), with purple arrows indicating pointers, and reassignment showing arrow changing targets
- **Purpose**: Teach reference semantics

---

## Chapter 15: Data Types

**Chapter Directory**: `apps/learn-app/docs/04-Python-Fundamentals/15-data-types/`

### Visual 37: Python Data Types Hierarchy (Primitive → Collections → Custom)

- **File**: `/img/part-4/chapter-15/python-data-types-hierarchy-three-tiers.png`
- **Placement**: Type hierarchy lesson - After "## Organizing Python Types" header
- **Alt Text**: Pyramid showing Primitives (blue base, int/float/str/bool), Collections (green middle, list/dict/set/tuple), Custom (purple top, classes), with examples demonstrating progression from simple to complex
- **Purpose**: Provide type organization framework

### Visual 38: Type Casting Flow (str → int, int → float, etc.)

- **File**: `/img/part-4/chapter-15/type-casting-flow-valid-invalid-conversions.png`
- **Placement**: Type casting lesson - After "## Converting Between Types" section
- **Alt Text**: Network diagram showing types (blue nodes: str, int, float, bool) connected by casting functions with green edges (valid: int("42"), str(42)) and red edges (invalid: int("hello"), float("text"))
- **Purpose**: Teach type conversion rules

---

## Embedding Execution Plan

### Phase 1: Verify Visual Files (DONE)

✅ All 38 visuals confirmed in `book-source/static/img/` directories

### Phase 2: Embed Markdown References (NEXT)

1. Read lesson file
2. Identify placement section (match header from mapping)
3. Insert markdown reference after section header
4. Save edited file
5. Repeat for all 38 visuals

### Phase 3: Validation

1. Check all 38 references embedded
2. Verify image paths correct (relative to docs root)
3. Confirm alt text included
4. Test sample pages render correctly

---

**Mapping Complete**: 38 visuals mapped to specific lesson files and sections
**Next Step**: Execute Phase 2 embedding workflow
