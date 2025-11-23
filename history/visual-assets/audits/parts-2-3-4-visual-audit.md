# Visual Asset Audit: Parts 2, 3, 4

**Date**: 2025-11-22
**Scope**: 25 chapters (Chapters 5-30)
**Total Visuals**: 63 identified
**Status**: ✅ APPROVED - Ready for autonomous batch generation

---

## PART 2: AI Tool Landscape (Chapters 5-9)

### Chapter 5: Claude Code Features and Workflows

**Proficiency**: B1 | **Layer**: L1-L2 | **Visuals**: 5

---

#### VISUAL 1: Traditional Chat AI vs Agentic Claude Code

**Status**: ✅ APPROVED

## The Story
Traditional AI chat tools (ChatGPT, Claude.ai web) exist in isolated conversations—copy code, paste into terminal, hope it works. Claude Code transforms this into an agentic system: AI sees your files, executes commands, reads errors, iterates autonomously. This is the shift from "assistant" to "autonomous agent."

## Emotional Intent
Should feel: Liberation from copy-paste drudgery, surprise at autonomy
Visual mood: Constrained (left) → Empowered (right)

## Visual Metaphor
Breaking free from handcuffs (manual copy-paste) to autonomous operation (agent with full context and execution)

## Key Insight to Emphasize
"Chat AI requires constant manual intervention. Claude Code operates autonomously with full context."

## Subject
Two-column comparison showing workflow transformation

## Composition
Split-screen comparison (50/50):
- Left: Traditional chat workflow (browser → manual copy → terminal → error → back to browser)
- Right: Claude Code workflow (AI reads files → executes commands → sees errors → iterates automatically)

## Action
Visual flow showing information transfer bottlenecks (left) vs seamless integration (right)

## Location/Context
Developer workspace comparison

## Style
Modern software comparison aesthetic (clear, not corporate/boring)
Reference: Linear.app comparison pages, Notion feature demonstrations

## Camera Perspective
Straight-on orthographic view (no distortion)

## Lighting
Flat even lighting (focus on workflow clarity, not visual effects)

## Color Semantics
- Red (#ef4444) = Bottleneck/friction (manual copy-paste steps on left)
- Blue (#3b82f6) = Automation/flow (autonomous steps on right)
- Gray (#6b7280) = Context isolation (traditional chat)
- Green (#10b981) = Context-aware (Claude Code with file access)

## Typography Hierarchy
- Largest: "Autonomous" vs "Manual" (core distinction)
- Medium: Step labels (workflow descriptions)
- Smallest: Technical details (file paths, command examples)

## Text Integration
Labels directly on workflow arrows and boxes (separation fragments understanding)

## Resolution
2K standard (web documentation)

## Teaching Goal
Teach workflow transformation: isolated chat assistant → autonomous agentic system

## Proficiency
B1: Moderate complexity (2 workflows compared, 6-8 steps each)

## Visual Type
Static comparison diagram

## Google Search Grounding
No (conceptual workflow, not factual specifications)

## Pedagogical Reasoning
Split-screen composition creates instant visual contrast (left=constrained, right=empowered). Color coding teaches through semantics: red bottlenecks make friction visible, green flow shows automation. Typography emphasizes core transformation (autonomous vs manual). This teaches the fundamental shift from passive tool to active agent.

**FILENAME**: `traditional-chat-vs-claude-code-workflow.png`

**ALT TEXT**: Split-screen workflow comparison showing traditional chat AI requiring manual copy-paste steps (left, red bottlenecks) versus Claude Code's autonomous workflow with file context and command execution (right, green flow), with visual sizing emphasizing the autonomous vs manual distinction

---

#### VISUAL 2: Claude Code Context Architecture

**Status**: ✅ APPROVED

## The Story
Claude Code isn't just a chatbot—it's a context-aware orchestration system. Terminal integration, file reading, MCP server tools, and specialized subagents compose into an agentic architecture that sees, understands, and acts on your entire development environment.

## Emotional Intent
Should feel: Architectural clarity, system intelligence
Visual mood: Organized layers, purposeful connections

## Visual Metaphor
Orchestra conductor coordinating specialized musicians (central Claude Code orchestrating terminal, files, MCP, subagents)

## Key Insight to Emphasize
"Claude Code orchestrates multiple context sources into unified understanding"

## Subject
Multi-layer architecture diagram showing Claude Code at center coordinating context sources

## Composition
Concentric circles or hierarchical layers:
- Center: Claude Code (core orchestrator)
- Inner ring: Terminal integration, File system access
- Outer ring: MCP servers, Subagents, Skills
- Arrows showing bidirectional communication

## Action
Information flowing from context sources → Claude Code → coordinated actions

## Location/Context
System architecture diagram (technical documentation style)

## Style
Modern cloud-native architecture aesthetic
Reference: AWS/Azure architecture diagrams, Kubernetes docs

## Camera Perspective
Orthographic top-down view (system overview perspective)

## Lighting
Flat even lighting (technical accuracy prioritized)

## Color Semantics
- Purple (#8b5cf6) = Core intelligence (Claude Code orchestrator)
- Blue (#3b82f6) = System integration (Terminal, Files)
- Green (#10b981) = Extensibility (MCP servers, Skills)
- Orange (#f97316) = Specialized agents (Subagents)

## Typography Hierarchy
- Largest: "Claude Code" (central orchestrator)
- Medium: Context sources (Terminal, Files, MCP, Subagents)
- Smallest: Capability descriptions (what each layer provides)

## Text Integration
Labels on diagram components with connecting arrows showing data flow

## Resolution
2K standard

## Teaching Goal
Teach architectural understanding: Claude Code as multi-source context orchestrator, not simple chatbot

## Proficiency
B1: Moderate complexity (4 layers, 8-10 components)

## Visual Type
Static architecture diagram

## Google Search Grounding
No (conceptual architecture)

## Pedagogical Reasoning
Concentric or hierarchical layout teaches system structure through spatial organization. Central positioning of Claude Code shows orchestration role. Color coding creates instant category recognition (core vs integration vs extensibility). Typography hierarchy guides attention from core concept (Claude Code) to supporting systems. This transforms abstract "context-aware" into concrete visual architecture.

**FILENAME**: `claude-code-context-architecture.png`

**ALT TEXT**: Multi-layer architecture diagram showing Claude Code at center orchestrating Terminal integration, File system access, MCP servers, and Subagents through bidirectional communication flows, with color-coded layers emphasizing core intelligence (purple), system integration (blue), and extensibility (green)

---

#### VISUAL 3: Claude Code Installation & Authentication Flowchart

**Status**: ✅ APPROVED

## The Story
Getting Claude Code running requires three precise steps: install via Homebrew, set API key, verify authentication. Miss one step, nothing works. Get them right, unlock autonomous development.

## Emotional Intent
Should feel: Clear, achievable, step-by-step confidence
Visual mood: Guided journey, success-oriented

## Visual Metaphor
Assembly instructions with visual checkpoints (like IKEA diagrams—clear visual steps, minimal text)

## Key Insight to Emphasize
"Three steps to autonomous development: Install → Authenticate → Verify"

## Subject
Linear flowchart with decision points and verification checkpoints

## Composition
Vertical or left-to-right flow:
1. Install Homebrew (if needed)
2. Install Claude Code (`brew install claude`)
3. Set API key (environment variable or config file)
4. Verify authentication (`claude --version`, test prompt)

Decision diamonds for "Already have Homebrew?" and "Authentication works?"

## Action
User progressing through installation steps with success/failure branches

## Location/Context
Terminal setup workflow (technical onboarding)

## Style
Modern developer documentation aesthetic
Reference: GitHub setup guides, Vercel deployment docs

## Camera Perspective
Straight-on orthographic (flowchart clarity)

## Lighting
Flat even lighting

## Color Semantics
- Green (#10b981) = Success checkpoints (✓ marks)
- Red (#ef4444) = Failure branches (troubleshooting)
- Blue (#3b82f6) = Process steps (action rectangles)
- Gray (#6b7280) = Decision points (diamonds)

## Typography Hierarchy
- Largest: Step numbers (1, 2, 3—progression tracking)
- Medium: Action descriptions (what to do)
- Smallest: Command examples (`brew install claude`)

## Text Integration
Commands and steps embedded in flowchart shapes

## Resolution
2K standard

## Teaching Goal
Teach installation workflow: clear step sequence with decision points and verification

## Proficiency
B1: Moderate complexity (4 main steps, 2 decision points)

## Visual Type
Static flowchart

## Google Search Grounding
No (procedural workflow, not factual claims)

## Pedagogical Reasoning
Linear flow creates step-by-step confidence (no guessing what's next). Decision diamonds acknowledge troubleshooting (not hiding complexity). Green checkmarks provide psychological progress markers. Typography hierarchy guides scanning: step numbers → actions → commands. This reduces installation anxiety through visual clarity.

**FILENAME**: `claude-code-installation-flowchart.png`

**ALT TEXT**: Installation flowchart showing three-step setup process (Install Homebrew → Install Claude → Authenticate → Verify) with decision points for existing setup and verification checkpoints marked with green success indicators

---

#### VISUAL 4: Skills & Subagents Hierarchy Tree

**Status**: ✅ APPROVED

## The Story
Claude Code doesn't do everything itself—it orchestrates specialized tools. Skills provide domain expertise (frontend design, testing, documentation). Subagents handle complex multi-step tasks (code review, refactoring, debugging). This is distributed intelligence, not monolithic AI.

## Emotional Intent
Should feel: Organized specialization, delegated expertise
Visual mood: Tree structure showing specialization branches

## Visual Metaphor
Corporate org chart but for AI capabilities (CEO delegates to specialized departments)

## Key Insight to Emphasize
"Claude Code delegates specialized work to Skills and Subagents"

## Subject
Hierarchical tree showing Claude Code at top, Skills and Subagents as specialized branches

## Composition
Tree structure:
- Root: Claude Code (orchestrator)
- First branches: Skills (domain expertise) | Subagents (complex workflows)
- Second branches: Specific examples (frontend-design skill, code-reviewer subagent, etc.)

## Action
Delegation flow from orchestrator to specialists

## Location/Context
Capability architecture (technical system diagram)

## Style
Modern organizational chart aesthetic with technical twist
Reference: Company org charts but with code/tech theming

## Camera Perspective
Top-down tree view (hierarchy visualization)

## Lighting
Flat even lighting

## Color Semantics
- Purple (#8b5cf6) = Orchestrator (Claude Code root)
- Blue (#3b82f6) = Skills (domain-specific guidance)
- Orange (#f97316) = Subagents (autonomous task execution)

## Typography Hierarchy
- Largest: "Claude Code" (root orchestrator)
- Medium: Category labels (Skills, Subagents)
- Smallest: Specific examples (skill names, subagent names)

## Text Integration
Labels on tree nodes with connecting lines

## Resolution
2K standard

## Teaching Goal
Teach distributed intelligence: Claude Code orchestrates specialized Skills and Subagents

## Proficiency
B1: Moderate complexity (3 levels, 6-8 examples)

## Visual Type
Static tree diagram

## Google Search Grounding
No (conceptual hierarchy)

## Pedagogical Reasoning
Tree structure teaches through spatial hierarchy (top=orchestrator, branches=specialists). Color coding creates instant categorization (skills vs subagents). Typography hierarchy guides attention from core to categories to examples. This makes abstract "orchestration" concrete through familiar organizational structure.

**FILENAME**: `skills-subagents-hierarchy-tree.png`

**ALT TEXT**: Hierarchical tree diagram showing Claude Code at top orchestrating specialized Skills (blue branches for domain expertise like frontend-design) and Subagents (orange branches for complex workflows like code-reviewer), with visual sizing emphasizing the orchestration hierarchy

---

#### VISUAL 5: Settings Hierarchy (Global → Project → File)

**Status**: ✅ APPROVED

## The Story
Claude Code settings cascade through three layers: global defaults (apply everywhere), project overrides (specific repository), file-level specifics (individual documents). Understanding this hierarchy prevents configuration mysteries—why does this work here but not there?

## Emotional Intent
Should feel: Layered control, override clarity
Visual mood: Waterfall cascade, specificity increasing

## Visual Metaphor
Russian nesting dolls or CSS cascade (broader rules → specific overrides)

## Key Insight to Emphasize
"Settings cascade: Global defaults ← Project overrides ← File specifics"

## Subject
Three-layer hierarchy showing settings inheritance and override flow

## Composition
Vertical cascade or concentric circles:
- Outermost/Top: Global settings (.claude/config.json)
- Middle: Project settings (.claude/ in repo)
- Innermost/Bottom: File-level settings (frontmatter, inline configs)

Arrows showing inheritance and override direction

## Action
Settings flowing from global → project → file with override indicators

## Location/Context
Configuration hierarchy (technical documentation)

## Style
Modern developer documentation aesthetic
Reference: CSS specificity diagrams, config inheritance docs

## Camera Perspective
Orthographic (clarity for hierarchy)

## Lighting
Flat even lighting

## Color Semantics
- Gray (#6b7280) = Global (broadest, default)
- Blue (#3b82f6) = Project (repository-specific)
- Green (#10b981) = File (most specific, highest priority)

Color intensity increases with specificity (teaching hierarchy through visual weight)

## Typography Hierarchy
- Largest: Layer labels (Global, Project, File)
- Medium: Setting examples (what can be configured)
- Smallest: File paths (.claude/config.json, etc.)

## Text Integration
Labels on hierarchy layers with example settings

## Resolution
2K standard

## Teaching Goal
Teach configuration hierarchy: global defaults cascade to project then file-level overrides

## Proficiency
B1: Moderate complexity (3 layers, 4-6 setting examples)

## Visual Type
Static hierarchy diagram

## Google Search Grounding
No (conceptual configuration model)

## Pedagogical Reasoning
Vertical cascade or concentric circles teach specificity through visual layering. Color intensity progression (gray→blue→green) reinforces "more specific = higher priority." Typography hierarchy guides from concept (layer names) to examples. This demystifies configuration through spatial metaphor (outer=broad, inner=specific).

**FILENAME**: `settings-hierarchy-cascade.png`

**ALT TEXT**: Three-layer settings hierarchy showing global defaults (gray, outer) cascading to project overrides (blue, middle) and file-level specifics (green, inner), with arrows indicating inheritance flow and color intensity teaching specificity priority

---

### Chapter 6: Gemini CLI Installation and Basics

**Proficiency**: B1 | **Layer**: L1-L2 | **Visuals**: 4

---

#### VISUAL 6: Gemini 2.5 Pro Feature Comparison Matrix

**Status**: ✅ APPROVED

## The Story
Gemini 2.5 Pro isn't just "another AI"—it brings specific strengths: 2M token context window (vs Claude's 200K), native multimodal (image/video/audio), Google Search grounding for real-time facts. Understanding these differences helps choose the right tool for each task.

## Emotional Intent
Should feel: Informed comparison, clear differentiation
Visual mood: Side-by-side technical specs, objective assessment

## Visual Metaphor
Product specification sheet comparison (like car or phone comparisons—specs laid out for direct comparison)

## Key Insight to Emphasize
"Gemini 2.5 Pro: Massive context (2M tokens), multimodal native, Search-grounded"

## Subject
Feature comparison table: Gemini 2.5 Pro vs Claude Sonnet 4

## Composition
Side-by-side comparison grid:
- Rows: Features (context window, multimodal, grounding, code execution, etc.)
- Columns: Gemini 2.5 Pro | Claude Sonnet 4
- Checkmarks, numbers, and descriptions in cells

## Action
User scanning horizontally to compare features across models

## Location/Context
Technical feature matrix (documentation/decision-making)

## Style
Modern product comparison aesthetic
Reference: Apple product comparison pages, tech spec sheets

## Camera Perspective
Straight-on orthographic (table clarity)

## Lighting
Flat even lighting

## Color Semantics
- Green (#10b981) = Checkmark (feature present)
- Red (#ef4444) = X mark or "Limited" (feature absent/restricted)
- Blue (#3b82f6) = Numerical specs (2M tokens, 200K tokens)
- Gray (#6b7280) = Neutral text

## Typography Hierarchy
- Largest: Feature categories (Context, Multimodal, Grounding)
- Medium: Model names (Gemini 2.5 Pro, Claude Sonnet 4)
- Smallest: Specific values (2M tokens, image/video/audio)

## Text Integration
Table cells contain checkmarks, numbers, and brief descriptions

## Resolution
2K standard

## Teaching Goal
Teach model differentiation: understand Gemini's strengths vs Claude's to choose appropriately

## Proficiency
B1: Moderate complexity (5-7 feature rows, 2 columns)

## Visual Type
Static comparison table

## Google Search Grounding
Yes (verify current model specifications: context limits, multimodal capabilities)

## Pedagogical Reasoning
Grid structure enables direct feature-by-feature comparison (horizontal scanning). Color coding creates instant recognition (green checkmarks=yes, red X=no). Typography hierarchy guides from categories to models to specs. This reduces decision paralysis by making trade-offs visually explicit.

**FILENAME**: `gemini-2-5-pro-feature-comparison.png`

**ALT TEXT**: Feature comparison matrix showing Gemini 2.5 Pro vs Claude Sonnet 4 across context window (2M vs 200K tokens), multimodal capabilities (native vs limited), Search grounding (built-in vs none), with green checkmarks and red X marks visualizing feature availability

---

#### VISUAL 7: CLI vs Web Interface Workflow

**Status**: ✅ APPROVED

## The Story
Web interfaces (gemini.google.com) require browser tabs, manual copy-paste, and context switching. Gemini CLI stays in terminal—pipe outputs, chain commands, integrate with scripts. This is the shift from UI-driven to command-driven AI.

## Emotional Intent
Should feel: Efficiency gained, terminal fluency
Visual mood: Constrained (web) → Powerful (CLI)

## Visual Metaphor
GUI training wheels vs direct control (like automatic vs manual transmission)

## Key Insight to Emphasize
"Web interface = Manual clicks. CLI = Scriptable automation."

## Subject
Workflow comparison: web browser steps vs CLI command pipeline

## Composition
Split-screen or sequential comparison:
- Left/Top: Web workflow (open browser → paste prompt → wait → copy output → paste elsewhere)
- Right/Bottom: CLI workflow (single command pipeline: `cat file.md | gemini --prompt "summarize"`)

## Action
User actions in web (many clicks) vs CLI (one command)

## Location/Context
Developer workflow comparison

## Style
Modern developer tool comparison
Reference: Command-line tool comparison docs, developer productivity guides

## Camera Perspective
Orthographic straight-on

## Lighting
Flat even lighting

## Color Semantics
- Orange (#f97316) = Manual steps (web clicking)
- Green (#10b981) = Automated flow (CLI piping)
- Gray (#6b7280) = Context switching (tab switching)

## Typography Hierarchy
- Largest: "Web" vs "CLI" (core distinction)
- Medium: Workflow step labels
- Smallest: Command examples (`gemini --prompt "..."`)

## Text Integration
Labels on workflow steps, command examples in terminal

## Resolution
2K standard

## Teaching Goal
Teach workflow efficiency: web requires manual steps, CLI enables automation through piping

## Proficiency
B1: Moderate complexity (4-5 steps web, 1-2 steps CLI)

## Visual Type
Static workflow comparison

## Google Search Grounding
No (conceptual workflow)

## Pedagogical Reasoning
Split-screen creates visual contrast (many steps vs few steps). Color coding teaches through semantics (orange manual friction vs green automated flow). Typography emphasizes core shift (Web vs CLI). This makes abstract "terminal efficiency" concrete through step counting.

**FILENAME**: `cli-vs-web-interface-workflow.png`

**ALT TEXT**: Workflow comparison showing web interface requiring multiple manual steps (browser tabs, copy-paste, context switching in orange) versus CLI's single command pipeline (terminal command with output piping in green), emphasizing automation efficiency

---

#### VISUAL 8: Gemini CLI Installation & Setup Flowchart

**Status**: ✅ APPROVED

## The Story
Gemini CLI installation requires Python environment (uv or pip), API key from Google AI Studio, and environment variable configuration. Three steps to autonomous terminal AI.

## Emotional Intent
Should feel: Clear steps, achievable setup
Visual mood: Guided installation confidence

## Visual Metaphor
Assembly instructions with checkpoints

## Key Insight to Emphasize
"Three steps: Install CLI → Get API key → Configure environment"

## Subject
Linear installation flowchart with verification checkpoints

## Composition
Vertical or left-to-right flow:
1. Install Python package (`uv pip install google-generativeai-cli`)
2. Get API key (visit ai.google.dev/gemini-api)
3. Set environment variable (GOOGLE_API_KEY)
4. Verify (`gemini --version`, test prompt)

Decision points for Python environment choice (uv vs pip)

## Action
User progressing through installation with success verification

## Location/Context
Terminal setup workflow

## Style
Modern developer onboarding docs
Reference: GitHub CLI setup, npm install guides

## Camera Perspective
Orthographic flowchart view

## Lighting
Flat even lighting

## Color Semantics
- Green (#10b981) = Success checkpoints (✓)
- Blue (#3b82f6) = Process steps
- Gray (#6b7280) = Decision points

## Typography Hierarchy
- Largest: Step numbers (1, 2, 3)
- Medium: Action descriptions
- Smallest: Commands (`uv pip install...`)

## Text Integration
Commands embedded in flowchart shapes

## Resolution
2K standard

## Teaching Goal
Teach installation sequence: package install → API key → environment config → verification

## Proficiency
B1: Moderate complexity (4 steps, 1 decision point)

## Visual Type
Static flowchart

## Google Search Grounding
No (procedural workflow)

## Pedagogical Reasoning
Linear flow reduces installation anxiety through clear sequencing. Green checkmarks provide progress markers. Typography guides from step numbers → actions → commands. This creates installation confidence through visual step-by-step guidance.

**FILENAME**: `gemini-cli-installation-flowchart.png`

**ALT TEXT**: Installation flowchart showing three-step Gemini CLI setup (Install package → Get API key from ai.google.dev → Set environment variable → Verify), with decision point for Python environment and green checkmarks indicating verification success

---

#### VISUAL 9: Tool Comparison Matrix (Claude Code, Gemini CLI, Cursor, Zed)

**Status**: ✅ APPROVED

## The Story
Four AI coding tools, four different philosophies: Claude Code (agentic terminal), Gemini CLI (open-source scriptable), Cursor (VS Code integrated), Zed (performance-first native). Choosing requires understanding trade-offs.

## Emotional Intent
Should feel: Informed decision-making, clear trade-offs
Visual mood: Objective comparison, no single "winner"

## Visual Metaphor
Swiss Army knife comparison—each tool has specific strengths for specific jobs

## Key Insight to Emphasize
"No one tool dominates—choose based on workflow needs"

## Subject
4-column comparison matrix across key dimensions

## Composition
Grid comparison:
- Columns: Claude Code | Gemini CLI | Cursor | Zed
- Rows: Autonomy level, IDE integration, Context window, Cost, Proficiency needed, Best for

## Action
User scanning to match workflow needs with tool strengths

## Location/Context
Tool selection guide (decision-making documentation)

## Style
Modern product comparison matrix
Reference: Software comparison sites, technical decision matrices

## Camera Perspective
Orthographic table view

## Lighting
Flat even lighting

## Color Semantics
- Purple (#8b5cf6) = High autonomy (Claude Code)
- Blue (#3b82f6) = Scriptable (Gemini CLI)
- Green (#10b981) = IDE-integrated (Cursor)
- Orange (#f97316) = Performance-focused (Zed)

## Typography Hierarchy
- Largest: Tool names (columns)
- Medium: Feature categories (rows)
- Smallest: Specific values (free/paid, A2/B1, etc.)

## Text Integration
Table cells with brief descriptions and icons

## Resolution
2K standard

## Teaching Goal
Teach tool selection: understand trade-offs to choose appropriate tool for workflow

## Proficiency
B1: Moderate complexity (4 tools, 6-7 comparison dimensions)

## Visual Type
Static comparison table

## Google Search Grounding
Yes (verify current pricing, features, context limits)

## Pedagogical Reasoning
Grid enables direct comparison across tools. Color coding creates tool identity (purple=autonomous, blue=scriptable, etc.). Typography guides from tool names to categories to specifics. This reduces choice overload by making trade-offs explicit.

**FILENAME**: `ai-coding-tools-comparison-matrix.png`

**ALT TEXT**: Feature comparison matrix showing Claude Code, Gemini CLI, Cursor, and Zed across autonomy level, IDE integration, context window, cost, and proficiency needed, with color coding emphasizing each tool's primary strength (purple=autonomous, blue=scriptable, green=integrated, orange=performance)

---

### Chapter 7: Bash Essentials

**Proficiency**: B1 | **Layer**: L1 | **Visuals**: 5

---

#### VISUAL 10: Terminal Anatomy (Prompt, Command, Arguments, Output)

**Status**: ✅ APPROVED

## The Story
The terminal looks cryptic to beginners—random characters, blinking cursor, mysterious outputs. But it's structured: prompt shows context, command is the action, arguments modify behavior, output shows results. Understanding this anatomy demystifies the terminal.

## Emotional Intent
Should feel: Clarity emerging from confusion, "aha!" moment
Visual mood: Labeled diagram revealing hidden structure

## Visual Metaphor
Anatomy diagram (like medical diagrams labeling body parts)

## Key Insight to Emphasize
"Terminal = Prompt (where you are) + Command (what to do) + Arguments (how to do it) + Output (results)"

## Subject
Annotated terminal screenshot showing all four components

## Composition
Terminal window with callout labels:
- Prompt: `user@machine:~/path$` (labeled: "Who & where you are")
- Command: `ls` (labeled: "Action to perform")
- Arguments: `-la /Users` (labeled: "Modifiers & targets")
- Output: file listing (labeled: "Results of command")

## Action
Visual flow from prompt → command → arguments → output (execution sequence)

## Location/Context
Terminal interface explanation (beginner onboarding)

## Style
Educational diagram aesthetic
Reference: Anatomy diagrams, labeled technical illustrations

## Camera Perspective
Straight-on screenshot with overlay annotations

## Lighting
Flat even (terminal clarity)

## Color Semantics
- Blue (#3b82f6) = Prompt (context info)
- Green (#10b981) = Command (action)
- Yellow (#fbbf24) = Arguments (modifiers)
- Gray (#6b7280) = Output (results)

## Typography Hierarchy
- Largest: Component labels (Prompt, Command, Arguments, Output)
- Medium: Descriptions ("Who & where you are")
- Smallest: Actual terminal text (`ls -la`)

## Text Integration
Callout labels with arrows pointing to terminal components

## Resolution
2K standard

## Teaching Goal
Teach terminal structure: demystify cryptic interface through component anatomy

## Proficiency
B1: Moderate complexity (4 components, clear annotations)

## Visual Type
Static annotated screenshot

## Google Search Grounding
No (conceptual terminal anatomy)

## Pedagogical Reasoning
Labeled anatomy creates "aha!" moment by revealing hidden structure. Color coding teaches component recognition (blue prompt, green command, etc.). Typography guides from concept labels to descriptions to examples. This transforms intimidating terminal into comprehensible structured interface.

**FILENAME**: `terminal-anatomy-annotated.png`

**ALT TEXT**: Annotated terminal screenshot showing four components—prompt (blue, showing user@machine:~/path), command (green, 'ls'), arguments (yellow, '-la /Users'), and output (gray, file listing)—with callout labels explaining each part's purpose

---

#### VISUAL 11: File System Navigation Tree

**Status**: ✅ APPROVED

## The Story
File systems are trees: root at top, directories branch down, files are leaves. Terminal navigation (cd, pwd, ls) is tree traversal. Understanding this spatial model makes terminal navigation intuitive instead of abstract.

## Emotional Intent
Should feel: Spatial clarity, mental model click
Visual mood: Map-like orientation, "I know where I am now"

## Visual Metaphor
Family tree or organizational chart (hierarchical structure everyone knows)

## Key Insight to Emphasize
"File system = Tree structure. Directories branch, files are leaves."

## Subject
Tree diagram showing typical file system with common navigation paths

## Composition
Hierarchical tree (top-down or left-to-right):
- Root: `/` (or `C:\` on Windows)
- Branches: `/Users`, `/Applications`, `/System`
- Sub-branches: `/Users/username`, `/Users/username/Documents`
- Leaves: Actual files (`file.txt`, `project.py`)

Navigation commands annotated on paths (`cd /Users/username`)

## Action
User mentally traversing tree structure with cd commands

## Location/Context
File system conceptual model (beginner orientation)

## Style
Clean tree diagram aesthetic
Reference: Directory tree visualizations, file system docs

## Camera Perspective
Top-down or left-to-right tree view

## Lighting
Flat even

## Color Semantics
- Purple (#8b5cf6) = Root (top-level)
- Blue (#3b82f6) = Directories (branches)
- Green (#10b981) = Files (leaves)

## Typography Hierarchy
- Largest: Root path (`/`)
- Medium: Directory names
- Smallest: File names

## Text Integration
Path labels on tree nodes, command examples on navigation paths

## Resolution
2K standard

## Teaching Goal
Teach file system mental model: tree structure makes navigation spatial and intuitive

## Proficiency
B1: Moderate complexity (3-4 tree levels, 6-8 nodes)

## Visual Type
Static tree diagram

## Google Search Grounding
No (conceptual file system model)

## Pedagogical Reasoning
Tree structure transforms abstract "directories" into familiar spatial hierarchy. Color coding creates instant categorization (purple root, blue directories, green files). Typography hierarchy mirrors actual file system (root largest, files smallest). This builds mental map for terminal navigation.

**FILENAME**: `file-system-navigation-tree.png`

**ALT TEXT**: Hierarchical tree diagram showing file system structure from root (/) through directories (/Users, /Applications) to subdirectories (/Users/username/Documents) and files, with color coding distinguishing root (purple), directories (blue), and files (green), annotated with cd navigation commands

---

#### VISUAL 12: Common Bash Commands Reference Card

**Status**: ✅ APPROVED

## The Story
Beginners overwhelmed by terminal freeze when they forget commands. A visual reference card with the 10-12 essential commands (ls, cd, pwd, mkdir, rm, mv, cp, cat, grep, chmod) creates confidence through quick lookup.

## Emotional Intent
Should feel: "I have a cheat sheet," confidence through reference
Visual mood: Quick-scan reference, not intimidating

## Visual Metaphor
Periodic table or keyboard shortcut card (visual reference you keep nearby)

## Key Insight to Emphasize
"12 commands cover 90% of terminal work"

## Subject
Grid of essential Bash commands with syntax and purpose

## Composition
Grid layout (3×4 or 2×6):
Each cell contains:
- Command name (ls, cd, pwd)
- Syntax (ls -la, cd /path)
- Purpose (list files, change directory)
- Example usage

## Action
User scanning grid to find needed command

## Location/Context
Quick reference guide (keep visible while learning)

## Style
Reference card aesthetic
Reference: Git cheat sheets, keyboard shortcut cards

## Camera Perspective
Straight-on grid view

## Lighting
Flat even

## Color Semantics
- Blue (#3b82f6) = Navigation commands (cd, pwd, ls)
- Green (#10b981) = File manipulation (mv, cp, rm, mkdir)
- Yellow (#fbbf24) = Content viewing (cat, grep)
- Orange (#f97316) = Permissions (chmod)

## Typography Hierarchy
- Largest: Command names (ls, cd, pwd)
- Medium: Syntax examples (ls -la)
- Smallest: Purpose descriptions ("list files")

## Text Integration
Each grid cell is self-contained command reference

## Resolution
2K standard (or 4K for print reference)

## Teaching Goal
Teach command literacy: provide quick reference reducing terminal anxiety

## Proficiency
B1: Moderate complexity (12 commands, categorized)

## Visual Type
Static reference card

## Google Search Grounding
No (standard Bash commands)

## Pedagogical Reasoning
Grid layout enables quick scanning by category. Color coding groups related commands (navigation, file ops, content, permissions). Typography hierarchy guides from command name → syntax → purpose. This reduces memorization burden through visual organization.

**FILENAME**: `bash-commands-reference-card.png`

**ALT TEXT**: Reference grid showing 12 essential Bash commands organized by category—navigation (cd, pwd, ls in blue), file manipulation (mv, cp, rm, mkdir in green), content viewing (cat, grep in yellow), and permissions (chmod in orange)—with syntax examples and purpose descriptions

---

#### VISUAL 13: Environment Variables Flow

**Status**: ✅ APPROVED

## The Story
Environment variables (PATH, HOME, GOOGLE_API_KEY) are global settings that programs read. Shell sets them, programs use them. Understanding this flow explains why "export VAR=value" makes API keys work.

## Emotional Intent
Should feel: System configuration clarity, "that's why it works!"
Visual mood: Data flow from shell to programs

## Visual Metaphor
Broadcasting tower sending signal to receivers (shell broadcasts env vars, programs receive)

## Key Insight to Emphasize
"Environment variables = Global settings shell broadcasts to all programs"

## Subject
Flow diagram showing shell setting env vars and programs reading them

## Composition
Flow diagram:
- Shell (bash/zsh) sets variables (export PATH=..., export API_KEY=...)
- Environment (system memory) stores them
- Programs (Python, Gemini CLI, Node) read them

Arrows showing: Shell → Environment → Programs

## Action
Variables flowing from shell through environment to programs

## Location/Context
System configuration explanation (environment setup)

## Style
Modern system diagram aesthetic
Reference: OS architecture docs, config flow diagrams

## Camera Perspective
Orthographic flow view

## Lighting
Flat even

## Color Semantics
- Blue (#3b82f6) = Shell (source)
- Purple (#8b5cf6) = Environment (storage)
- Green (#10b981) = Programs (consumers)

## Typography Hierarchy
- Largest: Component names (Shell, Environment, Programs)
- Medium: Variable names (PATH, API_KEY)
- Smallest: Example values (/usr/bin:..., sk-...)

## Text Integration
Labels on flow components with example variables

## Resolution
2K standard

## Teaching Goal
Teach environment variable flow: shell sets → environment stores → programs read

## Proficiency
B1: Moderate complexity (3 components, 2-3 example variables)

## Visual Type
Static flow diagram

## Google Search Grounding
No (conceptual system model)

## Pedagogical Reasoning
Flow diagram creates causal understanding (shell sets → programs read). Color coding teaches component roles (blue source, purple storage, green consumers). Typography guides from components to variables to values. This demystifies "export" by visualizing information flow.

**FILENAME**: `environment-variables-flow.png`

**ALT TEXT**: Flow diagram showing environment variable propagation from shell (blue, setting export PATH and API_KEY) through system environment (purple, storing variables) to programs (green, Python/Gemini CLI reading them), with arrows indicating data flow direction

---

#### VISUAL 14: Bash Script Execution Workflow

**Status**: ✅ APPROVED

## The Story
Bash scripts aren't magic—they're saved commands. Write commands in file, make executable (chmod +x), run (./script.sh). Understanding this workflow transforms "mysterious scripts" into "saved command sequences."

## Emotional Intent
Should feel: Scripts demystified, "just saved commands"
Visual mood: Process transparency, step-by-step execution

## Visual Metaphor
Recipe card (write instructions → save → follow steps)

## Key Insight to Emphasize
"Bash scripts = Saved command sequences made executable"

## Subject
Three-step script creation and execution workflow

## Composition
Sequential workflow:
1. Write commands in file (vim script.sh, nano script.sh)
2. Make executable (chmod +x script.sh)
3. Run script (./script.sh or bash script.sh)

Show actual script content (#!/bin/bash, commands) and terminal execution

## Action
User creating and running script through three steps

## Location/Context
Script workflow explanation (automation basics)

## Style
Developer workflow docs
Reference: Shell scripting tutorials, automation guides

## Camera Perspective
Orthographic sequential view

## Lighting
Flat even

## Color Semantics
- Blue (#3b82f6) = Write step (creation)
- Yellow (#fbbf24) = Permissions step (chmod)
- Green (#10b981) = Execute step (running)

## Typography Hierarchy
- Largest: Step numbers (1, 2, 3)
- Medium: Step labels (Write, Chmod, Execute)
- Smallest: Commands (vim script.sh, chmod +x, ./script.sh)

## Text Integration
Commands embedded in sequential steps, script content shown

## Resolution
2K standard

## Teaching Goal
Teach script workflow: write commands → make executable → run

## Proficiency
B1: Moderate complexity (3 steps, script example)

## Visual Type
Static sequential workflow

## Google Search Grounding
No (procedural workflow)

## Pedagogical Reasoning
Sequential steps demystify scripting through familiar process. Color coding creates visual progression (blue write, yellow permission, green execute). Typography guides from step numbers → labels → commands. This transforms intimidating "scripting" into accessible "saved commands."

**FILENAME**: `bash-script-execution-workflow.png`

**ALT TEXT**: Three-step script execution workflow showing Write (blue, creating script.sh with commands), Chmod (yellow, making executable with chmod +x), and Execute (green, running ./script.sh), with example script content and terminal output

### Chapter 8: AI-Native IDEs

**Proficiency**: B1 | **Layer**: L1-L2 | **Visuals**: 3

---

#### VISUAL 15: IDE Comparison Matrix (VS Code, Cursor, Zed)

**Status**: ✅ APPROVED

## The Story
Three IDE philosophies: VS Code (traditional extensible editor), Cursor (AI-first fork), Zed (native performance). Understanding trade-offs between features, AI integration depth, and performance helps developers choose their primary environment.

## Emotional Intent
Should feel: Informed decision-making, objective comparison
Visual mood: No single winner, choose based on needs

## Visual Metaphor
Tool belt comparison—each tool excels at different jobs (hammer vs screwdriver vs wrench)

## Key Insight to Emphasize
"Cursor brings AI-first UX, Zed brings speed, VS Code brings ecosystem maturity"

## Subject
Three-column feature comparison across key dimensions

## Composition
Grid comparison:
- Columns: VS Code | Cursor | Zed
- Rows: AI Integration, Performance, Extensions, Learning Curve, Multi-file Edit, Proficiency Required, Best For

## Color Semantics
- Blue (#3b82f6) = VS Code (established ecosystem)
- Purple (#8b5cf6) = Cursor (AI-first experience)
- Orange (#f97316) = Zed (performance-native)

## Typography Hierarchy
- Largest: IDE names (column headers)
- Medium: Feature categories (row labels)
- Smallest: Specific values (native/plugin, fast/medium, etc.)

## Teaching Goal
Teach IDE selection: match editor capabilities to workflow priorities (AI depth, speed, ecosystem)

## Proficiency
B1: Moderate complexity (3 IDEs, 7 comparison dimensions)

## Visual Type
Static comparison table

## Google Search Grounding
Yes (verify current features, performance benchmarks, AI capabilities)

## Pedagogical Reasoning
Grid enables direct IDE comparison across dimensions. Color coding creates instant visual identity. Typography guides from tool names → categories → specifics. This reduces choice paralysis by making trade-offs explicit (can't have everything—choose priorities).

**FILENAME**: `ide-comparison-matrix-vscode-cursor-zed.png`

**ALT TEXT**: Feature comparison matrix showing VS Code (blue, ecosystem strength), Cursor (purple, AI-first integration), and Zed (orange, native performance) across AI integration depth, performance speed, extension ecosystem, learning curve, multi-file editing, proficiency requirements, and ideal use cases

---

#### VISUAL 16: AI Integration Architecture (How IDEs Connect to AI Models)

**Status**: ✅ APPROVED

## The Story
AI-native IDEs don't just embed chatbots—they integrate AI at multiple layers: inline completions (like Copilot), chat interfaces (sidebar conversations), command palette (text-to-code), and context management (what files AI sees). Understanding this architecture reveals how deeply AI can assist.

## Emotional Intent
Should feel: Architectural clarity, "AI is everywhere in my editor"
Visual mood: Layered integration, not bolt-on feature

## Visual Metaphor
Building integrated wiring—AI runs through walls, not extension cord plugged in

## Key Insight to Emphasize
"AI-native IDEs integrate at 4 layers: completions, chat, commands, context"

## Subject
Multi-layer architecture diagram showing IDE-AI integration points

## Composition
Layered diagram:
- Layer 1: Inline Completions (code suggestions as you type)
- Layer 2: Chat Interface (sidebar conversations)
- Layer 3: Command Palette (natural language commands)
- Layer 4: Context Management (file selection, RAG)

Arrows showing: User code → AI model → Integrated suggestions

## Color Semantics
- Green (#10b981) = Inline layer (immediate suggestions)
- Blue (#3b82f6) = Chat layer (deliberate conversations)
- Purple (#8b5cf6) = Command layer (intent-driven)
- Orange (#f97316) = Context layer (what AI sees)

## Typography Hierarchy
- Largest: "AI Integration Layers" (concept)
- Medium: Layer names (Inline, Chat, Commands, Context)
- Smallest: Examples (autocomplete, sidebar, Cmd+K, file picker)

## Teaching Goal
Teach AI integration depth: AI-native means integration at every interaction layer, not just chatbot

## Proficiency
B1: Moderate complexity (4 layers, 6-8 integration points)

## Visual Type
Static architecture diagram

## Google Search Grounding
No (conceptual integration architecture)

## Pedagogical Reasoning
Layered architecture reveals integration depth through visual stacking. Color coding teaches layer purposes (green=immediate, blue=deliberate, purple=intent, orange=context). Typography guides from concept → layers → examples. This shows "AI-native" means architectural integration, not feature addition.

**FILENAME**: `ai-ide-integration-architecture.png`

**ALT TEXT**: Multi-layer architecture diagram showing AI integration in modern IDEs across four layers—inline completions (green, autocomplete suggestions), chat interface (blue, sidebar conversations), command palette (purple, natural language commands), and context management (orange, file selection and RAG)—with arrows showing user-AI interaction flow

---

#### VISUAL 17: IDE Selection Decision Tree

**Status**: ✅ APPROVED

## The Story
Choosing an IDE isn't random—it follows decision logic: Need maximum AI integration? → Cursor. Need maximum speed? → Zed. Need maximum extensions? → VS Code. This decision tree guides developers to the right choice based on priorities.

## Emotional Intent
Should feel: Clear decision path, "I know which IDE fits me"
Visual mood: Guided choice, not overwhelming

## Visual Metaphor
Choose-your-own-adventure flowchart (answer questions → reach destination)

## Key Insight to Emphasize
"Your top priority determines your IDE: AI-first → Cursor, Speed → Zed, Ecosystem → VS Code"

## Subject
Decision tree flowchart with priority questions leading to IDE recommendations

## Composition
Flowchart structure:
- Start: "What's your top priority?"
- Branch 1: "AI integration depth" → Cursor
- Branch 2: "Editor performance/speed" → Zed
- Branch 3: "Extension ecosystem" → VS Code
- Secondary branches for proficiency level and platform

## Color Semantics
- Gray (#6b7280) = Decision nodes (questions)
- Purple (#8b5cf6) = Cursor path (AI priority)
- Orange (#f97316) = Zed path (speed priority)
- Blue (#3b82f6) = VS Code path (ecosystem priority)

## Typography Hierarchy
- Largest: Start question ("What's your top priority?")
- Medium: Branch labels (AI integration, Performance, Ecosystem)
- Smallest: IDE recommendations (Cursor, Zed, VS Code)

## Teaching Goal
Teach IDE selection through decision logic: priorities determine choice, not arbitrary preference

## Proficiency
B1: Moderate complexity (3 priority branches, 2-3 sub-decisions)

## Visual Type
Static decision tree

## Google Search Grounding
No (decision framework)

## Pedagogical Reasoning
Decision tree creates selection confidence through structured questions. Color coding shows paths (purple AI, orange speed, blue ecosystem). Typography guides from question → priorities → recommendations. This reduces decision paralysis by making priorities explicit.

**FILENAME**: `ide-selection-decision-tree.png`

**ALT TEXT**: Decision tree flowchart starting with "What's your top priority?" and branching to three IDE recommendations based on priorities—AI integration depth leads to Cursor (purple path), performance/speed leads to Zed (orange path), extension ecosystem leads to VS Code (blue path)—with secondary branches for proficiency level and platform considerations

---

### Chapter 9: Git and GitHub

**Proficiency**: A2-B1 | **Layer**: L1-L2 | **Visuals**: 5

---

#### VISUAL 18: Git Three-Stage Workflow (Working → Staging → Repository)

**Status**: ✅ APPROVED

## The Story
Git's three-stage architecture confuses beginners: Working Directory (your files), Staging Area (selected changes), Repository (committed history). Understanding these stages explains why "git add" before "git commit" isn't redundant—it's architectural.

## Emotional Intent
Should feel: "Aha! That's why staging exists"
Visual mood: Clarity emerging from confusion

## Visual Metaphor
Assembly line with quality checkpoints (raw materials → inspection → finished product)

## Key Insight to Emphasize
"Git = Working Directory (draft) → Staging (review) → Repository (permanent)"

## Subject
Three-box workflow showing file progression through Git stages

## Composition
Sequential three-stage flow:
- Stage 1: Working Directory (modified files, untracked)
- Stage 2: Staging Area (git add, ready to commit)
- Stage 3: Repository (git commit, permanent history)

Arrows showing: Modified files → git add → git commit

## Color Semantics
- Red (#ef4444) = Working Directory (unstaged, temporary)
- Yellow (#fbbf24) = Staging Area (reviewed, ready)
- Green (#10b981) = Repository (committed, permanent)

## Typography Hierarchy
- Largest: Stage names (Working, Staging, Repository)
- Medium: Command labels (git add, git commit)
- Smallest: File examples (file.txt modified, staged, committed)

## Teaching Goal
Teach Git architecture: three stages exist for deliberate commit construction, not complexity

## Proficiency
A2: Simple complexity (3 stages, 2 commands, linear flow)

## Visual Type
Static workflow diagram

## Google Search Grounding
No (Git conceptual architecture)

## Pedagogical Reasoning
Three-stage progression creates architectural understanding. Color coding teaches stage purposes (red=temporary, yellow=ready, green=permanent). Typography guides from stages → commands → examples. This demystifies "git add" by revealing intentional staging design.

**FILENAME**: `git-three-stage-workflow.png`

**ALT TEXT**: Three-stage Git workflow diagram showing Working Directory (red, modified files), Staging Area (yellow, git add reviewed changes), and Repository (green, git commit permanent history), with arrows indicating file progression and command labels

---

#### VISUAL 19: Git Branching Strategy (Feature Branches → Main)

**Status**: ✅ APPROVED

## The Story
Git branches enable parallel work: main branch holds production code, feature branches hold experimental work. Merge when ready. This workflow prevents "breaking main" while allowing rapid iteration.

## Emotional Intent
Should feel: Safe experimentation, organized collaboration
Visual mood: Parallel tracks converging

## Visual Metaphor
Railroad tracks branching and merging (train doesn't derail when switching tracks)

## Key Insight to Emphasize
"Branches = Safe parallel work. Merge when feature ready."

## Subject
Branch diagram showing feature branches splitting from and merging back to main

## Composition
Git graph visualization:
- Main branch (horizontal trunk)
- Feature branches (diverging lines)
- Merge points (branches converging back to main)
- Commit nodes along branches

## Color Semantics
- Blue (#3b82f6) = Main branch (production code)
- Green (#10b981) = Feature branches (active development)
- Purple (#8b5cf6) = Merge commits (integration points)

## Typography Hierarchy
- Largest: Branch names (main, feature/auth, feature/ui)
- Medium: Commit messages (brief descriptions)
- Smallest: Commit hashes (SHA identifiers)

## Teaching Goal
Teach branching workflow: isolate experimental work, merge when stable

## Proficiency
A2: Simple complexity (1 main branch, 2-3 feature branches, 4-6 commits)

## Visual Type
Static branch graph

## Google Search Grounding
No (Git branching concept)

## Pedagogical Reasoning
Branch visualization creates spatial understanding of parallel work. Color coding teaches branch purposes (blue stable, green experimental, purple integration). Typography guides from branches → messages → hashes. This makes abstract "branching" concrete through railroad metaphor.

**FILENAME**: `git-branching-strategy-diagram.png`

**ALT TEXT**: Git branch graph showing main branch (blue horizontal line) with two feature branches (green diverging lines labeled feature/auth and feature/ui) splitting off for parallel development and merging back (purple merge commits) at integration points, with commit nodes and messages along each branch

---

#### VISUAL 20: Pull Request Lifecycle (Create → Review → Merge)

**Status**: ✅ APPROVED

## The Story
Pull Requests aren't just "merge buttons"—they're collaborative review workflows. Developer creates PR, team reviews code, discussion happens, changes requested, developer updates, final approval, merge. This lifecycle teaches code quality through peer review.

## Emotional Intent
Should feel: Collaborative improvement, team quality process
Visual mood: Structured review pipeline, not gatekeeping

## Visual Metaphor
Peer review publishing process (submit draft → editorial review → revisions → publish)

## Key Insight to Emphasize
"PR = Collaborative code review workflow, not just merge button"

## Subject
Sequential PR lifecycle with review stages and decision points

## Composition
Workflow stages:
1. Create PR (developer submits)
2. Review (team comments, suggests changes)
3. Update (developer addresses feedback)
4. Approve (team accepts changes)
5. Merge (code enters main branch)

Decision diamonds for "Changes Requested?" and "Approved?"

## Color Semantics
- Blue (#3b82f6) = PR creation and updates (developer actions)
- Yellow (#fbbf24) = Review stage (team feedback)
- Green (#10b981) = Approval and merge (quality gates passed)
- Red (#ef4444) = Changes requested (iteration needed)

## Typography Hierarchy
- Largest: Stage numbers (1, 2, 3, 4, 5)
- Medium: Stage names (Create, Review, Update, Approve, Merge)
- Smallest: Action descriptions (submit code, comment on lines, push fixes)

## Teaching Goal
Teach PR workflow: collaborative review improves code quality through structured feedback loop

## Proficiency
A2: Simple complexity (5 stages, 2 decision points)

## Visual Type
Static workflow diagram

## Google Search Grounding
No (GitHub PR workflow concept)

## Pedagogical Reasoning
Sequential stages create process understanding. Color coding teaches workflow phases (blue=developer, yellow=review, green=approved, red=iterate). Typography guides from stage numbers → names → actions. This transforms "scary PR review" into "helpful quality process."

**FILENAME**: `pull-request-lifecycle-workflow.png`

**ALT TEXT**: Pull request lifecycle workflow showing five stages—Create PR (blue, developer submits), Review (yellow, team comments), Update (blue, address feedback), Approve (green, team accepts), Merge (green, code enters main)—with decision points for "Changes Requested?" and "Approved?" creating feedback loops

---

#### VISUAL 21: Merge Conflict Resolution Anatomy

**Status**: ✅ APPROVED

## The Story
Merge conflicts aren't errors—they're Git asking "Two people edited the same lines differently. Which version should I keep?" Understanding conflict marker anatomy (<<<<<<, ======, >>>>>>) and resolution process (choose version, remove markers, commit) demystifies conflicts.

## Emotional Intent
Should feel: "Conflicts are solvable," not "Git broke"
Visual mood: Annotated explanation, surgical precision

## Visual Metaphor
Medical diagram with labeled anatomy (conflict markers are organs you need to understand)

## Key Insight to Emphasize
"Conflicts = Git asking which version to keep. Not errors, decisions."

## Subject
Annotated conflict example showing marker anatomy and resolution steps

## Composition
Side-by-side comparison:
- Left: Conflict state (with <<<<<<, ======, >>>>>> markers)
- Right: Resolved state (chosen version, markers removed)

Callout labels explaining:
- <<<<<<< HEAD (your changes)
- ======= (separator)
- >>>>>>> branch-name (their changes)

## Color Semantics
- Blue (#3b82f6) = Your changes (HEAD version)
- Orange (#f97316) = Their changes (incoming version)
- Red (#ef4444) = Conflict markers (to be removed)
- Green (#10b981) = Resolved version (decision made)

## Typography Hierarchy
- Largest: "Conflict Anatomy" (concept)
- Medium: Marker labels (<<<<<<< HEAD, =======, >>>>>>>)
- Smallest: Actual code (conflicting lines)

## Teaching Goal
Teach conflict resolution: understand marker anatomy → choose version → remove markers → commit

## Proficiency
A2: Simple complexity (3 markers explained, 1 resolution example)

## Visual Type
Static annotated diagram

## Google Search Grounding
No (Git conflict marker syntax)

## Pedagogical Reasoning
Side-by-side creates before/after clarity. Color coding teaches marker meanings (blue=yours, orange=theirs, red=markers, green=resolved). Typography guides from concept → markers → code. This demystifies "scary conflict" through anatomical explanation.

**FILENAME**: `merge-conflict-resolution-anatomy.png`

**ALT TEXT**: Side-by-side merge conflict anatomy showing conflict state (left) with labeled markers (<<<<<<< HEAD in blue for your changes, ======= separator in red, >>>>>>> branch-name in orange for their changes) and resolved state (right, green, with chosen version and markers removed)

---

#### VISUAL 22: AI-Assisted Git Workflow (How AI Helps with Commits, PRs, Conflicts)

**Status**: ✅ APPROVED

## The Story
AI transforms Git from manual command memorization to natural language intent. Instead of "git rebase -i HEAD~3", say "AI, squash my last 3 commits into one." AI helps write commit messages, resolve conflicts, create PR descriptions—making Git accessible to beginners.

## Emotional Intent
Should feel: Git made approachable, AI as Git mentor
Visual mood: Natural language → Git commands

## Visual Metaphor
Translator (you speak intent, AI translates to Git syntax)

## Key Insight to Emphasize
"AI translates Git intent to Git commands—learn concepts, not syntax"

## Subject
Multi-scenario diagram showing AI assisting with Git tasks

## Composition
Three scenarios side-by-side:
1. Commit Messages: "Summarize my changes" → AI writes descriptive commit
2. Conflict Resolution: "Which version should I keep?" → AI explains differences, suggests resolution
3. PR Creation: "Create PR description" → AI generates title, summary, changes list

## Color Semantics
- Purple (#8b5cf6) = AI assistance (intent understanding)
- Blue (#3b82f6) = User intent (natural language)
- Green (#10b981) = Git output (commands/messages)

## Typography Hierarchy
- Largest: Scenario labels (Commits, Conflicts, PRs)
- Medium: User intent (natural language requests)
- Smallest: AI output (generated Git commands/messages)

## Teaching Goal
Teach AI-Git collaboration: AI handles syntax, you understand concepts and intent

## Proficiency
B1: Moderate complexity (3 scenarios, AI translation demonstrated)

## Visual Type
Static multi-scenario diagram

## Google Search Grounding
No (conceptual AI assistance)

## Pedagogical Reasoning
Three scenarios show breadth of AI help. Color coding teaches interaction flow (blue intent → purple AI → green Git). Typography guides from scenarios → intent → output. This makes Git accessible by shifting focus from syntax to concepts.

**FILENAME**: `ai-assisted-git-workflow.png`

**ALT TEXT**: Three-scenario diagram showing AI assisting Git workflows—Commit Messages (user requests summary, AI generates descriptive commit), Conflict Resolution (user asks which version, AI explains differences), PR Creation (user requests PR description, AI generates title and changes summary)—with color coding showing intent (blue) → AI (purple) → Git output (green)

---

## PART 3: Markdown, Prompt & Context Engineering (Chapters 10-12)

### Chapter 10: Markdown Language of AI

**Proficiency**: A2 | **Layer**: L1 | **Visuals**: 4

---

#### VISUAL 23: Markdown Syntax Anatomy (Headers, Lists, Code, Links)

**Status**: ✅ APPROVED

## The Story
Markdown isn't a programming language—it's structural markup for plain text. Headers (#), lists (- or 1.), code blocks (```), and links ([text](url)) create formatted documents from simple text. Understanding these five core elements covers 90% of Markdown use.

## Emotional Intent
Should feel: "Markdown is simple," not intimidating
Visual mood: Clean syntax patterns, visual reference

## Visual Metaphor
Recipe card with ingredient list (simple ingredients → formatted result)

## Key Insight to Emphasize
"5 Markdown patterns (# headers, - lists, ``` code, [links], **bold**) = 90% of usage"

## Subject
Reference grid showing Markdown syntax and rendered output side-by-side

## Composition
Two-column layout (Markdown → Rendered):
- Headers: # H1, ## H2, ### H3
- Lists: - item, 1. numbered
- Code: ```python code```
- Links: [text](url)
- Emphasis: **bold**, *italic*

## Color Semantics
- Gray (#6b7280) = Raw Markdown syntax
- Blue (#3b82f6) = Rendered headings
- Green (#10b981) = Rendered code blocks
- Purple (#8b5cf6) = Rendered links

## Typography Hierarchy
- Largest: Element category labels (Headers, Lists, Code, Links)
- Medium: Markdown syntax (# Header)
- Smallest: Rendered examples (actual formatting shown)

## Teaching Goal
Teach Markdown basics: 5 core patterns create formatted documents from plain text

## Proficiency
A2: Simple complexity (5 elements, side-by-side comparison)

## Visual Type
Static syntax reference

## Google Search Grounding
No (standard Markdown syntax)

## Pedagogical Reasoning
Side-by-side creates instant syntax-to-result connection. Color coding teaches element purposes (blue headers, green code, purple links). Typography guides from categories → syntax → rendered examples. This makes Markdown approachable through visual pattern matching.

**FILENAME**: `markdown-syntax-anatomy-reference.png`

**ALT TEXT**: Two-column Markdown reference showing five core elements—Headers (# syntax rendering to blue headings), Lists (- and 1. syntax), Code blocks (``` syntax rendering to green code), Links ([text](url) rendering to purple hyperlinks), and Emphasis (**bold**, *italic*)—with raw Markdown (gray) and rendered output side-by-side

---

#### VISUAL 24: Plain Text vs Rendered Markdown Comparison

**Status**: ✅ APPROVED

## The Story
Markdown's power: write plain text with simple markers, render to formatted HTML/PDF/docs. Same content, two views—source (editable, portable) and output (formatted, readable). This duality makes Markdown universal.

## Emotional Intent
Should feel: "One source, many outputs," flexibility
Visual mood: Transformation clarity

## Visual Metaphor
Blueprint vs finished building (same information, different presentation)

## Key Insight to Emphasize
"Markdown = Plain text source → Rendered formatted output (HTML, PDF, web)"

## Subject
Side-by-side comparison showing same document in plain Markdown and rendered format

## Composition
Split-screen (50/50):
- Left: Plain text Markdown (# Header, - list, ```code```)
- Right: Rendered output (formatted headings, bullets, styled code blocks)

Arrow in middle showing transformation direction

## Color Semantics
- Gray (#6b7280) = Plain text source (raw Markdown)
- Blue (#3b82f6) = Rendered headings
- Green (#10b981) = Rendered code blocks
- Black (#000000) = Rendered body text

## Typography Hierarchy
- Largest: "Source" vs "Rendered" (view labels)
- Medium: Content (same text in both views)
- Smallest: Markdown markers (# - ```)

## Teaching Goal
Teach Markdown duality: single plain text source renders to multiple formatted outputs

## Proficiency
A2: Simple complexity (1 document, 2 views)

## Visual Type
Static split-screen comparison

## Google Search Grounding
No (Markdown rendering concept)

## Pedagogical Reasoning
Split-screen creates source-to-output understanding. Color coding teaches rendering (gray source → colored formatted). Typography emphasizes transformation (same content, different presentation). This shows Markdown's portability advantage over WYSIWYG tools.

**FILENAME**: `markdown-plain-text-vs-rendered.png`

**ALT TEXT**: Split-screen comparison showing Markdown document in two views—plain text source (left, gray, with visible # - ``` markers) and rendered formatted output (right, with blue headings, green code blocks, formatted lists)—with arrow indicating transformation direction

---

#### VISUAL 25: Code Block Examples (Syntax Highlighting by Language)

**Status**: ✅ APPROVED

## The Story
Markdown code blocks support language-specific syntax highlighting. Write ```python, ```javascript, ```bash—rendered output applies appropriate color coding. This makes code readable in documentation.

## Emotional Intent
Should feel: "Code documentation made beautiful"
Visual mood: Syntax clarity through color

## Visual Metaphor
Highlighter marker on textbook (color emphasizes important parts)

## Key Insight to Emphasize
"```language syntax enables color-coded documentation"

## Subject
Examples of code blocks in different languages with syntax highlighting

## Composition
Vertical stack of 4 code examples:
- Python: ```python (blue/purple syntax colors)
- JavaScript: ```javascript (yellow/green syntax colors)
- Bash: ```bash (green/gray syntax colors)
- JSON: ```json (orange/red syntax colors)

Each showing Markdown source and rendered colored output

## Color Semantics
Language-specific:
- Python: Blue keywords, purple strings
- JavaScript: Yellow keywords, green strings
- Bash: Green commands, gray comments
- JSON: Orange keys, red values

## Typography Hierarchy
- Largest: Language labels (Python, JavaScript, Bash, JSON)
- Medium: Code content (syntax-highlighted)
- Smallest: Markdown fence markers (```)

## Teaching Goal
Teach code block syntax: ```language enables syntax highlighting for readable documentation

## Proficiency
A2: Simple complexity (4 language examples, clear syntax highlighting)

## Visual Type
Static code example grid

## Google Search Grounding
No (standard Markdown code fence syntax)

## Pedagogical Reasoning
Vertical stack shows pattern consistency across languages. Language-specific colors teach syntax highlighting (different languages = different color schemes). Typography guides from language labels → code → markers. This shows documentation best practice.

**FILENAME**: `markdown-code-block-syntax-highlighting.png`

**ALT TEXT**: Vertical stack showing four Markdown code blocks with syntax highlighting—Python (blue keywords, purple strings), JavaScript (yellow keywords, green strings), Bash (green commands, gray comments), JSON (orange keys, red values)—demonstrating ```language syntax for color-coded documentation

---

#### VISUAL 26: Markdown as Intent Layer (Structure Before AI Generation)

**Status**: ✅ APPROVED

## The Story
In AI-native development, Markdown serves as Intent Layer—structure your document outline (headers, sections, bullet points), then ask AI to fill content. Structure defines intent, AI generates implementation. This is specification-first thinking applied to writing.

## Emotional Intent
Should feel: "Markdown = Blueprint for AI content"
Visual mood: Structure enables generation

## Visual Metaphor
Architectural blueprint (you draw structure, AI builds content)

## Key Insight to Emphasize
"Markdown structure (headers, bullets) = Intent for AI content generation"

## Subject
Two-stage diagram showing Markdown outline → AI-generated content

## Composition
Sequential workflow:
- Stage 1: Markdown outline (headers, bullet points, empty sections)
- Arrow: "AI, fill this structure"
- Stage 2: Completed document (AI-generated content under each header)

## Color Semantics
- Blue (#3b82f6) = Structural Markdown (headers, bullets—human intent)
- Green (#10b981) = Generated content (AI-filled paragraphs)
- Purple (#8b5cf6) = AI transformation (generation process)

## Typography Hierarchy
- Largest: Stage labels (Outline Structure, Generated Content)
- Medium: Markdown headers (# Section 1, ## Subsection)
- Smallest: Generated text content (AI-filled paragraphs)

## Teaching Goal
Teach specification-first writing: structure Markdown outline defines intent, AI generates content

## Proficiency
A2: Simple complexity (2 stages, clear transformation)

## Visual Type
Static workflow diagram

## Google Search Grounding
No (conceptual AI workflow)

## Pedagogical Reasoning
Two-stage workflow creates intent-to-implementation understanding. Color coding teaches roles (blue structure=human, green content=AI). Typography guides from stages → structure → content. This teaches specification-first mindset applied to writing.

**FILENAME**: `markdown-as-intent-layer-ai-generation.png`

**ALT TEXT**: Two-stage workflow diagram showing Markdown outline structure (blue headers and bullet points defining intent) with arrow labeled "AI, fill this structure" leading to completed document (green AI-generated paragraphs under each structural header), demonstrating specification-first writing approach

---

### Chapter 11: Prompt Engineering for AIDD

**Proficiency**: A2-B1 | **Layer**: L2 | **Visuals**: 3

---

#### VISUAL 27: Vague vs Clear Specification Comparison

**Status**: ✅ APPROVED

## The Story
Prompt quality determines output quality. Vague prompt: "Create authentication." Clear prompt: "Create JWT authentication with bcrypt hashing, 24h expiry, rate limiting (5 attempts/hour)." Same intent, vastly different AI output quality. This comparison teaches specification thinking.

## Emotional Intent
Should feel: "Aha! Clarity unlocks AI power"
Visual mood: Before/after transformation

## Visual Metaphor
Blurry photo vs sharp focus (same subject, different clarity)

## Key Insight to Emphasize
"Vague prompts = vague outputs. Clear specifications = quality implementations."

## Subject
Side-by-side comparison of vague prompt → poor output vs clear prompt → quality output

## Composition
Two-column comparison:
- Left: Vague prompt ("Create auth") → AI confused output (missing features, no security)
- Right: Clear prompt (JWT, bcrypt, expiry, rate limit specified) → AI complete output (all features implemented)

## Color Semantics
- Red (#ef4444) = Vague (missing specifications)
- Green (#10b981) = Clear (complete specifications)
- Gray (#6b7280) = Neutral output examples

## Typography Hierarchy
- Largest: "Vague" vs "Clear" (prompt quality labels)
- Medium: Prompt text (what user wrote)
- Smallest: AI output (generated code snippets)

## Teaching Goal
Teach specification quality: clear constraints and requirements enable quality AI outputs

## Proficiency
A2: Simple complexity (2 prompts compared, obvious quality difference)

## Visual Type
Static side-by-side comparison

## Google Search Grounding
No (conceptual prompt engineering)

## Pedagogical Reasoning
Side-by-side creates instant quality contrast. Color coding teaches prompt quality (red vague vs green clear). Typography guides from quality labels → prompts → outputs. This makes abstract "good prompt" concrete through comparison.

**FILENAME**: `vague-vs-clear-specification-comparison.png`

**ALT TEXT**: Side-by-side prompt comparison showing vague prompt (left, red, "Create auth" with incomplete AI output missing security features) versus clear prompt (right, green, "JWT auth with bcrypt, 24h expiry, rate limiting" with complete AI implementation including all specified features)

---

#### VISUAL 28: Prompt Pattern Templates (Persona, Task, Context, Format)

**Status**: ✅ APPROVED

## The Story
Effective prompts follow reusable patterns. Four components: Persona ("Act as X expert"), Task ("Create Y"), Context ("For Z use case with A constraints"), Format ("Output as B structure"). This template creates consistent quality prompts.

## Emotional Intent
Should feel: "I have a formula," prompt confidence
Visual mood: Reusable template, fill-in-the-blanks

## Visual Metaphor
Mad Libs template (fill blanks, get complete prompt)

## Key Insight to Emphasize
"Persona + Task + Context + Format = Quality prompt template"

## Subject
Four-component prompt template with examples for each component

## Composition
Structured template layout:
- Component 1: Persona (Act as [expert role])
- Component 2: Task (Create [specific deliverable])
- Component 3: Context (For [use case] with [constraints])
- Component 4: Format (Output as [structure])

Example filled template shown below structure

## Color Semantics
- Purple (#8b5cf6) = Persona (expert role)
- Blue (#3b82f6) = Task (what to create)
- Orange (#f97316) = Context (constraints, use case)
- Green (#10b981) = Format (output structure)

## Typography Hierarchy
- Largest: Component labels (Persona, Task, Context, Format)
- Medium: Template placeholders ([expert role], [deliverable])
- Smallest: Filled example (actual prompt text)

## Teaching Goal
Teach prompt template: four-component structure creates reusable quality prompt pattern

## Proficiency
A2: Simple complexity (4 components, 1 filled example)

## Visual Type
Static template diagram

## Google Search Grounding
No (prompt engineering framework)

## Pedagogical Reasoning
Four-component structure creates prompt confidence through formula. Color coding teaches component purposes (purple role, blue task, orange constraints, green format). Typography guides from labels → placeholders → example. This transforms "how do I prompt?" into "fill these four blanks."

**FILENAME**: `prompt-pattern-template-four-components.png`

**ALT TEXT**: Four-component prompt template showing Persona (purple, "Act as [expert role]"), Task (blue, "Create [deliverable]"), Context (orange, "For [use case] with [constraints]"), Format (green, "Output as [structure]"), with filled example below demonstrating complete quality prompt using all components

---

#### VISUAL 29: Iteration Loop (Initial Prompt → Review → Refine → Better Output)

**Status**: ✅ APPROVED

## The Story
Prompting isn't one-shot—it's iterative. Initial prompt → AI output → Review quality → Refine prompt with feedback → Better output. This loop teaches prompt engineering as skill developed through practice, not perfect first attempts.

## Emotional Intent
Should feel: "Iteration is normal," not "I failed"
Visual mood: Learning spiral, continuous improvement

## Visual Metaphor
Spiral staircase climbing upward (each iteration improves)

## Key Insight to Emphasize
"Prompt engineering = Iteration loop, not perfect first try"

## Subject
Circular iteration loop showing prompt refinement cycle

## Composition
Circular workflow (clockwise):
1. Initial Prompt (rough attempt)
2. AI Output (initial result)
3. Review (identify gaps, missing features)
4. Refine Prompt (add constraints, clarify intent)
→ Back to step 1 with better prompt (spiral upward)

## Color Semantics
- Blue (#3b82f6) = Prompt stages (initial, refined)
- Green (#10b981) = Output improvement (getting better)
- Yellow (#fbbf24) = Review stage (analysis)
- Purple (#8b5cf6) = Refinement (learning)

## Typography Hierarchy
- Largest: Loop stage numbers (1, 2, 3, 4)
- Medium: Stage names (Prompt, Output, Review, Refine)
- Smallest: Action descriptions (identify gaps, add constraints)

## Teaching Goal
Teach iterative prompting: refinement through feedback loop creates quality, not perfect first attempts

## Proficiency
A2: Simple complexity (4-stage loop, clear progression)

## Visual Type
Static iteration diagram

## Google Search Grounding
No (iterative process concept)

## Pedagogical Reasoning
Circular loop creates iteration normalization (not failure, learning). Color coding teaches stages (blue prompt, green output, yellow review, purple refine). Typography guides from numbers → stages → actions. This reduces "one-shot pressure" by showing iteration as design.

**FILENAME**: `prompt-iteration-refinement-loop.png`

**ALT TEXT**: Circular iteration loop showing four stages of prompt refinement—Initial Prompt (blue), AI Output (green), Review (yellow, identify gaps), Refine Prompt (purple, add constraints)—with arrows creating clockwise cycle and spiral indicating continuous improvement through iteration

---

### Chapter 12: Context Engineering for AI-Driven Development

**Proficiency**: A2-B1 | **Layer**: L2 | **Visuals**: 3

---

#### VISUAL 30: Context Hierarchy (Always Relevant → Task-Specific → One-Time)

**Status**: ✅ APPROVED

## The Story
Not all context deserves equal attention. Three tiers: Always Relevant (project README, core architecture—always load), Task-Specific (files for current feature—load when needed), One-Time (rare reference—load only when asked). This hierarchy prevents context overload.

## Emotional Intent
Should feel: "Organize context strategically"
Visual mood: Priority pyramid, intentional inclusion

## Visual Metaphor
Priority pyramid (foundation → specific → rare)

## Key Insight to Emphasize
"Context tiers: Always (foundation) → Task (current work) → One-Time (rare reference)"

## Subject
Three-tier pyramid showing context priority levels

## Composition
Pyramid structure (bottom to top):
- Base: Always Relevant (README, architecture docs, style guides)
- Middle: Task-Specific (current feature files, related modules)
- Top: One-Time (rare references, specific examples)

Width represents frequency/importance

## Color Semantics
- Blue (#3b82f6) = Always Relevant (foundation, broad)
- Green (#10b981) = Task-Specific (current work, moderate)
- Yellow (#fbbf24) = One-Time (rare, narrow)

## Typography Hierarchy
- Largest: Tier labels (Always, Task-Specific, One-Time)
- Medium: File type examples (README, feature files, examples)
- Smallest: Specific filenames (README.md, auth.py)

## Teaching Goal
Teach context prioritization: organize files by relevance tier to prevent overload

## Proficiency
A2: Simple complexity (3 tiers, clear priority pyramid)

## Visual Type
Static pyramid diagram

## Google Search Grounding
No (context management framework)

## Pedagogical Reasoning
Pyramid structure teaches priority through visual size (wide base = always, narrow top = rare). Color coding creates tier recognition. Typography guides from tiers → file types → examples. This prevents "dump all files" by teaching strategic inclusion.

**FILENAME**: `context-hierarchy-pyramid-three-tiers.png`

**ALT TEXT**: Three-tier context priority pyramid showing Always Relevant context (blue, wide base, README and architecture docs), Task-Specific context (green, middle, current feature files), and One-Time context (yellow, narrow top, rare references), with visual width indicating frequency and importance

---

#### VISUAL 31: File Importance Matrix (High Impact × High Frequency)

**Status**: ✅ APPROVED

## The Story
Which files deserve context priority? Two dimensions: Impact (how critical for understanding?) and Frequency (how often referenced?). High Impact × High Frequency = Always load. Low × Low = Load only when needed. This matrix guides context decisions.

## Emotional Intent
Should feel: "Prioritize objectively," not guessing
Visual mood: Data-driven context selection

## Visual Metaphor
Eisenhower matrix for context (urgent/important quadrants)

## Key Insight to Emphasize
"High Impact × High Frequency files = Priority context. Low × Low = Load on demand."

## Subject
2×2 matrix plotting file importance and frequency

## Composition
Quadrant matrix:
- Quadrant 1 (top-right): High Impact × High Frequency (Always load: README, main modules)
- Quadrant 2 (top-left): High Impact × Low Frequency (Load when critical: architecture docs)
- Quadrant 3 (bottom-right): Low Impact × High Frequency (Load for quick reference: utils)
- Quadrant 4 (bottom-left): Low Impact × Low Frequency (Load rarely: examples, legacy)

Example files plotted in each quadrant

## Color Semantics
- Green (#10b981) = High Impact × High Frequency (priority)
- Blue (#3b82f6) = High Impact × Low Frequency (important when needed)
- Yellow (#fbbf24) = Low Impact × High Frequency (convenient reference)
- Gray (#6b7280) = Low Impact × Low Frequency (minimal priority)

## Typography Hierarchy
- Largest: Quadrant labels (Always Load, Load When Critical, etc.)
- Medium: File categories (main modules, utils, examples)
- Smallest: Specific filenames (plotted points)

## Teaching Goal
Teach context prioritization: evaluate Impact × Frequency to guide file inclusion decisions

## Proficiency
B1: Moderate complexity (2×2 matrix, 8-10 file examples plotted)

## Visual Type
Static quadrant matrix

## Google Search Grounding
No (context decision framework)

## Pedagogical Reasoning
2×2 matrix creates objective prioritization (not guessing). Color coding teaches quadrant meanings (green priority, gray minimal). Typography guides from quadrants → categories → files. This makes "which files?" answerable through two-dimension evaluation.

**FILENAME**: `file-importance-matrix-impact-frequency.png`

**ALT TEXT**: 2×2 context priority matrix showing file importance (vertical axis) versus frequency of reference (horizontal axis), with four quadrants—High Impact × High Frequency (green, Always Load like README), High Impact × Low Frequency (blue, Load When Critical like architecture docs), Low Impact × High Frequency (yellow, Quick Reference like utils), Low Impact × Low Frequency (gray, Load Rarely like examples)—with example files plotted in each quadrant

---

#### VISUAL 32: Decision Preservation (Why We Made This Choice)

**Status**: ✅ APPROVED

## The Story
Code shows WHAT you built, not WHY you chose it. Decision records (ADRs, comment explanations, spec rationale) preserve context: "Why JWT over sessions?" "Why PostgreSQL over MongoDB?" This prevents future confusion and rework.

## Emotional Intent
Should feel: "Preserve reasoning for future self"
Visual mood: Time capsule, intentional documentation

## Visual Metaphor
Archaeological record (preserve reasoning for future discovery)

## Key Insight to Emphasize
"Code = WHAT. Context = WHY. Preserve decisions to prevent future rework."

## Subject
Timeline showing code (what) vs decision records (why) persisting over time

## Composition
Timeline diagram:
- Present: Code written + Decision documented (ADR, comments, spec)
- 6 months later: Code remains, decision record explains WHY
- Without decision: Future developer confused, risks undoing intentional choice
- With decision: Future developer understands, maintains or improves intentionally

## Color Semantics
- Blue (#3b82f6) = Code (what was built)
- Purple (#8b5cf6) = Decision record (why it was built)
- Green (#10b981) = Preserved understanding (future clarity)
- Red (#ef4444) = Confusion/rework (missing context)

## Typography Hierarchy
- Largest: Timeline markers (Present, 6 Months Later)
- Medium: Scenario labels (With Decision vs Without Decision)
- Smallest: Decision examples (ADR text, comment snippets)

## Teaching Goal
Teach decision preservation: document WHY to prevent future confusion and unintentional rework

## Proficiency
B1: Moderate complexity (timeline, two future scenarios)

## Visual Type
Static timeline diagram

## Google Search Grounding
No (decision documentation concept)

## Pedagogical Reasoning
Timeline creates future-thinking mindset (today's decisions inform tomorrow). Color coding teaches documentation value (purple decision → green understanding vs red confusion). Typography guides from time → scenarios → examples. This motivates "why document?" through future consequences.

**FILENAME**: `decision-preservation-timeline-why-documentation.png`

**ALT TEXT**: Timeline diagram comparing future scenarios—Present shows code (blue) written with decision record (purple, ADR/comments documenting WHY), 6 months later shows two paths: With Decision leads to preserved understanding (green, future developer understands intent), Without Decision leads to confusion and rework (red, developer guesses and risks undoing intentional choices)

---

## PART 4: Python Fundamentals (Chapters 13-30)

### Chapter 13: Python UV Package Manager

**Proficiency**: A2 | **Layer**: L1 | **Visuals**: 2

---

#### VISUAL 33: UV Workflow (Install → Create Venv → Install Packages → Run)

**Status**: ✅ APPROVED

## The Story
UV simplifies Python environment setup to four steps: Install UV, Create virtual environment (isolated dependencies), Install packages (project-specific), Run code. This workflow replaces complex pip/venv/poetry confusion with single fast tool.

## Emotional Intent
Should feel: "Python setup is simple now"
Visual mood: Clear linear workflow, speed emphasis

## Visual Metaphor
Fast-lane highway (skip traffic, get to coding quickly)

## Key Insight to Emphasize
"UV = One fast tool replacing pip + venv + poetry complexity"

## Subject
Four-step linear workflow with speed indicators

## Composition
Sequential workflow (left-to-right):
1. Install UV (`curl | sh` or `brew install uv`)
2. Create venv (`uv venv`)
3. Install packages (`uv pip install fastapi`)
4. Run code (`uv run app.py`)

Speed indicators showing UV's performance advantage

## Color Semantics
- Purple (#8b5cf6) = UV (fast modern tool)
- Green (#10b981) = Success checkpoints (✓)
- Blue (#3b82f6) = Commands (actions)

## Typography Hierarchy
- Largest: Step numbers (1, 2, 3, 4)
- Medium: Step labels (Install, Create, Install Packages, Run)
- Smallest: Commands (uv venv, uv pip install)

## Teaching Goal
Teach UV workflow: four simple steps to isolated Python environment and execution

## Proficiency
A2: Simple complexity (4 linear steps)

## Visual Type
Static sequential workflow

## Google Search Grounding
Yes (verify UV installation commands, syntax)

## Pedagogical Reasoning
Linear workflow creates setup confidence through clear sequence. Color coding teaches tool identity (purple UV throughout). Typography guides from steps → labels → commands. Speed indicators reinforce UV's key benefit (performance over traditional tools).

**FILENAME**: `uv-workflow-four-step-setup.png`

**ALT TEXT**: Four-step UV workflow showing Install UV (purple, curl or brew command), Create venv (purple, uv venv), Install packages (purple, uv pip install fastapi), Run code (purple, uv run app.py), with green success checkpoints and speed indicators emphasizing UV's performance advantage over traditional Python tools

---

#### VISUAL 34: Virtual Environment Isolation Concept

**Status**: ✅ APPROVED

## The Story
Virtual environments solve dependency conflicts: Project A needs Django 3, Project B needs Django 4. Without isolation, one version overwrites the other. Venv creates isolated directories—each project gets its own dependency versions.

## Emotional Intent
Should feel: "Isolation prevents conflicts"
Visual mood: Separated containers, organized dependencies

## Visual Metaphor
Separate storage boxes (each project's dependencies in own container)

## Key Insight to Emphasize
"Virtual environments = Isolated dependency containers preventing version conflicts"

## Subject
Diagram showing global Python vs isolated virtual environments

## Composition
Three-tier visualization:
- Bottom: Global Python (system-wide, single version of each package)
- Middle: Virtual Environment A (Django 3, isolated)
- Top: Virtual Environment B (Django 4, isolated)

Arrows showing: venv isolates projects from global and each other

## Color Semantics
- Gray (#6b7280) = Global Python (shared, potential conflicts)
- Blue (#3b82f6) = Venv A (isolated, safe)
- Green (#10b981) = Venv B (isolated, safe)
- Red (#ef4444) = Conflict indicators (when isolation absent)

## Typography Hierarchy
- Largest: Environment labels (Global, Project A, Project B)
- Medium: Package versions (Django 3, Django 4)
- Smallest: File paths (.venv/lib/python)

## Teaching Goal
Teach virtual environment purpose: isolation prevents dependency version conflicts across projects

## Proficiency
A2: Simple complexity (3 environments, clear isolation benefit)

## Visual Type
Static isolation diagram

## Google Search Grounding
No (virtual environment concept)

## Pedagogical Reasoning
Three-tier structure visualizes isolation (separate containers). Color coding teaches safety (gray global conflicts vs blue/green isolated safe). Typography guides from environments → packages → paths. This makes abstract "dependency isolation" concrete through container metaphor.

**FILENAME**: `virtual-environment-isolation-concept.png`

**ALT TEXT**: Three-tier virtual environment diagram showing Global Python (gray, system-wide single versions with red conflict indicators) versus isolated Virtual Environment A (blue, Django 3) and Virtual Environment B (green, Django 4), with arrows demonstrating how venv prevents dependency version conflicts by creating separate package containers per project

---

### Chapter 14: Introduction to Python

**Proficiency**: A2 | **Layer**: L1 | **Visuals**: 2

---

#### VISUAL 35: Python REPL Anatomy (>>> Prompt, Input, Output)

**Status**: ✅ APPROVED

## The Story
Python REPL (Read-Eval-Print-Loop) is interactive playground: Type expression, press Enter, see result immediately. `>>>` prompt signals "waiting for input," result prints below. Understanding this loop makes Python experimentation intuitive.

## Emotional Intent
Should feel: "REPL = Immediate feedback playground"
Visual mood: Interactive conversation, low-pressure experimentation

## Visual Metaphor
Question-answer dialogue (you ask, Python answers instantly)

## Key Insight to Emphasize
"REPL = Type expression → Instant result (no file saving, immediate feedback)"

## Subject
Annotated REPL session showing prompt, input, output cycle

## Composition
Terminal-style interface showing:
- >>> prompt (input ready)
- User input (2 + 2)
- Output (4)
- Next >>> prompt (ready for more)

Callout labels explaining each component

## Color Semantics
- Blue (#3b82f6) = >>> prompt (waiting for input)
- Green (#10b981) = User input (expression typed)
- Gray (#6b7280) = Output (result printed)

## Typography Hierarchy
- Largest: "REPL Loop" (concept label)
- Medium: Component labels (Prompt, Input, Output)
- Smallest: Actual REPL text (>>> 2 + 2)

## Teaching Goal
Teach REPL interaction: immediate feedback loop for Python experimentation without file management

## Proficiency
A2: Simple complexity (3 components, clear cycle)

## Visual Type
Static annotated diagram

## Google Search Grounding
No (Python REPL standard interface)

## Pedagogical Reasoning
Annotated REPL creates component understanding. Color coding teaches interaction flow (blue prompt → green input → gray output). Typography guides from concept → components → examples. This reduces REPL intimidation through labeled anatomy.

**FILENAME**: `python-repl-anatomy-interactive-loop.png`

**ALT TEXT**: Annotated Python REPL interface showing >>> prompt (blue, waiting for input), user expression 2 + 2 (green, typed input), output 4 (gray, printed result), and next >>> prompt (blue, ready for more), with callout labels explaining Read-Eval-Print-Loop cycle for immediate feedback experimentation

---

#### VISUAL 36: Variable Memory Model (Name → Value in Memory)

**Status**: ✅ APPROVED

## The Story
Variables aren't "boxes storing values"—they're names pointing to values in memory. `x = 5` creates value 5 in memory, points name "x" to it. Reassignment (`x = 10`) changes what x points to, doesn't modify original 5. This mental model explains Python's reference semantics.

## Emotional Intent
Should feel: "Variables = Labels, not boxes"
Visual mood: Pointer clarity, memory visualization

## Visual Metaphor
Name tags on objects (labels point to things, aren't the things themselves)

## Key Insight to Emphasize
"Variables = Names pointing to values in memory (not containers holding values)"

## Subject
Memory diagram showing variable names and values as separate entities

## Composition
Two-part visualization:
- Left: Variable names (x, y, z)
- Right: Memory values (5, 10, "hello")
- Arrows: Names pointing to values

Reassignment showing arrow changing targets

## Color Semantics
- Blue (#3b82f6) = Variable names (labels)
- Green (#10b981) = Memory values (objects)
- Purple (#8b5cf6) = Pointers (arrows showing reference)

## Typography Hierarchy
- Largest: "Variables" vs "Memory Values" (concept separation)
- Medium: Variable names (x, y, z)
- Smallest: Value representations (5, "hello")

## Teaching Goal
Teach Python reference semantics: variables point to values, reassignment changes pointers

## Proficiency
A2: Simple complexity (3 variables, clear pointer visualization)

## Visual Type
Static memory diagram

## Google Search Grounding
No (Python memory model concept)

## Pedagogical Reasoning
Separate visualization of names vs values creates reference understanding. Color coding teaches component roles (blue labels, green values, purple pointers). Typography emphasizes separation (names ≠ values). This prevents "box storing value" misconception.

**FILENAME**: `variable-memory-model-name-value-pointers.png`

**ALT TEXT**: Python memory diagram showing variable names (blue, x, y, z on left) as separate from memory values (green, 5, 10, "hello" on right), with purple arrows indicating pointers from names to values, and reassignment example showing arrow changing targets to demonstrate reference semantics rather than container model

---

### Chapter 15: Data Types

**Proficiency**: A2 | **Layer**: L1 | **Visuals**: 2

---

#### VISUAL 37: Python Data Types Hierarchy (Primitive → Collections → Custom)

**Status**: ✅ APPROVED

## The Story
Python data types organize into three tiers: Primitives (int, float, str, bool—single values), Collections (list, tuple, dict, set—multiple values), Custom (classes—user-defined structures). Understanding this hierarchy guides type selection.

## Emotional Intent
Should feel: "Types organized logically"
Visual mood: Clear categorization, progression from simple to complex

## Visual Metaphor
Building blocks pyramid (simple pieces → combinations → custom structures)

## Key Insight to Emphasize
"Data types: Primitives (single) → Collections (multiple) → Custom (user-defined)"

## Subject
Three-tier hierarchy showing type progression

## Composition
Pyramid or tree structure:
- Base: Primitives (int, float, str, bool)
- Middle: Collections (list, tuple, dict, set)
- Top: Custom (class definitions)

Examples for each tier

## Color Semantics
- Blue (#3b82f6) = Primitives (foundation, simple)
- Green (#10b981) = Collections (built-in composite)
- Purple (#8b5cf6) = Custom (user-defined complex)

## Typography Hierarchy
- Largest: Tier labels (Primitives, Collections, Custom)
- Medium: Type names (int, list, class)
- Smallest: Example values (42, [1,2,3], User())

## Teaching Goal
Teach type hierarchy: organize Python types by complexity tier for selection guidance

## Proficiency
A2: Simple complexity (3 tiers, 4-6 types per tier)

## Visual Type
Static hierarchy diagram

## Google Search Grounding
No (Python type categories)

## Pedagogical Reasoning
Pyramid/tree creates visual progression (simple → complex). Color coding teaches tier purposes (blue foundation, green composite, purple custom). Typography guides from tiers → types → examples. This organizes "too many types" into comprehensible structure.

**FILENAME**: `python-data-types-hierarchy-three-tiers.png`

**ALT TEXT**: Three-tier Python data types hierarchy showing Primitives (blue, base tier with int, float, str, bool for single values), Collections (green, middle tier with list, tuple, dict, set for multiple values), and Custom (purple, top tier with class for user-defined structures), with example values demonstrating each type

---

#### VISUAL 38: Type Casting Flow (str → int, int → float, etc.)

**Status**: ✅ APPROVED

## The Story
Type casting converts between types: `int("42")` converts string to integer, `str(42)` converts integer to string. Some conversions work (`float(42)`), some fail (`int("hello")`). Understanding casting flow prevents errors.

## Emotional Intent
Should feel: "Type conversion is explicit"
Visual mood: Transformation clarity, know what works

## Visual Metaphor
Shape sorter toy (round peg fits round hole, square peg doesn't fit round hole)

## Key Insight to Emphasize
"Type casting = Explicit conversion between compatible types (str↔int, int↔float)"

## Subject
Flow diagram showing valid and invalid type conversions

## Composition
Network diagram showing types as nodes, conversion functions as edges:
- Nodes: str, int, float, bool, list
- Edges (green): Valid conversions (int("42"), str(42), float(42))
- Edges (red): Invalid conversions (int("hello"), float("text"))

## Color Semantics
- Green (#10b981) = Valid conversions (compatible types)
- Red (#ef4444) = Invalid conversions (incompatible types)
- Blue (#3b82f6) = Type nodes (data types)

## Typography Hierarchy
- Largest: Type labels (str, int, float, bool)
- Medium: Conversion functions (int(), str(), float())
- Smallest: Example values ("42", 42, 42.0)

## Teaching Goal
Teach type casting: explicit conversion between compatible types, predict success vs errors

## Proficiency
A2: Simple complexity (5 types, 6-8 conversion examples)

## Visual Type
Static conversion network

## Google Search Grounding
No (Python type casting standard functions)

## Pedagogical Reasoning
Network diagram creates conversion visualization (what connects to what). Color coding teaches validity (green works, red fails). Typography guides from types → functions → examples. This prevents casting errors through compatibility understanding.

**FILENAME**: `type-casting-flow-valid-invalid-conversions.png`

**ALT TEXT**: Type conversion network diagram showing Python types (blue nodes: str, int, float, bool, list) connected by casting functions with green edges indicating valid conversions (int("42"), str(42), float(42)) and red edges showing invalid conversions (int("hello"), float("text")), demonstrating explicit type casting compatibility

---

