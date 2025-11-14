---
description: Universal intelligence-driven LoopFlow workflow orchestrator. Reads constitution + project context to derive requirements automatically. Chains /sp.specify → /sp.plan → /sp.tasks → /sp.implement with quality gates. Works for ANY chapter, feature, or task (not limited to Python).
---

# /sp.loopflow: Universal LoopFlow Orchestrator

**Purpose**: Execute the complete LoopFlow+ workflow (Evals → Spec → Plan → Tasks → Implement → Validate) for ANY chapter, feature, or task using **vertical intelligence** (constitution + project context + domain skills). Goal-oriented, context-adaptive, and quality-assured.

**Intelligence Sources**:
- Constitution: Project vision, core principles, target audience, philosophies
- Project Context: Chapter index, book structure, existing specs
- Domain Skills: Available skills library (`.claude/skills/`)
- User Intent: Natural language description of what to build

**Adaptive Workflow**: 0-5 targeted questions based on what intelligence can't derive. NO hardcoded questionnaires.

## User Input

```text
$ARGUMENTS
```

---

## ⚠️ CRITICAL ANTI-PATTERN: DON'T OVER-ENGINEER WITH AI

### Constitutional Alignment

✅ **Principle 2 (Co-Learning Partner):** AI used strategically, not blindly for everything
✅ **Core Philosophy #1 (AI Spectrum):** Teaching when Assisted vs. Driven vs. Native makes sense
✅ **Graduated Teaching (Principle 13):** Direct foundations first, AI for complexity
✅ **"Specs Are the New Syntax":** Focus on high-value specification work, not trivial command execution

---

## 🎭 SUPER ORCHESTRA MODE (Optional Deep-Research)

**When to invoke**: Task requires comprehensive intelligence gathering + market-defining output

**Indicators**:
- User mentions "research Context7" or "gather from official sources"
- Gap identified that spans multiple scattered documentation sources
- Output must surpass market alternatives (not just meet internal specs)
- Strategic positioning required ("Is this better than official docs?")

**If triggered, apply**:
- Use `super-orchestra` agent
- Use `super-orchestra-session` output style
- Context7 library research (8000+ tokens)
- WebFetch official sources (3+ URLs)
- Iterative refinement with positioning validation
- Meta-learning capture for system evolution

**Example**: Chapter 5 Skills/Plugins/MCP session (see `.claude/agents/super-orchestra.md`)

---

## PHASE 0: INTELLIGENT CONTEXT DISCOVERY

**STEP 1: Read Authoritative Sources**

Immediately read these files FIRST (no user questions yet):

1. **Constitution** (`.specify/memory/constitution.md`):
   - Project vision: "Specs Are the New Syntax" (AI-native development)
   - Core principles (18 total, especially Principle 13: Graduated Teaching, Principle 18: Three Roles)
   - Core philosophies (8 total, especially Evals-First, Co-Learning, Spec-First, Validation-First)
   - Target audience: Aspiring/Professional/Founders with graduated complexity
   - Nine Pillars: AI CLI, Markdown, MCP, AI-First IDEs, Cross-Platform, TDD, SDD, Composable Skills, Cloud-Native
   - **CRITICAL**: "AI Development Spectrum" (Assisted 2-3x → Driven 5-10x → Native 50-99x)

2. **Chapter Index** (IF book chapter) (`specs/book/chapter-index.md`):
   - Chapter number → Part mapping
   - Prerequisites (what chapters must come before)
   - Complexity tier (A1-C2 CEFR levels)
   - Status (planned/in-progress/completed)

3. **Existing Specs** (`specs/` directory):
   - Similar chapters/features for pattern reference
   - Naming conventions
   - Structure examples

4. **Domain Skills** (`.claude/skills/`):
   - Available skills for this task type
   - Teaching patterns (if educational content)
   - Validation strategies (if technical feature)

**STEP 2: Automatic Derivations**

From sources above, derive WITHOUT asking user:

**For Book Chapters:**
- Part number (from chapter number range in chapter-index)
- Audience tier: Part 1-3 (Aspiring/Beginner A1-A2) → Part 4-5 (Intermediate B1-B2) → Part 6-8 (Advanced C1) → Part 9-13 (Professional C2)
- Complexity level and cognitive load limits
- Prerequisites (chapters that must exist first)
- Relevant domain skills (learning-objectives, concept-scaffolding, code-example-generator, etc.)
- Teaching pattern (Graduated Teaching Principle 13: Book teaches stable → AI handles complex → AI orchestrates scale)

**For Code Features:**
- Task type (authentication, API, database, deployment, etc.)
- Audience (professional developers)
- Prerequisites (existing codebase components)
- Relevant domain skills (technical-clarity, assessment-builder, etc.)

**For Documentation/Testing:**
- Scope from file paths and existing content
- Audience (contributors, users, developers)
- Prerequisites (what must exist to document/test)

**CRITICAL ANTI-PATTERN CHECK:**
- Is this task a simple, deterministic command? (like `uv init`, `docker build`, `git commit`)
  - ✅ IF YES: Don't create elaborate AI workflows. Document direct commands.
  - ✅ IF NO: This is where AI adds value (complex workflows, troubleshooting, strategic decisions).

**STEP 3: Apply Constitution Context**

Encode these principles into ALL downstream phases:

**From Core Philosophy:**
1. **Evals-First Development**: Define success criteria BEFORE specs
2. **Co-Learning Partnership**: Bidirectional learning (AI teaches student, student teaches AI feedback loop)
3. **Spec-First Development**: "Specs Are the New Syntax" - articulating intent is the primary skill
4. **Validation-First Safety**: Never trust, always verify

**From Graduated Teaching (Principle 13):**
- **Tier 1 (Book Content)**: Teach stable, foundational concepts that don't change often
- **Tier 2 (AI Companion)**: AI handles complex execution from specifications
- **Tier 3 (AI Orchestration)**: AI orchestrates multi-step workflows at scale (10+ items)

**WHEN TO USE AI (Critical Distinction):**
- ❌ **DON'T use AI for**: Simple commands (`npm install`, `uv add`, `git status`), straightforward operations, 1-step tasks
- ✅ **DO use AI for**: Troubleshooting, debugging complex errors, understanding concepts, strategic decisions, multi-step workflows

**From AI Development Spectrum:**
- **Assisted (2-3x)**: AI as helper for simple tasks (Parts 1-2)
- **Driven (5-10x)**: AI generates from specs (Parts 3-8) ← Primary focus
- **Native (50-99x)**: AI as core product capability (Parts 9-13)

**From Target Audience:**
- **Aspiring (A1-A2)**: Max 7 concepts per section, 2 options max, cognitive load managed
- **Intermediate (B1-B2)**: 7 concepts per section, 3-4 options, tradeoff discussions
- **Advanced (C1)**: 10 concepts per section, 5+ options, architecture patterns
- **Professional (C2)**: No artificial limits, production complexity

**STEP 4: Identify Genuine Ambiguities**

Now that you have full context, identify ONLY what's genuinely ambiguous (0-5 questions max):

**Ask IF**:
- Existing context found → "Use existing [spec/approach] or start fresh?"
- Goal is broad/vague → "What specific aspect to emphasize?"
- Multiple valid approaches → "Which strategy fits best: [A, B, or C]?"
- Capstone vs conceptual unclear → "Should students BUILD something hands-on?"
- Scope ambiguous → "What's in scope vs out of scope?"

**DON'T Ask IF**:
- Constitution already specifies it (audience tier, complexity, principles)
- Chapter index already defines it (prerequisites, part number)
- Task type is obvious from goal ("Add authentication" = code feature)
- Direct command is appropriate (simple `uv init` doesn't need AI workflow)

**Output**: Intelligence object containing:
```json
{
  "task_type": "book_chapter | code_feature | documentation | testing | refactoring",
  "audience_tier": "aspiring | intermediate | advanced | professional",
  "complexity_level": "A1 | A2 | B1 | B2 | C1 | C2",
  "prerequisites": ["chapter-11", "chapter-12"],
  "domain_skills": ["learning-objectives", "concept-scaffolding"],
  "teaching_pattern": "direct_commands | ai_companion | ai_orchestration",
  "ai_usage_strategy": "describe when AI adds value vs direct commands",
  "cognitive_load_limit": 7,
  "sandbox_validation_required": true,
  "commands_to_test": ["list all CLI commands students will run"],
  "ambiguities_clarified": {"question": "answer"}
}
```

---

## PHASE 0.5: GROUND TRUTH INTELLIGENCE GATHERING (For Tool/API/Framework Chapters)

**CRITICAL**: For chapters teaching external tools, APIs, frameworks, or CLIs, gather verified documentation BEFORE writing specifications. Never write specs from assumed knowledge.

**Detect if Intelligence Gathering Required**:

```
→ Analyze Phase 0 intelligence object:
  ├─ IF task_type includes: "external_tool" | "api_integration" | "framework_usage" | "cli_tutorial"
  ├─ THEN: Ground truth gathering REQUIRED
  └─ Examples requiring gathering:
      - Claude Code features (settings, plugins, MCP, commands)
      - Docker workflows (Dockerfile syntax, commands, best practices)
      - UV package manager (CLI commands, configuration)
      - GitHub API (endpoints, authentication, response schemas)
      - FastAPI framework (decorators, dependency injection patterns)
```

**If Ground Truth Gathering Required:**

```
→ STEP 1: Identify What Needs Verification
  ├─ Tool-specific claims:
  │   - Configuration schema (JSON/YAML structure, valid keys/values)
  │   - CLI commands (syntax, flags, expected output)
  │   - UI workflows (menu navigation, button labels)
  │   - API endpoints (URLs, parameters, response formats)
  │   - Version-specific features (tool X version Y has feature Z)
  │   - Integration patterns (how tool X connects to tool Y)
  │
  └─ Create preliminary list: "Claims requiring verification: [N]"

→ STEP 2: Assess Available Tools for Information Gathering
  ├─ Check connected MCP servers:
  │   - Context7 MCP (if available): Library documentation with 8000+ token depth
  │   - GitHub MCP (if available): Repository README, docs/, examples/
  │   - Filesystem MCP (if available): Local documentation files
  │   - Web MCP (if available): Official documentation sites
  │
  ├─ Check built-in tools:
  │   - WebFetch: Retrieve and parse official documentation URLs
  │   - Bash: Execute tool --help, man pages, version checks
  │   - Read: Access local docs if URLs provided by user
  │
  └─ Report available tools: "Can use: [tool1, tool2, tool3]"

→ STEP 3: Identify Information Gaps (Optional MCP Request)
  ├─ IF specialized MCP would help:
  │   - Example: "Context7 for React/Next.js documentation"
  │   - Example: "Playwright MCP for testing documentation"
  │   - Example: "Slack MCP for integration examples"
  │
  ├─ Ask user: "Would you like to connect [MCP-name] for deeper [tool] documentation?"
  │   - Explain benefit: "Context7 provides 8000+ token library docs vs WebFetch's summary"
  │   - Wait for user decision: YES (user connects) | NO (use available tools)
  │
  └─ Report: "Using tools: [final-list]"

→ STEP 4: Gather Verified Information (Tool-Adaptive Strategy)

  **Strategy A: Context7 MCP Available (PREFERRED for libraries/frameworks)**
  ```
  → For library/framework documentation:
    ├─ Use: mcp__context7__resolve-library-id
    │   - Input: Library name (e.g., "Claude Code", "FastAPI", "Docker")
    │   - Output: Context7-compatible library ID
    │
    ├─ Use: mcp__context7__get-library-docs
    │   - Input: Library ID + topic focus (e.g., "settings configuration")
    │   - Output: 5000-8000 tokens of focused documentation
    │   - Benefit: Comprehensive, structured, current
    │
    └─ Extract verified facts:
        - Actual configuration schema
        - Real command syntax with examples
        - Current version compatibility
        - Official usage patterns
  ```

  **Strategy B: WebFetch + Bash (Fallback for tools without Context7)**
  ```
  → For official documentation sites:
    ├─ Use: WebFetch("https://official-docs-url")
    │   - Retrieve: HTML converted to markdown
    │   - Extract: Settings, commands, API schemas
    │   - Limitation: Summary-level, may miss depth
    │
    ├─ Use: Bash for tool introspection:
    │   - Example: "claude --help" (capture actual CLI output)
    │   - Example: "docker version" (verify installation format)
    │   - Example: "uv --version" (check current syntax)
    │
    └─ Cross-reference: CLI output vs documentation
  ```

  **Strategy C: GitHub MCP (For open-source tools)**
  ```
  → For tools with GitHub repositories:
    ├─ Use: GitHub MCP to fetch:
    │   - README.md (installation, quick start)
    │   - docs/ directory (detailed guides)
    │   - examples/ (real working code)
    │   - CHANGELOG.md (version-specific features)
    │
    └─ Extract: Up-to-date patterns directly from source
  ```

→ STEP 5: Cache Verified Intelligence
  ├─ Create: intelligence/[feature-slug]-verified-docs.md
  │
  │   Structure:
  │   ```markdown
  │   # Verified Documentation: [Tool Name]
  │
  │   **Generated**: [ISO timestamp]
  │   **Tool Version**: [version if applicable]
  │   **Sources**: [list of tools/URLs used]
  │
  │   ## Verified Facts
  │
  │   ### Configuration Schema
  │   - Setting name: `permissions.defaultMode` (not `permission_mode`)
  │   - Valid values: "acceptEdits" | "default" | "plan"
  │   - Source: Context7 Claude Code docs / https://code.claude.com/docs/en/settings
  │   - Verified: [timestamp]
  │
  │   ### CLI Commands
  │   - Command: `/plugin marketplace add anthropics/skills`
  │   - Expected output: "Successfully added marketplace: anthropic-agent-skills"
  │   - Source: Terminal test + official docs
  │   - Verified: [timestamp]
  │
  │   ### UI Workflows
  │   - Navigation: `/plugin` → "Browse and install plugins" → select marketplace
  │   - Source: Actual tool testing
  │   - Verified: [timestamp]
  │
  │   ## Assumptions Flagged (Require Later Verification)
  │   - [List any unverified claims]
  │
  │   ## Tool Usage Log
  │   - Context7: Resolved "Claude Code" → /anthropics/claude-code
  │   - Context7: Fetched 6500 tokens on settings + plugins
  │   - WebFetch: https://github.com/anthropics/skills (plugin bundles)
  │   - Bash: Tested `/plugin marketplace add` in terminal
  │   ```
  │
  └─ Report: "✅ Cached [N] verified facts from [M] sources"

→ STEP 6: Quality Gate Before Spec
  ├─ Verify completeness:
  │   - All tool-specific claims have source URLs
  │   - Critical commands tested in actual environment
  │   - Configuration examples validated against schema
  │
  ├─ Calculate verification coverage:
  │   - Verified claims: [N]
  │   - Assumptions flagged: [M]
  │   - Coverage: [N/(N+M)]%
  │
  └─ Gate decision:
      - IF coverage < 80%: BLOCK spec creation, gather more intelligence
      - IF coverage >= 80%: ALLOW spec creation with flagged assumptions documented
      - Report: "✅ Verification coverage: [X]%. Ready to spec." OR "❌ Coverage too low: [Y]%. Need more sources."

WAIT: Intelligence gathering complete
→ User reviews: intelligence/[feature-slug]-verified-docs.md
→ User confirms: "✅ Verified facts look good" or requests additional verification
  ├─ If more verification needed: Return to STEP 4 with additional tools
  └─ If approved: Continue to PHASE 1 with verified intelligence
```

**Anti-Patterns Prevented**:
- ❌ Writing specs from memory ("I think Claude Code uses permission_mode...")
- ❌ Assuming API structure without checking docs ("Settings are probably JSON...")
- ❌ Inventing plausible-sounding setting names (permission_mode, output_format, editor)
- ❌ Skipping verification because "it seems right"

**Intelligence-First Success Criteria**:
- ✅ Every tool-specific claim has source citation
- ✅ Critical commands tested in actual environment
- ✅ Configuration examples validated against official schema
- ✅ Verification coverage >= 80%
- ✅ Assumptions explicitly flagged for later verification
- ✅ Tools used documented for reproducibility

**MCP Server Recommendations by Task Type**:

| Task Type | Recommended MCP | Why |
|-----------|-----------------|-----|
| Library/Framework docs | Context7 | 8000+ token depth, current versions |
| Open-source tools | GitHub | Source of truth (README, docs/) |
| API integration | Web + specific API MCP | Live endpoint testing |
| CLI tools | Bash + WebFetch | Introspection + official docs |
| Cloud services | Provider-specific MCP | AWS/Azure/GCP MCPs for live schemas |

**Output**: Verified intelligence cache ready for spec creation

---

## PHASE 1: SPECIFICATION + CLARIFICATION GATE

```
→ Invoke: /sp.specify [intelligence-context]
  ├─ Pass: Full intelligence object from Phase 0
  ├─ Apply: Evals-first (success criteria BEFORE implementation)
  ├─ Apply: AI usage strategy (when to use AI vs direct commands)
  ├─ Apply: Graduated teaching pattern (Tier 1/2/3 mapping)
  ├─ Create: specs/[feature-slug]/spec.md
  └─ Report: "Spec created with evals, AI strategy, and teaching tiers defined."

→ Invoke: /sp.clarify (Quality Gate)
  ├─ Read: specs/[feature-slug]/spec.md
  ├─ Identify: Underspecified areas, missing evals, ambiguous AI usage
  ├─ Check: Is this over-engineering simple tasks with AI?
  ├─ Ask: Up to 5 targeted clarification questions
  ├─ Update: spec.md with answers encoded
  └─ Report: "Spec clarified. AI usage strategy validated."

→ Create Feature Branch (AFTER spec exists)
  ├─ Derive branch name from spec directory (e.g., specs/part-4-chapter-15/ → part-4-chapter-15)
  ├─ Check current branch:
  │   IF current == main → Create new branch matching spec directory
  │   IF current == spec directory → Stay on it
  │   IF current != spec directory → Warn and ask user to switch
  ├─ Execute: git checkout -b [spec-directory-name] (only if on main)
  └─ Report: "✅ Branch: [branch-name]"

WAIT: User reviews spec.md
→ User confirms: "✅ Spec approved" or provides feedback
  ├─ If feedback: Update spec.md iteratively (may re-run /sp.clarify)
  └─ If approved: Continue to PHASE 2
```

**Spec Must Include**:
- **Evals Section**: Success criteria defined FIRST (before any implementation details)
- **AI Usage Strategy**: Clear distinction between direct commands vs AI collaboration
  - Example: "Students run `uv init` directly (1 second). Use AI for troubleshooting dependency conflicts."
- **Teaching Tiers** (if book chapter):
  - Tier 1: What concepts book teaches directly
  - Tier 2: When AI companion handles complexity
  - Tier 3: When AI orchestrates multi-step workflows
- **Duration**: Realistic time estimates (not inflated)
  - Example: "Installation takes 1 minute, not 45 minutes"
- **Cognitive Load**: Respects audience tier limits (A2 = max 7 concepts)
- **Verified Intelligence Integration** (if Phase 0.5 ran):
  - Reference: `intelligence/[feature-slug]-verified-docs.md`
  - All tool-specific examples must use VERIFIED facts (not assumptions)
  - Source citations required for configuration/commands/APIs
  - Example spec content:
    ```markdown
    ## Settings Configuration (Verified)

    Claude Code uses these ACTUAL settings ([verified](intelligence/chapter-5-verified-docs.md)):
    - `permissions.defaultMode`: "acceptEdits" | "default" | "plan"
    - `outputStyle`: "Concise" | "Explanatory" | "Verbose"

    Source: Context7 Claude Code library + terminal testing [2025-01-14]
    ```

**Anti-Pattern Detection**:
- ❌ If spec says "Use AI to run `docker build`" → FLAG: This is over-engineering
- ❌ If duration is 50+ minutes for simple operations → FLAG: Inflated estimate
- ❌ If every task involves "Ask AI to..." → FLAG: Not teaching strategic AI use
- ❌ If tool-specific examples don't cite intelligence cache → FLAG: Unverified assumptions
- ❌ If spec contradicts verified intelligence → BLOCK: Must use verified facts

---

## PHASE 2: PLANNING + ADR GATE

```
→ Invoke: /sp.plan [spec-context]
  ├─ Read: specs/[feature-slug]/spec.md (clarified)
  ├─ Apply: Teaching pattern (direct commands vs AI collaboration)
  ├─ Apply: Proficiency levels (CEFR if book chapter)
  ├─ Apply: Constitutional principles (Graduated Teaching, Co-Learning)
  ├─ Create: specs/[feature-slug]/plan.md
  └─ Report: "Plan created with teaching strategy and AI usage mapped."

→ Invoke: /sp.adr (Architectural Decision Gate)
  ├─ Read: specs/[feature-slug]/plan.md
  ├─ Detect: Architecturally significant decisions
  ├─ Suggest: "📋 Decision detected: [X]. Document with /sp.adr [title]?"
  ├─ Wait: User consent (never auto-create)
  ├─ Create: history/adr/[NNN]-[decision-title].md (if approved)
  └─ Report: "ADR created and linked." OR "Suggestion noted."

WAIT: User reviews plan.md (+ any ADRs)
→ User confirms: "✅ Plan approved" or provides feedback
  ├─ If feedback: Update plan.md iteratively
  └─ If approved: Continue to PHASE 3
```

**Plan Must Include**:
- **Direct Commands Section**: List all commands students run directly (with timing)
  - Example: "`uv init my-project` (1 second), `uv add requests` (1-3 seconds)"
- **AI Collaboration Section**: When/why to use AI (strategic, not everything)
  - Example: "Use AI for: understanding pyproject.toml structure, troubleshooting version conflicts, explaining virtual environments"
- **Lesson Structure** (if book chapter):
  - Estimated reading time (realistic)
  - "Try with AI" prompts (3-4 focused, not 8+ verbose)
  - Chapter 1 format: `### Prompt N: Title` → code block → `**Expected outcome:**` description

---

## PHASE 3: TASKS + ANALYSIS GATE

```
→ Invoke: /sp.tasks [spec+plan-context]
  ├─ Read: specs/[feature-slug]/spec.md + plan.md
  ├─ Apply: Direct commands vs AI workflow mapping
  ├─ Apply: Acceptance criteria from evals
  ├─ Create: specs/[feature-slug]/tasks.md
  └─ Report: "Tasks created with clear AI usage boundaries."

→ Invoke: /sp.analyze (Cross-Artifact Consistency Gate)
  ├─ Read: specs/[feature-slug]/{spec,plan,tasks}.md
  ├─ Validate: Objectives → plan → tasks traceability
  ├─ Check: AI usage strategy consistency (not over-engineering)
  ├─ Detect: Missing tasks, orphaned objectives, scope drift
  ├─ Report: Issues (critical/major/minor) + recommendations
  └─ Output: analysis-report.md

WAIT: User reviews tasks.md + analysis report
→ User confirms: "✅ Tasks approved" or provides feedback
  ├─ If critical issues: Must fix before proceeding
  ├─ If major issues: Should fix (user decision)
  ├─ If minor issues: Nice to fix (user decision)
  └─ If approved: Continue to PHASE 4
```

**Task Anti-Pattern Checks**:
- ❌ "Create AI prompt for running `npm install`" → Should be "Run `npm install` directly"
- ❌ "Write 50-minute lesson for 1-minute operation" → Should be realistic duration
- ❌ "8 verbose 'Try with AI' prompts" → Should be 3-4 focused prompts

---

## PHASE 4: IMPLEMENTATION + VALIDATION GATE

```
→ Invoke: /sp.implement [feature-slug]
  ├─ Read: specs/[feature-slug]/{spec,plan,tasks}.md (all approved)
  ├─ Strategy: Task-type dependent (lessons/code/tests/docs)
  ├─ Invoke: Appropriate subagent with FULL context including:
  │   - Intelligence object (audience, complexity, prerequisites)
  │   - AI usage strategy (direct commands vs collaboration)
  │   - Teaching pattern (Tier 1/2/3 if applicable)
  │   - Constitutional principles (Co-Learning, Graduated Teaching)
  │   - Anti-pattern warnings (don't over-engineer with AI)
  ├─ Create: Implementation artifacts (lessons, code, tests, etc.)
  └─ Report: "Implementation complete. Reviewing for AI over-engineering..."

→ Validation Review (Conceptual)
  ├─ For book chapters:
  │   - Duration realistic? (not inflated for simple operations)
  │   - Direct commands documented clearly? (not hidden behind AI)
  │   - "Try with AI" section uses Chapter 1 format? (3-4 focused prompts)
  │   - AI usage strategic? (troubleshooting, understanding, not trivial commands)
  │   - Line count reasonable? (not verbose explanations of simple operations)
  ├─ For code features:
  │   - Tests pass?
  │   - Code quality meets standards?
  │   - Documentation clear?
  ├─ Invoke: technical-reviewer + proof-validator (if applicable)
  └─ Report: PASS / CONDITIONAL PASS / FAIL with conceptual issues

→ If CONDITIONAL PASS or FAIL:
  ├─ Apply fixes for critical issues
  ├─ Re-run validation
  └─ Repeat until conceptual validation PASS

→ Sandbox Validation (Hands-On Testing) **CRITICAL**
  ├─ Philosophy: "If you have not run anything in sandbox, chances are it won't work"
  ├─ For book chapters with hands-on commands:
  │   - Extract ALL commands students will run
  │   - Test EVERY command in actual environment
  │   - Verify command syntax (CLI vs session commands)
  │   - Verify output matches lesson claims
  │   - Test "Try With AI" prompts for achievability
  │   - Document what actually works vs what's documented
  ├─ For code features:
  │   - Run full test suite in sandbox
  │   - Execute code in target environment
  │   - Verify deployment steps work end-to-end
  │   - Test edge cases and error paths
  ├─ Create: SANDBOX-AUDIT-REPORT.md with:
  │   - Commands tested (with actual output)
  │   - Errors found (with line numbers)
  │   - Fixes applied (with evidence)
  │   - Re-test results (verification)
  └─ Report: SANDBOX PASS / SANDBOX FAIL with specific command errors

→ If SANDBOX FAIL:
  ├─ Apply fixes for ALL command syntax errors
  ├─ Re-run sandbox tests
  ├─ Update SANDBOX-AUDIT-REPORT.md with fix verification
  └─ Repeat until SANDBOX PASS

WAIT: User reviews implementation + validation report + sandbox audit
→ User confirms: "✅ Implementation approved"
  └─ Proceed to PHASE 5
```

**Implementation Intelligence Context** (CRITICAL for subagents):

Pass this FULL context to subagents (lesson-writer, general-purpose, etc.):

```json
{
  "intelligence": { /* Phase 0 intelligence object */ },
  "ai_usage_strategy": "Direct commands for X, Y, Z. Use AI for A, B, C.",
  "teaching_pattern": "Tier 1: Book teaches [concepts]. Tier 2: AI handles [complexity]. Tier 3: AI orchestrates [scale].",
  "anti_patterns": [
    "Don't use AI for simple commands like `uv init`",
    "Don't inflate durations (1-minute tasks shouldn't be 45-minute lessons)",
    "Don't create 8+ verbose 'Try with AI' prompts (use 3-4 focused ones)",
    "Use Chapter 1 format: ### Prompt N: Title → code block → **Expected outcome:**"
  ],
  "constitutional_principles": [
    "Principle 13 (Graduated Teaching): Direct → AI Companion → AI Orchestration",
    "Principle 18 (Three Roles): AI as Teacher/Student/Co-Worker",
    "Core Philosophy #2 (Co-Learning): Bidirectional learning, not one-way instruction",
    "Core Philosophy #1 (AI Spectrum): Assisted → Driven → Native (teach when each applies)"
  ],
  "validation_criteria": {
    "duration_realistic": true,
    "direct_commands_clear": true,
    "ai_usage_strategic": true,
    "try_with_ai_format": "Chapter 1 clean format",
    "line_count_reasonable": true
  }
}
```

---

## PHASE 5: FINALIZATION + OPTIONAL GIT WORKFLOW

```
→ Update project tracking:
  ├─ For chapters: Update chapter-index.md status
  ├─ For features: Update feature tracking (if exists)
  └─ Report: "Project tracking updated."

→ Optional: Git workflow
  ├─ User may request: "/sp.git.commit_pr" for automated commit + PR
  ├─ Or: Manual commit with summary
  └─ Report: "Git workflow completed" OR "Manual commit required"

→ Create PHR (Prompt History Record):
  ├─ Document: User goal, intelligence derived, decisions, AI strategy applied
  ├─ Include: Anti-pattern checks performed, validation results
  ├─ Save: history/prompts/[feature-slug]/
  └─ Report: "PHR created for future reference."
```

---

## KEY LESSONS FROM CHAPTER 12 UV REVIEW

### What We Fixed

**Problem**: Lessons over-engineered simple operations with AI:
- "Tell AI to install UV" (30-second command became 10+ minute AI conversation)
- "Ask AI to run `uv init`" (1-second command became elaborate prompt)
- 667-line lesson for 3 simple commands
- 8 verbose "Try with AI" prompts instead of 3-4 focused ones
- Duration: 45 minutes for 1-minute installation

**Solution Applied**:
- Direct commands for simple operations: "Run `uv init`" (not "Ask AI to run `uv init`")
- AI for complex problems: troubleshooting, understanding concepts, strategic decisions
- Realistic durations: 15 minutes for installation (actual time), not 45 minutes
- Clean "Try with AI" format: 3-4 focused prompts using Chapter 1 style
- Line count reduction: 38% overall (3,456 → 2,144 lines)

**Results**:
- Students learned WHEN to use AI strategically (not for everything)
- Lessons became efficient, not verbose
- Constitutional alignment: Graduated Teaching (direct → AI → orchestration)

### Encode These Lessons

**In Specifications**:
- ✅ "Students run `uv init` directly (1 second)"
- ❌ "Students tell AI to initialize a project using UV"

**In Plans**:
- ✅ "Direct commands section: Installation (30 sec), Project creation (1 sec), Add dependency (1-3 sec)"
- ✅ "AI collaboration section: Troubleshooting PATH errors, understanding pyproject.toml, resolving version conflicts"
- ❌ "AI workflow for every command"

**In Implementations**:
- ✅ Chapter 1 "Try with AI" format (3-4 prompts, clean structure)
- ✅ Realistic durations matching actual time
- ❌ Verbose explanations of trivial operations
- ❌ 8+ "Try with AI" prompts with lengthy pre-explanations

---

## CRITICAL SUCCESS FACTORS

1. **Vertical Intelligence**: Constitution + project context read FIRST, questions SECOND
2. **Goal-Oriented**: User states GOAL, AI derives STRATEGY
3. **AI Usage Strategy**: Clear distinction between direct commands (simple) vs AI (complex)
4. **Graduated Teaching**: Apply Principle 13 (Book → AI Companion → AI Orchestration)
5. **Quality Gates**: Every phase prevents bad patterns from propagating
6. **Context Preservation**: Full intelligence + AI strategy passed through all phases
7. **Anti-Pattern Detection**: Flag over-engineering with AI for simple tasks
8. **Constitutional Compliance**: All outputs align with vision and principles
9. **Realistic Expectations**: Durations, line counts, and complexity match actual needs
10. **Shipping-Ready**: Built-in quality, not post-hoc validation

---

## REFERENCES

- **Constitution**: `.specify/memory/constitution.md` (source of truth)
- **Project Structure**:
  - Chapter Index: `specs/book/chapter-index.md`
  - Specs Directory: `specs/`
  - Skills Library: `.claude/skills/`
- **LoopFlow Commands**:
  - `/sp.specify` - Create specifications
  - `/sp.clarify` - Clarify underspecified areas
  - `/sp.plan` - Create implementation plans
  - `/sp.adr` - Document architectural decisions
  - `/sp.tasks` - Generate task checklists
  - `/sp.analyze` - Cross-artifact consistency
  - `/sp.implement` - Execute implementation
  - `/sp.git.commit_pr` - Git workflow automation

---

## ONE COMMAND. UNIVERSAL INTELLIGENCE. COMPLETE WORKFLOW.

Run `/sp.loopflow [goal]` and the system executes:

**PHASE 0: Intelligent Discovery** → Constitution + context + AI usage strategy derived
**PHASE 1: Specification + Clarification** → Evals-first + AI strategy + approval gate
**PHASE 2: Planning + ADR** → Teaching pattern + direct vs AI mapping + approval gate
**PHASE 3: Tasks + Analysis** → Anti-pattern checks + consistency + approval gate
**PHASE 4: Implementation + Validation** → Context-aware execution + quality verification + approval gate
**PHASE 5: Finalization** → Tracking + git + PHR

**Result**: Shipping-ready output with:
- ✅ Strategic AI use (not over-engineered)
- ✅ Constitutional compliance
- ✅ Realistic expectations
- ✅ Quality built-in
- ✅ Decision trail documented
