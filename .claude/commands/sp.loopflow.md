---
description: Universal intelligence-driven LoopFlow workflow orchestrator implementing AIDD (AI-Driven Development) methodology. Reads constitution + project context to derive requirements automatically. Chains /sp.specify → /sp.plan → /sp.tasks → /sp.implement with quality gates. Works for ANY chapter, feature, or task.
---

# /sp.loopflow: Universal LoopFlow Orchestrator

**Purpose**: Execute the complete LoopFlow+ workflow (Evals → Spec → Plan → Tasks → Implement → Validate) for ANY chapter, feature, or task using **vertical intelligence** (constitution + project context + domain skills). Implements **AIDD (AI-Driven Development)** methodology through spec-first workflow with intelligent verification.

**Intelligence Sources**:
- Constitution: Project vision, core principles, target audience, philosophies (AIDD foundation)
- Project Context: Chapter index, book structure, existing specs
- Domain Skills: Available skills library (`.claude/skills/`)
- User Intent: Natural language description of what to build

**Workflow Pattern** (AIDD Spec-First):
1. **Phase 0**: Deep Search (pre-spec intelligence - ALWAYS runs)
2. **Phase 1**: Specification (spec-first using /sp.specify in terminal - AIDD core)
3. **Phase 0.5**: Deep Research (post-spec verification - CONDITIONAL)
4. **Phases 2-5**: Plan (/sp.plan) → Tasks (/sp.tasks) → Implement (/sp.implement) → Validate

**Adaptive Workflow**: 0-5 targeted questions based on what intelligence can't derive. NO hardcoded questionnaires.

Tip: Use docker sandbox setup your container and run your code in a controlled environment. Validate using this stratey when possible.

## User Input

```text
$ARGUMENTS
```

---

## ⚠️ UNIVERSAL AI USAGE DECISION FRAMEWORK

### Constitutional Grounding

✅ **Principle 2 (Co-Learning Partner):** AI used strategically where it adds value, not reflexively for everything
✅ **Core Philosophy #1 (AI Spectrum):** Teaching when Assisted vs. Driven vs. Native makes sense
✅ **Graduated Teaching (Principle 13):** Direct execution for foundations, AI for complexity, AI orchestration for scale
✅ **"Specs Are the New Syntax":** Focus on high-value specification work, not trivial command execution

### Decision Framework: When to Use AI vs. Direct Execution

**Apply this reasoning to ANY tool, framework, or operation:**

```
→ Analyze the task characteristics:
  ├─ Is it deterministic? (same inputs → same outputs, no decisions required)
  ├─ Is it simple? (1-2 steps, no branching logic, documented procedure)
  ├─ Is it fast? (completes in < 5 seconds with direct command)
  └─ Is it foundational? (students need to understand the direct mechanism)

→ Decision tree:
  ├─ IF all true → DIRECT EXECUTION
  │   - Document the command clearly
  │   - Show expected output
  │   - Execution time: actual duration (not inflated)
  │   - AI role: troubleshooting errors, understanding concepts
  │
  ├─ IF any false → EVALUATE COMPLEXITY
  │   ├─ Non-deterministic (requires decisions) → AI COMPANION
  │   ├─ Multi-step with branches → AI COMPANION
  │   ├─ Requires understanding tradeoffs → AI COMPANION
  │   └─ Involves 10+ items or orchestration → AI ORCHESTRATION

→ Examples across different domains:
  
  **Package Management** (uv, npm, pip):
  - `uv init my-project` → DIRECT (deterministic, 1 step, < 1 sec)
  - Resolving dependency conflicts → AI COMPANION (non-deterministic, requires analysis)
  - Setting up 10 microservices → AI ORCHESTRATION (scale)
  
  **Version Control** (git):
  - `git status` → DIRECT (deterministic, 1 step, < 1 sec)
  - Resolving merge conflicts → AI COMPANION (requires understanding code context)
  - Managing 50 feature branches → AI ORCHESTRATION (scale)
  
  **Containerization** (docker):
  - `docker build -t myapp .` → DIRECT (deterministic, documented)
  - Optimizing multi-stage builds → AI COMPANION (requires tradeoff analysis)
  - Orchestrating 20 microservices → AI ORCHESTRATION (scale with k8s)
  
  **Cloud Deployment** (AWS, Azure):
  - Deploy via CLI with known config → DIRECT (if well-documented, < 2 min)
  - Choosing instance types for workload → AI COMPANION (requires analysis)
  - Multi-region failover architecture → AI ORCHESTRATION (complex strategy)
```

### Teaching Pattern (Graduated)

**Tier 1 - Foundational (Book Teaches)**:
- Direct commands for deterministic operations
- Clear documentation of syntax and expected output
- Build muscle memory and direct understanding
- AI positioned as troubleshooter (errors) and explainer (concepts)

**Tier 2 - Complex Execution (AI Companion)**:
- Student specifies WHAT (intent, requirements, constraints)
- AI handles HOW (complex syntax, multi-step procedures, tradeoffs)
- Student validates and understands the output
- Learn strategy and intent, not memorization

**Tier 3 - Scale Operations (AI Orchestration)**:
- Operations involving 10+ items or multi-file workflows
- Student orchestrates high-level strategy
- AI manages tactical execution and coordination
- Learn supervision and quality assurance skills

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

## PHASE 0: DEEP SEARCH (Pre-Spec Intelligence)

**Purpose**: Gather constitutional intelligence and project context BEFORE spec creation. This is the foundation for AIDD spec-first methodology.

**STEP 1: Read Authoritative Sources (Execute NOW)**

YOU MUST immediately read these files (no user questions yet):

1. **Constitution** (`.specify/memory/constitution.md`):
   - Extract project vision, core principles, target audience
   - Extract Nine Pillars, AI Development Spectrum
   - Extract teaching patterns and philosophies

2. **Chapter Index** (IF book chapter) (`specs/book/chapter-index.md`):
   - Extract chapter number → Part mapping
   - Extract prerequisites (what chapters must come before)
   - Extract complexity tier (CEFR levels)
   - Extract status (planned/in-progress/completed)

3. **Existing Specs** (`specs/` directory):
   - Search for similar chapters/features for pattern reference
   - Extract naming conventions and structure examples

4. **Domain Skills** (`.claude/skills/`):
   - List available skills for this task type
   - Identify teaching patterns (if educational content)
   - Identify validation strategies (if technical feature)

**STEP 2: Automatic Derivations (Apply Constitutional Reasoning)**

From sources above, derive WITHOUT asking user:

**For Book Chapters:**
- Part number (from chapter number range in chapter-index)
- Audience tier: Part 1-3 (Aspiring/Beginner A1-A2) → Part 4-5 (Intermediate B1-B2) → Part 6-8 (Advanced C1) → Part 9-13 (Professional C2)
- Complexity level and cognitive load limits
- Prerequisites (chapters that must exist first)
- Relevant domain skills (learning-objectives, concept-scaffolding, code-example-generator, etc.)
- Teaching pattern (Graduated Teaching Principle 13: Direct → AI Companion → AI Orchestration)

**For Code Features:**
- Task type (authentication, API, database, deployment, etc.)
- Audience (professional developers)
- Prerequisites (existing codebase components)
- Relevant domain skills (technical-clarity, assessment-builder, etc.)

**For Documentation/Testing:**
- Scope from file paths and existing content
- Audience (contributors, users, developers)
- Prerequisites (what must exist to document/test)

**CRITICAL: Apply AI Usage Decision Framework**
- Identify which tasks are deterministic/simple → document as DIRECT
- Identify which tasks require decisions/complexity → position as AI COMPANION
- Identify which tasks involve scale (10+) → position as AI ORCHESTRATION
- This is PRINCIPLE-BASED reasoning, not tool-specific templates

**STEP 3: Apply Constitution Context (Encode Principles)**

Extract and encode these principles into ALL downstream phases:

**From Core Philosophy:**
1. **Evals-First Development**: Define success criteria BEFORE specs
2. **Co-Learning Partnership**: Bidirectional learning (AI teaches student, student teaches AI via feedback loop)
3. **Spec-First Development**: "Specs Are the New Syntax" - articulating intent is the primary skill
4. **Validation-First Safety**: Never trust, always verify

**From Graduated Teaching (Principle 13):**
- **Tier 1 (Direct Execution)**: Teach stable, foundational operations that students execute directly
- **Tier 2 (AI Companion)**: AI handles complex execution from specifications
- **Tier 3 (AI Orchestration)**: AI orchestrates multi-step workflows at scale (10+ items)

**From AI Development Spectrum:**
- **Assisted (2-3x)**: AI as helper for simple tasks
- **Driven (5-10x)**: AI generates from specs ← Primary focus for most work
- **Native (50-99x)**: AI as core product capability

**From Target Audience:**
- **Aspiring (A1-A2)**: Max 7 concepts per section, 2 options max, cognitive load managed
- **Intermediate (B1-B2)**: 7 concepts per section, 3-4 options, tradeoff discussions
- **Advanced (C1)**: 10 concepts per section, 5+ options, architecture patterns
- **Professional (C2)**: No artificial limits, production complexity

**STEP 4: Identify Genuine Ambiguities (0-5 Questions Max)**

Now that you have full context, identify ONLY what's genuinely ambiguous:

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
- Decision framework applies clearly (simple command = direct execution)

**Output**: Intelligence object containing:
```json
{
  "task_type": "book_chapter | code_feature | documentation | testing | refactoring",
  "audience_tier": "aspiring | intermediate | advanced | professional",
  "complexity_level": "A1 | A2 | B1 | B2 | C1 | C2",
  "prerequisites": ["chapter-11", "chapter-12"],
  "domain_skills": ["learning-objectives", "concept-scaffolding"],
  "teaching_pattern": "direct_execution | ai_companion | ai_orchestration",
  "ai_usage_strategy": "Direct: [operations]. AI Companion: [complexity]. AI Orchestration: [scale].",
  "cognitive_load_limit": 7,
  "validation_required": true,
  "ambiguities_clarified": {"question": "answer"}
}
```

**Output from Phase 0**: 
- Intelligence object ready for spec creation
- 0-5 clarifying questions answered
- Constitutional principles identified for application
- Ready to proceed to Phase 1 (Specification)

**CRITICAL**: Do NOT create git branch yet. Branch creation happens in PHASE 1 AFTER spec.md is created.

**CRITICAL**: Do NOT run Phase 0.5 (Deep Research) yet. That happens AFTER spec creation if needed (see Phase 0.5 below).

---

## PHASE 1: SPECIFICATION + CLARIFICATION GATE

**THIS PHASE INVOKES /sp.specify AUTOMATICALLY** - No user permission needed to invoke the command.

**STEP 1: Prepare Full Context** (Automatic)

Gather context from Phase 0:
- Intelligence object (task type, audience, complexity, prerequisites)
- AI usage strategy (direct vs. AI companion vs. orchestration)
- Teaching pattern (Tier 1/2/3 mapping)
- Domain skills required
- Verified intelligence cache (if Phase 0.5 ran)
- Constitutional principles to apply

**STEP 2: Invoke /sp.specify NOW** (Execute immediately)

YOU MUST invoke this command NOW with full context:

```
/sp.specify [feature-slug]

[Pass complete intelligence object]
[Pass AI usage strategy]
[Pass teaching pattern]
[Pass evals-first requirement]
[Pass verified intelligence reference if available]
[Pass constitutional principles]
```

**CRITICAL**: DO NOT ask user "Should I run /sp.specify?" - EXECUTE IT IMMEDIATELY.

**STEP 3: Wait for /sp.specify Completion**

The command will create: `specs/[feature-slug]/spec.md`

**STEP 4: Invoke /sp.clarify (Quality Gate - Execute Automatically)**

YOU MUST invoke this command NOW to identify underspecified areas:

```
/sp.clarify [feature-slug]

Read: specs/[feature-slug]/spec.md
Identify: Underspecified areas, missing evals, ambiguous AI usage
Check: Is this over-engineering simple tasks with AI?
Ask: Up to 5 targeted clarification questions
Update: spec.md with answers encoded
```

**CRITICAL**: DO NOT ask user "Should I run /sp.clarify?" - EXECUTE IT IMMEDIATELY after /sp.specify completes.

**STEP 5: Create Feature Branch NOW** (Execute After Spec Exists)

YOU MUST create the feature branch NOW using these literal bash commands:

```bash
# Step 1: Get current branch name
CURRENT_BRANCH=$(git branch --show-current)

# Step 2: Derive spec directory name from spec path
# Example: specs/part-4-chapter-15/ → part-4-chapter-15
SPEC_DIR="[feature-slug]"  # Replace with actual spec directory name

# Step 3: Check if we're already on the correct branch
if [ "$CURRENT_BRANCH" = "$SPEC_DIR" ]; then
    echo "✅ Already on branch: $SPEC_DIR"
elif [ "$CURRENT_BRANCH" = "main" ]; then
    # Step 4: Create new branch from main
    git checkout -b "$SPEC_DIR"
    echo "✅ Created and switched to branch: $SPEC_DIR"
else
    # Step 5: We're on a different feature branch - warn user
    echo "⚠️  Currently on branch: $CURRENT_BRANCH"
    echo "⚠️  Expected branch: $SPEC_DIR"
    echo "Please switch branches manually or confirm to continue on $CURRENT_BRANCH"
fi
```

**Execute these commands NOW** - do not ask permission, do not skip this step.

**STEP 6: Report to User and BLOCK for Approval**

Output:
```
✅ PHASE 1 COMPLETE: Specification Created & Clarified

📋 Spec: specs/[feature-slug]/spec.md
🌿 Branch: [branch-name]

Spec includes:
- ✅ Evals section (success criteria defined FIRST)
- ✅ AI usage strategy (direct vs. companion vs. orchestration)
- ✅ Teaching tiers (if book chapter)
- ✅ Duration estimates (realistic, not inflated)
- ✅ Cognitive load limits (respects audience tier)
- ✅ Verified intelligence integration (if Phase 0.5 ran)

🚫 BLOCKED: You MUST review specs/[feature-slug]/spec.md before proceeding.

Respond with:
- ✅ "Spec approved" → Continue to PHASE 2
- 📝 Feedback → Update spec iteratively
- ❓ Questions → Clarification dialog
```

**DO NOT PROCEED** until user explicitly confirms "✅ Spec approved" or equivalent.

**Spec Must Include**:
- **Evals Section**: Success criteria defined FIRST (before implementation details)
- **AI Usage Strategy**: Clear distinction:
  - Direct: `uv init` (deterministic, 1 sec)
  - AI Companion: Troubleshooting dependency conflicts
  - AI Orchestration: Setting up 10 microservices
- **Teaching Tiers** (if book chapter):
  - Tier 1: Direct commands book teaches
  - Tier 2: When AI companion handles complexity
  - Tier 3: When AI orchestrates multi-step workflows
- **Duration**: Realistic time estimates (1 min for simple ops, not 45 min)
- **Cognitive Load**: Respects audience tier limits
- **Verified Intelligence Integration** (if Phase 0.5 ran):
  - Reference: `intelligence/[feature-slug]-verified-docs.md`
  - All tool-specific examples use VERIFIED facts
  - Source citations for configuration/commands/APIs

**Anti-Pattern Detection in Spec**:
- ❌ If spec says "Use AI to run `docker build`" → FLAG: Over-engineering
- ❌ If duration is 50+ minutes for simple operations → FLAG: Inflated estimate
- ❌ If every task involves "Ask AI to..." → FLAG: Not teaching strategic AI use
- ❌ If tool-specific examples don't cite intelligence cache → FLAG: Unverified assumptions
- ❌ If spec contradicts verified intelligence → BLOCK: Must use verified facts

---

## PHASE 0.5: DEEP RESEARCH (Post-Spec Verification - CONDITIONAL)

**Purpose**: After spec creation, validate assumptions and gather deeper intelligence if spec reveals verification needs. This phase is OPTIONAL and runs ONLY when triggered by spec analysis.

**This Phase Is CONDITIONAL** - It runs ONLY if verification needs are detected.

### DECISION GATE: Does Spec Require Deep Research?

After user reviews spec in Phase 1, analyze if deep research is needed:

**Triggers for Phase 0.5** (run deep research if ANY apply):

```
→ Analyze spec.md content:
  ├─ Tool/API/Framework-Specific Claims:
  │   - Spec mentions external tool configuration (settings schema, CLI flags)
  │   - Spec describes API contracts (endpoints, parameters, responses)
  │   - Spec references framework patterns (decorators, lifecycle, conventions)
  │   - Spec includes version-specific features
  │   → TRIGGER: Need to verify against authoritative sources
  │
  ├─ High Assumption Rate:
  │   - Spec has >30% content marked as assumptions/unverified
  │   - Spec uses tentative language ("probably", "might", "should be")
  │   - Spec contradicts known information or existing verified docs
  │   → TRIGGER: Need to validate assumptions
  │
  ├─ User Requests Verification:
  │   - User says: "Validate spec assumptions before planning"
  │   - User says: "Research [tool/API] to confirm this is accurate"
  │   - User says: "Check official docs to verify"
  │   → TRIGGER: Explicit user request
  │
  ├─ Missing Critical Context:
  │   - Spec identifies gaps in understanding ("need to research X")
  │   - Spec defers decisions ("to be determined after research")
  │   - Spec requests examples from official sources
  │   → TRIGGER: Spec explicitly requests more intelligence
  │
  └─ No Triggers Found:
      → SKIP Phase 0.5, proceed directly to Phase 2 (Planning)
```

### IF TRIGGERED: Execute Deep Research

**STEP 1: Reason from Constitutional Principles (Tool-Agnostic)**

Apply verification strategy based on WHAT tools are available, not hardcoded tool list:

```
→ What needs verification?
  ├─ Extract from spec: All tool-specific claims, API contracts, configuration schemas
  ├─ Prioritize: Critical assumptions that affect implementation
  ├─ Create: Verification requirements list with priorities
  │
→ What verification methods are available?
  ├─ Check: Connected MCP servers (Context7, GitHub, etc.)
  ├─ Check: Built-in tools (WebFetch, Bash, Read)
  ├─ Check: User-provided resources (local docs, URLs)
  ├─ Assess: Which tools best match verification needs
  │
→ Should we suggest super-orchestra mode?
  ├─ IF (verification requires 4+ different sources AND synthesis complexity high)
  │   → Suggest: "This requires multi-source synthesis. Use super-orchestra mode?"
  │   → Wait for user decision: YES (switch mode) | NO (standard tools)
  │
  └─ Report: "Verification strategy: [tools-and-approach]"
```

**STEP 2: Execute Verification Strategy**

Use WHATEVER tools are available (adapt to context):

```
→ IF Context7 MCP available:
  - Resolve library ID for the tool/framework
  - Fetch 5000-8000 tokens of focused documentation
  - Extract verified facts (configuration, commands, patterns)

→ ELSE IF WebFetch available:
  - Fetch official documentation URLs
  - Extract and convert HTML to markdown
  - Cross-reference with existing knowledge

→ IF Bash available (for CLI tools):
  - Execute tool --help, --version for syntax verification
  - Test example commands in safe environment
  - Capture actual output as source of truth

→ IF GitHub MCP available (for open-source tools):
  - Fetch README.md, docs/, examples/ from repository
  - Extract current patterns from source
  - Check CHANGELOG for version-specific features

→ ALWAYS:
  - Document verification source for each claim
  - Flag unverified assumptions explicitly
  - Calculate coverage: verified/(verified + assumptions)
```

**STEP 3: Create Verified Intelligence Cache**

Document all findings in: `intelligence/[feature-slug]-verified-docs.md`

```markdown
# Verified Documentation: [Tool/API/Framework Name]

**Generated**: [ISO timestamp]
**Tool Version**: [version if applicable]
**Verification Tools Used**: [Context7 | WebFetch | Bash | GitHub | etc.]
**Triggered By**: [spec assumption rate | user request | tool-specific claims]

## Verified Facts

### Configuration
- Setting: `key.name` accepts ["value1", "value2", "value3"]
- Source: [Context7 library docs | WebFetch https://url | Bash output]
- Verified: [timestamp]

### Commands/APIs
- Syntax: `command subcommand --flag=value`
- Expected output: [actual output captured]
- Source: [verification method]
- Verified: [timestamp]

## Assumptions Flagged (Still Unverified)
- [List remaining assumptions with reasoning]

## Verification Coverage
- Verified claims: [N]
- Assumptions remaining: [M]
- Coverage: [N/(N+M)]%

## Verification Log
- [Tool 1]: [What verified, how, when]
- [Tool 2]: [What verified, how, when]
```

**STEP 4: Update Spec with Verified Intelligence**

```
→ Update specs/[feature-slug]/spec.md:
  ├─ Replace assumptions with verified facts
  ├─ Add source citations for tool-specific content
  ├─ Reference intelligence cache: `intelligence/[feature-slug]-verified-docs.md`
  ├─ Update confidence level (assumption → verified)
  │
→ IF changes are significant (>20% of spec updated):
    ├─ Re-run /sp.clarify to check for new gaps
    └─ Request user re-approval of updated spec

→ ELSE:
    └─ Report changes and confirm user still approves
```

**STEP 5: Quality Gate Before Planning**

```
→ Verify verification completeness:
  ├─ Critical claims verified: [yes/no]
  ├─ Verification coverage: [percentage]
  ├─ Remaining assumptions: [count and acceptable?]
  │
→ Gate decision:
    IF coverage < 70% AND critical claims unverified:
      → BLOCK: Need more verification sources
      → Suggest: Additional tools or super-orchestra mode
    
    IF coverage >= 70% OR user accepts remaining assumptions:
      → ALLOW: Proceed to Phase 2 (Planning)
      → Report: "✅ Verification complete. Coverage: [X]%"
```

**STEP 6: Report and Continue**

Output:
```
✅ PHASE 0.5 COMPLETE: Deep Research & Verification

📋 Verified Intelligence: intelligence/[feature-slug]-verified-docs.md
📋 Updated Spec: specs/[feature-slug]/spec.md

Verification results:
- Verified facts: [N]
- Remaining assumptions: [M]
- Coverage: [X]%
- Tools used: [list]

Changes to spec:
- [Summary of what was verified and updated]

Ready to proceed to Phase 2 (Planning).
```

### IF NOT TRIGGERED: Skip Phase 0.5

If decision gate determines no deep research needed:

Output:
```
ℹ️  PHASE 0.5 SKIPPED: No verification triggers detected

Spec analysis:
- ✅ No tool-specific claims requiring external verification
- ✅ Low assumption rate (< 30%)
- ✅ No user request for additional research
- ✅ No critical gaps identified

Proceeding directly to Phase 2 (Planning) with Phase 0 intelligence.
```

---

## PHASE 2: PLANNING + ADR GATE

**THIS PHASE INVOKES /sp.plan AUTOMATICALLY** - No user permission needed to invoke the command.

**STEP 1: Verify Prerequisites** (Automatic Check)

Confirm:
- ✅ specs/[feature-slug]/spec.md exists and is approved
- ✅ User confirmed "Spec approved"
- ✅ Feature branch exists and is checked out

**STEP 2: Invoke /sp.plan NOW** (Execute Immediately)

⚠️ **CRITICAL WARNING**: DO NOT run `.specify/scripts/bash/setup-plan.sh` manually!
- This script is for the `/sp.plan` command to use internally
- Running it directly will OVERWRITE any existing plan.md with the template
- ONLY use `/sp.plan` slash command (which handles setup script safely)

YOU MUST invoke this command NOW with full context:

```
/sp.plan [feature-slug]

Read: specs/[feature-slug]/spec.md (clarified and approved)
Read: Existing lesson file (to extract YAML frontmatter structure)
Apply: Teaching pattern (direct commands vs AI collaboration)
Apply: Proficiency levels (CEFR if book chapter)
Apply: Constitutional principles (Graduated Teaching, Co-Learning)
Apply: AI usage decision framework (direct/companion/orchestration)
Apply: Verified intelligence (if Phase 0.5 ran)
Apply: Minimal code examples (if first hands-on chapter)
Apply: YAML frontmatter format (match existing lesson metadata structure)
Create: specs/[feature-slug]/plan.md
```

**CRITICAL LESSON-SPECIFIC INSTRUCTIONS**:

For **first hands-on chapters** (e.g., Chapter 12 Python tooling, Chapter 7 Bash basics):
- Code examples MUST be minimal (3-5 line snippets)
- Focus is on **tool installation, configuration, verification**
- NOT on teaching programming language features
- Example: "print('hello')" to test Ruff, NOT complex functions

For **all book chapters**:
- plan.md MUST specify YAML frontmatter format matching existing lessons
- Read existing lesson files to extract metadata structure
- Include: title, chapter, lesson, duration_minutes, skills (CEFR/Bloom's/DigComp), learning_objectives, cognitive_load
- Lesson-writer subagent will convert plan to actual YAML

**CRITICAL**: DO NOT ask user "Should I run /sp.plan?" - EXECUTE IT IMMEDIATELY.

**STEP 3: Invoke /sp.adr (Architectural Decision Gate - Suggest Only)**

After /sp.plan completes, YOU MUST check for architectural decisions:

Read: specs/[feature-slug]/plan.md
Detect: Architecturally significant decisions (lesson structure, pedagogical approaches, tech choices)

IF architectural decisions detected:
  Suggest: "📋 Architectural decision detected: [X]. Document with /sp.adr [title]?"
  WAIT: User consent to create ADR (NEVER auto-create)
  
  IF user approves:
    Invoke: /sp.adr [title]
    Create: history/adr/[NNN]-[decision-title].md
  ELSE:
    Note: "ADR suggestion recorded for future reference"

**STEP 4: Report to User and BLOCK for Approval**

Output:
```
✅ PHASE 2 COMPLETE: Plan Created

📋 Plan: specs/[feature-slug]/plan.md
📋 ADRs: [list if any created]

Plan includes:
- ✅ Direct Commands Section (with timing)
  Example: "`uv init my-project` (1 second)"
- ✅ AI Collaboration Section (strategic use cases)
  Example: "Use AI for: understanding pyproject.toml, resolving version conflicts"
- ✅ Lesson Structure (if book chapter): Reading time, "Try with AI" prompts
- ✅ Teaching tier mapping (direct → companion → orchestration)

🚫 BLOCKED: You MUST review specs/[feature-slug]/plan.md before proceeding.

Respond with:
- ✅ "Plan approved" → Continue to PHASE 3
- 📝 Feedback → Update plan iteratively
- ❓ Questions → Clarification dialog
```

**DO NOT PROCEED** until user explicitly confirms "✅ Plan approved" or equivalent.

**Plan Must Include**:
- **Direct Commands Section**: List commands students execute directly (with timing)
- **AI Collaboration Section**: When/why to use AI (strategic, not everything)
- **Lesson Structure** (if book chapter): Realistic estimates, focused "Try with AI" prompts
- **Teaching tier mapping**: Clear delineation of direct vs. companion vs. orchestration

---

## PHASE 3: TASKS + ANALYSIS GATE

**THIS PHASE INVOKES /sp.tasks AUTOMATICALLY** - No user permission needed to invoke the command.

**STEP 1: Verify Prerequisites** (Automatic Check)

Confirm:
- ✅ specs/[feature-slug]/spec.md exists and is approved
- ✅ specs/[feature-slug]/plan.md exists and is approved
- ✅ User confirmed "Plan approved"

**STEP 2: Invoke /sp.tasks NOW** (Execute Immediately)

YOU MUST invoke this command NOW with full context:

```
/sp.tasks [feature-slug]

Read: specs/[feature-slug]/spec.md + plan.md
Apply: Direct commands vs AI workflow mapping
Apply: Acceptance criteria from evals
Apply: AI usage decision framework
Create: specs/[feature-slug]/tasks.md
```

**CRITICAL**: DO NOT ask user "Should I run /sp.tasks?" - EXECUTE IT IMMEDIATELY.

**STEP 3: Invoke /sp.analyze (Cross-Artifact Consistency Gate - Execute Automatically)**

YOU MUST invoke this command NOW for quality assurance:

```
/sp.analyze [feature-slug]

Read: specs/[feature-slug]/{spec,plan,tasks}.md
Validate: Objectives → plan → tasks traceability
Check: AI usage strategy consistency (not over-engineering)
Detect: Missing tasks, orphaned objectives, scope drift
Report: Issues (critical/major/minor) + recommendations
Output: analysis-report.md
```

**CRITICAL**: DO NOT ask user "Should I run /sp.analyze?" - EXECUTE IT IMMEDIATELY after /sp.tasks completes.

**STEP 4: Report to User and BLOCK for Approval**

Output:
```
✅ PHASE 3 COMPLETE: Tasks Created & Analyzed

📋 Tasks: specs/[feature-slug]/tasks.md
📋 Analysis: specs/[feature-slug]/analysis-report.md

Analysis results:
- Critical issues: [count]
- Major issues: [count]
- Minor issues: [count]

Task anti-pattern checks:
- ✅ No "Create AI prompt for `npm install`" patterns found
- ✅ Durations are realistic (not 50min for 1min operations)
- ✅ "Try with AI" prompts are focused (3-4, not 8+)

🚫 BLOCKED: You MUST review tasks.md + analysis-report.md before proceeding.

Respond with:
- ✅ "Tasks approved" → Continue to PHASE 4 (IF no critical issues)
- 🔧 "Fix critical issues first" → Must fix before proceeding
- 📝 Feedback → Update tasks iteratively
- ❓ Questions → Clarification dialog
```

**DO NOT PROCEED** until:
- User explicitly confirms "✅ Tasks approved" or equivalent
- AND critical issues are resolved (if any were found)

**Task Anti-Pattern Checks**:
- ❌ "Create AI prompt for running `npm install`" → Should be "Run `npm install` directly"
- ❌ "Write 50-minute lesson for 1-minute operation" → Should be realistic duration
- ❌ "8 verbose 'Try with AI' prompts" → Should be 3-4 focused prompts

---

## PHASE 4: IMPLEMENTATION + VALIDATION GATE

**THIS PHASE INVOKES /sp.implement AUTOMATICALLY** - No user permission needed to invoke the command.

**STEP 1: Verify Prerequisites** (Automatic Check)

Confirm:
- ✅ specs/[feature-slug]/spec.md exists and is approved
- ✅ specs/[feature-slug]/plan.md exists and is approved
- ✅ specs/[feature-slug]/tasks.md exists and is approved
- ✅ User confirmed "Tasks approved"
- ✅ Critical issues resolved (if any were found in analysis)

**STEP 2: Invoke /sp.implement NOW** (Execute Immediately)

YOU MUST invoke this command NOW with FULL intelligence context:

```
/sp.implement [feature-slug]

Read: specs/[feature-slug]/{spec,plan,tasks}.md (all approved)
Pass to subagent: COMPLETE intelligence object including:

{
  "intelligence": { /* Phase 0 intelligence */ },
  "ai_usage_strategy": "Direct: [ops]. AI Companion: [complexity]. AI Orchestration: [scale].",
  "teaching_pattern": "Tier 1: Direct [concepts]. Tier 2: AI [complexity]. Tier 3: AI orchestrates [scale].",
  "anti_patterns": [
    "Don't use AI for deterministic simple commands",
    "Don't inflate durations (1min tasks ≠ 45min lessons)",
    "Don't create 8+ verbose 'Try with AI' prompts (use 3-4 focused)",
    "Use decision framework: deterministic → direct, complex → AI companion, scale → AI orchestration"
  ],
  "constitutional_principles": [
    "Principle 13 (Graduated Teaching): Direct → AI Companion → AI Orchestration",
    "Principle 18 (Three Roles): AI as Teacher/Student/Co-Worker",
    "Core Philosophy #2 (Co-Learning): Bidirectional learning",
    "Core Philosophy #1 (AI Spectrum): Assisted → Driven → Native (teach when each applies)"
  ],
  "validation_criteria": {
    "duration_realistic": true,
    "direct_commands_clear": true,
    "ai_usage_strategic": true,
    "decision_framework_applied": true,
    "line_count_reasonable": true
  },
  "verified_intelligence": "intelligence/[feature-slug]-verified-docs.md"  // if Phase 0.5 ran
}

Strategy: Task-type dependent (lessons/code/tests/docs)
Invoke: Appropriate subagent (lesson-writer, general-purpose, etc.)
Create: Implementation artifacts
```

**CRITICAL**: DO NOT ask user "Should I run /sp.implement?" - EXECUTE IT IMMEDIATELY.

**STEP 3: Conceptual Validation Review** (Automatic After Implementation)

After implementation completes, YOU MUST perform conceptual review:

For book chapters:
- ✅ Duration realistic? (not inflated for simple operations)
- ✅ Direct commands documented clearly? (not hidden behind AI)
- ✅ "Try with AI" section uses proper format? (3-4 focused prompts)
- ✅ AI usage strategic? (troubleshooting, understanding, not trivial commands)
- ✅ Decision framework applied? (deterministic → direct, complex → AI)
- ✅ Line count reasonable? (not verbose explanations of simple ops)

For code features:
- ✅ Tests pass?
- ✅ Code quality meets standards?
- ✅ Documentation clear?

IF conceptual issues found:
  Apply fixes for critical issues
  Re-run conceptual validation
  Repeat until PASS

**STEP 4: Technical Review** (Invoke technical-reviewer if applicable)

For book chapters or complex features:

YOU MUST invoke technical-reviewer:

```
Task(
  subagent_type="technical-reviewer",
  prompt="Validate [feature-slug] against constitution and quality standards"
)
```

Wait for validation report (PASS / CONDITIONAL PASS / FAIL)

IF CONDITIONAL PASS or FAIL:
  Apply fixes for critical issues
  Re-run technical-reviewer
  Repeat until PASS

**STEP 5: Sandbox Validation** (Hands-On Testing - CRITICAL)

**Philosophy**: "If you have not run anything in sandbox, chances are it won't work"

For chapters with hands-on commands:
  Extract ALL commands students will run
  Test EVERY command in actual environment
  Verify command syntax (CLI vs session commands)
  Verify output matches lesson claims
  Test "Try With AI" prompts for achievability
  Document what works vs. what's documented

For code features:
  Run full test suite in sandbox
  Execute code in target environment
  Verify deployment steps work end-to-end
  Test edge cases and error paths

Create: SANDBOX-AUDIT-REPORT.md with:
  - Commands tested (with actual output)
  - Errors found (with line numbers)
  - Fixes applied (with evidence)
  - Re-test results (verification)

IF SANDBOX FAIL:
  Apply fixes for ALL command syntax errors
  Re-run sandbox tests
  Update SANDBOX-AUDIT-REPORT.md
  Repeat until SANDBOX PASS

**STEP 6: Report to User and BLOCK for Approval**

Output:
```
✅ PHASE 4 COMPLETE: Implementation + Validation

📚 Implementation: [file paths]
📋 Validation Report: [PASS/CONDITIONAL PASS/FAIL]
📋 Sandbox Audit: SANDBOX-AUDIT-REPORT.md

Validation results:
- Conceptual review: [PASS/issues fixed]
- Technical review: [PASS/CONDITIONAL PASS/FAIL]
- Sandbox testing: [PASS/FAIL]
- AI usage strategy: [verified/issues]
- Decision framework applied: [yes/no]

🚫 BLOCKED: You MUST review implementation + validation reports before proceeding.

Respond with:
- ✅ "Implementation approved" → Continue to PHASE 5
- 🔧 "Fix issues first" → Address validation findings
- 📝 Feedback → Iterative improvements
- ❓ Questions → Clarification dialog
```

**DO NOT PROCEED** until user explicitly confirms "✅ Implementation approved" or equivalent.

---

## PHASE 5: FINALIZATION + OPTIONAL GIT WORKFLOW

**STEP 1: Update Project Tracking** (Execute Automatically)

For chapters:
  Update chapter-index.md status (📋 Planned → ✅ Implemented & Validated)

For features:
  Update feature tracking (if exists)

**STEP 2: Offer Git Workflow** (User Decision)

Ask user:
```
📋 Feature complete and validated.

Git workflow options:
A) Auto-commit and PR: `/sp.git.commit_pr`
B) Manual commit: Provide summary for manual commit
C) Skip for now: Continue without git operations

Which option?
```

Wait for user choice:
- IF A: Invoke `/sp.git.commit_pr` automatically
- IF B: Provide commit summary, wait for user to commit manually
- IF C: Skip git operations

**STEP 3: Create PHR (Prompt History Record - Execute Automatically)**

YOU MUST create PHR:

```
/sp.phr

Document: User goal, intelligence derived, decisions made, AI strategy applied
Include: Anti-pattern checks performed, validation results
Save: history/prompts/[feature-slug]/
```

**STEP 4: Final Report**

Output:
```
🎉 LOOPFLOW COMPLETE: [Feature Name]

✅ All phases executed successfully:
- Phase 0: Intelligent discovery ([N] derivations, [M] questions)
- Phase 1: Specification + clarification
- Phase 2: Planning + ADR ([N] ADRs created)
- Phase 3: Tasks + analysis ([issues] resolved)
- Phase 4: Implementation + validation (PASS)
- Phase 5: Finalization + tracking updated

📋 Artifacts created:
- specs/[feature-slug]/{spec,plan,tasks}.md
- [implementation files]
- VALIDATION_REPORT.md
- SANDBOX-AUDIT-REPORT.md
- history/prompts/[feature-slug]/[timestamp].phr.md

📊 Quality metrics:
- AI usage strategy: Applied decision framework
- Constitutional compliance: ✅
- Validation: All gates passed
- Sandbox testing: All commands verified

🌿 Git status:
- Branch: [branch-name]
- Commit: [if completed] / Manual commit needed [if option B]

Ready for: [next steps]
```

---

## CRITICAL SUCCESS FACTORS

1. **Vertical Intelligence**: Constitution + project context read FIRST, questions SECOND
2. **Goal-Oriented**: User states GOAL, AI derives STRATEGY using constitutional principles
3. **Universal Decision Framework**: Deterministic/simple → direct, complex → AI companion, scale → AI orchestration (works for ANY tool)
4. **Graduated Teaching**: Apply Principle 13 across all content
5. **Quality Gates**: Every phase has automatic gates that prevent bad patterns
6. **Context Preservation**: Full intelligence + AI strategy passed through all phases with explicit context objects
7. **Constitutional Reasoning**: Principles guide decisions, not tool-specific templates
8. **Imperative Execution**: Commands invoked automatically (not "should I run?"), approval gates between phases only
9. **Branch Creation**: Literal bash commands executed after spec creation
10. **Verification-First**: Tool/API content uses verified intelligence, not assumptions
11. **Shipping-Ready**: Built-in quality, not post-hoc validation

---

## EXECUTION PATTERNS (How Commands Are Actually Invoked)

### ✅ CORRECT: Imperative Automatic Invocation

```
PHASE 1:

YOU MUST invoke /sp.specify NOW:

/sp.specify [feature-slug]
[full context passed]

[Wait for completion]

YOU MUST invoke /sp.clarify NOW:

/sp.clarify [feature-slug]

[Wait for completion]

YOU MUST create feature branch NOW:

[Execute literal bash commands from lines XXX-YYY]

BLOCK: User must approve spec before proceeding to Phase 2.
```

### ❌ INCORRECT: Passive Suggestions

```
→ Invoke: /sp.specify [context]
  ├─ Pass: intelligence object
  └─ Report: "Spec created"

WAIT: User reviews spec.md
```

**Problem**: "Invoke" reads as documentation, not as imperative command. No enforcement mechanism.

### ✅ CORRECT: Approval Gates Between Phases

```
BLOCK: You MUST NOT proceed until user confirms "✅ Spec approved"

IF user confirms approval:
  Proceed to PHASE 2
ELSE IF user provides feedback:
  Update spec iteratively
  Re-run /sp.clarify if needed
  BLOCK again for re-approval
```

### ❌ INCORRECT: Passive Waiting

```
WAIT: User reviews spec.md
→ User confirms: "✅ Spec approved"
```

**Problem**: "WAIT" is passive. No blocking enforcement. No clear decision branches.

---

## REFERENCES

- **Constitution**: `.specify/memory/constitution.md` (source of truth for principles)
- **Project Structure**:
  - Chapter Index: `specs/book/chapter-index.md`
  - Specs Directory: `specs/`
  - Skills Library: `.claude/skills/`
- **LoopFlow Commands** (invoked automatically by this orchestrator):
  - `/sp.specify` - Create specifications with evals-first
  - `/sp.clarify` - Clarify underspecified areas (up to 5 questions)
  - `/sp.plan` - Create implementation plans with teaching tiers
  - `/sp.adr` - Document architectural decisions (user consent required)
  - `/sp.tasks` - Generate task checklists with acceptance criteria
  - `/sp.analyze` - Cross-artifact consistency validation
  - `/sp.implement` - Execute implementation with full intelligence context
  - `/sp.git.commit_pr` - Git workflow automation (optional)
  - `/sp.phr` - Create Prompt History Record (automatic)

---

## ONE COMMAND. UNIVERSAL INTELLIGENCE. COMPLETE WORKFLOW.

Run `/sp.loopflow [goal]` and the system EXECUTES (AIDD Spec-First Methodology):

**PHASE 0: Deep Search** (Pre-Spec Intelligence - ALWAYS)  
→ Constitution + context + decision framework derived  
→ 0-5 clarifying questions (only genuine ambiguities)  
→ Intelligence object ready for spec creation

**PHASE 1: Specification + Clarification** (AIDD Core - ALWAYS)  
→ Evals-first + AI strategy + spec.md creation  
→ /sp.specify + /sp.clarify invoked automatically  
→ Feature branch created  
→ APPROVAL GATE (user must approve spec)

**PHASE 0.5: Deep Research** (Post-Spec Verification - CONDITIONAL)  
→ Triggered IF spec has tool-specific claims, high assumption rate, or user requests  
→ Constitutional reasoning drives verification strategy  
→ Verified intelligence cache created  
→ Spec updated with verified facts  
→ APPROVAL GATE (if significant spec changes)

**PHASE 2: Planning + ADR** (ALWAYS)  
→ Teaching pattern + direct vs AI mapping  
→ /sp.plan invoked automatically  
→ ADRs suggested (user consent required)  
→ APPROVAL GATE (user must approve plan)

**PHASE 3: Tasks + Analysis** (ALWAYS)  
→ Anti-pattern checks + consistency validation  
→ /sp.tasks + /sp.analyze invoked automatically  
→ APPROVAL GATE (user must approve tasks)

**PHASE 4: Implementation + Validation** (ALWAYS)  
→ Context-aware execution with full intelligence  
→ /sp.implement invoked automatically  
→ Technical review + sandbox testing  
→ APPROVAL GATE (user must approve implementation)

**PHASE 5: Finalization** (ALWAYS)  
→ Tracking updates + optional git workflow  
→ PHR created automatically  
→ Ready for deployment/publication

**Result**: Shipping-ready output with:
- ✅ Strategic AI use (decision framework applied, not over-engineered)
- ✅ Constitutional compliance (principles-driven, not template-driven)
- ✅ Realistic expectations (durations, complexity, scope)
- ✅ Quality built-in (validation at every gate)
- ✅ Decision trail documented (ADRs + PHR)
- ✅ Verification-first (tool/API content uses verified intelligence)
- ✅ Universal applicability (works for ANY tool/framework/language using constitutional reasoning)
