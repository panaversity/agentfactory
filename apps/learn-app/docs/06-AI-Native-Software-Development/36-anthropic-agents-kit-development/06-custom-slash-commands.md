---
sidebar_position: 6
title: "Custom Slash Commands"
chapter: 36
lesson: 6
duration_minutes: 25
description: "Create custom slash commands that encapsulate agent workflows. Define what tools your agents need, then invoke commands from code."
keywords: ["Claude Agent SDK", "Custom Slash Commands", ".claude/commands/", "YAML Frontmatter", "Command Workflows", "Agent Prompting", "Command Arguments", "Skill Integration"]

# HIDDEN SKILLS METADATA
skills:
  - name: "Slash Command Design and Filesystem Organization"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Technical Communication"
    measurable_at_this_level: "Student can create a `.claude/commands/` markdown file with proper YAML frontmatter, define allowed tools, and invoke it from SDK code"

  - name: "Workflow Encapsulation (Commands vs Skills)"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can distinguish when to create a slash command (specific workflow) vs a skill (general expertise), and explain the difference with examples"

  - name: "Command Reusability and Library Building"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Create"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can build a library of commands for their domain, configure each with appropriate tool restrictions, and compose them into agent workflows"

learning_objectives:
  - objective: "Create a custom slash command via markdown file in `.claude/commands/`"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates `/code-review` command and uses it from SDK code"

  - objective: "Configure command-level tool access through YAML frontmatter"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student writes YAML with correct `allowed-tools` syntax and explains security implications"

  - objective: "Invoke commands with arguments from agent code and understand parameter passing"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student passes command arguments and validates correct parsing"

cognitive_load:
  new_concepts: 6
  assessment: "File structure (.claude/commands/), YAML frontmatter (allowed-tools, description), command invocation syntax, parameter passing, command vs skill distinction, library patterns. Concepts chunk naturally around file organization and configuration."

differentiation:
  extension_for_advanced: "Build multi-command workflows where commands chain together (e.g., `/analyze` → `/review` → `/test`). Implement permission callbacks that grant tools dynamically based on command context. Create command aliases that invoke with different tool configurations."
  remedial_for_struggling: "Start with simple single-file command (e.g., `/hello` that just echoes). Focus on correct YAML structure and invocation syntax. Defer multi-tool configurations until comfortable with basics. Use checklist: 'Did I create the file?' → 'Did I add YAML?' → 'Does it run?'"

---

# Custom Slash Commands

Type `/review` and your agent runs a complete code review. Type `/deploy production` and your agent knows exactly which tools it needs to verify production readiness. This is the power of custom slash commands.

Slash commands are **workflow shortcuts** that encapsulate intent, required tools, and instructions in one reusable file. Unlike skills (which teach general expertise), commands are **task-specific**. A slash command says "when you see `/deploy`, do this specific thing with these specific tools."

The key insight: Commands bring **structure and constraints** to agent workflows. An agent with every tool enabled is powerful but undisciplined. An agent configured for `/review` with just `Read` and `Grep` is focused and safe.

## Command File Structure

Commands live in `.claude/commands/` as markdown files. They have two parts: YAML frontmatter defining tool access, and a markdown description of what the command does.

### Creating Your First Command

Create `.claude/commands/code-review.md`:

```markdown
---
allowed-tools: Read, Grep
description: "Code review with security and quality focus"
---

Review the code for security vulnerabilities and quality issues.

Focus on these areas:
1. Security: Input validation, authentication checks, SQL injection risks
2. Quality: Error handling, performance bottlenecks, code duplication
3. Best practices: Following language conventions, test coverage

When reviewing, identify:
- Critical issues (must fix before merge)
- Major issues (should fix, impacts users)
- Minor issues (nice to have, code style)
```

This command:
- **File location**: `.claude/commands/code-review.md`
- **File name becomes command**: Type `/code-review` to use it
- **Allowed tools**: `Read, Grep` (can read code, search patterns, but NOT modify)
- **Description**: What appears when user types `/help code-review`
- **Content**: Detailed instructions the agent follows

### Command Invocation from SDK Code

Use the command from your agent:

```python
import asyncio
from claude_agent_sdk import query, ClaudeAgentOptions

async def main():
    options = ClaudeAgentOptions(
        setting_sources=["project"],  # Load commands from .claude/commands/
        allowed_tools=["Read", "Grep"]
    )

    async for message in query(
        prompt="/code-review src/auth.py",
        options=options
    ):
        print(message)

asyncio.run(main())
```

**What happens**:
1. Agent sees `/code-review` (matches `.claude/commands/code-review.md`)
2. Agent loads the command definition (instructions + allowed tools)
3. Agent restricts itself to `Read, Grep` only
4. Agent reviews `src/auth.py` using the security-focused instructions

---

## YAML Frontmatter: Configuring Tool Access

The YAML section at the top of each command file is where you specify what tools the agent can use while executing that command.

### Syntax and Options

```yaml
---
allowed-tools: Read, Grep, Bash
description: "Brief description of what this command does"
---
```

**Fields**:

| Field | Type | Required? | Example |
|-------|------|-----------|---------|
| `allowed-tools` | Comma-separated string | Yes | `Read, Grep, Bash` |
| `description` | String | Yes | `"Code review focusing on security"` |

**Important**: Only list tools your command *actually needs*. This is the "deny by default" principle applied to command configuration.

### Examples: Restricting Tools by Task

#### Example 1: Read-Only Code Analyzer

```yaml
---
allowed-tools: Read, Glob, Grep
description: "Analyze code for patterns without modifying files"
---

Scan the codebase for specific patterns:
1. Find all TODO comments
2. Identify dead code (imports never used)
3. List functions with high complexity
Report findings without making changes.
```

**Use case**: Code audit, analysis, reporting

**Why these tools**:
- `Read`: View file contents
- `Glob`: Find files matching patterns
- `Grep`: Search for specific patterns
- NOT `Write/Edit`: Cannot modify files (safe for read-only analysis)
- NOT `Bash`: Cannot run arbitrary commands

---

#### Example 2: Refactoring Agent

```yaml
---
allowed-tools: Read, Glob, Grep, Edit, Bash
description: "Refactor code with automated testing"
---

Refactor the provided code:
1. Identify refactoring opportunities
2. Make changes using precise edits
3. Run tests to validate no functionality changed
4. Report what changed and why

Safety: Do NOT edit files outside the scope provided.
```

**Use case**: Automated refactoring with safety validation

**Why these tools**:
- `Read/Glob/Grep`: Understand the code
- `Edit`: Make refactoring changes
- `Bash`: Run `npm test` to validate changes
- NOT `Write`: Don't create new files (only modify existing)

---

#### Example 3: Deployment Verification

```yaml
---
allowed-tools: Read, Bash
description: "Verify deployment readiness"
---

Check if the system is ready for production deployment:
1. Verify environment variables are set
2. Run security audit
3. Check database migrations
4. Validate API endpoints respond
5. Generate deployment checklist

Do NOT make changes. Only verify and report.
```

**Use case**: Pre-deployment validation

**Why these tools**:
- `Read`: Check config files, environment setup
- `Bash`: Run verification scripts, curl API endpoints
- NOT `Edit/Write`: Cannot change configuration
- NOT `Glob/Grep`: Not needed (shell commands suffice)

---

## Using Commands with Arguments

Commands become powerful when you pass arguments. The command instruction is the **base behavior**, and arguments **customize** it.

### Passing Arguments

```python
async for message in query(
    prompt="/code-review src/auth.py --severity=critical",
    options=options
):
    print(message)
```

The agent sees the full prompt: `/code-review src/auth.py --severity=critical`

The agent extracts:
- **Command**: `code-review`
- **Arguments**: `src/auth.py` (file), `--severity=critical` (filter level)

The command instructions then guide: "Review this file, but focus only on critical-severity issues."

### Example: Deploy Command with Arguments

Create `.claude/commands/deploy.md`:

```markdown
---
allowed-tools: Read, Bash
description: "Deploy to target environment"
---

Deploy the application to the specified environment.

Arguments:
- Environment: staging | production
- Validate: true | false (run checks before deploy)

Instructions:
1. If validate=true, run security audit and tests
2. Verify environment variables for [environment]
3. Run deploy scripts
4. Report deployment status

Safety: Always verify before production deployment.
```

**Usage from code**:

```python
# Deploy to staging (with validation)
async for msg in query(
    prompt="/deploy staging --validate=true",
    options=options
):
    print(msg)

# Deploy to production (requires explicit confirmation)
async for msg in query(
    prompt="/deploy production --validate=true",
    options=options
):
    print(msg)
```

---

## Commands vs Skills: When to Create Which

You now know two ways to extend agents: **Skills** (from Lesson 5) and **Commands** (this lesson).

When should you use each?

### Skills: General Expertise

**Purpose**: Teach an agent a broad domain expertise that applies across multiple tasks.

**Example**: A "Python Security Auditing" skill teaches threat modeling, vulnerability patterns, security best practices. It applies to *any* code review, test generation, or documentation task.

**Characteristics**:
- Reusable across many commands
- Teaches reasoning/decision-making
- Large scope (multiple concepts)
- Stays loaded for whole session

**When to create**: When a pattern appears across 3+ different workflows.

---

### Commands: Specific Workflows

**Purpose**: Encapsulate a concrete workflow that performs one task with specific tools.

**Example**: A `/code-review` command runs a specific sequence (analyze security → find issues → generate report) for the exact tools it needs.

**Characteristics**:
- Task-specific (does one thing)
- Configures tool access for that task
- Small scope (specific workflow)
- Invoked on demand

**When to create**: When you have a specific workflow you invoke repeatedly with `/command` syntax.

---

### Decision Framework

| Question | If Yes → Create Skill | If Yes → Create Command |
|----------|----------------------|------------------------|
| Does this apply across multiple different workflows? | ✓ | ✗ |
| Is this general domain expertise? | ✓ | ✗ |
| Is this a specific task you run with `/command` syntax? | ✗ | ✓ |
| Do you want to configure tool restrictions for it? | ✗ | ✓ |
| Will this be referenced by multiple commands? | ✓ | ✗ |

**Example**: Building a code review system

```
Skill: "Python Security Auditing"
├─ Teaches threat modeling, vulnerability patterns
├─ Used by: /code-review, /test-generation, /documentation
└─ Stays loaded during session

Command: /code-review
├─ Invokes security auditing skill
├─ Restricts to Read + Grep only
└─ Runs on demand for specific files
```

---

## Building a Command Library for Your Organization

Once you understand command patterns, you can build a reusable library tailored to your domain.

### Example: Development Team Command Library

Create these commands in `.claude/commands/`:

**`.claude/commands/review-pr.md`** (Code review for pull requests)
```markdown
---
allowed-tools: Read, Glob, Grep
description: "Review pull request for quality and security"
---

Review the files in this pull request:
1. Security: Check for injection vulnerabilities, auth issues
2. Performance: Identify N+1 queries, inefficient algorithms
3. Style: Verify code follows team conventions
4. Tests: Ensure test coverage for new code

Focus on: Critical issues only
```

**`.claude/commands/generate-test.md`** (Test generation)
```markdown
---
allowed-tools: Read, Write, Bash
description: "Generate tests for provided code"
---

Generate comprehensive unit tests for the provided code:
1. Analyze the function/class
2. Identify test cases (happy path, edge cases, errors)
3. Write tests using [framework]
4. Run tests to verify they pass
```

**`.claude/commands/security-audit.md`** (Security scan)
```markdown
---
allowed-tools: Read, Glob, Grep, Bash
description: "Security audit of codebase"
---

Perform security audit:
1. Find hardcoded credentials
2. Check for SQL injection risks
3. Validate authentication flows
4. Review dependency vulnerabilities
5. Generate security report
```

**`.claude/commands/refactor.md`** (Code refactoring)
```markdown
---
allowed-tools: Read, Edit, Bash, Glob, Grep
description: "Refactor code with automated testing"
---

Refactor code while maintaining functionality:
1. Identify refactoring opportunities
2. Make precise edits
3. Run test suite to validate
4. Report changes
```

### Using the Library

```python
# Code review
async for msg in query(prompt="/review-pr src/", options=opts):
    print(msg)

# Security audit
async for msg in query(prompt="/security-audit", options=opts):
    print(msg)

# Generate tests
async for msg in query(prompt="/generate-test src/auth.py", options=opts):
    print(msg)

# Refactor
async for msg in query(prompt="/refactor src/utils.py", options=opts):
    print(msg)
```

Each command carries its own tool restrictions, instructions, and context. Your team has a curated set of proven workflows that any developer can invoke with a single command.

---

## Try With AI

Create three practical custom commands for your own domain.

### Prompt 1: Design Your First Command

```
I want to create a custom slash command for [your domain/project].
The command should [specific task: e.g., "analyze database migrations"].

For this command:
1. Name the command (what would the /command syntax be?)
2. List the tools this command should have access to (and justify each)
3. List the tools it should NOT have (and explain why)
4. Write the YAML frontmatter with allowed-tools and description
5. Write the command instructions (what should the agent do?)

Show me the complete .claude/commands/ file I would create.
```

**What you're learning**: Translating a workflow requirement into a concrete command definition. This is how you architect agent workflows—defining what tools each task truly needs.

### Prompt 2: Build a Multi-Command Workflow

```
I'm building a system that needs these three workflows:
1. [Workflow A: e.g., "Analyze new code for security"]
2. [Workflow B: e.g., "Run performance benchmarks"]
3. [Workflow C: e.g., "Generate documentation"]

Create three slash commands (one for each workflow):
- What tools does each command need?
- Where would these commands be restricted for safety?
- Could any commands share the same tools?
- How would a developer use all three commands together in a workflow?

Show me the three .claude/commands/*.md files with complete YAML and instructions.
```

**What you're learning**: Designing command libraries that work together. This is systems thinking for agent workflows—understanding how specialized commands compose into larger workflows.

### Prompt 3: Security Review of Command Configuration

```
Our team created these commands (show your prompts 1 and 2 configurations).

Review them for security:
1. Are any commands over-privileged (have tools they don't need)?
2. What dangerous patterns could each command accidentally enable?
3. For the most powerful command, what Bash patterns should we block?
4. Could a malicious developer use these commands to exfiltrate data or damage systems?
5. What hooks or permission callbacks would make these commands safer?

Suggest improvements to the tool configurations.
```

**What you're learning**: Security-first design. Commands appear simple, but tool access is powerful. Learning to audit configurations prevents production disasters.

**Safety Note**: Just like tools, commands follow "deny by default"—only grant specific tools your command truly needs. An agent with full tool access is like a user with root privileges; most workflows don't need it.

