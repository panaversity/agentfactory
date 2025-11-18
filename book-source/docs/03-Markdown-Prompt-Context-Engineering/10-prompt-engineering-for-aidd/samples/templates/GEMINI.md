# Project Context for Codebase Analysis (Gemini CLI)

**Analysis Date**: [YYYY-MM-DD]
**Analyzed By**: [Your Name/Role]
**Analysis Purpose**: [Vendor Evaluation / Competitive Analysis / Feasibility Assessment / Onboarding]

---

## Project Overview (Layer 1: Foundation Context)

### Business Context
**What problem does this codebase solve?**
[Describe the business problem or domain. Example: "Payment processing API for e-commerce platforms"]

**Target Users**: [Who uses this software?]

**Scale**: [LOC, team size, maturity level]

### Technology Stack
**Languages**: [Python, JavaScript, Go, etc.]
**Frameworks**: [Django, React, FastAPI, etc.]
**Databases**: [PostgreSQL, MongoDB, etc.]
**Infrastructure**: [Cloud provider or self-hosted]

---

## Architecture Context (Layer 2: Code Structure)

### Entry Points
**Main entry file(s)**: [Path to main file]

### Module Organization
```
Project structure:
/module-1/  # [Purpose]
/module-2/  # [Purpose]
/module-3/  # [Purpose]
```

### Critical Modules
1. **[Module]**: [Responsibility]
2. **[Module]**: [Responsibility]
3. **[Module]**: [Responsibility]

### External Integrations
- [Service 1]: [Purpose]
- [Service 2]: [Purpose]

---

## Analysis Constraints (Layer 3: Goals & Requirements)

### Analysis Goals
**Key questions to answer**:
1. [Example: "What are security risks?"]
2. [Example: "How scalable is the architecture?"]
3. [Example: "What's the integration effort?"]

### Success Criteria
- [ ] [Identify architectural style]
- [ ] [Document security risks]
- [ ] [Assess technical debt]

### Focus Areas
**Must analyze**: [Critical paths]
**Should analyze**: [Secondary priorities]
**Can skip**: [Out of scope]

---

## Analyst Context (Layer 4: Your Background)

**Your Role**: [Product Manager / Engineer / Founder]

**Technical Background**: [Your experience level]

**Communication Preference**: [Business terms vs technical detail]

**Output Format**: [Diagrams? Code snippets? Risk ratings?]

---

## Gemini CLI Commands for Analysis

### @filename Command (File Context)
**Purpose**: Include specific files in conversation context.

**Syntax**: `@path/to/file.py`

**Example Usage**:
```
@src/main.py @src/config.py
Analyze these entry points. What's the application architecture?
```

**When to use**:
- Reading specific files (equivalent to Claude's Read tool)
- Providing code context for questions
- Analyzing configuration files

**Limitations**:
- File must exist in accessible directory
- Large files may hit context limits

---

### !command Execution (Shell Commands)
**Purpose**: Run shell commands and include output in context.

**Syntax**: `!shell-command`

**Example Usage**:
```
!ls -R src/
Show me the directory structure, then explain module organization.
```

```
!grep -r "password" src/
Search for hardcoded credentials. Are there security risks?
```

**When to use**:
- Directory exploration (equivalent to Claude's Glob)
- Pattern searching (equivalent to Claude's Grep)
- Dependency analysis (`!cat requirements.txt`)

**Limitations**:
- Requires shell access
- Output length limits apply

---

### Custom Commands (TOML Workflow Automation)

**Purpose**: Create reusable analysis workflows.

**Configuration File**: `.gemini/commands.toml`

#### Example 1: Security Audit Command
```toml
[security-audit]
description = "Run comprehensive security analysis"
command = """
!grep -rn "password\\|secret\\|api_key" src/ &&
!grep -rn "SELECT.*\\+" src/ &&
echo "===SECURITY FINDINGS===" &&
cat security-patterns.txt
"""
help = "Searches for hardcoded credentials and SQL injection patterns"
```

**Usage**: `!security-audit` (executes full workflow)

#### Example 2: Architecture Overview Command
```toml
[arch-overview]
description = "Generate architecture summary"
command = """
!find src/ -name "*.py" -type f | head -20 &&
!cat README.md &&
!cat src/main.py
"""
help = "Shows project structure, README, and entry point for architecture analysis"
```

**Usage**: `!arch-overview`

#### Example 3: Dependency Analysis Command
```toml
[dep-check]
description = "List all dependencies with versions"
command = """
!cat requirements.txt &&
!pip list --outdated
"""
help = "Shows Python dependencies and identifies outdated packages"
```

**Usage**: `!dep-check`

---

## Analysis Workflow (Gemini CLI)

### Step 1: Project Structure Discovery
**Objective**: Understand codebase layout.

**Gemini CLI approach**:
```
!ls -R src/
!cat README.md

Based on this structure, what's the architecture style? What are the main components?
```

### Step 2: Technology Stack Validation
**Objective**: Document all dependencies.

**Gemini CLI approach**:
```
@package.json
OR
@requirements.txt
OR
@go.mod

List all dependencies. Flag any that are deprecated or have security vulnerabilities.
```

### Step 3: Security Assessment
**Objective**: Find security anti-patterns.

**Gemini CLI approach**:
```
!grep -rn "password\|secret\|api_key" src/
!grep -rn "SELECT.*+" src/  # SQL injection patterns

Analyze these search results. What are the security risks? Rate severity (Critical/High/Medium/Low).
```

### Step 4: Code Quality Evaluation
**Objective**: Assess technical debt.

**Gemini CLI approach**:
```
@src/critical-module.py

Assess code quality:
1. Function complexity (>50 lines?)
2. Documentation (docstrings present?)
3. Error handling (try/catch blocks?)
4. Test coverage

Rate technical debt: Low/Medium/High with rationale.
```

### Step 5: Integration Feasibility
**Objective**: Estimate integration effort.

**Gemini CLI approach**:
```
@src/auth-module.py

We need to integrate this with our OAuth2 system. What changes are needed?
Estimate effort: Small (days), Medium (weeks), Large (months).
```

---

## Validation Checklist

Before finalizing analysis:

- [ ] **Cross-reference AI claims with actual code** (use `@filename` to re-check)
- [ ] **Verify search results** (run `!grep` commands manually if needed)
- [ ] **Confirm dependency versions** (check package manifests)
- [ ] **Document evidence** (file paths, line numbers for all findings)
- [ ] **Test recommendations** (ensure actionable, not vague)

---

## Custom Command Library for Analysis

Create `.gemini/commands.toml` with these workflows:

```toml
# Quick codebase overview
[codebase-summary]
description = "Generate project summary"
command = """
echo "=== PROJECT STRUCTURE ===" &&
!ls -R src/ | head -50 &&
echo "\\n=== DEPENDENCIES ===" &&
!cat requirements.txt &&
echo "\\n=== ENTRY POINT ===" &&
!cat src/main.py | head -30
"""

# Find all TODOs and FIXMEs
[tech-debt-markers]
description = "List technical debt markers"
command = """
!grep -rn "TODO\\|FIXME\\|HACK\\|XXX" src/
"""

# Count lines of code
[code-metrics]
description = "Calculate code metrics"
command = """
!find src/ -name "*.py" -exec wc -l {} + | sort -rn | head -20
"""

# List test files
[test-coverage]
description = "Show test file organization"
command = """
!find . -name "*test*.py" -o -name "test_*.py"
"""

# Find database queries
[db-queries]
description = "Extract all database queries"
command = """
!grep -rn "SELECT\\|INSERT\\|UPDATE\\|DELETE" src/
"""
```

**Usage**:
```
!codebase-summary
!tech-debt-markers
!code-metrics
```

---

## Gemini CLI vs Claude Code: Tool Equivalence

| Task | Claude Code | Gemini CLI |
|------|-------------|-----------|
| **Read file** | Read tool | `@filename` |
| **Search patterns** | Grep tool | `!grep -rn "pattern" src/` |
| **List files** | Glob tool | `!ls -R` or `!find` |
| **Directory structure** | Glob tool | `!tree` or `!ls -R` |
| **Run commands** | Bash tool | `!command` |
| **Custom workflows** | N/A (manual) | TOML custom commands |

**Key Difference**: Gemini CLI uses shell commands (`!grep`, `!find`) while Claude Code uses specialized tools (Grep, Glob). Both achieve same outcomes.

---

## Best Practices (Gemini CLI)

### Combine @filename with Questions
```
# Good
@src/auth.py
Is this authentication implementation secure? Check for common vulnerabilities.

# Bad (no context)
Is this code secure?
```

### Use !grep Before Reading Files
```
# Efficient
!grep -rn "class " src/
Show me all class definitions, then I'll @filename the important ones.

# Inefficient
@src/file1.py @src/file2.py @src/file3.py ... (100 files)
```

### Create Custom Commands for Recurring Patterns
```
# Reusable
[security-scan]
command = "!grep -rn 'password\\|secret' src/"

# One-off (don't repeat manually)
Every analysis: "!grep -rn 'password' src/" (copy-paste each time)
```

### Validate AI Responses
```
# After AI claims "Found SQL injection in db.py line 42"
@src/db.py
Show me line 42. Confirm this is actually a SQL injection risk.
```

---

## Sample Analysis Session (Gemini CLI)

**Initial Prompt**:
```
I'm evaluating a Django e-commerce API for acquisition.

PROJECT CONTEXT:
- Business: Order management system
- Scale: 50K LOC, production, 5-person team
- Stack: Python 3.11, Django, PostgreSQL, AWS

ANALYSIS GOALS:
1. Map architecture
2. Identify security risks
3. Assess integration with our OAuth2 system

MY BACKGROUND:
- Product Manager
- Limited Python experience
- Need explanations in business terms

Let's start with project overview:
!ls -R src/ | head -50
@README.md
@src/settings.py

Based on this, what's the architecture style? What are the main modules?
```

**Iterative Refinement**:
```
# After AI provides overview
!grep -rn "password\|secret" src/

Analyze security. What risks exist? Rate severity.

# After security findings
@src/auth/views.py

How hard to integrate this with our OAuth2? What changes needed?
```

---

## Anti-Patterns to Avoid

❌ **No file context**: "Analyze the code" (which code?)
✅ **Explicit files**: `@src/main.py` "Analyze this entry point"

❌ **Overloading context**: `@file1 @file2 @file3 ... @file100`
✅ **Targeted context**: Use `!grep` to narrow down, then `@filename` key files

❌ **Blind trust**: Accept all AI findings without verification
✅ **Cross-reference**: `@filename` to verify AI's claims about specific files

❌ **Manual repetition**: Typing same `!grep` commands every analysis
✅ **Automation**: Create custom TOML commands for recurring workflows

---

**Template Version**: 1.0.0
**Last Updated**: 2025-01-18
**Source**: Chapter 10, Lesson 5 & 6
**Constitution**: v6.0.0 Compliance

**Usage**: Copy to project root as `.gemini/GEMINI.md`. Fill in bracketed placeholders. Create `.gemini/commands.toml` for custom workflows. Use as reference when analyzing codebases with Gemini CLI.
