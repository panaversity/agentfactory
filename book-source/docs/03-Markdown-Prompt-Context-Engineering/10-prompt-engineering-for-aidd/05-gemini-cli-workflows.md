# Lesson 5: Gemini CLI Workflows for Markdown Automation

---

## What Should This Lesson Enable You to Do?

By the end of this lesson, you should be able to:

- **Use Gemini CLI file reference syntax** (@filename) to provide precise context
- **Execute shell commands through Gemini** (!command) for integrated workflows
- **Create custom TOML commands** that encode recurring prompt patterns
- **Apply the Three Roles framework** while using Gemini CLI (AI teaches you patterns, you teach AI constraints, together you converge on workflows)
- **Create GEMINI.md files** that encode project-specific context and custom commands

**Success criteria**: You can create a custom TOML command for a recurring markdown generation workflow, test it across multiple contexts, and explain the Persona + Questions + Principles pattern you encoded.

---

## The Workflow Automation Challenge

### The Scenario

You are a **Technical Documentation Lead** for a team building microservices. Every new service needs a standardized README with these sections:
- Architecture Overview (diagram + design rationale)
- API Endpoints (RESTful patterns + request/response examples)
- Configuration (environment variables + secrets management)
- Deployment (Docker + Kubernetes manifests)
- Testing Strategy (unit + integration + e2e)

Currently, engineers write these READMEs manually, and quality is inconsistent:
- Some skip sections (missing testing strategy)
- Some use vague language (no concrete examples)
- Format varies (some use tables, some use bullets)
- Updates lag behind code changes

**You need**: A systematic workflow that generates consistent, high-quality READMEs efficiently.

**Without automation**, each README takes 2-3 hours and still has gaps.

**With Gemini CLI custom commands**, you will:
- Encode best-practice prompt patterns as reusable commands
- Generate consistent READMEs in 20 minutes
- Iterate rapidly with AI to refine outputs
- Maintain quality across 15+ services

**This lesson teaches you how to build that workflow.**

---

## Concept 1: Gemini CLI File Reference (@filename)

### Why File References Matter

When you're working with documentation or code, you often need to reference multiple files for context. Typing out file contents manually is inefficient.

**Gemini CLI's @filename syntax** lets you reference files directly in your prompts.

### The Syntax

```bash
# Reference a single file
gemini "Explain this code: @src/auth.py"

# Reference multiple files
gemini "Compare these implementations: @flask_app.py @fastapi_app.py"

# Reference files in subdirectories
gemini "Analyze these docs: @docs/architecture.md @docs/api.md"
```

### What Happens Behind the Scenes

When you use `@filename`:
1. Gemini reads the file contents
2. Injects the contents into your prompt automatically
3. Provides context to the AI without manual copy-paste

**Example**:
```bash
gemini "What's missing from this README? @README.md"
```

Is equivalent to:
```bash
gemini "What's missing from this README?

[File contents of README.md pasted here]"
```

But **much faster** and less error-prone.

---

### Strategic Use of File References

**Pattern 1: Template + Content Generation**
```bash
# Reference a template to guide generation
gemini "Generate a README following this template: @templates/readme-template.md
        For this service: @src/main.py"
```

**Pattern 2: Multi-File Analysis**
```bash
# Compare multiple files
gemini "Which service has better error handling? @service_a/main.py @service_b/main.py"
```

**Pattern 3: Documentation + Code Validation**
```bash
# Check if docs match code
gemini "Does this documentation match the actual code? @docs/api.md @src/routes.py"
```

**Pattern 4: Iterative Refinement**
```bash
# Reference previous output for improvement
gemini "Improve this draft: @drafts/readme_v1.md
        Add testing strategy section"
```

---

## Concept 2: Shell Command Execution (!command)

### Why Inline Commands Matter

Sometimes your prompt needs information from the system (git history, file listings, command outputs). Instead of running commands separately and pasting results, Gemini CLI lets you execute commands inline.

### The Syntax

```bash
# Execute command and include output in prompt
gemini "Summarize recent changes: !(git log --oneline -10)"

# List files and analyze
gemini "Which Python files need docstrings? !(find . -name '*.py')"

# Check system info
gemini "Recommend Docker base image for: !(python --version)"
```

### What Happens Behind the Scenes

When you use `!(command)`:
1. Gemini executes the shell command
2. Captures the output
3. Injects output into your prompt
4. AI analyzes the result

**Example**:
```bash
gemini "What changed in the last commit? !(git diff HEAD~1)"
```

Is equivalent to:
```bash
# First, run command separately
git diff HEAD~1 > output.txt

# Then paste output
gemini "What changed in the last commit?

[Output of git diff pasted here]"
```

But **integrated** and **dynamic** (always current).

---

### Strategic Use of Shell Commands

**Pattern 1: Git History Analysis**
```bash
gemini "Summarize security-related changes: !(git log --grep='security' --oneline)"
```

**Pattern 2: File Discovery**
```bash
gemini "Which markdown files need updating? !(find docs/ -name '*.md' -mtime +30)"
```

**Pattern 3: Dependency Analysis**
```bash
gemini "Are our dependencies up-to-date? !(cat requirements.txt)"
```

**Pattern 4: Real-Time System Info**
```bash
gemini "Recommend memory settings based on: !(free -h)"
```

---

## Concept 3: Custom TOML Commands — Encoding Reusable Workflows

### The Problem: Repeating Complex Prompts

Imagine you generate READMEs frequently. Each time, you provide the same instructions:
```bash
gemini "Generate a README for @src/main.py following these guidelines:
        - Architecture section with design rationale
        - API endpoints with request/response examples
        - Configuration with environment variables
        - Deployment with Docker instructions
        - Testing strategy with examples
        - Use markdown tables for configuration
        - Include mermaid diagrams for architecture
        - Keep sections under 200 words
        - Provide concrete examples, not placeholders"
```

This is **50+ words every time**. Inefficient and error-prone (you might forget guidelines).

### The Solution: Custom TOML Commands

Gemini CLI lets you define custom commands in a `commands.toml` file:

```toml
[readme-gen]
description = "Generate production README following team standards"
prompt = """
Generate a comprehensive README for the provided service following these requirements:

## Structure
1. **Architecture Overview**: Design rationale + mermaid diagram
2. **API Endpoints**: RESTful patterns + request/response examples (use tables)
3. **Configuration**: Environment variables + secrets (use table format)
4. **Deployment**: Docker + Kubernetes manifests with commands
5. **Testing Strategy**: Unit + integration + e2e with examples

## Quality Standards
- Each section ≤ 200 words (concise, not comprehensive)
- Concrete examples (not placeholders like "your-api-key")
- Use markdown tables for structured data (configs, endpoints)
- Include mermaid diagrams for architecture
- Cross-reference related services

## Context
Service file: {file}
Related services: {related_services}

Generate README now.
"""
args = ["file", "related_services"]
```

### Using the Custom Command

After defining the command, you use it like this:

```bash
gemini readme-gen file=@src/auth_service.py related_services="user-service, payment-service"
```

This **expands** to the full 50+ word prompt automatically. You never retype guidelines.

---

### Why This Matters (Reusable Intelligence)

Custom TOML commands are **encoding prompt expertise** into reusable workflows:

1. **Consistency**: Every README follows the same standards
2. **Efficiency**: Type 10 words instead of 50
3. **Iteration**: Refine the TOML once, improves all future uses
4. **Knowledge Transfer**: New team members use your prompt patterns

**This is Stage 3 thinking** (Intelligence Design): Encode recurring patterns into reusable intelligence.

---

## Concept 4: Three Roles Demonstration — Building a Markdown Workflow

### Setup

You're using Gemini CLI to create a custom command for generating API documentation. Your team has 10 services, each with 5-10 endpoints, and documentation is currently inconsistent (some services have good docs, some have none).

Your goal: Create a TOML command that generates consistent API docs.

Watch how AI plays three roles as you work together.

---

### Role 1: AI as Teacher (Suggests TOML Structure Pattern)

**You start with a basic request**:
```bash
I want to create a custom Gemini CLI command for generating API documentation.
Can you help me write the TOML configuration?
```

**AI responds** (teaching you a pattern):
```
To create an effective custom command, use this structure:

1. **Description**: What does this command do? (1 sentence)
2. **Prompt**: The full prompt template with variables
3. **Args**: What inputs does the command need?

For API documentation, I suggest these variables:
- {service_file}: The Python file with route definitions
- {base_url}: API base URL (e.g., https://api.example.com)
- {auth_type}: Authentication method (Bearer, API Key, OAuth)

Here's a starter TOML:

[api-docs]
description = "Generate API documentation for service endpoints"
prompt = """
Analyze the service code: {service_file}

Generate API documentation with:
1. Endpoint table (Method, Path, Description)
2. Request examples (with {auth_type} authentication)
3. Response examples (success + error cases)
4. Base URL: {base_url}

Use markdown tables for endpoints.
"""
args = ["service_file", "base_url", "auth_type"]

This gives you a reusable template. You fill in the variables each time:
gemini api-docs service_file=@src/routes.py base_url=https://api.example.com auth_type=Bearer
```

**What you learned**: You didn't know about the variable interpolation pattern ({variable_name}) in TOML commands. The AI taught you a reusable template structure. This is **AI as Teacher**.

**Evidence**: The AI suggested a specific TOML pattern (description + prompt + args) and explained WHY each part matters. This is reasoning about workflow design, not just template generation.

---

### Role 2: AI as Student (Adapts to Your Team's Standards)

**You provide domain feedback**:
```
Good start, but our team has specific requirements you missed:
- We always include rate limiting info (requests per minute)
- We use mermaid sequence diagrams for complex endpoints
- We document error codes (400, 401, 403, 404, 500) explicitly
- We need a "Quick Start" section showing curl examples

Can you update the TOML to include these requirements?
```

**AI responds** (adapted to your correction):
```
Ah, thank you for those team-specific requirements. That changes the prompt structure.

Updated TOML:

[api-docs]
description = "Generate API documentation with team standards (rate limiting, mermaid, error codes, quick start)"
prompt = """
Analyze the service code: {service_file}

Generate API documentation with these sections:

## Quick Start
- 3 curl examples (GET, POST, authentication)
- Base URL: {base_url}

## Endpoints
- Table: Method | Path | Description | Rate Limit
- Include requests per minute for each endpoint

## Authentication
- Type: {auth_type}
- Example header format

## Request/Response Examples
- For each endpoint: request + success response + error responses
- Document these error codes explicitly: 400, 401, 403, 404, 500

## Sequence Diagrams
- For endpoints with multi-step flows, include mermaid sequence diagrams

Use markdown tables. Be specific with examples (not placeholders).
"""
args = ["service_file", "base_url", "auth_type"]

Your team standards (rate limiting, error codes, sequence diagrams, quick start)
are now encoded in the prompt. Every API doc generated will include these.
```

**What AI learned**: You taught the AI your team's documentation standards (rate limiting, specific error codes, mermaid diagrams, curl examples). The AI adapted the TOML to include those requirements. This is **AI as Student**.

**Evidence**: The AI's initial TOML was generic. You corrected it with team-specific standards. The AI incorporated your requirements into the prompt template. Bidirectional learning.

---

### Role 3: AI as Co-Worker (Convergence Through Iteration)

**You test the command and refine**:
```bash
I tested the command on our auth-service:
gemini api-docs service_file=@src/routes.py base_url=https://api.example.com auth_type=Bearer

The output was good, but I noticed:
- Rate limits are hardcoded guesses (AI doesn't know our actual limits)
- Error responses are generic (not our actual error JSON structure)

Can we add variables for these?
```

**AI responds** (convergence loop):
```
Excellent catch. The command needs more context to be accurate.

Let's add two more variables:

1. {rate_limits_file}: Reference to your rate limiting config
2. {error_schema}: Your actual error response format

Updated TOML:

[api-docs]
description = "Generate accurate API docs with team standards + actual rate limits and error schemas"
prompt = """
Analyze the service code: {service_file}

Generate API documentation with these sections:

## Quick Start
- 3 curl examples (GET, POST, authentication)
- Base URL: {base_url}

## Endpoints
- Table: Method | Path | Description | Rate Limit
- Use ACTUAL rate limits from: {rate_limits_file}

## Authentication
- Type: {auth_type}
- Example header format

## Request/Response Examples
- For each endpoint: request + success response + error responses
- Use ACTUAL error schema from: {error_schema}
- Document these error codes: 400, 401, 403, 404, 500

## Sequence Diagrams
- For endpoints with multi-step flows, include mermaid sequence diagrams

Use markdown tables. Be specific with examples (not placeholders).
"""
args = ["service_file", "base_url", "auth_type", "rate_limits_file", "error_schema"]

Now you call it like this:
gemini api-docs \\
  service_file=@src/routes.py \\
  base_url=https://api.example.com \\
  auth_type=Bearer \\
  rate_limits_file=@config/rate_limits.yaml \\
  error_schema=@schemas/error_response.json

This references your actual config files, so the output is accurate.

I initially created a generic template. You tested it and found gaps
(missing rate limit specifics, generic error schemas). Together, we
converged on a command that references YOUR actual configuration files.

Neither of us had this solution initially:
- I didn't know you had rate_limits.yaml and error_response.json files
- You didn't know TOML supports multiple file references

Collaboration produced a better workflow.
```

**What happened**: Neither you nor the AI had the complete solution initially:
- You knew your team's standards but not TOML's full capabilities
- The AI knew TOML syntax but not your specific config files
- Together, you designed a command that references actual configs (not generic templates)

**This is AI as Co-Worker**: Convergence through iteration toward a better workflow.

**Evidence**: The final TOML command emerged from collaboration. Your testing feedback + AI's variable suggestion produced a result neither had alone.

---

## Creating GEMINI.md for Persistent Context

### Why GEMINI.md Matters

Just like CLAUDE.md (Lesson 4), GEMINI.md encodes persistent context for your projects. But GEMINI.md has an additional purpose: **defining custom commands locally**.

### What Goes in GEMINI.md

**Section 1: Project Context** (same as CLAUDE.md)
```markdown
# Project: Microservices API Documentation

## Decision Context
- Building consistent API docs for 10 microservices
- Team: 5 engineers, varying documentation quality
- Goal: Standardize docs across all services

## Success Criteria
- All services have README + API docs
- Consistent format (tables, mermaid, examples)
- Generated in <30 minutes per service
```

**Section 2: Custom Commands** (GEMINI.md specific)
```toml
[readme-gen]
description = "Generate production README"
prompt = """[Full prompt here]"""
args = ["service_file"]

[api-docs]
description = "Generate API documentation"
prompt = """[Full prompt here]"""
args = ["service_file", "base_url", "auth_type", "rate_limits_file", "error_schema"]
```

**Section 3: Workflow Guidance**
```markdown
## Common Workflows

### New Service Documentation
1. Generate README: `gemini readme-gen service_file=@src/main.py`
2. Generate API docs: `gemini api-docs [args]`
3. Validate: Check for missing sections

### Update Existing Docs
1. Check what changed: `gemini "What's outdated in @README.md compared to @src/main.py"`
2. Regenerate sections: Use custom commands with targeted prompts
```

### How Gemini Uses GEMINI.md

When GEMINI.md exists in your project:
1. Gemini loads custom commands automatically
2. You can use commands without specifying full TOML path
3. Context applies to all prompts in that project

---

## Self-Assessment: Workflow Design Practice

### Exercise 1: File References

You have these files:
- `src/auth_service.py` (authentication code)
- `src/user_service.py` (user management code)
- `docs/api.md` (existing API documentation)

**Write Gemini CLI commands using @filename syntax for these tasks**:

**Task A**: Check if API documentation matches the auth_service code
```bash
gemini "___________"
```

**Task B**: Compare error handling in auth_service vs user_service
```bash
gemini "___________"
```

**Task C**: Generate a security audit report analyzing auth_service
```bash
gemini "___________"
```

---

### Exercise 2: Shell Command Integration

**Write Gemini CLI commands using !(command) syntax for these tasks**:

**Task A**: Summarize changes in the last 5 commits
```bash
gemini "___________"
```

**Task B**: List all Python files modified in the last week
```bash
gemini "___________"
```

**Task C**: Check which dependencies in requirements.txt have security vulnerabilities
```bash
gemini "___________"
```

---

### Exercise 3: Design a Custom TOML Command

**Scenario**: Your team needs to generate consistent git commit messages following these rules:
- Format: `[TYPE] Brief summary (max 50 chars)`
- Types: feat, fix, docs, refactor, test, chore
- Include bullet points for changes (if multiple)
- Reference related issues (if any)

**Design a TOML command**:
```toml
[commit-msg]
description = "___________"
prompt = """
___________
"""
args = [___________]
```

---

### Answer Key (Self-Check)

**Exercise 1**:
- Task A: `gemini "Does @docs/api.md match @src/auth_service.py?"`
- Task B: `gemini "Compare error handling: @src/auth_service.py @src/user_service.py"`
- Task C: `gemini "Security audit for @src/auth_service.py"`

**Exercise 2**:
- Task A: `gemini "Summarize changes: !(git log --oneline -5)"`
- Task B: `gemini "Which Python files changed recently? !(find . -name '*.py' -mtime -7)"`
- Task C: `gemini "Check for vulnerabilities: !(cat requirements.txt)"`

**Exercise 3**:
```toml
[commit-msg]
description = "Generate standardized commit message following team conventions"
prompt = """
Generate a git commit message for these changes: {changes}

Format:
[TYPE] Brief summary (max 50 chars)

- Bullet point 1
- Bullet point 2

Related issues: {issues}

Types: feat, fix, docs, refactor, test, chore
Choose the most appropriate type.
"""
args = ["changes", "issues"]
```

---

## Try With AI

**Setup**: Open Gemini CLI in any terminal. Create a test project directory with a few markdown files or code files to practice.

**Exercise**: Apply file references, shell commands, and custom TOML commands.

---

### Practice Task 1: File Reference Mastery

**Goal**: Use @filename syntax to analyze multiple files.

**Prompts to Try**:

**Prompt 1: Single File Analysis**
```bash
gemini "What's the main purpose of @README.md? Summarize in 2 sentences."
```

**Expected outcome**: AI reads README.md and provides concise summary.

---

**Prompt 2: Multi-File Comparison**
```bash
gemini "Compare these files and identify inconsistencies: @file1.md @file2.md"
```

**Expected outcome**: AI identifies differences in structure, content, or quality.

---

**Prompt 3: Template-Based Generation**
```bash
gemini "Generate a new README following this template: @templates/readme-template.md
        For this service: @src/main.py"
```

**Expected outcome**: AI generates README matching template structure using service code context.

---

### Practice Task 2: Shell Command Integration

**Goal**: Use !(command) syntax to integrate system information.

**Prompts to Try**:

**Prompt 1: Git History Analysis**
```bash
gemini "Summarize what changed in the last 3 commits: !(git log --oneline -3)"
```

**Expected outcome**: AI reads git log output and provides summary.

---

**Prompt 2: File Discovery**
```bash
gemini "Which markdown files exist in this project? !(find . -name '*.md')"
```

**Expected outcome**: AI lists markdown files and suggests analysis.

---

### Practice Task 3: Custom TOML Command Creation

**Goal**: Create a custom command for a recurring workflow.

**Steps**:

1. **Create commands.toml file**:
```toml
[doc-check]
description = "Check documentation quality"
prompt = """
Analyze this documentation file: {doc_file}

Check for:
1. Missing sections (introduction, examples, API reference)
2. Broken links or references
3. Outdated information (flag anything mentioning old versions)
4. Code examples (do they include expected outputs?)

Provide:
- Quality score (1-10)
- Specific improvements needed
- Priority (high/medium/low) for each issue
"""
args = ["doc_file"]
```

2. **Test the command**:
```bash
gemini doc-check doc_file=@README.md
```

3. **Iterate based on results**:
- Did AI identify real issues?
- Were recommendations actionable?
- What would you add to the prompt?

---

### Challenge: Three Roles Validation

**As you work with Gemini CLI, identify these moments**:

1. **AI as Teacher**: When did AI suggest a file reference or command pattern you didn't know?
   - What did you learn? _________

2. **AI as Student**: When did you correct or refine AI's output based on your domain knowledge?
   - What did AI learn from you? _________

3. **AI as Co-Worker**: When did you converge on a workflow neither had initially?
   - What emerged from collaboration? _________

---

### Safety Note

**Validate command outputs**: Always verify that:
- **@filename** references return actual file contents (spot-check key sections)
- **!(command)** executes safely (avoid destructive commands like `rm`, `git reset --hard`)
- **Custom TOML commands** produce expected outputs (test on small examples first)
- **Generated documentation** is accurate (cross-reference with actual code)

Gemini CLI is powerful, but you remain responsible for validating outputs and ensuring commands are safe.

---

**Lesson Metadata**:
- **Stage**: 2 (AI Collaboration)
- **Modality**: Hands-on exploration
- **Concepts**: 4 (@filename, !command, custom TOML, GEMINI.md)
- **Cognitive Load**: B1 tier (4 ≤ 10)
- **AI Tools**: Gemini CLI (@filename, !command, custom commands)
- **Duration**: 75 minutes
- **Three Roles**: Demonstrated with explicit callouts (AI as Teacher, Student, Co-Worker)
- **Version**: 1.0.0
- **Constitution**: v6.0.0 Compliance
- **Generated**: 2025-01-18
- **Source Spec**: `specs/025-chapter-10-redesign/spec.md`
