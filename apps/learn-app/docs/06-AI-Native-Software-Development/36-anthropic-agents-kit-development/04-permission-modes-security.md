---
sidebar_position: 4
title: "Permission Modes and Runtime Security"
description: "Configure permission_mode for different security postures and implement dynamic runtime permissions using canUseTool. Design security policies that adapt based on context."
keywords: [Claude Agent SDK, permission modes, canUseTool callback, security policy, runtime permissions, agent security]
chapter: 36
lesson: 4
duration_minutes: 30

# HIDDEN SKILLS METADATA
skills:
  - name: "Configuring Permission Modes for Security Postures"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety and Security"
    measurable_at_this_level: "Student can select appropriate permission_mode (default, acceptEdits, bypassPermissions, plan) based on agent use case and trust boundary"

  - name: "Implementing canUseTool Runtime Permission Callbacks"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety and Security"
    measurable_at_this_level: "Student can write canUseTool callbacks that evaluate tool requests in context, modify inputs, and make dynamic allow/deny decisions"

  - name: "Designing Context-Aware Security Policies"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Analyze"
    digcomp_area: "Digital Safety and Security"
    measurable_at_this_level: "Student can design security policies that account for file paths, user roles, operation types, and organizational constraints"

learning_objectives:
  - objective: "Understand the four permission modes and when each is appropriate for different agent configurations"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Scenario matching: given 4 agent use cases, select correct permission_mode with justification"

  - objective: "Implement canUseTool callbacks that make dynamic permission decisions based on tool, input, and context"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Code implementation: student writes callback that correctly evaluates multiple security constraints"

  - objective: "Design security policies that balance agent capability with organizational safety requirements"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Design task: student articulates security policy for given business scenario with risk analysis"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (4 permission modes + canUseTool + context evaluation + policy design + error handling + input modification + deny patterns + compliance) within B1-B2 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Design multi-tiered permission system with escalation rules; implement audit logging with PostToolUse hooks; create policy DSL for complex organizational requirements"
  remedial_for_struggling: "Start with permission_mode selection only; understand default vs acceptEdits difference. Then add simple canUseTool callbacks that check file paths. Build complexity incrementally."

---

# Permission Modes and Runtime Security

You've built agents that read files, run commands, and modify code. Now comes the critical question: How much power should your agent actually have?

An agent with full access is powerful. It can fix anything. But what if it accidentally deletes something critical? What if it modifies protected files? What if a security vulnerability in your system gets exploited through the agent?

This lesson teaches the security architecture that separates powerful agents from *responsible* agents—the difference between tools that work in development and tools your customers trust in production.

## The Permission Problem

Every tool your agent can access is a **trust boundary**. Some tools are read-only (safe). Some can modify systems (dangerous). Some can delete things (catastrophic).

Traditional security approaches say: "Give minimum permissions from the start." But with agents, there's a subtlety: You might want an agent to modify files in most cases, but protect specific directories. You might want it to run Bash commands for development, but block it during deployments. You might want it to ask permission the first time, then auto-approve afterward.

Static permission rules (allowing or blocking tools entirely) can't express this nuance. You need **dynamic, context-aware security decisions**.

## Permission Modes: Four Security Postures

Claude Agent SDK offers four permission modes, each representing a different trust boundary:

### Mode 1: `default`

**Behavior**: Standard permission checks. Agent makes tool requests. System prompts user to approve/deny.

**When to use**: Development, research, one-off tasks where human oversight is practical.

**Tradeoff**: Safest but slowest. Requires human to evaluate each tool call.

**Example**:
```python
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Write", "Edit"],
    permission_mode="default"  # User prompted for each tool call
)
```

**Scenario**: You're debugging a production issue. You want the agent to suggest file changes, but you review each change before it applies.

### Mode 2: `acceptEdits`

**Behavior**: Auto-approve file modifications (Edit, Write). Other tools still prompt.

**When to use**: Development environments where you trust the agent's modifications. Building features locally.

**Tradeoff**: Faster iteration, but file changes happen without review. Other tools (Bash, WebFetch) still require approval.

**Example**:
```python
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Write", "Edit", "Bash"],
    permission_mode="acceptEdits"  # File edits auto-approved, Bash still prompts
)
```

**Scenario**: You're working in a local development branch. You want the agent to fix style issues automatically, but running untested Bash commands should still require your approval.

### Mode 3: `bypassPermissions`

**Behavior**: No permission prompts. All approved tools execute automatically.

**When to use**: Fully autonomous deployments. CI/CD pipelines. Production systems where human review isn't practical.

**WARNING**: Use only when you have confidence in your security policy. This mode should be paired with strict `allowed_tools` lists and robust `canUseTool` callbacks.

**Example**:
```python
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Edit"],  # Intentionally narrow
    permission_mode="bypassPermissions",  # Execute without prompts
    can_use_tool=security_policy  # Backed by runtime validation
)
```

**Scenario**: Your CI/CD agent reformats code on commit. It has Edit permission only. No prompts needed because the tool scope is already restricted.

### Mode 4: `plan`

**Behavior**: Planning mode. Agent reasons and plans but doesn't execute any tools.

**When to use**: Understanding what an agent *would* do without actually doing it. Safety validation. Code review before execution.

**Example**:
```python
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Write", "Edit"],
    permission_mode="plan"  # Agent proposes changes, doesn't apply them
)
```

**Scenario**: You want the agent to analyze what migrations a database upgrade would require, without executing the migrations.

### Decision Framework: Selecting Permission Modes

| Use Case | Mode | Reasoning |
|----------|------|-----------|
| Development (human present) | `default` | Review each tool call manually |
| Local development (trust agent) | `acceptEdits` | Auto-approve files, prompt for system commands |
| CI/CD (automated, restricted tools) | `bypassPermissions` | No prompts, enforce via tool restrictions + canUseTool |
| Analysis/planning (no execution) | `plan` | See what agent would do without doing it |
| Research (human reviewing process) | `default` | Transparency during exploration |
| Production automation (high trust) | `bypassPermissions` + canUseTool | Max autonomy with runtime guardrails |

## Dynamic Runtime Permissions: canUseTool Callback

Permission modes set the baseline. But real-world security needs rules that change based on *what* the agent is trying to do.

This is where **canUseTool** comes in—a callback function that evaluates every tool request in context.

### Understanding canUseTool

The `canUseTool` callback intercepts every tool call:

```python
async def can_use_tool(tool: str, input: dict, context: dict):
    # tool: Name of tool being called ("Write", "Bash", "WebSearch", etc.)
    # input: The tool's input parameters (file_path, command, search_query, etc.)
    # context: Agent state (user_id, session_metadata, etc.)

    # Return decision
    return {
        "behavior": "allow" | "deny" | "updatedInput",
        "message": "Optional reason (shown in logs)",
        "updatedInput": {...}  # Only for "updatedInput" behavior
    }
```

**Key insight**: You can evaluate the request, modify it, or block it entirely.

### Security Rule 1: Protect Configuration Files

Scenario: You want to allow your agent to write code, but never touch configuration files.

```python
async def can_use_tool(tool: str, input: dict, context: dict):
    # Block writes to protected config directories
    if tool == "Write":
        file_path = input.get("file_path", "")
        protected_dirs = ["/etc", "/config", ".env"]

        for protected in protected_dirs:
            if protected in file_path:
                return {
                    "behavior": "deny",
                    "message": f"Config files protected. Cannot write to {file_path}"
                }

    # Allow everything else
    return {"behavior": "allow"}
```

**What's happening**:
1. Check if tool is Write
2. Extract file path from input
3. Check against protected directories
4. If protected, deny with reason
5. Otherwise, allow

**Impact**: Agent can refactor code freely but cannot modify configs, secrets, or infrastructure files.

### Security Rule 2: Time-Based Permissions

Scenario: Allow code changes during business hours, but deny during production window.

```python
import datetime

async def can_use_tool(tool: str, input: dict, context: dict):
    # Restrict Edit operations outside business hours
    if tool == "Edit":
        now = datetime.datetime.now()
        hour = now.hour

        # Block edits 10pm-6am (production window)
        if hour < 6 or hour >= 22:
            return {
                "behavior": "deny",
                "message": "Edits blocked during production window (10pm-6am)"
            }

    return {"behavior": "allow"}
```

**Impact**: Prevents accidental changes during critical periods.

### Security Rule 3: Input Modification

Scenario: Allow Bash commands but sanitize potentially dangerous ones.

```python
async def can_use_tool(tool: str, input: dict, context: dict):
    # Sanitize Bash commands
    if tool == "Bash":
        command = input.get("command", "")

        # Block destructive commands
        dangerous_patterns = [
            "rm -rf /",
            "sudo",
            ": () { : | : & }",  # Fork bomb
            "dd if=/dev/zero"  # Disk wipe
        ]

        for pattern in dangerous_patterns:
            if pattern in command:
                return {
                    "behavior": "deny",
                    "message": f"Dangerous command blocked: {pattern}"
                }

        # Sanitize: wrap in timeout, disable dangerous flags
        safe_command = f"timeout 30s {command}"
        return {
            "behavior": "updatedInput",
            "updatedInput": {"command": safe_command},
            "message": "Command wrapped with 30s timeout"
        }

    return {"behavior": "allow"}
```

**Impact**: Bash commands execute safely with automatic timeout protection.

### Security Rule 4: Role-Based Access Control

Scenario: Different agents have different permissions based on their role.

```python
async def can_use_tool(tool: str, input: dict, context: dict):
    user_role = context.get("user_role", "viewer")

    # Roles: viewer, editor, admin
    if user_role == "viewer":
        # Viewers can only read
        if tool not in ["Read", "WebSearch"]:
            return {"behavior": "deny", "message": "Viewers can only read"}

    elif user_role == "editor":
        # Editors can read/write/edit but not run Bash
        if tool in ["Bash"]:
            return {"behavior": "deny", "message": "Editors cannot execute system commands"}

    elif user_role == "admin":
        # Admins can do everything
        pass

    return {"behavior": "allow"}
```

**Impact**: Multi-tenant system where users have different capabilities.

### Security Rule 5: Audit Logging

Scenario: Log all tool usage for compliance.

```python
import json
from datetime import datetime

async def can_use_tool(tool: str, input: dict, context: dict):
    # Log every tool call
    log_entry = {
        "timestamp": datetime.now().isoformat(),
        "tool": tool,
        "user": context.get("user_id", "unknown"),
        "action": tool,
        "resource": input.get("file_path") or input.get("command", "N/A")
    }

    # In real system, write to audit log
    print(f"AUDIT: {json.dumps(log_entry)}")

    # Process normally
    return {"behavior": "allow"}
```

**Output**:
```json
AUDIT: {"timestamp": "2025-12-26T10:15:30", "tool": "Edit", "user": "alice", "action": "Edit", "resource": "src/auth.py"}
AUDIT: {"timestamp": "2025-12-26T10:15:35", "tool": "Bash", "user": "alice", "action": "Bash", "resource": "npm test"}
```

**Impact**: Complete audit trail for compliance and security investigation.

## Case Study: Building a Read-Only Research Agent

Let's design an agent that researches competitive analysis but can't modify anything.

```python
from claude_agent_sdk import query, ClaudeAgentOptions

async def research_agent():
    """Agent that researches but cannot modify local files"""

    async def research_policy(tool: str, input: dict, context: dict):
        # Allow reads and web searches
        if tool in ["Read", "WebSearch", "WebFetch"]:
            return {"behavior": "allow"}

        # Block any modification
        if tool in ["Write", "Edit", "Bash"]:
            return {
                "behavior": "deny",
                "message": f"Research agent cannot execute {tool}"
            }

        # Other tools (Glob, Grep) only for reading
        return {"behavior": "allow"}

    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Glob", "Grep", "WebSearch", "WebFetch"],
        permission_mode="bypassPermissions",  # No prompts, agent is safe
        can_use_tool=research_policy  # Belt and suspenders
    )

    prompt = """Research our top 3 competitors:
    1. Find their pricing models
    2. Identify their main product features
    3. List their target customer segments
    Create a summary."""

    async for message in query(prompt=prompt, options=options):
        if hasattr(message, "result"):
            print(message.result)
```

**Security analysis**:
- Allowed tools: Read (files), Glob/Grep (search), WebSearch/WebFetch (research)
- Denied tools: Write, Edit, Bash (no modifications possible)
- Permission mode: bypassPermissions (no prompts, safe because agent is restricted)
- canUseTool: Redundant but provides defense in depth
- Risk profile: **Very low**. Agent cannot modify anything locally or systemically.

## Case Study: Building a Development Agent with Guards

Now let's build an agent that can fix code bugs but has guardrails.

```python
async def development_agent_policy(tool: str, input: dict, context: dict):
    """Allows code changes with protection for critical files"""

    # Protect critical paths
    protected_patterns = [
        "package.json",
        "pyproject.toml",
        ".env",
        "docker-compose.yml",
        "terraform/",
        "infrastructure/"
    ]

    if tool in ["Write", "Edit"]:
        file_path = input.get("file_path", "").lower()

        # Check if file matches protected pattern
        for pattern in protected_patterns:
            if pattern.lower() in file_path:
                return {
                    "behavior": "deny",
                    "message": f"Cannot modify {pattern} - infrastructure protection"
                }

        # Allow code changes
        return {"behavior": "allow"}

    # Bash: Sanitize commands
    if tool == "Bash":
        command = input.get("command", "")

        # Allow safe commands
        safe_prefixes = ["npm", "python", "cargo", "go", "make"]
        if any(command.startswith(p) for p in safe_prefixes):
            # Limit to dev environment
            if "ENVIRONMENT" not in context or context["ENVIRONMENT"] != "production":
                return {"behavior": "allow"}

        # Deny production deployments
        if "deploy" in command.lower():
            return {
                "behavior": "deny",
                "message": "Deployment commands must be approved manually"
            }

        # Other Bash commands require approval
        return {"behavior": "allow"}

    # Allow reads
    if tool in ["Read", "Glob", "Grep", "WebSearch", "WebFetch"]:
        return {"behavior": "allow"}

    return {"behavior": "deny", "message": f"Tool {tool} not allowed"}


options = ClaudeAgentOptions(
    allowed_tools=["Read", "Write", "Edit", "Bash", "Glob", "Grep"],
    permission_mode="acceptEdits",  # Auto-approve file edits
    can_use_tool=development_agent_policy,
    system_prompt="You are a bug-fixing specialist. Fix code issues without modifying infrastructure."
)
```

**Security analysis**:
- Protected files: Configuration, environment, infrastructure files cannot be modified
- Bash restrictions: Development commands allowed, deployment commands denied
- Permission mode: acceptEdits (files auto-approved, which is OK because canUseTool prevents protected files)
- Risk profile: **Medium**. Agent can fix code but cannot modify critical files.

## Error Handling: What Happens When Permission Is Denied

When canUseTool returns `deny`, the agent receives an error message, not tool execution.

```python
async def strict_policy(tool: str, input: dict, context: dict):
    if tool == "Bash":
        return {
            "behavior": "deny",
            "message": "Bash not allowed in this context"
        }
    return {"behavior": "allow"}

# Agent receives: "Tool Bash denied: Bash not allowed in this context"
# Agent must adapt strategy—perhaps suggesting the fix instead of executing
```

**How the agent responds**:
1. Receives deny error
2. Understands tool is unavailable
3. Adapts by suggesting command instead of executing
4. Or asks for human help
5. Continues with available tools

This is where **agent reasoning matters**. A well-designed agent adapts when tools are unavailable.

## Try With AI

Now you'll design security policies for real scenarios.

### Prompt 1: Research-to-Production Pipeline

**Setup**: Your organization runs customer support research agents during the day and deployment automation at night. They share the same codebase.

**Prompt**:
```
Design a canUseTool callback for a shared agent that:
1. During business hours (6am-6pm): Allows Read, WebSearch, and Edit for support research
2. During off-hours (6pm-6am): Allows Read, Bash, and Edit for deployment automation
3. At all times: Protects .env files and terraform/ directory
4. Blocks dangerous Bash patterns: rm -rf, sudo, reboot

Provide the Python function with the full policy logic.
```

**What you're learning**: Time-based access control, context-aware decisions, protecting critical infrastructure while allowing necessary operations.

### Prompt 2: Multi-User Agent System

**Setup**: Your company has research, developer, and ops roles using the same agent. Each role needs different capabilities.

**Prompt**:
```
Design a canUseTool callback that implements role-based access control:
- Research role: Can Read, WebSearch, WebFetch (view-only research)
- Developer role: Can Read, Write, Edit, Bash for local development
- Ops role: Can Read, Edit, Bash but NOT Write (prevent file creation in production)

Each role also has these constraints:
- No role can modify /etc or /config directories
- Bash commands are limited to 60-second timeout
- All tool usage must be logged with timestamp and user_id

Provide the complete implementation.
```

**What you're learning**: Composing multiple security rules, auditing for compliance, enforcing constraints across user tiers.

### Prompt 3: Your Scenario

Think of an agent you want to build for your own work. Describe the task and the security constraints you'd need.

**Prompt**:
```
I want to build an agent that [describe your scenario: "analyzes code quality", "manages database migrations", "updates documentation", etc.]

What permission mode would you recommend? Design a canUseTool callback that:
1. Allows the agent to accomplish its core task
2. Protects critical files or operations
3. Prevents dangerous actions
4. Logs usage for auditability

Explain the security tradeoffs in your design.
```

**What you're learning**: Translating business requirements into security policies, balancing capability with safety.

---

**Key Takeaway**: Security isn't about blocking everything. It's about designing trust boundaries so agents can be powerful and safe at the same time. Permission modes give you baseline controls. `canUseTool` gives you surgical precision. Together, they let you deploy agents in production with confidence.

