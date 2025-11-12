# What Data You Actually Get from Claude Code Telemetry

**Important**: Claude Code's built-in telemetry captures **high-level metrics**, not detailed trace logs of every interaction.

---

## ✅ What IS Captured

### 1. User Prompts
**Event**: `claude_code.user_prompt`

**Data Included**:
- Prompt length (character count)
- Timestamp
- Session ID

**Data NOT Included**:
- ❌ Actual prompt text (unless you set `OTEL_LOG_USER_PROMPTS=1`)
- ❌ User identity beyond UUID

**Example**:
```json
{
  "event": "claude_code.user_prompt",
  "prompt_length": 145,
  "session_id": "abc-123"
}
```

### 2. Tool Execution Results
**Event**: `claude_code.tool_result`

**Data Included**:
- Tool name (Read, Write, Edit, Bash, etc.)
- Success/failure status
- Execution duration (milliseconds)
- User decision (accept/reject)
- Tool parameters (what files, what commands)

**Data NOT Included**:
- ❌ File contents
- ❌ Command output
- ❌ Detailed error messages

**Example**:
```json
{
  "event": "claude_code.tool_result",
  "tool": "Edit",
  "file": "src/main.py",
  "success": true,
  "duration_ms": 125,
  "decision": "accept"
}
```

### 3. API Requests to Claude
**Event**: `claude_code.api_request`

**Data Included**:
- Model used (claude-sonnet-4.5, etc.)
- Token counts (input, output, cache read, cache write)
- Cost in USD
- Duration (milliseconds)
- Session ID

**Data NOT Included**:
- ❌ Actual messages sent/received
- ❌ API response content

**Example**:
```json
{
  "event": "claude_code.api_request",
  "model": "claude-sonnet-4.5",
  "tokens": 450,
  "input_tokens": 150,
  "output_tokens": 300,
  "cost": 0.0045,
  "duration_ms": 1200
}
```

### 4. API Errors
**Event**: `claude_code.api_error`

**Data Included**:
- Error type
- Status code
- Retry attempt number

**Data NOT Included**:
- ❌ Full error details
- ❌ Request that caused error

### 5. Tool Permission Decisions
**Event**: `claude_code.tool_decision`

**Data Included**:
- Decision (accept/reject)
- Tool being authorized
- Decision source (user, auto-approve)

### 6. Aggregate Metrics

**Metrics Captured**:
- `claude_code.session.count` - Number of sessions
- `claude_code.token.usage` - Total tokens consumed
- `claude_code.cost.usage` - Total cost in USD
- `claude_code.lines_of_code.count` - Lines modified
- `claude_code.commit.count` - Git commits created
- `claude_code.pull_request.count` - PRs created
- `claude_code.active_time.total` - Active session time (seconds)

---

## ❌ What is NOT Captured

### 1. Subagent/Task Tool Details ⚠️

**Key Limitation**: The documentation does **not explicitly list** subagent invocations as a captured event.

**What this means**:
- ❌ When you invoke a subagent (like `lesson-writer`, `technical-reviewer`), it may NOT appear as a distinct event
- ⚠️ Subagent activity is likely captured as **tool calls** (the tools the subagent uses)
- ⚠️ You may NOT see "lesson-writer started" or "technical-reviewer completed" in telemetry

**What you WILL see**:
- ✅ API requests the subagent makes (costs, tokens)
- ✅ Tool calls the subagent executes (Read, Write, Edit)
- ✅ Overall session metrics (total cost, total tokens)

**What you WON'T see**:
- ❌ Which subagent made which tool call
- ❌ Subagent start/end times
- ❌ Subagent-specific costs
- ❌ Subagent hierarchy (parent/child relationships)

### 2. Content Data

**Never Captured**:
- ❌ File contents
- ❌ Code snippets
- ❌ Command outputs (stdout/stderr)
- ❌ Conversation history
- ❌ API request/response bodies
- ❌ User prompt text (unless OTEL_LOG_USER_PROMPTS=1)

### 3. Detailed Execution Traces

**Not Available**:
- ❌ Step-by-step workflow traces
- ❌ Decision trees (why AI chose action X)
- ❌ Reasoning process
- ❌ Tool call sequences with causality

### 4. Sensitive Information

**Explicitly Filtered**:
- ❌ API keys
- ❌ Credentials
- ❌ Personal data
- ❌ Proprietary code

---

## 🤔 What This Means for Andrew Ng's Strategy

### ✅ What You CAN Do

**1. Cost Analysis**
- Track total spend by session, user, day
- Identify expensive workflows
- Budget tracking

**2. High-Level Error Analysis**
- Identify sessions with API errors
- Track retry rates
- Find patterns in failures

**3. Tool Usage Patterns**
- Which tools are used most
- Success/failure rates
- Performance (duration)

**4. Token Usage Optimization**
- Track token consumption trends
- Identify inefficient prompts (high tokens, low value)
- Cache effectiveness

**5. Session Metrics**
- Active time per session
- Code modification volume
- Git activity

### ⚠️ What You CANNOT Do

**1. Detailed Workflow Tracing**
- Cannot trace "user asked X → subagent Y ran → tools A,B,C executed → result Z"
- Cannot attribute tool calls to specific subagents
- Cannot build causality graphs

**2. Content-Based Analysis**
- Cannot analyze prompt quality (no text unless opt-in)
- Cannot review actual AI responses
- Cannot inspect generated code

**3. Subagent-Specific Insights**
- Cannot measure subagent performance individually
- Cannot compare lesson-writer vs proof-validator costs
- Cannot optimize subagent workflows specifically

---

## 💡 Recommended Use Cases

Given these limitations, here's what the telemetry system is BEST for:

### High-Value Use Cases ✅

1. **Cost Management**
   - Track spending trends
   - Identify cost spikes
   - Budget alerts

2. **Performance Monitoring**
   - Average session costs
   - Token efficiency
   - API error rates

3. **Usage Patterns**
   - Active hours
   - Tool preferences
   - Session frequency

4. **High-Level Error Detection**
   - Sessions with multiple retries
   - API failure patterns
   - Tool permission issues

### Limited-Value Use Cases ⚠️

1. **Detailed Debugging**
   - Use Claude Code's built-in logging instead
   - Check session files manually

2. **Content Quality Analysis**
   - No access to actual outputs
   - Need manual review

3. **Subagent Optimization**
   - Cannot isolate subagent costs
   - Need to instrument subagents separately

4. **Workflow Redesign**
   - Limited causality data
   - Need manual observation

---

## 🔍 Augmenting Telemetry Data

To get the detailed traces you need for Andrew Ng's methodology, **combine telemetry with other sources**:

### 1. Session Logs (Manual)
- Review Claude Code session transcripts
- Manually trace workflow steps
- Identify failure points

### 2. Git History
- Correlate telemetry sessions with commits
- Track what changed in each session
- Quality assessment via code review

### 3. Custom Instrumentation
- Add logging to subagents (future)
- Track workflow steps explicitly
- Export custom events

### 4. Manual Annotation
- Tag sessions with outcome (success/failure)
- Note workflow type (chapter writing, debugging, etc.)
- Document learnings

---

## Example: What You'll Actually See

### Scenario: You write a chapter using lesson-writer subagent

**Telemetry Captures**:
```json
Session Start: 14:30:00

Events:
- user_prompt (length: 52) at 14:30:05
- api_request (tokens: 450, cost: $0.004) at 14:30:10
- tool_result (Write, lesson.md, success) at 14:30:15
- api_request (tokens: 1200, cost: $0.012) at 14:30:20
- tool_result (Edit, lesson.md, success) at 14:30:25
- api_request (tokens: 800, cost: $0.008) at 14:30:30

Metrics:
- Total cost: $0.024
- Total tokens: 2450
- Lines modified: 145
- Active time: 300 seconds
```

**What you DON'T see**:
- ❌ That lesson-writer subagent was used
- ❌ Which Write was from lesson-writer vs main agent
- ❌ What the prompt actually said
- ❌ What was written to lesson.md
- ❌ Whether output met quality standards

**What you CAN infer**:
- ✅ 3 API calls, $0.024 total cost
- ✅ 2 tool calls (Write + Edit)
- ✅ 2450 tokens consumed
- ✅ Session lasted 5 minutes

---

## Setting Expectations

**Telemetry is NOT**:
- ❌ A detailed trace logger
- ❌ A debugging tool
- ❌ A quality assessment system
- ❌ A workflow visualization tool

**Telemetry IS**:
- ✅ A cost tracking system
- ✅ A usage pattern monitor
- ✅ A high-level error detector
- ✅ A performance dashboard

---

## Recommendations

### For Your Use Case (Book Development Team)

**Primary Value**:
1. **Cost tracking** - Monitor team spend
2. **Usage patterns** - Who's using Claude Code when
3. **Error detection** - Flag problematic sessions for manual review

**Limited Value**:
4. **Workflow optimization** - Cannot trace subagent workflows automatically
5. **Quality analysis** - Need manual content review

**Suggested Workflow**:
1. Use telemetry for **cost alerts** and **error flagging**
2. Use **manual session review** for workflow analysis
3. Use **git history** for quality tracking
4. **Annotate** high-cost or error sessions with manual notes

### For Andrew Ng's Methodology

**What Telemetry Provides**:
- ✅ Breaking data silos: Centralized cost/usage data
- ⚠️ Error analysis: High-level detection, not detailed traces
- ⚠️ Evals-first: Can track cost trends, not quality improvements

**What You Need to Add**:
- Manual workflow traces
- Quality scoring (separate from telemetry)
- Content review process
- Custom instrumentation for subagents

---

## Bottom Line

You get:
- ✅ **Costs** (total, per-session, per-user)
- ✅ **Tokens** (total, input/output breakdown)
- ✅ **Tool calls** (which tools, success/failure, duration)
- ✅ **API calls** (count, cost, tokens)
- ✅ **Sessions** (count, duration, activity)

You DON'T get:
- ❌ **Subagent traces** (which subagent did what)
- ❌ **Content** (prompts, code, outputs)
- ❌ **Detailed workflows** (step-by-step causality)
- ❌ **Quality metrics** (good/bad output determination)

**This is still valuable** for cost management and high-level monitoring, but **not a complete solution** for detailed workflow analysis.

For full workflow tracing, you'll need to **supplement with manual observation** and potentially **custom instrumentation** of your subagents.
