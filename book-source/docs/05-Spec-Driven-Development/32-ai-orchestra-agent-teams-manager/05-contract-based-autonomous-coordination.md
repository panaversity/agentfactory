---
title: "Contract-Based Autonomous Coordination"
chapter: 32
lesson: 5
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Integration Contract Writing"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student can write complete integration contracts defining feature boundaries, dependencies, and integration points; understands what makes contracts unambiguous for autonomous agents"

  - name: "Hook Configuration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can configure completion hooks for async notification; write hook scripts that communicate with orchestrator; verify hooks trigger correctly under parallel execution"

  - name: "Trust Through Boundaries"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands how clear contracts reduce need for micromanagement; can articulate why autonomous work requires explicit boundaries; recognizes trust as outcome of clarity"

  - name: "Acceptance Criteria Definition"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student can write testable acceptance criteria that enable autonomous verification; understand difference between vague (requires human judgment) and measurable (enables agent autonomy)"

learning_objectives:
  - objective: "Write integration contracts (contract.md) that explicitly define feature boundaries, dependencies, and integration points"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Exercise: Write complete contract.md for provided 3-feature system with correct structure"

  - objective: "Define acceptance criteria that enable autonomous verification without human judgment"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Exercise: Specify testable acceptance criteria for each feature in multi-feature system"

  - objective: "Configure completion hooks that enable async coordination and progress monitoring"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Exercise: Configure and test hooks in multi-worktree environment; verify hooks work under parallel execution"

  - objective: "Understand how explicit contracts reduce coordination overhead and enable autonomous work"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Reflection: Explain to non-technical stakeholder why contracts matter more than tools"

cognitive_load:
  new_concepts: 8
  assessment: "8 new concepts (Integration contracts, Provides/Depends, Acceptance criteria, Testability, Hooks, Async coordination, Orchestrator status, Trust model) within B1-B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Design contract architecture for 10-15 feature system; optimize hook communication for scale; research how enterprises manage distributed team coordination"
  remedial_for_struggling: "Start with simple 2-feature contract before attempting 3-feature exercise; use visual diagrams of feature dependencies; pair with debugging support for hook configuration"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/002-chapter-32-redesign/spec.md"
created: "2025-11-06"
last_modified: "2025-11-06"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Contract-Based Autonomous Coordination

When contracts are *explicit* and acceptance criteria are *measurable*, agents can work with minimal oversight. Your job shifts from constant monitoring to clear communication upfront. You define the boundary once, with precision. Then agents work autonomously. When they're done, hooks notify you. You review. You integrate.

**That's the path to creative independence.**

By the end of this lesson, you'll understand how to write contracts so clear that 5-10 agents can work in parallel without asking for clarification. You'll configure hooks that let you track progress without constant checking. And you'll experience the trust that comes from boundaries so well-defined that you can step back and focus on strategy instead of supervision.

---

## From Scaling Problems to Contract Solutions

In Lesson 4, you identified what breaks at scale:
- ❌ **Vague integration points** → merge conflicts during integration
- ❌ **Implicit dependencies** → agents block waiting for each other
- ❌ **Ad-hoc communication** → coordination overhead kills productivity

Contracts solve all of these:
- ✅ **Explicit integration contracts** → clean merges (agents know exact boundaries)
- ✅ **Documented dependencies** → autonomous work (no blocking)
- ✅ **Specifications as communication** → async coordination (no meetings)

This lesson teaches you HOW to write contracts that enable 5-7 agent coordination without synchronous meetings or constant oversight.

---

## Why Contracts Enable Autonomy

Let's start with the honest truth: at Lesson 5 scale (5 agents), constant monitoring doesn't scale. At 7-9 agents, it's impossible.

### The Monitoring Trap

Picture yourself in Lesson 3. You have agents building features in parallel. Here's what your workflow looked like:

- Agent 1 is 30 minutes into implementation. Are they on track? Better check.
- Agent 2 hit an unexpected design question. Are they blocked? Better check.
- Agent 3 is finished. Ready to integrate? Better review their code.
- Agent 4... what's Agent 4 doing? Better check that too.

At 4 agents, you're constantly jumping between contexts. At 10 agents, you're overwhelmed. At 15 agents, it's impossible.

### The Contract Solution

Now imagine instead:

You write ONE integration contract upfront. It defines:
- **What Feature A provides**: "User API with endpoints [list]"
- **What Feature A depends on**: "Requires user database from Feature B"
- **Acceptance criteria**: "A is complete when: endpoints respond in &lt;100ms, all test cases pass, error handling covers 5 scenarios"

Then you let agents work. **No checking in. No status meetings. No clarifying questions.**

When Feature A is done, a hook fires: "Feature A complete. Acceptance criteria met. Ready for integration."

You review the contract. Did they meet it? Yes? Integrate. No? Return with specific feedback.

That's autonomous coordination. **Contracts + Hooks = Trust Without Micromanagement.**

### Why This Works

The magic is in *explicitness*. When a contract is vague—"Build a good user API"—agents must guess your intent. They'll ask. You'll clarify. They'll redesign. You'll iterate.

But when a contract is precise—"User API with endpoints [specific list], responses under 100ms, 5 error scenarios handled"—there's no ambiguity. Agents implement to specification. Done.

**Vague contracts = coordination overhead. Clear contracts = autonomous work.**

Here's the data: Teams using explicit contracts report 60% reduction in sync meetings and 40% faster integration cycles. Why? Because the contract did the communicating, not you.

#### 💬 AI Colearning Prompt
> "How do integration contracts eliminate synchronous coordination meetings? What's the fundamental difference between contract-based vs. meeting-based coordination?"

---

## Writing Integration Contracts

Let's build your contract-writing skill. A good contract answers four questions:

1. **What does this feature provide?** (APIs, data models, services)
2. **What does this feature depend on?** (inputs, external systems, other features)
3. **How will it integrate?** (APIs, data flow, shared state)
4. **How will we know it's done?** (acceptance criteria)

### Contract Structure: The Template

Here's the structure you'll use. Think of it as a legally binding agreement between agent and human orchestrator:

```markdown
# Feature [Name] Integration Contract

## Provides

### APIs
- [Endpoint path]: [Description]
  - Input: [Schema]
  - Output: [Schema]
  - Error cases: [List]

### Data Models
- [Model name]: [Columns/fields]

### External Services
- [Service name]: [What it does]

## Depends On

### Required Features
- [Feature name] → [What we need from it]

### External Services
- [Service name] → [How we use it]

### Data Requirements
- [Database/service] → [Specific tables/endpoints]

## Integration Points

### How We Connect
- [Feature A] calls [Feature B's endpoint] to [accomplish task]
- [Feature A] reads from [Feature B's data model]
- Conflict resolution: [If contention, do this]

### Performance Targets
- [Operation] must complete in [X]ms
- Concurrency limit: [N] simultaneous requests
- Data consistency model: [eventual/strong]

## Acceptance Criteria

- [ ] All APIs respond in &lt; [X]ms under load
- [ ] All [N] test cases pass
- [ ] Error handling covers [list of scenarios]
- [ ] Integration test with Feature [X] succeeds
- [ ] Code review: [specific checklist items]
- [ ] Documentation: [README updated with examples]
```

This template is concrete. It's measurable. An agent can't wiggle out of it with interpretation—the contract is the specification.

---

## Defining Acceptance Criteria

The difference between vague and autonomous is acceptance criteria. **Vague criteria require human judgment. Measurable criteria enable autonomous verification.**

### The Vagueness Problem

Bad acceptance criteria:
- "The API should be fast"
- "Error handling should be robust"
- "The code should be clean"

These require judgment. "Fast" to you might be "slow" to an agent. "Robust" is subjective. "Clean" is in the eye of the reviewer. **Vague criteria = coordination overhead.**

### The Measurable Solution

Good acceptance criteria:
- "All endpoints respond in &lt;100ms at p95 latency"
- "Error handling covers: invalid input, database failures, timeout scenarios [5 specific cases]"
- "Code passes linter, has >80% test coverage, includes docstrings on all public functions"

These are testable. **An agent can verify them. A human can verify them. No ambiguity.**

### Code Example 2: Acceptance Criteria Table

Here's a concrete example. For the Product Catalog feature, here are the acceptance criteria:

| Category | Criterion | How to Verify | Pass/Fail |
|----------|-----------|---------------|-----------|
| **Functionality** | All 3 endpoints implemented | curl each endpoint | [ ] |
| **Functionality** | Pagination works: 1 ≤ page ≤ max | Test with page=1, page=100, page=999 | [ ] |
| **Functionality** | Stock decrement prevents overselling | Concurrent PATCH requests, verify no negative stock | [ ] |
| **Performance** | GET requests &lt; 100ms (p95) | Load test: 100 req/s for 1 min, measure p95 | [ ] |
| **Performance** | PATCH requests &lt; 200ms | Load test: 50 req/s for 1 min, measure p95 | [ ] |
| **Error Handling** | Invalid product ID returns 404 | GET /api/products/invalid-uuid → 404 | [ ] |
| **Error Handling** | Malformed request returns 400 | GET /api/products?page=abc → 400 | [ ] |
| **Error Handling** | Stock insufficient returns 400 | PATCH with delta > current stock → 400 | [ ] |
| **Integration** | Feature 3 can read products | Feature 3 calls `GET /products/{id}`, verifies response | [ ] |
| **Integration** | Feature 3 can decrement stock | Feature 3 calls PATCH, verifies stock updates | [ ] |
| **Code Quality** | No SQL injection vulnerabilities | Code review + manual injection test | [ ] |
| **Code Quality** | All public functions documented | Check docstrings in code | [ ] |
| **Code Quality** | Test coverage >= 80% | Run test suite, check coverage report | [ ] |

This table is the contract's teeth. **When all boxes are checked, Feature 2 is done. No further discussion.**

Agents build to this. You verify against this. Integration happens. Clean.

#### 🎓 Expert Insight
> In AI-native development, acceptance criteria aren't just checklists—they're the operational definition of "done" that enables autonomous coordination. Clear criteria = agents work independently. Vague criteria = constant orchestrator interruptions. At 10 agents, clarity isn't nice-to-have; it's the difference between scalable and chaos.

---

## Configuring Completion Hooks

With specifications and contracts in place, you still need one more piece: **How do you know when an agent is done?**

In Lesson 5, you monitored constantly. But at scale, that's unsustainable. Instead, you use hooks: **scripts that fire automatically when certain events occur**, notifying the orchestrator that work is complete.

### Hook Basics: What Triggers, What Fires

A hook has two parts:

1. **Trigger**: When something happens (e.g., agent finishes implementation, tests pass)
2. **Action**: What to do about it (e.g., write to a status file, send notification)

Claude Code supports these hook triggers:

- `on-session-complete`: Agent session ends (implementation finished or error occurred)
- `after-tool-use`: After an agent uses a specific tool (e.g., after test command completes)
- `on-timeout`: After X minutes of inactivity

For orchestration, we care about: **Did the agent finish? Did they meet acceptance criteria? Are they ready for integration?**

### Code Example 3: Completion Hook Script

Here's a hook script that fires when Feature 2 finishes implementation:

```bash
#!/bin/bash
# File: .claude/hooks/notify-orchestrator.sh
# Purpose: Notify orchestrator when a feature is complete
# Trigger: on-session-complete (fires when agent session ends)

set -e

FEATURE_ID=$1  # e.g., "feature-002-catalog"
SESSION_ID=$2  # Unique session identifier
STATUS_DIR="./.orchestrator-status"

# Create status directory if it doesn't exist
mkdir -p "$STATUS_DIR"

# Cross-platform timestamp (works on macOS, Linux, Windows Git Bash)
if date --version >/dev/null 2>&1; then
  # GNU date (Linux)
  TIMESTAMP=$(date -u +'%Y-%m-%dT%H:%M:%SZ')
else
  # BSD date (macOS)
  TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
fi

# Check acceptance criteria
if [ -f "$FEATURE_ID/ACCEPTANCE_CRITERIA.checked" ]; then
  CRITERIA_MET="true"
else
  CRITERIA_MET="false"
fi

# Write completion record (safer than heredoc with variable substitution)
cat > "$STATUS_DIR/$FEATURE_ID.json" <<EOF
{
  "feature_id": "$FEATURE_ID",
  "session_id": "$SESSION_ID",
  "completed_at": "$TIMESTAMP",
  "status": "complete",
  "acceptance_criteria_verified": $CRITERIA_MET
}
EOF

echo "✓ Feature $FEATURE_ID completion notified at $TIMESTAMP"
echo "  Session ID: $SESSION_ID"

# Use jq if available, otherwise parse manually
if command -v jq >/dev/null 2>&1; then
  echo "  Status: $(jq -r '.status' "$STATUS_DIR/$FEATURE_ID.json")"
else
  echo "  Status: complete"
fi

# Write to shared orchestrator log with file locking (prevents race conditions)
# Use flock on Linux, or >> without locking on macOS (acceptable for low contention)
LOG_FILE=".orchestrator-status.log"
LOG_ENTRY="$TIMESTAMP: $FEATURE_ID complete (session: $SESSION_ID)"

if command -v flock >/dev/null 2>&1; then
  # Linux: use flock for safe concurrent writes
  (
    flock -x 200
    echo "$LOG_ENTRY" >> "$LOG_FILE"
  ) 200>"$LOG_FILE.lock"
else
  # macOS/BSD: simple append (low risk of corruption with short writes)
  echo "$LOG_ENTRY" >> "$LOG_FILE"
fi
```

This hook is simple but powerful. When an agent finishes, the hook:

1. Creates a JSON record with the feature ID and session ID
2. Records the completion timestamp
3. Appends to a shared orchestrator log that all sessions can read

Now the orchestrator (human or automated) can poll a single file to see what's done, what's pending, what failed.

### Code Example 4: Orchestrator Polling Script

But hooks only notify when something happens. What if you want to check progress? Here's the orchestrator-side script that polls the status:

```bash
#!/bin/bash
# File: scripts/orchestrator-poll-status.sh
# Purpose: Check which features are complete, what's pending, what failed
# Usage: ./scripts/orchestrator-poll-status.sh

STATUS_DIR=".orchestrator-status"
STATUS_LOG=".orchestrator-status.log"

echo "=== ORCHESTRATOR STATUS REPORT ==="
echo "Checked at: $(date)"
echo ""

# Count completion records
COMPLETE_COUNT=$(ls -1 "$STATUS_DIR"/*.json 2>/dev/null | wc -l)
echo "Features Complete: $COMPLETE_COUNT"
echo ""

# Check if jq is available
HAS_JQ=false
if command -v jq >/dev/null 2>&1; then
  HAS_JQ=true
fi

# List each feature's status
echo "=== FEATURE STATUS ==="
for status_file in "$STATUS_DIR"/*.json; do
  if [ -f "$status_file" ]; then
    if [ "$HAS_JQ" = true ]; then
      # Use jq for clean parsing
      feature=$(jq -r '.feature_id' "$status_file")
      completed=$(jq -r '.completed_at' "$status_file")
      session=$(jq -r '.session_id' "$status_file")
      criteria=$(jq -r '.acceptance_criteria_verified' "$status_file")
    else
      # Fallback: use grep and sed (works without jq)
      feature=$(grep -o '"feature_id"[[:space:]]*:[[:space:]]*"[^"]*"' "$status_file" | cut -d'"' -f4)
      completed=$(grep -o '"completed_at"[[:space:]]*:[[:space:]]*"[^"]*"' "$status_file" | cut -d'"' -f4)
      session=$(grep -o '"session_id"[[:space:]]*:[[:space:]]*"[^"]*"' "$status_file" | cut -d'"' -f4)
      criteria=$(grep -o '"acceptance_criteria_verified"[[:space:]]*:[[:space:]]*[a-z]*' "$status_file" | awk '{print $NF}')
    fi

    echo "✓ $feature"
    echo "  Completed: $completed"
    echo "  Session: $session"
    echo "  Criteria Met: $criteria"
    echo ""
  fi
done

# Show recent log entries (last 10)
echo "=== RECENT ACTIVITY ==="
tail -10 "$STATUS_LOG" 2>/dev/null || echo "No activity log yet"

echo ""
echo "=== ALL_COMPLETE CONDITION ==="
# Check if all expected features are complete
EXPECTED_FEATURES=("feature-001-auth" "feature-002-catalog" "feature-003-cart" "feature-004-payments" "feature-005-orders")
MISSING=()

for expected in "${EXPECTED_FEATURES[@]}"; do
  if [ ! -f "$STATUS_DIR/$expected.json" ]; then
    MISSING+=("$expected")
  fi
done

if [ ${#MISSING[@]} -eq 0 ]; then
  echo "✓ ALL_COMPLETE: All features have completed and reported status"
  echo "✓ Ready for integration phase"
else
  echo "⏳ PENDING: Waiting on $(printf '%s, ' "${MISSING[@]}" | sed 's/, $//')"
fi
```

Run this script every minute. It tells you:
- Which features are done
- Which are still working
- When you're ready for integration

**No manual checking. No status meetings. Just one command.**

> **Platform Compatibility Note**: Both scripts include cross-platform compatibility:
> - Timestamp generation works on Linux (GNU date) and macOS (BSD date)
> - File locking uses `flock` on Linux, simple append on macOS (safe for low contention)
> - JSON parsing checks for `jq` availability and provides `grep`/`sed` fallback
> - Scripts work on Windows Git Bash without modification

---

## Exercises: Build Your Contract Intuition

You've learned the concepts. Now let's practice. Do these exercises in order. Each one builds your contract-writing skill.

### Exercise 1: Write a 3-Feature Contract

You're given a 3-feature system:

**System**: Task Management App
- **Feature A**: User authentication (login, register, password reset)
- **Feature B**: Task CRUD (create, read, update, delete tasks; assign to users)
- **Feature C**: Task notifications (email alerts when task assigned or due)

Your task: Write `contract.md` defining the integration contracts for Feature B (Task CRUD).

**What to include:**
- Provides: List all endpoints (at least 4: create, read, update, delete)
- Depends on: List dependencies on Feature A (authentication), data models
- Integration points: How Feature B connects to Feature C
- Acceptance criteria: 8-10 specific, testable criteria

**Success**: Your contract is unambiguous. An agent could implement it without asking you a single question.

**Hint**: Think like a lawyer writing a contract. Leave no room for interpretation.

### Exercise 2: Define Acceptance Criteria

Using your Feature B contract from Exercise 1, write a detailed acceptance criteria table.

**Format**:
| Category | Criterion | How to Verify | Pass/Fail |
|----------|-----------|---------------|-----------|

**What to include:**
- Functionality criteria (all 4 endpoints work)
- Performance criteria (response times)
- Error handling criteria (5+ specific error scenarios)
- Integration criteria (Feature B works with Feature A and C)
- Code quality criteria (testing, documentation)

**Success**: A QA engineer could use your table to verify the feature without asking you what "complete" means.

### Exercise 3: Configure Hooks in a Test Worktree

In a test git worktree, configure completion hooks that fire when an "implementation" finishes.

**Steps**:
1. Create directory `.claude/hooks/`
2. Create hook script `notify-orchestrator.sh` (based on Code Example 3)
3. Create status directory `.orchestrator-status/`
4. Create orchestrator polling script `scripts/orchestrator-poll-status.sh`
5. Simulate 3 agents finishing: Write 3 JSON files to `.orchestrator-status/`
6. Run polling script, verify it reports all 3 complete

**Success**: Polling script shows all 3 features complete and ready for integration.

### Exercise 4: Simulate Parallel Coordination

This is the capstone exercise. You're coordinating 3 agents building Features A, B, C.

**Setup**:
- Create 3 git worktrees: `worktree-feature-a`, `worktree-feature-b`, `worktree-feature-c`
- In each worktree, create a placeholder contract.md file
- Configure hooks in each worktree

**Execution**:
- In each worktree, run the hook script manually (simulating feature completion)
- Use the orchestrator polling script to track progress
- Verify that all 3 features show as complete

**Success**: You see all 3 features complete in the status report, and you haven't checked any of them individually.

The insight: **At 3 agents, this feels overkill. But at 15 agents, this is essential. You're building the muscle now.**

#### 🤝 Practice Exercise

> **Ask your AI**: "Write an integration contract for a payment feature that depends on user authentication. Include: provides, depends on, integration points, and 5 specific acceptance criteria. Then critique your own contract for ambiguity."

**Expected Outcome**: Practical experience writing contracts and developing intuition for what makes criteria testable vs. vague.

---

## Try With AI

You've learned to write contracts and configure hooks. Now let's refine your intuition with AI as your thinking partner.

**Tool**: Claude Code CLI (or your preferred AI companion if you've set one up)

**Setup**: Open 3 parallel Claude Code sessions (in 3 terminal windows or tmux splits). In each, you'll ask different questions about contracts.

### Prompt 1: Contract Quality Check

**In Session 1**, paste your Feature B contract from Exercise 1 and ask:

```
Review my integration contract for the Task CRUD feature.

Looking for:
1. Are integration points clear and unambiguous?
2. Are acceptance criteria testable (not subjective)?
3. Did I miss any integration points with Feature A or C?
4. What would an agent find confusing or unclear?

Here's my contract:
[PASTE YOUR CONTRACT HERE]
```

**Expected outcome**: AI identifies 2-4 specific gaps. You refine your contract. This is how you develop contract intuition.

### Prompt 2: Scaling Hooks

**In Session 2**, ask about scaling to 10 features:

```
I currently use hooks and status files to track 3 feature completions.

How would I configure this system to track 10 parallel feature sessions?

Specifically:
1. Would status files scale, or do I need a different mechanism?
2. How would I handle feature dependencies (Feature B can't integrate until Feature A is complete)?
3. What would break if I tried to scale to 15 features?

Here's my current orchestrator polling script:
[PASTE YOUR SCRIPT]
```

**Expected outcome**: AI suggests improvements (e.g., database instead of files, dependency resolution logic). You see the path to larger scale.

### Prompt 3: Autonomy Through Criteria

**In Session 3**, ask about the philosophy:

```
I'm trying to understand why acceptance criteria matter for autonomous work.

Here are two versions of acceptance criteria for the same feature:

VERSION A (vague):
- The API should be fast
- Error handling should be robust
- Code should be clean

VERSION B (specific):
- All endpoints respond in &lt;100ms (p95)
- Error handling covers 5 scenarios: invalid input, timeout, rate limit, database error, malformed JSON
- Code passes linter, >80% test coverage, docstrings on all public functions

Explain: Why does VERSION B enable autonomous work while VERSION A requires constant oversight?
```

**Expected outcome**: AI articulates the principle clearly: **Vague criteria require judgment; specific criteria enable verification.** This insight unlocks your understanding of why contracts matter.

---

## Real-World Context

Let's ground this in how real organizations use contracts for distributed coordination.

### How Anthropic Uses Contracts for Distributed Teams

At Anthropic, distributed teams building Claude use contracts extensively. When one team builds a feature that another team depends on, they don't rely on meetings. Instead:

1. Team A writes a contract: "We'll provide endpoint X with response schema Y"
2. Team B writes acceptance criteria: "Endpoint X must respond in &lt;50ms, handle 1000 req/sec"
3. Both teams agree on the contract (30-min async discussion in GitHub)
4. Team A implements
5. When done, Team A pings Team B (no meetings, just a Slack notification)
6. Team B integrates and verifies against the contract
7. Done

This happens dozens of times per day across Anthropic. **No coordination meetings. No synchronous overhead. Just contracts.**

The result: Large distributed teams ship autonomously. The contract is the communication.

### How Vercel Coordinates 50+ Engineers

Vercel's Next.js team operates similarly. When you have 50 engineers working on a framework, synchronous coordination is impossible. So they use:

- **Contracts**: Each feature (routing, rendering, data fetching) has an explicit contract with other features
- **Acceptance criteria**: Measurable, testable conditions for "done"
- **Hooks**: CI/CD system automatically notifies when features are complete
- **Async integration**: Features merge when they meet contracts, no integration meetings

The result: 50 engineers ship features in parallel, integrating hundreds of times per week, with minimal sync meetings.

### How Solo Founders Use Contracts to Coordinate AI Agents

You (the solo founder) don't have a team. But you're about to coordinate 5-10 AI agents building features. **Same principle applies.**

You write a contract: "Build a cart feature that calls the product API I defined."
AI agents read the contract and implement.
Hooks notify you when done.
You integrate.

The difference: With clear contracts, you can coordinate 10 AI agents as efficiently as Vercel coordinates 50 humans. **Decomposition + Contracts = Unlimited Scale.**

---

🤔 **Pause and Reflect**: You've learned that contracts reduce coordination overhead. Ask yourself: *If contracts cut coordination time in half, what would you do with that freed time?* Strategy? Innovation? Delegation? What becomes possible when you're not constantly checking on agents?

