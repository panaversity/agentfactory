---
title: "SpecKit-Orchestrated Autonomous Execution"
chapter: 32
lesson: 6
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "SpecKit-Based Orchestration"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use SpecKit Plus commands to orchestrate multi-session feature workflows, understanding how specifications act as coordination mechanisms"

  - name: "Contract Generation with AI"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student can co-create contract.md through /sp.specify, defining integration requirements between parallelizable features"

  - name: "Multi-Session Coordination"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can spawn 5-10 autonomous sessions via hooks and completion notifications, understanding orchestration without manual script management"

  - name: "Strategic Checkpoint Review"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can review orchestrated work strategically, validating against contracts and deciding merge sequence without micromanaging execution"

learning_objectives:
  - objective: "Use SpecKit Plus commands to generate contract specifications that coordinate multi-session autonomous execution"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Generate contract.md via /sp.specify for provided multi-feature system"

  - objective: "Coordinate 5-10 autonomous AI agent sessions through well-defined specifications and completion hooks"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Spawn and monitor multiple autonomous sessions, validate against contracts"

  - objective: "Experience creative independence: the shift from managing execution to strategic oversight"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Reflection on how specification-driven orchestration enables autonomous work"

cognitive_load:
  new_concepts: 8
  assessment: "8 new concepts (SpecKit orchestration, contract generation, autonomous spawning, hooks, completion notifications, strategic review, integration validation, creative independence) within B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Extend to 10 features; analyze scaling requirements; design monitoring dashboard for 7-9 agent coordination"
  remedial_for_struggling: "Start with 3-feature system; emphasize contract.md clarity; use manual terminal spawning before tmux scripting"

# Generation metadata
generated_by: "Claude Code (content-implementer)"
source_spec: "specs/002-chapter-32-redesign/spec.md"
created: "2025-11-06"
last_modified: "2025-11-06"
git_author: "Claude Code"
workflow: "/sp.implement (Lesson 6: Second Climax)"
version: "1.0.0"
---

# Lesson 6: SpecKit-Orchestrated Autonomous Execution

**SECOND CLIMAX: Creative Independence Through Specification-Driven Orchestration**

You've learned to decompose systems into parallelizable units (Lessons 1-4) and automated their coordination through validation and shared context (Lessons 5-6). Now comes the payoff: **watching 5-10 autonomous AI agent sessions execute independently while you focus on strategy.**

In Lesson 5, you experienced *contracts* as the bridge between human intention and autonomous work. This lesson shows you how **SpecKit Plus itself becomes your orchestrator**—not through complex bash scripts, but through the same tools you've been using all along: `/sp.specify`, `/sp.plan`, `/sp.tasks`, and simple completion hooks.

The journey unfolds in five phases:

1. **Human designs (30 minutes)**: Decompose your system, define clear contracts
2. **SpecKit plans (10 minutes)**: Generate tasks.md with parallelizable work units
3. **Human spawns (5 minutes)**: Open 5-10 terminals, start Claude sessions reading the contracts
4. **AI agents execute (2-3 hours)**: Sessions run independently, writing completion status
5. **Human reviews (30 minutes)**: Validate against contracts, decide merge strategy

This is not simulation. You walk away from your computer. The agents work. They notify you when done. This is creative independence.

---

## The SpecKit Orchestrator Paradigm: Not Bash Scripts, Just Specifications

Before diving into execution, let's clarify what makes this approach different from traditional scripting orchestration.

### What Changed from Lesson 5

In Lesson 5, you used contracts (specifications) to define what each agent MUST achieve. Those contracts lived in spec.md and plan.md. Helpful, but you still managed agents manually across 5 terminals.

**Now, in Lesson 6**, contracts become the *entire coordination layer*. Here's what shifts:

**Old (Manual Coordination)**:
- Human monitors 5 terminals
- Human intervenes if agents drift
- Human manually checks progress every 10 minutes
- Coordination is synchronous (human watching)

**New (Specification-Driven Orchestration)**:
- Specifications define ALL integration requirements upfront
- Agents read contracts, execute autonomously
- Completion hooks notify human asynchronously
- Human focuses on strategic decisions, not execution monitoring

**The Key Insight**: When specifications are clear enough, agents don't need human supervision during execution. They need **strategic oversight afterward**—did they deliver what was promised? Should we merge? What's next?

### Why SpecKit Plus is the Orchestrator

You might expect: "Shouldn't we use a complex orchestration script (bash or Python) to coordinate agents?"

The answer: **No, use SpecKit Plus itself.**

Here's why:

**SpecKit Plus as orchestrator means**:
- `/sp.specify` generates contract.md (what agents MUST achieve)
- `/sp.plan` breaks work into independent features
- `/sp.tasks` creates parallelizable task lists
- Completion hooks notify when work is done
- Human reviews results against contracts

**This is orchestration**: Coordinating multiple agents through specifications. No bash JSON parsing. No session ID management. No headless mode complexity. Just clear specifications and completion hooks.

### What SpecKit Plus Provides vs. What You Do

**SpecKit Plus provides**:
- Specification generation (`/sp.specify`, `/sp.plan`, `/sp.tasks`)
- Execution within a single session (`/sp.implement`)
- Standardized workflows and validation

**You (the orchestrator) provide**:
- Decomposition thinking (breaking system into 5 features)
- Session spawning (opening 5 terminals, starting Claude in each)
- Strategic review (validation against contracts)

**The "orchestration"**: You coordinate multiple Claude Code sessions through shared specifications and completion hooks. SpecKit Plus ensures each session knows what to build; you ensure they work together.

#### 💬 AI Colearning Prompt
> "How does SpecKit Plus orchestration differ from writing bash scripts that spawn 5 Claude sessions? What's the fundamental paradigm shift?"

---

## Phase 1: Generate Contract with SpecKit Plus (20 minutes)

Your orchestration journey begins with **contract generation**. A contract is a specification that defines:

- What each agent MUST build (feature boundaries)
- What data contracts exist (API contracts, data formats)
- What success looks like (acceptance criteria)
- What dependencies exist (if any)

### Understanding Contract Specifications

A contract.md file generated by `/sp.specify` might look like this (simplified example):

```markdown
# Integration Contracts: E-Commerce Platform

## Feature 1: Authentication Service
**Responsibility**: User registration, login, session management
**API Contract**:
- POST /auth/register → { user_id, token }
- POST /auth/login → { user_id, token }
- GET /auth/verify → { is_valid: boolean }

**Data Contract**:
- User schema: { id, email, password_hash, created_at }

**Integration Points**:
- All other features depend on this
- Must be completed before Features 2-5 proceed

## Feature 2: Product Catalog
**Responsibility**: Product CRUD, search, filtering
**API Contract**:
- GET /products → { products: [...] }
- POST /products → { product_id }
- GET /products/:id → { product: {...} }

**Data Contract**:
- Product schema: { id, name, price, inventory }

**Integration Points**:
- Depends on Feature 1 (auth)
- Used by Features 3 and 4
```

This contract answers the critical question: **If I build Feature 2 in isolation, will it integrate cleanly with Feature 4?**

Good contracts = clean integration. Bad contracts = merge conflicts and rework.

### Your Task: Generate a Contract with /sp.specify

You've used `/sp.specify` in Lesson 1, but now with a **contract-specific focus**.

**Specification Reference** (Reference to the Spec-Driven Development workflow):
- When: After decomposing your system into 5 independent features
- Input: Detailed feature descriptions with integration points identified
- Output: spec.md + contract.md

**Example Prompt to Claude Code**:

```
/sp.specify

Build an e-commerce platform with 5 features: authentication,
product catalog, shopping cart, payment processing, and order history.

**Requirements for this spec**:
- Define clear API contracts between features
- Identify dependencies (which features depend on which?)
- Show data schemas for each feature
- For each feature, describe: responsibility, API surface, data contracts, integration points

**Validation**: After /sp.specify completes, I'll review contract.md and
verify all integration points are clear.
```

**What Happens**:
1. Claude Code runs `/sp.specify`
2. Generates spec.md (vision and requirements)
3. Generates contract.md (integration contracts between agents)
4. You review and refine contracts until they're crystal clear

**Code Example 1: Contract Generation Specification**

Here's what a well-formed specification looks like (this is what you'd pass to `/sp.specify`):

```markdown
**Feature 1: Authentication Service**
- Provides user registration, login, session validation
- API exports: POST /register, POST /login, GET /verify
- Data exports: User schema with id, email, verified_email, created_at
- Dependencies: None (all other features depend on this)
- Success criteria: Login latency < 200ms, 99.9% uptime

**Feature 2: Product Catalog**
- Provides product listing, search, filtering
- API imports: User auth from Feature 1
- API exports: GET /products, GET /products/:id, POST /products
- Data exports: Product schema with id, name, price, inventory
- Dependencies: Feature 1 (requires auth before accessing products)
- Integration: Other features query products via this API

**Feature 3: Shopping Cart**
- Provides cart CRUD operations
- API imports: User auth from Feature 1, Product queries from Feature 2
- API exports: GET /cart, POST /cart/items, DELETE /cart/items
- Data exports: Cart schema with id, user_id, items[]
- Dependencies: Features 1 and 2
- Integration: Cart items reference Product IDs from Feature 2

**Feature 4: Payment Processing**
- Provides payment validation and order confirmation
- API imports: Cart data from Feature 3, User auth from Feature 1
- API exports: POST /payments, GET /payment-status
- Data exports: Transaction schema with id, user_id, amount, status
- Dependencies: Features 1 and 3
- Integration: Validates cart contents before charging

**Feature 5: Order History**
- Provides order retrieval and status tracking
- API imports: User auth from Feature 1, Transaction data from Feature 4
- API exports: GET /orders, GET /orders/:id
- Data exports: Order schema with id, user_id, items[], total, created_at
- Dependencies: Features 1 and 4
- Integration: Retrieves previous transactions via Feature 4
```

**Specification reference**: This shows what `/sp.specify` needs to generate useful contracts. Notice:
- Each feature's responsibility is crystal clear
- Integration points are explicit (who depends on whom?)
- API contracts show exactly what data flows between features

**Validation Steps**:
1. After `/sp.specify` completes, review contract.md
2. For each feature, verify:
   - [ ] Responsibility is unambiguous
   - [ ] API contracts are specific (not "integration with payment" but specific like: `POST /payments` with body containing cart_id and user_id)
   - [ ] Dependencies are listed (Feature 3 depends on 1 and 2)
   - [ ] No circular dependencies (Feature A depends on B, and B depends on A)
3. If contracts are unclear, refine and regenerate

---

## Phase 2: Plan Multi-Feature System with SpecKit (15 minutes)

With contracts defined, SpecKit Plus generates your orchestration plan: **what tasks can run in parallel, and which must wait for dependencies?**

### Understanding the Orchestration Plan

Run `/sp.plan` on your contract.md. The plan will include:

```markdown
# Multi-Feature Orchestration Plan

## Dependency Graph
Feature 1 (Authentication) → no dependencies
Feature 2 (Products) → depends on Feature 1
Feature 3 (Cart) → depends on Features 1, 2
Feature 4 (Payments) → depends on Features 1, 3
Feature 5 (Orders) → depends on Features 1, 4

## Execution Phases
Phase 1 (Parallel): Feature 1 - Authentication
  - Time: ~1 hour
  - Success condition: Tests passing, API endpoints working

Phase 2 (Parallel after Phase 1): Features 2 - Products
  - Time: ~1-1.5 hours
  - Success condition: Product CRUD working, integrates with Feature 1

Phase 3 (Parallel after Phase 2): Feature 3 - Cart
  - Time: ~1 hour
  - Success condition: Cart operations working, references Products

Phase 4 (Parallel after Phase 3): Feature 4 - Payments
  - Time: ~1.5 hours (complex with external API)
  - Success condition: Payment processing validated, no hardcoded secrets

Phase 5 (Parallel after Phase 4): Feature 5 - Orders
  - Time: ~45 minutes
  - Success condition: Order history retrieves transactions correctly
```

This plan tells you: **Which agents can work simultaneously, and which must wait for prerequisites?**

In this example:
- Feature 1 must complete first (everything depends on it)
- Features 2 can start as soon as Feature 1 is done
- Feature 3 can start as soon as Features 1 and 2 are done
- And so on...

### Run /sp.tasks to Generate Parallelizable Work Units

Now run `/sp.tasks` to break each feature into atomic work units:

**Code Example 2: Generated tasks.md (Simplified View)**

The tasks.md file shows parallelizable tasks:

```markdown
# Orchestration Tasks: E-Commerce Platform

## Phase 1: Feature 1 - Authentication (Parallel)
### Session 1: Authentication Service
- Task 1.1: Set up user schema and database
- Task 1.2: Implement POST /register endpoint
- Task 1.3: Implement POST /login endpoint with JWT generation
- Task 1.4: Implement GET /verify endpoint
- Task 1.5: Write unit tests for auth service (100% coverage)
- Task 1.6: Integration test with mock database
- Task 1.7: Error handling for edge cases (invalid email, weak passwords)
- Estimated time: 60 minutes

## Phase 2: Feature 2 - Product Catalog (After Phase 1 Complete)
### Session 2: Product Catalog
- Task 2.1: Set up product schema, verify integration with Feature 1
- Task 2.2: Implement GET /products (with pagination)
- Task 2.3: Implement GET /products/:id
- Task 2.4: Implement POST /products (with auth requirement from Feature 1)
- Task 2.5: Add search and filtering logic
- Task 2.6: Write tests verifying auth integration
- Estimated time: 75 minutes

## Phase 3: Feature 3 - Shopping Cart (After Phases 1-2 Complete)
### Session 3: Shopping Cart
- Task 3.1: Set up cart schema, verify integration points
- Task 3.2: Implement GET /cart/:user_id
- Task 3.3: Implement POST /cart/items (validates product exists via Feature 2)
- Task 3.4: Implement DELETE /cart/items
- Task 3.5: Implement cart total calculation
- Task 3.6: Write integration tests with Features 1 and 2
- Estimated time: 60 minutes

## Phase 4: Feature 4 - Payment Processing (After Phases 1-3 Complete)
### Session 4: Payment Processing
- Task 4.1: Set up transaction schema
- Task 4.2: Implement POST /payments with cart validation
- Task 4.3: Integrate with payment provider API (with env var secrets, never hardcoded)
- Task 4.4: Implement error handling for failed payments
- Task 4.5: Write tests for payment validation (without hitting real API)
- Task 4.6: Integration test with Feature 3 cart
- Estimated time: 90 minutes

## Phase 5: Feature 5 - Order History (After Phases 1, 4 Complete)
### Session 5: Order History
- Task 5.1: Set up order schema
- Task 5.2: Implement GET /orders/:user_id
- Task 5.3: Implement GET /orders/:user_id/:order_id
- Task 5.4: Query transaction history from Feature 4
- Task 5.5: Write tests verifying data consistency
- Task 5.6: Integration test end-to-end
- Estimated time: 45 minutes
```

**Key Insight**: This tasks.md file is **readable by humans and agents alike**. When you spawn Session 2 (Products feature), that agent reads tasks.md and sees: "I'm Task 2.1 through 2.6. Feature 1 must be complete first. Here are my specific responsibilities."

---

## Phase 3: Spawn Autonomous Sessions (20 minutes)

Now for the moment you've been building toward: **spawning agents and walking away.**

### Setting Up Completion Hooks

Before spawning sessions, configure completion hooks so agents can notify you when done:

**Specification Reference** (Reference to Contract.md):
- Each session will write completion status to a hook
- Hook format: Simple log file that tracks which tasks completed
- Trigger: When ALL tasks complete, notify human

**Code Example 3: Completion Hook Configuration**

Create a `.hooks` directory in your project:

```bash
mkdir -p .hooks
```

Create `.hooks/task-completion.sh`:

```bash
#!/bin/bash
# This script is called when a session completes all tasks
# Arguments: $1 = session_name, $2 = status (success/failure)

SESSION_NAME=$1
STATUS=$2
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

# Write to completion log
cat >> orchestrator-status.log << EOF
[$TIMESTAMP] $SESSION_NAME: $STATUS
EOF

# Check if all sessions complete
TOTAL_SESSIONS=5
COMPLETED=$(grep -c "success\|failure" orchestrator-status.log 2>/dev/null || echo 0)

if [ $COMPLETED -eq $TOTAL_SESSIONS ]; then
    echo "🎉 All $TOTAL_SESSIONS sessions complete!"
    # Optional: Send notification (email, Slack, etc.)
fi
```

**Make it executable**:

```bash
chmod +x .hooks/task-completion.sh
```

### Spawning Sessions: The Manual Approach

In practice, you open 5 terminals and do this:

**Terminal 1 (Main)**:
```bash
cd /path/to/project
claude  # Start your orchestrator session here
```

**Terminal 2 (Session 1 - Auth)**:
```bash
cd /path/to/project/worktree-001-auth
claude  # This session reads contract.md and tasks.md, executes Feature 1
```

**Terminal 3 (Session 2 - Products)**:
```bash
cd /path/to/project/worktree-002-products
# Wait for Terminal 2 (Feature 1) to complete before starting this
claude
```

**Terminal 4 (Session 3 - Cart)**:
```bash
cd /path/to/project/worktree-003-cart
# Wait for Features 1 and 2 to complete
claude
```

**Terminal 5 (Session 4 - Payments)**:
```bash
cd /path/to/project/worktree-004-payments
# Wait for Features 1 and 3 to complete
claude
```

**Terminal 6 (Session 5 - Orders)**:
```bash
cd /path/to/project/worktree-005-orders
# Wait for Features 1 and 4 to complete
claude
```

Each session opens Claude Code in its worktree. The agent reads:
1. `/sp.specify` output (what's the vision?)
2. `contract.md` (what are my integration contracts?)
3. `tasks.md` (what are my specific tasks?)
4. And executes the feature implementation

### Spawning Sessions

Open 5-10 terminal windows or tabs, one for each feature.

> **Tip**: Use whatever terminal tool you prefer—separate windows, tabs, tmux, VS Code's integrated terminal, or any other multiplexer. The goal is to run multiple Claude sessions simultaneously.

**For each worktree, navigate and start Claude:**

**Terminal 1:**
```bash
cd worktree-001-auth
claude
```

**Terminal 2:**
```bash
cd worktree-002-products
claude
```

**Terminal 3:**
```bash
cd worktree-003-cart
claude
```

**Terminal 4:**
```bash
cd worktree-004-payments
claude
```

**Terminal 5:**
```bash
cd worktree-005-orders
claude
```

Each Claude session will read the contract.md and tasks.md in its worktree and begin autonomous execution.

> **Important**: If features have dependencies (e.g., Feature 2 depends on Feature 1), start dependent features AFTER their prerequisites complete. Check contract.md for the dependency order.

#### 🎓 Expert Insight
> In AI-native development, your job transforms from "telling agents what to do" to "ensuring agents know what to build." SpecKit Plus specifications enable autonomous execution—you define contracts once, agents execute independently. At 5 features, this saves hours. At 15 features, this is the only scalable approach.

---

## Phase 4: Autonomous Execution + Hooks (10 minutes)

This is when you **walk away from your computer**.

### What Happens While You're Gone

Each Claude session:

1. **Reads contract.md** - "What are my integration requirements?"
2. **Reads tasks.md** - "What specific tasks do I own?"
3. **Executes tasks** - Implements feature, writes code, runs tests
4. **Reports completion** - Writes to completion hook

**Example of Session 2 (Products) executing**:

```
User: "You're implementing the Product Catalog feature.
Here's your contract (from contract.md).
Here are your tasks (from tasks.md).
Proceed with implementation."

Claude:
✓ Task 2.1: Set up product schema
✓ Task 2.2: Implement GET /products with pagination
✓ Task 2.3: Implement GET /products/:id
✓ Task 2.4: Implement POST /products with auth requirement
✓ Task 2.5: Add search and filtering
✓ Task 2.6: Write integration tests

[After 75 minutes of work]

Completed all 6 tasks. Running: .hooks/task-completion.sh products success
```

The `.hooks/task-completion.sh` script writes to `orchestrator-status.log`:

```
[2025-11-06 14:30:15] Session 1 (auth): success
[2025-11-06 15:45:20] Session 2 (products): success
[2025-11-06 16:15:45] Session 3 (cart): success
[2025-11-06 17:50:30] Session 4 (payments): success
[2025-11-06 18:20:10] Session 5 (orders): success
🎉 All 5 sessions complete!
```

---

## Phase 5: Strategic Review (5 minutes)

When the completion hook notifies you: "All 5 sessions complete!" — you return to your computer and perform **strategic oversight**, not micromanagement.

Your questions now are:
- Did agents deliver what contracts promised?
- Are there any integration issues?
- What's the merge strategy?
- What's next?

**This is creative independence**: You delegated execution. You focus on strategy.

### Decision Framework

1. **Quick Validation** (5 min):
   - Review orchestrator-status.log (all sessions successful?)
   - Spot-check integration tests (are they passing?)
   - Scan contract checklist above (major gaps?)

2. **Merge Strategy** (10 min):
   - If Feature 1 (auth) is clean, merge to main first
   - Then Feature 2 (products), merge to main
   - Continue through dependency chain
   - If conflicts appear, treat as learning: what decomposition lesson do they teach?

3. **Next Decision**:
   - Deploy? (covered in Parts 10-11)
   - Add more features? (repeat Lesson 7 workflow)
   - Iterate on existing features? (specs from orchestration inform refinement)

---

## Exercises: Try This Yourself

### Exercise 1: Generate Contracts for Your System

Using the approach in Phase 1:

1. Choose a system (e-commerce, task manager, content platform)
2. Decompose it into 5 independent features
3. Run `/sp.specify` with contract-specific prompts
4. Review contract.md and refine until crystal clear
5. Identify dependencies (Feature X depends on Feature Y)

### Exercise 2: Analyze tasks.md for Parallelization

After running `/sp.tasks`:

1. Review the task list
2. Mark which tasks can run in parallel (no dependencies)
3. Identify which tasks have prerequisites
4. Create a dependency graph (visual or text)
5. Estimate total execution time if all parallelizable tasks run simultaneously

### Exercise 3: Configure and Test Hooks

1. Create `.hooks/task-completion.sh` (code example 4)
2. Create dummy worktrees and test the completion hook
3. Simulate an agent completing Feature 1: manually call `.hooks/task-completion.sh auth success`
4. Verify orchestrator-status.log records the completion
5. Test with 3 simulated sessions (Feature 1 auth, Feature 2 products, Feature 3 cart)

### Exercise 4: Run Full Orchestration Workflow

1. Use contract.md and tasks.md from Exercise 1
2. Spawn 5 Claude sessions (manually or with tmux script)
3. Provide each with the same contract.md and tasks.md
4. Let them execute for the allotted time
5. Review completion logs
6. Validate against contract checklist (Exercise 5 from Phase 5)
7. Document actual execution time vs estimated time

---

## Key Concepts Summary

**New concepts introduced in Lesson 7**:

1. **SpecKit Orchestrator Paradigm**: Using SpecKit Plus commands (`/sp.specify`, `/sp.plan`, `/sp.tasks`) as your coordination mechanism instead of bash scripts
2. **Contract Generation**: Co-creating contract.md via `/sp.specify` to define integration requirements
3. **Completion Hooks**: Simple scripts that agents call when tasks complete, enabling asynchronous notification
4. **Autonomous Spawning**: Opening multiple Claude sessions that read the same specifications and execute independently
5. **Strategic Review**: Shifting from execution monitoring to strategic validation against contracts
6. **Creative Independence**: The moment when you can delegate execution and focus on strategy
7. **Integration Validation**: Checklist-based approach to verifying agents delivered on their contracts
8. **Scaling Pathway**: Understanding how patterns from 5 agents extend to 7-9 agents

#### 🤝 Practice Exercise

> **Ask your AI**: "Design a 5-feature system decomposition for an e-commerce platform. Generate integration contracts using SpecKit Plus principles. Then create a dependency graph showing execution phases. Explain your orchestration strategy."

**Expected Outcome**: Hands-on practice applying SpecKit orchestration thinking to real system design.

---

## Try With AI

Use Claude Code (your preferred AI companion from earlier chapters) for the following prompts. These are designed to deepen your understanding of orchestration:

### Prompt 1: Contract Refinement

```
Review this contract.md generated by /sp.specify:

[Paste your contract.md or example from Phase 1]

Questions:
1. Are there any circular dependencies? (e.g., Feature A depends on B, B depends on A)
2. Are API contracts specific enough that agents won't misunderstand boundaries?
3. Are there hidden integration points I missed?
4. What would make these contracts even clearer?
```

**Expected outcome**: AI provides feedback on contract clarity, identifies potential issues, suggests specific improvements for how agents will interpret boundaries.

### Prompt 2: Orchestration Scaling Strategy

```
I've successfully orchestrated 5 features using SpecKit Plus and hooks.
Now I want to scale to 10 features.

1. What changes in my orchestration approach at 10 features vs 5?
2. What monitoring/tooling becomes essential at 10+ features that wasn't at 5?
3. How would you modify my completion hook script to handle 10 features?
4. What's the path from my current approach to coordinating 15 agents?
```

**Expected outcome**: AI outlines scaling implications, suggests monitoring enhancements, discusses where manual hooks become insufficient and automation scripts become necessary, validates your mental model of progression.

### Prompt 3: Reflect on Creative Independence

```
I just walked away from my computer for 2 hours while 5 AI agents
executed features independently, coordinated by specifications and completion hooks.

Reflect with me:
1. How is this different from the manual terminal management in Lesson 6?
2. What enabled the shift from "I'm managing execution" to "I'm doing strategic review"?
3. How does this pattern apply if I were coordinating human developers instead of AI?
4. What's the biggest insight about orchestration I've gained?
```

**Expected outcome**: AI helps you articulate the conceptual shift you've experienced, reinforces that specifications are the true coordination mechanism (not tools), and draws parallels to team leadership that transfer beyond AI agent coordination.

**Safety note**: Specifications are powerful coordination tools, but they work only when decomposition is excellent. Poor specifications → autonomous agents create integration chaos. Good specifications → autonomous work integrates cleanly. Always validate contracts before unleashing agents.

