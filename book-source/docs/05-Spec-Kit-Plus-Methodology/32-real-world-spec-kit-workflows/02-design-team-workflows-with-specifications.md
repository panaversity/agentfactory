---
title: "Design Team Workflows With Specifications"
chapter: 32
lesson: 2
duration: "2 hours"
skills:
  - name: "Team Process Design"
    proficiency: "B1"
    category: "Conceptual"
  - name: "Workflow Specification"
    proficiency: "B1"
    category: "Technical"
  - name: "Human-AI Coordination"
    proficiency: "B1"
    category: "Soft"
learning_objectives:
  - "Design team workflows where specifications are the coordination mechanism (B1)"
  - "Map decision points, approvals, and handoffs in specification-driven processes (B1)"
  - "Apply SDD to operational team structures and roles (B1)"
---

# Design Team Workflows With Specifications

## What You're About to Design

In Lesson 1, you watched two companions coordinate through specifications. Now you're going to design what a *team* using specifications looks like. Not a technical design—a workflow design. How do people work together when specifications are the source of truth?

You'll imagine a growing team (from 1 person to 5 people), design the workflows at each stage, and discover professional practices that enable teams to scale specification-driven development.

---

## Part 1: From One Person to a Team

Let's trace how specification-driven development scales.

### Stage 1: Solo Developer (Just You)

**The workflow:**
1. You write a specification
2. You implement from the spec
3. You test against the spec
4. You deploy

**Communication needed**: Zero. You're the only person.

**Time from spec to deployment**: Fast. No coordination overhead.

**Risk**: One person means one perspective. You might miss edge cases or make bad architectural decisions.

### Stage 2: Two People (You + One Engineer)

**What changes:**
- You write the spec
- Engineer A implements from the spec
- Both of you test against the spec
- You deploy together

**New questions arise**:
- "What if Engineer A disagrees with the spec?"
- "How do they ask clarifying questions?"
- "What if they find a bug in the spec during implementation?"

This is where workflow matters.

### Stage 3: Three People (Spec Writer + Two Engineers)

**Now it's more complex:**
- One person owns the specification (you)
- Two engineers implement different features in parallel
- They need to coordinate, but through the spec, not through meetings

**Potential problems**:
- One engineer finishes their feature, but the other isn't done. How do you integrate?
- If one engineer gets stuck, who helps them?
- What if both engineers interpret the spec differently?

**This is where formal workflows become essential.**

### Stage 4: Five People (Growing Team)

**Team structure:**
- Specification lead (writes and maintains specs)
- Three feature developers (each owning a feature)
- One QA engineer (testing against acceptance criteria in the spec)

**New complexity:**
- How do specs change when you discover ambiguities?
- Who decides whether a change to the spec is approved?
- How do feature developers stay aligned?
- What's the process for integrating three features?

---

## Part 2: Design Your Team's Specification Workflow

Let's design the workflows for a team using your grading system as the example.

### Step 1: Ask Your Companion for Workflow Ideas

Ask your AI companion:

> **Prompt**: "I'm designing team workflows for a specification-driven team. Imagine a 5-person team building a grading system: 1 spec lead, 3 developers, 1 QA engineer. All work flows through specifications. What are the key workflow stages? For each stage, tell me: (1) Who participates? (2) What are the inputs and outputs? (3) What decisions need approval? (4) What could go wrong?"

Your companion will likely suggest a workflow like:

1. **Specification Review** (before implementation)
   - Spec lead presents spec to team
   - Developers ask clarifying questions
   - QA engineer reviews acceptance criteria
   - Team votes: spec is clear enough to start? (yes/no)
   - If no: spec lead revises spec, repeat

2. **Implementation** (parallel, independent)
   - Each developer implements their feature from the spec
   - They follow the spec exactly—no deviations without approval
   - They write tests that validate acceptance criteria

3. **Spec Change Requests** (if ambiguities surface)
   - Developer finds something unclear in the spec
   - Developer asks spec lead for clarification
   - Spec lead updates the spec or clarifies in writing
   - Spec lead notifies all other developers of the change
   - Developers adjust their implementation if needed

4. **Integration Testing** (when all features are complete)
   - QA engineer tests all features together
   - QA validates that features work through their defined contracts
   - If integration fails, determine: Is it a feature bug or a spec bug?
   - If spec bug: spec lead updates spec, developer fixes feature
   - If feature bug: developer fixes implementation

5. **Deployment** (together)
   - All features deployed together
   - Rollback plan if something breaks

### Step 2: Design Your Specific Processes

Now let's make this concrete for *your* team. For each workflow stage, write down:

**Stage: Specification Review**
- Who attends: Spec lead, 3 developers, QA engineer
- What we check:
  - Are acceptance criteria measurable?
  - Do developers understand their feature?
  - Does QA know how to test this?
  - Are integration contracts clear?
- Decision: "Is this spec clear enough to implement?" (yes/no)
- If no: What questions need answering before we proceed?

**Stage: Implementation**
- Who: Each developer on their own feature
- What developers commit to:
  - Implement the exact spec (no additions, no shortcuts)
  - Write tests for acceptance criteria
  - Ask clarifying questions if anything is ambiguous
  - Report blockers immediately
- Integration point: Each developer uses the defined integration contract
- Timeline: Parallel development, estimated completion date

**Stage: Spec Changes**
- If a developer finds the spec is ambiguous:
  - They message the spec lead with the specific question
  - Spec lead responds with clarification or spec update
  - If spec update: All developers are notified
  - Developers decide: Do I need to change my implementation?
- Goal: Keep specs as stable as possible
- Rule: No code changes without spec clarity

**Stage: Integration Testing**
- Who: QA engineer tests all features together
- What's tested:
  - Each feature works according to acceptance criteria
  - Features work together through integration contracts
  - Data flows correctly between features
  - Error cases are handled
- Process:
  - If feature works: Mark as PASSED
  - If feature fails: Is it the feature or the spec?
    - Feature bug: Send back to developer with detailed report
    - Spec bug: Notify spec lead; update spec; developer re-implements

**Stage: Deployment**
- Who: Spec lead + all developers + QA engineer
- What we verify:
  - All tests passing
  - Integration tests passing
  - Rollback plan documented
  - Everyone agrees: We're ready to deploy
- Decision: Deploy or wait?

### Step 3: Write Your Team's Specification Workflow Document

Create a document called "Specification Workflow" that includes:

```markdown
# Our Grading System: Specification Workflow

## Stage 1: Specification Review (Before Implementation)
**Participants**: Spec lead, 3 developers, QA engineer
**Input**: Draft specification
**Output**: Approved specification (or questions to answer)

**Checklist**:
- [ ] Acceptance criteria are measurable
- [ ] Developers understand their feature
- [ ] QA engineer knows how to test it
- [ ] Integration contracts are clear
- [ ] All questions answered

**Decision**: Ready to implement? YES / NO

---

## Stage 2: Implementation (Parallel)
**Participants**: Each developer independently
**Input**: Approved specification for their feature
**Output**: Complete, tested implementation

**Commitments**:
- Implement the spec exactly
- Write tests for acceptance criteria
- Ask spec lead for clarifications
- Use integration contracts
- Report blockers

**Timeline**: [Estimated dates for each feature]

---

## Stage 3: Specification Changes (If Needed)
**When**: If developer finds spec is ambiguous
**Process**: Developer → Spec Lead → All Developers Notified
**Decision**: Update spec or clarify in writing?

---

## Stage 4: Integration Testing
**Participants**: QA engineer, spec lead, developers available for debugging
**Input**: Completed features from each developer
**Output**: Integration test results (PASS or identified bugs)

**Test Cases**:
- Feature 1 meets spec acceptance criteria
- Feature 2 meets spec acceptance criteria
- Features integrate through defined contracts
- Data flows correctly end-to-end

**On Failure**: Feature bug or spec bug?

---

## Stage 5: Deployment
**Participants**: Spec lead, all developers, QA engineer
**Verification**: All tests passing, rollback plan ready
**Decision**: Deploy or hold?

---

## Key Principle
All coordination happens through specifications. Developers don't ask "What should I do?" They ask "Is the spec clear?" Specs are the contract. Code must match the contract.
```

This document becomes your team's way of working.

---

## Part 3: Identify Team Coordination Points

Now let's extract the places where your team *must* communicate synchronously (meetings, messages, etc.):

Ask your companion:

> **Prompt**: "I've designed a specification-driven team workflow with 5 people. Here are the stages: [paste your workflow]. Where do people absolutely have to communicate synchronously? Where can they work asynchronously (through specs and documentation)?"

Your companion will identify things like:

**Must be synchronous:**
- Specification review meeting (everyone needs to agree the spec is clear)
- Integration problems (if features don't connect, you need real-time debugging)
- Decisions about spec changes (affects everyone)

**Can be asynchronous:**
- Implementation (each developer works independently)
- Daily questions about the spec (message the spec lead, they respond when they can)
- Test results (post in a shared document)

### Step 4: Create a Communication Plan

Based on your conversation, create a simple communication plan:

**Synchronous (Meetings)**:
- Specification review: 1 hour before implementation starts
- Integration problems: As needed (usually 30 minutes to diagnose)
- Deploy decision: 30 minutes before deployment

**Asynchronous (Specs, Documents, Messages)**:
- Developers implement independently
- Spec questions: Message spec lead; they respond within 24 hours
- Test results: Posted in shared document
- Integration contracts: Written in specification

---

## Part 4: Reflect on Scaling

Now let's imagine your team grows beyond 5 people.

Ask your companion:

> **Prompt**: "My team has 5 people using this specification workflow. Now imagine the team grows to 10 people. How does the workflow change? What breaks? What stays the same? What new roles emerge?"

Think about:
- **More developers**: How do you coordinate 5 feature developers?
- **More complexity**: How do you handle 10 interdependent features?
- **More specs**: Do you need multiple spec leads or one?
- **More testing**: Does one QA engineer become a bottleneck?

At larger scales, you'll discover:
- **Specification governance**: Who decides if a spec change is approved? A single spec lead becomes a bottleneck. Maybe you need a "spec review committee."
- **Feature teams**: Instead of 1 developer per feature, maybe teams own features. How do teams coordinate?
- **Service contracts**: Features become services. Instead of integration contracts, you have API specs.
- **Spec versioning**: Multiple versions of specs live in production. How do you manage that?

All of these are just refinements of the same principle: **Specifications are how teams coordinate.**

---

## Part 5: Compare With Traditional (Non-Spec) Workflows

Let's contrast specification-driven workflows with traditional workflows.

Ask your companion:

> **Prompt**: "Compare two approaches: (1) Specification-driven team workflow (what I just designed), (2) Traditional team workflow (meetings, emails, daily standups, lots of synchronous communication). Show me a table comparing: (a) How often people communicate, (b) What causes delays, (c) How changes are decided, (d) How integration happens, (e) Where rework happens."

You'll see something like:

| Aspect | Specification-Driven | Traditional |
|--------|---------------------|-------------|
| **Communication Frequency** | Before implementation (spec review); after if ambiguities; integration phase | Constant (daily standups, meetings, Slack) |
| **Delays Caused By** | Unclear specs | Waiting for feedback, unclear requirements |
| **Change Decisions** | Spec lead approves changes | Product manager, maybe via meetings |
| **Integration** | Automatic (specs define contracts) | Manual coordination, often last-minute |
| **Rework** | Rare (if specs are clear) | Common (misunderstood requirements) |
| **Parallelization** | Natural (specs define boundaries) | Blocked (developers wait for clarity) |

Notice: Specification-driven teams don't communicate less—they communicate more *efficiently*. Communication happens when it matters most (before implementation), not constant context-switching.

---

## Part 6: Identify Risks in Your Workflow

Every workflow has risks. Let's identify yours.

Ask your companion:

> **Prompt**: "Here's my specification-driven team workflow: [paste your workflow]. What are the top 5 risks? For each risk, tell me: (1) When does it happen? (2) What's the impact? (3) How do I prevent it?"

Likely risks:
1. **Ambiguous specs** → Developers make different assumptions → Rework during integration
   - Prevention: Spec review meeting must validate clarity

2. **Spec changes mid-implementation** → Developers rework their code → Delays
   - Prevention: Freeze specs once implementation starts; only allow emergency changes

3. **Integration problems** → Features don't connect despite clear contracts → Debugging chaos
   - Prevention: Integration test early with stub implementations

4. **Spec lead bottleneck** → Everyone waits for spec lead answers → Team blocked
   - Prevention: Spec lead responds quickly; or empower spec lead team

5. **QA testing bottleneck** → One QA engineer can't test all features → Delayed deployment
   - Prevention: Developers write tests in addition to QA

Document these risks and mitigations. This becomes your team's risk management plan.

---

## Part 7: Create Your Professional Workflow Document

Pull together everything you've learned in this lesson and create one comprehensive document:

**Your Team's Specification-Driven Workflow Guide**

This document should include:
1. The 5 workflow stages (spec review, implementation, spec changes, integration, deployment)
2. Who participates at each stage
3. What decisions are made
4. What could go wrong and how to prevent it
5. Communication plan (sync vs. async)
6. How to handle scale (what happens at 10 people?)

This becomes a professional artifact. If you were to onboard a new engineer to your team, you'd hand them this document and say: "This is how we work."

---

## Part 8: Reflection

Answer these questions:

1. **What's the difference** between a specification-driven workflow and a traditional workflow? Write one clear paragraph.

2. **Where does synchronous communication** still matter in a specification-driven team? Why can't it be all asynchronous?

3. **What's your biggest risk?** In your designed workflow, what could cause the most damage if it goes wrong? How will you prevent it?

4. **How does this enable teams?** Why can 5 people using specification-driven workflows deliver faster than 5 people using traditional workflows?

5. **What happens at scale?** How would your workflow change at 10 people? 50 people? 500 people?

---

## Try With AI

Solidify your learning by exploring a real challenge.

**Setup**: Use your AI companion. This lesson benefits from exploring specific challenges and edge cases.

**Prompt Set**:

1. **Understand the constraint:**
   > "I just designed a specification-driven team workflow. The key assumption is that specs are clear enough to guide implementation with minimal communication. In practice, what ambiguities in specifications cause the most coordination problems? Give me 3 real examples."

2. **Explore the edge case:**
   > "My workflow assumes specs don't change during implementation. But what if the customer asks for a significant change? How do I handle that in a specification-driven way without causing chaos?"

3. **Design for your context:**
   > "I'm building [your project type]. How would I adapt this specification-driven workflow for my team size ([X people]) and our technology stack ([your tech])? What would change? What would stay the same?"

4. **Scale your thinking:**
   > "Take my team from 5 to 15 people. Show me what breaks in my workflow. What new processes do I need? What new roles emerge?"

**Expected Outcomes**:
- You understand that specifications enable asynchronous parallel work
- You recognize that sync communication is still necessary, but strategic (not constant)
- You can design workflows appropriate to your team size
- You know how to scale workflows as teams grow

**Professional Note**: The best teams balance specification clarity with psychological safety. Clear specs reduce miscommunication, but they can also feel rigid. Make sure your workflows leave room for questions, creativity, and human judgment. Specifications are guardrails, not prisons.
