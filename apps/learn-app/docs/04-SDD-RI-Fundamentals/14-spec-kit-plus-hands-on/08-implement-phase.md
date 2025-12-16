---
title: "Implement Phase - Execute Tasks with AI Collaboration"
chapter: 14
lesson: 8
duration_minutes: 60
proficiency_level: "B1"
cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (implementation execution, checkpoint validation, iterative refinement, AI collaboration in execution) within B1 limit of 10 ✓"
learning_objectives:
  - objective: "Understand implementation as executing tasks that fulfill specification"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of implementation phase purpose and structure"
  - objective: "Use /sp.implement command to execute tasks with AI assistance"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful execution of /sp.implement on research paper project"
  - objective: "Apply checkpoint pattern to maintain control during implementation"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Evidence of review and decision-making at each phase boundary"
  - objective: "Validate task completion against specification success criteria"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Verification that output meets spec requirements"
generated_by: "content-implementer v1.0.0"
source_spec: "specs/036-chapter-14-sdd-ri-hands-on/spec.md"
created: "2025-11-26"
last_modified: "2025-11-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Implement Phase — Execute Tasks with AI Collaboration

You have a specification that defines what you're building, a plan that outlines the strategy, and tasks that break the work into atomic units. Now comes the execution phase: actually doing the work with your AI companion.

This lesson focuses on **control and validation**. Implementation isn't just "run tasks autonomously." It's you and AI working together—you deciding direction, AI handling execution, both of you validating results against the specification.

---

## What Implementation Means in SDD-RI

**Implementation = executing your tasks.md to fulfill your specification.**

This is fundamentally different from ad-hoc coding. Consider the difference:

**Without specification (ad-hoc)**:
```
You: "Generate text for a research paper introduction"
AI: [Produces 500 words]
You: "Good enough?" (You have no objective standard)
```

**With specification (SDD-RI)**:
```
You: "Execute Task 1.1: Write research paper introduction (500-700 words,
      academic tone, establish context for three key claims from spec)"
AI: [Produces text]
You: Check against spec:
  ✓ 650 words (within 500-700)
  ✓ Academic tone maintained
  ✓ All three key claims established
  ✓ Task complete
```

The difference: **Objective acceptance criteria from your spec**.

Implementation executes against these criteria. When a task is done, you know it's done because it meets the spec's success criteria.

---

## The /sp.implement Command

When you run `/sp.implement` in Claude Code, the command reads your tasks.md and orchestrates their execution with your AI companion.

**Basic usage**:
```
/sp.implement
```

The agent will:
1. Read your tasks.md
2. Begin executing tasks in dependency order
3. Show outputs and intermediate results
4. Wait for your review at checkpoint boundaries
5. Continue on your approval

**You maintain control.** The agent doesn't proceed autonomously; it presents work and waits for your decision.

---

## Checkpoint Pattern: The Core Control Mechanism

Implementation uses checkpoints to maintain human decision-making at critical boundaries.

**The pattern:**

```
Task 1 → Task 2 → Task 3 → CHECKPOINT 1
                            (You review)
                            (You approve or iterate)
                                ↓
                            Task 4 → Task 5 → CHECKPOINT 2
                                            (You review)
```

**At each checkpoint**, you answer one question: **"Does the output meet the specification?"**

If yes: Commit and move forward.
If no: Work with AI to refine until it meets spec.

### Example Checkpoint Review

**Your tasks.md specifies:**
```
Task 1.1: Write research paper introduction
- Success criteria: 500-700 words, academic tone, introduces three main arguments
- Dependencies: None
```

**Agent completes Task 1.1, shows output:**
```
[Introduction text, 623 words]
```

**Your checkpoint review (2 minutes)**:
- Count words? 623 (within 500-700) ✓
- Academic tone? Yes, peer-reviewed style ✓
- Three arguments introduced? Argument 1, 2, 3 all present ✓
- Ready for next task? Yes ✓

**Your decision**: "Task 1.1 meets spec. Committing. Proceed to Task 1.2."

This prevents cascading problems. If Task 1.1 had missed an argument, you'd catch it before Task 2 builds on faulty foundation.

---

## The Four Concepts You're Learning

### Concept 1: Implementation ≠ Autonomous Execution

Many students assume implementation means "Tell AI to build it and walk away."

**Reality**: Implementation is orchestrated execution with human judgment at every phase.

You direct. AI executes. You validate. You iterate if needed. This cycle repeats for each phase.

### Concept 2: Spec Success Criteria Are Your Acceptance Standard

Before running /sp.implement, your spec already defines what "done" means.

**In your spec:**
- Introduction section: 500-700 words, academic tone, three arguments
- Literature review: 800-1000 words, 10+ sources, critical analysis
- Methodology: Clear procedure, reproducible, 300-400 words

**During implementation:**
- Task completes
- You check: "Does it meet success criteria from spec?"
- Not yet? "AI, adjust X aspect and try again"
- Meets spec? "Commit. Next task"

This objective standard prevents scope creep and "almost good enough" mediocrity.

### Concept 3: Iteration Loops Accelerate Problem-Solving

When a task doesn't meet spec, iteration (not abandonment) is the solution.

**Failure scenario**: You request introduction. AI produces 1200 words (exceeds 700-word spec).

**Without iteration (wrong)**:
```
You: "That's too long. Project failed."
```

**With iteration (correct)**:
```
You: "This exceeds the 700-word spec limit. Can you condense to 500-700
      while keeping all three arguments?"
AI: [Revises to 680 words]
You: [Validates against spec]
Done.
```

Iteration transforms failures into learning. The spec guides refinement.

### Concept 4: AI Collaboration During Execution

AI doesn't just execute—it collaborates during execution.

**Example iteration dialogue:**

You: "Task 2.3: Write methodology section (300-400 words, reproducible procedure)"

AI proposes approach: "I'll structure this with (1) research population, (2) data collection method, (3) analysis approach. Does this align with what you're aiming for?"

You refine: "Add one more: (4) validity checks. Want to ensure readers can replicate this."

AI adjusts proposal: "Perfect. Here's revised structure with validity checks integrated..."

AI executes revised version: [Produces 385-word methodology]

You validate against spec: ✓ 385 words (within 300-400), ✓ Reproducible, ✓ Validity checks present

**This is AI collaboration in implementation.** Not "execute blindly," but "suggest → refine → execute → validate."

---

## Example: Implementing Your Research Paper

Let's walk through what happens when you execute `/sp.implement` for your paper project.

### Setup Before Running /sp.implement

You have:
- **spec.md**: Paper specification with success evals (5-7 sections, 3000-4000 total words, academic sources, coherent argument)
- **plan.md**: Implementation plan (write intro → lit review → methodology → analysis → conclusion, each section builds on previous)
- **tasks.md**: Atomic tasks (Task 1.1, 1.2, 1.3... Task 5.4, 5.5)

### Running the Command

```
/sp.implement
```

Agent reads tasks.md, recognizes structure:
```
Phase 1: Introduction & Literature Foundation
├─ Task 1.1: Research paper introduction (500-700 words)
├─ Task 1.2: Literature review outline (sources identified, 10+ required)
└─ CHECKPOINT 1: Validate intro + outline against spec

Phase 2: Methodology & Analysis
├─ Task 2.1: Methodology section (300-400 words, reproducible)
├─ Task 2.2: Analysis section (800-1000 words, critical synthesis)
└─ CHECKPOINT 2: Validate sections meet quality criteria

Phase 3: Integration & Refinement
├─ Task 3.1: Conclusion synthesizing all sections
├─ Task 3.2: Final review and adjustment
└─ CHECKPOINT 3: Full document validation against spec
```

### CHECKPOINT 1 Review

**Agent reports:**
```
Task 1.1: Introduction (623 words)
- Opening hook explaining research context
- Three main arguments clearly stated
- Transition to literature review prepared

Task 1.2: Literature outline
- 12 sources identified (exceeds 10-source requirement)
- Categories: foundational (3), recent empirical (5), contrasting (4)
- Annotations show critical analysis planned
```

**Your 3-minute review**:
- ✓ Introduction 623 words (within 500-700)
- ✓ Three arguments present and clear
- ✓ Outline has 12 sources (meets 10+ requirement)
- ✓ Sources span theoretical, empirical, and contrasting perspectives

**Your checkpoint decision**: "Both tasks meet spec criteria. Committing Phase 1. Ready for Phase 2."

### CHECKPOINT 2 Review

**Agent reports:**
```
Task 2.1: Methodology (385 words)
- Clear procedure described
- Population/sample specified
- Data collection method documented
- Analysis approach outlined
- Reproducibility criteria met

Task 2.2: Analysis (920 words)
- Synthesizes all 12 sources
- Identifies patterns across sources
- Addresses contradictions
- Connects to research questions
```

**Your 5-minute review**:
- Methodology: 385 words (within 300-400) ✓
- All steps reproducible ✓
- Analysis: 920 words (within 800-1000) ✓
- Critical synthesis visible ✓
- Connects back to spec's three main arguments ✓

**Your checkpoint decision**: "Both tasks meet spec. Committing Phase 2. Proceed to Phase 3."

### CHECKPOINT 3: Final Validation

**Agent reports:**
```
Complete document assembled:
- Introduction: 623 words ✓
- Literature Review: 1050 words (from outline + full section)
- Methodology: 385 words ✓
- Analysis: 920 words ✓
- Conclusion: 450 words ✓

Total: 3,428 words (within 3000-4000 spec requirement)

Final checks:
✓ All sections present
✓ Academic tone throughout
✓ 12 sources integrated
✓ Argument coherence verified
✓ Meets specification success criteria
```

**Your validation**:
- Word count: 3,428 (within 3000-4000) ✓
- All required sections ✓
- Sources properly cited ✓
- Argument is coherent and supported ✓

**Your final decision**: "Document meets specification. Implementation complete. Ready for next project phase."

---

## What Happens When Tasks Don't Meet Spec

**Real scenario**: Agent completes Task 1.1 (introduction) with 450 words. Your spec requires 500-700.

**At Checkpoint 1**:
```
Agent: "Task 1.1 complete: 450 words"
You: "This is below the 500-700 word spec requirement.
      Missing depth on which aspect?"
Agent: "The second argument needs more context. I can expand
       that section to add 75-100 words."
You: "Yes, expand the second argument while keeping tone consistent."
Agent: [Revises to 520 words]
You: [Validates]
Result: Task 1.1 now meets spec. Move forward.
```

**The key**: You used the spec to identify the gap, guided the refinement, and validated the fix. This is iterative implementation—normal, expected, productive.

---

## Common Implementation Patterns

### Pattern 1: Serial Task Execution (Most Common)

Tasks execute in order. Each builds on previous.

```
Task 1 → Task 2 → Task 3 → CHECKPOINT → Task 4 → Task 5 → CHECKPOINT
```

**When to use**: Linear projects (writing, sequential analysis)

**Your role**: Validate after each task block, ensure quality before next block uses output

### Pattern 2: Parallel Task Execution (When Possible)

Some tasks don't depend on others. Execute simultaneously.

```
Task 1.1 ─┐
Task 1.2 ─┼─→ CHECKPOINT 1
Task 1.3 ─┘
           Task 2.1 → Task 2.2 → CHECKPOINT 2
```

**When to use**: Independent sections (research paper: intro, methodology, analysis can draft in parallel)

**Your role**: Ensure parallel tasks don't create integration conflicts at checkpoint

### Pattern 3: Iterative Refinement (When First Pass Insufficient)

Task executes, checkpoint review shows gap, task re-executes with refinement.

```
Task 1 → Review → Gap Identified → Task 1 (refined) → Review → Approved
```

**When to use**: Complex tasks, creative work (good first draft exists, needs polish)

**Your role**: Provide specific feedback ("expand argument 2," "add example," "tighten analysis")

---

## Validation Against Specification

At each checkpoint, validation answers: **"Does this meet our specification?"**

### Validation Checklist Structure

**For each task completion:**

1. **Success Criteria Met?**
   - Does output match explicit criteria from task definition?
   - Word count? Format? Tone? Completeness?

2. **Specification Requirements Fulfilled?**
   - Does this task's output advance toward spec's overall success evals?
   - Are we building the right thing?

3. **Quality Standards?**
   - Does output meet your constitutional standards (from Lesson 3)?
   - Academic rigor? Clarity? Coherence?

4. **Ready for Next Task?**
   - Can the next task safely build on this output?
   - Any risks or dependencies that aren't met?

**Example checklist for Task 1.1 (Introduction)**:

```
Success Criteria:
  ☐ 500-700 words
  ☐ Academic tone
  ☐ Introduces three main arguments
  ☐ Smooth transition to literature review

Specification Requirements:
  ☐ Advances toward "coherent research argument" eval
  ☐ Establishes context for paper
  ☐ Reader understands what paper will argue

Quality Standards:
  ☐ Engaging opening hook
  ☐ Clear language (no jargon without definition)
  ☐ Proper grammar and citation format

Ready for Next Task:
  ☐ Literature review can build on these arguments
  ☐ No placeholder text or unresolved questions
```

If all checkboxes checked: Commit and continue.
If any unchecked: Iterate.

---

## The AI Collaboration Dynamic

Implementation isn't you alone or AI alone. It's **collaboration with clear structure**.

### Your Responsibilities
- **Direction**: "Here's what we're building (spec). Here's the next task."
- **Judgment**: "Does this meet our spec? If not, what's missing?"
- **Decision**: "Commit this task. Iterate on that task. Move forward."

### AI's Responsibilities
- **Execution**: "I'll complete this task to your specification."
- **Suggestion**: "I notice this approach might work better..." (AI as Teacher)
- **Adaptation**: "You want to add X requirement. Adjusting..." (AI as Student)
- **Refinement**: "Re-executing with your feedback..." (AI as Co-Worker)

### The Cycle
```
You specify task → AI proposes approach → You refine → AI executes
                    ↑                                     ↓
                    ←←← You validate ←←←←←←←←←←←←←←←←
```

This cycle repeats for each task. It's predictable, controlled, and produces measurable results.

---

## Checkpoint Decisions

At each checkpoint, you make one of three decisions:

### Decision 1: "Commit and Proceed"

**When**: All tasks meet spec requirements.

**Your statement**: "Phase X complete. All success criteria met. Committing. Proceed to Phase Y."

**What happens**: Agent records completion, moves to next phase.

### Decision 2: "Iterate on This Task"

**When**: A specific task doesn't meet spec.

**Your statement**: "Task 2.1 needs adjustment. It's 450 words but spec requires 500-700. Expand the analysis section by 75-100 words while keeping tone consistent."

**What happens**: Agent refines that task, re-executes, shows updated output.

**You review again**: Does it meet spec now? If yes → commit. If no → iterate again.

### Decision 3: "Revise the Plan"

**When**: A task completed reveals a planning problem.

**Your statement**: "Task 1.2 showed that the literature review is larger than planned (1050 vs 1000 words budgeted). We need to adjust the conclusion (reduce by 100 words) to keep total under 4000. AI, adjust plan and re-execute Task 1.3 accordingly."

**What happens**: Agent updates plan, adjusts downstream tasks, re-executes.

**When to use**: Infrequent. Reserve for when checkpoint review reveals structural issues, not just tweaks.

---

## Anti-Patterns: What Not to Do

### Anti-Pattern 1: Approving Without Review

```
❌ WRONG:
Agent: "Task complete"
You: "Looks good" (without checking against spec)

✓ RIGHT:
Agent: "Task complete"
You: [Spend 2-5 minutes validating against spec]
```

Skipping review defeats the purpose. Specs guide validation; validation ensures quality.

### Anti-Pattern 2: Accepting "Close Enough"

```
❌ WRONG:
Spec says: 500-700 words
Task delivers: 780 words
You: "Close enough, let's move on"

✓ RIGHT:
Spec says: 500-700 words
Task delivers: 780 words
You: "Over spec limit. AI, condense to 500-700 while keeping key points."
```

"Close enough" is how requirements creep. Specs exist to prevent this.

### Anti-Pattern 3: Not Iterating When First Pass Fails

```
❌ WRONG:
Agent: "Task 2.1 complete"
You: "This doesn't meet spec"
Agent: "Too bad. Moving on."
You: "Okay, I guess project is compromised now"

✓ RIGHT:
Agent: "Task 2.1 complete"
You: "This doesn't meet spec. Here's the gap: [specific feedback]"
Agent: "Understood. Adjusting and re-executing..."
Agent: [Shows revised output]
You: [Validates again]
Result: Task now meets spec
```

Iteration is normal. It's how you converge on specification compliance.

---

## Try With AI

Ready to understand `/sp.implement` deeply? Explore these prompts with your AI companion:

**Explore the Command Structure:**

> "I'm about to use `/sp.implement` to execute a research paper project. Walk me through what happens: (1) What does the agent read first (spec, plan, or tasks)? (2) How does it decide execution order? (3) How are checkpoints triggered? (4) What happens if I say 'iterate on Task 2' at a checkpoint? (5) When is implementation truly 'done'?"

**Practice Checkpoint Review:**

> "I just completed Task 1.2 of my research paper (literature review outline). The spec says 'identify 10+ academic sources, span theoretical and empirical work, show critical perspective.' The outline has 12 sources. How do I validate this meets spec in 3 minutes? What questions should I ask? What would make me say 'iterate' vs 'commit'?"

**Handle Iteration:**

> "My research paper introduction is 450 words, but spec requires 500-700. Rather than just asking to 'add 50-250 words,' how should I direct the AI? What specific aspect should expand? How do I ensure iteration improves the introduction rather than padding it with fluff?"

**Reflect on Specification Alignment:**

> "During implementation, I notice the research paper's conclusion feels disconnected from the introduction's three main arguments. This is a checkpoint review moment. Should I (a) iterate on the conclusion, (b) revise the plan to reconnect pieces, (c) go back and adjust the specification? What's the right decision and why?"

**Design Your First Checkpoint:**

> "For my next project, I'll write a technical spec document (not a research paper). Based on `/sp.implement` patterns, design what my first checkpoint should look like: What tasks go into Phase 1? What success criteria matter? What 3-5 questions should I answer during Phase 1 review?"

---
