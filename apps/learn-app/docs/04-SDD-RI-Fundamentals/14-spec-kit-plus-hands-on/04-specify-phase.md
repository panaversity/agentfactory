---
title: "Specify Phase — Writing Complete Specifications"
chapter: 14
lesson: 4
duration_minutes: 45
proficiency_level: "A2-B1"

learning_objectives:
  - objective: "Execute the `/sp.specify` command to convert vague requirements into clear specifications"
    bloom_level: "Apply"
    assessment_method: "Student produces valid spec.md with all required sections"

  - objective: "Write specification with clear Intent, Constraints, Success Evals, and Non-Goals"
    bloom_level: "Apply"
    assessment_method: "Specification review checklist confirms all sections present and SMART"

  - objective: "Distinguish what from how (specification vs implementation)"
    bloom_level: "Understand"
    assessment_method: "Student identifies implementation details leaked into spec and corrects them"

cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (Specification definition, `/sp.specify` command, SMART success criteria, Non-Goals scope boundaries) within A2-B1 limit ✓"

generated_by: "content-implementer v1.0.0"
source_spec: "specs/037-chapter-14-research-paper-pivot/spec.md"
created: "2025-11-26"
last_modified: "2025-11-27"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Specify Phase — Writing Complete Specifications

Welcome to the most important phase: Specification. This is where you translate your vague ideas ("I want to write a research paper") into crystal-clear requirements that AI can build from.

Here's the paradigm shift: **In AI-native development, your ability to write a clear specification is more valuable than your ability to write code.** Bad code can be refactored—but a bad spec breaks everything downstream. In this lesson, you'll learn how to collaborate with AI to define success and turn it into a complete, testable specification.

---

## Thinking About What Success Looks Like

Before writing a specification, define criteria for evaluating whether the solution succeeds. This is NOT a formal phase; it's an informal human-AI conversation that clarifies intent.

Not "does it compile?" or "do tests pass?" (those are technical validation). Real evals are business/user criteria that define success:

**For a research paper:**
- ✅ "Paper defines 3+ concrete AI applications in K-12 classrooms"
- ✅ "All claims supported by peer-reviewed sources"
- ✅ "Reader can explain ROI of classroom AI after reading"
- ✅ "Paper passes plagiarism check with 0% match"

**For any document:**
- ✅ "Audience can complete the intended action after reading"
- ✅ "All factual claims are traceable to sources"
- ✅ "Structure follows the established format requirements"

---

## Part A: Pre-Specification Conversation

Here's how professionals work: **YOU drive the exploration through questions.** The AI doesn't interview you; you use the AI to think through requirements.

### Step 1: Start the Conversation

Open your AI tool in your `my-research-paper` directory:

```
I want to write a research paper on AI's impact on K-12 classroom efficiency.
Before I write the formal spec, help me think through:

1. What aspects of "AI in education" should I focus on?
2. Who is my target audience?
3. What does success look like for this paper?
4. What edge cases or constraints should I consider?
```

**Example conversation:**

```
You: "I want to write a research paper on AI in K-12 education.
      What aspects should I focus on?"

AI: "Key focus areas for K-12 AI research:
     - Teacher workload reduction (grading, planning automation)
     - Student engagement improvements (adaptive learning)
     - Administrative efficiency (scheduling, resource allocation)
     - Learning outcome measurement (assessment analytics)
     Which resonates with your goals?"

You: "I'm interested in teacher workload and student outcomes.
      What would success look like for this paper?"

AI: "For a paper on those topics, success criteria might be:
     - Identifies 3+ specific AI applications with evidence
     - Cites peer-reviewed sources (8+ minimum for academic credibility)
     - Reader understands economic ROI (time saved, outcomes improved)
     - Clear enough that school administrators could act on recommendations"

You: "What constraints should I consider?"

AI: "Key constraints for academic papers:
     - Word count: 3000-5000 words is standard for this scope
     - Citation style: APA is common for education research
     - Source quality: Peer-reviewed journals, recent publications (past 10 years)
     - Scope boundaries: What you're NOT covering (ethics, specific products)"
```

### Step 2: Summarize What You Learned

From the conversation, you now know:
- **Focus**: Teacher workload reduction + student outcomes
- **Audience**: Education administrators and technology decision-makers
- **Success**: 3+ AI applications, 8+ sources, clear ROI explanation
- **Constraints**: 3000-5000 words, APA format, peer-reviewed sources
- **Not covering**: Ethics debates, specific product comparisons

This becomes your specification.

---

## Part B: Write Your Specification

Now formalize the conversation into a specification using `/sp.specify`.

### Step 1: Run `/sp.specify`

```
/sp.specify Research paper on AI's impact on K-12 classroom efficiency

Target audience: Education administrators evaluating AI adoption
Focus: Teacher workload reduction and student outcome improvements

Success criteria:
- Identifies 3+ concrete AI applications with evidence
- Cites 8+ peer-reviewed academic sources
- Reader can explain ROI of classroom AI after reading
- All claims supported by evidence

Constraints:
- Word count: 3000-5000 words
- Format: Markdown source, APA citations
- Sources: Peer-reviewed journals, published within past 10 years
- Timeline: Complete within 2 weeks

Not building:
- Comprehensive literature review of entire AI field
- Comparison of specific AI products/vendors
- Discussion of ethical concerns (separate paper)
- Implementation guide or code examples
```

**What the agent does:**
- Creates a new feature branch automatically
- Generates comprehensive spec file at `specs/[feature-name]/spec.md`
- Defines user scenarios and edge cases
- Establishes acceptance criteria
- Sets up testing requirements

### Step 2: Review the Generated Specification

After the agent creates your spec, review it:

```
Show me the generated specification and explain what each section contains.
```

**Agent shows:**
- **Intent** — What you're building and why
- **Constraints** — Boundaries and requirements
- **Success Evals** — Measurable acceptance criteria
- **Non-Goals** — What you're explicitly NOT building

### Step 3: Verify Completeness

Check that your specification has:

```
Specification Checklist:

[ ] Intent is clear (someone unfamiliar can understand the goal)
[ ] Constraints are specific and testable (not vague "do good work")
[ ] Success Evals are SMART (Specific, Measurable, Achievable, Relevant, Time-bound)
[ ] Non-Goals are explicit (prevents scope creep)
[ ] No "how" leaked in (describes what, not how to build)
[ ] Written clearly enough that another person could write from it
```

---

## Part C: The SMART Test

Success Evals must be **SMART**: Specific, Measurable, Achievable, Relevant, Time-bound.

### Test Your Success Criteria

**❌ Vague (NOT SMART):**
```
- Paper is high-quality
- Sources are credible
- Writing is clear
- Reader finds it valuable
```

**✅ SMART:**
```
- Paper is between 3000-5000 words
- Paper cites 8+ peer-reviewed academic sources
- Each major claim is supported by evidence
- Reader can explain 3 concrete AI use cases after reading
- Paper completed within 2-week timeframe
```

### Ask AI to Review

```
Review my specification at specs/[feature-name]/spec.md.

For each success criterion, check if it's SMART:
- Specific (not vague)?
- Measurable (can verify objectively)?
- Achievable (realistic)?
- Relevant (matters for this paper)?
- Time-bound (has deadline)?

Identify any vague criteria and suggest specific alternatives.
```

---

## Common Mistakes

### Mistake 1: Leaking Implementation Into Specification

**The Error:**
```markdown
## Specification: Research Paper

The paper will be written using Claude AI to:
1. Research the topic
2. Outline the structure
3. Generate each section based on the outline
4. Ask Claude to review and refine
```

**Why It's Wrong:** This is IMPLEMENTATION (HOW). Specification is WHAT.

**The Fix:** Keep specification focused on outcomes, not process:
```markdown
## Intent
3000-5000 word research paper on AI in K-12 education.

## Success Evals
- Paper identifies 3+ concrete AI applications
- At least 8 citations from peer-reviewed sources
- Each claim supported by evidence
```

### Mistake 2: Vague Success Criteria

**The Error:** "Paper should be well-written" or "Sources should be credible"

**Why It's Wrong:** "Well-written" and "credible" are subjective. No one can verify these.

**The Fix:** Use testable criteria:
- ❌ Vague: "Paper should be well-researched"
- ✅ SMART: "Paper cites 8+ peer-reviewed sources; average publication date within past 10 years"

### Mistake 3: Missing Non-Goals

**The Error:** Only specifying what you ARE building, not what you're NOT building.

**Why It's Wrong:** Scope creep happens. AI might add "helpful" sections you didn't want.

**The Fix:** Explicitly state boundaries:
```markdown
## Non-Goals
- Not a comprehensive literature review (focused analysis only)
- Not a comparison of specific AI products
- Not an implementation guide
- No discussion of ethical concerns (separate paper)
```

---

## Validation: The Cascade Effect

Now test your specification's quality by asking: **Will this spec produce a good plan?**

**A good spec has:**
- ✅ Crystal-clear intent (no ambiguity)
- ✅ Explicit constraints (no surprises during planning)
- ✅ Measurable success criteria (AI can build acceptance tests from them)
- ✅ Constitution alignment (specification respects Constitution standards)

**A bad spec has:**
- ❌ Vague intent ("should work correctly"—what's "correct"?)
- ❌ Missing constraints (surprises emerge during implementation)
- ❌ Ambiguous criteria ("handle errors"—how?)
- ❌ Ignores Constitution (specification asks for things Constitution forbids)

**Action:** Read your specification aloud. Does it sound clear? Would someone else understand exactly what to write?

---

## Try With AI

Ready to write your specification? Practice with your AI companion:

**Explore Success Criteria:**

> "I want to write a research paper on [your topic]. Before I write the formal spec, help me define success: What does 'good' mean for this paper? What would prove it's valuable? Don't let me write vague requirements like 'well-researched'—push me toward specific, measurable criteria."

**Practice SMART Criteria:**

> "Review my specification at specs/[feature-name]/spec.md. For each success criterion, check if it's SMART (Specific, Measurable, Achievable, Relevant, Time-bound). Identify vague criteria like 'good quality' and suggest specific alternatives."

**Test Specification Completeness:**

> "Looking at my spec, identify what's missing: (1) Are constraints explicit? (2) Are non-goals defined? (3) Could someone else write this paper from just the spec? (4) Does it respect my Constitution standards? Generate a list of improvements."

**Apply to Your Paper:**

> "Help me write a specification for my research paper on [topic]. Walk me through: (1) What exactly am I building? (2) Who is the audience? (3) What does success look like—specifically? (4) What constraints apply? (5) What am I NOT covering? Then run /sp.specify with everything we discussed."
