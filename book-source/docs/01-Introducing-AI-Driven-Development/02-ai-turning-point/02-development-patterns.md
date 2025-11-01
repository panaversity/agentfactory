---
sidebar_position: 2
title: "Development Patterns — Vibe Coding vs. Spec-Driven Development"
chapter: 2
lesson: 2
duration_minutes: 25

# HIDDEN SKILLS METADATA
skills:
  - name: "Distinguishing Development Patterns"
    proficiency_level: "A1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can identify differences between vibe coding and spec-driven development approaches"

  - name: "Understanding Vibe Coding Tradeoffs"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain when vibe coding works well and where it breaks down"

  - name: "Recognizing Specification-First Value"
    proficiency_level: "A2"
    category: "Soft"
    bloom_level: "Understand"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student can articulate why specifications matter for AI-era development"

learning_objectives:
  - objective: "Identify two fundamental development patterns: vibe coding (intuition-led) and spec-driven (specification-led)"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Comparison of characteristics and contexts for each approach"

  - objective: "Understand tradeoffs of vibe coding: when it excels (learning, exploration, solo work) and where it fails (scaling, teams, AI)"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explanation of specific scenarios where vibe coding breaks down"

  - objective: "Evaluate why specification-first development is essential with AI tools"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Reflection on how AI needs clarity and specifications to be effective"

cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (vibe coding, spec-driven, pattern tradeoffs, AI compatibility) within A1-A2 limit ✓"

differentiation:
  extension_for_advanced: "Research code quality metrics comparing vibe vs. spec-driven; analyze refactoring costs"
  remedial_for_struggling: "Use one simple example (personal project vs. team project); build understanding incrementally"
---

# Development Patterns — Vibe Coding vs. Spec-Driven Development

You've probably experienced this: you start a project and move *fast*. No specs, no planning, just coding. The first feature ships in hours. It works. You ship it.

Then week two arrives. You need to add a related feature, but the code resists. The structure doesn't accommodate it. You discover edge cases weren't tested. A teammate asks, "Why did you do it this way?" You don't have a good answer—it just felt right at the time.

By week three, you're slower than if you'd spent the first two days planning.

This reveals something important: **there are fundamentally different ways of building software, and they have different tradeoffs.** Neither is universally "right" or "wrong." But understanding when each one works—and why—is critical for using AI effectively.

## Vibe Coding: Intuition-Led Development

**Vibe coding** is development driven by intuition, exploration, and immediate feedback. You write code, run it, see what happens, and adjust. There's no formal specification. Requirements emerge as you explore.

### Why Vibe Coding Works

Vibe coding is **genuinely excellent** for certain contexts:

- **Learning**: When you're new to a language or framework, vibe coding gives immediate feedback. You try something, it fails, you learn why. This tight feedback loop is one of the best ways to internalize how something works.

- **Exploration and Discovery**: When you're prototyping a new idea or investigating a technical question, rigid planning can slow you down. "What if I try this?" is often faster than "Let me document this first."

- **Low Stakes Solo Work**: If you're building a personal tool or learning project, the cost of rework is low. You can afford to refactor.

- **Creative Flow**: Some of the best code comes from a state of creative focus where you're responding to the problem intuitively.

Many experienced developers will tell you their best work came from vibe coding sessions. There's real value there.

### Where Vibe Coding Breaks Down

Vibe coding has **predictable failure modes** that show up when stakes increase:

- **Ambiguous Requirements**: Without a written spec, different team members have different mental models. "I thought it would work like X" versus "I built it to work like Y" becomes a recurring problem.

- **Missing Tests**: Vibe coding naturally skips tests. You test manually, which works until you don't test that one edge case—the one that breaks in production at 3 AM.

- **Architecture Drift**: The code evolves organically, which is fine until the structure no longer supports new features. Adding the third variant reveals the code wasn't designed for extensibility.

These aren't design flaws—they're natural properties of exploration-first development. They don't matter much when stakes are low. They matter enormously when you're shipping to production or working in a team.

## Spec-Driven Development: Method-Led Development

**Spec-Driven Development (SDD)** inverts the order. You write a specification first, defining what you're building, why, and how success looks. Then you write tests that encode that specification. Only then do you implement the feature.

### The 7-Step SDD Workflow

The complete workflow integrates Test-Driven Development (TDD) and knowledge capture:

1. **Specify**: Write a clear specification. What should this feature do? What are edge cases? How does it interact with the rest of the system?

2. **Plan**: Break the specification into implementable tasks. Identify dependencies, sequencing, and resources needed.

3. **Tasks**: Create a checklist of concrete, testable tasks with acceptance criteria.

4. **Implement (Red-Green)**:
   - **Red**: Write tests that encode the spec. They fail because the feature doesn't exist yet.
   - **Green**: Write code to pass the tests. Your code's job is clear: satisfy the spec and pass the tests.

5. **Refactor**: Clean up the implementation while keeping tests passing. Improve structure, clarity, and performance.

6. **Explain**: Document decisions, trade-offs, and reasoning. Future maintainers (including you) will thank you.

7. **Record/Share**: Capture decisions in Architecture Decision Records (ADRs). Share work via Pull Requests with context.

**Note**: We'll formalize this methodology in a dedicated part later in the book. For now, understand the principles.

### Why SDD Works

- **Predictable Delivery**: Because requirements are clear upfront, estimates are more accurate. Surprises are fewer.

- **Sustainable Velocity**: As the codebase grows, SDD codebases remain navigable. New features don't require heroic refactoring.

- **Team Collaboration**: A written spec means everyone has the same understanding. Code reviews focus on implementation quality, not debating what should be built.

- **Maintainability**: Someone joining the team six months later can read the spec and tests and understand not just *what* the code does, but *why*.

- **Confident Refactoring**: With tests in place, you can refactor internal structure without fear.

## A Concrete Comparison: Team A vs. Team B

Let's make this concrete. Two teams building the same feature: **add a `/summarize` endpoint that takes a PDF file, extracts its content, and returns a summary.**

### Team A: Vibe Coding

**Week 1, Tuesday morning**: A developer starts coding. No spec. "I'll figure it out as I go."

- **Tuesday afternoon**: The endpoint works locally. Reads PDF, extracts text, sends it to summarization API, returns result. Manual testing looks good. Shipped.
- **Total time: 10 hours**

The code is fast. Velocity looks amazing.

**Wednesday morning in staging**: The PDF parsing library behaves differently on the staging machine. The code crashes on certain PDFs. The developer didn't test those cases.

**Wednesday afternoon**: Fix is rolled out, but it's patchy. The architecture doesn't accommodate variations well.

**Friday**: A teammate asks, "Can we add Word document support?" The developer realizes the PDF extraction is tightly coupled to the summarization logic. Adding Word means rewriting half of it.

**Monday of week 2**: The team decides `/summarize` needs to be rebuilt from scratch. They allocate three days for redesign.

**Total cost: 10 hours + 24 hours rework = 34 hours. Plus damage to team confidence.**

### Team B: Spec-Driven Development

**Week 1, Tuesday morning**: Before coding, the team writes a specification:

> "The `/summarize` endpoint accepts multipart form data containing a document file (initially PDF, designed for future extensions). It extracts content, summarizes using external API, and returns structured JSON. Edge cases: files > 100MB, corrupted PDFs, missing API key, rate limiting. Design: document parser abstraction allowing multiple format handlers."

- **Tuesday, 2 hours**: Specification complete. Everyone agrees on what "done" looks like.
- **Tuesday-Wednesday, 3 hours**: Write comprehensive tests. Tests validate the spec—large files, corrupted PDFs, API down scenarios. Tests fail because the feature doesn't exist yet.
- **Wednesday-Thursday, 4 hours**: Implement the feature to pass tests. Developer knows exactly what to build.
- **Thursday afternoon, 1 hour**: Code review. Spec and tests make review focused.
- **Total time: 10 hours. Feature ships reliably. No staging surprises.**

**Friday**: Teammate asks, "Can we add Word support?" The developer looks at the spec and code. Parser is abstracted. Adding Word means writing a Word parser handler. Existing code doesn't change.

- **Friday afternoon, 2 hours**: Word support added, tested, shipped.

**Total cost: 10 hours initial + 2 hours extension = 12 hours. Sustainable. Confident.**

### What's Different?

Team A's velocity *appears* high until you count rework. Team B's velocity appears lower initially but is actually *sustained* because the code doesn't accumulate friction.

This isn't about smart engineers versus careless ones. It's about **method matching context**. Team A used exploration-mode thinking for production-mode stakes.

## When Each Approach Fits

| **Context** | **Vibe Coding** | **Spec-Driven Development** |
|---|---|---|
| **Learning a new technology** | Excellent | Unnecessary friction |
| **Shipping to production** | High risk | Appropriate |
| **Solo project, low stakes** | Perfect | Overhead |
| **Team feature, production code** | Coordination costs | Essential |
| **Exploration/prototyping** | Ideal | Premature structure |
| **Extended maintenance** | Knowledge silos | Clarity and velocity |

**The key insight**: Vibe coding is powerful. SDD is powerful. They're designed for different contexts. Using vibe coding for production features creates the "fast week one, slow week two" pattern. Using SDD for learning creates unnecessary ceremony.

## Why This Matters for AI Development

Here's where AI changes the stakes: **AI is an amplifier of whatever practice you bring to it.**

When you vibe code with Claude or ChatGPT, the AI generates code quickly. It feels amazing—you ask for something, you get working code in seconds. But you're amplifying every weakness of vibe coding. The AI won't write a spec you didn't ask for. It won't write tests you didn't request. You ship fast and encounter the same staging surprises Team A did—except now there's AI-generated code no one fully understands.

Conversely, when you use SDD with AI, the AI becomes a force multiplier for the things that matter. You ask it to help you write a clear spec. You ask it to generate tests from your spec. You ask it to implement against those tests. The AI handles the mechanical parts while you handle the judgment—is the spec right? Are edge cases covered?

**The discipline becomes MORE critical with AI, not less.** AI makes it easier to code fast. That makes vibe coding more tempting. And that's precisely when you need discipline most.

---

:::note Skeptic's Corner: "Isn't SDD just bureaucracy?"

**Fair pushback. Let's be specific.**

SDD *can* become bureaucracy if applied dogmatically. Writing 10-page specifications for a 20-line function is overhead. The question is: what problem are you solving?

**When SDD is overhead:**
- Solo learning projects where rework costs nothing
- Prototypes you'll throw away
- Exploration where requirements are genuinely unknown

**When SDD prevents friction:**
- Production features serving real users
- Team projects with multiple contributors
- Code that will be maintained for months/years
- Regulated environments requiring audit trails

The difference isn't "bureaucracy vs. freedom." It's "clarity upfront vs. surprises later." SDD trades a little time today for a lot of time tomorrow. That trade only makes sense when tomorrow matters.

Think of it like blueprints for construction. Building a treehouse in your backyard? Sketch on a napkin. Building an office tower? You need blueprints. The discipline isn't bureaucracy—it's appropriate for the stakes.

:::

---

## Try With AI

Use your AI companion tool set up (e.g., ChatGPT web, Claude Code, Gemini CLI), you may use that instead—the prompts are the same.

### Prompt 1: Compare Development Approaches
```
Explain the difference between 'vibe coding' (jumping in and iterating quickly) and 'spec-driven development' (planning first, then building). Which approach is better for: (a) learning a new skill, (b) building a personal project, (c) creating a product for real users? Give me specific examples of when each approach works best.
```

**Expected outcome**: Clear understanding of when to plan vs. when to explore (without technical jargon).

### Prompt 2: Plan A Real Project
```
I want to build [describe something you'd like to create: a website, an app, a business tool, etc.] using AI as my collaborator. Walk me through what 'planning first' would look like for this project. What questions should I answer before asking AI to help me build it? What could go wrong if I skip the planning step?
```

**Expected outcome**: Practical insight into planning a real project with AI collaboration.

### Prompt 3: Debug Common Problems
```
Here's a realistic scenario: I'm building a simple tool and I use AI to generate code quickly. It works on my computer, but when I share it with others, it breaks. What likely went wrong? What should I have done differently? Give me a simple checklist I can use next time to avoid this problem.
```

**Expected outcome**: Awareness of common beginner mistakes (like "works on my machine" syndrome) and how to avoid them.

### Prompt 4: Create Decision Framework
```
Create a simple decision framework for me: Given a project or idea, how do I decide whether to 'explore quickly with AI' versus 'plan carefully first'? Make it practical—something I can use as a beginner without needing to know technical jargon.
```

**Expected outcome**: A simple, reusable decision framework for choosing your approach.


