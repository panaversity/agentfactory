---
title: "What Is Specification-Driven Development?"
chapter: 30
lesson: 2
duration: "2-2.5 hours"
skills:
  - name: "Specification Comprehension"
    proficiency: "A2"
    category: "Conceptual"
  - name: "SDD Methodology Understanding"
    proficiency: "A2"
    category: "Conceptual"
  - name: "AI Collaboration"
    proficiency: "B1"
    category: "Soft"
learning_objectives:
  - "Define Specification-Driven Development and explain its three key principles (A2)"
  - "Distinguish between spec-first, spec-anchored, and spec-as-source approaches (B1)"
  - "Understand the SDD Loop: Spec → Plan → Execute → Validate → Iterate (A2)"
---

# What Is Specification-Driven Development?

In Lesson 1, you discovered that vague specs fail. Your companion built something that technically worked but didn't match your intent.

**Here's the key insight you discovered**: Clarity prevents miscommunication.

Now the question is: **What methodology turns this insight into professional practice?**

**That methodology is Specification-Driven Development (SDD).**

SDD is a software engineering methodology where development begins with a specification — a structured, machine-readable description of how the code should behave. This specification acts as a contract between the developer and the coding agent. It defines inputs, outputs, constraints, design principles, and test expectations before any code is written.

Instead of coding first and documenting later, SDD reverses the order:

1. Write the spec: Define what you want — not in prose, but in structured form.
2. Generate code: Let coding agents produce implementations that conform to the spec.
3. Test and validate: The spec becomes the reference truth used by automated validators or agents to verify correctness.

This approach minimizes guesswork, aligns all stakeholders, and creates a traceable record of why and how decisions were made.

Instead of coding first and writing docs later, in spec-driven development, you start with a spec. This is a contract for how your code should behave and becomes the source of truth your tools and AI agents use to generate, test, and validate code. The result is less guesswork, fewer surprises, and higher-quality code.

The key distinction of SDD in the age of AI coding agents is that these specs become the primary interface for AI collaboration. Rather than generating code from loose natural language descriptions, AI agents work from structured specifications that leave less room for misinterpretation.

---

## ⚠️ What SDD Is NOT: The Semantic Diffusion Problem

As spec-driven development gains attention, the term "spec" is becoming diluted. People now use "spec" as a synonym for "detailed prompt" or "good instructions." This semantic diffusion makes it harder to distinguish genuine SDD from simply writing better prompts.

**Real SDD requires these five characteristics:**

1. ✅ **Structured artifacts** - Not just any instructions, but deliberately formatted documents following consistent patterns
2. ✅ **Behavior-oriented** - Focus on what the system should do, not just how to implement it
3. ✅ **Source of truth** - The spec remains authoritative throughout development, not just a one-time input
4. ✅ **Machine and human readable** - Designed for both AI consumption and human review
5. ✅ **Integration with workflow** - Specs are part of a broader development process, not standalone documents

**Without these characteristics**, you're doing good prompting, not spec-driven development.

**Why this distinction matters**: SDD implies commitments about process, tooling, and long-term maintenance that casual prompting doesn't. When evaluating SDD tools or practices, check whether they meet these five characteristics.

---

## The Spec Implementation Levels

In SDD, you are not treating the coding agent as a simple code generator; you're leveraging it as a literal-minded, highly capable **pair programmer** that excels when given explicit, unambiguous instructions. The focus shifts from rapidly producing code snippets to meticulously defining intent.

However, it's important to note that "spec-driven development" exists at multiple implementation levels:

- **Spec-first**: A well thought-out spec is written first, and then used in the AI-assisted development workflow for the task at hand.
- **Spec-anchored**: The spec is kept even after the task is complete, to continue using it for evolution and maintenance of the respective feature.
- **Spec-as-source**: The spec is the main source file over time, and only the spec is edited by the human; the human never touches the code.

All SDD approaches are spec-first, but not all strive to be spec-anchored or spec-as-source. Understanding which level a tool targets is crucial for evaluating its fit for your development needs.

---

## Memory Banks vs Specs: Understanding the Distinction

Before diving deeper into specs, it's crucial to understand an important distinction: **specs are not the same as memory banks**.

### Memory Banks (Constitutions / Steering)

**Memory banks** are persistent knowledge that applies across ALL AI coding sessions in your codebase:

- Rules files and coding standards
- High-level product vision and descriptions
- Architecture patterns and principles
- Technology stack decisions
- Security and compliance requirements

Think of memory banks as **permanent organizational knowledge** - the foundation that every feature must respect.

**Examples**:

- "All passwords must use bcrypt with cost factor 12+"
- "All API endpoints require authentication via JWT"
- "Test coverage must exceed 80%"
- "We use FastAPI for backends, React for frontends"

### Specs (Specifications)

**Specs** are ephemeral or semi-permanent artifacts tied to **specific features or changes**:

- Functional requirements for a particular feature
- Acceptance criteria for a user story
- Test scenarios for specific functionality
- Implementation details for one component

Think of specs as **temporary blueprints** - relevant only to the tasks that create or change that particular functionality.

**Examples**:

- "Password reset system: 30-minute token expiry, rate limiting, email-only recovery"
- "User profile page: display name, avatar, bio, edit functionality"
- "Payment checkout: support credit cards via Stripe, handle declined cards"

### The Key Difference

| Aspect         | Memory Bank                           | Spec                            |
| -------------- | ------------------------------------- | ------------------------------- |
| **Scope**      | Entire codebase                       | Specific feature/change         |
| **Lifetime**   | Permanent (or long-lived)             | Temporary or feature-lifetime   |
| **Applies to** | ALL development work                  | Only this feature's tasks       |
| **Changes**    | Rarely (represents stable principles) | Frequently (as feature evolves) |
| **Example**    | "Use bcrypt always"                   | "Password reset: 30-min expiry" |

**Why this matters**: When an AI agent works on your code, it should:

1. **Always reference** the memory bank (permanent rules)
2. **Only reference** the relevant spec (feature-specific details)

This distinction prevents confusion and ensures AI agents apply the right level of context to their work.

---

## Wait what is a Spec Anyway?

A **spec** (or specification) is a structured, behavior-oriented artifact, typically written in natural language, that clearly expresses a piece of software's functionality, constraints, and success criteria.

It is **more than just documentation**; it is an _executable_ contract that drives the entire development lifecycle. A good spec should be detailed enough to answer the _what_ (user stories, requirements, constraints) and inform the _how_ (technical context, integration points) without over-specifying the exact code implementation.

It is a formal description of what a system, function, or component should do. In the SDD context, it’s not just a text file — it’s a living document that drives generation, validation, and memory.

A good spec typically includes:

- Intent: What problem the system solves.
- Inputs and Outputs: Data formats, constraints, and expectations.
- Functional Requirements: What must always be true.
- Non-Functional Requirements: Performance, reliability, scalability, etc.
- Test Scenarios: Example inputs and expected outputs.
- Contextual Principles: Design philosophies, architecture rules, or “immutable laws.”

In essence, a spec isn’t a suggestion — it’s a constitution for your project. Let's move to lesson 3 and build your first Spec.