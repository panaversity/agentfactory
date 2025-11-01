---
title: "Ask Your Companion: What Is Specification-Driven Development?"
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

# Ask Your Companion: What Is Specification-Driven Development?

In Lesson 1, you discovered that vague specs fail. Your companion built something that technically worked but didn't match your intent.

**Here's the key insight you discovered**: Clarity prevents miscommunication.

Now the question is: **What methodology turns this insight into professional practice?**

**That methodology is Specification-Driven Development (SDD).**

But instead of me explaining it, you're going to ask your companion.

---

## Your Collaboration: Discover SDD Together

### Step 1: Ask Your Companion to Explain SDD

Open your AI companion (Claude, ChatGPT, Gemini, etc.) and ask:

```
I just learned that vague specifications lead to bad code, and clear
specifications lead to good code.

What is Specification-Driven Development? How is it different from
the code-first approach I've been using?

Explain it like I'm learning it for the first time.
```

**Let your companion explain.** Read the explanation. Ask follow-up questions if something's unclear.

Your companion will likely say something like:

> "SDD is an approach where you write a detailed specification BEFORE
> writing code. The spec serves as the source of truth. Your AI partner
> (me) uses the spec to generate correct code."

### Step 2: Ask About the Three Levels

Now ask:

```
I've heard there are different "levels" of SDD. Can you explain:
- Spec-first
- Spec-anchored
- Spec-as-source

When would you use each one?
```

Your companion will explain something like:

**Spec-first**: Write spec, use it once, discard it
- Example: Building a one-off feature
- When to use: Rapid prototypes, experimental code

**Spec-anchored**: Write spec, keep it updated as code evolves
- Example: Feature that will change over time
- When to use: Team development, long-lived systems

**Spec-as-source**: Specs are the ONLY thing you edit; code is always generated
- Example: Medical device software (regulatory requirement)
- When to use: Safety-critical, heavily regulated systems

### Step 3: Ask About the SDD Loop

Now ask:

```
What does the actual workflow look like? Walk me through the steps
from "I have an idea" to "code is ready to deploy."
```

Your companion will describe something like:

```
1. SPECIFY: Write the specification (what are we building?)
2. PLAN: Design the architecture (how will we build it?)
3. TASKS: Break into small tasks (what are the work units?)
4. IMPLEMENT: Generate the code (write or auto-generate)
5. VALIDATE: Test against the spec (does it work?)
```

**Draw this out.** Visualize it. Understand that specs come BEFORE code, not after.

### Step 4: Ask About Cost-Benefit

Now the practical question:

```
I'm worried specs will slow me down. Does writing specifications
actually save time or cost time? Give me a realistic comparison.
```

Your companion will say something like:

> "Upfront: Writing specs takes 2-6 hours (more upfront time).
>
> Downstream: Code generation is faster, fewer iterations, less rework.
>
> Total: For features >8 hours of work, specs save time. For throwaway
> code, specs cost more than they save."

---

## What You Just Learned (Through Discovery)

By asking your companion, you've discovered:

### ✅ SDD Definition
Specifications are written BEFORE code. They're the source of truth that drives implementation.

### ✅ Three Levels
- **Spec-first**: One-time use
- **Spec-anchored**: Maintained alongside code
- **Spec-as-source**: Only thing edited (rare)

### ✅ The SDD Loop
Idea → Spec → Plan → Tasks → Implementation → Validation

### ✅ Cost-Benefit
Specs cost upfront time but save downstream time through reduced rework and iteration.

---

## Why You Learned It This Way

Notice what happened: **You didn't read a lecture. You asked questions.**

This is crucial because:

1. **You own the understanding** — Your companion's explanation is connected to YOUR question, YOUR confusion
2. **You can follow up** — If something's unclear, you ask again (not possible with a lecture)
3. **You discover nuance** — Your companion can explain "spec-anchored vs spec-first" and give examples, adjusted to your level
4. **You model how professionals work** — Professional developers ASK their tools/experts questions. They don't passively read textbooks.

---

## Going Deeper

Your companion has basic understanding now. But there are insights that deepen this. Ask:

### Insight 1: Why AI Agents Need Specs

```
Why do AI agents specifically benefit from specifications?
What's different about how AI agents work compared to human colleagues?
```

Your companion will explain (or you'll learn):

> "AI agents are literal-minded. They excel at pattern matching but can't
> infer your intent. Human colleagues can ask clarifying questions and
> use context. AI agents need explicit, unambiguous instructions.
> Specifications provide that."

### Insight 2: The History Context

```
Specifications have been around since the 1970s (formal methods, MDD, etc).
Why is SDD only becoming standard NOW, in 2024? What changed?
```

Your companion will connect:

> "AI coding agents got good enough to generate production code from specs.
> That didn't exist before 2023. Before, specs were nice but not essential
> because code generation wasn't reliable. Now it is. Specs became essential."

### Insight 3: Memory Banks and Constitutions

```
You mentioned specifications, but what about rules that apply to ALL code
(not just one feature)? How do those work?
```

Your companion will introduce:

> "Memory banks (or Constitutions) are persistent rules that apply to
> everything: 'all passwords use bcrypt,' 'all APIs are rate-limited,'
> 'all code has 80% test coverage.' Specs are feature-specific. Memory
> banks are system-wide."

---

## Your Reflection: Connect to Your Experience

Now reflect on what you've learned:

**Question 1**: In your own work, where would SDD help most?
- A project where miscommunication caused rework?
- A team where coordination was hard?
- Work with AI agents that didn't understand what you wanted?

**Question 2**: Which SDD level fits YOUR context?
- Solo developer? (Spec-first probably enough)
- Team of 5? (Spec-anchored makes sense)
- Regulated domain? (Constitutional governance)

**Question 3**: What would it change?
- How would you start a project differently?
- How would you communicate with teammates?
- How would you work with AI agents?

Write down your thoughts. You'll return to them in Lesson 8.

---

## Key Takeaway

**SDD isn't mysterious.** It's systematic:

Write clear specs (the source of truth) → Generate code from specs → Validate code against specs → Update specs as requirements change → Everything else follows

Your companion helped you understand this through conversation, not lecture.

**That's how professionals learn too.**

---
