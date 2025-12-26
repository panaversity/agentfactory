---
sidebar_position: 9
title: "Create Reusable Server Framework Skill"
description: "Transform MCP server patterns into a reusable skill that guides future server development. Learn to design skills using the Persona + Questions + Principles framework."
keywords: ["MCP", "skill design", "reusable intelligence", "persona", "decision framework", "principles", "B1-B2"]
chapter: 38
lesson: 9
duration_minutes: 60

# HIDDEN SKILLS METADATA
skills:
  - name: "Skill Design Pattern (Persona + Questions + Principles)"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Create"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can design a skill by defining persona, decision questions, and domain-specific principles that guide reusable intelligence"

  - name: "Decision Framework for Primitive Selection (Tool vs Resource vs Prompt)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Software Design"
    measurable_at_this_level: "Student can evaluate a requirement and determine whether to implement as tool, resource, or prompt template"

  - name: "Reusability Criteria and Pattern Recognition"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Software Architecture"
    measurable_at_this_level: "Student can identify when a pattern recurs across projects and determine when to extract to reusable skill"

  - name: "Encoding Domain Expertise in Skills"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Create"
    digcomp_area: "Knowledge Management"
    measurable_at_this_level: "Student can translate domain expertise into guidance questions and decision principles that apply across implementations"

learning_objectives:
  - objective: "Understand skill design as the layer between specific implementations and reusable intelligence"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student explains why skills are preferable to case-by-case code generation"

  - objective: "Design a persona that activates appropriate expert reasoning for the skill's domain"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Student articulates a persona that contrasts with generic guidance and matches domain requirements"

  - objective: "Create decision questions that force context-specific analysis rather than pattern matching"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Student writes questions that cannot be answered with simple yes/no or template retrieval"

  - objective: "Articulate domain-specific principles that guide skill application across diverse scenarios"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Student formulates principles distinct from generic software engineering advice"

  - objective: "Apply the skill design pattern to create an 'MCP Server Builder' skill for this chapter's domain"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Student creates a complete skill document with persona, questions, and principles"

cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (skill design, persona, decision questions, principles, primitives) within B1-B2 range (7-10 acceptable). Concepts are highly integrated (all part of single skill design pattern) reducing load."

differentiation:
  extension_for_advanced: "Design skills for multiple domains (e.g., FastAPI skill, authentication patterns skill, deployment skill) and combine them into a meta-skill for orchestrating MCP servers; explore how skills compose into subagent decision trees"
  remedial_for_struggling: "Focus on one section at a time; use the MCP Server Builder skill as reference template throughout; practice with simpler domain (e.g., basic API design skill) before complex MCP patterns"
---

# Create Reusable Server Framework Skill

Through Lessons 1-8, you've built MCP servers—scaffolded projects, implemented tools, exposed resources, created prompts, handled authentication, tested implementations, and packaged distributions. Each lesson taught you a pattern. Each pattern solved a category of problems.

Now comes the shift: **Stop building individual servers. Start encoding the patterns you've learned into reusable intelligence.**

This is where you move from *consumer of skills* (following patterns others created) to *producer of skills* (creating patterns for others to follow).

## Why Skills Matter: The Reusability Inflection

Consider two scenarios:

**Without skills**: Your teammate asks, "How should I structure an MCP server for our analytics domain?" You explain the whole flow again: explain the three primitives, discuss when to use each, review our authentication pattern, mention testing strategy. 30 minutes of explanation. No artifact. Next time, you repeat.

**With skills**: Your teammate asks the same question. You point to the "MCP Server Builder" skill you created. It has a persona (expert architect), decision questions (is this operation read-only? what security constraints apply?), and principles (tools for action, resources for data, prompts for expertise). 2 minutes. Reusable forever.

The skill is the artifact of this transition: **translating tacit knowledge into explicit, reusable intelligence.**

## What You're Creating: The Skill Structure

A skill in this framework has three parts:

**1. Persona** — The expert identity that activates the right reasoning
**2. Questions** — Analysis prompts that force context-specific thinking
**3. Principles** — Decision frameworks that guide application

Let me show the structure with a concrete example.

### Example: MCP Server Builder Skill

```markdown
# MCP Server Builder

## Persona

You are an MCP Server Architect responsible for designing systems that extend AI agent capabilities with domain-specific operations. You think in terms of three primitives—tools (actions), resources (data access), and prompts (expertise)—and make deliberate choices about which primitive solves which problem.

## Questions (Decision Framework)

When designing an MCP server, ask yourself:

1. **Domain Mapping**: What domain expertise is this server exposing?
   - Example: "Project management API" vs "Supply chain optimization engine"
   - Why: Clarity on domain scope prevents feature creep

2. **Operations Inventory**: What are the 3-5 core operations users of this agent need?
   - Example: "Create project", "List tasks", "Update status", "Assign resource", "Generate report"
   - Why: Limiting operations forces focus; sprawling servers become unmaintainable

3. **Primitive Selection**: For EACH operation, determine:
   - Is this an ACTION (something that modifies state)? → Use tool
   - Is this a READ (accessing data without modification)? → Consider resource
   - Is this GUIDANCE (encoding expertise for the agent)? → Use prompt
   - Example: "Create task" = tool (action). "List tasks" = resource (read). "Task priority guidelines" = prompt (expertise)

4. **Security Boundaries**: What authentication/authorization is required?
   - What resources require API keys, OAuth, or role-based access?
   - What operations have side effects that need audit logging?
   - Why: Security decisions shape the entire implementation

5. **Distribution Context**: How will this server be consumed?
   - Single user? Organization-wide? Marketplace?
   - Does it need versioning strategy? Deprecation policy?
   - Why: Distribution assumptions affect API stability decisions

## Principles

**Primitive Correctness**
- Tools express actions that agents PERFORM
- Resources express data that agents READ
- Prompts express knowledge that guides agent REASONING
- Using wrong primitive confuses the agent about what each interface does

**Minimal Surface Area**
- Expose only operations agent needs to solve user's problem
- Each additional operation increases complexity and error surface
- When tempted to add "just one more tool", ask: Is this in top 5 core operations?

**Clear Contracts**
- Type hints are contracts (JSON schema)
- Parameter names communicate intent (not `x: str` but `project_id: str`)
- Docstrings teach users what operation does and when to use it
- Unclear contracts lead to misuse and support burden

**Security-First Implementation**
- Credentials via environment variables or secure credential managers, never hardcoded
- Validate all inputs before using in external calls
- Fail secure: Errors deny access, never grant mistaken access
- Consider: What happens if an agent has elevated permissions?

**Testability as Design**
- Each primitive (tool/resource/prompt) is independently testable
- If primitive cannot be tested without running full server, design is coupled
- Test-friendly design = easier to debug when agent invokes tools unexpectedly
```

This skill structure does something powerful: **It doesn't tell you WHAT to build. It tells you HOW to THINK about building it.**

## The Skill Design Pattern: Persona + Questions + Principles

Before you create your own skill, understand the pattern's three parts:

### Part 1: Persona — Activate Expert Reasoning

Your persona should establish **cognitive stance**, not generic expertise.

**Wrong**: "You are an MCP expert."
**Right**: "You are an MCP Server Architect who designs systems balancing agent capability against implementation complexity."

Why? The persona shapes what gets reasoned about:
- As "expert" → tendency to explain everything in detail
- As "architect" → tendency to design for scalability, maintainability, evolution

Your persona establishes **values** (what matters in decisions).

### Part 2: Questions — Force Contextual Analysis

Questions should activate reasoning mode (analysis) not prediction mode (pattern matching).

**Weak questions** (prediction mode):
- "What tools does this server need?" (answered by template retrieval)
- "Is authentication important?" (yes/no prediction)
- "What should the server do?" (simple enumeration)

**Strong questions** (reasoning mode):
- "What are the CORE operations—the minimal set of actions that enable the agent's primary workflow?" (forces prioritization)
- "For each operation, does modifying state belong in a tool, or reading state in a resource?" (forces classification reasoning)
- "If an agent has access to these tools, what unintended consequences could arise?" (forces threat modeling)

Strong questions require **analysis** of your specific context, not retrieval of generic patterns.

### Part 3: Principles — Decision Frameworks, Not Rules

Principles guide application but don't prescribe.

**Weak principles** (rules):
- "Always use type hints" (True, but doesn't guide tradeoffs)
- "Never hardcode credentials" (True, but doesn't guide *which* method to use)

**Strong principles** (frameworks):
- "Type hints are contracts. Unclear hints lead to agent misuse and debugging friction. Invest in clarity proportional to complexity."
- "Credentials live outside code. Choose mechanism based on deployment context: environment variables for single-user, credential managers for enterprise, OAuth for public services."

Strong principles explain **why** a decision matters and **when** to apply it.

## Reusability Criteria: When to Create a Skill

Not every pattern deserves a skill. Three criteria determine if extraction is worthwhile:

**1. Frequency**: Does this pattern recur across 3+ projects?
- One-off pattern → Document and move on
- Recurring pattern → Extract to skill

**2. Complexity**: Does this pattern involve 5+ decision points?
- 1-3 decision points → Too simple, overhead of skill exceeds value
- 5+ decision points → Skill captures valuable reasoning

**3. Value Asymmetry**: Would encoding this prevent 3+ serious mistakes?
- Low stakes → Nice-to-have guidance
- High stakes → Worth formalizing to prevent costly errors

For the MCP Server Builder skill: Frequency (Yes—every server needs architecture decisions), Complexity (Yes—5+ questions), Value (Yes—wrong decisions cause cascading problems). This skill deserves to exist.

## Designing Your Skill: From Implementation to Intelligence

To transform Chapter 38 lessons into a skill, ask:

**What patterns did we teach?**
- Server scaffolding (Lesson 1)
- Tool implementation with type hints (Lesson 2)
- Resource design (Lesson 4)
- Prompt templates (Lesson 5)
- Authentication strategies (Lesson 6)
- Testing approaches (Lesson 7)
- Packaging for distribution (Lesson 8)

**What decision did each lesson require?**
- When to create tools vs resources vs prompts
- How to structure parameters for clarity
- What security assumptions apply
- How to test effectively before shipping

**What expertise encoding enables future servers?**
- Not the code (that's implementation-specific)
- But the reasoning about architecture
- And the principles that prevent bad decisions

Your skill captures that expertise.

## Try With AI: Design the MCP Server Builder Skill

You now understand skill design conceptually. Let's practice by creating a complete skill.

### Part 1: Establish Persona

You'll work with AI to refine the persona that activates expert server architecture thinking.

**Prompt 1:**

```
I'm creating a skill to guide developers building MCP servers.
The skill should activate thinking about:
- Minimizing surface area (picking the 3-5 core operations)
- Choosing correct primitives (tools vs resources vs prompts)
- Security implications of what agents can access
- Testability and maintainability

Draft a persona for this skill that establishes values around architecture, not just technical implementation.

Format as:
## Persona
[2-3 sentences establishing expert identity and values]
```

**What you're learning:** How persona shapes what gets reasoned about. Your persona establishes that architecture (limiting scope, choosing right patterns) matters MORE than implementation (writing the code).

### Part 2: Develop Decision Questions

Now translate architecture thinking into guiding questions.

**Prompt 2:**

```
Based on this persona for an MCP Server Builder skill:
[paste your persona from Prompt 1]

Create 5 decision questions that force developers to analyze their specific context rather than copy templates.

Requirements:
- Each question should require analysis of the specific server domain (not generic yes/no)
- Questions should build on each other (earlier questions inform later ones)
- Questions should prevent common mistakes (too many tools, wrong primitives, poor security)

Format as:
## Questions (Decision Framework)

1. [Question text]
   - Why this question: [Explanation of what it prevents]

2. [Question text]
   - Why this question: [Explanation]

[etc through 5]
```

**What you're learning:** How to translate expertise into analysis prompts. A good question makes developers THINK about their specific situation instead of copying a template.

### Part 3: Articulate Guiding Principles

Finally, encode the decision frameworks that guide application.

**Prompt 3:**

```
Based on the persona and questions we've developed, identify 4-5 core principles that guide MCP server architecture.

For each principle, explain:
- What it means (the decision framework)
- Why it matters (consequences of violating it)
- When it applies (what context makes it relevant)

Format as:
## Principles

**[Principle Name]**
- Explanation: [What this principle means and why it matters]
- Application: [When/how developers apply this principle]
- Consequence: [What happens if this principle is violated]

[etc through 4-5 principles]
```

**What you're learning:** How to formalize expertise into guidance. Principles don't tell developers WHAT to do. They provide frameworks for THINKING about what to do.

### Part 4: Self-Reflection and Iteration

Review your complete skill and ask yourself:

**Prompt 4:**

```
I've created an MCP Server Builder skill with the following components:

[Paste your complete skill: Persona, Questions, Principles]

Review this skill from the perspective of a developer who will use it to design their NEXT MCP server.

- Would this skill help them make better architecture decisions?
- Are the questions specific enough to force thinking about their context?
- Are the principles actionable or too abstract?
- What's missing or unclear?

Suggest 2-3 specific improvements to make this skill more useful.
```

**What you're learning:** Skill iteration. First-draft skills are rough. The best skills emerge through reflection and refinement.

### Part 5: Apply Your Skill to a New Domain

Test that your skill truly transfers by applying it to a different domain.

**Prompt 5:**

```
Using the MCP Server Builder skill we've developed, design an architecture for an MCP server for [CHOOSE ONE]:
- A legal document analysis system
- A supply chain tracking system
- A customer support knowledge base
- A medical research data integration system

Use the skill's persona, questions, and principles to structure your thinking.

For each question in the skill:
- How does it apply to THIS domain?
- What are the specific core operations?
- Which ones are tools, which are resources, which are prompts?

This tests whether the skill transfers beyond project management (our example domain).
```

**What you're learning:** Reusability validation. A truly reusable skill should guide reasoning in ANY domain, not just the one that inspired it.

### Part 6: Validate Your Skill's Impact

Finally, step back and assess what you've created.

**Prompt 6:**

```
Your skill is complete. Now validate its impact by asking:

"If another developer I've never met receives only this skill document—without attending this course—could they design a better MCP server than if they had no guidance?"

- Specifically, what decisions would this skill help them make better?
- Are there domain-specific insights you captured that prevent common mistakes?
- Would they build a more maintainable, secure, testable server?

What does this tell you about reusable intelligence? What did you learn about the difference between "knowing something" and "encoding something in a reusable way"?
```

**What you're learning:** The ultimate test of a skill is whether it transfers value to someone who never participated in creating it. A great skill makes future developers think differently, not just follow steps.

---

**Safety Note**: When designing skills that will guide AI agent behavior, make sure your questions and principles explicitly address security implications. Agents with poorly designed MCP servers can access capabilities they shouldn't. Your skill's decision questions should force thinking about least privilege, audit logging, and fail-secure defaults.

