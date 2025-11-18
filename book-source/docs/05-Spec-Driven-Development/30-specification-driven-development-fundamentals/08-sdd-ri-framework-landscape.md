---
title: "SDD-RI Framework Landscape & What's Next"
chapter: 30
lesson: 8
duration: "45-60 minutes"
skills:
  - name: "Framework Context Assessment"
    proficiency: "B1"
    category: "Soft"
  - name: "Architectural Decision Reasoning"
    proficiency: "B1"
    category: "Technical"
learning_objectives:
  - "Compare four SDD approaches (Kiro, Spec-Kit, Spec-Kit Plus, Tesel) and understand their evolution (B1)"
  - "Assess context factors (team size, compliance, scale) that determine framework fit (B1)"
  - "Understand why this book teaches Spec-Kit Plus and how it implements SDD-RI concepts (B1)"
---

# SDD-RI Framework Landscape & What's Next

You've now completed your journey through SDD fundamentals (Lessons 1-5) and learned how to design reusable intelligence with RI (Lessons 6-7). Before you move into implementation in Chapter 31, it's critical to understand the landscape of SDD frameworks and recognize where Spec-Kit Plus — the framework this book teaches — fits within the larger ecosystem.

**This lesson is not about choosing a framework for YOUR projects** (that's a decision you'll make later with your own team constraints). Instead, this is **landscape awareness**: understanding four major SDD approaches that emerged in 2025, recognizing what context factors matter in framework selection, and understanding why Panaversity designed Spec-Kit Plus specifically for AI-native teams learning SDD-RI.

---

## Know Your Context

Before comparing frameworks, ground yourself in reality. **These three questions determine which tool actually fits.**

### Question 1: Who's Building This?

- **Solo developer** — Just you, learning or building side projects
- **Small team** — 2-5 people, probably in an early-stage company or internal project
- **Medium team** — 5-15 people, established project or startup scaling
- **Large team** — 15+ people across multiple services/domains

### Question 2: What's the Problem Scale?

- **Simple** — One feature, one service
- **Medium** — Multiple features, system integrations
- **Complex** — Multiple services, architectural decisions, or ongoing maintenance

### Question 3: Are There Regulatory/Compliance Constraints?

- **No constraints** — Build what works. Move fast. Iterate.
- **Light constraints** — "Best practices" (like 80% test coverage)
- **Heavy constraints** — Regulated domain (healthcare, finance, payments, aerospace)
- **Strict constraints** — FDA/SOX/HIPAA/PCI compliance required. Auditable decisions matter.

**Quick self-assessment**: Even though you won't select a framework today, answer these questions for your current understanding. These factors appear in every real framework decision.

---

## Four SDD Approaches

Four main SDD approaches emerged to serve different needs. Here's how to think about each one and when it fits.

### 1: Kiro — "Start Simple"

**Philosophy**: SDD shouldn't require learning complex processes.

**Workflow**: Simple requirements → Design → Tasks → Code

**Best For**:
- Solo developers or tiny teams (1-3 people)
- Learning SDD for the first time
- Building medium-sized features (not systems, not tiny bug fixes)

**Trade-off**: You get simplicity and low cognitive load. You sacrifice governance and consistency enforcement across projects.

---

### 2: Spec-Kit (GitHub's Framework) — "Strong Governance"

**Philosophy**: Immutable principles (Constitution) enforce consistency across everything.

**Workflow**: Constitution defines rules → Specify requirements → Plan architecture → Tasks → Code (with Constitution checks at each phase)

**Best For**:
- Teams of 5-50+ people where consistency matters
- Enterprise environments requiring strong governance
- Open-source frameworks (GitHub's official SDD standard)
- Projects where "all passwords use bcrypt" must be enforced

**Strengths**: Comprehensive traceability (requirements → plan → tasks → code → tests), scales to large teams, strong governance

**Limitations**: Doesn't document architectural "why" decisions, no AI interaction tracking, no built-in domain expertise

---

### 3: Spec-Kit Plus (Panaversity's Evolution) — "Spec-Kit + Intelligence"

**Philosophy**: Spec-Kit foundation + three critical intelligence layers for AI-native teams.

**What It Adds**:

1. **Architectural Decision Records (ADRs)** — Document the "why" behind choices (why JWT instead of sessions? why this database? why this language?)
2. **Prompt History Records (PHRs)** — Track every AI interaction (what prompt → what response → accepted/modified/why?) for compliance and learning
3. **Intelligence Templates** — Pre-built domain expertise (education, healthcare, fintech, aerospace) so teams don't rebuild domain-specific rules from scratch

**Best For**:
- Teams collaborating with AI agents (the focus of this book)
- Building systems that last years (long-term maintainability matters)
- Regulated domains where auditable decisions are critical
- Learning from AI interactions over time

**How It Implements SDD-RI Concepts**:
- ADRs document the reasoning you've been learning in Lessons 6-7 (why you chose skill X over subagent Y)
- PHRs capture the co-learning pattern you've seen throughout this book (AI suggests → you refine → you converge)
- Intelligence Templates encode the domain expertise you've been building

---

### 4: Tesel — "Specs as Source of Truth"

**Philosophy**: Take spec-driven development to its logical extreme—specs are the ONLY source of truth. Code is generated and never hand-edited.

**Workflow**: Write specification → Run code generator → Code produced (read-only) → Later: update spec → regenerate code

**Best For**:
- Safety-critical systems (aerospace, medical devices)
- Domains where code-spec divergence is unacceptable
- Still in private beta (invite-only in 2025)

---

## Why This Book Teaches Spec-Kit Plus

You might wonder: "If Kiro is simpler, shouldn't I start there? If Spec-Kit is GitHub's standard, why not use that?"

The answer lies in how this book teaches SDD-RI and what you need to succeed.

### The SDD-RI Choice: Why Spec-Kit Plus

**Chapter 30 (This Chapter)** teaches SDD fundamentals—the WHAT and HOW of specification-driven development.

**Chapter 31+ (Next)** teaches SDD-RI implementation—the WHY and HOW TO DESIGN alongside AI agents.

When you're learning to design reusable intelligence (L6-7) and orchestrate components (L8+), you need a framework that:

1. **Captures Your Reasoning (ADRs)**
   - In Lesson 7, you learned the Persona + Questions + Principles pattern for designing skills
   - ADRs are how you DOCUMENT that reasoning for your team and AI agents
   - Example: "Why did we create a git-workflow skill instead of a single git-commit subagent?" → ADR explains the tradeoff

2. **Logs AI Interactions (PHRs)**
   - Throughout this book, you've seen AI collaboration: suggest → refine → converge
   - PHRs are how you CAPTURE and LEARN from those interactions
   - Example: "Prompt A generated vulnerable code. Prompt B fixed it. Use B next time."

3. **Provides Domain Expertise (Intelligence Templates)**
   - You've been learning the Education template (Bloom's levels, CEFR proficiency, code requirements)
   - Templates prevent teams from rebuilding this knowledge every time
   - Example: "New healthcare team? Start with healthcare template. Know HIPAA rules before coding."

**These three additions are not optional overhead.** They are how AI-native teams learn, reason together, and scale knowledge across projects.

### The Pedagogical Choice

**This book chose Spec-Kit Plus because it's designed for exactly what you're learning:**

- Spec-Kit Plus assumes you're working with AI agents (not against them)
- Spec-Kit Plus tracks decisions and interactions (not just deliverables)
- Spec-Kit Plus includes domain templates (not generic, one-size-fits-all rules)

You're not learning abstract theory. You're learning the practical framework that professional AI-native teams use today.

---

## From Understanding to Implementation

### What You've Completed

- **Lessons 1-5**: SDD fundamentals — how to write clear specifications and plan implementations
- **Lessons 6-7**: RI concepts — how to design reusable intelligence, capture reasoning, recognize when to create skills vs subagents
- **Lesson 8**: Framework landscape — industry context and why Spec-Kit Plus fits AI-native teams

### What's Next 

Next chapter 31 takes everything you've learned and puts it into practice:

- **Build real features** using Spec-Kit Plus (not toy examples)
- **Practice the complete SDD-RI workflow**: Specification → ADR decisions → Prompt → Code → PHR logging → Validation
- **Experience framework-in-action**: See how ADRs capture your reasoning, how PHRs log AI interactions, how Intelligence Templates guide your domain choices
- **Compose components**: Use skills and subagents you've learned about, orchestrating them through specifications

### The Synthesis

```
Chapter 30 Complete
  └─ SDD Fundamentals (L1-5)
  └─ RI Concepts (L6-7)
  └─ Framework Awareness (L8)
       ↓
Chapter 31+ Implementation
  └─ Spec-Kit Plus in practice
  └─ Real feature development
  └─ SDD-RI workflow executed
```

**In Chapter 31, you'll see:**
- How specifications trigger Spec-Kit Plus phases
- How ADRs capture architectural reasoning
- How PHRs log AI interactions and improve future prompts
- How Intelligence Templates guide domain decisions
- How all of this works together in real, professional development

You're ready. Let's implement.

---

## Try With AI

**Setup**: Open your AI tool (Claude, ChatGPT, etc.) and explore these framework-awareness prompts. Remember: the goal is understanding the landscape, not making a framework choice for your project yet.

**Prompt 1: Explore Framework Tradeoffs**
> "I'm learning about four SDD frameworks (Kiro, Spec-Kit, Spec-Kit Plus, Tesel). My situation: [describe team size, problem scale, any compliance constraints]. For THIS situation, compare what each framework gives and sacrifices. Which seems like the best fit?"

**Expected Outcome**: Your AI should ask clarifying questions and explain tradeoffs specific to your context (governance vs simplicity, team size, compliance needs).

---

**Prompt 2: Understand Spec-Kit Plus Evolution**
> "Compare GitHub's Spec-Kit with Panaversity's Spec-Kit Plus. What three features does Spec-Kit Plus add? Why does each matter for teams working with AI agents?"

**Expected Outcome**: Your AI should explain ADRs (capture reasoning), PHRs (log interactions), and Intelligence Templates (domain expertise) and connect each to AI-native development.

---

**Prompt 3: Recognize When Frameworks Matter**
> "Explain when a solo developer should use Kiro vs Spec-Kit Plus. What threshold (team size, project complexity, AI agent usage) makes you switch from one to the other?"

**Expected Outcome**: Your AI should articulate decision factors: governance burden increases with team size, complexity justifies overhead, AI agent collaboration requires ADRs/PHRs.

---

**Prompt 4 (Optional Stretch): Apply to Your Context**
> "My project: [describe your actual situation]. Am I currently using any SDD framework? If I adopted one, which would fit best and why? What would I need to change about my current process?"

**Expected Outcome**: Concrete framework selection criteria based on YOUR situation (not abstract).

---
