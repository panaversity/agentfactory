---
sidebar_position: 6
title: "Three Requirements for Vertical Success"
description: "All three elements are necessary. Missing any one, and you fail."
reading_time: "2 minutes"
chapter: 3
lesson: 6
duration_minutes: 12

# HIDDEN SKILLS METADATA
skills:
  - name: "Recognizing Critical Success Factors"
    proficiency_level: "A1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can identify three non-negotiable requirements for vertical market success"

  - name: "Understanding Interdependencies"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain why missing any one requirement dooms the entire venture"

  - name: "Evaluating Personal Readiness"
    proficiency_level: "A2"
    category: "Soft"
    bloom_level: "Understand"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student can assess whether they have (or can develop) all three capabilities"

learning_objectives:
  - objective: "Identify the three requirements: domain expertise (via fine-tuned models or vertical intelligence), deep integrations, complete agentic solutions"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Recognition and description of each requirement, including two paths for domain expertise"

  - objective: "Understand why all three are interdependent and why missing any one causes failure"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Analysis of failure case (OpenAI Study Mode) showing missing requirements"

  - objective: "Evaluate the feasibility of building all three elements with available resources"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Assessment of personal capability to develop fine-tuned models, integrations, and solutions"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (fine-tuning, vertical intelligence, sub-agents/skills, integrations, agentic solutions) within A2 limit (max 7) ✓"

differentiation:
  extension_for_advanced: "Research model fine-tuning techniques; analyze real integrations in chosen vertical"
  remedial_for_struggling: "Focus on understanding why 'all-or-nothing' requirement is necessary before diving into technical details"
---

# Three Requirements for Vertical Success: All Three, or None

You now have strategy (Snakes and Ladders), economics (super orchestrators), architecture (vertical intelligence), and a playbook (PPP). But execution requires three capabilities working in perfect sync. Lack any one, and you fail.

## Requirement 1: Increase Domain Expertise with Fine-Tuned Models and/or Vertical Reusable Intelligence

Your subagents must be smarter than general-purpose AI. A general ChatGPT conversation does anything at 70% quality. Your finance subagent must do portfolio analysis at 99% quality—money is at stake. Your healthcare subagent must diagnose at 99% accuracy—lives are at stake.

**There are two paths to achieving this 99% domain expertise:**

1. **Fine-tune the underlying AI model** on domain-specific data (expensive, but powerful)
2. **Build vertical reusable intelligence** through specialized sub-agents and agent skills (faster to iterate, more flexible)

Both paths work. The choice depends on your resources, timeline, and the characteristics of your vertical market. Many successful companies use both together.

### Path 1: Fine-Tuned Models

Fine-tuning means training the underlying model (Claude, Gemini, GPT-4, or others) on domain-specific data: financial earnings reports, healthcare clinical literature, education curriculum standards. The model learns the language, patterns, and nuances of your domain at a deep level.

**Example**: A healthcare AI fine-tuned on 100,000 clinical papers will recognize medical terminology, understand drug interactions, and follow treatment protocols that a general AI would miss.

**Strengths**:
- Deeply understands domain language and patterns
- Handles ambiguity better (learned from thousands of examples)
- Less prompt engineering needed

**Challenges**:
- Requires large domain-specific datasets (thousands of high-quality examples)
- Expensive to create and update (retraining costs time and money)
- Longer iteration cycles (weeks to retrain vs. hours to update prompts)

### Path 2: Vertical Reusable Intelligence with Sub-agents and Agent Skills

Instead of training the model, you encode domain expertise in specialized prompts, workflows, and integration logic. Think of it as building a "skill library" that teaches general AI how to behave like a domain expert.

**How it works**:

- **Sub-agents**: Specialized AI assistants for specific domain tasks. Your finance orchestrator might have sub-agents for: portfolio analysis, risk assessment, trade execution, and compliance checking. Each sub-agent has domain-specific prompts and tools.

- **Agent skills**: Reusable capabilities that give AI domain-specific knowledge. This book uses this approach—the `.claude/skills/` directory contains skills like `learning-objectives`, `assessment-builder`, `code-example-generator` that provide pedagogical expertise without fine-tuning the model.

**Example**: Instead of fine-tuning on 100,000 clinical papers, you build:
- A "diagnosis sub-agent" with access to drug databases and clinical guidelines (via MCP tools)
- A "treatment planning skill" that knows hospital protocols and insurance rules
- Integration workflows that validate outputs against FDA regulations

The AI stays general-purpose, but your *system* has domain expertise built into its structure.

**Strengths**:
- Faster to build and iterate (update prompts/workflows in hours, not weeks)
- More transparent and debuggable (you can see exactly what knowledge the system has)
- Easier to update when domain rules change (update a skill, not retrain a model)
- Works well when expertise is procedural ("follow these steps") rather than pattern-based

**Challenges**:
- Requires careful prompt engineering and workflow design
- May need more tokens per request (longer prompts)
- Less effective for highly ambiguous domains where pattern recognition is critical

**Why it's defensible**: Just like fine-tuned models, vertical intelligence takes months to build. Your competitors must replicate all your prompts, skills, integration workflows, and domain-specific validation rules. A well-architected skill library is as defensible as a fine-tuned model.

### Choosing Your Path

**Use Path 1 (Fine-Tuning) when**:
- You have access to large, high-quality domain datasets
- Your domain has subtle patterns that are hard to encode in rules
- You need the AI to "sound like" domain experts naturally
- You have budget for training and retraining

**Use Path 2 (Vertical Intelligence) when**:
- Domain expertise can be captured in structured workflows
- You need faster iteration (startup moving quickly)
- Fine-tuning data is limited or expensive to acquire
- Your domain has clear rules and procedures to follow

**Use both when**:
- You have resources for fine-tuning AND need fast iteration
- Different subagents need different approaches (e.g., fine-tune the diagnosis subagent, use skills for the compliance subagent)

**Without domain expertise (via either path)**, you're just a thin wrapper around general AI. Competitors replicate you in weeks.

## Requirement 2: Deep Integrations with Existing Systems

Your subagent must speak the language of incumbent systems. Not just *read* data from them, but *write back* in ways that respect workflows, security models, and approval processes.

A healthcare subagent that reads from Epic but can't write clinical notes in the right format is useless. A finance subagent that reads Bloomberg but can't execute trades through proper channels is a demo, not a product.

These integrations are expensive (months of API documentation, regulatory compliance, security audits) and they're defensible (competitors must rebuild them).

Without this, you're building in a sandbox, not serving real customers.

## Requirement 3: Complete Agentic Solutions

Your subagent must solve an *end-to-end* problem, not a slice of one. A healthcare subagent that reads clinical literature but doesn't integrate with hospital systems is a curiosity. A healthcare subagent that reads EHR, clinical literature, insurance rules, *and* FDA regulations, then recommends treatment plans doctors can act on immediately—that's a product.

This means coordinating five components (system prompt, horizontal skills, vertical skills, horizontal MCPs, vertical MCPs) in a workflow that makes sense to your customer.

Without this, you're a toy. With this, you're indispensable.

## The OpenAI Lesson: Study Mode

Consider OpenAI's Study Mode experiment (2024). OpenAI has:

- **Requirement 1: Domain Expertise** — Partially. GPT-4 is state-of-the-art general AI, but lacks education domain expertise. OpenAI didn't fine-tune on curriculum standards or build education-specific sub-agents with pedagogical skills.
- **Requirement 2: Integrations** — Partially. Study Mode integrates with some LMS platforms, but not deeply (Canvas, Google Classroom API, but not the full ecosystem)
- **Requirement 3: Agentic solution** — Partially. Study Mode can answer questions, but it doesn't adapt learning paths, doesn't coordinate with teacher workflows, doesn't integrate grade books

Result: Study Mode is a feature, not a product. Teachers use it occasionally. It doesn't replace their workflow.

A PPP strategy would have chosen **either** path for Requirement 1:
1. **Path 1**: Fine-tuned GPT-4 on education data (curriculum standards, lesson plans, student work samples)
2. **Path 2**: Built vertical intelligence—sub-agents for: lesson planning, adaptive learning, grading, differentiation—each with education-specific skills and tools

Then layered:
- Deep integrations with *all* major LMS platforms (Requirement 2)
- A complete solution: adaptive learning + teacher assistant + grading automation (Requirement 3)

With all three, Study Mode would be a $10M+ annual revenue business. Without all three, it's a feature OpenAI ships once and deprioritizes.

## The Consequence of Missing Any Element

If you have domain expertise + integrations but NO agentic solution, you're just a data pipeline. Useful, but not transformative.

If you have domain expertise + agentic solution but NO integrations, you're building in a sandbox. No real customer workflows.

If you have integrations + agentic solution but NO domain expertise (via fine-tuning OR vertical intelligence), you're a wrapper around general AI. Competitors replicate in weeks.

**All three elements must work together.** This is why PPP matters: it systematically builds all three. Phase 1 (infrastructure layering) addresses integrations. Phase 2 (market validation) provides domain expertise—you can collect data for fine-tuning OR build vertical intelligence through sub-agents and skills. Phase 3 (strategic pivot) layers the agentic solutions.

---

## Try With AI

Use your AI companion tool set up (e.g., ChatGPT web, Claude Code, Gemini CLI), you may use that instead—the prompts are the same.

### Prompt 1: Prioritize The Three Requirements
```
The lesson says I need THREE requirements for success: (1) fine-tuned models, (2) deep integrations, (3) complete agentic solutions. Rank these from EASIEST to HARDEST for a beginner to achieve. For each one, explain: what would the first small step look like? Don't overwhelm me—just give me ONE concrete action per requirement.
```

**Expected outcome**: Prioritized roadmap: which requirement to tackle first (with first steps).

### Prompt 2: Learn From Failure Cases
```
The OpenAI Study Mode example shows what happens when you're missing requirements. Help me learn from this: Pick a different product or startup (maybe one I've heard of) that FAILED because it was missing one of the three requirements. Explain what they had, what they lacked, and what the lesson is for me.
```

**Expected outcome**: Pattern recognition from real failure case studies.

### Prompt 3: Choose Between Two Paths
```
The lesson says there are TWO ways to get domain expertise: (1) fine-tuned models or (2) vertical reusable intelligence with sub-agents and skills. I'm confused about which one is right for me. Help me understand: If I wanted to build a solution for [pick: teachers / small business owners / healthcare workers / legal professionals], which path should I start with? Give me concrete reasons based on my situation (I'm a beginner with limited budget).
```

**Expected outcome**: Personalized recommendation on whether to pursue Path 1 (fine-tuning) or Path 2 (vertical intelligence) based on your vertical and resources.

### Prompt 4: Demystify Model Fine-Tuning
```
I'm confused about 'fine-tuning models with domain expertise.' Explain this like I'm 10 years old. Then give me a realistic example: If I wanted to build a solution for [pick: teachers / small business owners / healthcare workers], what would 'fine-tuning' actually involve? Do I need to be a data scientist?
```

**Expected outcome**: Beginner-friendly understanding of model fine-tuning (without PhD-level jargon).

### Prompt 5: Assess Incremental vs. All-Or-Nothing
```
The lesson says 'all three or none'—meaning if I'm missing even ONE requirement, I fail. But that feels extreme. Help me understand: could I start with just ONE requirement (maybe integrations) and gradually add the others? Or is there really no middle ground? Give me an honest answer with reasoning.
```

**Expected outcome**: Honest assessment of whether you can build incrementally or need all three upfront.






