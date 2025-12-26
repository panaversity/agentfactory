---
sidebar_position: 8
title: "Three Requirements for Vertical Success"
description: "All three elements are necessary. Missing any one, and you fail."
reading_time: "2 minutes"
chapter: 3
lesson: 8
duration_minutes: 18

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
    measurable_at_this_level: "Student can assess whether they have (or can develop) all three capabilities and identify mitigation strategies"

learning_objectives:
  - objective: "Identify the three requirements: domain expertise (via fine-tuned models or vertical intelligence), deep integrations, complete agentic solutions"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Recognition and description of each requirement, including two paths for domain expertise"

  - objective: "Understand why all three are interdependent and why missing any one causes failure"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Analysis of failure case (OpenAI Study Mode) showing missing requirements"

  - objective: "Evaluate the feasibility of building all three elements with available resources and create mitigation plans for capability gaps"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Self-assessment checklist (1-10 scale) plus identification of concrete mitigation strategies for identified gaps"

cognitive_load:
  new_concepts: 1
  assessment: "1 cohesive framework (three requirements as interdependent system) within A2 limit ✓"

differentiation:
  extension_for_advanced: "Research model fine-tuning techniques; analyze real integrations in chosen vertical; develop detailed mitigation strategies"
  remedial_for_struggling: "Focus on understanding why 'all-or-nothing' requirement is necessary; work through each requirement separately before assessing interdependencies"
---

# Three Requirements for Vertical Success: All Three, or None

**Lesson Video:**

<iframe width="100%" height="400" src="https://www.youtube.com/embed/Cot9wISQivk" title="Three Requirements for Vertical Success" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

You've mapped your PPP strategy and understand how to enter a vertical market. But execution requires three capabilities working in perfect sync. Lack any one, and you fail.

![Venn diagram showing three overlapping requirements for vertical market success: domain expertise, deep integrations, and complete agentic solutions, with center intersection labeled "SUCCESS ZONE" illustrating that all three must be present simultaneously](https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/images/part-1/chapter-3/three-requirements-venn-diagram.png)

## Requirement 1: Increase Domain Expertise with Fine-Tuned Models and/or Vertical Reusable Intelligence

Your Custom Agents must be smarter than general-purpose AI. A general ChatGPT conversation does anything at 70% quality. Your finance Custom Agent must do portfolio analysis at 99% quality because money is at stake. Your healthcare Custom Agent must diagnose at 99% accuracy because lives are at stake.

**There are two paths to achieving this 99% domain expertise:**

1. **Fine-tune the underlying AI model** on domain-specific data (expensive, but powerful)
2. **Build vertical reusable intelligence** through specialized sub-agents and agent skills (faster to iterate, more flexible)

Both paths work. The choice depends on your resources, timeline, and the characteristics of your vertical market. Many successful companies use both together.

![Decision diagram showing two paths to achieving domain expertise for AI sub-agents: Path 1 (Fine-Tuned Models) for deep pattern recognition with expensive updates, and Path 2 (Vertical Intelligence with sub-agents and skills) for fast iteration with transparent design, helping entrepreneurs choose based on resources and timeline](https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/images/part-1/chapter-3/two-paths-domain-expertise.png)

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
- Longer iteration cycles (weeks to retrain vs hours to update prompts)

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

### Why Intelligence Is the New Competitive Asset

Remember Lesson 3 and the Instagram story: 13 employees built a $1 billion company. How? Because they accumulated intelligence—deep understanding of why people share photos, what features drive engagement, how to prioritize through noise. That accumulated knowledge was their moat. Competitors with more employees and resources couldn't replicate what Instagram understood about human behavior because understanding takes time.

In AI-driven markets, the same principle applies. Intelligence (accumulated domain knowledge) has replaced effort as the source of competitive advantage. A generic AI system available to everyone creates no defensibility. But AI enhanced with your months of accumulated knowledge—whether encoded as fine-tuned models or vertical intelligence—creates a barrier competitors cannot quickly overcome.

Why? Because replication requires more than copying features. It requires understanding thousands of domain decisions: Which patterns matter in your vertical? What edge cases require special handling? How do workflows in healthcare differ from finance? What validation rules protect against errors? These decisions accumulate over months. Competitors starting fresh must rebuild not just your product, but your entire understanding of the domain. A company without domain expertise can copy your features in weeks. But they cannot copy months of accumulated knowledge.

Both Path 1 (fine-tuned models) and Path 2 (vertical intelligence) are defensible for exactly this reason. A fine-tuned model embeds pattern recognition learned from thousands of examples in your domain. A vertical intelligence system embeds decision logic learned from months of workflow analysis and integration design. In either case, competitors face the same problem: they must invest months rebuilding what you've learned. Your moat isn't the code or the prompts—it's the accumulated intelligence those systems encode.

This is why missing Requirement 1 is fatal. Without domain expertise, you're distributing generic AI wrapped differently. Every competitor has access to the same general-purpose models. Your only differentiation is packaging, which others replicate in weeks. But with domain expertise—however you encode it—you create defensibility that mirrors Instagram's: accumulated knowledge competitors must rebuild themselves.

When you choose between Path 1 and Path 2, you're not choosing defensibility (both paths create it). You're choosing implementation strategy. Fine-tuning requires large datasets and longer iteration cycles. Vertical intelligence requires careful workflow design but enables faster updates. Choose based on your resources and timeline, not defensibility. Both paths build the moat.

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

#### 🎓 Expert Insight

> The choice between Path 1 (fine-tuning) and Path 2 (vertical intelligence) isn't binary. It's strategic. Fine-tuning excels at pattern recognition (medical diagnosis from imaging, legal document analysis). Vertical intelligence excels at procedural workflows (insurance claim routing, compliance checking). Many successful companies use both: fine-tune the AI model for domain language understanding, then layer vertical intelligence (sub-agents, skills, MCPs) for specific workflows. The key insight: domain expertise is non-negotiable, but how you encode it is flexible.

## Requirement 2: Deep Integrations with Existing Systems

Your Custom Agent must speak the language of incumbent systems. Not just read data from them, but write back in ways that respect workflows, security models, and approval processes.

A healthcare Custom Agent that reads from Epic but can't write clinical notes in the right format is useless. A finance Custom Agent that reads Bloomberg but can't execute trades through proper channels is a demo, not a product.

These integrations are expensive (months of API documentation, regulatory compliance, security audits) and they're defensible (competitors must rebuild them).

Without this, you're building in a sandbox, not serving real customers.

#### 💬 AI Colearning Prompt

> **Explore with your AI**: "The lesson says deep integrations are 'expensive and defensible.' Let's test this—ask your AI: 'Give me an example of a company that built deep integrations as their moat. How long did it take competitors to replicate those integrations?' Then challenge it: Are integrations ACTUALLY defensible in the API economy, or can competitors just build integrations quickly?"

## Requirement 3: Complete Agentic Solutions

Your Custom Agent must solve an end-to-end problem, not a slice of one. A healthcare Custom Agent that reads clinical literature but doesn't integrate with hospital systems is a curiosity. A healthcare Custom Agent that reads EHR, clinical literature, insurance rules, and FDA regulations, then recommends treatment plans doctors can act on immediately; that's a product.

This means coordinating five components (system prompt, horizontal skills, vertical skills, horizontal MCPs, vertical MCPs) in a workflow that makes sense to your customer.

Without this, you're a toy. With this, you're indispensable.

## The OpenAI Lesson: Study Mode

Consider OpenAI's Study Mode feature (launched July 2025). OpenAI has:

- **Requirement 1: Domain Expertise**: Partially. GPT-4 is state-of-the-art general AI, but lacks education domain expertise. OpenAI didn't fine-tune on curriculum standards or build education-specific sub-agents with pedagogical skills.
- **Requirement 2: Integrations**: Partially. Study Mode integrates with some LMS platforms, but not deeply (Canvas, Google Classroom API, but not the full ecosystem)
- **Requirement 3: Agentic solution**: Partially. Study Mode can answer questions, but it doesn't adapt learning paths, doesn't coordinate with teacher workflows, doesn't integrate grade books

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

All three elements must work together. This is why PPP matters: it systematically builds all three. Phase 1 (infrastructure layering) addresses integrations. Phase 2 (market validation) provides domain expertise (you can collect data for fine-tuning or build vertical intelligence through sub-agents and skills). Phase 3 (strategic pivot) layers the agentic solutions.

---

## Readiness Assessment: Where Are You Today?

Before you commit to building a Digital FTE in your vertical, assess your current capability across all three requirements.

### Assessment Checklist (Rate 1-10 for Each)

**Requirement 1: Domain Expertise**

- Do you have deep knowledge of your vertical's workflows, pain points, and terminology? (1-10: ___)
- Can you articulate the 5-10 most important domain decisions your Custom Agent will need to make? (1-10: ___)
- Do you have or can you access domain-specific datasets (for fine-tuning) OR can you encode workflows (for vertical intelligence)? (1-10: ___)
- **Total Domain Expertise Score**: Average of three ratings above

**Requirement 2: Integration Capability**

- Do you understand the major incumbent systems in your vertical and their APIs? (1-10: ___)
- Do you have or can you develop relationships with system vendors (for technical documentation, support)? (1-10: ___)
- Can you realistically build integrations with 3-5 major platforms in 3-6 months? (1-10: ___)
- **Total Integration Score**: Average of three ratings above

**Requirement 3: Agentic Solution Design**

- Can you specify what end-to-end workflow your Custom Agent will automate for customers? (1-10: ___)
- Do you understand how to architect sub-agents and skills to solve complete problems (not just slices)? (1-10: ___)
- Can you articulate how your solution saves customers time, money, or risk? (1-10: ___)
- **Total Solution Design Score**: Average of three ratings above

**Overall Readiness**: If all three scores are 7+, you're ready to start. If any score is below 5, you have a critical gap.

---

## Mitigation Strategies for Capability Gaps

### If Domain Expertise Score is Low (< 7)

**Mitigation Strategy A: Partner with Domain Expert**
- Recruit a co-founder or advisor with 5-10+ years of domain experience
- They guide your architecture and validate domain decisions
- Cost: Equity stake (10-15%)
- Timeline: 1-2 months to recruit and align

**Mitigation Strategy B: Embedded Learning**
- Spend 3-6 months in your vertical: work part-time at a company, attend industry conferences, interview 20-30 practitioners
- This builds domain knowledge directly while validating market opportunity
- Cost: Time (3-6 months of reduced coding)
- Timeline: 3-6 months, then execute PPP with much stronger foundation

**Mitigation Strategy C: Strategic Data Collection**
- If pursuing fine-tuning (Path 1): Start with publicly available domain data, then negotiate access to customer data during Phase 2
- If pursuing vertical intelligence (Path 2): Document your first 5-10 customer workflows in detail, encoding patterns into skills/sub-agents
- Cost: Time to organize data
- Timeline: Parallelize with Phase 1 and Phase 2 execution

### If Integration Capability Score is Low (< 7)

**Mitigation Strategy A: Start with One Platform**
- Don't try to bridge 5 platforms in Phase 1
- Start with the most popular platform in your vertical, build a perfect integration
- Expand to additional platforms in Phase 2 once you understand integration patterns
- Cost: Slower customer acquisition in Phase 1
- Timeline: Still 3-6 months, but with 1-2 platforms instead of 5

**Mitigation Strategy B: Leverage MCP Pre-Built Connectors**
- The MCP (Model Context Protocol) ecosystem is growing with pre-built connectors for popular platforms
- Use these as starting points instead of building from scratch
- Cost: Potentially free (open-source) or subscription (commercial MCPs)
- Timeline: Reduces Phase 1 from 4-6 months to 2-3 months

**Mitigation Strategy C: Become a Platform Specialist**
- If you lack integration expertise, partner with an integration specialist or contractor
- They build the integration layer while you focus on domain expertise and sub-agents
- Cost: $10-30K per platform integration (budget 3-5 platforms = $30-150K)
- Timeline: Can parallelize with your other work

### If Solution Design Score is Low (< 7)

**Mitigation Strategy A: Customer Discovery First**
- Before architecting your Custom Agent, spend 2-3 months interviewing 10-15 target customers
- Document their end-to-end workflows, bottlenecks, and what they'd pay to solve each
- This clarifies what "complete agentic solution" means for YOUR vertical
- Cost: Time
- Timeline: 2-3 months of customer research before Phase 1

**Mitigation Strategy B: Build Incrementally, Validate Continuously**
- Don't design the complete 5-component solution upfront
- Start with one sub-agent solving one critical workflow
- Deploy with Phase 2 customers, measure impact, iterate
- Layer additional sub-agents only after validating first one works
- Cost: Slower feature velocity in Phase 1-2
- Timeline: More iterations, but risk-reduced

**Mitigation Strategy C: Study Competitor Solutions**
- Find 2-3 competitors solving similar problems (even in adjacent verticals)
- Analyze their architectures: What do they automate? What don't they?
- Apply their patterns to your vertical, adapting for domain specifics
- Cost: Time (reverse-engineering)
- Timeline: 2-4 weeks of analysis

---

#### 🤝 Practice Exercise

> **Ask your AI**: "I've rated myself on the three requirements [share your scores]. My lowest score is [requirement]. Help me pick ONE mitigation strategy that fits my situation: [describe your constraints: time, budget, existing relationships]. Then we'll flesh out the specific action steps."

**Expected Outcome**: You'll have a concrete mitigation plan for your lowest-scoring requirement, integrated into your PPP timeline.

---

## Try With AI

Use your AI companion tool set up (e.g., ChatGPT web, Claude Code, Gemini CLI), you may use that instead—the prompts are the same.

### Prompt 1: Complete Your Readiness Assessment
```
I'm going to assess my readiness for a Digital FTE in [pick your vertical: education / healthcare / finance / etc.]. Let me rate myself on the three requirements:

Requirement 1 (Domain Expertise): ___/10
Requirement 2 (Integrations): ___/10
Requirement 3 (Solution Design): ___/10

For my lowest-scoring requirement, help me understand: What specifically am I missing? And what's the fastest way to build that capability? Be honest about whether it's a "gap I can close" or a "misalignment with my strengths."
```

**Expected outcome**: Clear diagnosis of your gaps plus realistic remediation path.

### Prompt 2: Choose Mitigation Strategies
```
Based on my assessment above, I have a gap in [requirement]. Here are the mitigation strategies from the lesson:
- [Strategy A]
- [Strategy B]
- [Strategy C]

I'm [describe your constraints: "working full-time and can only commit 10 hours/week" OR "have 6 months of savings" OR "know someone in this industry"]. Which mitigation strategy fits best? Help me evaluate the tradeoffs.
```

**Expected outcome**: Personalized mitigation plan aligned to your constraints.

### Prompt 3: Plan Your Path Forward
```
After assessing my three requirements and choosing mitigation strategies, here's my picture:

- Domain Expertise: [your score and chosen mitigation]
- Integrations: [your score and chosen mitigation]
- Solution Design: [your score and chosen mitigation]

Help me create a timeline: When can I realistically be "ready to execute PPP"? What are the 3-5 concrete steps I need to take in the next 30 days?
```

**Expected outcome**: 30-day action plan to move closer to readiness.
