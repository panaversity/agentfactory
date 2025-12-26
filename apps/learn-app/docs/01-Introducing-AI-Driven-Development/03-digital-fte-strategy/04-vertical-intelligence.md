---
sidebar_position: 4
title: "From Code Reuse to Vertical Intelligence"
description: "The paradigm shift: Why disposable code and reusable intelligence are the new architecture."
reading_time: "2.5 minutes"
chapter: 3
lesson: 4
duration_minutes: 15

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding Architectural Paradigm Shift"
    proficiency_level: "A1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can identify the difference between code reuse (old) and intelligence reuse (new)"

  - name: "Recognizing Reusable Intelligence Components"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify five components of reusable intelligence (prompts, skills, MCPs)"

  - name: "Evaluating Intelligence vs. Code Tradeoffs"
    proficiency_level: "A2"
    category: "Soft"
    bloom_level: "Understand"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student can assess when intelligence reuse is superior to code reuse"

learning_objectives:
  - objective: "Understand the paradigm shift from DRY (code reuse) to intelligence reuse"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Explanation of why code is disposable when AI generates it quickly"

  - objective: "Identify the five components of reusable intelligence (system prompts, skills, MCPs)"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Recognition and description of each component's role"

  - objective: "Evaluate defensibility through vertical integrations vs. generic code"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Assessment of which components create competitive moats"

cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (disposable code, intelligence reuse, five components, vertical moats) within A1-A2 limit ✓"

differentiation:
  extension_for_advanced: "Research MCP standard; analyze how MCPs enable intelligence reuse"
  remedial_for_struggling: "Focus on system prompts and skills; skip deep technical details about MCPs"
---

# From Code Reuse to Vertical Intelligence: The New Architecture of Software

**Lesson Video:**

<iframe width="100%" height="400" src="https://www.youtube.com/embed/bj5Mc5END0Y" title="From Code Reuse to Vertical Intelligence" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

For decades, software architecture followed principles of code reuse, formalized as DRY (Don't Repeat Yourself) in 1999. The goal was to write code once, reuse it everywhere. Libraries, frameworks, microservices all built on the logic of code reuse.

This logic breaks down in the AI era.

## Why Code Reuse Mattered (And Doesn't Anymore)

In the traditional era, code reuse was expensive to maintain. If your payment library had a bug, you had to fix it once, and every application benefited. This incentivized heavy upfront investment in reusable code.

In the AI era, code is disposable. A subagent can generate 10,000 lines of specialized code in ten seconds. Maintaining that code across multiple applications is expensive. Generating fresh code for each application is free.

The new principle is: Reuse intelligence, not code.

#### 🎓 Expert Insight

> This paradigm shift is counterintuitive for developers trained in traditional software engineering. For decades, we learned "write once, run everywhere" and "don't repeat yourself." But when code generation is nearly free (seconds via AI), maintaining reusable code becomes more expensive than regenerating specialized code. Think of it like mass production vs. 3D printing: mass production requires expensive molds (reusable code), but 3D printing can create custom objects on demand (regenerated code). When customization is free, standardization loses its economic advantage.

## The Five Components of a Reusable Subagent

An expert developer building a Digital FTE relies on five architectural components that scale across applications:

![Layered architecture diagram showing five components of reusable AI sub-agents: system prompt, horizontal skills, vertical skills, MCP horizontal connections, and MCP vertical connections, arranged from generic (top) to domain-specific (bottom)](https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/images/part-1/chapter-3/five-components-architecture.png)

These five components form a stack where each layer builds defensibility. The top three layers (system prompts, horizontal skills, vertical skills) define intelligence. The bottom two layers (horizontal MCPs, vertical MCPs) define integration.

### 1. System Prompt (Persona + Scope)

This is the personality layer. A system prompt defines who the Custom Agent is, what expertise it possesses, and what boundaries constrain its actions.

**Example for a financial analyst Custom Agent:**
- **Persona**: "You are a senior portfolio manager with 20 years of experience in equities, macroeconomics, and risk management."
- **Knowledge base**: "You understand sector rotation, earnings models, SEC filing analysis, portfolio construction, and regulatory constraints."
- **Constraints**: "You only recommend trades within the fund's risk limits ($5M max position, 20% max sector allocation). Never recommend penny stocks or illiquid instruments."

This intelligence is reusable. Write the system prompt once, then deploy it across 100 fund management firms. Each firm benefits from 20 years of simulated expertise without hiring a human expert.

A system prompt typically costs 200-500 tokens and requires 2-4 hours to craft expertly.

### 2. Horizontal Skills (Infrastructure)

These are generic, domain-agnostic skills that every Custom Agent needs: Docker containerization, Kubernetes orchestration, cloud authentication, monitoring and logging, database connections, CI/CD pipeline integration.

Horizontal skills are reusable unchanged across all Custom Agents. An expert who writes a "Deploy to Kubernetes" skill can deploy it to 50 different Custom Agents without modification.

**Token cost**: ~100 tokens per skill (via SKILL.md progressive disclosure, covered below)

### 3. Vertical Skills (Domain Expertise)

This is where specialization lives. A finance Custom Agent needs Bloomberg API knowledge, portfolio risk models, tax-loss harvesting strategies. A healthcare Custom Agent needs ICD-10 code classification, FHIR HL7 message parsing, clinical guideline cross-referencing, insurance coverage workflows.

Vertical skills are NOT reusable across domains—a healthcare billing skill is worthless in finance. But they are absolutely reusable *within* a domain.

**A healthcare Custom Agent's vertical skills example:**
- HL7/FHIR message parsing (read structured data from hospital EHR systems)
- Clinical guideline cross-referencing (match diagnoses to Cochrane/UpToDate evidence)
- Insurance prior authorization workflows (understand CPT codes, denial reasons, appeals)
- Lab result interpretation (clinical normal ranges, significance thresholds, trend analysis)
- Medication interaction checking (cross-reference with drug databases, flagging contraindications)

A healthcare domain expert writes these skills once, then licenses them to 20 healthcare startups. Each startup's Custom Agent gains deep healthcare domain expertise.

**Token cost**: ~100 tokens per skill (on-demand via progressive disclosure)

### 4. MCP Horizontal Connections (Dev Tools)

MCP stands for **Model Context Protocol**, the industry standard for connecting AI agents to external tools and APIs. Horizontal MCPs connect to generic developer tools: GitHub repositories, Docker registries, AWS/GCP/Azure platforms, Slack workspaces, CI/CD systems (GitHub Actions, GitLab CI, Jenkins).

A Custom Agent using horizontal MCPs can:
- Read code from repositories and understand architecture
- Deploy containerized applications to Kubernetes
- Trigger automated pipelines and monitor job results
- Post notifications to team Slack channels
- Query cloud infrastructure and estimate costs

These connections are infrastructure—they're not your competitive advantage, but they're essential infrastructure that all Custom Agents need.

**Token cost**: ~100-200 tokens per MCP connection (via SKILL.md references)

### 5. MCP Vertical Connections (Industry APIs) — THE MOAT

This is where competitive defensibility lives. A finance Custom Agent connects to Bloomberg Terminal API, real-time equity feeds, SEC EDGAR filings database. A healthcare Custom Agent connects to hospital EHR systems (Epic, Cerner, Athena), pharmaceutical databases (DrugBank, RxNorm), clinical literature APIs (PubMed, UpToDate).

These integrations are **not reusable across industries**. A Bloomberg integration is worthless in healthcare. But within an industry, vertical MCP connections are extraordinarily defensible. An expert who builds tight, production-grade integrations with Epic Systems (used by 55% of U.S. hospital beds) creates a moat that competitors must rebuild from scratch.

Why? Because integrations require:
- Credentials and contractual access (Epic won't share with every developer)
- Deep API knowledge and undocumented quirks
- Customer trust and regulatory compliance
- Maintenance burden as APIs change

A solo developer who controls the Epic integration for a healthcare vertical becomes the gatekeeper. Every healthcare startup in that vertical either licenses your integration or rebuilds it (6 months, $200K).

**Token cost**: ~500-2000 tokens per major integration (context-dependent on API complexity)

**The Defensibility Hierarchy:**
1. System prompts: Expensive to write, cheap to copy (anyone can paraphrase)
2. Horizontal skills: Easy to share, easy to copy (open-source equivalents exist)
3. Vertical skills: Moderate difficulty to copy (requires domain expertise)
4. Horizontal MCPs: Easy to replicate (standard APIs)
5. **Vertical MCPs: Expensive to replicate** (requires credentials + relationship + time) ← YOUR MOAT

The economist Paul Graham calls this "unfair advantage": competitors can copy your system prompt, but they can't copy your production integration with your customer's proprietary EHR system.

#### 💬 AI Colearning Prompt

> **Explore with your AI**: "The lesson describes five components of a reusable subagent. Let's test which components create the most defensibility—ask your AI: 'Rank these five components from EASIEST to HARDEST for a competitor to replicate.' Then challenge its ranking: Do you agree? What makes vertical MCP connections harder to replicate than, say, system prompts?"

## SKILL.md Format: Packaging Your Vertical Intelligence

Now you understand the five components. The question becomes: How do you actually package these components so they're reusable?

The answer is **SKILL.md** — a standardized format for encoding reusable intelligence that Custom Agents can discover, load, and execute.

### What is SKILL.md?

SKILL.md is a markdown file that teaches a Custom Agent how to perform a specific, repeatable task. Instead of embedding instructions inside a system prompt (expensive, inflexible), you organize them into discrete SKILL.md files that agents load on-demand.

**Why SKILL.md matters:**
- **Modular**: Each skill is a standalone file (~100 tokens) that agents can discover
- **Discoverable**: Agents can ask "What skills are available?" and browse
- **Reusable**: The same SKILL.md works across Claude Code, OpenAI agents, LangChain, and other frameworks
- **Progressive disclosure**: Only the skill description loads at startup; detailed docs load on-demand
- **Token efficient**: Instructions (~100 tokens) load once; supporting docs and code load only when needed

### SKILL.md File Structure

```markdown
---
name: skill-name
description: One-line description for agent discovery
---

# Skill Title

## When to Use
Trigger conditions and use cases—when should an agent use this skill?

## Instructions
Step-by-step guidance for the agent to execute this skill.

## Examples
Concrete examples of correct behavior and outcomes.

## References
See [REFERENCE.md](./REFERENCE.md) for detailed specifications.
See [EXAMPLES.md](./EXAMPLES.md) for additional examples.
```

**Key pattern**: The SKILL.md file itself (~100 tokens) is always loaded. The reference docs and examples are loaded on-demand, only when the agent needs deeper details. This progressive disclosure keeps your startup token budget low.

### Folder Organization: The Healthcare Example

Here's how a healthcare domain expert organizes vertical skills:

```
.claude/skills/healthcare/
├── billing/
│   ├── SKILL.md                    # CPT code classification
│   ├── REFERENCE.md                # Full CPT code database
│   └── scripts/
│       └── lookup_cpt.py           # Executed (0 tokens)
│
├── clinical-guidelines/
│   ├── SKILL.md                    # Guideline matching
│   ├── REFERENCE.md                # Cochrane + UpToDate summaries
│   └── examples/
│       └── EXAMPLES.md             # Real diagnosis→treatment flows
│
├── ehr-integration/
│   ├── SKILL.md                    # HL7/FHIR parsing
│   ├── REFERENCE.md                # Message structure specs
│   └── scripts/
│       ├── parse_hl7.py            # Parser (0 tokens)
│       └── validate_fhir.py        # Validator (0 tokens)
│
├── medication-safety/
│   ├── SKILL.md                    # Drug interaction checking
│   ├── REFERENCE.md                # Drug database structure
│   └── scripts/
│       └── check_interactions.py   # Query tool (0 tokens)
│
└── prior-auth-workflows/
    ├── SKILL.md                    # Insurance approval logic
    ├── REFERENCE.md                # Payer-specific rules
    └── examples/
        └── EXAMPLES.md             # Approval + denial patterns
```

This structure has immediate benefits:

**For the healthcare domain expert:**
- Write each skill once (5-10 hours per skill)
- License to 50 healthcare startups (50x return on investment)
- Update the billing SKILL.md once → all 50 customers benefit

**For each healthcare startup:**
- Gain production-grade healthcare expertise instantly
- Don't hire a healthcare compliance specialist (saves $150K/year)
- Focus engineering on vertical MCPs (integrating with Epic, Cerner)

**For the Custom Agent:**
- Load healthcare/billing/SKILL.md (~100 tokens)
- Execute scripts/lookup_cpt.py (0 tokens—executed locally)
- Reference healthcare/billing/REFERENCE.md only if code lookup fails
- Result: Deep healthcare expertise in fewer than 200 tokens

### Naming Conventions and Discovery

Skills follow a consistent naming convention:
- Domain: `healthcare`, `finance`, `legal`, `manufacturing`
- Capability: `billing`, `clinical-guidelines`, `ehr-integration`
- File: `.claude/skills/[domain]/[capability]/SKILL.md`

Custom Agents can discover skills using an MCP server called **SkillPort**:

```python
# Agent asks SkillPort for available healthcare skills
available_skills = skillport.search_skills(domain="healthcare")
# Returns: ["billing", "clinical-guidelines", "ehr-integration", ...]

# Load the billing skill
billing_skill = skillport.load_skill("healthcare", "billing")
# Returns: SKILL.md (~100 tokens) + folder path

# When needed, access deeper references
advanced_codes = read_file("healthcare/billing/REFERENCE.md")  # On-demand
```

### Why Domain Organization Matters

Organizing skills by domain creates **discoverable intellectual property**.

In the old world, expertise lived in people's heads or scattered across codebases. With SKILL.md:
- Expertise is organized (domain → capability)
- Expertise is discoverable (agents can list healthcare skills)
- Expertise is monetizable (license healthcare skills to 50 startups)
- Expertise is portable (works across agent frameworks)

A financial domain expert creates `.claude/skills/finance/` with 15 vertical skills:
- Portfolio risk modeling
- SEC filing analysis
- Tax-loss harvesting
- Sector rotation patterns
- Earnings model construction

Each skill is 5-20 hours to write. Total: 75-300 hours one-time investment. License result to 100 fintech startups at $500/month each = $600K/month revenue.

The value of your expertise is proportional to how many domain experts contributed. A healthcare system with 5 healthcare domain experts writing healthcare skills is 10x more valuable than a generic system.

## Progressive Disclosure and Token Efficiency

You now understand the architectural question: "How do I package reusable intelligence?" The next question is: "How do I fit it all in context?"

Large language models have context windows (e.g., Claude 3.5 Sonnet has 200K tokens). But practical work happens in much smaller windows (e.g., 50K tokens for cost efficiency). How do you fit 500,000 tokens of healthcare knowledge into a 50K-token agent?

**Answer: Progressive disclosure through SKILL.md.**

### The Token Problem

Traditional approaches load everything upfront:

| Approach | Token Cost | Problem |
|----------|-----------|---------|
| **Embed in system prompt** | 5,000-50,000 tokens | Inflexible; wastes tokens on unused knowledge |
| **MCP tools (raw definitions)** | 14,000-80,000 tokens per MCP server | Every tool definition loaded always |
| **Load full knowledge base** | 100,000+ tokens | Impractical; breaks affordability |

A healthcare Custom Agent needs 5 major MCPs: Epic EHR, DrugBank, insurance payer rules, clinical literature, lab standards. That's ~80,000 tokens just for tool definitions. Before the agent starts working, you've burned 40% of a typical context window.

### The Solution: Progressive Disclosure

SKILL.md implements progressive disclosure in layers:

**Layer 1: Startup (~100 tokens per skill)**
```markdown
# Billing Skill
## When to Use
Use when an agent needs to classify medical procedures into CPT codes.
## Instructions
1. Take input: medical procedure description
2. Search CPT code database
3. Return: Code + description + modifier flags
```

That's it. The agent loads this once and knows the skill exists. Total cost: ~100 tokens.

**Layer 2: On-Demand (~1,000 tokens, only if needed)**
```markdown
[REFERENCE.md loads only when agent says:]
"I need to classify a billing scenario."
→ Load healthcare/billing/REFERENCE.md
→ Contains: Full CPT code taxonomy, modifiers, bundling rules
```

**Layer 3: Execution (0 tokens)**
```python
# For complex lookups, use a script instead of loading docs
scripts/lookup_cpt.py is EXECUTED, never loaded into context
Input: "knee arthroscopy with debridement"
Output: "CPT 29881"
Cost: 0 tokens
```

### Comparison: Token Cost

| Scenario | Token Cost | Result |
|----------|-----------|--------|
| **Embed everything in system prompt** | 50,000+ | One-time cost; slow, inflexible |
| **SKILL.md + Progressive Disclosure** | 100-500 per session | Load only what you use; fast, flexible |
| **Ratio** | 100:1 reduction | Same intelligence; 100x more efficient |

This is why SKILL.md creates defensibility: Competitors can build healthcare expertise, but they can't build it *efficiently*. An expert who packages 15 healthcare skills with progressive disclosure patterns is 10x more capital-efficient than a team that embeds everything in prompts.

### MCP + SKILL.md: The Defensibility Moat

Vertical MCP connections become truly powerful when paired with SKILL.md progressive disclosure.

**Scenario 1: Competitor steals your system prompt**
- Cost to you: Rewrite system prompt (4 hours)
- You stay ahead because your SKILL.md and MCPs are still proprietary

**Scenario 2: Competitor reverse-engineers your SKILL.md files**
- Cost to you: Rewrite skills (20 hours per skill)
- You stay ahead because your MCP connections (Epic, Cerner integration) still require:
  - API credentials (they don't have)
  - Customer trust (takes years to build)
  - Maintenance burden (Epic updates constantly)

**Scenario 3: Competitor builds their own MCP integrations**
- Cost to them: 6 months, $200K
- Your defensibility: First-mover advantage + customer lock-in
- Result: Competitor can't catch up before you've captured 80% of the market

The economist Stuart Kauffman calls this "adjacent possible": You can do adjacent things faster because you've already solved the hard parts. Your Epic integration is adjacent to Cerner integration (both healthcare EHRs). A competitor building from scratch is miles away.

## Traditional Code Reuse vs. Vertical Intelligence Reuse

| Dimension | Traditional Code Reuse | Vertical Intelligence Reuse |
|-----------|----------------------|------------------------------|
| **Unit of Reuse** | Libraries, APIs | System prompts, skill definitions, MCP connections |
| **Lifetime** | Long-lived (used for years) | Disposable (regenerated per application) |
| **Maintenance** | Centralized (one library, many users) | Distributed (each application owns its copy) |
| **Scalability** | Limited (library updates risk breaking changes) | Unlimited (new applications get fresh code) |
| **Value Source** | Code logic | Domain expertise and integrations |

## A Concrete Example: Accounting Library vs. Accounting Subagent

**Traditional approach**: You build an accounting library with Chart of Accounts, General Ledger, Tax reporting. You maintain it across five accounting software products. Every time tax code changes, you update the library once, and every app benefits. But the library is complex because it supports every feature of every app.

**AI-driven approach**: You build an accounting subagent with:
- System prompt defining an expert accountant persona
- Knowledge base of current tax code (updated monthly via MCP)
- Integrations with QuickBooks, Xero, Freshbooks, Wave (all major accounting software)
- Vertical skills: GAAP standards, tax schedules, audit workflows

When you want to serve a new customer, you don't reuse code. You generate new code tailored to that customer's workflows. But you reuse the intelligence: the system prompt, the tax knowledge, the integrations.

The code is disposable. The intelligence is permanent. The value per developer stays high because you focus on domain expertise and integrations, not code maintenance.

#### 🤝 Practice Exercise

> **Ask your AI**: "Pick a domain I might know well (small business accounting, teacher lesson planning, real estate). Design the five components of a subagent for this domain. For each component, give me one specific example. Then help me validate: Are these components realistic, or are we making assumptions that don't match reality?"

**Expected Outcome**: You'll practice applying the five-component framework to a specific domain and learn to reality-check AI designs with domain knowledge. This teaches you to think architecturally about intelligence reuse.

---

Now you understand the architecture of reusable intelligence. The next insight is how to actually enter a vertical market and execute this strategy: the Piggyback Protocol Pivot.

---

## Try With AI: Architect Your Vertical Skills

This is where you build. You're going to design a SKILL.md structure for a domain you know (or want to master), then refine it collaboratively with your AI until it's realistic and achievable.

### Setup

You'll need Claude Code, ChatGPT, Gemini, or another capable AI assistant. Use the following prompts sequentially—they build on each other. The prompts are designed so your AI teaches you, you refine the AI's suggestions, and together you converge on a practical skills architecture.

---

### Stage 1: Discover the Pattern

**Your first prompt to AI:**

```
I want to build a Digital FTE for the [DOMAIN] vertical (examples: healthcare,
real estate, e-commerce support, legal discovery, construction management—pick
one you know or want to master).

Here's what I already understand: There are five architectural components: system
prompt, horizontal skills, vertical skills, horizontal MCPs, and vertical MCPs.

But I'm unclear on how to ACTUALLY ORGANIZE this. The lesson showed a healthcare
example with a folder structure like:

.claude/skills/healthcare/
├── billing/
├── clinical-guidelines/
├── ehr-integration/
└── medication-safety/

For my domain ([DOMAIN]), help me brainstorm what the equivalent structure would
be. What would the domain-specific folders be? What vertical skills would I need
to write first? Don't try to be exhaustive—just give me 5-7 core folders that
would make sense for someone like me to start with.
```

**What AI teaches you**: A concrete folder structure tailored to your domain. This is the pattern discovery moment—you learn how the framework applies specifically to you.

---

### Stage 2: You Refine and Challenge (You as Teacher)

**Your second prompt:**

Now AI has suggested a structure. Refine it based on reality. Use this pattern:

```
Looking at your structure, I want to refine based on my actual situation:

1. Which 3 of the folders you suggested would be EASIEST for me to start with
   (considering my skills, time, and existing expertise)?

2. Which folder requires the deepest domain knowledge that I'd need to learn or
   hire for?

3. One of your suggestions doesn't quite fit my reality because [EXPLAIN].
   What should I replace it with?

Then reorganize the structure with MY constraints in mind, not generic best
practices.
```

**What happens**: You challenge the AI's generic suggestion. The AI learns your constraints (time, expertise, capital). This is the second role—you teaching the AI about your world.

---

### Stage 3: Convergence (Bidirectional Learning)

**Your third prompt:**

```
Okay, I like this revised structure better. Now let's think about defensibility.
Looking at the five components again (system prompt, horizontal skills, vertical
skills, horizontal MCPs, vertical MCPs), which ONE COMPONENT creates the most
defensibility in my domain?

For example, in healthcare, vertical MCPs (Epic, Cerner integrations) are the
moat. But in real estate, maybe it's vertical skills (zoning, comps analysis,
title interpretation) because there's no central API.

What's the moat in [DOMAIN]? And given that, should my SKILL.md organization
emphasize that component?
```

**What emerges**: A refined understanding of where your defensibility lives. The AI contributes strategic thinking; you contribute domain intuition. Neither of you had this answer alone—it emerged through dialogue.

---

### Stage 4: Validation (Building Confidence)

**Your fourth prompt:**

```
Let's reality-check this. Here's my time and capital constraint:
- Time: I can dedicate [10 / 30 / 50 / 100] hours in the next 3 months
- Capital: I have $[0 / 5K / 50K / 200K] to invest in infrastructure
- Current position: I have [zero / some / deep] expertise in [DOMAIN]

Given the SKILL.md structure we just designed, can I actually execute it in
3 months with my constraints? If not, what should I cut or defer? Help me
create a realistic Phase 1 (first 3 months) and Phase 2 (months 4-12) roadmap.
```

**What happens**: You bring financial and time reality. AI helps you scope realistically. This prevents the common mistake of designing a system you can't execute.

---

### Stage 5: Final Architecture (Your Vision)

**Your final prompt:**

```
Summarize our conversation into a single architecture diagram (ASCII art or prose
description). Here's what I want to see:

1. My domain: [DOMAIN]
2. Five components:
   - System prompt: [What will the persona be?]
   - Horizontal skills: [1-2 examples]
   - Vertical skills: [3-5 specific examples you'll build first]
   - Horizontal MCPs: [What generic tools need connecting?]
   - Vertical MCPs: [What are the critical industry APIs?]
3. Defensibility: [Which component creates the moat?]
4. Phase 1 timeline: [What do you build first?]
5. Revenue assumption: [How might this monetize?]

Show me how these pieces fit together. This is my north star—the architecture
I'll execute against.
```

**What you get**: A coherent architecture that you own, tested against reality, grounded in your constraints.

---

### Key Moment: Active Collaboration, Not Q&A

Notice something important: You didn't ask AI to "explain SKILL.md." You co-designed with AI—bringing domain knowledge, pushing back on suggestions that wouldn't work in your context, and refining until the architecture matched your reality.

This is how expert developers work with General Agents. You're not getting AI to "do work"—you're offloading architecture thinking to an AI partner while you provide judgment, domain knowledge, and constraints.

The result is an architecture that:
- Is grounded in your domain (not generic)
- Respects your constraints (not optimistic)
- Has a clear moat (not easily copied)
- Can execute in Phase 1 (not 5-year plan)
- Maps to revenue (not research project)

This is the skill that Digital FTE builders develop: turning your expertise into an architecture, an executable roadmap, and a defensible product—all in 5 iterative conversations with an AI.




