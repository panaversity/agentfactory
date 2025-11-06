# Part 1 Master Skill Registry

## Overview

This is the authoritative reference for all skills taught in Part 1. Each skill entry defines:
- **Skill Name**: Canonical name used consistently across chapters
- **Definition**: What proficiency in this skill means
- **Domain**: Conceptual grouping (Development, Business, Architecture, etc.)
- **Category**: Technical / Conceptual / Soft
- **CEFR Path**: Progression from A1 → A2 → B1 → B2
- **First Appearance**: Chapter and lesson where skill is introduced
- **Dependencies**: Prerequisites that should be mastered first
- **Validation Criteria**: How to assess mastery at each level

---

## Domain 1: AI Development & Tools

### 1.1 Understanding AI Capability Evolution
- **Definition**: Recognize what AI models can do now vs. what they couldn't do 18 months ago; distinguish breakthrough capabilities from marketing hype
- **Domain**: Development
- **Category**: Conceptual
- **CEFR Path**:
  - **A1 (Remember/Understand)**: Can identify evidence of capability breakthroughs (ICPC World Finals, benchmarks)
  - **A2 (Understand/Apply)**: Can evaluate strength and reliability of evidence; distinguish hype from substance
  - **B1 (Apply/Analyze)**: Can analyze capability gaps and predict likely near-term improvements
  - **B2 (Analyze/Evaluate)**: Can benchmark models against real-world tasks; design experiments to measure capability
- **First Appearance**: Ch2, L1 (The Inflection Point)
- **Dependencies**: None (foundational)
- **Validation Criteria**:
  - A1: Explain why ICPC World Finals matters; identify three capability examples
  - A2: Compare 2024 vs. 2025 capabilities; assess evidence quality
  - B1: Predict capability trajectory for 2026-2027

---

### 1.2 Distinguishing Development Patterns (Vibe Coding vs. Spec-Driven)
- **Definition**: Recognize when to use intuition-led exploration vs. specification-first discipline; understand tradeoffs between each approach
- **Domain**: Development
- **Category**: Conceptual
- **CEFR Path**:
  - **A1 (Understand)**: Can identify characteristics of vibe coding vs. spec-driven development
  - **A2 (Understand/Apply)**: Can explain when each approach is appropriate; assess personal tendency
  - **B1 (Apply/Analyze)**: Can design hybrid approaches mixing both patterns; evaluate team culture fit
  - **B2 (Analyze/Evaluate)**: Can architect methodology transitions; measure impact on quality/velocity
- **First Appearance**: Ch2, L2 (Development Patterns)
- **Dependencies**: Requires Team/Organization context (typically part of Chapter 6+)
- **Validation Criteria**:
  - A1: Explain Team A vs. Team B example; identify pattern differences
  - A2: Assess your project: should you use vibe coding or spec-driven?
  - B1: Design a methodology for your team; explain why

---

### 1.3 Understanding AI as Amplifier
- **Definition**: Recognize that AI magnifies existing practices—strong processes become better, weak processes become worse; no tool fixes dysfunction
- **Domain**: Development
- **Category**: Conceptual
- **CEFR Path**:
  - **A1 (Understand)**: Can explain amplifier concept with simple analogy
  - **A2 (Understand/Apply)**: Can assess whether current practices will amplify well or break
  - **B1 (Apply/Analyze)**: Can design process improvements before adding AI
  - **B2 (Analyze/Evaluate)**: Can predict failure modes when AI meets dysfunction
- **First Appearance**: Ch2, L3 (DORA Perspective)
- **Dependencies**: Understanding AI Capability Evolution (1.1)
- **Validation Criteria**:
  - A1: Explain why same tool produces opposite outcomes in different organizations
  - A2: Assess your team: are you ready for AI amplification?
  - B1: Identify three guardrails your team needs before AI adoption

---

### 1.4 Understanding Modern AI Development Stack (Three-Layer Architecture)
- **Definition**: Recognize separation between frontier models (Layer 1), AI-first IDEs (Layer 2), and development agents (Layer 3); understand why separation matters
- **Domain**: Development / Architecture
- **Category**: Conceptual
- **CEFR Path**:
  - **A1 (Remember/Understand)**: Can identify three layers and describe each; recognize tools in each layer
  - **A2 (Understand/Apply)**: Can evaluate vendor independence; assess lock-in risks
  - **B1 (Apply/Analyze)**: Can design tool stack choices for specific workflows
  - **B2 (Analyze/Evaluate)**: Can predict stack evolution; assess MCP adoption impact
- **First Appearance**: Ch2, L4 (Modern AI Development Stack)
- **Dependencies**: Understanding AI Capability Evolution (1.1)
- **Validation Criteria**:
  - A1: Explain three-layer stack using non-technical analogy
  - A2: Assess vendor lock-in in your current tools
  - B1: Design optimal stack for your project type

---

### 1.5 Understanding AI Development Methodology (AIDD)
- **Definition**: Recognize AIDD as specification-first methodology that elevates developers to architects; understand how it differs from traditional approaches
- **Domain**: Development / Methodology
- **Category**: Conceptual
- **CEFR Path**:
  - **A1 (Understand)**: Can identify AIDD as specification-first vs. code-first
  - **A2 (Understand/Apply)**: Can list nine characteristics; assess applicability to context
  - **B1 (Apply/Analyze)**: Can implement AIDD workflow; design spec templates
  - **B2 (Analyze/Evaluate)**: Can architect AIDD-based process improvements; measure outcomes
- **First Appearance**: Ch4, L2 (AIDD Defined)
- **Dependencies**: Distinguishing Development Patterns (1.2), Understanding AI as Amplifier (1.3)
- **Validation Criteria**:
  - A1: Explain how AIDD differs from traditional development
  - A2: Assess whether AIDD is right for your team
  - B1: Design an AIDD implementation roadmap

---

## Domain 2: Business Models & Market Strategy

### 2.1 Recognizing AI-Era Opportunity Timing
- **Definition**: Understand why 2025 is an inflection point; distinguish real shifts from hype; assess personal/organizational timing
- **Domain**: Business / Strategy
- **Category**: Conceptual
- **CEFR Path**:
  - **A1 (Remember/Understand)**: Can identify convergent validation signals (academic, financial, adoption)
  - **A2 (Understand/Apply)**: Can assess personal timing; evaluate urgency vs. readiness
  - **B1 (Apply/Analyze)**: Can predict second-order effects of timing; plan adoption cadence
  - **B2 (Analyze/Evaluate)**: Can benchmark industry timing; position strategically
- **First Appearance**: Ch1, L1 (Moment That Changed Everything); reinforced Ch2, L1 (Inflection Point)
- **Dependencies**: None (foundational)
- **Validation Criteria**:
  - A1: Identify three evidence signals showing 2025 is different
  - A2: Assess: am I too early, at the right time, or late?
  - B1: Plan a 24-month AI adoption strategy

---

### 2.2 Understanding Competitive Layer Strategy (Snakes & Ladders)
- **Definition**: Recognize four competitive layers in AI markets; understand why consumer layer (snakes) fails for individuals but vertical markets (ladders) succeed
- **Domain**: Business / Strategy
- **Category**: Conceptual
- **CEFR Path**:
  - **A1 (Understand)**: Can identify four layers and example companies in each
  - **A2 (Understand/Apply)**: Can explain why vertical dominance beats head-on competition; assess layer for own idea
  - **B1 (Apply/Analyze)**: Can design layer-specific strategies; predict incumbent responses
  - **B2 (Analyze/Evaluate)**: Can analyze cross-layer dynamics; predict ecosystem evolution
- **First Appearance**: Ch3, L2 (Snakes and Ladders)
- **Dependencies**: Recognizing AI-Era Opportunity Timing (2.1)
- **Validation Criteria**:
  - A1: Explain why competing in consumer layer fails; identify layer for your idea
  - A2: Explain why your chosen layer is defensible
  - B1: Design market entry strategy; predict competitive responses

---

### 2.3 Understanding Economic Scaling (90-10 Rule)
- **Definition**: Recognize that 90% of work is mechanical (AI-automatable) and 10% is strategic judgment; understand how this changes business economics
- **Domain**: Business / Economics
- **Category**: Conceptual
- **CEFR Path**:
  - **A1 (Understand)**: Can identify mechanical vs. strategic work in a business
  - **A2 (Understand/Apply)**: Can explain value-per-employee growth; assess revenue potential
  - **B1 (Apply/Analyze)**: Can design outsourcing strategies; calculate unit economics
  - **B2 (Analyze/Evaluate)**: Can predict scaling limits; optimize time allocation
- **First Appearance**: Ch3, L3 (Super Orchestrators)
- **Dependencies**: Understanding Competitive Layer Strategy (2.2)
- **Validation Criteria**:
  - A1: Identify 90/10 split in an example business
  - A2: Map 90/10 split in your idea; identify bottleneck (the 10%)
  - B1: Design revenue model reflecting 90/10 economics

---

### 2.4 Understanding Vertical Market Dominance Strategy (PPP)
- **Definition**: Recognize three-phase Piggyback Protocol Pivot: infrastructure bridging → market validation → strategic pivoting
- **Domain**: Business / Strategy
- **Category**: Conceptual
- **CEFR Path**:
  - **A1 (Understand)**: Can describe three phases and timeline; identify incumbent fragmentation
  - **A2 (Understand/Apply)**: Can assess phase appropriateness for context; evaluate defensibility
  - **B1 (Apply/Analyze)**: Can design PPP roadmap; identify critical inflection points
  - **B2 (Analyze/Evaluate)**: Can predict competitive responses; design defensive strategies
- **First Appearance**: Ch3, L5 (PPP Strategy)
- **Dependencies**: Understanding Competitive Layer Strategy (2.2), Understanding Economic Scaling (2.3)
- **Validation Criteria**:
  - A1: Explain three phases using education vertical example
  - A2: Assess: can I realistically execute PPP in my context?
  - B1: Create detailed 18-month PPP roadmap for chosen vertical

---

### 2.5 Recognizing Critical Success Requirements (Three Requirements)
- **Definition**: Understand that vertical success requires three interdependent elements: fine-tuned models, deep integrations, complete agentic solutions; missing any one causes failure
- **Domain**: Business / Strategy
- **Category**: Conceptual
- **CEFR Path**:
  - **A1 (Understand)**: Can identify three requirements; recognize when products lack them
  - **A2 (Understand/Apply)**: Can assess feasibility of building all three; identify priority sequence
  - **B1 (Apply/Analyze)**: Can design roadmap covering all three; predict failure modes
  - **B2 (Analyze/Evaluate)**: Can design contingency for missing capabilities; manage tradeoffs
- **First Appearance**: Ch3, L6 (Three Requirements)
- **Dependencies**: Understanding Vertical Market Dominance Strategy (2.4)
- **Validation Criteria**:
  - A1: Explain why OpenAI Study Mode missed requirement(s)
  - A2: Assess your idea: which requirement is easiest/hardest?
  - B1: Design build order: which requirement first?

---

## Domain 3: Professional Growth & Career

### 3.1 Recognizing AI's Impact on Developer Role Evolution
- **Definition**: Understand how AI shifts developers from "code writers" to "system architects and specification engineers"; recognize new responsibilities and opportunities
- **Domain**: Career / Professional Growth
- **Category**: Soft
- **CEFR Path**:
  - **A1 (Understand)**: Can identify role changes (from code-writing to orchestration)
  - **A2 (Understand/Apply)**: Can assess personal fit for evolving role; identify skill gaps
  - **B1 (Apply/Analyze)**: Can design personal development path; mentor others through transition
  - **B2 (Analyze/Evaluate)**: Can architect organizational role transformations
- **First Appearance**: Ch1, L1 (Moment That Changed Everything); reinforced throughout
- **Dependencies**: Understanding AI Capability Evolution (1.1)
- **Validation Criteria**:
  - A1: Explain how developer role has changed in two years
  - A2: Assess: am I ready for new role expectations?
  - B1: Design personal 18-month development plan

---

### 3.2 Evaluating Personal Learning Strategy & Adaptation
- **Definition**: Assess one's own learning approach; evaluate whether current methods prepare for AI-era development; identify gaps and opportunities
- **Domain**: Career / Professional Growth
- **Category**: Soft
- **CEFR Path**:
  - **A1 (Understand)**: Can identify learning gaps (what traditional education didn't teach)
  - **A2 (Understand/Apply)**: Can assess personal learning style; design adaptation strategy
  - **B1 (Apply/Analyze)**: Can architect learning pathways; support others through transitions
  - **B2 (Analyze/Evaluate)**: Can design educational transformations; measure learning effectiveness
- **First Appearance**: Ch1, L8 (Traditional CS Education Gaps)
- **Dependencies**: Recognizing AI's Impact on Developer Role Evolution (3.1)
- **Validation Criteria**:
  - A1: Identify three gaps in traditional CS education for AI era
  - A2: Assess: which gaps do I have? What's my learning plan?
  - B1: Design learning path; identify resources and support needed

---

### 3.3 Understanding M-Shaped Developer Profile
- **Definition**: Recognize that developers can now maintain deep expertise across multiple complementary domains; understand how AI enables this multi-domain mastery
- **Domain**: Career / Professional Growth
- **Category**: Soft
- **CEFR Path**:
  - **A1 (Understand)**: Can distinguish M-shaped from T-shaped and generalist profiles
  - **A2 (Understand/Apply)**: Can identify complementary domains; assess personal potential
  - **B1 (Apply/Analyze)**: Can design multi-domain learning path; manage skill building
  - **B2 (Analyze/Evaluate)**: Can architect organizational capability; lead multi-domain teams
- **First Appearance**: Ch4, L5 (M-Shaped Developer)
- **Dependencies**: Recognizing AI's Impact on Developer Role Evolution (3.1), Understanding AI Development Methodology (1.5)
- **Validation Criteria**:
  - A1: Explain M-shaped vs. T-shaped with examples
  - A2: Identify 2-3 complementary domains for own career
  - B1: Design 24-month multi-domain mastery plan

---

## Domain 4: Systems Thinking & Integration

### 4.1 Understanding AIDD Nine Pillars (Foundational Technologies)
- **Definition**: Recognize nine integrated technologies that make AIDD possible; understand what barrier each removes; recognize interdependencies
- **Domain**: Systems / Architecture
- **Category**: Conceptual
- **CEFR Path**:
  - **A1 (Remember/Understand)**: Can identify nine pillars; match each to a barrier it removes
  - **A2 (Understand/Apply)**: Can explain how pillars integrate; assess relevance to goals
  - **B1 (Apply/Analyze)**: Can design pillar-based infrastructure; identify dependencies
  - **B2 (Analyze/Evaluate)**: Can predict pillar evolution; design contingencies for missing pillars
- **First Appearance**: Ch4, L3 (Nine Pillars Overview)
- **Dependencies**: Understanding AI Development Methodology (1.5)
- **Validation Criteria**:
  - A1: List nine pillars; match each to barrier it removes
  - A2: Assess: which pillars matter most for my context?
  - B1: Design infrastructure using all nine pillars

---

### 4.2 Understanding Systems Completeness
- **Definition**: Recognize why partial adoption creates bottlenecks; understand why all nine pillars working together produce disproportionate returns
- **Domain**: Systems / Architecture
- **Category**: Conceptual
- **CEFR Path**:
  - **A1 (Understand)**: Can identify gaps when pillars are missing; recognize bottleneck effects
  - **A2 (Understand/Apply)**: Can explain why 6/9 pillars create problems; assess personal/organizational completeness
  - **B1 (Apply/Analyze)**: Can design path to completeness; sequence pillar adoption
  - **B2 (Analyze/Evaluate)**: Can analyze ecosystem maturity; predict sustainable competitive advantage
- **First Appearance**: Ch4, L6 (Why All Nine Matter)
- **Dependencies**: Understanding AIDD Nine Pillars (4.1), Understanding AI as Amplifier (1.3)
- **Validation Criteria**:
  - A1: Explain why 6/9 pillars create gaps
  - A2: Assess: are we complete? Where are our gaps?
  - B1: Design path to completeness over 18 months

---

### 4.3 Recognizing Technological Interdependencies
- **Definition**: Understand how technologies support each other; recognize that removing one pillar destabilizes the system; understand network effects
- **Domain**: Systems / Architecture
- **Category**: Conceptual
- **CEFR Path**:
  - **A1 (Understand)**: Can identify which pillar depends on which
  - **A2 (Understand/Apply)**: Can explain consequences of missing pillar; predict failure modes
  - **B1 (Apply/Analyze)**: Can design resilient architectures; identify single points of failure
  - **B2 (Analyze/Evaluate)**: Can analyze ecosystem dynamics; predict adoption sequences
- **First Appearance**: Ch4, L3-L6 (throughout Nine Pillars section)
- **Dependencies**: Understanding AIDD Nine Pillars (4.1)
- **Validation Criteria**:
  - A1: Explain relationship between two pillars
  - A2: Identify consequences if one pillar is missing
  - B1: Design architecture that accounts for interdependencies

---

## Skill Dependency Graph

```
Understanding AI Capability Evolution (1.1)
├─ Distinguishing Development Patterns (1.2)
│  └─ Understanding AI Development Methodology (1.5)
├─ Understanding AI as Amplifier (1.3)
│  └─ Understanding AIDD Nine Pillars (4.1)
└─ Understanding Modern AI Stack (1.4)
   └─ Recognizing AI-Era Opportunity Timing (2.1)
      ├─ Understanding Competitive Layer Strategy (2.2)
      │  ├─ Understanding Economic Scaling (2.3)
      │  │  └─ Understanding Vertical Market Dominance (2.4)
      │  │     └─ Recognizing Critical Success Requirements (2.5)
      │  └─ Recognizing AI's Impact on Role Evolution (3.1)
      │     └─ Understanding M-Shaped Developer Profile (3.3)
      │        └─ Understanding Systems Completeness (4.2)
      │           └─ Recognizing Technological Interdependencies (4.3)
      └─ Evaluating Personal Learning Strategy (3.2)

Understanding AIDD Nine Pillars (4.1)
└─ Understanding Systems Completeness (4.2)
   ├─ Recognizing Technological Interdependencies (4.3)
   └─ Understanding AI Development Methodology (1.5)
```

---

## Validation Checklist for Part 2+

Before adding new skills to Part 2, verify:

- [ ] **Uniqueness**: Does this skill already exist in registry? If so, are we extending it (next proficiency level)?
- [ ] **Naming**: Is skill name stable, consistent, and distinct from similar skills?
- [ ] **Prerequisites**: Have all prerequisite skills been taught?
- [ ] **Proficiency Order**: If skill appears multiple times, does proficiency increase monotonically (A1 → A1 or A2 is OK; A2 → A1 is NOT)?
- [ ] **Bloom Level**: Does cognitive level match CEFR proficiency? (A1=Remember/Understand, A2=Understand/Apply, B1=Apply/Analyze, etc.)
- [ ] **Cognitive Load**: Is the lesson within concept limits? (A1 max 5, A2 max 7, B1 max 10)
- [ ] **Measurability**: Can proficiency at each level be assessed objectively?

---

## Future Enhancements

1. **Skill Assessment Rubrics**: Define specific rubric for each skill at each CEFR level
2. **Skill Badging**: Create student-facing badges showing completed skills
3. **Prerequisite Enforcement**: Warn instructors/students when prerequisites haven't been met
4. **Learning Pathways**: Allow students to track personal progression through skill graph
5. **Competency Transcripts**: Generate transcripts showing skill mastery by domain
6. **Institutional Integration**: Export registry to LMS, credential systems, employer partnerships
