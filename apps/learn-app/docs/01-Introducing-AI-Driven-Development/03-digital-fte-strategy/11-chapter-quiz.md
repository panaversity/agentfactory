---
sidebar_position: 11
title: "Chapter 3: The Digital FTE Strategy Quiz"
description: "Comprehensive assessment of Digital FTE strategy frameworks: monetization, competitive positioning, vertical intelligence, market entry, and guardrails"
reading_time: "20 minutes"
chapter: 3
lesson: 11
duration_minutes: 45
---

# Chapter 3: The Digital FTE Strategy Quiz

Test your understanding of strategic frameworks for building and monetizing Digital Full-Time Employees, from expertise positioning to guardrails.

<Quiz
  title="Chapter 3: Digital FTE Strategy and Monetization"
  questions={[
    {
      question: "Why is expertise positioning the foundation of Digital FTE competitiveness?",
      options: [
        "Expertise matters less in the AI era because AI can access all knowledge instantly",
        "Your domain knowledge is the moat competitors cannot replicate; AI is the execution tool",
        "Expertise is important but only for traditional software companies, not Digital FTEs",
        "Generic positioning enables faster scaling than specialized expertise"
      ],
      correctOption: 1,
      explanation: "Chapter 3 emphasizes that your expertise IS your competitive moat. AI can write code anyone can write. But understanding YOUR healthcare workflows, YOUR finance operations, YOUR legal workflows—that takes months of domain research. Your expertise makes AI's code valuable. Without it, you're just a generic wrapper. Option B (correct) captures this foundation. Options A and C contradict the lesson's core premise. Option D is backwards; specialization creates defensibility.",
      source: "Lesson 1: Your Expertise as Product"
    },
    {
      question: "What makes a specialist positioning defensible against AI commoditization?",
      options: [
        "Specialist tools are more expensive to build so competitors avoid the market",
        "Specialists have exclusive access to proprietary AI models others cannot use",
        "Specialists understand specific workflows deeply; generic tools cannot match domain depth",
        "Specialists avoid technical complexity by only serving unsophisticated customers"
      ],
      correctOption: 2,
      explanation: "Generic ChatGPT handles general questions at 70% quality. Your healthcare subagent must handle clinical decisions at 99% quality because patient safety depends on it. This domain depth is irreplaceable. Option C (correct) explains the defensibility. Option A is backwards; specialization doesn't preclude competition. Option B is false; all AI competitors access same models. Option D contradicts that specialists serve demanding markets.",
      source: "Lesson 1: Your Expertise as Product"
    },
    {
      question: "Which competitive layer typically offers the fastest path to profitability?",
      options: [
        "Layer 1 (Consumer AI Backbone) offers fastest path because scale is largest",
        "Layer 3 (Vertical Market) offers fastest path because specialization reduces competition",
        "Layer 4 (Orchestrator) offers fastest path because network effects compound immediately",
        "Layer 2 (Developer Tools) offers fastest path because developers adopt tools quickly"
      ],
      correctOption: 1,
      explanation: "Layer 3 paths to profitability: deep domain relationships + strong integrations = $500K-$2M ARR in 18-24 months. Layer 1 demands billions in compute. Layer 4 requires multi-vertical network that takes years. Layer 2 has intense competition. Layer 3 offers sweet spot: specialized enough to have defensibility, large enough to reach $10M+ ARR. Option B (correct) identifies this. Option A contradicts that scale requires billions. Option C misses that network effects take time. Option D isn't path to profitability fastest.",
      source: "Lesson 2: Snakes and Ladders"
    },
    {
      question: "Why do Layer 2 tools have different defensibility than Layer 3 verticals?",
      options: [
        "Layer 2 tools are inherently more defensible due to larger markets",
        "Layer 3 verticals have deeper relationship-based moats; Layer 2 tools compete on feature parity",
        "Layer 2 tools scale faster because developers are easier to acquire than enterprise customers",
        "Layer 3 offers no defensibility compared to general developer tools"
      ],
      correctOption: 1,
      explanation: "Layer 2 defensibility: fast developer adoption, but features copy easily. Within 6 months, Claude's features are replicated. Layer 3 defensibility: deep integrations (Epic, Cerner, Bloomberg, CoStar), relationships with regulators, compliance approvals, workflow knowledge. These take 12+ months to replicate. Both scale but with different moats. Option B (correct) shows the distinction. Options A, C, D misunderstand the defense mechanisms.",
      source: "Lesson 2: Snakes and Ladders"
    },
    {
      question: "What is the core insight of 'disposable code' vs 'permanent intelligence'?",
      options: [
        "Code is more important than intelligence in software development generally",
        "AI generates code instantly per-use; intelligence (relationships, integrations) takes months to build",
        "Code is permanent and should be carefully maintained across applications",
        "Intelligence is less important than writing efficient code"
      ],
      correctOption: 1,
      explanation: "Pre-AI: maintain one code library across 5 products (high maintenance cost). AI era: generate code per-application instantly, maintain shared intelligence (system prompts, integrations, skills). The inversion is crucial. Code becomes commoditized; intelligence becomes scarce. Option B (correct) captures the inversion. Options A and D overstate code importance. Option C contradicts the lesson's core paradigm shift.",
      source: "Lesson 4: Vertical Intelligence"
    },
    {
      question: "Which component of vertical intelligence creates the most defensibility?",
      options: [
        "System prompts describing persona and knowledge scope",
        "Horizontal skills like Docker and Kubernetes available to all domains",
        "Deep MCP integrations with industry-specific systems (Epic, Bloomberg, CoStar)",
        "General training data available to all AI models"
      ],
      correctOption: 2,
      explanation: "MCP vertical integrations create the highest moat: competitors must rebuild months of API work, regulatory approvals, security audits, relationship-building. System prompts are valuable but easy to reverse-engineer. Horizontal skills are generic. Training data is public. Option C (correct) is the relationship-based moat. Others lack switching cost barriers.",
      source: "Lesson 4: Vertical Intelligence"
    },
    {
      question: "What does the 'five-component architecture' of vertical intelligence include?",
      options: [
        "Only system prompts and code generation (two components)",
        "System prompts, horizontal skills, vertical skills, MCP horizontal connections, MCP vertical connections",
        "Data storage, cloud infrastructure, databases, networks, and APIs",
        "Five different AI models combined to improve accuracy"
      ],
      correctOption: 1,
      explanation: "The five-component architecture: (1) System prompts (persona + scope), (2) Horizontal skills (Docker, Kubernetes—infrastructure), (3) Vertical skills (ICD-10, FHIR codes—domain), (4) MCP horizontal connections (dev tools), (5) MCP vertical connections (industry APIs—the moat). Option B (correct) lists all five. Options A, C, D misidentify components.",
      source: "Lesson 4: Vertical Intelligence"
    },
    {
      question: "Why is SKILL.md format important for Digital FTE development?",
      options: [
        "SKILL.md format is optional and doesn't affect Digital FTE quality",
        "SKILL.md provides standardized structure for packaging reusable domain intelligence",
        "SKILL.md is required only for open-source projects, not commercial products",
        "SKILL.md is outdated and replaced by newer specification formats"
      ],
      correctOption: 1,
      explanation: "SKILL.md standardizes how to package domain expertise: Persona definition, Questions to ask, Principles guiding application. This structure enables reuse across agents and enables composition into larger systems. Option B (correct) shows importance. Options A and D contradict its usefulness. Option C is incorrect; commercial products benefit most from structure.",
      source: "Lesson 4: Vertical Intelligence"
    },
    {
      question: "When would Subscription model be the better choice over Success Fee?",
      options: [
        "Subscription works better when outcomes are measurable and client incentives align",
        "Subscription works better when clients want predictable recurring costs and hands-off automation",
        "Subscription is always better regardless of domain or business model",
        "Success Fee is always better because it directly ties payment to performance"
      ],
      correctOption: 1,
      explanation: "Subscription model: predictable $500-2K/month recurring revenue, client delegates work to you. Success Fee: commission on measured outcomes, high trust required, client only pays if you deliver results. Different business models for different situations. Option B (correct) shows when subscription shines. Option A describes when Success Fee shines. Options C and D are too absolute.",
      source: "Lesson 5: Monetization Models Part 1"
    },
    {
      question: "What are the key tradeoffs in choosing between Subscription and Success Fee models?",
      options: [
        "Subscription has higher risk but lower profitability; Success Fee has lower risk but higher profitability",
        "Subscription offers predictable revenue but requires customer retention; Success Fee aligns incentives but requires measurable outcomes",
        "Both models offer identical risk and revenue profiles with no meaningful differences",
        "Success Fee has higher customer retention; Subscription has better customer alignment"
      ],
      correctOption: 1,
      explanation: "Subscription tradeoff: predictable monthly revenue but you must retain customers (support burden, renewal risk). Success Fee tradeoff: client only pays for value delivered (aligned incentives) but outcomes must be precisely measurable. Option B (correct) captures these tradeoffs. Options A and D misstate the tradeoffs. Option C ignores real differences.",
      source: "Lesson 5: Monetization Models Part 1"
    },
    {
      question: "When is License model the preferred monetization approach?",
      options: [
        "License model works best for consumer products with millions of users",
        "License model works best for enterprise customers needing data sovereignty and IP protection",
        "License model is never used in modern SaaS businesses",
        "License model is preferred for all B2B Digital FTE products equally"
      ],
      correctOption: 1,
      explanation: "License model characteristics: enterprise pricing ($50K-200K/year), customers keep data on-premise (HIPAA, SOC 2 requirements), IP protection. Works for healthcare (HIPAA), finance (PCI-DSS), regulated industries. Subscription/Success Fee assume cloud deployment. Option B (correct) identifies when license shines. Options A and C contradict the reality. Option D misses that some domains require on-premise.",
      source: "Lesson 6: Monetization Models Part 2"
    },
    {
      question: "Why might Marketplace model be wrong for specialized vertical Digital FTEs?",
      options: [
        "Marketplace model maximizes revenue and profitability for all products equally",
        "Specialized verticals need relationship-based sales (direct enterprise) more than marketplace reach",
        "Marketplace model is objectively superior for all monetization scenarios",
        "Specialized products always avoid marketplace because they're not sophisticated enough"
      ],
      correctOption: 1,
      explanation: "Marketplace (OpenAI, Anthropic platforms) offers reach to millions but commoditizes pricing. Specialized healthcare/finance/legal verticals need relationship-based trust and compliance, not marketplace reach. Enterprise customers won't buy regulated solutions from a marketplace plugin. Option B (correct) shows the mismatch. Options A and C overstate marketplace value. Option D is irrelevant.",
      source: "Lesson 6: Monetization Models Part 2"
    },
    {
      question: "What makes the decision matrix between four monetization models useful?",
      options: [
        "The matrix forces you to think through profitability, complexity, and competitive risk tradeoffs",
        "The matrix eliminates the need for strategic thinking about your specific domain",
        "All four models produce identical profitability across all domains",
        "The matrix applies equally to consumer products and enterprise Digital FTEs"
      ],
      correctOption: 0,
      explanation: "The decision matrix compares profitability vs complexity vs competitive risk across models. Forces strategic thinking: subscription is low-complexity/medium-profit but high retention risk. License is high-profit/high-complexity/enterprise-only. Marketplace is high-reach/low-profit/race-to-bottom. Your domain determines which tradeoff fits. Option A (correct) shows matrix value. Options B, C, D misunderstand its purpose.",
      source: "Lesson 6: Monetization Models Part 2"
    },
    {
      question: "In Phase 1 of PPP strategy, what is 'protocol' building specifically?",
      options: [
        "A protocol is a legal contract committing you to market entry",
        "A protocol is a technical standard (MCP) translating between fragmented incumbent systems",
        "A protocol is a marketing strategy for customer acquisition",
        "A protocol is a communication framework for team interactions"
      ],
      correctOption: 1,
      explanation: "Phase 1 protocol: technical standard becoming the bridge connecting Canvas, Blackboard, Google Classroom, etc. The 'protocol' is MCP—you're not replacing incumbents, you're augmenting them. This reduces competitive risk (you're not a replacement threat) and CAC (you're an add-on). Option B (correct) identifies the technical protocol. Others misidentify the concept.",
      source: "Lesson 7: PPP Market Entry Strategy"
    },
    {
      question: "Why is 60-80 customers 'critical mass' rather than 100 or 200?",
      options: [
        "60-80 customers is an arbitrary number with no strategic significance",
        "At 60-80, you've proven market fit, validated unit economics, and built key advocate relationships for Phase 3 pivot",
        "60-80 customers represents the maximum market size for any vertical",
        "More customers are always better; critical mass increases with each customer"
      ],
      correctOption: 1,
      explanation: "Critical mass criteria: (1) Proven market exists and has demand, (2) Unit economics stable (CAC, LTV, churn predictable), (3) Key decision-maker relationships built (these become Phase 3 advocates), (4) Workflow data collected (informs which subagents to build). 60-80 customers = 18-24 months of learning. More customers doesn't change critical mass definition. Option B (correct) defines it. Options A, C, D misunderstand the strategic purpose.",
      source: "Lesson 7: PPP Market Entry Strategy"
    },
    {
      question: "Why can't incumbents respond quickly to Phase 3 subagent pivot?",
      options: [
        "Incumbents have access to better AI models than independent developers",
        "Incumbents move slowly due to legacy code, regulatory delays, and misaligned financial incentives",
        "Incumbents are technically incompetent and cannot implement AI agents",
        "Incumbents typically respond to Phase 3 pivots faster than independent developers"
      ],
      correctOption: 1,
      explanation: "Incumbent constraints: millions of lines of legacy code (months to restructure), regulatory approval delays (HIPAA, PCI-DSS, SOC 2 audits), misaligned incentives (revenue comes from licenses, not automation). Solo developer: no legacy code, can pivot in weeks. Option B (correct) identifies structural constraints. Options A and C are false. Option D contradicts the lesson.",
      source: "Lesson 7: PPP Market Entry Strategy"
    },
    {
      question: "What triggers the transition from 'bridge' (Phase 2) to 'super orchestrator' (Phase 3)?",
      options: [
        "The entrepreneur suddenly receives venture capital funding",
        "The incumbent systems go out of business or shutdown",
        "You've validated workflow data proving which automations matter and earned decision-maker trust",
        "A fixed timeline passes regardless of customer understanding or readiness"
      ],
      correctOption: 2,
      explanation: "Phase transition is data-driven: (1) Customer data shows which workflows drive ROI (2) Decision-maker relationships built through bridge phase (3) You know exactly what subagents to build because you watched customers use bridge for 18 months. Option C (correct) shows the trigger. Options A and B are incorrect. Option D misses the data-driven nature.",
      source: "Lesson 7: PPP Market Entry Strategy"
    },
    {
      question: "Why is Domain Access listed as Requirement 1 for vertical success?",
      options: [
        "Domain expertise is less important than technical skill or capital availability",
        "Without domain access, your solution is generic AI and competitors replicate you in weeks",
        "Domain access is nice-to-have but not essential for Digital FTE success",
        "All three requirements are equally weighted without any primary requirement"
      ],
      correctOption: 1,
      explanation: "Domain access = relationships (customer introductions), credentials (trust), knowledge (workflows, pain points). Without it, you build generic healthcare AI that ChatGPT already does at 70%. Competitors with same API access build identical solutions unless you've added domain depth. Option B (correct) explains why it's Requirement 1. Options A and C understate its importance.",
      source: "Lesson 8: Three Requirements"
    },
    {
      question: "What does 'capital efficiency' specifically mean in the three requirements?",
      options: [
        "Capital efficiency means raising the maximum venture capital possible",
        "Capital efficiency means bootstrapping using General Agents and existing integrations instead of hiring large teams",
        "Capital efficiency is irrelevant to Digital FTE building in the AI era",
        "Capital efficiency means building your solution with the cheapest cloud infrastructure"
      ],
      correctOption: 1,
      explanation: "Capital efficiency in three requirements context: use Claude Code to build, use existing SKILL library for domain logic, avoid hiring expensive engineers. Enables $0-100K bootstrapped path to $1-2M ARR. Option B (correct) defines capital efficiency. Options A and D confuse other cost factors. Option C ignores its centrality.",
      source: "Lesson 8: Three Requirements"
    },
    {
      question: "Why is 'timing' considered a non-negotiable third requirement?",
      options: [
        "Timing is less important than domain access and capital efficiency",
        "The 2025-2027 window exists due to convergence of capability, adoption, and economics—missing it is existential",
        "Timing doesn't matter; you can build Digital FTEs successfully at any point",
        "Timing applies only to consumer products, not enterprise Digital FTEs"
      ],
      correctOption: 1,
      explanation: "Timing requirement: AI capability crossed threshold (can write expert-level code), adoption mainstream (51% of developers use AI daily), economics changed (human judgment bottleneck, not infrastructure). This convergence is rare and temporary. Window closes if adoption drops or capability plateau. Option B (correct) shows urgency. Options A, C, D misunderstand the window's significance.",
      source: "Lesson 8: Three Requirements"
    },
    {
      question: "What's an example of 'fully autonomous legal decisions' being inappropriate?",
      options: [
        "AI researching legal precedents and drafting briefs under attorney supervision",
        "AI making final legal decisions without human review before binding the client",
        "AI helping lawyers work faster by handling research and initial analysis",
        "AI providing training materials on legal concepts to junior lawyers"
      ],
      correctOption: 1,
      explanation: "Inappropriate: AI recommends filing suit without attorney review. Client is bound. Appropriate: AI researches, drafts, attorney reviews and decides. Lesson 9 emphasizes that AI can handle 90% (research, drafting) but humans must own 10% (final legal judgment). Option B (correct) shows inappropriate automation. Options A, C, D show appropriate human-in-the-loop patterns.",
      source: "Lesson 9: When NOT to Use Agents"
    },
    {
      question: "When would financial transaction automation be appropriate with guardrails?",
      options: [
        "Fully autonomous transactions without human authorization ever",
        "All financial transactions require human authorization regardless of amount or type",
        "Routine reconciliation and reporting with human sign-off for transactions over threshold ($10K, $50K)",
        "Financial automation is never appropriate for Digital FTEs in any situation"
      ],
      correctOption: 2,
      explanation: "Guardrailed approach: AI handles routine accounting and reconciliation (zero-risk mechanical work), AI recommends payments to vendors, human CFO authorizes amounts >$10K. This splits 90% (mechanics) and 10% (judgment). Option C (correct) shows appropriate guardrails. Options A and B are extremes. Option D contradicts that guardrailed automation is appropriate.",
      source: "Lesson 9: When NOT to Use Agents"
    },
    {
      question: "Why are audit trails non-negotiable in regulated industries?",
      options: [
        "Audit trails are optional and only needed for compliance theater",
        "Audit trails are required by regulations (HIPAA, SOC 2, PCI-DSS) to prove agent decisions were tracked and validated",
        "Audit trails slow performance and should be avoided in production systems",
        "Audit trails are only needed for customer-facing products, not internal tools"
      ],
      correctOption: 1,
      explanation: "Audit trails create accountability: which agent recommended this action, when, based on what data, with what confidence. Required for HIPAA (patient decisions), PCI-DSS (financial transactions), SOC 2 (security events). Non-negotiable for regulated verticals. Option B (correct) shows their purpose. Options A, C, D misunderstand their importance.",
      source: "Lesson 9: When NOT to Use Agents"
    },
    {
      question: "What is the 'shadow mode' testing strategy for guardrailed automation?",
      options: [
        "Shadow mode means keeping agents secret from customers until they're perfect",
        "Shadow mode means running agents in parallel with human decisions, comparing recommendations without customer seeing them",
        "Shadow mode is outdated terminology that no longer applies to modern AI agents",
        "Shadow mode refers to operating agents outside business hours to test them quietly"
      ],
      correctOption: 1,
      explanation: "Shadow mode: agent runs in background, makes recommendations, you compare to what humans would have decided. Build confidence before going live. Example: agent recommends patient discharge path, you compare to actual physician path. When >95% agreement achieved, deploy agent for real decisions with human review. Option B (correct) defines shadow mode. Others misunderstand the testing approach.",
      source: "Lesson 9: When NOT to Use Agents"
    },
    {
      question: "Why should you 'pause and reflect' before launching your Digital FTE?",
      options: [
        "Reflection is optional busy-work that doesn't contribute to Digital FTE success",
        "Reflection forces integration of all Chapter 3 frameworks into coherent strategy before execution",
        "You should skip reflection and launch immediately to capitalize on the opportunity window",
        "Reflection only matters for consumer products, not enterprise Digital FTEs"
      ],
      correctOption: 1,
      explanation: "Chapter 3 teaches six framework dimensions. Pause & Reflect lesson integrates them: expertise audit (L1), competitive positioning (L2), revenue model (L5-6), market entry timeline (L7), readiness check (L8), risk guardrails (L9). Without synthesis, you might optimize locally (pick wrong revenue model) while missing global strategy (wrong competitive layer). Option B (correct) shows reflection's value. Options A and C promote premature launch.",
      source: "Lesson 10: Pause and Reflect"
    },
    {
      question: "Which competitive layer matches a specialist building custom agents for their domain?",
      options: [
        "Layer 1 (Consumer AI Backbone) because reaching consumers is most scalable",
        "Layer 2 (Developer Tools) because agents are inherently developer-focused",
        "Layer 3 (Vertical Market) because domain expertise creates defensibility and relationships",
        "Layer 4 (Orchestrator) because single specialists should immediately coordinate multiple verticals"
      ],
      correctOption: 2,
      explanation: "Layer 3 is the specialist path: you have healthcare expertise (Layer 3) so you build healthcare agents. Not developer tools (Layer 2). You serve hospital workflow decisions, not developers. Network effects come later (Layer 4). Option C (correct) matches specialist to appropriate layer. Options A, B, D misalign expertise to layer.",
      source: "Lesson 2 + Lesson 1"
    },
    {
      question: "How does 'vertical intelligence reuse' differ from traditional 'code reuse'?",
      options: [
        "Vertical intelligence is less important than code reuse in modern systems",
        "Code reuse uses shared libraries; vertical intelligence generates specialized code per-customer but reuses domain knowledge",
        "Vertical intelligence and code reuse are the same concept with different names",
        "Code reuse is better for fast scaling; vertical intelligence is slower"
      ],
      correctOption: 1,
      explanation: "Traditional: maintain one library, use across 5 customers (maintenance burden). Vertical intelligence: generate fresh code per-customer (easy with AI), reuse domain knowledge (FHIR skills, integrations, system prompts). Inversion of value. Option B (correct) shows the paradigm shift. Options A, C, D misunderstand the distinction.",
      source: "Lesson 4: Vertical Intelligence"
    },
    {
      question: "What makes incumbents vulnerable to Layer 3 specialists despite having larger resources?",
      options: [
        "Incumbents are technically incompetent compared to specialists",
        "Incumbents move slowly due to legacy architecture, regulatory constraints, and misaligned incentives",
        "Incumbents intentionally ignore opportunities to focus on legacy markets",
        "Specialists have exclusive access to better technology than incumbents"
      ],
      correctOption: 1,
      explanation: "Incumbent vulnerability: Epic spent 30+ years building EHR. Specialist can rebuild for AI in weeks. Legacy code isn't bad; it's just slow to change. Rewriting for AI agents takes 6-12 months for incumbent. Specialist has no legacy, ships AI-first. Option B (correct) identifies structural vulnerability. Options A and C are false. Option D contradicts shared tech access.",
      source: "Lesson 2: Snakes and Ladders"
    },
    {
      question: "Why is the 'opportunity window' closing rather than opening wider over time?",
      options: [
        "The window is infinite and never closes",
        "The window closes when capability plateau, adoption drops, or incumbents adapt—reducing the force convergence",
        "The window closes when AI becomes too sophisticated and takes over all human decisions",
        "The window doesn't close; it only opens wider as more people realize the opportunity"
      ],
      correctOption: 1,
      explanation: "Window depends on three forces: (1) Capability high (crossing threshold), (2) Adoption mainstream (not niche), (3) Economics favor solo developers (human judgment bottleneck). If any force deteriorates, window tightens. If incumbents successfully integrate AI, window closes. Option B (correct) shows the forces' fragility. Options A, C, D misunderstand dynamics.",
      source: "Lesson 1: Your Expertise as Product"
    },
    {
      question: "What does the 'strategy company' vs 'software company' mindset shift really mean?",
      options: [
        "Strategy companies write no code and hire consultants for strategy advice",
        "Strategy companies win through understanding domain, relationships, and decision optimization—AI handles code",
        "Software companies are obsolete in the AI era and should all become strategy companies",
        "There's no meaningful distinction between strategy and software company mindsets"
      ],
      correctOption: 1,
      explanation: "Paradigm shift: traditional scaling requires hiring more engineers (code is bottleneck). Strategy company scales through better decisions (judgment is bottleneck). With AI handling code, you win by understanding customers better, relationships deeper, markets faster. Option B (correct) captures the mindset. Options A and C overstate. Option D misses the core distinction.",
      source: "Lesson 1: Your Expertise as Product"
    },
    {
      question: "How should you handle a domain where outcomes aren't easily measurable?",
      options: [
        "Success Fee model is best because outcomes don't need to be measurable",
        "Skip that domain entirely because unmeasurable outcomes prevent Digital FTE success",
        "Use Subscription or License model where client pays regardless of measurable outcome",
        "Unmeasurable outcome domains can't support any monetization model for Digital FTEs"
      ],
      correctOption: 2,
      explanation: "Success Fee requires measurable outcomes ($5 per qualified lead, 2% of savings). If outcomes not measurable (creative brainstorming, strategy advice), use Subscription ($2K/month managed service) or License (enterprise, outcome irrelevant). Option C (correct) shows how to adapt. Options A, B, D misunderstand that different models suit different outcome structures.",
      source: "Lesson 5-6: Monetization Models"
    }
  ]}
  questionsPerBatch={20}
/>
