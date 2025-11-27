# Chapter 33: Introduction to AI Agents — LoopFlow Prompt

## Command

```
/sp.loopflow.v2 Chapter 33: Introduction to AI Agents
```

## Context for Claude

You are writing **Chapter 33: Introduction to AI Agents**, the first chapter of **Part 6: AI Native Software Development**. This is a pivotal chapter that transitions students from learning AI-assisted development (Parts 1-5) to building AI-native applications.

### Why This Chapter Matters

**The AI landscape has fundamentally shifted:**
- More than 800 million people now use ChatGPT every week
- Over 90% of software developers use AI coding tools like Claude Code
- McKinsey reports AI agents could perform tasks occupying 44% of US work hours today
- Demand for AI fluency has grown 7x in two years—faster than any other skill in US job postings
- By 2030, $2.9 trillion of economic value could be unlocked through human-agent partnerships

Students completing this chapter will understand what AI agents are, why they matter, and how they differ from traditional software and simple LLM interactions.

### Audience Profile (From Parts 1-5)

Students arriving at Chapter 33 have:
- ✅ AI-Driven Development mindset (Part 1: Nine Pillars of AIDD)
- ✅ Proficiency with Claude Code and Gemini CLI (Part 2)
- ✅ Strong prompt and context engineering skills (Part 3)
- ✅ Spec-Driven Development with Reusable Intelligence (Part 4: SDD-RI)
- ✅ Production Python fundamentals (Part 5: Chapters 15-32)

They do NOT yet have:
- ❌ Understanding of agent architectures
- ❌ Knowledge of agent SDKs (OpenAI, Google ADK, Anthropic)
- ❌ Experience with MCP (Model Context Protocol)
- ❌ Multi-agent system design skills

### Teaching Framework: 4-Layer Model

Each lesson must progress through these layers:

**Layer 1 — Manual Foundation**: Explain concepts using official documentation, hands-on implementation without AI assistance. Students build mental models.

**Layer 2 — AI-Driven Development (AIDD)**: Demonstrate accomplishing the same objectives using Claude Code. Apply Three Roles framework (AI as Teacher, Student, Co-Worker) BUT keep framework INVISIBLE—students experience it through action prompts, not meta-commentary.

**Layer 3 — Reusable Intelligence**: Create subagents and skills that encode patterns for reuse. Students learn to avoid repetitive detailed prompts.

**Layer 4 — Spec-Driven Development (SDD)**: Chapter capstone project integrating all lessons using specification-driven approach.

### Key Resources to Reference

**Primary Source — Kaggle/Google Whitepaper**:
- "Introduction to Agents" (42 pages) by Julia Wiesinger, Patrick Marlow, Vladimir Vuskovic
- Covers: agent architectures, tool integrations, orchestration layers, training/evaluation
- URL: https://www.kaggle.com/whitepaper-introduction-to-agents
- Also reference: Agent Companion, Agent Quality whitepapers

**McKinsey Research — "Agents, robots, and us: Skill partnerships in the age of AI"**:
- AI agents could automate 44% of US work hours (not job losses—workflow transformation)
- Future work = partnership between people, agents, and robots
- 70%+ of employer-sought skills apply to both automatable and non-automatable work
- Skills like negotiation, coaching, design thinking remain uniquely human
- URL: https://www.mckinsey.com/mgi/our-research/agents-robots-and-us-skill-partnerships-in-the-age-of-ai

### Chapter Objectives

By completing this chapter, students will be able to:

1. **Define** what an AI agent is and distinguish agents from simple LLM interactions, chatbots, and traditional automation
2. **Explain** the core components of an agent system (model, tools, orchestration, memory, evaluation)
3. **Identify** the major agent frameworks and SDKs (OpenAI Agents SDK, Google ADK, Anthropic Agents Kit, LangChain/LangGraph)
4. **Recognize** agent design patterns (ReAct, Plan-and-Execute, Multi-Agent Systems)
5. **Understand** the human-agent partnership model (where humans frame problems, provide guidance, interpret results, and make decisions)
6. **Apply** Layer 1-3 thinking to simple agent conceptual examples
7. **Prepare** for subsequent chapters on specific SDK implementations

### Lesson Structure Guidance

**Suggested Lessons** (adjust based on spec development):

1. **What Is an AI Agent?** — Definition, distinction from chatbots/automation, the agency spectrum
2. **Anatomy of an Agent** — Core components: model, tools, orchestration, memory, evaluation
3. **Agent Architectures & Patterns** — ReAct, Plan-and-Execute, Multi-Agent, Human-in-the-Loop
4. **The Agent SDK Landscape** — Overview of OpenAI, Google ADK, Anthropic, LangChain/LangGraph
5. **Human-Agent Partnerships** — Where agents excel, where humans are essential, collaboration models
6. **Your First Agent Concept** — Specification exercise (no code yet—that's Chapter 34+)

### Critical Constraints

**DO**:
- Use real statistics (800M ChatGPT users/week, 90%+ developers using AI tools, $2.9T value)
- Reference Kaggle whitepaper concepts with proper attribution
- Show why agents matter for developers (career relevance)
- Connect to SDD-RI mindset from Part 4
- End each lesson with "Try With AI" section (action prompts, not meta-commentary)

**DO NOT**:
- Write code implementations (that's Chapters 34-48)
- Deep-dive into any single SDK (that's dedicated chapters)
- Use meta-commentary ("Notice how AI is teaching you...")
- Create toy examples disconnected from professional context
- Skip Layer 1 manual understanding before AI assistance

### Differentiation from Subsequent Chapters

| Chapter | Focus |
|---------|-------|
| **33 (This)** | Concepts, architectures, landscape overview |
| 34 | OpenAI Agents SDK hands-on |
| 35 | Google ADK hands-on |
| 36 | Anthropic Agents Kit hands-on |
| 37 | MCP Fundamentals |
| 38+ | Advanced patterns, databases, testing |

Chapter 33 provides the conceptual foundation; subsequent chapters provide implementation depth.

### Success Criteria

✅ Student can explain agents vs chatbots to a colleague
✅ Student can name and describe 3+ agent frameworks
✅ Student understands the human-agent partnership model
✅ Student is excited and prepared for Chapter 34 (first hands-on SDK chapter)
✅ All lessons follow 4-Layer progression
✅ No code implementations (conceptual only)
✅ All statistics properly cited

---

## Execute

Run `/sp.loopflow.v2` with this context. The orchestrator will:
1. Read chapter-index.md and constitution
2. Generate spec using this prompt as input
3. Generate plan with lesson breakdown
4. Generate tasks
5. Implement with validation gates

The chapter should be **conceptually rich but implementation-light**—setting up students for the deep SDK chapters that follow.
