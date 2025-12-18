---
title: "The Concept Behind Skills"
sidebar_position: 5
chapter: 5
lesson: 5
duration_minutes: 12

# PEDAGOGICAL LAYER METADATA
primary_layer: "Layer 1"
layer_progression: "L1 (Conceptual Framework)"
layer_1_foundation: "Understanding skills architecture, expertise gap, platform-level design philosophy, MCP complementarity"
layer_2_collaboration: "N/A (preparation for L2 in Lesson 06)"
layer_3_intelligence: "N/A"
layer_4_capstone: "N/A"

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding Agent Skill Architecture"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can explain the skills architecture, distinguish skills from MCP servers, and articulate why simplicity enables adoption"

learning_objectives:
  - objective: "Explain why the bottleneck is expertise, not intelligence"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Articulation of the expertise gap concept"
  - objective: "Describe the three-level loading architecture"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of metadata → instructions → supporting files"
  - objective: "Distinguish skills from MCP servers"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Explanation of expertise vs. connectivity"
  - objective: "Explain the stack analogy (models/runtimes/skills)"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Mapping of computing concepts to agent concepts"

# Cognitive load tracking
cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (expertise gap, three-level loading, three sources, MCP complementarity, compounding value, stack analogy, non-technical accessibility) - at B1 limit ✓"

# Differentiation guidance
differentiation:
  extension_for_advanced: "Analyze how skills would evolve with explicit dependencies, versioning, and cross-skill composition"
  remedial_for_struggling: "Focus on tax professional analogy and stack analogy as primary mental models"

# Generation metadata
generated_by: "content-implementer v1.0.0 (044-lesson-05-concept-behind-skills)"
source_spec: "specs/044-lesson-05-concept-behind-skills/spec.md"
created: "2025-12-17"
last_modified: "2025-12-17"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"

# Legacy compatibility
prerequisites:
  - "Lesson 04: Teach Claude Your Way of Working"
  - "Understanding of personal procedures worth encoding"
---

# The Concept Behind Skills

"Stop building agents. Build skills instead."

That's not advice from a random blog post. It's the conclusion Anthropic reached after building Claude Code and watching how people actually extended it. The talk that introduced Agent Skills challenged a fundamental assumption about AI customization.

![lesson-4-skills-growth](https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/images/part-2/chapter-05/lesson-4-skills-growth.png)

**The assumption**: Making AI useful for YOUR domain requires building a specialized agent. Finance needs a finance agent. Legal needs a legal agent. Each domain demands its own infrastructure.

**The discovery**: The agent underneath is far more universal than anyone expected. Code isn't just a use case—it's the universal interface to the digital world. A coding agent can pull data from APIs, organize files, analyze with Python, synthesize reports. The scaffolding can be as thin as Bash and a file system.

The agent already exists. What's missing is your expertise in a format it can access.

---

## Intelligence + Code = Execution (But Not Expertise)

Here's how the Anthropic team frames what they built:

**Models provide intelligence.** Claude can reason, analyze, synthesize, and generate. That's the cognitive capability—the "thinking" part.

**Code provides execution.** Through a terminal, Claude can call APIs to pull data, organize information in the file system, analyze with Python, and output results in any format. Code is the universal interface to the digital world.

Put them together: an intelligent agent that can execute. Claude Code isn't just a chatbot—it can actually DO things. Read your files. Run your tests. Commit your code. Generate reports.

**But intelligence + execution still isn't expertise.**

Agents today have intelligence and capabilities, but not always the expertise needed for real work. They can do amazing things when you provide detailed guidance. But they're missing the important context up front. They can't absorb your organizational knowledge efficiently. They don't learn from your feedback over time.

That's the gap skills fill. Not more intelligence. Not more execution capability. **Expertise**—the domain-specific knowledge that makes generic capability specifically useful.

---

## Skills Are Universal, Not Just for Coding

Here's a critical insight: **skills work for ANY domain, not just software development.**

![skills-universal-domains](https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/images/part-2/chapter-05/skills-universal-domains.png)

Claude Code runs in a terminal, yes. But "code" is the universal interface to the digital world. Through code, Claude can:
- Write and format documents (Word, PowerPoint, Excel, PDF)
- Analyze data and create visualizations
- Process and organize information
- Generate reports in any format
- Automate repetitive digital tasks

**Skills encode expertise in ANY field:**

| Domain | Example Skills |
|--------|---------------|
| **Finance** | Quarterly report formatting, audit procedures, compliance checklists |
| **Legal** | Contract review workflow, clause analysis, due diligence procedures |
| **Marketing** | Brand voice guidelines, campaign brief templates, social media style |
| **Education** | Lesson plan structure, assessment rubrics, student feedback formats |
| **Healthcare** | Clinical documentation standards, patient communication templates |
| **Recruiting** | Candidate evaluation criteria, interview question frameworks |

The accountant who knows exactly how audits should be structured? That's a skill.
The recruiter who knows what makes candidate evaluations useful? That's a skill.
The legal team's contract review workflow? That's a skill.

**You don't need to be a programmer to create or use skills.** You need domain expertise and the willingness to document your procedures clearly.

---

## The Real Bottleneck Isn't Intelligence

Who do you want doing your taxes? The 300-IQ mathematical genius who figures out tax code from first principles? Or the experienced tax professional who knows the patterns, the edge cases, the specific procedures that work?

You want the professional. Not because they're smarter—because they have encoded expertise.

This is exactly the gap with AI agents today. Claude is brilliant—it can do amazing things when you invest effort in guidance. But it often lacks the context you've built up over years. It can't absorb your organizational expertise efficiently. And it doesn't learn from your feedback over time.

The solution isn't making the model smarter. The solution is giving it access to expertise it doesn't have.

The question becomes: how do you give Claude that same advantage?

---

## Won't Many Skills Overload Context? No.

You might worry: "If I have 50 skills, won't Claude run out of memory loading them all?"

This is exactly the problem skills are designed to solve.

**Skills are organized collections of files**—in simpler terms, they're folders. This simplicity is deliberate. The design principle: anything that anyone—human OR agent—can create and use, as long as they have a computer.

**The folder structure:**

```
.claude/skills/
├── meeting-notes/           # Each skill is a folder
│   ├── SKILL.md             # Main instructions (loaded on-demand)
│   └── templates/           # Supporting files (loaded if needed)
│       └── standup.md
├── code-review/
│   ├── SKILL.md
│   └── checklist.md
└── blog-planner/
    └── SKILL.md
```

Skills work with what you already have. Version them in Git. Throw them in Google Drive. Zip them up and share with your team. Files have been a primitive for decades. Why change now?

**The three-level architecture protects context:**

Claude has limited working memory (context window). If it loaded every skill's full instructions at startup, it would run out of space before doing any actual work. Skills solve this with progressive disclosure:

**Level 1 — Brief Metadata (Always Loaded)**: A short description of what the skill does and when it's relevant. Just enough for Claude to know the skill exists.

**Level 2 — Full Instructions (On-Demand)**: When Claude decides a skill applies, it loads the complete SKILL.md with detailed procedures, workflows, examples.

**Level 3 — Supporting Files (If Needed)**: Scripts, reference documentation, tools in the skill's directory. Accessed only when executing the skill.

Think of your smartphone. You have 100 apps installed. Your phone doesn't run all 100 at once—it would crash. Apps stay closed until you tap them. Skills work the same way: available when needed, dormant otherwise.

This architecture enables scale. You can have hundreds of skills without overwhelming context. Claude activates the relevant ones and ignores the rest.

But if skills are just folders, where do they come from?

---

## Three Sources of Encoded Expertise

Skills emerge from different places, each serving different needs.

**Foundational Skills**: Basic capabilities that extend what Claude can do out of the box.

Examples: Creating Word documents, PowerPoint presentations, Excel spreadsheets, PDFs. These are like basic life skills—everyone needs them.

**Partner Skills**: Help Claude work effectively with specific software or services.

Examples: Browserbase built a skill for their open-source browser automation tooling (Stagehand). Now Claude equipped with this skill can navigate the web and automate browser tasks far more effectively. Notion launched skills that help Claude understand your entire workspace and do deep research across your documents.

These are like specialized certifications—expertise in specific tools.

**Enterprise and Custom Skills**: Created by organizations for their specific needs.

Examples: Your company's coding style guide. Internal documentation standards. Organization-specific workflows that encode "how we do things here."

**Here's where the real traction is**: Anthropic has been talking to Fortune 100 companies using skills to teach agents about organizational best practices and the unique ways they use bespoke internal software. Developer productivity teams—serving thousands or tens of thousands of engineers—deploy skills to teach Claude Code about internal code style standards and preferred workflows.

Within five weeks of launch, thousands of skills were created across all three categories. The ecosystem is growing fast because the format is simple enough that anyone can contribute.

What makes skills especially powerful is what they combine with.

---

## Skills + MCP = Expertise + Connectivity

MCP servers (covered later in this chapter) connect Claude to external data and tools—APIs, databases, file systems, web browsers.

Skills and MCP serve different functions:

| Component | What It Provides |
|-----------|------------------|
| **MCP Servers** | Connection to outside world (data, tools, APIs) |
| **Skills** | Expertise for USING those connections effectively |

**Consider a concrete example:**

An MCP server connects Claude to your company's database. Claude can now query data, retrieve records, run reports.

A skill encodes your company's data analysis procedures—what reports to generate, what format to use, what insights to highlight, what anomalies to flag.

Without the skill: Claude can access data but doesn't know your reporting standards. It produces generic output.

Without MCP: Claude knows your standards but can't access the data. Knowledge without action.

Together: Claude queries the database (MCP), analyzes using your procedures (skill), and produces reports that match your organization's expectations.

**The pattern developers are building**: Skills that orchestrate workflows across multiple MCP tools stitched together. MCP provides the connectivity. Skills provide the expertise for using that connectivity well.

They're complementary, not competing. Equip Claude with the right MCP servers AND the right library of skills, and you have an agent configured for your specific domain.

This architecture is already enabling rapid deployment. When Anthropic launched Claude for financial services and life sciences, each offering came with a set of MCP servers for domain-specific data access AND a set of skills encoding domain-specific expertise.

But this raises a question: who creates these skills?

---

## The Accessibility Revolution

You might still be thinking: "This sounds like a developer thing."

Here's what Anthropic observed in the first weeks after launch: skills are being built by people who aren't technical. People in finance. Recruiting. Accounting. Legal.

This is early validation of the design principle. Skills help people who don't write code extend general-purpose agents. They make AI assistants accessible for day-to-day work across functions.

**Why this works:**

Domain experts have the knowledge. A senior accountant knows exactly how audits should be structured. A recruiting lead knows what makes candidate evaluations useful. A legal team knows their contract review workflow inside and out.

What these experts lacked was a mechanism to transfer that knowledge to AI. Traditional tools required technical implementation. Skills require clear instructions in a folder.

A recruiter's candidate evaluation checklist becomes a skill. An accountant's audit procedure becomes a skill. A legal team's contract review workflow becomes a skill.

**The writing analogy:**

You don't need to be a programmer to write a great email template. You don't need to be a programmer to document your meeting note procedure. Skills extend this principle: document your expertise clearly, and Claude applies it.

The format—markdown files with YAML metadata—is accessible to anyone who can write structured text. The barrier isn't technical skill. It's willingness to articulate your procedures.

If skills can be created by anyone, what happens when many people create them?

---

## Skills as Strategic Assets

Here's what most people miss: **skills aren't just convenient—they're valuable intellectual property.**

Compare manual prompting (explaining your preferences every time) to encoded skills:

| Aspect | Manual Prompting | Agent Skills |
|--------|-----------------|--------------|
| **Reliability** | Ad-hoc, best effort | Deterministic, script-backed |
| **Token Cost** | Pay for "rules" in every conversation | Load rules only when triggered |
| **Asset Type** | Disposable conversation | **Reusable, scalable IP** |
| **Integration** | Requires human copy-paste | **API-ready via Agent SDKs** |

That last row is critical. Skills you create in Claude Code can be:
- **Shared** with your team (everyone benefits from your expertise)
- **Versioned** in Git (track improvements over time)
- **Integrated** into Custom Agents (Part 6 teaches this)
- **Monetized** as part of vertical AI solutions

When you create a skill, you're not just saving keystrokes. You're encoding expertise that compounds in value—for yourself, your team, and potentially your business.

---

## The Compounding Value of Shared Knowledge

The vision that excites the Anthropic team most: a collective, evolving knowledge base of capabilities curated by people AND agents inside an organization.

Skills provide the procedural knowledge for your agents to do useful things. As you interact and give feedback, skills improve. The agent gets better.

**The organizational multiplier:**

When your skills improve, ALL agents in your team and organization get better. Not just yours. Everyone using the same skill library benefits from the refinement.

**The onboarding transformation:**

When someone new joins your team and starts using Claude for the first time, it already knows what your team cares about. It knows your day-to-day workflows. It knows how to be most effective for YOUR work.

No ramp-up period. No weeks of context-sharing. The expertise is already encoded.

**Beyond your organization:**

Just as someone else building an MCP server makes your agent more useful (you gain connectivity you didn't build), someone else building a skill makes your agent more capable.

A skill built by someone in the community helps agents everywhere. Contribution compounds capability.

**The continuous learning direction:**

Skills are designed as a concrete step toward continuous learning. Here's the key insight: anything Claude writes down can be used efficiently by a future version of itself.

This makes learning transferable. As you build context with Claude over time, skills make memory tangible. They don't capture everything—just procedural knowledge Claude can use on specific tasks.

Claude can already create skills for you today using a "skill creator" skill. The goal: Claude on Day 30 of working with you is dramatically better than Claude on Day 1.

Where does this all fit in the bigger picture?

---

## The Stack Analogy

The Anthropic team draws a parallel to what we've already seen in computing.

![lesson-4-skills-in-stack](https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/images/part-2/chapter-05/lesson-4-skills-in-stack.png)

**Models are like processors.** Both require massive investment and contain immense potential. But a processor by itself isn't that useful. You need systems built around it.

**Agent runtimes are like operating systems.** The OS made processors far more valuable by orchestrating processes, resources, and data around the processor. In AI, agent runtimes (like Claude Code) play this role—the cleanest, most efficient abstractions to get the right tokens in and out of the model.

**Skills are like applications.** A few companies build processors and operating systems. But millions of developers build software that encodes domain expertise and unique points of view.

**This is the layer that opens for everyone.**

You don't need to build the model. Anthropic did that. You don't need to build the agent runtime. Claude Code exists. What you need to build: the applications layer. Skills that encode YOUR expertise, YOUR procedures, YOUR organizational knowledge.

The paradigm shift: **Stop building agents. Build skills instead.**

The agent infrastructure is mature. The extensibility mechanisms (MCP for connectivity, skills for expertise) are in place. The value creation happens in what YOU contribute—the domain knowledge that makes the general-purpose agent specifically useful for your work.

---

## From Understanding to Action

You now understand the architecture:

- The bottleneck isn't AI intelligence—it's access to specialized expertise
- Skills are intentionally simple (folders) to enable universal adoption
- Three-level loading keeps context efficient at scale
- Skills complement MCP (expertise + connectivity)
- Non-technical users can create high-value skills
- Skills compound across teams and organizations
- Skills are the "applications layer" of the agent stack

Lesson 06 teaches execution:

- The SKILL.md format and YAML metadata structure
- Creating a working blog-planner skill from scratch
- Testing skill activation and refinement
- Decision criteria: when skills vs. subagents

The procedure you mapped in Lesson 04—your repeated task with clear preferences and quality criteria—that's your raw material. Lesson 06 shows you how to encode it.

---

## Try With AI

**Understand the Architecture:**

> "Anthropic says 'code is all we need' as the universal interface for agents. Help me understand: how does a coding agent (like Claude Code) become useful for non-coding tasks like finance reports or legal research? Walk me through the architecture—what role do skills play in making a coding agent general-purpose?"

**Identify Skill Opportunities in Your Domain:**

> "I work in [your domain: marketing / finance / research / operations / legal / etc.]. Based on the three skill sources (foundational, partner, enterprise), help me identify: (1) What foundational skills probably already exist for my work? (2) What third-party skills might exist for tools I use? (3) What custom skills would capture MY team's specific procedures?"

**Evaluate the Stack Analogy:**

> "Explain the 'skills are like applications' analogy. If Claude is the processor and Claude Code is the operating system, what makes skills different from just being good at prompting? Why is encoding expertise as a skill different from getting better at asking questions?"

**Challenge the Paradigm:**

> "The Anthropic talk claims 'stop building agents, build skills instead.' When WOULD someone still need to build a custom agent? What can't skills do? Help me understand where the boundary is between 'a skill is enough' and 'you need something more.'"

Note: These prompts explore the conceptual framework. Lesson 06 teaches hands-on skill creation—the syntax, structure, and workflow for actually building your first skill.

---

## Watch the Original Talk

This lesson is based on the Anthropic talk "Don't Build Agents, Build Skills Instead" by Barry Zhang and Mahesh Murag. Watch the full presentation to hear these concepts directly from the team that built Claude Code and Agent Skills:

<iframe
  width="100%"
  height="450"
  src="https://www.youtube.com/embed/CEvIs9y1uog"
  title="Don't Build Agents, Build Skills Instead - Barry Zhang & Mahesh Murag (Anthropic)"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
  allowfullscreen>
</iframe>

**Key timestamps:**
- **0:21** — Introduction and the expertise gap
- **1:16** — "Code is all we need" — the universal interface insight
- **2:14** — The tax professional vs. mathematical genius analogy
- **2:59** — What are Agent Skills? (folders!)
- **4:20** — Progressive disclosure architecture
- **5:00** — The skills ecosystem (foundational, partner, enterprise)
- **9:06** — The complete architecture diagram
- **12:02** — Sharing and distribution vision
- **14:32** — The stack analogy (processors → OS → applications)
