---
title: "Introduction to Reusable Intelligence"
chapter: 30
lesson: 6
duration: "60-75 minutes"
skills:
  - name: "Pattern Recognition"
    proficiency: "B1"
    category: "Analytical"
  - name: "Reusable Intelligence Design"
    proficiency: "B1"
    category: "Technical"
  - name: "Decision Framework Application"
    proficiency: "B1"
    category: "Conceptual"
learning_objectives:
  - "Transform recurring specification patterns into reusable components (Skills and Subagents) (B1)"
  - "Distinguish between Skills (2-4 decisions, guidance frameworks) and Subagents (5+ decisions, autonomous reasoning) (B1)"
  - "Apply decision framework to identify when to encode patterns versus write fresh specifications (B1)"
---

# Introduction to Reusable Intelligence

In Lessons 1-5, you've learned the foundation: writing clear specifications, collaborating with AI to refine them, and creating shared governance rules (Constitutions) to ensure consistency across your team.

But you've probably noticed something: **specifications for authentication, validation, error handling, and security reviews start to look similar**. The same decisions appear again and again. The same questions get asked. The same principles apply.

This lesson introduces the next level of SDD-RI thinking: **Reusable Intelligence (RI)**—the idea that once you've solved a pattern well, you can capture that solution and reuse it across projects, features, and teams.

---

## Real Skill Example — Error Handling Pattern

Let's start with a concrete example from actual professional practice.

### The Pattern You've Seen Before

Imagine you've written specifications for three features:

1. **User Authentication API**: "If login fails, return 401 Unauthorized with error message"
2. **Payment Processing API**: "If payment fails, return 402 Payment Required with error details"
3. **File Upload Service**: "If upload exceeds size limit, return 413 Payload Too Large with guidance"

Same pattern repeating:
- HTTP status codes (4xx for client errors, 5xx for server errors)
- Structured error responses (JSON with `error`, `message`, `code`)
- Logging requirements (errors logged with request ID for debugging)

**Question**: Do you want to re-specify this every time? Or can you capture the pattern once and reuse it?

### Meet a Real Skill: `assessment-builder`

In the `.claude/skills/` directory, there's a skill called **`assessment-builder`** that captures this exact kind of pattern:

**What it does**:
- Provides a framework for designing assessments aligned with learning objectives
- Suggests appropriate question types (MCQ, code-writing, debugging, projects)
- Guides you toward meaningful distractors based on common misconceptions
- Ensures cognitive distribution (not just memorization questions)

**When you use it**:
- Any time you're creating a quiz or test
- When designing chapter assessments
- When you want consistent assessment quality across the book

**What it encodes**:
- Evals-first design: Define success criteria BEFORE creating questions
- Bloom's taxonomy alignment: Match question type to cognitive level
- Question design patterns: MCQ structure, code-challenge structure, rubric criteria
- Quality checks: Is this assessment measuring what we intend?

**Reusable across**:
- All 50+ chapters of the book
- Different domains (Python chapters, AI chapters, architecture chapters)
- Different question types (still uses same framework)

**Example**: Instead of writing "Create a challenging question about decorators," you specify:
```
Learning objective: Students understand decorator syntax and use cases
Bloom's level: Apply (not just Remember)
Question type: Code-completion challenge
Domain: Python syntax
```

The skill provides the framework—you fill in domain specifics. The framework is reusable; the domain details are context-specific.

### Anatomy of a Skill

A Skill is **2-4 decisions with guidance**. It answers recurring questions:

```
Skill: "Error Handling Pattern"

Decision 1: What HTTP status codes apply?
  → Guidance: Use 4xx for client errors, 5xx for server errors
  → Reference table: 400 Bad Request, 401 Unauthorized, 404 Not Found, etc.

Decision 2: What should error response JSON contain?
  → Guidance: Always include: error code, human-readable message, request ID for debugging
  → Template: { "code": "...", "message": "...", "request_id": "..." }

Decision 3: How should errors be logged?
  → Guidance: Log at ERROR level with full context for server errors, INFO for expected 4xx
  → Pattern: Include request ID in every log line

Decision 4: Should errors include stack traces?
  → Guidance: In development (helpful), NOT in production (security risk)
  → Implementation: Check environment variable, log stacktrace conditionally
```

**Skill characteristics**:
- Horizontal applicability (applies broadly across many features)
- Guidance-based (provides framework, not rigid rules)
- Reusable across projects without major modification
- Typically 2-4 decision points

### Practice: Which Patterns from L1-5 Could Be Skills?

Look at the specifications you've written (or reviewed) in Lessons 1-5:

1. **Password validation** (strength requirements, hashing, rate limiting)
   - Would this be a good Skill? Why/Why not?
   - How often would you reuse it?

2. **API pagination** (page size limits, cursor management, response format)
   - Pattern? Reusable? Decision count?

3. **Database transaction handling** (rollback, retry logic, deadlock recovery)
   - Could this be captured as guidance?

**Reflection**: _Write 1-2 sentences about which patterns from your own experience could become Reusable Intelligence._

---

## Real Subagent Example — Validation Auditor

Now let's look at something more complex. Some patterns have more decisions and deeper expertise.

### The Complex Pattern: Quality Assurance

Imagine you're building a large system with multiple development teams. Each team writes features—database features, API features, UI features, infrastructure features.

You need consistent quality validation across all features:
- **Technical correctness**: Does code execute? Are dependencies managed?
- **Security**: Are inputs validated? Secrets hardcoded?
- **Performance**: Could this become a bottleneck?
- **Accessibility**: Are UI components accessible?
- **Pedagogical effectiveness** (if teaching code): Does it explain concepts clearly?

This isn't a simple checklist. Different features need different emphasis. A database feature needs different validation than a UI component. A security feature needs different scrutiny than a logging utility.

**Question**: Could you capture this as a simple 4-decision Skill? Or does it need autonomous reasoning?

### Meet a Real Subagent: `validation-auditor`

In the `.claude/agents/` directory, there's a subagent called **`validation-auditor`** that does exactly this:

**What it does**:
- Reviews content (lessons, code, features) across 4 quality dimensions
- Makes independent quality judgments (not just checklists)
- Requires reasoning: "This is pedagogically weak because..."
- Adapts validation based on context (lesson vs code vs infrastructure)

**When you use it**:
- Any time content is complete and needs publication review
- When you need multi-dimensional quality assessment
- When simple checklists aren't enough (context matters)

**What it reviews**:
- **Dimension 1**: Technical correctness (code execution, best practices)
- **Dimension 2**: Pedagogical effectiveness (clarity, scaffolding, examples)
- **Dimension 3**: Factual accuracy (verifiable claims, citations)
- **Dimension 4**: Accessibility (inclusive, clear language)

**Autonomous reasoning**:
- Makes independent judgments ("This lesson is pedagogically weak because X")
- Provides reasoning ("The example is disconnected from real use because...")
- Identifies complex issues (not just checklist violations)
- Adapts to context (validation differs by artifact type)

**Reusable across**:
- All chapters and lessons
- Code features and architecture decisions
- Documentation and design artifacts
- Different domains (Python chapters, AI chapters, etc.)

### Anatomy of a Subagent

A Subagent is **5+ decisions with autonomous reasoning**. It requires domain expertise and contextual judgment:

```
Subagent: "validation-auditor"

Decision 1: Is code technically correct?
  → Checks: Execution, best practices, dependencies, security
  → Reasoning: Makes judgment calls ("This async pattern could deadlock because...")
  → Autonomy: Doesn't just verify checklist; provides analysis

Decision 2: Is pedagogy effective?
  → Checks: Learning objectives, scaffolding, examples, engagement
  → Reasoning: "This example doesn't connect to real use because..."
  → Autonomy: Decides whether pedagogical effectiveness meets bar

Decision 3: Is content factually accurate?
  → Checks: Verifiable claims, citations, current best practices
  → Reasoning: "This claim about GIL is outdated; here's the 2025 context..."
  → Autonomy: Makes judgment about accuracy level needed

Decision 4: Is content accessible?
  → Checks: Inclusive language, clear terminology, multiple explanations
  → Reasoning: "This assumes prior knowledge of decorators that isn't taught..."
  → Autonomy: Evaluates accessibility independently

Decision 5+: Synthesis and recommendations
  → Integrates findings across dimensions
  → Prioritizes issues (critical vs. nice-to-have)
  → Provides actionable remediation guidance
```

**Subagent characteristics**:
- Vertical applicability (deep expertise in specific domain)
- Autonomous reasoning (not just guided by checklist)
- Requires contextual judgment (not rule-based)
- Typically 5+ decision points
- Provides explanations and trade-off analysis

### Practice: When Would You Need the Validation Auditor?

Think about your project context:

1. **If you're teaching a Python lesson**: Would validation-auditor help catch pedagogy issues you might miss?
2. **If you're writing infrastructure code**: Could autonomous security reasoning catch vulnerabilities?
3. **If you're designing a feature spec**: Would multi-dimensional validation improve quality?

**Reflection**: _Describe one situation in L1-5 where you would have benefited from this Subagent's reasoning. What issue might it have caught?_

---

## Decision Framework — When to Use What

Now that you've seen a Skill and a Subagent, how do you decide which type of Reusable Intelligence to create?

### The Decision Matrix

Three types of reusable patterns emerge as you practice SDD:

#### Type 1: Constitutional Rules (Applies to ALL Specs)

**Recognition signals**:
- Applies to literally every specification in your organization
- Non-negotiable governance rule
- Prevents common mistakes across all domains

**Examples**:
- "All specs must have Intent, Requirements, Constraints, Non-Goals"
- "All API specs must define error responses"
- "All password systems must use bcrypt hashing"

**What you already did**: L5 (Constitutions) covers this. Constitutional rules are the highest level of reusable intelligence—they apply universally.

#### Type 2: Skills (2-4 Decisions, Horizontal Applicability)

**Recognition signals**:
- Pattern repeats 2-3 times across similar features
- 2-4 key decision points
- Applies broadly without major customization
- Answers recurring questions

**Examples**:
- **Error Handling Skill**: "How should APIs return errors?" (applies to ALL APIs)
- **Pagination Skill**: "How should paginated responses work?" (applies to LIST endpoints)
- **Authentication Skill**: "What authentication patterns suit my project?" (applies to protected endpoints)
- **Validation Skill**: "How should I validate user input?" (applies to ALL user-facing features)

**Format**: Decision 1 → Decision 2 → Decision 3 → Decision 4 → Reusable framework

**Reusability test**:
```
If I'm writing spec #3 with similar needs to specs #1-2:
- Do I ask the same questions?
- Are the answers similar?
- Could I apply the same guidance with minor tweaks?
→ YES: Create a Skill
```

#### Type 3: Subagents (5+ Decisions, Vertical Expertise)

**Recognition signals**:
- Pattern requires domain expertise (not just process)
- 5+ decision points with complex reasoning
- Applies deeply in specific domain
- Requires autonomous judgment (not just guided decisions)

**Examples**:
- **Security Auditor**: Reviews input validation, encryption, authentication, authorization (5+ security decisions)
- **Performance Optimizer**: Analyzes data volumes, latency requirements, caching strategies, query patterns (5+ performance decisions)
- **Accessibility Reviewer**: Checks color contrast, screen reader compatibility, keyboard navigation, cognitive load (5+ accessibility decisions)
- **API Designer**: Reviews endpoint design, resource modeling, versioning, backward compatibility (5+ design decisions)

**Format**: Domain expertise + 5+ decision points + Autonomous reasoning = Subagent

**Autonomy test**:
```
If I give a Subagent a spec and ask "Does this meet our quality bar?"
- Can it reason independently about the answer?
- Does it need expertise (not just process)?
- Does it make judgment calls (not just verify checklist)?
→ YES: Create a Subagent
```

### Decision Tree: Skill vs Subagent vs Constitution

```
Does this rule apply to ALL specs?
├─ YES → Constitution (L5 already taught this)
└─ NO ↓

How many decision points?
├─ 2-4 decisions → Skill
│  (Framework guidance for common questions)
│
└─ 5+ decisions → Subagent
   (Domain expertise + autonomous reasoning)


How broadly applicable?
├─ Horizontal (many features) → Skill
│  (Error handling applies to all APIs)
│
└─ Vertical (specific domain) → Subagent
   (Performance optimization requires deep expertise)


Does it require autonomous judgment?
├─ NO → Skill
│  (Guided decisions, framework-based)
│
└─ YES → Subagent
   (Autonomous reasoning, expertise-driven)
```

### Practice: Categorize Patterns from L1-5

From the specifications you've written in Lessons 1-5, identify patterns and categorize them:

**Exercise**:

1. **List 5 patterns** you noticed repeating in multiple specs (e.g., "error handling format," "pagination structure," "input validation approach")

2. **For each pattern, answer**:
   - Does it apply to ALL specs? (Constitution?)
   - How many decision points? (2-4 = Skill, 5+ = Subagent)
   - Is it horizontal or vertical? (Broad or domain-specific?)
   - Does it need autonomous judgment?

3. **Categorize** each pattern:
   - Constitution (global rule)
   - Skill (2-4 decisions, horizontal)
   - Subagent (5+ decisions, vertical, autonomous)

**Example analysis**:

```
Pattern: "Error Response Format"

Decision points:
1. HTTP status code (4xx for client, 5xx for server)
2. JSON structure { code, message, request_id }
3. Should stacktrace be included?
4. How should errors be logged?

Count: 4 decisions → SKILL ✓

Applicability: ALL APIs need this → Horizontal ✓

Judgment: Mostly guided by standards → Not Subagent ✓

Conclusion: This is a SKILL (Error Handling Framework)
```

---

## The Bigger Picture — Intelligence Accumulation

You now understand three levels of Reusable Intelligence:

### Level 1: Constitutional Rules (L5)
- **Apply to**: Every specification
- **Purpose**: Ensure consistency and quality at scale
- **Example**: "All specs must have Intent, Requirements, Constraints"

### Level 2: Skills (L6 - This Lesson)
- **Apply to**: Related features that share patterns
- **Purpose**: Capture recurring decisions and guidance
- **Example**: "Error Handling Skill" (applies to all error responses)

### Level 3: Subagents (L6 - This Lesson)
- **Apply to**: Domain-specific concerns requiring expertise
- **Purpose**: Provide autonomous reasoning for complex judgments
- **Example**: "Security Auditor" (reviews all security-sensitive features)

**What this means**: SDD-RI isn't just writing better specs. It's building organizational intelligence:
- First, you write consistent specs (Constitution)
- Then, you capture recurring patterns (Skills)
- Then, you encode domain expertise (Subagents)
- Finally, you orchestrate intelligence at scale (Chapter 31+)

---

## Reflection: Your SDD-RI Journey So Far

**What you've accomplished**:
- L1-3: Learned to write clear, quality specifications
- L4: Discovered AI as collaborative partner for spec refinement
- L5: Built team governance (Constitutions ensure consistency)
- **L6: Identified patterns and created Reusable Intelligence (Skills and Subagents)**

**What you can do now**:
- Recognize when patterns repeat
- Distinguish between Constitutions (universal), Skills (horizontal), Subagents (vertical)
- Know when to encode intelligence vs write fresh specs

**What's next (L7)**:
- Learn **HOW** to design Skills and Subagents using the **Persona + Questions + Principles (P+Q+P)** pattern
- Design your own Skill and Subagent (documented, not yet implemented)
- Understand why P+Q+P activates reasoning instead of just pattern matching

---

## Try With AI

**Setup**: Open Claude (or your AI assistant) and explore reusable intelligence patterns in your own experience.

**Prompt Set**:

**Prompt 1 (Pattern Recognition)**:
```
I'm thinking about the last 3 projects I worked on.
What patterns appeared in ALL of them?
(Error handling, logging, authentication, validation, pagination, etc.)

For each pattern, tell me:
- What decisions did I face repeatedly?
- How many decisions? (2-4 or 5+?)
- Could I have reused guidance instead of re-deciding?
```

**Prompt 2 (Skill vs Subagent)**:
```
I have a pattern that repeats often: [describe pattern from Prompt 1]

Is this a Skill (2-4 guided decisions) or Subagent (5+ domain expertise decisions)?

Help me count the decision points and explain my reasoning.
```

**Prompt 3 (Design Exploration)**:
```
If I were to document this pattern as reusable intelligence for my team,
what would they need to know?

What framework could I provide that others could reuse without my help?
```

**Expected Outcomes**:
- Recognition of patterns in your own work
- Clear understanding of when patterns warrant Reusable Intelligence
- Language to discuss Skills vs Subagents with your team

**Optional Stretch**:
```
Describe a time when you WISHED you had captured a pattern as reusable intelligence.
What would the team have gained by doing so?
```
