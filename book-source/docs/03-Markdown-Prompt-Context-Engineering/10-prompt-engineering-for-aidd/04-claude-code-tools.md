# Lesson 4: Claude Code Tool Ecosystem for Documentation Exploration

---

## What Should This Lesson Enable You to Do?

By the end of this lesson, you should be able to:

- **Select the right Claude Code tool** (Read, WebFetch, Grep, Glob, Bash) for specific documentation exploration tasks
- **Orchestrate multiple tools** to navigate complex documentation efficiently
- **Understand tool capabilities and limitations** to avoid common pitfalls
- **Apply the Three Roles framework** while using Claude Code tools (AI teaches you patterns, you teach AI constraints, together you converge on solutions)
- **Create CLAUDE.md files** that encode project-specific context for AI collaboration

**Success criteria**: You can efficiently explore technical documentation (FastAPI, Django, Flask) using appropriate Claude Code tools, and you can teach someone else the tool selection framework you learned.

---

## The Documentation Exploration Challenge

### The Scenario

You are a **Solutions Architect** evaluating whether your company should adopt FastAPI for a new microservices project. Your team currently uses Flask, and leadership wants evidence-based comparison before migrating 15 services.

You have **2 hours** to produce an architecture overview covering:
- Core design decisions (what makes FastAPI different from Flask)
- Type system integration (your team values static typing)
- Async capabilities (your services are I/O-bound)
- Production deployment patterns (you use Kubernetes)

**The problem**: FastAPI documentation has 100+ pages. You can't read everything linearly. You need **strategic navigation**.

**Without the right tools**, you would:
- Read documentation top-to-bottom (inefficient)
- Search generic keywords ("FastAPI tutorial") missing architectural insights
- Miss critical information scattered across multiple pages

**With Claude Code tools**, you will:
- Use Read to scan specific documentation pages
- Use WebFetch to extract content from official docs
- Use Grep to find patterns across documentation
- Use Glob to discover structure
- Orchestrate these tools systematically

**This lesson teaches you which tool to use when.**

---

## Concept 1: The Claude Code Tool Ecosystem

### Understanding Tool Roles

Before you can select the right tool, you need to understand what each tool does and when it's appropriate.

**Think of Claude Code tools like a research team**:
- **Read**: Your detail-oriented analyst (reads specific files deeply)
- **WebFetch**: Your web researcher (fetches content from URLs)
- **Grep**: Your pattern detective (finds specific text across many files)
- **Glob**: Your scout (discovers what files exist)
- **Bash**: Your command specialist (runs shell commands for specific tasks)

Each tool has a specific strength. The key skill is **matching tool to task**.

### Tool 1: Read — Deep Analysis of Known Files

**When to use Read**:
- You know WHICH file you want to analyze (specific path)
- You need detailed understanding of that file's contents
- You want to extract specific sections, code examples, or concepts

**Example scenario**:
```
Task: Understand FastAPI's dependency injection system
Right tool: Read
Reasoning: You know the concept lives in dependency injection docs,
           you need deep understanding, not broad scanning
```

**What Read does well**:
- Reads entire file contents with line numbers
- Allows offset/limit for large files (pagination)
- Preserves formatting (code blocks, headings, structure)

**What Read does NOT do**:
- Search across multiple files (use Grep instead)
- Fetch web content (use WebFetch instead)
- Discover what files exist (use Glob instead)

---

### Tool 2: WebFetch — Retrieving Documentation from URLs

**When to use WebFetch**:
- You have a URL to official documentation
- You need to extract content from a webpage
- The documentation isn't available as local files

**Example scenario**:
```
Task: Extract FastAPI deployment best practices from official docs
Right tool: WebFetch
Reasoning: Documentation lives at fastapi.tiangolo.com,
           you need specific page content converted to readable format
```

**What WebFetch does well**:
- Fetches HTML and converts to markdown
- Handles documentation sites, blogs, technical articles
- Provides clean text output for analysis

**What WebFetch does NOT do**:
- Search local files (use Read or Grep)
- Handle sites requiring authentication
- Fetch non-text content (images, videos)

**Critical insight**: WebFetch is your gateway to external knowledge. When documentation lives on the web (most framework docs), WebFetch + specific URL beats generic web search.

---

### Tool 3: Grep — Pattern Matching Across Files

**When to use Grep**:
- You're looking for specific patterns across MANY files
- You don't know which file contains what you need
- You want to see all occurrences of a concept or code pattern

**Example scenario**:
```
Task: Find all mentions of "async" in FastAPI documentation
Right tool: Grep
Reasoning: Async appears across multiple docs (tutorials, advanced, API),
           you need comprehensive view, not deep dive in one file
```

**What Grep does well**:
- Searches with regex patterns (powerful text matching)
- Filters by file type (*.md, *.py)
- Shows context lines (before/after matches)
- Returns files containing matches or full content

**What Grep does NOT do**:
- Read single files (use Read for deep analysis)
- Handle web URLs (use WebFetch)
- Understand semantic meaning (it's pattern matching, not comprehension)

**Pro tip**: Grep is your "show me everywhere this appears" tool. When you don't know where information lives, Grep finds it.

---

### Tool 4: Glob — Discovering File Structure

**When to use Glob**:
- You need to understand directory structure
- You want to find files matching a pattern
- You're exploring unfamiliar codebase/documentation

**Example scenario**:
```
Task: Discover all markdown files in FastAPI repo
Right tool: Glob
Reasoning: You need structure overview before detailed reading,
           Glob shows "what files exist" before you decide what to read
```

**What Glob does well**:
- Pattern matching for file discovery (*.md, **/*.py)
- Fast directory traversal
- Returns sorted file paths

**What Glob does NOT do**:
- Read file contents (use Read after Glob)
- Search within files (use Grep)

**Strategic use**: Start with Glob to map territory, then use Read/Grep to explore specific areas.

---

### Tool 5: Bash — Command Execution for Specific Tasks

**When to use Bash**:
- You need to run shell commands (git, npm, ls, find)
- You need to combine multiple operations
- You need system information (OS, environment)

**Example scenario**:
```
Task: Check if FastAPI repository has recent commits
Right tool: Bash
Reasoning: Need git command to check commit history,
           not a file reading task
```

**What Bash does well**:
- Executes any shell command
- Combines operations with pipes/chains
- Accesses system utilities

**What Bash does NOT do**:
- Provide optimized file reading (use Read instead)
- Search efficiently across files (use Grep instead)

**Caution**: Bash is powerful but less specialized. Use Read/Grep/Glob for file operations when possible (they're optimized for those tasks).

---

## Concept 2: Tool Selection Decision Framework

### The Framework

When you encounter a documentation exploration task, ask these questions:

**Question 1: Do I know which specific file I need?**
- **YES → Use Read** (deep analysis of known file)
- **NO → Ask Question 2**

**Question 2: Do I need to search across multiple files?**
- **YES → Use Grep** (pattern matching across files)
- **NO → Ask Question 3**

**Question 3: Do I need content from a web URL?**
- **YES → Use WebFetch** (fetch and extract web content)
- **NO → Ask Question 4**

**Question 4: Do I need to discover what files exist?**
- **YES → Use Glob** (file structure discovery)
- **NO → Use Bash** (specialized command execution)

### Decision Tree Visualized

```
Start: "What do I need to do?"
         ↓
    Know specific file?
    ├─ YES → Read (deep analysis)
    └─ NO → Search many files?
        ├─ YES → Grep (pattern matching)
        └─ NO → Web URL?
            ├─ YES → WebFetch (fetch web content)
            └─ NO → Discover structure?
                ├─ YES → Glob (file discovery)
                └─ NO → Bash (command execution)
```

---

## Concept 3: Three Roles Demonstration — Exploring FastAPI Documentation

### Setup

You're using Claude Code to explore FastAPI documentation. You've cloned the FastAPI repository locally and have access to the docs/ directory. Your goal: understand FastAPI's dependency injection system conceptually (not just how to use it, but WHY it's designed that way).

Watch how AI plays three roles as you work together.

---

### Role 1: AI as Teacher (Suggests Tool Orchestration Pattern)

**You start with generic request**:
```
I need to understand FastAPI's dependency injection system.
Can you help me explore the documentation?
```

**AI responds** (teaching you a pattern):
```
To understand dependency injection conceptually, we should use a 3-tool approach:

1. **Glob** first to discover where dependency injection docs live
   Pattern: `docs/**/*depend*.md`

2. **Read** those specific files for deep understanding
   Focus: Design rationale, not just API usage

3. **Grep** for examples across the entire docs/
   Pattern: `Depends\(` to see usage patterns

This orchestration (Glob → Read → Grep) is a common pattern for
"understand concept deeply" tasks. You discover structure, read
conceptual docs, then see real usage.

Let me start with Glob to map the territory.
```

**What you learned**: You didn't know about the Glob → Read → Grep orchestration pattern. The AI taught you a systematic approach for documentation exploration. This is **AI as Teacher**.

**Evidence**: The AI suggested a 3-step tool chain you weren't using. This is reasoning about tool selection, not just executing commands.

---

### Role 2: AI as Student (Adapts to Your Constraints)

**You provide domain feedback**:
```
Good approach, but I already know HOW to use Depends() from tutorials.
What I need is understanding WHY FastAPI chose dependency injection
over Flask's decorators. Can you focus on design rationale, not usage examples?
```

**AI responds** (adapted to your correction):
```
Ah, thank you for that clarification. That changes my search strategy.

Instead of looking for usage examples (Depends() calls), I should:

1. **WebFetch** the "Advanced User Guide" sections where design
   decisions are explained (often in introduction/philosophy pages)

2. **Grep** for comparison keywords: "Flask", "decorator", "rationale"
   to find where they explicitly discuss design choices

3. **Read** those specific sections for conceptual understanding

Your constraint (WHY, not HOW) means I should prioritize:
- Architecture decision sections (not tutorials)
- Comparison discussions (not API references)
- Philosophy/design pages (not usage examples)

Let me search for "design" and "philosophy" in the docs first.
```

**What AI learned**: You taught the AI your learning goal (conceptual understanding, not usage patterns). The AI adapted its tool selection and search strategy based on your feedback. This is **AI as Student**.

**Evidence**: The AI initially focused on usage examples (Depends() calls). You corrected it. The AI changed approach to prioritize design rationale. Bidirectional learning.

---

### Role 3: AI as Co-Worker (Convergence Through Iteration)

**You continue exploring together**:
```
I found a section mentioning "Dependency Injection vs Global State".
But it's comparing to global variables, not Flask decorators specifically.
Can we look for discussions about testability? That's often why
dependency injection is chosen.
```

**AI responds** (convergence loop):
```
Excellent insight. Testability is a key architectural driver.

Let me **Grep** for "test" + "dependency" together:
Pattern: `test.*dependency|dependency.*test`

I found three sections:
1. Tutorial on testing with dependency overrides
2. Advanced guide on dependency injection for testing
3. Design doc comparing testability in FastAPI vs other frameworks

Section 3 is what you're looking for. Let me **Read** that file.

[After reading]

Now I understand the design rationale:

FastAPI chose dependency injection specifically for:
1. **Testability**: You can override dependencies for tests
   (Flask decorators make this harder)
2. **Explicit dependencies**: Function signatures show what they need
   (vs Flask's implicit `current_app` global)
3. **Type safety**: Dependencies are type-checked
   (vs Flask's runtime-only validation)

So the answer to your original question (WHY dependency injection
over Flask decorators) is: **testability + explicitness + type safety**.

I initially searched for "design" broadly. You focused me on
"testability" specifically. Together, we converged on the architectural
rationale that wasn't obvious from either approach alone.
```

**What happened**: Neither you nor the AI had the complete answer initially:
- You knew to look for testability (domain knowledge)
- The AI knew how to search docs systematically (tool expertise)
- Together, you found the design rationale (testability + explicitness + type safety)

**This is AI as Co-Worker**: Convergence through iteration toward a better understanding.

**Evidence**: The final insight emerged from collaboration. Your domain knowledge + AI's search capability produced a result neither had alone.

---

## Concept 4: Tool Orchestration Patterns

### Common Patterns You'll Use

**Pattern 1: Structure Discovery → Deep Dive**
```
Use case: Exploring unfamiliar documentation
1. Glob: Discover what files exist (structure)
2. Read: Deep dive into specific files (content)
Example: New framework exploration
```

**Pattern 2: Broad Search → Focused Analysis**
```
Use case: Finding specific information across many files
1. Grep: Search for pattern across all files (broad)
2. Read: Analyze files that match (focused)
Example: Finding all async usage in docs
```

**Pattern 3: Web Research → Local Verification**
```
Use case: Validating web documentation against source code
1. WebFetch: Get official docs from website
2. Grep: Verify examples exist in actual code
Example: Checking if tutorial examples are up-to-date
```

**Pattern 4: Conceptual Understanding → Usage Examples**
```
Use case: Learning both WHY and HOW
1. Read: Conceptual/philosophy sections
2. Grep: Usage examples across codebase
Example: Understanding dependency injection (this lesson's demo)
```

---

## Concept 5: Creating CLAUDE.md for Persistent Context

### Why CLAUDE.md Matters

Every time you work with Claude Code, you could provide 4-layer context manually. But for recurring projects, that's inefficient.

**Solution**: Create a **CLAUDE.md** file in your project root that encodes persistent context.

### What Goes in CLAUDE.md

**Section 1: Project Context**
```markdown
# Project: FastAPI Evaluation

## Decision Context
- Evaluating FastAPI vs Flask for microservices migration
- 15 services currently in Flask, considering migration
- Timeline: 2-week proof of concept

## Success Criteria
- Architecture comparison (design decisions, not features)
- Migration effort estimate
- Risk assessment (breaking changes, learning curve)
```

**Section 2: Code Context**
```markdown
## Codebase Structure
- Repository: https://github.com/tiangolo/fastapi
- Focus areas: docs/tutorial, docs/advanced, fastapi/dependencies.py
- Ignore: tests/, scripts/, .github/
```

**Section 3: Constraints Context**
```markdown
## Team Constraints
- Python 3.11+, PostgreSQL, Redis, Kubernetes
- Team: 5 engineers, 2 years Flask experience, 0 years FastAPI
- Must maintain API compatibility during migration
```

**Section 4: Analyst Context**
```markdown
## My Background
- Solutions Architect, 8 years Python, 5 years microservices
- Strong typing advocate (prefer MyPy-validated code)
- Need evidence-based decision support, not opinions
```

### How AI Uses CLAUDE.md

When you open Claude Code in a project with CLAUDE.md:
1. AI reads CLAUDE.md automatically (persistent context)
2. AI applies that context to every query (no need to repeat)
3. AI focuses on your decision criteria (not generic analysis)

**Example**:
```
Without CLAUDE.md:
You: "Analyze dependency injection"
AI: [Generic explanation of dependency injection]

With CLAUDE.md:
You: "Analyze dependency injection"
AI: [Comparison to Flask decorators, migration effort estimate,
     testability implications for your 15-service architecture]
```

---

## Self-Assessment: Tool Selection Practice

### Exercise 1: Choose the Right Tool

For each task below, identify which Claude Code tool to use and why.

**Task A**: Find all mentions of "security" in FastAPI documentation
- **Tool**: ___________
- **Reasoning**: ___________

**Task B**: Read the specific file `docs/tutorial/security/oauth2.md` in detail
- **Tool**: ___________
- **Reasoning**: ___________

**Task C**: Discover all Python files in the `fastapi/security/` directory
- **Tool**: ___________
- **Reasoning**: ___________

**Task D**: Fetch content from `https://fastapi.tiangolo.com/deployment/docker/`
- **Tool**: ___________
- **Reasoning**: ___________

**Task E**: Check git commit history for recent security updates
- **Tool**: ___________
- **Reasoning**: ___________

---

### Exercise 2: Design a Tool Orchestration

**Scenario**: You need to understand how FastAPI handles database connections compared to Flask-SQLAlchemy.

**Design a 3-step tool orchestration**:
1. Tool: ___________, Goal: ___________
2. Tool: ___________, Goal: ___________
3. Tool: ___________, Goal: ___________

**Reasoning**: Why this sequence? What does each step accomplish?

---

### Answer Key (Self-Check)

**Task A**: **Grep** — Searching across multiple files for pattern
**Task B**: **Read** — Deep analysis of known specific file
**Task C**: **Glob** — Discovering files matching pattern
**Task D**: **WebFetch** — Fetching web content from URL
**Task E**: **Bash** — Executing git command

**Exercise 2 Suggested Answer**:
1. **Grep**: Search for "database" across docs to find relevant sections
2. **Read**: Deep dive into database connection docs for FastAPI patterns
3. **WebFetch**: Fetch Flask-SQLAlchemy docs for comparison context

---

## Try With AI

**Setup**: Open Claude Code in any terminal. If you don't have a FastAPI repository locally, you can practice with ANY documentation repository (Django, Flask, React, etc.).

**Exercise**: Apply the tool selection framework and Three Roles pattern.

---

### Practice Task: Framework Documentation Exploration

**Goal**: Understand a framework's architecture using systematic tool orchestration.

**Prompts to Try**:

**Prompt 1: Structure Discovery**
```
I need to explore [Framework Name] documentation to understand its architecture.
Can you:
1. Use Glob to discover documentation structure
2. Identify which files likely contain architectural/design decisions
3. Suggest a reading sequence based on structure
```

**Expected outcome**: AI should use Glob first, then suggest specific files to Read based on filenames that indicate architecture docs (philosophy.md, architecture.md, design.md).

---

**Prompt 2: Conceptual Deep Dive**
```
I found these 3 files contain architecture information:
- [file1]
- [file2]
- [file3]

I specifically need to understand [concept, e.g., "request lifecycle"].
Can you Read those files and extract design rationale for this concept?
```

**Expected outcome**: AI should Read files sequentially and synthesize conceptual understanding (not just quote documentation).

---

**Prompt 3: Pattern Validation**
```
You explained [concept] conceptually. Now I want to see how it's used in practice.
Can you Grep for examples of [concept] across the documentation/examples?
```

**Expected outcome**: AI should use Grep to find usage patterns and show concrete examples.

---

### Challenge: Three Roles Validation

**As you work with AI, identify these moments**:

1. **AI as Teacher**: When did AI suggest a tool or pattern you didn't know?
   - What did you learn? _________

2. **AI as Student**: When did you correct or refine AI's approach?
   - What did AI learn from you? _________

3. **AI as Co-Worker**: When did you converge on a solution neither had initially?
   - What emerged from collaboration? _________

---

### Safety Note

**Validate tool outputs**: Always verify that:
- **Read** returns actual file contents (check line numbers match)
- **Grep** matches are accurate (spot-check a few results)
- **WebFetch** content matches the source URL (cross-reference key claims)
- **Glob** results are complete (compare to your own `ls` or `find` if uncertain)

AI tools are powerful, but you remain responsible for validating their outputs against ground truth.

---

**Lesson Metadata**:
- **Stage**: 2 (AI Collaboration)
- **Modality**: Hands-on exploration
- **Concepts**: 5 (Read, WebFetch, Grep, Glob/Bash, CLAUDE.md, tool selection framework)
- **Cognitive Load**: B1 tier (5 ≤ 10)
- **AI Tools**: Claude Code (Read, WebFetch, Grep, Glob, Bash)
- **Duration**: 80 minutes
- **Three Roles**: Demonstrated with explicit callouts (AI as Teacher, Student, Co-Worker)
- **Version**: 1.0.0
- **Constitution**: v6.0.0 Compliance
- **Generated**: 2025-01-18
- **Source Spec**: `specs/025-chapter-10-redesign/spec.md`
