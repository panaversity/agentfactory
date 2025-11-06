---
title: Progressive Context Loading"
chapter: 11
lesson: 4
duration_minutes: 20
sidebar_position: 4
description: "Learn to load context strategically as needed, not all at once, to prevent context rot"
keywords: [progressive loading, context management, just-in-time loading, context strategy, AIDD]

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Applying Progressive Loading Strategy"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement 3-phase loading (Foundation → Current Work → On-Demand) for real projects to prevent context overload"

  - name: "Creating Context Loading Plans"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can design loading sequence for unfamiliar codebase, identifying what to load first, second, and on-demand"

  - name: "Analyzing Context Needs"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can evaluate which files/information are needed immediately vs later based on current task scope"

learning_objectives:
  - objective: "Apply the progressive loading strategy to avoid context overload"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student implements 3-phase loading for sample project with correct sequencing"

  - objective: "Create a 3-phase loading plan for an unfamiliar project"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student produces loading plan document identifying foundation files, work files, and on-demand files"

  - objective: "Analyze which context is needed now vs later"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Given a development task, student categorizes 10-15 files into immediate/later/never-needed groups with justification"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (Progressive Loading, 3-Phase Strategy, Just-In-Time Loading, Context Prioritization, Loading Plans) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Design adaptive loading algorithm that monitors context utilization and auto-loads based on task patterns; research RAG vs progressive loading tradeoffs"
  remedial_for_struggling: "Practice with small projects first (5-10 files); use provided templates for loading plans; focus on foundation vs current work distinction only"
---

# Progressive Context Loading

## The Problem: Loading Everything Upfront

Now that you understand context windows and context rot, let's address the most common beginner mistake.

### A Common Mistake

**New Developer's Approach:**
```bash
# Session start
"Read all files in my project:
- All 50 files in src/
- All 30 test files
- All documentation
- Configuration files
- Everything!"
```

**What happens?**
1. Context window fills to 80%+ immediately
2. By interaction 3-5, context rot starts
3. AI slows down drastically
4. Performance degrades before you've even started building
5. Most of that context isn't even relevant yet!

**The Result:** You've wasted your context budget before doing any actual work.

### Why This Doesn't Work

Think about learning a new subject:

**Bad Learning Approach:**
"Let me read the entire 500-page textbook cover-to-cover before starting the first exercise."

**Result:** Information overload, can't remember most of it, gets confused.

**Good Learning Approach:**
"Let me read the chapter introduction, then the relevant section, then try the exercise."

**Result:** Focused learning, better retention, can reference back as needed.

**Progressive context loading is the "good learning approach" for AI agents.**

---

## What is Progressive Context Loading?

### WHAT: The Strategy

**Progressive Context Loading** is the practice of loading information into your AI's context as you need it, not all at once.

**Core Principle:**
> Load the minimum context necessary for the current task, then expand as needed.

### The Three-Phase Approach

Think of it like exploring a new building:

**Phase 1: Overview** (Understand the Layout)
- Get the "lay of the land"
- High-level structure
- Don't dive into details yet

**Phase 2: Module Focus** (Enter the Relevant Room)
- Load only the specific area you're working in
- Understand patterns and conventions
- Still don't load unrelated sections

**Phase 3: Deep Dive** (Work on Your Specific Task)
- Now you're ready to implement
- Context is focused and relevant
- No wasted space on irrelevant information

### Visual Comparison

**All-at-Once Loading:**
```
Context Window: [████████████████░░░░] 80% full immediately
Available Space: 20%
Time Until Rot: 5-10 interactions
```

**Progressive Loading:**
```
Context Window: [████░░░░░░░░░░░░░░░░] 20% full after Phase 1
Available Space: 80%
Time Until Rot: 25-30+ interactions
```

**The difference:** Progressive loading gives you 3-5x more working room!

---

## WHY: Why Progressive Loading Works

### Three Key Benefits

**1. Keeps Context Focused**
- Only relevant information is loaded
- AI doesn't get "distracted" by irrelevant code
- Faster, more accurate responses

**2. Prevents Early Context Rot**
- Context fills gradually, not immediately
- You get much longer productive sessions
- More work done before needing to refresh

**3. Matches Your Workflow**
- You don't need to know everything at once anyway
- Load context just-in-time as tasks evolve
- Natural progression mirrors how humans work

### Real-World Analogy

**All-at-Once:** Like bringing every tool from your toolbox to a job site. Most tools sit unused, get in your way, and you waste energy carrying them.

**Progressive:** Like bringing only the tools you need for each phase. Hammer and nails for framing, then paintbrush and paint for finishing. Efficient and focused.

---

## HOW: Implementing Progressive Loading

Let's walk through a complete real-world example.

### Scenario: Joining a New Project

You've just joined a Python FastAPI project with 50+ files. You need to add a new feature: "User Profile Management."

### 🚫 The Wrong Way (All-at-Once)

```bash
claude "Read all files in src/, tests/, and docs/ so you 
understand the entire project"
```

**Result:** Context immediately 75% full, most information not relevant to your task.

### ✅ The Right Way (Progressive Loading)

---

#### **Phase 1: High-Level Overview**

**Goal:** Understand project structure without reading file contents.

```bash
claude "Analyze the directory structure of this project without 
reading file contents yet. Tell me:

1. What are the main directories?
2. What appears to be the general architecture?
3. Where would user-related features likely be located?
4. What testing structure is being used?

Just analyze the structure—don't read files yet."
```

**What the AI Does:**
- Lists directories (sees src/api/, src/services/, src/models/, tests/)
- Infers architecture (layered: API → Service → Model)
- Identifies user-related files are probably in src/services/user_service.py
- Notes testing structure (pytest, tests/ mirrors src/)

**Context Used:** ~1,000 tokens (just directory listings and your conversation)

**Why This Works:** You now understand the landscape without filling context with actual code.

---

#### **Phase 2: Module Focus**

**Goal:** Understand the specific area relevant to your task.

```bash
claude "Now I need to understand how user-related features work. 
Read these specific files:

1. src/models/user.py - to see the User model
2. src/services/user_service.py - to understand user service patterns
3. src/api/routes/users.py - to see how user endpoints are structured

Analyze the patterns: How do we structure services? What's the code style?"
```

**What the AI Does:**
- Reads ONLY the 3 files needed
- Understands your service pattern (constructor pattern, async methods, error handling)
- Notes your code style (type hints, docstrings, custom exceptions)
- Sees how API routes connect to services

**Context Used:** ~5,000 tokens (3 files + conversation)

**Total Context So Far:** ~6,000 tokens (about 3% of Claude's 200K window!)

**Why This Works:** AI now knows your patterns without loading 50 files.

---

#### **Phase 3: Task-Specific Implementation**

**Goal:** Build the new feature with full context of relevant patterns.

```bash
claude "Following the exact patterns I saw in user_service.py, 
create a new profile management feature:

1. Add profile fields to User model (bio, avatar_url, social_links)
2. Create ProfileService following the UserService pattern
3. Add API endpoints for profile CRUD operations
4. Follow the same error handling and async patterns

Use ONLY our existing patterns—don't introduce new dependencies."
```

**What the AI Does:**
- Implements ProfileService matching your UserService structure exactly
- Uses your established patterns for models, services, routes
- Follows your code style (type hints, docstrings, error handling)
- Stays within existing dependency set

**Context Used:** ~10,000 tokens (previous context + new conversation)

**Total Context:** ~16,000 tokens (about 8% of context window!)

**Why This Works:** You've built a complete feature using only 8% of available context, leaving 92% for iteration, debugging, and expansion.

---

### The Progression Visualized

```
Phase 1: Overview
├─ Context: 1% full
└─ Knowledge: Project structure, architecture pattern

Phase 2: Module Focus  
├─ Context: 3% full
└─ Knowledge: Specific patterns, code style, conventions

Phase 3: Implementation
├─ Context: 8% full
└─ Knowledge: Everything needed to build feature perfectly

Remaining capacity: 92% for refinement, testing, documentation!
```

---

## Practical Guidelines

### When to Use Each Phase

**Phase 1 - Overview** (Always start here)
- ✓ Starting a new project
- ✓ Joining an existing codebase
- ✓ Haven't worked on project in days/weeks
- ✓ Exploring unfamiliar architecture

**Phase 2 - Module Focus** (After you know the landscape)
- ✓ About to add a feature
- ✓ Need to understand a specific subsystem
- ✓ Fixing a bug in particular module
- ✓ Learning project patterns

**Phase 3 - Implementation** (When you're ready to build)
- ✓ Clear task in mind
- ✓ Understand relevant patterns
- ✓ Ready to write/modify code
- ✓ Have context for current work

### How Much to Load in Each Phase

**Phase 1 Guidelines:**
- Just directory structures
- Key documentation (README.md)
- Architecture overview docs
- **Don't:** Read actual code files yet

**Phase 2 Guidelines:**
- 2-5 example files showing patterns
- Relevant service/model files
- Related configuration
- **Don't:** Load unrelated modules

**Phase 3 Guidelines:**
- Specific files you're modifying
- Direct dependencies
- Related tests
- **Don't:** Load "just in case" files

---

## Practice Exercise: Create Your Loading Plan

Let's practice creating a progressive loading plan.

### Scenario

You're joining a new project: An e-commerce API built with FastAPI and PostgreSQL. 

**Project structure:**
```
project/
├── src/
│   ├── api/routes/
│   │   ├── products.py
│   │   ├── orders.py
│   │   ├── users.py
│   │   └── payments.py
│   ├── services/
│   │   ├── product_service.py
│   │   ├── order_service.py
│   │   ├── user_service.py
│   │   └── payment_service.py
│   ├── models/
│   │   ├── product.py
│   │   ├── order.py
│   │   └── user.py
│   └── database/
│       ├── connection.py
│       └── migrations/
├── tests/
└── docs/
    ├── API.md
    └── ARCHITECTURE.md
```

**Your task:** Add a "shopping cart" feature.

### Your Turn: Plan the Three Phases

Before looking at the answer below, try planning:

1. **Phase 1**: What should you analyze without reading code?
2. **Phase 2**: Which 3-5 files should you read to understand patterns?
3. **Phase 3**: What specific context do you need for implementing cart?

<details>
<summary><strong>Click to reveal suggested answer</strong></summary>

**Phase 1: Overview**
```bash
"Analyze project structure without reading files:
- What's the architecture pattern?
- Where should cart feature live?
- How do existing features connect (API → Service → Model)?"
```

**Phase 2: Module Focus** (Cart is similar to Products and Orders)
```bash
"Read these files to understand patterns:
1. src/models/product.py - see model pattern
2. src/services/product_service.py - see service pattern
3. src/services/order_service.py - see how orders work with products
4. src/api/routes/orders.py - see API route structure
5. docs/ARCHITECTURE.md - understand database approach"
```

**Phase 3: Implementation**
```bash
"Create cart feature following patterns:
1. Cart model (similar to Order)
2. CartService (following ProductService pattern)
3. Cart API routes (matching existing route structure)
Follow all established patterns—don't introduce new ones"
```

</details>

---

## Validation Checkpoints

How do you know if your progressive loading strategy is working?

### ✅ Signs You're Doing It Right

**After Phase 1:**
- [ ] You understand project structure
- [ ] You know where relevant code lives
- [ ] You haven't read any actual implementation yet
- [ ] Context is under 5%

**After Phase 2:**
- [ ] You understand the coding patterns used
- [ ] You know how similar features are implemented
- [ ] You've only loaded 3-7 files
- [ ] Context is under 15%

**After Phase 3:**
- [ ] You've implemented the feature
- [ ] Generated code matches existing patterns
- [ ] Context is under 40%
- [ ] Still have room for iteration and refinement

### ❌ Signs You Need to Adjust

**Red Flags:**
- ❌ Context over 50% after Phase 2
- ❌ AI asks "Which pattern should I follow?" (means too many patterns loaded)
- ❌ Loaded files you haven't referenced yet
- ❌ AI seems confused about which approach to use

**What to do:** Start a new session, reload only what's actually needed.

---

## Try With AI

**Tool:** Claude Code

Now let's practice progressive loading thinking.

### Prompt 1: Understanding the Concept

```bash
claude "I'm starting work on a Python project with 30 files. Should I ask my AI to read all files first, or load them progressively as needed? Why? Explain using the three-phase progressive loading approach."
```

**Expected Outcome:**
- The AI recommends progressive loading
- Explanation of why all-at-once causes problems
- Description of the 3-phase approach

**Check:** Can you explain why progressive loading is better?

---

### Prompt 2: Creating a Loading Plan

```bash
claude "I need to add user authentication to a FastAPI project I just joined. The project has 40+ files. I want to use progressive context loading.

Create a 3-phase loading plan for me:
- Phase 1 (Overview): What should I analyze first?
- Phase 2 (Module Focus): Which files should I read?
- Phase 3 (Implementation): What context do I need for building auth?

Assume the project structure is:
src/api/, src/services/, src/models/, tests/"
```

**Expected Outcome:**
- Phase 1: Structure analysis, no files read yet
- Phase 2: Similar existing features (if any), service patterns, model patterns
- Phase 3: Specific files to modify, security patterns, testing patterns

**Reflection:** Does this plan make sense? Would you follow it?

---

### Prompt 3: Real-World Application

```bash
claude "I loaded 20 files into my AI context at the start of my session. Now I'm only using 3 of them for my current task. The other 17 are just sitting there taking up space.

1. What problem does this cause?
2. What should I have done instead?
3. What should I do now to fix this?"
```

**Expected Outcome:**
- Problem: Wasted context, faster rot, slower AI
- Should have: Used progressive loading, loaded only needed 3
- Fix now: Start fresh session with just the 3 needed files

**Action:** This is a common situation—now you know how to fix it!


