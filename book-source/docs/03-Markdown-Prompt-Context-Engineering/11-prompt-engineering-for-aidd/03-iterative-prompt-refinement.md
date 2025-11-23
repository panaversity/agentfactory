---
title: "Iterative Prompt Refinement"
description: "Master AI collaboration through iterative refinement using bidirectional learning patterns"
sidebar_label: "Iterative Prompt Refinement"
sidebar_position: 3
chapter: 10
lesson: 3
duration_minutes: 45
proficiency: "B1"
concepts: 7

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "AI Collaboration Through Iteration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student demonstrates iterative refinement achieving 30%+ quality improvement through AI collaboration"

learning_objectives:
  - objective: "Demonstrate bidirectional learning through iterative prompt refinement"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Live iteration session showing collaborative improvement patterns"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (iteration loop, bidirectional learning, convergence, progressive refinement) within B1 limit of 7-10 ✓"

differentiation:
  extension_for_advanced: "Study Jake Heller's 60%→97% accuracy improvement methodology; analyze iteration patterns in production AI systems; experiment with multi-round refinement tracking metrics"
  remedial_for_struggling: "Focus on experiencing single iteration cycle: Write prompt → Review AI output → Identify one gap → Refine prompt → Compare improvement"
---

# Iterative Prompt Refinement

In Lessons 1-2, you built prompt foundations through manual analysis. Now you're ready to work **with AI** to refine prompts through iteration—the process Jake Heller used to transform 60% accuracy into 97% production-quality outputs.

But here's what makes this different from "just trying prompts until something works":

**AI collaboration is bidirectional.**

You're not a passive user issuing commands. You're in a partnership where both you and AI contribute knowledge, identify gaps, and converge on solutions through dialogue. By the end of this lesson, you'll experience how iteration produces professional results that neither you nor AI could achieve on the first try.

---

## The Iteration Reality: First Attempts Get You 60%

Let's start with honesty: **Your first prompt will not be perfect.**

Even experienced developers don't nail prompts on the first try. Here's why:

1. **You have domain knowledge AI doesn't** (project constraints, team preferences)
2. **AI has pattern knowledge you don't** (best practices, conventions, standards)
3. **Neither of you has the complete picture** (convergence happens through dialogue)

Jake Heller's team discovered this building CoCounsel:

> "Spend weeks tweaking prompts to get from 60% accuracy to 97%+. Most people quit too early."
>
> — Jake Heller [[20:03]](https://www.youtube.com/watch?v=l0h3nAW13ao&t=1203s)

**Most developers quit after 1-2 tries.** They get 60% results, think "AI doesn't work for this," and give up.

**Professional developers iterate 10-20 times.** They understand that **60% is the starting point**, not the failure point.

---

## The Iteration Loop Pattern

Every successful prompt refinement follows this loop:

```
Initial Prompt → AI Output → Analyze → Refine → Improved Output → Repeat
```

![Circular iterative refinement workflow showing: Initial Prompt (starting point, basic intent), AI Output (first generation, 60% quality), Review & Analyze (identify gaps, missing constraints, wrong assumptions), Refine Prompt (add specificity, examples, constraints), Better AI Output (75% quality), continuing in upward spiral. Each iteration adds layer of precision (structure → examples → reasoning) moving from 60% to 97% quality through collaborative convergence.](/img/part-3/chapter-11/prompt-iteration-refinement-loop.png)

Let's break down each step:

### Anthropic's Progressive Refinement: Adding Layers of Precision

Anthropic teaches iteration as **adding specificity layers progressively**:

**Iteration Layer 1: Start Simple**
```
"Create a Git commit message for authentication changes"
```
→ Gets you 60% (generic message)

**Iteration Layer 2: Add Structure**
```
"Create a Git commit message using Conventional Commits format.
Changes: Added JWT refresh, extended token lifetime, fixed logout race condition."
```
→ Gets you 75% (structured but missing project context)

**Iteration Layer 3: Add Examples**
```
"Create a Git commit message using Conventional Commits format.

Example format:
feat(auth): add JWT refresh endpoint

- Added /auth/refresh for token renewal
- Extended expiration to 24h
- Fixed logout race condition

Changes to document: [same as above]"
```
→ Gets you 85% (AI sees desired structure)

**Iteration Layer 4: Add Reasoning**
```
"[Previous prompt]

Before generating, consider:
- What's the primary change type? (feat/fix/refactor)
- What's the business value of each change?
- How would a teammate understand this without reading code?"
```
→ Gets you 95% (AI thinks through quality criteria)

**Notice the pattern**: Simple → Structured → Examples → Reasoning

**This aligns with Jake Heller's 60% → 97% journey**: Each iteration adds precision.

---

### Step 1: Initial Prompt
Write your best first attempt using Intent → Constraints → Success Criteria structure (from Lesson 2)

### Step 2: AI Output
Submit prompt, receive AI's response

### Step 3: Analyze
Ask yourself:
- What's good about this output?
- What's missing or wrong?
- What assumptions did AI make?
- What patterns did AI suggest that I hadn't considered?

### Step 4: Refine
Update prompt based on analysis:
- Add constraints AI violated
- Clarify ambiguous instructions
- Incorporate AI's good suggestions
- Remove unnecessary constraints that limit AI

### Step 5: Improved Output
Submit refined prompt, get better result

### Repeat
Continue until output meets your success criteria (usually 3-7 iterations for routine tasks, 10-20 for complex ones)

---

## Real Example: Git Commit Message Iteration

Let's walk through a complete iteration session showing how you and AI improve together.

**Your goal**: Generate a Git commit message for changes you just made to your project's authentication system.

---

### Iteration 1: Your Initial Prompt

```
Create a commit message for my authentication changes
```

**AI's Response**:
```
Updated authentication system

Made improvements to the login functionality and fixed some bugs
in the authentication flow.
```

**Analysis**:
- Generic and vague ("improvements", "some bugs")
- No specifics about what changed
- Doesn't follow any particular convention
- Would confuse teammates reading git history

**Quality**: ~40% (too vague to be useful)

---

### Iteration 2: Add Structure

You remember Lesson 2's anatomy: Intent → Constraints → Success Criteria.

**Refined Prompt**:
```
GENERATE a Git commit message for authentication changes

CHANGES MADE:
- Added JWT token refresh endpoint
- Updated token expiration from 1 hour to 24 hours
- Fixed logout race condition

CONSTRAINTS:
- Be specific about each change
- Use imperative mood ("add" not "added")
- Under 50 characters for subject line
```

**AI's Response**:
```
Add JWT refresh and extend token lifetime

- Add /auth/refresh endpoint for JWT token renewal
- Update token expiration from 1h to 24h
- Fix race condition in logout handler
```

**Analysis**:
- ✅ Specific changes listed
- ✅ Imperative mood used
- ✅ Concise subject line
- ❓ **Interesting**: AI used a structured format with subject + bulleted body

**Quality**: ~70% (good structure, but you notice AI used conventions you didn't specify)

---

### Iteration 3: Discover Patterns

You notice AI's structure looks professional. Let's ask:

**Discovery Prompt**:
```
This commit message structure is good. Is this following a particular convention?
What makes this format effective?
```

**AI's Response**:
```
Yes! This follows Conventional Commits format:

<type>(<scope>): <description>

[body with detailed changes]

Common types: feat, fix, docs, refactor, test, chore

This format helps because:
- Type categorizes change (feat = new feature)
- Scope indicates area (auth, api, docs)
- Description summarizes in 50 chars
- Body provides details for complex changes

For your changes, it would be:
feat(auth): add JWT refresh and extend token lifetime
```

**What you discovered**: Conventional Commits is a real standard used in production

**Your decision**: This pattern looks valuable—let's use it for your project

---

### Iteration 4: Apply Discovered Patterns with Your Requirements

Now combine the convention AI suggested with your team's specific needs:

**Integration Prompt**:
```
GENERATE commit message using Conventional Commits format

CHANGES:
- Added JWT refresh endpoint
- Extended token expiration to 24h
- Fixed logout race condition

TEAM REQUIREMENTS:
- Include Jira ticket number from branch name (currently on branch PROJ-1234-jwt-refresh)
- Use scope "auth" for authentication changes
- List "why" for each change, not just "what"
```

**AI's Response**:
```
feat(auth): add JWT refresh and extend token lifetime [PROJ-1234]

- Add /auth/refresh endpoint: enables mobile apps to maintain sessions without re-login
- Extend token expiration to 24h: reduces login friction for daily active users
- Fix logout race condition: prevents tokens remaining valid after logout
```

**Analysis**:
- ✅ Jira ticket included
- ✅ Scope specified
- ✅ Each change explains "why" (not just "what")
- This combines AI's conventional format with your project requirements

**Quality**: ~85% (good, but could be even better)

---

### Iteration 5: Refine Business Value Focus

You review the "why" explanations and refine:

**Refinement Prompt**:
```
Good! One refinement: our "why" explanations should focus on business value, not technical implementation.

Example:
Instead of: "enables mobile apps to maintain sessions"
Write: "improves mobile user experience by eliminating frequent re-logins"
```

**AI's Response**:
```
feat(auth): add JWT refresh and extend token lifetime [PROJ-1234]

- Add /auth/refresh endpoint: improves mobile UX by eliminating frequent re-logins
- Extend token expiration to 24h: reduces authentication friction for daily users
- Fix logout race condition: ensures users are fully logged out (security improvement)
```

**Analysis**:
- ✅ Business value focus ("improves mobile UX", "reduces friction")
- ✅ Security implication called out (logout fix)
- ✅ Combines: your business focus + AI's conventional format + team's ticket requirement

**Quality**: ~95% (production-ready)

---

## What Happened in That Iteration Session?

Look back at the progression:

**Iteration 1 → 2**: You added structure and constraints
- Quality improved from 40% → 70%

**Iteration 2 → 3**: You noticed AI suggested a professional format you didn't know
- You asked AI to explain the pattern
- You discovered Conventional Commits standard

**Iteration 3 → 4**: You integrated AI's pattern with your team's requirements
- AI adapted its conventional format to match your Jira ticket needs
- Quality improved from 70% → 85%

**Iteration 4 → 5**: You refined business value wording
- AI adjusted language based on your feedback
- Convergence produced optimal result combining both inputs
- Quality improved from 85% → 95%

**What emerged**: A commit message that's both conventional AND project-specific—something neither you nor AI had initially.

---

## Reflection: Patterns of Collaborative Improvement

Three patterns emerged in that iteration sequence:

### Pattern 1: Discovering Knowledge You Didn't Have

**What happened**: AI suggested Conventional Commits format without you asking for it

**Your response**: Instead of forcing AI back to your original vague approach, you asked AI to explain

**Result**: You learned a production standard that makes your commits better

**Key insight**: When AI suggests patterns you didn't specify, investigate instead of rejecting

---

### Pattern 2: Providing Context AI Doesn't Have

**What happened**: AI produced generic format missing your team's Jira ticket requirement

**Your response**: You taught AI your project-specific constraints

**Result**: AI adapted its conventional format to match your requirements

**Key insight**: AI has pattern knowledge; you have domain knowledge. Share your context.

---

### Pattern 3: Converging Through Iteration

**What happened**: Neither you nor AI had the perfect message initially

**Your response**: You refined business value focus based on AI's structure; AI refined wording based on your feedback

**Result**: Convergence produced optimal result combining both inputs

**Key insight**: The best solutions emerge through dialogue, not one-shot prompting

---

## Why This Matters for AI-Driven Development

**Traditional development**: You implement everything yourself based on your knowledge alone.

**AI-Native development**: You and AI collaborate:
- AI suggests patterns and best practices from millions of code examples
- You provide project requirements, team conventions, and business context
- Through iteration, you converge on solutions better than either could produce alone

**This is the future of software development**: Not "human OR AI" but "human AND AI" collaborating through iterative refinement.

---

## When to Stop Iterating: Convergence Criteria

How do you know when you're done? Use these signals:

### 1. Success Criteria Met
If you defined success criteria in your prompt (Lesson 2), check if output meets them:
- ✅ Meets format requirements?
- ✅ Passes validation tests?
- ✅ Teammate could understand without asking questions?

**If all yes → Done**

### 2. Diminishing Returns
Each iteration improves output less than previous iteration:
- Iteration 1 → 2: 40% → 70% (30% improvement)
- Iteration 2 → 3: 70% → 85% (15% improvement)
- Iteration 3 → 4: 85% → 88% (3% improvement)

**When improvement drops below 5% per iteration → Done** (or reassess approach)

### 3. Time Budget Exceeded
Jake Heller spent weeks iterating. You probably don't have weeks for a commit message.

**Set iteration budgets**:
- Simple tasks (commit message, quick doc): 3-5 iterations, ~10 minutes
- Medium tasks (script creation, refactoring): 5-10 iterations, ~30 minutes
- Complex tasks (architecture design, spec writing): 10-20 iterations, hours to days

**When budget exhausted → Use best result so far or escalate complexity**

---

## Common Iteration Mistakes

### Mistake 1: Giving Up Too Early

**Symptom**: First or second prompt doesn't work perfectly, so you conclude "AI can't do this."

**Reality**: First prompts get you 60%. Professional results require iteration.

**Fix**: Commit to at least 5 iterations before judging AI's capability.

---

### Mistake 2: Ignoring Patterns AI Suggests

**Symptom**: AI returns output with improvements you didn't ask for, but you ignore them and force AI back to your original (inferior) approach.

**Example**:
- You: "List commit changes with dashes"
- AI: Returns bulleted list with clear categorization and semantic grouping
- You: "No, just simple dashes like I said"
- Result: You get exactly what you asked for (inferior to what AI suggested)

**Fix**: When AI suggests a better pattern, investigate it! Ask "Why did you use this format?"

---

### Mistake 3: Not Providing Your Context

**Symptom**: AI keeps producing generic output that doesn't fit your project, and you keep regenerating without explaining your requirements.

**Example**:
- AI: Generic Python code
- You: "Try again"
- AI: Different generic Python code
- You: "Try again"
- AI: Still generic because it doesn't know your constraints

**Fix**: Explain your specific requirements: "Our project uses Bash, not Python. Our scripts follow Google Shell Style Guide. We log to /var/log/."

---

### Mistake 4: Changing Too Many Things at Once

**Symptom**: Output is wrong, so you rewrite entire prompt with 10 new constraints. New output is different but still wrong in different ways.

**Problem**: You can't tell which changes improved things and which made them worse.

**Fix**: Iterate **one or two constraints at a time**. This makes it clear what each change contributes.

---

## Iteration Strategies for Different Scenarios

### Strategy 1: Add Constraints Incrementally (OpenAI's Approach)

**Use when**: Output is vague or generic

**OpenAI recommends**: Start with core intent, then add constraints one or two at a time.

**Why incremental?** You see which constraint fixes which problem (vs changing 10 things and not knowing what helped).

**Approach**:
1. Start with basic Intent + Success Criteria
2. Iteration 1: Add technical constraints
3. Iteration 2: Add format constraints
4. Iteration 3: Add quality constraints

**Example** (from OpenAI prompting guide, adapted):
```
Iteration 1: "Create backup script"
→ Too vague

Iteration 2: + "Must be Bash, backup .md files only"
→ Better scope

Iteration 3: + "Output to timestamped folders (YYYY-MM-DD_HH-MM-SS)"
→ Format specified

Iteration 4: + "Include error handling: log permission errors, don't crash"
→ Quality/robustness added
```

**Result after 4 iterations**: Production-ready prompt that generates working code.

**Compare to "change everything at once"**:
```
Iteration 1: "Create backup script"
Iteration 2: "Must be Bash, backup .md files, timestamped folders, error handling, logging, preserve structure, skip hidden files, handle spaces in filenames"
→ If output is wrong, which constraint caused problems? Unknown.
```

**Incremental constraints = debuggable prompt refinement.**

---

### Strategy 2: Discover Then Apply

**Use when**: You're unfamiliar with the domain or best practices

**Approach**:
1. Ask AI to suggest approach/structure
2. Investigate AI's suggestion (ask for explanation)
3. Apply discovered pattern with your constraints
4. Refine based on project specifics

**Example**:
```
Iteration 1: "How should I structure commit messages?" (discover)
Iteration 2: "Use Conventional Commits format for my auth changes" (apply)
Iteration 3: "Add our Jira ticket format requirement" (refine)
```

---

### Strategy 3: Converge Through Dialogue

**Use when**: Requirements are complex or evolving

**Approach**:
1. Start with rough requirement
2. Let AI ask clarifying questions
3. Answer questions (provide your context)
4. Iterate based on AI's refined understanding

**Example**:
```
You: "Help me write documentation for this script"
AI: "What audience? What format? What depth?"
You: "Developers who will modify it. Markdown. Cover usage and architecture."
AI: [Generates targeted documentation]
You: "Good, but add troubleshooting section"
AI: [Refines with troubleshooting]
```

---

## Try With AI: Commit Message Refinement

Now it's your turn to practice iterative refinement with AI.

**Setup**: You've made changes to a Bash script that backs up files. You need a commit message.

### Part 1: Initial Request

Open Claude Code or Gemini CLI and prompt:

```
Create a Git commit message for my changes to backup.sh
```

**Observe**: What does AI return? Is it specific enough? Does it follow any particular convention?

---

### Part 2: Critical Evaluation

Don't immediately accept AI's output. Analyze it:

**Ask yourself**:
- Does this explain WHAT changed?
- Does this explain WHY changes were made?
- Would a teammate understand this without reading the code diff?
- Did AI use any format or structure you didn't specify?

**If AI used an unfamiliar convention**: Ask AI to explain it!

---

### Part 3: Provide Your Project Context

Now add your specific requirements:

```
Good start, but we use this commit format:
- First line: [ticket-id] type: brief description
- Body: bullet list of changes with business value explanation

Our standards:
- Include Jira ticket from branch name
- Types: feat, fix, docs, refactor
- Explain "why" for each change (business value, not just technical details)

Try again with these requirements.
```

**Observe**: Does AI adapt to your constraints? Does output improve?

---

### Part 4: Iterative Refinement

Continue refining. Add one or two new requirements per iteration:

**Possible refinements**:
- "Subject line must be under 50 characters"
- "Include link to relevant documentation if architectural change"
- "Mention breaking changes in footer if any"
- "Use past tense for type, imperative for description"

**Iterate 3-5 times**, each time making output more aligned with your actual project needs.

---

### Part 5: Reflection

After reaching a satisfactory commit message, reflect:

**What patterns did AI suggest that you hadn't considered?**
- New format or convention?
- Better wording or structure?
- Organizational patterns you hadn't thought of?

**What context did you provide that improved AI's output?**
- Project-specific requirements?
- Team conventions?
- Business value focus?

**What emerged from iteration that neither of you had initially?**
- Combination of AI's conventional format + your project constraints?
- Refinements discovered through back-and-forth dialogue?
- Solutions that incorporated both your inputs?

**How did output quality change from iteration 1 to final iteration?**
- Estimate percentage improvement (e.g., 40% → 95%)
- Track how many iterations it took to reach "good enough"

---

## What You've Learned

You've experienced AI collaboration as a **bidirectional partnership**, not a one-way tool:

1. **Iteration loop pattern** (Prompt → Output → Analyze → Refine → Repeat)
2. **Discovering patterns** AI suggests that you didn't know about
3. **Providing context** that AI doesn't have about your project
4. **Converging through dialogue** to produce better results than either alone
5. **Convergence criteria** (When to stop: success met, diminishing returns, time budget)
6. **Common mistakes** (Quitting early, ignoring AI's suggestions, not providing context, changing too much at once)

In Lessons 1-2, you built prompt foundations manually. In this lesson, you started **collaborating with AI** to refine prompts through iteration.

Next lesson, you'll learn **specification-first prompting**—how to define "what good looks like" BEFORE writing prompts, using Jake Heller's framework for achieving 97% accuracy.

**Remember**: Your first prompt gets you to 60%. Professional developers embrace iteration to reach 97%. Don't quit early—that's when the magic happens.
