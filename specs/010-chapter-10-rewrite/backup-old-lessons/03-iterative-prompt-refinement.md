---
title: "Iterative Prompt Refinement"
chapter: 10
lesson: 3
part: 3
duration_minutes: 45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "AI Collaboration Through Iteration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student demonstrates iterative refinement achieving 30%+ quality improvement through AI collaboration"

learning_objectives:
  - objective: "Demonstrate bidirectional learning with AI (AI teaches patterns, student teaches constraints, both converge on solution)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Live iteration session showing all three collaboration roles"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (iteration loop, AI as Teacher/Student/Co-Worker, convergence, bidirectional learning) within B1 limit of 7-10 ✓"
---

# Iterative Prompt Refinement

In Lessons 1-2, you built prompt foundations through manual analysis. Now you're ready to work **with AI** to refine prompts through iteration—the process Jake Heller used to transform 60% accuracy into 97% production-quality outputs.

But here's what makes this different from "just trying prompts until something works":

**AI collaboration is bidirectional.**

You're not a passive user issuing commands. You're in a partnership where:
- AI teaches you patterns and approaches you didn't know
- You teach AI your specific requirements and constraints
- Together, through iteration, you converge on solutions neither of you had initially

This is the shift from "using AI tools" to "co-creating with AI partners." By the end of this lesson, you'll experience all three roles AI plays in your development workflow, and you'll understand why iteration—not one-shot prompting—produces professional results.

---

## The Iteration Reality: First Attempts Get You 60%

Let's start with honesty: **Your first prompt will not be perfect.**

Even experienced developers don't nail prompts on the first try. Here's why:

1. **You don't know what AI knows** (patterns, best practices AI can suggest)
2. **AI doesn't know your context** (project constraints, team preferences)
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

Let's break down each step:

### Step 1: Initial Prompt
Write your best first attempt using Intent → Constraints → Success Criteria structure (from Lesson 2)

### Step 2: AI Output
Submit prompt, receive AI's response

### Step 3: Analyze
Ask yourself:
- What's good about this output?
- What's missing or wrong?
- What assumptions did AI make?
- What did AI suggest that I hadn't considered?

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

## Three Roles: How AI Collaborates With You

Here's the key insight most developers miss:

**AI plays three different roles in the iteration loop**, and understanding these roles transforms how you collaborate.

### Role 1: AI as Teacher (AI → You)

**When this happens**: AI suggests patterns, conventions, or approaches you didn't know

**What it looks like**:
- You prompt: "Create a Git commit message for authentication changes"
- AI returns: Message using Conventional Commits format (`feat(auth): add JWT refresh`)
- You think: "I didn't know about Conventional Commits—this is better than my format!"

**What you learn**: Patterns and best practices from AI's training on millions of code examples

**Your response**: Adopt AI's suggestion, incorporate pattern into future prompts

---

### Role 2: AI as Student (You → AI)

**When this happens**: AI's output is generic or wrong, and you teach AI your specific requirements

**What it looks like**:
- AI returns: Generic commit message missing your project context
- You respond: "Our team uses this format: `[ticket-id] type: description`. Include Jira ticket from branch name."
- AI learns: Your team's specific commit convention
- AI returns: Commit message matching your actual requirements

**What you teach**: Project-specific constraints, team conventions, domain context

**AI's response**: Adapted output incorporating your corrections

---

### Role 3: AI as Co-Worker (You ↔ AI)

**When this happens**: Neither you nor AI has the complete solution initially, but iteration produces something better than either of you started with

**What it looks like**:
- You start with basic requirement: "Commit message for auth changes"
- AI suggests: Conventional Commits structure (teaches you)
- You add: Project-specific ticket format (teach AI)
- AI refines: Combines both patterns + suggests including breaking change notes
- You evaluate: Good idea, but adds "BREAKING CHANGE" footer per convention
- AI iterates: Perfect commit message using best of both contributions

**What emerges**: Solution combining your domain knowledge with AI's pattern knowledge

**The result**: Neither of you could have created this alone on first try

---

## Real Example: Git Commit Message Iteration

Let's walk through a complete iteration session showing all three roles.

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
- ❌ **BUT**: AI used a conventional format you didn't know about!

**What AI just taught you**: There's a standard pattern (subject + body with bullet points)

**Quality**: ~70% (good, but you notice AI knows conventions you don't)

---

### Iteration 3: Learn from AI (AI as Teacher)

You notice AI's structure is better than what you asked for. Let's ask AI to explain:

**Learning Prompt**:
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

**What you learned**: Conventional Commits is a real standard used in production

**Your decision**: Adopt this for your project

---

### Iteration 4: Teach AI Your Project (AI as Student)

Your team has specific requirements AI doesn't know about:

**Teaching Prompt**:
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
- This is better than iteration 2 because it combines AI's convention knowledge with your project requirements

**What you taught AI**: Your team's specific commit format

**Quality**: ~85% (good, but could be even better)

---

### Iteration 5: Co-Worker Convergence

You and AI both contribute to making this even better:

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

**What emerged from collaboration**:
- You learned: Conventional Commits standard (from AI)
- AI learned: Your team's ticket + business value requirements (from you)
- Together: Combined both into message that's conventional AND project-specific

---

## Reflection: The Three Roles in Action

Look back at that iteration sequence:

**Iteration 2 → 3**: **AI as Teacher**
- AI showed you Conventional Commits format
- You learned a pattern you didn't know existed
- You asked AI to explain the convention
- AI taught you the structure and rationale

**Iteration 3 → 4**: **AI as Student**
- You taught AI your team's Jira ticket requirement
- You taught AI your "why" explanation preference
- AI adapted its output to match your constraints
- AI learned project-specific requirements

**Iteration 4 → 5**: **AI as Co-Worker**
- Neither of you had the perfect message initially
- You refined business value focus based on AI's structure
- AI refined wording based on your feedback
- Convergence produced optimal result combining both inputs

**This is bidirectional learning**—not "AI does what I say" but "AI and I improve together."

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

### Mistake 2: Not Learning from AI's Suggestions

**Symptom**: AI returns output with improvements you didn't ask for, but you ignore them and force AI back to your original (inferior) approach.

**Example**:
- You: "List commit changes with dashes"
- AI: Returns bulleted list with clear categorization and semantic grouping
- You: "No, just simple dashes like I said"
- Result: You get exactly what you asked for (inferior to what AI suggested)

**Fix**: When AI suggests a better pattern, ask "Why did you use this format?" Learn from it.

---

### Mistake 3: Not Teaching AI Your Constraints

**Symptom**: AI keeps producing generic output that doesn't fit your project, and you keep regenerating without explaining your requirements.

**Example**:
- AI: Generic Python code
- You: "Try again"
- AI: Different generic Python code
- You: "Try again"
- AI: Still generic because it doesn't know your constraints

**Fix**: Teach AI your specific requirements: "Our project uses Bash, not Python. Our scripts follow Google Shell Style Guide. We log to /var/log/."

---

### Mistake 4: Changing Too Many Things at Once

**Symptom**: Output is wrong, so you rewrite entire prompt with 10 new constraints. New output is different but still wrong in different ways.

**Problem**: You can't tell which changes improved things and which made them worse.

**Fix**: Iterate **one or two constraints at a time**. This makes it clear what each change contributes.

---

## Iteration Strategies for Different Scenarios

### Strategy 1: Add Constraints Incrementally

**Use when**: Output is vague or generic

**Approach**:
1. Start with basic Intent + Success Criteria
2. Iteration 1: Add technical constraints
3. Iteration 2: Add format constraints
4. Iteration 3: Add quality constraints

**Example**:
```
Iteration 1: "Create backup script"
Iteration 2: + "Must be Bash, backup .md files only"
Iteration 3: + "Output to timestamped folders"
Iteration 4: + "Include error handling and logging"
```

---

### Strategy 2: Learn Then Apply

**Use when**: You're unfamiliar with the domain or best practices

**Approach**:
1. Ask AI to suggest approach/structure
2. Learn from AI's suggestion
3. Apply learned pattern with your constraints
4. Refine based on project specifics

**Example**:
```
Iteration 1: "How should I structure commit messages?" (learn)
Iteration 2: "Use Conventional Commits format for my auth changes" (apply)
Iteration 3: "Add our Jira ticket format requirement" (refine)
```

---

### Strategy 3: Converge Through Dialogue

**Use when**: Requirements are complex or evolving

**Approach**:
1. Start with rough requirement
2. Let AI ask clarifying questions
3. Answer questions (teach AI your context)
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
- Is there a format or structure AI used that you didn't know about?

**If AI used an unfamiliar convention**: Ask AI to explain it! (Learn from AI as Teacher)

---

### Part 3: Teach AI Your Project Standards

Now teach AI your specific requirements:

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

**What did AI teach you?**
- New format or convention?
- Better wording or structure?
- Patterns you hadn't considered?

**What did you teach AI?**
- Project-specific requirements?
- Team conventions?
- Business value focus?

**What emerged from iteration that neither of you had initially?**
- Combination of AI's conventional format + your project constraints?
- Refinements discovered through back-and-forth dialogue?

**How did output quality change from iteration 1 to final iteration?**
- Estimate percentage improvement (e.g., 50% → 90%)

---

## What You've Learned

You've experienced AI collaboration as a **bidirectional partnership**, not a one-way tool:

1. **Iteration loop pattern** (Prompt → Output → Analyze → Refine → Repeat)
2. **AI as Teacher** (AI suggests patterns and conventions you didn't know)
3. **AI as Student** (You teach AI your project-specific requirements)
4. **AI as Co-Worker** (Convergence through dialogue produces better results than either alone)
5. **Convergence criteria** (When to stop: success met, diminishing returns, time budget)
6. **Common mistakes** (Quitting early, ignoring AI's suggestions, not teaching constraints, changing too much at once)

In Lessons 1-2, you built prompt foundations manually. In this lesson, you started **collaborating with AI** to refine prompts through iteration.

Next lesson, you'll learn **specification-first prompting**—how to define "what good looks like" BEFORE writing prompts, using Jake Heller's framework for achieving 97% accuracy.

**Remember**: Your first prompt gets you to 60%. Professional developers embrace iteration to reach 97%. Don't quit early—that's when the magic happens.
