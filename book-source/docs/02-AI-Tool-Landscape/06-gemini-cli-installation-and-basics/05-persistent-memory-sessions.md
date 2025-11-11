---
sidebar_position: 5
title: "Saving & Resuming Conversations"
duration: "30 min"
---

# Saving & Resuming Conversations

## The Power of Conversation Checkpointing

In Lesson 3, you learned the `/chat` family of commands. Now we dive deep into how to use them strategically to maintain long-running projects.

**Key insight**: You have two persistence systems:
- **GEMINI.md** (Lesson 4) = What Gemini should know about your PROJECT
- **Saved conversations** (this lesson) = What you've DISCUSSED with Gemini

Together, they create complete context persistence.

---

## Understanding Conversation Checkpoints

### What Gets Saved?

When you run `/chat save my-project`, Gemini CLI saves:

✅ **Conversation history** — Everything you and Gemini discussed
✅ **Context built up** — Knowledge shared across the conversation
✅ **Decisions made** — What you decided on together
✅ **Code reviewed** — Files you examined
✅ **Current state** — Where you left off in the work

❌ **NOT saved:** Your actual code files (those stay in Git)

### The `/chat` Command Family

| Command | Purpose |
|---------|---------|
| `/chat save <tag>` | Save current conversation |
| `/chat resume <tag>` | Restore a saved conversation |
| `/chat list` | Show all saved conversations |
| `/chat delete <tag>` | Remove a saved conversation |
| `/chat share file.md` | Export conversation to markdown |

---

## Naming Your Saved Conversations

Good names help you find checkpoints quickly:

✅ **Good names:**
- `auth-jwt-complete`
- `database-schema-v2-approved`
- `email-feature-started`
- `debugging-nov-7-morning`

❌ **Avoid:**
- `save1`, `save2`, `checkpoint` (too generic)
- `test`, `temp` (confusing)

**Naming pattern**: `[feature]-[status]` or `[date]-[task]`

---

## Conversation Lifecycle: Save → Resume → Update

### The Cycle

**Step 1: Work on something**
```bash
You: Help me implement JWT authentication
Gemini: I'll help. Here's the approach...
(You discuss and work for 2-3 hours)
```

**Step 2: Save when done with a meaningful chunk**
```bash
You: /chat save jwt-auth-complete
Gemini: ✓ Saved conversation as "jwt-auth-complete"
```

**Step 3: Resume next time you want to continue**
```bash
(Later, in new session)
You: /chat resume jwt-auth-complete
Gemini: ✓ Resuming conversation "jwt-auth-complete"
        (Shows conversation history)

You: Now let's add refresh token rotation
```

**Step 4: Continue your work seamlessly**

Everything continues from where you left off. No re-explaining.

---

## Real Workflow: Multi-Day Project

### Day 1: Authentication Foundation

```bash
$ gemini

You: Let's build JWT authentication. Here are the requirements:
     - Register endpoint
     - Login endpoint
     - Protected routes with JWT validation
     - Refresh tokens

Gemini: I'll help design this...
(Spend 2 hours designing and discussing)

You: /chat save auth-endpoints-designed
Gemini: ✓ Saved conversation as "auth-endpoints-designed"

You: /quit
```

### Day 2: Implementation

```bash
$ gemini

You: /chat resume auth-endpoints-designed
Gemini: ✓ Resumed conversation "auth-endpoints-designed"
        (Shows your design discussion from yesterday)

You: Great, let's implement the register endpoint first
(Continue seamlessly from yesterday - Gemini has context)

(After 2 hours)
You: /chat save register-endpoint-complete
Gemini: ✓ Saved
```

### Day 3: Testing & Refinement

```bash
$ gemini

You: /chat resume register-endpoint-complete
Gemini: ✓ Resumed

You: Now let's write comprehensive tests for the register endpoint

(After testing work)
You: /chat save jwt-auth-system-complete
Gemini: ✓ Saved
```

### Next Project: Switch Contexts

```bash
$ gemini

You: /chat list
Gemini CLI:
  - auth-endpoints-designed
  - register-endpoint-complete
  - jwt-auth-system-complete
  - (other projects)

You: /chat save current-work
You: /clear
(New conversation starts with fresh context)

You: /chat resume frontend-redesign-v2
Gemini: ✓ Resumed previous frontend project
```

---

## Managing Your Saved Conversations

### Listing Conversations

```bash
You: /chat list
Gemini CLI:
  - auth-jwt-complete (Nov 7, 2h 45m)
  - database-migration-in-progress (Nov 5, 1h 20m)
  - api-error-fixes (Oct 28, 45m)
  - frontend-redesign-v2 (Oct 15, 3h 10m)
```

### Exporting Conversations

Share what you've discussed with others:

```bash
You: /chat share project-design.md
Gemini: ✓ Exported conversation to project-design.md
```

You can now share `project-design.md` with teammates so they see your design decisions.

### Deleting Old Conversations

Clean up completed projects:

```bash
You: /chat delete old-feature-spike
Gemini: ✓ Deleted "old-feature-spike"
```

---

## Advanced Patterns

### Pattern 1: Experimentation With Safety

Try a risky approach without losing current work:

```bash
You: /chat save current-stable-state
Gemini: ✓ Saved

(Try experimental refactoring)
You: Hmm, this approach isn't working well...

You: /clear
Gemini: ✓ Conversation cleared

You: /chat resume current-stable-state
Gemini: ✓ Loaded "current-stable-state"
(You're back to a known good state)
```

### Pattern 2: Parallel Exploration

Work on two approaches simultaneously:

```bash
You: /chat save approach-a-v1
Gemini: ✓ Saved

You: /clear
Gemini: ✓ Fresh conversation

You: Let me try a different architecture approach...
(Work for 1 hour)

You: /chat save approach-b-v1
Gemini: ✓ Saved

You: /clear
You: /chat resume approach-a-v1
(Compare both approaches later)
```

### Pattern 3: Pair Programming handoff

Developer A saves their work, Developer B resumes:

```bash
# Developer A (morning)
You: [Work on feature for 2 hours]
You: /chat save feature-auth-state-for-bob
You: /quit

# Developer B (afternoon)
You: /chat resume feature-auth-state-for-bob
Gemini: ✓ Resumed Bob's conversation
(Bob's context and decisions are preserved)
```

### Pattern 4: Multi-Hour Sessions with Checkpoints

Break up long work with periodic saves:

```bash
(Work for 45 min)
You: /chat save auth-endpoints-skeleton
Gemini: ✓ Saved

(Work another 45 min on auth tests)
You: /chat save auth-tests-complete
Gemini: ✓ Saved

(Work another 45 min on rate limiting)
You: /chat save auth-rate-limiting-complete
Gemini: ✓ Saved

(Later, if something breaks)
You: /clear
You: /chat resume auth-tests-complete
(Jump back to a known good state)
```

---

## GEMINI.md + Saved Conversations Together

These two systems complement each other:

**GEMINI.md** (persistent, project-wide):
- What Gemini should know about your project
- Architecture, decisions, conventions
- Loaded automatically every session

**Saved conversations** (per-person, per-task):
- What YOU discussed with Gemini
- Your specific design decisions THIS session
- Context for THIS task continuation

### Complete Workflow

```bash
Day 1:
  $ cd auth-service && gemini
  (GEMINI.md loads automatically)
  You: [Discuss architecture for 2 hours]
  You: /chat save arch-design-approved
  You: /quit

Day 2:
  $ cd auth-service && gemini
  (GEMINI.md loads automatically - same as before)
  You: /chat resume arch-design-approved
  (Now you have BOTH the project context AND yesterday's work)
  You: Let's implement the routes now

Day 3 (switch projects):
  $ cd frontend && gemini
  (Different GEMINI.md loads automatically)
  You: /clear
  You: /chat resume frontend-components-v2
  (Now working on frontend with its own GEMINI.md)
```

**Result**: You always have the right context for the right project.

---

## Common Questions

**Q: What's the difference between saving conversations and GEMINI.md?**

A:
- **GEMINI.md** = "Project architecture is X, we use TypeScript, our conventions are Y"
- **Saved conversation** = "We discussed X yesterday, decided on approach Y, need to implement Z today"

Save conversations for work continuity. Use GEMINI.md for project knowledge.

**Q: Can I resume someone else's conversation?**

A: Not directly—saved conversations are personal. But you can export with `/chat share` and send to a teammate.

**Q: If I don't save, is my conversation lost?**

A: Yes. Always save important work with `/chat save` before closing Gemini CLI.

**Q: Can I rename a saved conversation?**

A: Not directly. Use `/chat export` to file, then save as new conversation.

**Q: How long are conversations kept?**

A: Conversations are stored locally on your machine. They persist until you delete them with `/chat delete`.

**Q: How much storage do saved conversations use?**

A: Conversations are text, so very little. A typical 2-hour conversation might be 500KB-2MB.

---

## Exercises

### Exercise 1: Save Your First Conversation

Have a real conversation about something meaningful:

```bash
$ gemini
You: [Work on a real task for 20+ minutes]
You: /chat save first-saved-session
Gemini: ✓ Saved
```

**Expected**: You can now resume this conversation anytime.

### Exercise 2: List and Inspect

```bash
You: /chat list
(See all your saved conversations)

You: /chat save another-session
You: /chat list
(See it appears in the list)
```

### Exercise 3: Resume and Continue

Close Gemini CLI. Reopen it:

```bash
$ gemini
You: /chat resume first-saved-session
```

**Expected**: Your conversation history is restored. Gemini remembers what you discussed.

### Exercise 4: Export and Share

```bash
You: /chat share my-design.md
(Check that my-design.md was created)

You: cat my-design.md
(Read the exported conversation)
```

### Exercise 5: Multi-Session Workflow

Save, clear, resume:

```bash
You: [Work for 10 minutes]
You: /chat save session-one
You: /clear
(Fresh conversation - old context is gone)

You: [Work on something different for 10 minutes]
You: /chat save session-two
You: /clear

You: /chat resume session-one
(Conversation one context restored)
```

---

## Best Practices

### ✅ DO

- ✅ Save when you complete meaningful work
- ✅ Use clear, descriptive names
- ✅ Save before switching projects
- ✅ Export important decisions with `/chat share`
- ✅ List your conversations occasionally to stay organized
- ✅ Delete old completed projects to keep list clean

### ❌ DON'T

- ❌ Forget to save—close Gemini without `/chat save`
- ❌ Use generic names like "save1", "temp", "test"
- ❌ Accumulate dozens of conversations (clean up periodically)
- ❌ Assume conversations are synced across devices (they're local)

---

## Key Takeaways

- **Saved conversations** persist your work across sessions
- **`/chat save <tag>`** saves current conversation
- **`/chat resume <tag>`** loads a previous conversation
- **`/chat list`** shows all saved conversations
- **`/chat share file.md`** exports for sharing with others
- **Combined with GEMINI.md** = complete context persistence
- **Use clear names** like `feature-complete` or `date-task`
- **Save strategically** after completing meaningful chunks
- **Clean up periodically** with `/chat delete`

Next lesson: You'll learn **MCP servers** to extend Gemini CLI with external integrations (web browsing, API documentation, GitHub).


<Quiz title="Chapter 5 Quiz" questions={[{"question":"What are the six phases of the traditional software development lifecycle mentioned in the chapter?","options":{"a":"Discovery, Design, Development, Debugging, Deployment, Decommissioning","b":"Planning, Design, Implementation, Testing, Deployment, Operations","c":"Requirements, Architecture, Coding, Review, Release, Retirement","d":"Idea, Prototype, Build, Test, Launch, Iterate"},"correct_answer":"b","explanation":"The chapter explicitly lists the phases: \u0027The traditional software development lifecycle looks something like this: Planning → Design → Implementation → Testing → Deployment → Operations.\u0027"},{"question":"How does the AI-augmented approach to the \u0027Planning \u0026 Requirements\u0027 phase differ from the traditional approach?","options":{"a":"AI completely replaces product managers.","b":"AI helps extract requirements, suggest edge cases, and identify inconsistencies before development starts.","c":"The planning phase is skipped entirely.","d":"AI only helps with formatting the requirements document."},"correct_answer":"b","explanation":"The text states the AI-augmented approach includes: \u0027Natural language processing helps extract requirements... AI agents suggest edge cases... Automated analysis identifies inconsistencies and ambiguities...\u0027"},{"question":"In the \u0027Testing \u0026 Quality Assurance\u0027 phase, what is a key benefit of using an AI-augmented approach?","options":{"a":"It eliminates the need for human QA engineers.","b":"It only works for unit tests, not integration tests.","c":"AI can generate comprehensive test suites and identify edge cases that humans might miss.","d":"It makes testing slower but more thorough."},"correct_answer":"c","explanation":"The chapter highlights that with AI, it \u0027generates comprehensive test suites from requirements and code\u0027 and \u0027Automatically identifies edge cases developers didn\u0027t think to test.\u0027"},{"question":"What is the \u0027compounding effect\u0027 of AI transformation across the development lifecycle?","options":{"a":"Each phase becomes more complex and expensive.","b":"Improvements in one phase are isolated and do not affect other phases.","c":"An improvement in an early phase (like planning) leads to benefits and efficiencies in all subsequent phases.","d":"The total time for development increases due to the need for more reviews."},"correct_answer":"c","explanation":"The text explains, \u0027Each phase improvement compounds with others. When AI helps you identify edge cases during planning, you write better requirements. Better requirements lead to better architecture,\u0027 and so on."},{"question":"What is happening to specialized roles like \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 as a result of AI tools?","options":{"a":"These roles are becoming more distinct and siloed.","b":"The boundaries between these roles are blurring as AI enables individuals to handle more responsibilities.","c":"These roles are being completely eliminated.","d":"Only the developer role is changing."},"correct_answer":"b","explanation":"The chapter notes, \u0027The boundaries between \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 are blurring. AI tools enable individual contributors to handle responsibilities that previously required dedicated specialists.\u0027"}]} />

