---
title: "Code Blocks for Specifications (Not Code)"
description: "Use code blocks to show expected outputs and API descriptions without implementation code"
sidebar_label: "Code Blocks for Specifications"
sidebar_position: 3
lesson: 3
chapter: 9
part: 3
duration: 55
concepts: 7
proficiency: A2
stage: 2
generated_by: content-implementer v1.0.0
source_spec: specs/034-chapter-9-markdown-redesign/spec.md
created: 2025-11-18
workflow: /sp.implement
version: 1.0.0
skills:
  markdown-code-blocks:
    proficiency: A2
  specification-outputs:
    proficiency: A2
  ai-iteration:
    proficiency: A2
---

# Code Blocks for Specifications (Not Code)

You've learned headings (Lesson 1) and lists (Lesson 2). Now comes a powerful markdown feature that's **easy to misunderstand**: code blocks.

**Critical distinction**: Code blocks can show specifications (WHAT system does) without showing implementation (HOW it's coded). This lesson teaches you to use code blocks for **expected outputs**, **API descriptions**, and **system behavior**—NOT programming code.

You'll continue collaborating with AI, this time focusing on **specification clarity**. AI will help you describe outputs precisely, you'll refine with domain knowledge, and together you'll converge on unambiguous specifications.

---

## The Code Block Misconception

**Common mistake**: "Code blocks are for showing programming code."

**Reality**: Code blocks are for showing **formatted text that should be displayed exactly as written**. This includes:
- Expected system outputs (WHAT user sees)
- API request/response formats (WHAT data flows)
- Error messages (WHAT system displays)
- Specification examples (WHAT behavior is expected)

Code blocks preserve spacing, line breaks, and formatting—making them perfect for specifications.

---

## Concept 1: Code Block Syntax (Triple Backticks)

Code blocks use three backticks (` ``` `) to mark the beginning and end.

### Basic Syntax

````
```
This text appears in a code block.
It preserves    spacing.
And line breaks.
```
````

**Important**:
- Three backticks to open (` ``` `)
- Content on separate lines
- Three backticks to close (` ``` `)
- Blank line before and after for readability

### With Context Labels

Always label what the code block represents:

````
**Expected Output:**
```
Welcome to Task Manager!

Your tasks:
1. Buy groceries - incomplete
2. Write report - complete
3. Call dentist - incomplete

Total: 3 tasks | Complete: 1 | Remaining: 2
```
````

The label "Expected Output:" tells readers (and AI) this is specification, not code.

---

## Concept 2: Expected Output Specifications

Use code blocks to show WHAT the system displays to users.

### Example: Login Success Message

````
## Login Feature

### Success Case

**Expected Output:**
```
Login successful!

Welcome back, Sarah!
Last login: November 17, 2025 at 3:42 PM

Redirecting to dashboard...
```
````

**What this specification provides**:
- Exact message text
- Personalization (user's name)
- Last login timestamp format
- Redirection notice

No ambiguity about what the user should see.

### Example: Task List Display

````
## View Tasks Feature

**Expected Output:**
```
=== My Tasks ===

[ ] 1. Buy groceries
    Created: Nov 17, 2025
    Priority: High

[✓] 2. Write report
    Created: Nov 16, 2025
    Priority: Medium
    Completed: Nov 18, 2025

[ ] 3. Call dentist
    Created: Nov 18, 2025
    Priority: Low

---
Total: 3 | Complete: 1 | Incomplete: 2
```
````

**What this specifies**:
- Checkbox symbols ([ ] for incomplete, [✓] for complete)
- Task numbering format
- Metadata display (created date, priority, completion date)
- Summary footer format

Every visual detail is clear.

---

## Concept 3: API Endpoint Descriptions

Use code blocks to describe WHAT data an API endpoint accepts and returns—without showing HOW it's implemented.

### Example: Create Task Endpoint

````
## POST /tasks/create

**Request Format:**
```
{
  "name": "Buy groceries",
  "priority": "high",
  "due_date": "2025-11-20"
}
```

**Success Response (200 OK):**
```
{
  "id": 42,
  "name": "Buy groceries",
  "priority": "high",
  "due_date": "2025-11-20",
  "created_at": "2025-11-18T10:30:00Z",
  "status": "incomplete"
}
```

**Error Response (400 Bad Request):**
```
{
  "error": "Task name required",
  "field": "name"
}
```
````

**What this specifies**:
- Request structure (what client sends)
- Success response structure (what server returns when it works)
- Error response structure (what server returns when it fails)
- Field names and data types
- HTTP status codes

AI can now implement this API endpoint because the contract is unambiguous.

---

## Concept 4: Error Message Specifications

Use code blocks to specify exact error messages and when they appear.

### Example: Form Validation Errors

````
## Create Account Form

### Validation Rules

**If email field is empty:**
```
Error: Email address required
Please enter a valid email address.
```

**If email format invalid:**
```
Error: Invalid email format
Email must be in format: name@example.com
```

**If password too short:**
```
Error: Password too short
Password must be at least 8 characters.
```

**If passwords don't match:**
```
Error: Passwords do not match
Please ensure both password fields are identical.
```
````

**Why this matters**:
- Developers know exact error text
- UX designers know what users will see
- QA testers know what to validate
- AI knows what to implement

No one has to guess what "invalid input" means.

---

## Iterating with AI: From Vague to Precise

Let's see how collaborating with AI improves specification clarity.

### Your Initial Attempt (Vague)

You're specifying a user dashboard and write:

````
## Dashboard

When user logs in, they see their dashboard with recent activity and some stats.
````

This is too vague. What does "recent activity" look like? What stats? What format?

### AI Suggests Output Format

You ask: "Help me describe what the dashboard output looks like clearly."

AI suggests:

````
## Dashboard

**Expected Output:**
```
Welcome, [Username]!

=== Recent Activity ===
• Created task "Buy groceries" - 2 hours ago
• Completed task "Write report" - 5 hours ago
• Created task "Call dentist" - 1 day ago

=== Quick Stats ===
Total Tasks: 12
Completed: 8 (67%)
Incomplete: 4 (33%)
Completed This Week: 5
```
````

**What you learned from AI**:
- Specific format for activity items (bullet points, action + timestamp)
- Specific stats with labels and percentages
- How to show username personalization
- Structure with section headers

AI introduced precision you didn't have initially.

### You Refine with Domain Knowledge

You review and realize: "Good structure, but my users need different information."

You refine:

````
## Dashboard (Refined)

**Expected Output:**
```
Hello, [Username]! Today is [Current Date]

=== Today's Tasks (Due Today) ===
[ ] Buy groceries (Priority: High)
[ ] Call dentist (Priority: Low)

=== Overdue Tasks ===
[ ] Review contract (Due: Nov 15) - 3 days overdue

=== Completed Today ===
[✓] Write report
[✓] Send emails

=== This Week's Progress ===
Monday: 2 tasks completed
Tuesday: 3 tasks completed
Wednesday: 1 task completed
Today: 2 tasks completed so far

Weekly Goal: 15 tasks | Current: 8 tasks (53%)
```
````

**What changed based on your feedback**:
- Focus shifted to TODAY's tasks (not all recent activity)
- Overdue tasks now highlighted (time-sensitive)
- Daily breakdown emphasized over overall stats
- Weekly goal tracking added

### Together, Converge on Edge Cases

Through iteration, you discover edge cases:

**Iteration 1**: What if no tasks due today?

````
=== Today's Tasks (Due Today) ===
No tasks due today. Great job staying ahead!
````

**Iteration 2**: What if no overdue tasks?

````
=== Overdue Tasks ===
✓ No overdue tasks! You're on track.
````

**Iteration 3**: What if user hasn't set weekly goal?

````
=== This Week's Progress ===
Monday: 2 tasks
Tuesday: 3 tasks
Today: 2 tasks

Set a weekly goal to track progress!
````

**What emerged**:
- Neither of you started with empty state handling
- AI didn't know weekly goals were optional
- You didn't think about "no overdue" case
- Iteration revealed all edge cases

Final specification is comprehensive because you co-worked through scenarios.

---

## Concept 5: Specification Context (Always Label Code Blocks)

**Critical practice**: Always precede code blocks with context labels so readers know what they're looking at.

### Good Examples (With Context)

````
**Expected CLI Output:**
```
Task created successfully.
ID: 42
Name: Buy groceries
```

**API Response:**
```
{
  "status": "success",
  "task_id": 42
}
```

**Error Message:**
```
Error: Task name cannot be empty
Please enter a task name and try again.
```
````

### Bad Examples (No Context)

````
```
Task created successfully.
ID: 42
```

```
{
  "status": "success"
}
```
````

**Problem**: Reader doesn't know if this is expected output, API response, example input, or something else.

**Always label** with:
- "Expected Output:"
- "API Request:"
- "API Response:"
- "Error Message:"
- "System Behavior:"
- "CLI Display:"

Context makes specifications unambiguous.

---

## Concept 6: When NOT to Use Code Blocks

Code blocks are for **formatted output specifications**, not narrative explanations.

### Don't Use Code Blocks For:

**Regular text descriptions:**
```
❌ Wrong:
```
The system allows users to create tasks with names and priorities.
```
```

Use regular markdown paragraph instead:

```
✓ Right:
The system allows users to create tasks with names and priorities.
```

**Lists:**
```
❌ Wrong:
```
Features:
- Create task
- View tasks
- Delete task
```
```

Use markdown lists instead:

```
✓ Right:
Features:
- Create task
- View tasks
- Delete task
```

**Headings:**
```
❌ Wrong:
```
## Create Task Feature
```
```

Use markdown headings instead:

```
✓ Right:
## Create Task Feature
```

### Do Use Code Blocks For:

- Expected system outputs
- API request/response formats
- Error messages
- Command-line interface displays
- Data structure examples
- Console logs
- Any text that must be formatted exactly as shown

---

## Concept 7: Validating Specification Clarity Through AI Feedback

After writing specifications with code blocks, validate clarity by asking AI for feedback.

### Validation Process

**Step 1**: Write your specification with code blocks

**Step 2**: Ask AI: "Can you identify what features I need to implement from this specification? Are there any ambiguous requirements?"

**Step 3**: AI reviews and identifies:
- ✅ Clear requirements
- ⚠️ Ambiguities or missing information
- ⚠️ Potential edge cases not covered

**Step 4**: Refine specification based on feedback

**Step 5**: Re-validate until AI confirms clarity

### Example Validation

**Your Specification:**
````
## Login Feature

**Expected Output:**
```
Login successful!
Welcome, [Username]!
```
````

**AI Feedback:**
> "Specification is clear for success case, but:
> - What happens if username/password are incorrect?
> - What happens if account is locked?
> - Is there a rate limit on login attempts?
> - Should system show account status (e.g., 'verified' vs 'unverified')?"

**You Refine:**
````
## Login Feature

### Success Case

**Expected Output:**
```
Login successful!
Welcome, [Username]!
Account Status: Verified
Last login: [Timestamp]
```

### Error Cases

**If credentials incorrect:**
```
Login failed.
Incorrect username or password.
Attempts remaining: 2
```

**If account locked:**
```
Account locked.
Too many failed login attempts.
Try again in 15 minutes or reset password.
```

**If account unverified:**
```
Account not verified.
Please check your email for verification link.
[Resend verification email]
```
````

**AI Confirms:**
> "Specification is now unambiguous. All cases covered:
> - Success path clear
> - Error handling defined
> - Rate limiting visible (attempts remaining)
> - Account states handled (verified, locked, unverified)"

Iteration improved clarity.

---

## Practice Exercise: Add Expected Outputs to Feature Specification

Take this vague feature description and add code blocks showing expected outputs.

**Feature Description:**
```
## Password Reset Feature

Users who forget their password can request a reset. They enter their email,
system sends reset link, user clicks link and enters new password, system
confirms password changed.
```

**Your Task:**

1. Open text editor
2. Create file: `password-reset-spec.md`
3. Add code blocks for:
   - Expected output when user requests reset
   - Expected email content (what user receives)
   - Expected output on reset link page
   - Expected success message after password changed
   - Expected error messages (invalid email, link expired, password too weak)

### Example Structure

````
## Password Reset Feature

### Step 1: Request Reset

**User Input:** Email address

**Expected Output (Success):**
```
Password reset requested.
We've sent a reset link to: user@example.com
Check your email and click the link to reset your password.
Link expires in 1 hour.
```

**Expected Output (Email Not Found):**
```
[Add error message here]
```

### Step 2: Email Content

**Expected Email:**
```
[Add email content here with reset link]
```

### Step 3: Reset Link Page

**Expected Output:**
```
[Add password reset form display here]
```

### Step 4: Success Confirmation

**Expected Output:**
```
[Add success message here]
```

### Edge Cases

**If link expired:**
```
[Add expired link message here]
```

**If new password too weak:**
```
[Add weak password error here]
```
````

**Success Criteria**:
- ✅ All code blocks have context labels ("Expected Output:", "Expected Email:", etc.)
- ✅ Code blocks show WHAT user sees (not HOW it's implemented)
- ✅ All happy path and error cases covered
- ✅ Exact message text specified (no vague "show error")

---

## Try With AI

Practice iterating with AI to refine code block specifications.

**Setup**: Open Claude Code or Gemini CLI.

**Part 1: Initial Precision Request**

Take your password reset specification from the practice exercise. Ask AI:

```
I'm specifying a password reset feature. Here's my current specification:

[Paste your specification from practice exercise]

Can you suggest improvements to make the expected outputs more precise?
What details am I missing?
```

**Part 2: Critical Review**

Review AI's suggestions. Ask yourself:
- Which suggestions improve clarity?
- Which suggestions add complexity beyond my MVP?
- What did AI assume about my project that isn't accurate?

**Part 3: Constraint Communication**

Based on your review, tell AI your actual requirements:

```
Good suggestions. But for my MVP:
- No password strength meter (just basic validation: min 8 chars)
- Email is plain text (no HTML)
- No "resend link" feature (one-time send only)

Please revise the specification with these constraints.
```

**Part 4: Clarity Validation**

Test whether the specification is implementable. Ask AI:

```
Review this updated specification. Can you identify what needs to be implemented?
Are there any ambiguous parts?
```

If AI identifies ambiguities (e.g., "What happens if user clicks expired link twice?"), refine your specification and re-validate.

**Part 5: Final Check**

Compare your original specification to the final version:
- Which output details became more precise through iteration?
- Which requirements did you add based on AI's questions?
- Which suggestions did you reject as out-of-scope?
- Can someone implement this specification without asking clarifying questions?
