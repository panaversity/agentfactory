---
title: "Lesson 2: Lists for Structured Requirements"
description: "Organize features and requirements using bullet and numbered lists"
sidebar_label: "Lesson 2: Lists for Requirements"
sidebar_position: 2
lesson: 2
chapter: 9
part: 3
duration: 50
concepts: 7
proficiency: A2
stage: 2
generated_by: content-implementer v1.0.0
source_spec: specs/034-chapter-9-markdown-redesign/spec.md
created: 2025-11-18
workflow: /sp.implement
version: 1.0.0
skills:
  markdown-lists:
    proficiency: A2
  ai-collaboration:
    proficiency: A2
  specification-clarity:
    proficiency: A2
---

# Lesson 2: Lists for Structured Requirements

In Lesson 1, you used headings to create document structure. Now you'll learn to enumerate requirements using **lists**.

Lists transform vague prose into discrete, machine-readable requirements. AI agents parse each list item as a separate requirement—making your specifications more precise.

This is also where you'll start collaborating with AI. You'll see how AI can suggest structure you didn't think of, how you refine AI's suggestions with your knowledge, and how iteration produces better specifications than either of you could create alone.

---

## Why Lists Matter for Specifications

Compare these two approaches:

**Without Lists (Vague)**:
```
The task manager should let users create tasks with names and maybe priorities,
and they should be able to view all their tasks in some kind of list format,
and mark them complete when done, plus delete tasks they don't need anymore.
```

**With Lists (Clear)**:
```
## Core Features

### Create Task
Required inputs:
- Task name (text, max 100 characters)
- Priority (optional: high, medium, low)

### View Tasks
Display format:
- Task name
- Priority level
- Completion status
- Creation date

### Manage Tasks
Available actions:
- Mark task as complete
- Delete task
- Edit task name
```

**What changed?**
- Continuous paragraph → Discrete list items
- Vague "some kind of list format" → Specific display elements
- Mixed concepts → Organized by feature

AI agents can now parse exactly what each feature requires.

---

## Concept 1: Bullet Lists for Alternative Options

Use bullet lists (`-`) when describing **alternatives**, **options**, or **independent requirements** that don't have to happen in order.

### Syntax

```
- First item
- Second item
- Third item
  - Sub-item (indent with 2 spaces)
  - Another sub-item
- Fourth item
```

**Important**: Space after the `-` is required.

### When to Use Bullet Lists

**Scenario 1: Alternative Options**

```
## User Authentication Methods

Choose ONE method:
- Email and password
- Google OAuth
- GitHub OAuth
```

These are alternatives. User picks one, not all.

**Scenario 2: Independent Features**

```
## Dashboard Widgets

Available widgets:
- Recent activity feed
- Quick statistics
- Upcoming deadlines
- Team collaboration panel
```

Each widget is independent. Order doesn't matter.

**Scenario 3: Feature Requirements**

```
## Mobile App Requirements

Must support:
- iOS 15 or higher
- Android 11 or higher
- Tablet layouts
- Offline mode
```

All required, but no specific sequence.

---

## Concept 2: Numbered Lists for Sequential Steps

Use numbered lists when describing **sequences**, **steps**, or **procedures** that happen in specific order.

### Syntax

```
1. First step
2. Second step
3. Third step
   1. Sub-step (indent with 3 spaces)
   2. Another sub-step
4. Fourth step
```

**Important**: Numbers will auto-increment in rendered markdown, but write them explicitly for clarity.

### When to Use Numbered Lists

**Scenario 1: User Workflow**

```
## User Onboarding Process

When new user signs up:
1. User enters email and password
2. System sends verification email
3. User clicks verification link
4. System activates account
5. User redirected to dashboard
```

This MUST happen in order. Step 2 can't happen before Step 1.

**Scenario 2: Implementation Steps**

```
## Database Migration Process

Execute in sequence:
1. Backup current database
2. Run migration scripts
3. Verify data integrity
4. Update application config
5. Restart services
```

Wrong order → broken system. Sequence matters.

**Scenario 3: Installation Instructions**

```
## Setup Instructions

Follow these steps:
1. Install Python 3.8 or higher
2. Clone repository from GitHub
3. Install dependencies: `pip install -r requirements.txt`
4. Create `.env` file with API keys
5. Run application: `python app.py`
```

Users must complete step 1 before step 2, etc.

---

## Concept 3: Choosing Bullets vs Numbers

**Decision framework**:

**Use bullet lists (`-`) when**:
- Order doesn't matter
- Items are alternatives or options
- Requirements are independent
- Feature list where sequence is irrelevant

**Use numbered lists (`1.`) when**:
- Order DOES matter
- Steps must happen sequentially
- Procedure or workflow
- Installation/setup instructions

### Example: Same Content, Different Context

**Context 1: Feature Planning (Bullets - No Order)**

```
## Features to Build (Priority Planning)

MVP features:
- User authentication
- Task creation
- Task viewing
- Task completion

Future features:
- Task sharing
- Advanced filtering
- Calendar integration
```

Order doesn't matter. We'll build all MVP features.

**Context 2: Implementation Schedule (Numbers - Specific Order)**

```
## Implementation Sequence

Build in this order:
1. User authentication (foundation)
2. Task creation (can't view without creating first)
3. Task viewing (need tasks to exist)
4. Task completion (requires tasks to be viewable)
```

Same features, but now sequence matters for dependencies.

**Key insight**: Format follows function. Think about whether order matters, then choose the right list type.

---

## Concept 4: Nested Lists for Hierarchy

You can nest lists to show relationships between items.

### Nested Bullet Lists

```
## System Requirements

Technical stack:
- Backend
  - Python 3.8+
  - Flask framework
  - PostgreSQL database
- Frontend
  - React 18
  - TypeScript
  - Tailwind CSS
- DevOps
  - Docker containers
  - GitHub Actions CI/CD
  - AWS deployment
```

Each sub-item relates to its parent category.

### Nested Numbered Lists

```
## User Story: Complete Task

1. User opens task list
   1. System displays all tasks
   2. Tasks sorted by creation date
   3. Incomplete tasks shown first
2. User selects task to mark complete
   1. User clicks checkbox next to task
   2. System updates task status
   3. Visual feedback: strikethrough text
3. System confirms completion
   1. Success message appears
   2. Completed task moves to bottom
   3. Progress percentage updates
```

Sub-steps clarify what happens within each main step.

---

## Working with AI: From Structure to Clarity

Now let's see how collaborating with AI improves your specifications through iteration.

### Starting Point: Your Initial Draft

Imagine you're specifying a task manager's "Create Task" feature. You write:

```
### Create Task

User wants to add new task to system. Task should have a name. Maybe add priority
or due date? Should be fast and easy to use. Mobile friendly would be good.
```

This is vague. What's required vs optional? What does "fast" mean? What's "mobile friendly"?

### AI Suggests Structure

You ask AI: "Help me organize these requirements clearly using lists."

AI suggests:

```
### Create Task

**Required Inputs:**
- Task name (text, max 100 characters)
- Creation timestamp (auto-generated)

**Optional Inputs:**
- Priority (dropdown: high/medium/low)
- Due date (date picker)
- Description (textarea, max 500 characters)

**Success Criteria:**
- Task saved within 1 second
- Confirmation message displayed
- New task appears at top of list

**Mobile Considerations:**
- Large touch targets (44x44 pt minimum)
- Optimized keyboard layout for text input
- Swipe gestures for quick save
```

**What you learned from AI:**
- Separating required vs optional (you hadn't made this distinction)
- Defining "fast" with measurable criterion (1 second)
- Specific mobile features (touch targets, keyboard, gestures) instead of vague "mobile friendly"

AI introduced structure you weren't considering.

### You Refine Based on Project Constraints

You review AI's suggestion and realize: "This is great, but for my MVP (Minimum Viable Product), I need simpler scope."

You refine:

```
### Create Task (MVP Scope)

**Required Inputs:**
- Task name (text, max 100 characters)

**Success Criteria:**
- Task saved within 1 second
- Confirmation: "Task created successfully"
- Task appears at top of list

**Deferred to v2:**
- Priority levels (not in MVP)
- Due dates (not in MVP)
- Description field (not in MVP)
- Mobile optimizations (desktop-first MVP)
```

**What emerged from your feedback:**
- MVP constraint clarified (simple scope only)
- Desktop-first approach specified
- Future features explicitly deferred

### Together, You Converge on Clarity

Through iteration:

**Iteration 1**: Your vague paragraph → AI structures it
**Iteration 2**: AI's suggestion → You constrain to MVP
**Iteration 3**: What if user submits empty task name?

```
### Create Task (Final Specification)

**Required Inputs:**
- Task name (text, max 100 characters, cannot be empty)

**Validation:**
- If name empty: Show error "Task name required"
- If name > 100 chars: Show error "Name too long (max 100 characters)"

**Success Criteria:**
- Valid task saved within 1 second
- Confirmation: "Task created: [task name]"
- Task appears at top of list with timestamp

**Deferred to v2:**
- Priority levels
- Due dates
- Description field
- Mobile optimizations
```

**What emerged through iteration:**
- Validation rules (neither started with these)
- MVP scope constraints (emerged from your feedback)
- Edge cases (discovered through questioning)
- Final spec is clearer than the initial draft

---

## Concept 5: Lists Make Requirements Machine-Readable

Why do AI agents prefer lists over paragraphs?

**Paragraph (Hard to Parse)**:
```
The user can authenticate using either email and password or they can use Google
OAuth or GitHub OAuth, and once authenticated they get redirected to the dashboard
where they can see recent activity and statistics.
```

AI reads this as **one continuous requirement**. Hard to parse where one feature ends and another begins.

**Lists (Easy to Parse)**:
```
## Authentication Options

User chooses ONE method:
- Email and password
- Google OAuth
- GitHub OAuth

## Post-Authentication Behavior

After successful login:
1. System generates session token
2. User redirected to dashboard
3. Dashboard displays:
   - Recent activity feed
   - Quick statistics
```

AI parses this as:
- **3 authentication options** (alternatives)
- **3 sequential steps** after login
- **2 dashboard components** (independent)

Lists create **discrete, parseable units** instead of continuous prose.

---

## Practice Exercise: Convert Unstructured to Structured Lists

Here's a vague feature description. Your task: Rewrite using lists.

**Original (Unstructured)**:
```
## Shopping Cart Feature

Users should be able to add items to their cart and see what's in the cart and
change quantities or remove items they don't want anymore. When they're ready to
checkout, they should see the total price including tax and shipping, and then
they can enter payment info and shipping address and submit the order.
```

**Your Task**:

1. Open your text editor
2. Create file: `shopping-cart-spec.md`
3. Rewrite the specification using:
   - Bullet lists for cart operations (no specific order)
   - Numbered lists for checkout process (specific order)
   - Nested lists where helpful

**Questions to guide you**:
- Which actions can happen in any order? (bullets)
- Which actions must happen sequentially? (numbers)
- What sub-details need clarification? (nested items)

### Example Solution Structure

```
## Shopping Cart Feature

### Cart Operations

Available actions (any order):
- Add item to cart
- View cart contents
- Update item quantity
- Remove item from cart

### Cart Display

Show for each item:
- Product name
- Quantity selected
- Unit price
- Subtotal (quantity × price)

### Checkout Process

1. User reviews cart contents
2. System calculates totals:
   1. Subtotal (sum of all items)
   2. Tax (based on shipping address)
   3. Shipping cost (based on weight/destination)
   4. Grand total
3. User enters shipping address
4. User enters payment information
5. User confirms order
6. System processes payment
7. System sends confirmation email
```

**Success Criteria**:
- ✅ Cart operations use bullet lists (order doesn't matter)
- ✅ Checkout process uses numbered list (order matters)
- ✅ Nested lists clarify details (like calculation breakdown)
- ✅ Each requirement is discrete and clear

---

## Try With AI

Practice iterating with AI to refine your list structures.

**Setup**: Open Claude Code or Gemini CLI.

**Part 1: Initial Structure Request**

Take your shopping cart specification from the practice exercise. Ask AI:

```
I'm creating a shopping cart specification. Here's what I wrote:

[Paste your specification]

Can you suggest how to organize this more clearly using lists?
What requirements am I missing?
```

**Part 2: Critical Evaluation**

Review AI's response. Ask yourself:
- Does this match my MVP scope?
- Are there features too complex for v1?
- What assumptions did AI make about my project?

**Part 3: Constraint Teaching**

Based on your evaluation, tell AI your constraints:

```
Good suggestions, but I'm building an MVP. Let's simplify:
- No tax calculation (flat pricing for now)
- No multiple shipping options (standard shipping only)
- No saved addresses (one-time entry)

Please revise the specification with these constraints.
```

**Part 4: Edge Case Discovery**

Ask AI to identify what could go wrong:

```
What happens if:
- User's cart is empty when they click checkout?
- Item goes out of stock after being added to cart?
- Payment fails?

Add these edge cases to the specification.
```

**Part 5: Validation**

Compare your original specification to the final version after iteration:
- Which list types did you use? (Bullets for options, numbers for sequences)
- What requirements emerged that you didn't initially write?
- What did AI suggest that you rejected as out-of-scope?
- Is this specification clearer than your first draft? Why?
