---
title: "Providing Technical Context - Project-Specific Code"
chapter: 10
lesson: 3
duration_minutes: 45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Providing Project Context"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student adds 4-layer context to basic prompt, producing project-specific AI code"

  - name: "Leveraging Existing Code Patterns"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student includes code snippet as example; AI matches existing style"

# Learning objectives with proficiency levels
learning_objectives:
  - objective: "Provide 4-layer context (project, code, constraints, developer) to AI agents"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Exercise where student adds all 4 context layers to prompt"

  - objective: "Transform generic AI code into project-specific, tailored implementations"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Side-by-side comparison of generic vs. contextual AI output"

  - objective: "Use file structure and existing code as context examples"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Exercise where student includes code snippet; AI matches style"

# Cognitive load tracking (prevent overload)
cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (4-layer context stack, project context, code context, constraint context, developer context) within CEFR A2 limit of 7 concepts per lesson ✓"

# Optional: Differentiation guidance
differentiation:
  extension_for_advanced: "Create context layers for a multi-service microservices project; analyze tradeoffs between over-documentation and clarity"
  remedial_for_struggling: "Use provided template for each context layer; focus on one layer at a time before combining all four"
---

## The AIDD Context Stack: Giving Your AI Agent the Full Blueprint

In Lesson 2, you learned that **strong commands** guide AI toward better code. Now we're going to unlock something even more powerful: **context**.

Imagine you're hiring a contractor to build an addition to your house. You could tell them "Build a bedroom" and let them guess. Or you could give them complete blueprints showing:
- Your home's existing structure and style
- The exact materials you've already used elsewhere
- Building codes and safety requirements that apply
- Your own preferences and constraints

The second approach gets better results, faster. **AI agents work the same way.**

Generic prompts produce generic, boilerplate code. **Contextual prompts produce project-specific, production-ready code.** In this lesson, you'll learn the **4-layer context stack** that transforms your AI-generated code from "works somewhere" to "works perfectly for your project."

---

## Why Context Matters: Generic vs. Tailored

Let's see the difference with a real example.

### Generic Prompt (No Context)

```
Create a function to validate email addresses
```

What does the AI generate? It might output code that:
- Works in JavaScript because that's its training default
- Uses a basic regex pattern (may not match your project's standards)
- Returns a value in any format the AI decides
- Has no type hints (if you're using Python)
- Doesn't follow your project's style or conventions

**Problem**: The AI had to guess your project, language, and requirements. Most guesses are wrong.

### Contextual Prompt (With 4-Layer Context)

```
**PROJECT CONTEXT**:
- Project: SaaS analytics platform backend
- Tech Stack: Python 3.11, FastAPI 0.104, PostgreSQL

**CODE CONTEXT**:
- File: src/utils/validators.py
- Related files use these validators

**CONSTRAINT CONTEXT**:
- Must use type hints
- Return boolean (True/False)
- Follow PEP 8

**DEVELOPER CONTEXT**:
- Team prefers explicit over clever

Create a function to validate email addresses
```

What does the AI generate? It produces:
- Python 3.11 code (because you specified it)
- With type hints and docstrings (because you required it)
- Matching your project's style (because you showed examples)
- Production-ready and integration-ready (because you provided constraints)

**Result**: The AI has the complete picture. It generates code that fits *your* project, not some generic template.

**The difference?**
- Generic: 30 minutes of modification to make it fit
- Contextual: Copy-paste ready, maybe 2 minutes to integrate

---

## WHAT: The 4-Layer Context Stack

The AIDD Context Stack has four layers, each answering a critical question the AI needs answered:

### Layer 1: Project Context
**The Question**: "What is this project, and what does it use?"

**What to Include**:
- Project name and purpose
- Technology stack (languages, frameworks, databases)
- Architecture style (monolithic vs. microservices, REST vs. GraphQL)
- Current development phase (MVP, scaling, maintenance)

**Example**:
```
Project: E-commerce platform backend
Tech Stack: Python 3.11, FastAPI 0.104, PostgreSQL 15, SQLAlchemy 2.0
Architecture: Monolithic REST API (planning microservices transition later)
Current Phase: MVP development, focusing on user authentication and product catalog
```

**Why it matters**: Tells AI what tools and patterns your project uses, so it generates matching code.

---

### Layer 2: Code Context
**The Question**: "Where are we building, and what's the current state?"

**What to Include**:
- The file you're working on (full path)
- Related files or modules that interact with this code
- Current data models or database schema
- Any known issues or problems you're solving

**Example**:
```
Working File: /src/auth/services.py
Related Files:
  - /src/auth/models.py (User model definition)
  - /src/auth/schemas.py (Pydantic validation schemas)
  - /src/database.py (database session management)

Database:
  - Users table: id, email, hashed_password, created_at, is_active

Current Issue: Token refresh endpoint returns 401 errors inconsistently
```

**Why it matters**: Tells AI about your current project state so it generates code that integrates seamlessly with what you already have.

---

### Layer 3: Constraint Context
**The Question**: "What rules must the code follow?"

**What to Include**:
- Coding standards (PEP 8, specific style guides)
- Type hints and documentation requirements
- Security constraints (input validation, authentication methods)
- Performance requirements (response time, resource limits)
- Testing requirements (minimum coverage, test framework)

**Example**:
```
Coding Standards:
  - Follow PEP 8 style guidelines
  - All functions require type hints
  - Include docstrings in Google format

Security Requirements:
  - All user inputs must be validated/sanitized
  - Use parameterized queries (never raw SQL)
  - Rate limiting: 100 requests/minute per IP
  - Passwords hashed with bcrypt (cost factor 12)

Performance:
  - API response time under 200ms

Testing:
  - Minimum 80% code coverage
  - Use pytest framework
```

**Why it matters**: Prevents AI from generating code that looks good but violates your project's standards or security policies.

---

### Layer 4: Developer Context
**The Question**: "Who's using this code, and what are their needs?"

**What to Include**:
- Your experience level with the technology
- Team preferences (explicit vs. clever code, detailed comments, etc.)
- Time constraints (how fast do you need this?)
- Integration preferences (copy-paste ready vs. needs modification)

**Example**:
```
Developer Context:
- Experience Level: Intermediate Python, learning FastAPI
- Preferences: Explicit over implicit code; comprehensive comments
- Time Constraint: Need working prototype by Friday
- Integration: Must work with existing authentication middleware
```

**Why it matters**: Helps AI tailor explanations and code complexity to match your needs. A senior engineer and a junior developer need different solutions.

---

## WHY: How Context Eliminates Back-and-Forth Iteration

Without context, you end up in **the iteration trap**:

```
You: "Create an API endpoint"
AI: [Generates generic endpoint]
You: "Wait, it needs to use PostgreSQL"
AI: [Modifies for SQL, but ignores type hints]
You: "And add type hints"
AI: [Adds types, but now queries are wrong]
You: "Why is authentication missing?"
AI: [Adds auth, but breaks existing tests]

[... 5-6 more iterations ...]

You (frustrated): "Just make it work!"
```

**Cost**: 30-45 minutes of back-and-forth for code that should have taken 5 minutes.

---

With **4-layer context**, you provide everything upfront:

```
You: [Provide complete 4-layer context + single clear command]
AI: [Generates production-ready code on first try]
You: "Great, integrating now"

[... actual work begins immediately ...]
```

**Cost**: 2-3 minutes of reading and integration. That's it.

---

## HOW: Building Your 4-Layer Context

Let's build a complete, realistic example. Imagine you're working on an **e-commerce platform** and need a user registration endpoint.

### Step 1: Define Your 4 Layers

**LAYER 1: PROJECT CONTEXT**
```
Project: E-commerce Platform Backend
Tech Stack: Python 3.11, FastAPI 0.104, PostgreSQL 15, SQLAlchemy 2.0
Architecture: Monolithic REST API
Current Phase: MVP - User authentication and account management
```

**LAYER 2: CODE CONTEXT**
```
File: /src/api/routes/auth.py
Related Files:
  - /src/models/user.py (User SQLAlchemy model)
  - /src/schemas/user.py (Pydantic schemas for validation)
  - /src/database.py (Session management)

Database Schema:
  Users table:
    - id (Integer, primary key)
    - email (String, unique)
    - hashed_password (String)
    - full_name (String)
    - created_at (DateTime)
    - is_active (Boolean)
```

**LAYER 3: CONSTRAINT CONTEXT**
```
Code Standards:
  - Python type hints required (validate with mypy)
  - Docstrings in Google format
  - PEP 8 style guide

Security:
  - Email validation with regex
  - Password hashing with bcrypt (12 rounds)
  - No passwords returned in responses
  - Input validation on all fields

Testing:
  - Minimum 80% code coverage
  - Use pytest
```

**LAYER 4: DEVELOPER CONTEXT**
```
Experience: Intermediate Python, learning FastAPI
Preferences:
  - Explicit error messages
  - Comprehensive inline comments
  - Show all imports and full functions
Time: Need working prototype by Friday
Integration: Must work with existing JWT middleware at /src/auth/middleware.py
```

### Step 2: Combine All Layers Into Your Prompt

Here's what your complete prompt looks like:

```
**COMMAND**: Create a user registration endpoint

**PROJECT CONTEXT**:
Project: E-commerce Platform Backend
Tech Stack: Python 3.11, FastAPI 0.104, PostgreSQL 15, SQLAlchemy 2.0
Architecture: Monolithic REST API
Current Phase: MVP - User authentication

**CODE CONTEXT**:
File: /src/api/routes/auth.py
Related Files:
  - /src/models/user.py
  - /src/schemas/user.py
  - /src/database.py

Database:
  Users table: id, email, hashed_password, full_name, created_at, is_active

**CONSTRAINT CONTEXT**:
Code Standards:
  - Type hints required
  - Docstrings (Google format)
  - PEP 8

Security:
  - Email regex validation
  - bcrypt hashing (12 rounds)
  - No passwords in responses

Testing:
  - 80% coverage minimum
  - pytest framework

**DEVELOPER CONTEXT**:
Experience: Intermediate Python, learning FastAPI
Preferences: Explicit code with inline comments
Time: Prototype needed by Friday
Integration: Works with JWT middleware at /src/auth/middleware.py

Requirements:
1. Validate email format
2. Check email isn't already registered
3. Hash password securely
4. Create user in database
5. Return user data (no password)
6. Handle errors appropriately
```

The AI now has the **complete picture**. It will generate code that:
- Uses Python 3.11 and FastAPI specifically
- Includes proper type hints and docstrings
- Follows your security requirements
- Integrates with your existing authentication
- Matches your coding standards
- Is ready to use immediately

---

## PRACTICE: Building Context Layers

Now it's your turn. Let's do three exercises to practice using the 4-layer context stack.

### Exercise 1: Identify Missing Context Layers (5 minutes)

Below are three prompts. For each one, **identify which context layers are missing** and explain why the missing layer matters.

**Prompt A**:
```
Create a function to calculate the total price of items in a shopping cart
```

**Prompt B**:
```
Create a function to calculate the total price of items in a shopping cart.

Project: E-commerce platform
Tech Stack: Python 3.11, FastAPI, PostgreSQL
File: /src/services/cart.py
```

**Prompt C**:
```
Create a function to calculate the total price of items in a shopping cart.

Project: E-commerce platform
Tech Stack: Python 3.11, FastAPI, PostgreSQL
File: /src/services/cart.py
Constraints: Type hints required, handle taxes, work with USD and EUR currencies
Developer: Intermediate Python, need working code by Thursday
```

**Your Task**: Which layers does each prompt have? What's missing?

**Solution**:

**Prompt A**: Missing ALL 4 layers
- Missing Project Context (What tech stack?)
- Missing Code Context (Where in the codebase?)
- Missing Constraint Context (Handling taxes? Multiple currencies?)
- Missing Developer Context (Experience level? Time deadline?)

**Prompt B**: Has Project + Code Context, missing Constraints + Developer
- ✓ Knows the tech stack and file location
- ✗ Doesn't know about taxes, currencies, or code standards
- ✗ Doesn't know your experience or deadline

**Prompt C**: Has all 4 layers
- ✓ Project context (tech stack)
- ✓ Code context (file location)
- ✓ Constraint context (type hints, taxes, currencies)
- ✓ Developer context (experience, deadline)

**Why it matters**: Prompt C is specific enough that AI can generate exactly what you need. Prompts A and B are vague enough that AI will have to guess.

---

### Exercise 2: Add Context to a Basic Prompt (10 minutes)

Start with this basic prompt:

```
Create a user authentication function
```

Your task: **Expand this into a complete 4-layer context prompt** using a real or hypothetical project.

You can either:
- Use a **real project you're working on** (give accurate details)
- Use **the example e-commerce project** from earlier (adapt the context)

Build all four layers:

1. **PROJECT CONTEXT**: What's the project, tech stack, and purpose?
2. **CODE CONTEXT**: Where in your codebase? What files exist?
3. **CONSTRAINT CONTEXT**: What rules must it follow? (Security, testing, standards)
4. **DEVELOPER CONTEXT**: Your experience, preferences, time constraints

**Success Criteria**:
- All 4 layers are present
- Each layer has specific, concrete details (not generic placeholders)
- Together, they answer "What, where, how, and why"

**Hint**: Use the e-commerce example as a template if you don't have a real project. Just substitute your own project name, tech stack, and constraints.

Once you've written your contextual prompt, you'll test it with AI in the next section.

---

### Exercise 3: Use Existing Code as Context (5 minutes)

This is the secret weapon for **style matching**. When you show AI an example of existing code in your project, it learns your style and generates matching code.

Here's an existing code example from a hypothetical project:

```python
class ProductService:
    """Service for managing product operations."""

    def __init__(self, db: Session):
        """Initialize with database session.

        Args:
            db: SQLAlchemy database session
        """
        self.db = db

    async def get_product(self, product_id: int) -> ProductSchema:
        """Retrieve a product by ID.

        Args:
            product_id: The product's unique identifier

        Returns:
            ProductSchema: The product data

        Raises:
            HTTPException: If product not found
        """
        product = self.db.query(Product).filter(
            Product.id == product_id
        ).first()

        if not product:
            raise HTTPException(status_code=404, detail="Product not found")

        return product
```

**Notice the style**:
- Classes use docstrings at class and method level
- Type hints on all parameters and returns
- Explicit error handling with HTTPException
- Async methods
- Detailed Args/Returns/Raises sections

Now write a prompt that says:

```
**COMMAND**: Create a UserService class following the same pattern as ProductService

**EXAMPLE**:
[Paste the ProductService code here]

Create a similar UserService for managing user operations with methods:
- get_user(user_id)
- create_user(user_data)
- delete_user(user_id)

Follow the exact style, docstring format, and error handling from the example.
```

**What happens**: AI sees the example and generates a UserService with:
- Identical docstring style
- Same type hints format
- Same error handling patterns
- Same method signatures and structure

**Success Criteria**:
- AI-generated code matches the example's style
- Docstrings follow the same format
- Error handling is consistent
- You don't need to reformat anything

---

## Try With AI: Seeing Context in Action

Now let's test the power of context with your AI agent.

### Setup

Use **Claude Code, Gemini CLI, or ChatGPT** (whichever you have access to). You'll run **two prompts** side-by-side to see the dramatic difference context makes.

### Test 1: Generic Prompt (No Context)

Use this prompt exactly:

```
Create a function to validate email addresses
```

**Observe**:
- What language did AI choose?
- Does it include type hints?
- What validation method does it use?
- Is it styled to match any particular project?

**Expected Output** (generic):
- Probably Python or JavaScript (random choice)
- May or may not include type hints
- Basic regex pattern (generic)
- No docstring or project-specific styling

**Time to integrate**: 10-15 minutes (need to modify for your project)

---

### Test 2: Contextual Prompt (With 4-Layer Context)

Copy this prompt and **modify the context sections** to match your real project (or use the e-commerce example):

```
**COMMAND**: Create a function to validate email addresses

**PROJECT CONTEXT**:
Project: [Your project name or "E-commerce Platform Backend"]
Tech Stack: Python 3.11, FastAPI 0.104, PostgreSQL
Architecture: Monolithic REST API
Current Phase: User authentication MVP

**CODE CONTEXT**:
File: /src/utils/validators.py
Related: /src/auth/routes.py imports and uses this validator
Database: Users table with email field (unique constraint)

**CONSTRAINT CONTEXT**:
- Use Python 3.11 type hints
- Return boolean (True if valid, False if not)
- Include docstring in Google format
- Use regex pattern (no complex validation)
- Follow PEP 8 naming conventions

**DEVELOPER CONTEXT**:
Experience Level: Intermediate Python, learning FastAPI
Preferences: Prefer explicit code with detailed comments
Time Constraint: Need working code today

**Requirement**:
Validate email addresses for user registration. Return True if valid, False otherwise.
```

**Observe**:
- Is it Python 3.11 with type hints?
- Does it match your specified constraints?
- Is it styled for your project?
- Can you copy-paste it directly?

**Expected Output** (specific):
- Python 3.11 code with type hints ✓
- Docstring in Google format ✓
- Clear regex pattern with comments ✓
- Returns boolean as specified ✓

**Time to integrate**: 1-2 minutes (copy-paste ready)

---

### Test 3: Your Exercise 2 Prompt

Run the **contextual prompt you built in Exercise 2** with your AI agent.

If you built it for the user authentication function:
1. Copy your complete 4-layer context prompt into your AI tool
2. Run it
3. **Compare the output to what you got from Test 1 (generic prompt)**

**Reflection Questions**:
- How much more specific is this AI output?
- Could you use it immediately in your project, or does it need modification?
- What aspects match your project (language, framework, style)?
- Which context layer was most important for that match?

---

## Key Takeaways

- **4-Layer Context Stack** provides AI with the complete blueprint:
  - Layer 1: Project context (tech stack, architecture)
  - Layer 2: Code context (file location, existing code)
  - Layer 3: Constraint context (standards, security, performance)
  - Layer 4: Developer context (experience, preferences, timeline)

- **Context transforms generic code into project-specific code** — the difference between "works somewhere" and "ready to integrate"

- **More context = less iteration** — upfront effort (3-5 minutes of context) saves 20-30 minutes of back-and-forth modification

- **AI can't read your mind** — provide the complete project picture so AI doesn't have to guess

- **Existing code examples help AI match your style** — show a pattern once, AI replicates it perfectly

- **Context prevents the iteration trap** — generic prompts lead to 5-6 cycles of modification; contextual prompts are usually right the first time

---

## Try With AI

In this final section, you'll use your AI agent to confirm everything you've learned about context.

**Your AI Tool**: Use Claude Code, Gemini CLI, ChatGPT web, or your preferred AI companion. (If you've already set up Claude Code or Gemini CLI from previous lessons, use those. Otherwise, ChatGPT web works perfectly.)

### Prompt Set: Context Creates Specificity

**Prompt 1 — The Generic Baseline**

Run this first to establish what "no context" produces:

```
Create a function to validate email addresses
```

Copy the AI's response. Note:
- What language it chose
- Whether it includes type hints
- The validation approach used
- Time you estimate to integrate into a project

**Prompt 2 — Adding Minimal Context**

Run this next:

```
Create a Python function to validate email addresses with type hints and a docstring
```

Compare to Prompt 1:
- More specific direction?
- Closer to project-ready?
- Still generic, or starting to match your needs?

**Prompt 3 — Full 4-Layer Context** (Use your Exercise 2 prompt)

Use the **contextual prompt you built and refined in Exercise 2**. This should include all four layers (project, code, constraints, developer).

Compare to Prompts 1 and 2:
- How much more specific is the output?
- Could you integrate it immediately?
- Does it match your project's style and standards?
- How many adjustments would you need to make?

### Expected Outcomes

**After running all three prompts**:
- Prompt 1 output: Generic, requires 10-15 minutes adaptation
- Prompt 2 output: Better, requires 5-10 minutes adaptation
- Prompt 3 output: Project-specific, requires 1-2 minutes integration

**Your Key Insight**: The difference between "helps a little" and "production-ready" is not complex. It's just **providing complete context upfront**.

### Safety & Verification Note

As you review each AI output:

- **Read the code first** — don't run code you don't understand
- **Check for secrets** — no hardcoded passwords or API keys
- **Verify against your constraints** — does it match what you specified?
- **Plan how you'll integrate it** — can you copy-paste it, or does it need modification?

Remember: **Trust but verify.** AI can hallucinate or miss requirements. Your 4-layer context helps prevent this, but always validate.

### Next Steps After This Exercise

You've now mastered three critical AI prompting skills:
1. **Strong commands** (Lesson 2) — Tell AI clearly WHAT to do
2. **4-Layer context** (Lesson 3) — Give AI the complete project picture
3. Coming next: **Implementation logic** (Lesson 4) — Tell AI HOW to build it

These three foundations will handle 80% of your AI-driven development work. The remaining lessons add advanced techniques for complex features, validation, and deployment-ready code.

Keep your best prompt templates. Starting in Lesson 8, you'll organize them into a reusable library that accelerates all your future projects.
