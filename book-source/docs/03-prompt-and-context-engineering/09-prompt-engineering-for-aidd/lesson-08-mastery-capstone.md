---
title: "Mastery Integration and Capstone — Systematize and Ship"
chapter: 9
lesson: 8
duration_minutes: 40

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Designing Reusable Prompt Templates"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student drafts 2+ reusable templates that include all 8 AIDD elements"

  - name: "Capstone Planning via QDD"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student uses QDD to elicit 10+ clarifying answers before implementation"

learning_objectives:
  - objective: "Systematize prompt engineering with templates for recurring tasks (feature, bug fix, refactor)"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Templates reviewed for presence of Command, Context, Logic, Role, Formatting, Constraints, Examples, Iterative Questions"

  - objective: "Plan and begin a portfolio-ready capstone using all 8 elements and validation checklist"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Capstone plan includes QDD answers, constraints, validation steps, and initial implementation"

cognitive_load:
  new_concepts: 0
  assessment: "Synthesis only (0 new concepts). Integrates all elements from Lessons 1–7 at B1 ✓"

differentiation:
  extension_for_advanced: "Create organization-wide templates; add CI gates for lint, typecheck, coverage"
  remedial_for_struggling: "Start with one template; use a smaller CLI capstone and expand gradually"
---

## Before We Build Templates: Anatomy of a Complete AIDD Prompt

Before you create your own templates, let's study a fully worked example showing all 8 elements working together. This is the "master prompt" that demonstrates professional AIDD in action.

Understanding this example will help you see how each element connects and why they matter.

---

### Complete Example: User Registration with Email Verification

This is the exact prompt a professional developer would write for a production feature. Every element serves a purpose.

```
**COMMAND:**
Implement a complete user registration system with email verification

**CONTEXT:**
Project: SaaS analytics platform backend
Tech Stack: Python 3.11, FastAPI 0.104, PostgreSQL 15, SQLAlchemy 2.0, Redis for cache
Architecture: Monolithic REST API (transitioning to microservices later)
Current State: Basic FastAPI app structure exists, database models for User table defined
Team: 2 backend developers, 1 frontend developer

**LOGIC:**
Implementation flow:
1. User submits registration form (email, password, name)
2. Validate email format and check uniqueness
3. Hash password using bcrypt (cost factor 12)
4. Generate verification token (UUID4) with 24-hour expiration
5. Store user in database with is_verified=False status
6. Send verification email via SendGrid API (existing email service)
7. User clicks verification link in email
8. API verifies token and updates is_verified=True
9. Return JWT access token upon successful verification

**ROLEPLAY:**
You are a senior backend engineer specializing in authentication systems and secure API design. You have deep expertise in FastAPI, SQLAlchemy, and modern Python async patterns. You prioritize security best practices (OWASP guidelines), proper error handling, and code that's easy to test and maintain.

**FORMATTING:**
Provide the implementation as:
1. File: src/auth/registration.py (complete file with all imports)
2. Pydantic schemas in: src/auth/schemas.py (just the new schemas)
3. Database model changes in: src/auth/models.py (additions only)
4. New API routes in: src/api/routes/auth.py (route definitions)
5. Include example curl commands for testing each endpoint

Format code with Black formatter standards, include comprehensive docstrings.

**CONSTRAINTS:**
Technical Requirements:
- Use FastAPI dependency injection for database sessions
- All endpoints must have proper type hints (validate with mypy)
- Password hashing: bcrypt with cost factor 12 (no argon2 or other libraries)
- Email service: Use existing EmailService class in src/services/email.py
- Database: Use existing session management (src/database/session.py)
- Token storage: Store verification tokens in Redis (TTL: 24 hours)
- Rate limiting: 3 registration attempts per hour per IP
- Validation: Use Pydantic models for request/response schemas
- Logging: Use existing structlog configuration

Security Requirements:
- No sensitive data in logs (passwords, tokens)
- Generic error messages (don't reveal if email exists)
- Input sanitization for all fields
- CORS configured for frontend domain only

**EXAMPLES:**
Here's how we structure other service classes in our codebase:

```python
class ProductService:
    def __init__(self, db: Session, cache: Redis):
        self.db = db
        self.cache = cache
    
    async def create_product(self, product_data: ProductCreate) -> ProductResponse:
        """Create new product with caching."""
        try:
            # Validate product data
            if await self._product_exists(product_data.name):
                raise HTTPException(status_code=409, detail="Product already exists")
            
            # Create product
            product = Product(**product_data.dict())
            self.db.add(product)
            await self.db.commit()
            await self.db.refresh(product)
            
            # Cache result
            await self.cache.set(f"product:{product.id}", product.dict(), ex=3600)
            
            return ProductResponse.from_orm(product)
            
        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Error creating product: {str(e)}")
            raise HTTPException(status_code=500, detail="Failed to create product")
```

Follow this same structure for the RegistrationService:
- Constructor with dependencies (db, cache/redis, email service)
- Explicit error handling with try/except
- Specific HTTPExceptions with appropriate status codes
- Generic error messages for security
- Logging for debugging (no sensitive data)
- Type hints throughout

**QUESTIONS:**
Before implementing, ask me 8 questions about:
1. Email template content and verification link URL structure
2. Error handling preferences (return errors vs raise exceptions)
3. Logging detail level (what should be logged at each step)
4. Testing approach (what tests do you want generated)
5. User duplicate handling (case-sensitive email? normalized email?)
6. Password policy beyond minimum length (complexity requirements?)
7. Token regeneration policy (can user request new verification email?)
8. Post-verification behavior (redirect user where? return what data?)
```

---

### Exercise: Map the 8 Elements (5 min)

Now that you've seen a complete prompt, identify where each of the 8 elements appears.

**Task**: Fill in this mapping:

| Element | Where in the Prompt | Why It Matters |
|---------|---------------------|----------------|
| 1. Command | "Implement a complete user registration system with email verification" | Clear, specific action |
| 2. Context (4-layer) | [You fill in] | [Why it matters] |
| 3. Logic | [You fill in] | [Why it matters] |
| 4. Roleplay | [You fill in] | [Why it matters] |
| 5. Formatting | [You fill in] | [Why it matters] |
| 6. Constraints | [You fill in] | [Why it matters] |
| 7. Examples | [You fill in] | [Why it matters] |
| 8. Iterative Questions | [You fill in] | [Why it matters] |

<details>
<summary>Answer Key</summary>

| Element | Where in the Prompt | Why It Matters |
|---------|---------------------|----------------|
| 1. Command | First line: "Implement a complete user registration system with email verification" | Sets clear direction and scope |
| 2. Context (4-layer) | CONTEXT section with project, tech stack, architecture, current state, team | AI understands your environment completely |
| 3. Logic | LOGIC section with 9 numbered implementation steps | AI follows your architectural design, not generic patterns |
| 4. Roleplay | ROLEPLAY section: "senior backend engineer specializing in authentication" | Gets expert-level responses with security best practices |
| 5. Formatting | FORMATTING section specifying 5 files with structure details | Output is ready to integrate, not generic code blocks |
| 6. Constraints | CONSTRAINTS section with technical and security requirements | Ensures code fits your stack and meets security standards |
| 7. Examples | EXAMPLES section with ProductService code showing style | AI matches your project's coding patterns |
| 8. Iterative Questions | QUESTIONS section requesting 8 specific clarifications | AI fills gaps before coding, reducing iterations |

**Key insight**: Each element eliminates a category of problems. Without constraints, AI might use wrong versions. Without examples, AI won't match your style. Without logic, AI guesses architecture. Together, they produce production-ready code.

</details>

---

### What Makes This Prompt Powerful?

**1. Completeness**: Every element is present, so AI has full context

**2. Specificity**: 
- Not "hash passwords" but "bcrypt with cost factor 12"
- Not "send email" but "via SendGrid API using existing EmailService"
- Not "store tokens" but "in Redis with 24-hour TTL"

**3. Integration Focus**:
- References existing code (EmailService, session management)
- Shows existing patterns (ProductService example)
- Specifies exact file locations

**4. Security Consciousness**:
- Multiple security constraints
- Example shows generic error messages
- Logging guidelines (no sensitive data)

**5. Question-Driven Approach**:
- Asks 8 specific questions before coding
- Addresses ambiguities upfront
- Reduces back-and-forth iterations

**The Result**: AI generates code that fits your project perfectly, follows your patterns, meets security standards, and integrates seamlessly—all from one well-structured prompt.

---

## What: Systematizing Prompt Engineering

Now that you've seen a complete prompt in action, let's systematize this approach into reusable templates.

You've learned all eight elements. Now you'll package them into templates so your future work is faster and more reliable. Templates prevent "forgetting an element," keep quality consistent, and make collaboration easier.

Common use cases:
- New feature or API endpoint
- Bug fix investigation and patch
- Refactoring for maintainability
- Optimization (performance or memory)
- Test generation and docs updates

## Why: Templates Save Time (and Errors)

Instead of reinventing your prompt each time, use a template that encodes the best practices you've already proven: clear command, rich context, explicit logic, constraints, examples, role, formatting, and QDD questions.

Study the complete example above—that's your model. Now we'll create templates for recurring scenarios.

## How: Build Prompt Templates (3 examples)

Each template includes all 8 elements: Command, Context (4-layer), Logic, Role, Formatting, Constraints, Examples, Iterative Questions.

### Template 1 — New API Endpoint
```
Command: Implement a new FastAPI endpoint: [path + method].

Context (4-layer):
- Project: [stack, architecture, phase]
- Code: [files, models, related modules]
- Constraints: [versions, security, quality, integration]
- Developer: [experience, preferences, deadline]

Logic (numbered steps):
1) [validation]
2) [business rule]
3) [DB call]
4) [response structure]

Role & Style: Senior Backend Engineer; pragmatic, defensive coding; no bare except; docstrings + type hints.

Formatting: Complete file with imports, Google-style docstrings, type hints, and tests.

Constraints: Python 3.11, FastAPI 0.104.0 (Annotated), SQLAlchemy 2.0, ≥80% coverage.

Examples: Error handling pattern and API response JSON format (provide snippets).

Iterative Questions: Ask 10 questions before coding about auth, data model, edge cases, performance, and testing.
```

### Template 2 — Bug Fix
```
Command: Debug [module.function] failing with [error + context].

Context: include logs, stack trace, minimal repro, related files.

Logic: Diagnose cause → propose fix → implement fix → add regression tests.

Role & Style: Defensive; add error handling and logging consistent with project.

Formatting: Provide diff-style changes plus updated tests.

Constraints: No API changes; maintain backward compatibility; follow style guide.

Examples: Include current error handling snippet and test naming conventions.

Iterative Questions: Ask about environment, versions, recent changes, and data edge cases.
```

### Template 3 — Refactor
```
Command: Refactor [module/function] to reduce complexity and improve readability without changing behavior.

Context: current code, usage sites, performance needs.

Logic: Identify duplication → extract helpers → simplify branches → keep public API stable.

Role & Style: Pragmatic; ensure small, reviewable commits; include docstrings and type hints.

Formatting: Provide complete refactored file and a summary of changes.

Constraints: Maintain tests green; cyclomatic complexity < 10; lint clean.

Examples: Provide a representative class or function from the codebase to mirror style.

Iterative Questions: Ask 10 clarifying questions before refactor.
```

---

## Capstone Project: Portfolio-Worthy Application

Choose one:
- REST API (e.g., feature-rich Task or E-commerce module)
- Data Processor (ETL or analytics pipeline)
- CLI Tool (automation or developer productivity)

Requirements:
- Demonstrate all 8 elements in your prompts
- Use QDD workflow (10+ questions answered)
- Apply the 5-step validation checklist
- Provide working code + tests + docs
- Include at least 2 templates adapted to your project

## Capstone Phase 1: Planning (10 min)

Task: Run QDD to define requirements before building.

Prompt to your AI collaborator:
```
I’m choosing the [REST API | Data Processor | CLI Tool] capstone.

Before proposing an implementation, ask me 10 detailed questions about architecture, security, performance, data model, constraints, and testing. Then, based on my answers, outline a phased plan and initial tasks mapped to my constraints.
```

Success criteria: Clear, answer-backed plan with explicit constraints and a first implementation slice you can start immediately ✓

---

## Capstone Phase 2: Implementation (25 min)

Task: Use your customized templates and all 8 AIDD elements to generate code and tests with your AI collaborator. Apply the 5-step validation checklist to every artifact.

Guidance:
- Drive with a clear Command and Logic steps; keep Constraints active in each prompt.
- Provide Examples (style, API responses, tests) to shape outputs.
- Maintain the Role and Style modifiers for consistency.
- After each AI output: Validate (Read → Secrets → Issues → Test → Compare to Spec) and iterate with targeted fix prompts.

Success criteria: Working code that meets your constraints, passes tests, and aligns with examples ✓

---

## Capstone Phase 3: Documentation (10 min)

Task: Document the project for portfolio use.

Checklist:
- README: problem statement, architecture overview, setup/run steps
- Prompts: include the final versions of your best-performing templates
- Validation: note key issues found and how they were fixed
- “Built with AI” note: explain how AI collaborated and where human validation mattered

Success criteria: Clear, portfolio-ready documentation that showcases your AI-native methodology ✓

---

### Measuring Your Prompting (Synthesis Checklist)

Use these lightweight metrics to evaluate your prompts—no new concepts, just how to judge effectiveness:

- First-Time Success Rate
  - Goal: ≥70% of AI outputs work on first try
  - Improve by adding context, examples, and constraints

- Iteration Count
  - Goal: 2–3 iterations to production-ready
  - Improve by using QDD earlier (ask questions first)

- Code Quality Metrics
  - Goal: Pass same checks as human code (lint, types, tests)
  - Improve by stating quality requirements in prompts

- Time to Completion
  - Goal: 3–5× faster than manual for suitable tasks
  - Improve by templating recurring work

- Maintenance Overhead
  - Goal: Fewer follow-up fixes vs manual code
  - Improve by emphasizing maintainability and documentation

Track these during your capstone to refine both prompts and templates.

---

## Try With AI

Goal: Put templates and QDD into action.

1) Pick one of the three templates above and customize it to your capstone.

2) Run a QDD round using the "ask 10 questions first" pattern. Answer thoroughly.

3) Generate the initial implementation and tests. Apply the 5-step validation checklist.

Outcome: You've systematized your prompting, kicked off a capstone with QDD, and validated code like a professional AI collaborator.

---

## Appendix: Tool-Specific Workflows for Claude Code and Gemini CLI

Throughout this chapter, you've learned universal prompting techniques that work with any AI coding agent. But Claude Code and Gemini CLI each have unique superpowers that require specialized workflows. This appendix shows you how to leverage tool-specific features for maximum productivity.

### Claude Code: Multi-File Intelligence and Project Context

Claude Code excels at understanding your entire project structure and coordinating changes across multiple files. Here's how to maximize its strengths.

---

#### **Workflow 1: Project Context Loading**

**Claude Code's superpower**: It can scan and understand your entire project structure before making changes.

**When to use**: Starting new features, large refactorings, or when AI needs to understand architectural patterns.

**Step-by-Step Workflow**:

```bash
# Step 1: Let Claude scan your project structure
"Analyze this project structure and give me a high-level overview:
- What's the architecture? (monolith, microservices, layered?)
- What patterns are used? (MVC, services, repositories?)
- What's the tech stack?
- What are the main modules and their responsibilities?
- What conventions do you notice? (file naming, folder structure, testing approach)"

# Step 2: Ask Claude to identify patterns before coding
"Now that you understand the project, identify the authentication patterns used across these files:
- src/auth/services.py
- src/middleware/auth.py
- tests/test_auth.py

List the patterns and conventions I should follow for new auth features."

# Step 3: Request work that follows discovered patterns
"Following the patterns you identified, implement a password reset feature that matches the existing auth architecture."
```

**Why this works**: Claude builds a mental model of your project first, then generates code that naturally fits your existing patterns.

---

#### **Workflow 2: Multi-File Coordinated Refactoring**

**Claude Code's superpower**: It can refactor across multiple related files while maintaining consistency.

**When to use**: Renaming classes/functions used across files, architectural changes, extracting shared logic.

**Step-by-Step Workflow**:

```bash
# Request coordinated changes
"Refactor the user authentication system across these files:

Files to modify:
- src/auth/services.py (UserAuthService class)
- src/auth/models.py (User model)
- src/api/routes/auth.py (auth endpoints)
- tests/test_auth.py (test suite)

Refactoring goal:
- Rename UserAuthService → AuthenticationService
- Extract token generation into separate TokenService class
- Update all imports and references
- Update tests to use new structure

Show me the changes for each file in order."
```

**Why this works**: Claude tracks dependencies and updates all affected files, preventing broken imports and inconsistencies.

---

#### **Workflow 3: Iterative Development Sessions**

**Claude Code's superpower**: It maintains context across multiple prompts in a session, allowing iterative refinement.

**When to use**: Complex features requiring multiple passes (design → implement → refine → test).

**Step-by-Step Workflow**:

```bash
# Round 1: Broad design
"I need to add a file upload feature for user profiles. Before implementing, ask me 10 questions about:
- File types allowed
- Storage location (local, S3, etc.)
- Size limits
- Security requirements
- Image processing needs"

# [Answer the questions]

# Round 2: Implementation
"Based on my answers, implement the file upload feature with:
1. FileUploadService class
2. FastAPI endpoint for uploads
3. Validation (file type, size)
4. Storage to S3
5. Error handling"

# Round 3: Refinement
"Good implementation. Now improve it:
- Add progress tracking for large uploads
- Implement thumbnail generation for images
- Add virus scanning placeholder
- Improve error messages to be more user-friendly"

# Round 4: Testing
"Generate comprehensive tests covering:
- Successful upload
- File type validation
- Size limit enforcement
- S3 storage errors
- Malicious file detection"
```

**Why this works**: Each round builds on previous context. Claude remembers your decisions from earlier in the session.

---

#### **Workflow 4: Using MCP Tools (Model Context Protocol)**

**Claude Code's superpower**: It can use MCP tools to search code, read files, and analyze patterns programmatically.

**When to use**: Finding code patterns, analyzing dependencies, identifying issues across large codebases.

**Step-by-Step Workflow**:

```bash
# Search for patterns
"Use the codebase search tool to find all database queries that might have N+1 problems. Look for patterns like:
- for loops containing db.query()
- list comprehensions with database calls
- async for loops with await db operations

List the files and line numbers where these patterns occur."

# Analyze file dependencies
"Read all files in src/services/ and create a dependency graph showing:
- Which services depend on which other services
- Which services call the database directly
- Which services have circular dependencies (potential issues)

Format as Mermaid diagram."

# Find security issues
"Scan the codebase for potential security vulnerabilities:
- Hardcoded credentials or API keys
- SQL queries using string concatenation (SQL injection risk)
- Missing input validation on API endpoints
- Use of insecure cryptographic functions

Report findings with file locations and severity."
```

**Why this works**: MCP tools give Claude programmatic access to your codebase for systematic analysis.

---

### Gemini CLI: Session Persistence and Multi-Language Flexibility

Gemini CLI excels at maintaining long-running sessions and converting code between languages. Here's how to leverage its strengths.

---

#### **Workflow 1: Session-Based Context Building**

**Gemini CLI's superpower**: Sessions persist context across terminal sessions, perfect for long-running projects.

**When to use**: Multi-day projects, complex features requiring multiple sessions, learning new frameworks.

**Step-by-Step Workflow**:

```bash
# Day 1: Start session and build context
gemini chat --session project-ecommerce "I'm building an e-commerce backend with Python FastAPI, PostgreSQL, and Redis. 

Current architecture:
- API routes in src/api/
- Business logic in src/services/
- Database models in src/models/
- Using SQLAlchemy 2.0 ORM
- JWT authentication already implemented

Today I'm working on the shopping cart feature. Remember this context for our session."

# Day 1 continued: Design phase
gemini chat --session project-ecommerce "Design the shopping cart system. Ask me 10 questions about requirements."

# [Answer questions, implement initial version]

# Day 2: Resume session (context persists!)
gemini chat --session project-ecommerce "Yesterday we built the shopping cart. Today let's add persistent cart storage using Redis. Update the CartService to:
1. Store cart state in Redis with user ID as key
2. Set 7-day TTL on cart data
3. Fall back to database if Redis fails
4. Sync cart to database on checkout"

# Day 3: Add tests (still in same session)
gemini chat --session project-ecommerce "Generate comprehensive tests for the cart system we built. Cover:
- Cart operations (add, remove, update)
- Redis persistence
- Database fallback
- Checkout synchronization"
```

**Why this works**: Session context accumulates over days. Gemini remembers your architecture decisions from previous days.

---

#### **Workflow 2: Code Generation with Precise Specifications**

**Gemini CLI's superpower**: Generates complete, production-ready files from detailed specifications.

**When to use**: Building new modules, generating boilerplate, creating API clients.

**Step-by-Step Workflow**:

```bash
# Generate complete module
gemini chat "Generate a complete Python FastAPI CRUD API module with these precise specs:

Resource: Products
Fields:
- id: int (primary key, auto-increment)
- name: str (max 200 chars, unique, required)
- description: str (max 2000 chars, optional)
- price: Decimal (2 decimal places, >= 0, required)
- stock: int (>= 0, required)
- category_id: int (foreign key to categories, required)
- created_at: datetime (auto-generated)
- updated_at: datetime (auto-updated)

Endpoints:
- GET /products → list all (with pagination: limit, offset)
- GET /products/{id} → get one (404 if not found)
- POST /products → create (validate all fields)
- PUT /products/{id} → update (partial updates allowed)
- DELETE /products/{id} → soft delete (set is_deleted=True)

Requirements:
- SQLAlchemy 2.0 models with type hints
- Pydantic schemas for request/response
- Dependency injection for database session
- Comprehensive error handling (400, 404, 422, 500)
- Logging for all operations
- Unit tests with 85%+ coverage

Output as complete, ready-to-use files:
1. models.py
2. schemas.py
3. routes.py
4. service.py
5. test_products.py"
```

**Why this works**: Gemini excels at generating complete implementations from detailed specs. One well-specified prompt → production-ready code.

---

#### **Workflow 3: Explanation and Documentation Generation**

**Gemini CLI's superpower**: Excellent at explaining code, generating docs, and identifying risks.

**When to use**: Code reviews, onboarding new developers, creating project documentation.

**Step-by-Step Workflow**:

```bash
# Explain complex code
gemini chat "Explain this Python code in detail:

[paste complex authentication middleware code]

Include:
1. High-level overview (what it does in 2-3 sentences)
2. Step-by-step walkthrough of the logic flow
3. Why each part is needed (purpose of decorators, dependencies, etc.)
4. Potential edge cases or issues
5. Performance considerations
6. Suggestions for improvement

Make the explanation beginner-friendly but technically accurate."

# Generate comprehensive docs
gemini chat "Generate documentation for this user authentication module:

[paste module code]

Create:
1. README.md with module overview, features, and quick start
2. API documentation for each function (parameters, returns, examples)
3. Architecture diagram in Mermaid format
4. Security considerations section
5. Troubleshooting guide for common issues
6. Testing guide with example test cases

Format everything as Markdown ready to commit to the repo."

# Risk analysis
gemini chat "Analyze this code for risks and issues:

[paste code]

Categorize findings:
- Security vulnerabilities (with severity: critical, high, medium, low)
- Performance bottlenecks (with estimated impact)
- Code quality issues (maintainability, readability)
- Missing error handling (where it could fail)
- Testing gaps (what's not covered)

For each issue, provide:
1. Description of the problem
2. Why it's a problem
3. Recommended fix (with code example if relevant)
4. Priority (must-fix, should-fix, nice-to-have)"
```

**Why this works**: Gemini is trained on extensive documentation and can generate comprehensive, accurate explanations.

---

#### **Workflow 4: Multi-Language Code Conversion**

**Gemini CLI's superpower**: Converts code between languages while preserving functionality and idioms.

**When to use**: Migrating services, building polyglot systems, learning new languages.

**Step-by-Step Workflow**:

```bash
# Python → TypeScript conversion
gemini chat "Convert this Python FastAPI code to TypeScript with Express:

[paste Python code]

Requirements:
- Use TypeScript with strict type checking
- Follow Express best practices
- Convert Pydantic models to Zod schemas
- Preserve same API contracts (endpoints, request/response formats)
- Include equivalent error handling
- Add JSDoc comments equivalent to Python docstrings
- Include package.json with necessary dependencies"

# Generate matching frontend client
gemini chat "Now generate a TypeScript React API client that calls the endpoints we just converted. 

Include:
- Axios for HTTP requests
- TypeScript types matching the API schemas
- Error handling with custom error classes
- Retry logic with exponential backoff
- Request/response interceptors for auth tokens
- React hooks for common operations (useProducts, useProduct, useCreateProduct)
- Example component showing usage"

# Synchronize changes
gemini chat "I updated the Python backend to add a 'featured' boolean field to products. Update the TypeScript Express code and React client to match:
- Add field to database model and schema
- Update endpoints to support filtering by featured=true
- Update TypeScript types
- Add UI filter in React client"
```

**Why this works**: Gemini understands idioms and best practices across languages, producing natural code in each.

---

### Exercise: Choose Your Tool Workflow (15 min)

Pick ONE workflow from above and implement it end-to-end with your capstone project.

**Option A: Claude Code Multi-File Refactoring**
- Identify 3-4 related files in your project
- Use Claude to refactor a class or function used across all files
- Verify all imports and references are updated correctly

**Option B: Gemini CLI Session-Based Development**
- Start a named session for your capstone
- Build a feature over 3+ prompts (design → implement → test)
- Resume the session after closing terminal to verify context persists

**Option C: Code Conversion**
- Take a Python module from your project
- Convert it to TypeScript using Gemini
- Generate a frontend client that uses the converted API

**Success criteria**:
- ✅ You successfully used a tool-specific workflow
- ✅ The workflow saved time vs. manual approach
- ✅ You understand when to use this workflow in future projects
- ✅ Output quality matches or exceeds manual coding

---

## Conclusion: Your AIDD Mastery Toolkit

You've now completed the full AIDD prompting framework:

**Lesson 1**: Understanding AI agents and context windows
**Lesson 2**: Writing clear commands with technical action verbs
**Lesson 3**: Providing 4-layer context (project, code, constraints, developer)
**Lesson 4**: Specifying logic with implementation steps and design patterns
**Lesson 5**: Validating AI output with 5-step checklist
**Lesson 6**: Adding constraints, examples, and formatting instructions
**Lesson 7**: Using QDD and roleplay for expert-level responses
**Lesson 8**: Systematizing with templates and tool-specific workflows

**Your AIDD skillset**:
- ✅ Build production-ready code with AI partners
- ✅ Think architecturally and communicate designs clearly
- ✅ Validate and refine AI outputs professionally
- ✅ Work 3-5× faster than manual coding for appropriate tasks
- ✅ Leverage tool-specific superpowers (Claude's multi-file intelligence, Gemini's session persistence)

**Next steps**:
1. Complete your capstone project using all 8 elements
2. Build your prompt template library for recurring tasks
3. Track your metrics (first-time success rate, iteration count)
4. Share your prompting approach with your team
5. Apply these techniques in the next chapters as you build real applications

You're ready to develop software at AI-native speed. Let's build.
