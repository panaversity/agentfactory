---
title: "Question-Driven Development (QDD) and Roleplay — Get Expert, Tailored Code"
chapter: 9
lesson: 7
duration_minutes: 50

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Practicing QDD (Iterative Clarifying Questions)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student prompts AI to ask 10 questions before complex implementation; answers produce better code"

  - name: "Adopting Specialized AI Roles"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student sets role (e.g., Senior Backend Engineer) and style modifiers; receives domain-grade responses"

learning_objectives:
  - objective: "Use QDD to replace guesswork with targeted clarifying questions before coding"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "AI asks ≥10 relevant questions; final output reflects answers"

  - objective: "Guide AI with specialized roles and style modifiers for higher-quality implementations"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Roleplay prompt yields expert, pragmatic code with defensive practices"

cognitive_load:
  new_concepts: 3
  assessment: "3 new concepts (QDD workflow, AI roleplay, AI as consultant) match CEFR B1 ✓"

differentiation:
  extension_for_advanced: "Chain QDD rounds (architecture → testing → observability); compare outputs across roles"
  remedial_for_struggling: "Provide a question bank starter; use a simpler feature (e.g., notes API)"
---

## What: Question-Driven Development (QDD)

QDD is the most powerful AIDD technique for complex tasks. Instead of the AI guessing your requirements, you explicitly ask it to question you first—then it generates code tailored to your answers.

Why it works: AI excels at structured inquiry. Ten minutes answering good questions can save hours of debugging a misaligned solution.

## Why: QDD Produces Better Code

Without QDD → generic patterns, wrong assumptions, mismatched integrations.
With QDD → specificity about auth method, data model, error handling, testing, performance, and deployment—before code is generated.

## How: The QDD Process

Follow these four steps (use the authentication example from your context):

1) Initial Prompt with Question Request
```
I need to implement a user authentication system for a FastAPI application.

Before you provide an implementation, ask me 10 detailed technical questions that will help you create the most appropriate solution for my specific needs.
```

2) AI Asks Clarifying Questions
Expect questions about JWT vs OAuth2, DB/ORM, RBAC, token lifetimes, email verification, rate limiting, testing strategy, etc.

3) Provide Detailed Answers
Answer concretely (e.g., PostgreSQL 15 + SQLAlchemy 2.0, JWT with refresh tokens, RBAC roles, coverage goals).

4) AI Generates Tailored Solution
Now the implementation fits your decisions instead of a generic tutorial.

### Advanced QDD Types

- Architecture Review Questions: scalability, security, performance, maintainability, missing features; “Be specific about which files or functions.”
- Debugging Question Process: environment, traffic, connection settings, monitoring, recent changes; ask in priority order.
- Optimization Questions: workload, metrics, caching, expected patterns.

## Roleplay: Specialized Technical Expertise

This is where AI prompting becomes truly powerful. Instead of getting "generic developer" responses, you can guide your AI collaborator to think and respond as a specialized expert with deep domain knowledge.

Think of it this way: If you need to build a secure authentication system, would you rather consult a "generic developer" or a **senior backend engineer with 10 years of experience in authentication, OAuth2, and security best practices**? The specialist gives you expert-level patterns, catches security holes, and suggests industry-standard approaches.

Your AI collaborator works exactly the same way. By setting a specialized role, you transform generic responses into domain-expert guidance.

### Why Roleplay Changes Everything

Without roleplay, AI defaults to general patterns. With roleplay, AI activates specialized knowledge from its training:

- **Security best practices** (OWASP guidelines, authentication patterns)
- **Performance optimization techniques** (caching strategies, database indexing)
- **Design patterns for specific problems** (Strategy for payment methods, Observer for events)
- **Domain-specific anti-patterns to avoid** (N+1 queries, bare except blocks, mutable defaults)
- **Testing strategies** (unit tests, integration tests, test data patterns)

### Common AIDD Roles

Here are the four most valuable roles for AI-driven development, with specific expertise areas:

#### 1. **Senior Backend Engineer**
**Specialization**: Python/FastAPI, authentication systems, database optimization, microservices architecture, API design

**When to use**: Building REST APIs, authentication/authorization, database schema design, service architecture

**Example prompt**:
```
You are a senior backend engineer with 10 years of experience building production FastAPI applications. You specialize in authentication systems, database optimization, and secure API design. You follow OWASP security guidelines and write defensive code that handles edge cases explicitly.

Implement a user registration endpoint with email verification...
```

**What this gets you**:
- Proper password hashing (bcrypt with appropriate cost factor)
- Input validation and sanitization
- Explicit error handling (no bare except)
- Security considerations (rate limiting, email validation)
- Database transaction safety
- Type hints and comprehensive docstrings

---

#### 2. **Expert Frontend Developer**
**Specialization**: React/TypeScript, accessibility (WCAG), performance optimization, state management, responsive design

**When to use**: Building UI components, forms, accessibility features, client-side state management

**Example prompt**:
```
You are an expert frontend developer specializing in React and TypeScript. You prioritize accessibility (WCAG 2.1 AA compliance), performance (lazy loading, code splitting), and maintainable component design. You always include proper ARIA labels and keyboard navigation.

Create a user registration form component...
```

**What this gets you**:
- Proper TypeScript types and interfaces
- WCAG-compliant markup with ARIA labels
- Form validation with user-friendly error messages
- Loading states and error boundaries
- Mobile-responsive design considerations
- Performance optimizations (useMemo, useCallback where appropriate)

---

#### 3. **DevOps Engineer**
**Specialization**: Docker/Kubernetes, CI/CD pipelines, infrastructure as code (Terraform), monitoring, deployment automation

**When to use**: Containerization, deployment pipelines, infrastructure setup, monitoring configuration

**Example prompt**:
```
You are a DevOps engineer with expertise in Docker, Kubernetes, and GitHub Actions. You prioritize reproducible builds, security scanning, and zero-downtime deployments. You follow the principle of infrastructure as code.

Create a Dockerfile and deployment pipeline for a FastAPI application...
```

**What this gets you**:
- Multi-stage Docker builds (smaller images, security)
- Health checks and readiness probes
- CI/CD pipeline with proper test stages
- Security scanning (dependency vulnerabilities)
- Secrets management best practices
- Deployment strategies (blue-green, canary)

---

#### 4. **Data Scientist**
**Specialization**: pandas/NumPy, scikit-learn, TensorFlow/PyTorch, MLOps, data pipelines, statistical analysis

**When to use**: Data processing, machine learning models, data analysis, ETL pipelines

**Example prompt**:
```
You are a data scientist with expertise in Python data analysis, machine learning pipelines, and MLOps. You prioritize reproducibility, proper train/test splits, and production-ready model deployment. You follow best practices for data validation and monitoring.

Build a classification pipeline for customer churn prediction...
```

**What this gets you**:
- Proper data preprocessing and feature engineering
- Train/test/validation splits
- Model evaluation metrics appropriate to the problem
- Cross-validation strategies
- Pipeline serialization for production deployment
- Data validation and monitoring considerations

---

### Style Modifiers: Fine-Tuning the Expert

After setting the role, add style modifiers to shape *how* the expert responds:

**Pragmatic vs. Perfectionist**:
```
Be pragmatic; prioritize working, maintainable code over perfect code.
Deliver production-ready solutions that work today.
```

**Defensive Coding**:
```
Write defensive code that anticipates edge cases and errors.
Include explicit error handling; never use bare except blocks.
Add input validation for all user-provided data.
```

**Anti-Pattern Awareness**:
```
Avoid common Python anti-patterns:
- No bare except (catch specific exceptions)
- No mutable default arguments
- Use context managers for resources (with statements)
- Avoid global state
- Prefer explicit over implicit
```

**Testing Emphasis**:
```
Include comprehensive tests with every implementation.
Cover happy path, edge cases, and error conditions.
Use descriptive test names that explain what's being tested.
```

---

### The Power of Roleplay: Side-by-Side Comparison

Let's see the dramatic difference roleplay makes with a real example.

#### Scenario: Build a Login Endpoint

**Prompt A (Generic - No Role)**:
```
Create a login endpoint for a FastAPI application.
```

**Prompt B (Specialized - Senior Backend Engineer)**:
```
You are a senior backend engineer with 10 years of experience building secure authentication systems for production FastAPI applications. You follow OWASP security guidelines, implement rate limiting, use proper password hashing (bcrypt), and write defensive code with comprehensive error handling.

Create a login endpoint for a FastAPI application with JWT token generation.
```

### Style Modifier Examples

Combine role with style modifiers for even more precise outputs:

**Example 1: Pragmatic + Defensive**
```
You are a senior backend engineer specializing in FastAPI and authentication.

Style:
- Be pragmatic: prioritize working, maintainable code over perfect code
- Write defensive code that anticipates edge cases
- Include explicit error handling; never use bare except
- Add comprehensive logging for debugging
- Keep it simple: avoid over-engineering

[Your task...]
```

**Example 2: Performance-Focused**
```
You are a senior backend engineer with expertise in high-performance systems.

Style:
- Prioritize performance and scalability
- Consider caching strategies
- Minimize database queries (avoid N+1)
- Use async/await for I/O operations
- Include performance considerations in comments

[Your task...]
```

**Example 3: Test-First**
```
You are a senior backend engineer who practices test-driven development.

Style:
- Generate comprehensive tests before implementation
- Cover happy path, edge cases, and error conditions
- Use descriptive test names
- Include setup/teardown for test data
- Aim for 80%+ code coverage

[Your task...]
```

---

## Exercise 1: Practice QDD Workflow (20 min)

Task: Choose a complex feature (e.g., payment processing). Ask the AI to interview you first with 10+ questions. Answer them, then request the implementation.

Success criteria: AI’s final code clearly reflects your answers (auth method, data schema, error handling, tests, constraints) ✓

---

## Exercise 2: Experience the Roleplay Difference (15 min)

This exercise demonstrates the dramatic impact of roleplay through direct comparison.

**Task**: You'll request the same feature twice—once without roleplay (generic), once with roleplay (specialized)—and compare the outputs.

### Part A: Generic Request (5 min)

Run this prompt:
```
Create a password reset endpoint for a FastAPI application. The endpoint should accept an email address and send a reset link.
```

**Observe**: What does the AI generate? Look for:
- Security considerations
- Error handling depth
- Input validation
- Logging and monitoring
- Code quality (type hints, docstrings)

### Part B: Specialized Request (5 min)

Now run this roleplay prompt:
```
You are a senior backend engineer with 10 years of experience building secure authentication systems. You follow OWASP security guidelines, implement rate limiting, and write defensive code with comprehensive error handling.

Style:
- Prioritize security and data protection
- Include explicit error handling
- Add logging for audit trails
- Use type hints and docstrings
- Validate all inputs

Create a password reset endpoint for a FastAPI application. The endpoint should accept an email address and send a reset link.
```

**Observe**: How does this output differ from Part A?

### Part C: Compare and Analyze (5 min)

Create a comparison table:

| Aspect | Generic (Part A) | Specialized (Part B) |
|--------|------------------|----------------------|
| Input validation | | |
| Security measures | | |
| Error handling | | |
| Rate limiting | | |
| Logging | | |
| Code quality | | |
| Production-ready? | | |

**Reflection questions**:
1. Which version would you feel confident deploying to production?
2. What security considerations appeared only in the specialized version?
3. How much additional work would Part A require to match Part B's quality?
4. What's the time savings: 15 minutes specifying the role vs. hours refactoring generic code?

**Success criteria**: You clearly see that roleplay produces more secure, production-ready code with defensive practices and comprehensive error handling ✓

---

## Exercise 3: Create Your Custom Role (10 min)

Task: Define a specialized role for your capstone project from Lesson 8.

**Template**:
```
You are a [role title] with [X years] of experience in [specialization areas].

You specialize in:
- [Technical area 1]
- [Technical area 2]
- [Technical area 3]

Style modifiers:
- [Coding philosophy: pragmatic/defensive/test-first]
- [Quality standards: error handling, logging, documentation]
- [Anti-patterns to avoid]

[Your specific task...]
```

**Example for a CLI tool capstone**:
```
You are a senior Python developer with 8 years of experience building command-line tools and developer productivity applications. You specialize in:
- Click/Typer frameworks for CLI
- Rich library for beautiful terminal UIs
- Configuration management (YAML, TOML)
- Cross-platform compatibility

Style modifiers:
- Write defensive code with explicit error messages
- Include comprehensive help text and examples
- Follow Click best practices (commands, options, arguments)
- Add colorful, user-friendly terminal output
- Handle edge cases (missing config, network errors)
```

**Success criteria**: Your role definition is specific enough that AI will produce domain-expert code for your capstone ✓

---

### Tool Tips: Claude Code and Gemini CLI

These tips help you get the most from QDD and roleplay without adding new concepts.

- Claude Code
  - Project context loading: Let Claude scan structure first, then request specific work.
  - Multi-file operations: Ask it to coordinate changes across related files and update tests.
  - Iterative sessions: Alternate between “ask questions → implement → refine.”

- Gemini CLI
  - Session-based context: Use sessions to persist context across prompts.
  - Code generation: Give precise specs; request complete, production-ready files.
  - Explanation/docs: Have it explain code, generate README/test docs, and identify risks.

Keep using QDD: ask for questions first, then switch into your specialized role for implementation.

## Try With AI

Goal: Experience QDD vs. direct implementation.

1) QDD approach for a Task Management REST API
```
I need to implement a Task Management REST API (tasks CRUD, auth, pagination).

Before writing any code, ask me 10 detailed questions to tailor the solution to my needs. Cover auth strategy, data model, validation, error handling, performance, and testing. After I answer, propose an implementation plan and then generate the code.
```

2) Direct approach (for comparison)
```
Create a FastAPI REST API for task management with CRUD endpoints.
```

3) Compare
- Which path produced code closer to your needs?
- Which required fewer follow-up fixes?

Outcome: You practiced the interview-first collaboration that unlocks expert, tailored code from the AI.

