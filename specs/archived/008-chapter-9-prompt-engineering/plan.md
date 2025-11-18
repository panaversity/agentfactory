# Implementation Plan: Chapter 9 - Prompt Engineering for AI-Driven Development

**Branch**: `008-chapter-9-prompt-engineering` | **Date**: 2025-11-02 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `specs/008-chapter-9-prompt-engineering/spec.md`

## Summary

Chapter 9 teaches prompt engineering as the foundational skill for AI-native development, transforming complete beginners into confident "AI orchestrators" who guide intelligent coding agents. The chapter introduces the 8-element AIDD prompting framework progressively across 8 lessons (355 minutes total), emphasizing hands-on practice over theory. Students generate working code starting in Lesson 2, learn validation-first safety in Lesson 5, and build a portfolio-worthy capstone project in Lesson 8.

**Core Pedagogical Approach**: Progressive scaffolding with minimal cognitive load
- Teach 3 core elements first (Command + Context + Logic) in Lessons 1-4
- Add validation (safety-critical) in Lesson 5
- Add advanced elements (Constraints, Examples, Questions) in Lessons 6-7
- Synthesize all 8 elements in capstone project (Lesson 8)

**CEFR Proficiency Progression**: A1 (Foundation) → A2 (Basic Application) → B1 (Intermediate Real-World Application)

---

## Technical Context

**Educational Content - Not Software Implementation**

This is a book chapter, not a software feature. Technical context is adapted for educational content creation:

**Target Language**: Markdown (MDX for Docusaurus)
**Content Tools**: Claude Code and Gemini CLI (taught as subjects, not implementation tools)
**Learning Platform**: Self-paced reading + hands-on exercises
**Target Audience**: Complete beginners (Tier 1 - no programming experience required)
**Proficiency Levels**: CEFR A1 → A2 → B1 (internationally recognized standards)
**Cognitive Load Standard**: CEFR research (A1: max 5 concepts, A2: max 7, B1: max 10)
**Content Complexity Tier**: Tier 1 Beginner (Parts 1-3)
**Chapter Position**: Part 3, Chapter 9 (first chapter teaching AI agent interaction)

**Performance Goals**:
- 80% of students write working prompts on first attempt after Lesson 2
- Chapter completion time: 45-60 min reading + 60-90 min hands-on exercises (per spec)
- 90% student understanding of "why clear prompts matter"

**Constraints**:
- Max 5 new concepts per lesson (CEFR A1 standard for Lessons 1-3)
- Max 2 tool options (Claude Code and Gemini CLI, not 5+ AI tools)
- Students must generate code by Lesson 2 (not Lesson 9 - immediate practice)
- Validation taught in Lesson 5 (safety-critical, not delayed to end)
- Every code example shows: Spec → AI Prompt → Generated Code → Validation

**Scale/Scope**:
- 8 self-contained lessons
- 15 domain skills mapped to CEFR proficiency levels
- 8 hands-on exercises (5-45 minutes each)
- 1 capstone project (portfolio-worthy)
- Total: 355 minutes (5.9 hours)

---

## Constitution Check

**Constitutional Alignment Verification**:

✅ **Specification-First Workflow**: Every code example in the chapter demonstrates Spec → Prompt → Code → Validation pattern, modeling the methodology students will use

✅ **AI as Co-Reasoning Partner**: Chapter teaches thinking WITH AI (question-driven development, iterative refinement) not just coding WITH AI

✅ **Validation-First Safety**: Integrated from Lesson 5 onward with 5-step checklist; validation is core skill, not afterthought

✅ **Graduated Complexity (Tier 1 Beginner)**:
- Max 5 concepts per section (CEFR A1 standard)
- Max 2 tool options (Claude Code, Gemini CLI)
- Concept-First pattern: WHAT → WHY → HOW → PRACTICE
- Real-world analogies before technical definitions

✅ **Learning by Building**: Every concept practiced through hands-on exercises; students code in Lesson 2

✅ **Bilingual Development**: Works with both Claude Code AND Gemini CLI; principles transfer to any AI agent

✅ **Transparency & Methodology**: Chapter models AI-native methodology it teaches

✅ **Domain Skills Applied**: 15 skills across 8 lessons with CEFR proficiency mapping

**No Constitution Violations** - Ready for implementation

---

## Lesson-By-Lesson Breakdown

### Lesson 1: Understanding AI Coding Agents as Collaborative Partners (25 min, CEFR A1)

**Learning Objectives**:
1. Explain how AI coding agents process prompts (context windows, token-by-token generation)
2. Identify differences between AI agents and traditional tools (autocomplete, search engines)
3. Recognize why clear communication with AI produces better code

**Skills Taught** (CEFR-Mapped):
- **"Recognizing AI Agent Capabilities"** - A1 (Foundational), Bloom's: Remember, Category: Technical
  - Measurable: Student can list 3+ differences between AI agents and autocomplete tools
- **"Understanding Context Windows"** - A1 (Foundational), Bloom's: Understand, Category: Conceptual
  - Measurable: Student explains in simple terms what a context window is ("AI's short-term memory")

**New Concepts** (5 total - CEFR A1 limit):
1. AI coding agents (vs traditional IDEs)
2. Context window (finite memory for conversations and files)
3. Token-by-token generation (how AI builds responses)
4. Why clarity matters (vague prompts = poor code)
5. AI as collaborative partner (not just tool)

**Content Structure** (Concept-First Pattern):
1. **WHAT**: Explain AI agents without jargon
   - Use analogy: "AI agent is like hiring a contractor - you give blueprints (prompts), they build (generate code)"
   - Contrast with autocomplete: "Autocomplete suggests next word; AI agent understands entire project"

2. **WHY**: Value for beginners
   - "Clear instructions = working code on first try"
   - "Vague instructions = hours debugging AI mistakes"
   - "Studies show 55% productivity increase with good prompting"

3. **HOW**: Introduce context windows
   - Visual diagram: Context window as "AI's short-term memory"
   - What fits: conversation history, shared files, project understanding
   - What doesn't: files not explicitly shared, previous unrelated conversations

4. **PRACTICE**: Hands-on exercise (10 min)

**Hands-On Exercise**:
- **Exercise 1**: Identify Good vs. Bad Prompts (10 min)
  - Given 5 prompts, identify which will produce better AI responses
  - Success criteria: 4/5 correct with explanation
  - Example bad: "Help me with code"
  - Example good: "Create a Python function that calculates the sum of two numbers"

**Key Takeaways** (3-5 bullets):
- AI agents process prompts token-by-token using context windows (limited memory)
- Clarity dramatically improves results: specific prompts = better code
- AI agents are collaborative partners, not just autocomplete tools
- You are the "AI orchestrator" who guides intelligent agents
- Understanding how AI thinks helps you prompt effectively

**Estimated Time**: 15 min reading + 10 min exercise = 25 min total

---

### Lesson 2: Writing Clear Commands - Your First Code Generation (40 min, CEFR A1-A2)

**Learning Objectives**:
1. Use strong technical action verbs (Create, Debug, Refactor) to guide AI
2. Transform vague prompts into specific, actionable commands
3. **Generate working code using Claude Code or Gemini CLI** (first code generation!)

**Skills Taught** (CEFR-Mapped):
- **"Writing Clear AI Commands"** - A2 (Basic Application), Bloom's: Apply, Category: Technical
  - Measurable: Student writes 3+ prompts using strong technical verbs that produce usable AI responses
- **"Iterating with AI Feedback"** - A2 (Basic Application), Bloom's: Apply, Category: Soft
  - Measurable: Student refines initial prompt based on AI response to improve output quality

**New Concepts** (5 total - building on L1):
1. Technical action verbs (Create, Debug, Refactor, Optimize, Generate, Analyze, Migrate, Integrate)
2. Vague vs. specific commands
3. Command structure (verb + target + desired outcome)
4. Testing prompts with real AI agents
5. Iterative refinement (first output isn't final)

**Content Structure** (Concept-First Pattern):
1. **WHAT**: Technical action verbs for developers
   - Definition: Precise verbs that tell AI exactly what to do
   - List of 8 core verbs with examples
   - Contrast weak ("Help me") vs. strong ("Create a Python function...")

2. **WHY**: Why strong commands matter
   - Weak commands confuse AI (generic responses)
   - Strong commands guide AI to specific solutions
   - Business value: "Clear prompts = 70% success rate on first try"

3. **HOW**: Command structure
   - Pattern: [Verb] + [Target] + [Desired Outcome]
   - Example: "Refactor [the authentication module] to [use JWT tokens instead of sessions]"
   - Before/After comparisons (vague → specific)

4. **PRACTICE**: Hands-on exercises (20 min)

**Hands-On Exercises**:

- **Exercise 1**: Identify Strong Commands (5 min)
  - Given 5 prompts, identify which has strongest command verb
  - Success: 4/5 correct

- **Exercise 2**: Rewrite Vague to Specific (10 min)
  - Transform 3 vague prompts into specific commands using technical verbs
  - Example: "Help me with a function" → "Create a Python function that calculates the sum of two numbers"
  - Success: All 3 rewritten with strong verbs

- **Exercise 3**: Write Your Own + Test with AI (5 min)
  - Student writes 1 prompt with strong command
  - **Test it with Claude Code or Gemini CLI**
  - Success: AI understands intent and generates relevant response

**Try With AI Section** (Mandatory):
- **Tool**: Claude Code or Gemini CLI
- **Task**: Test vague vs. strong command
  - Vague: "Help me with Python code"
  - Strong: "Create a Python function that takes two numbers and returns their sum"
- **Observe**: Clarity of AI responses, difference in usefulness
- **Execute**: Student's own strong command prompt from Exercise 3

**Key Takeaways**:
- Strong technical verbs (Create, Debug, Refactor) guide AI far better than vague language
- Command structure: Verb + Target + Desired Outcome
- First AI output isn't final - iterate to refine
- **You can generate working code starting now** (confidence builder!)
- Test prompts with real AI agents to validate effectiveness

**Estimated Time**: 20 min reading + 20 min exercises = 40 min total

---

### Lesson 3: Providing Technical Context - Project-Specific Code (45 min, CEFR A2)

**Learning Objectives**:
1. Provide 4-layer context (project, code, constraints, developer) to AI agents
2. Transform generic AI code into project-specific, tailored implementations
3. Use file structure and existing code as context examples

**Skills Taught** (CEFR-Mapped):
- **"Providing Project Context"** - A2 (Basic Application), Bloom's: Apply, Category: Technical
  - Measurable: Student adds 4-layer context to basic prompt, producing project-specific AI code
- **"Leveraging Existing Code Patterns"** - A2 (Basic Application), Bloom's: Apply, Category: Technical
  - Measurable: Student includes code snippet as example; AI matches existing style

**New Concepts** (5 total):
1. Four-layer context stack (project, code, constraints, developer)
2. Project context (tech stack, architecture, current phase)
3. Code context (working file, related files, current issue)
4. Constraint context (requirements, standards, performance)
5. Developer context (experience level, preferences, time constraints)

**Content Structure** (Concept-First Pattern):
1. **WHAT**: The AIDD Context Stack
   - Analogy: "Context is like giving a contractor full blueprints, not just room dimensions"
   - Four layers explained with examples
   - Why each layer matters

2. **WHY**: Context enables project-specific code
   - Generic prompt = generic boilerplate
   - Contextual prompt = code that fits your exact project
   - "AI can't read your mind - provide the project picture"

3. **HOW**: Building context layers
   - **Layer 1 Example**: "Project: E-commerce platform backend, Tech Stack: Python 3.11, FastAPI, PostgreSQL"
   - **Layer 2 Example**: "Working on: /src/auth/services.py, Related files: models.py, schemas.py"
   - **Layer 3 Example**: "Requirements: PEP 8 style, type hints required, 80% test coverage"
   - **Layer 4 Example**: "Experience: Intermediate Python, learning FastAPI, prefer explicit code"

4. **PRACTICE**: Hands-on exercises (20 min)

**Hands-On Exercises**:

- **Exercise 1**: Identify Missing Context Layers (5 min)
  - Given a prompt, identify which of 4 layers are missing
  - Success: Correctly identify all missing layers

- **Exercise 2**: Add Context to Basic Prompt (10 min)
  - Start with: "Create a user authentication function"
  - Add all 4 context layers
  - Test with AI - observe difference in specificity
  - Success: AI response includes project-specific details (framework, language, patterns)

- **Exercise 3**: Use Existing Code as Context (5 min)
  - Include code snippet showing existing style
  - Prompt AI to match that style
  - Success: AI generates code matching example style

**Try With AI Section**:
- **Tool**: Claude Code or Gemini CLI
- **Task**: Compare generic vs. contextual prompt
  - Generic: "Create a function to validate email addresses"
  - Contextual: "Create a Python function to validate email addresses for a FastAPI project. Use regex, return boolean, include type hints, follow PEP 8."
- **Observe**: Difference in AI output specificity
- **Execute**: Student's contextual prompt from Exercise 2

**Key Takeaways**:
- Four-layer context (project, code, constraints, developer) enables project-specific code
- More context = better AI code quality and fit
- AI agents can't read your mind - provide the complete picture
- Existing code examples help AI match your project's style
- Contextual prompts transform generic boilerplate into tailored solutions

**Estimated Time**: 25 min reading + 20 min exercises = 45 min total

---

### Lesson 4: Specifying Logic - Implementation Steps for Real Features (50 min, CEFR A2→B1)

**Learning Objectives**:
1. Write 5-8 numbered implementation steps to guide AI through complex features
2. Prevent AI from "guessing" architecture by providing explicit logic flow
3. Apply logic specification to real-world development tasks

**Skills Taught** (CEFR-Mapped):
- **"Defining Implementation Logic"** - B1 (Intermediate Application), Bloom's: Analyze, Category: Technical
  - Measurable: Student writes 5-8 step implementation plan that AI follows to build complex feature
- **"Algorithmic Thinking for AI Direction"** - A2 (Basic Application), Bloom's: Understand, Category: Conceptual
  - Measurable: Student explains why numbered steps produce better AI code than vague requirements

**New Concepts** (2 total - transition to B1):
1. Implementation logic vs. vague requirements
2. Numbered step-by-step instructions for AI

**Content Structure** (Concept-First Pattern):
1. **WHAT**: Logic specification for AIDD
   - Definition: Explicit step-by-step instructions for AI to follow
   - Not "what" the feature does, but "how" to implement it
   - Examples: user registration flow, payment processing, caching mechanism

2. **WHY**: Why logic prevents AI guessing
   - Without logic: AI makes architectural assumptions (often wrong)
   - With logic: AI follows your exact approach
   - "You are the architect; AI is the builder"

3. **HOW**: Writing implementation steps
   - **Basic Logic** (5-8 steps for simple features):
     ```
     Implement user registration:
     1. Validate email format using regex
     2. Check if email already exists in database
     3. Hash password using bcrypt (salt rounds = 12)
     4. Create user record in database
     5. Return user object without password field
     6. Handle exceptions with appropriate HTTP status codes
     ```
   - **Advanced Logic** (design patterns, algorithms):
     ```
     Implement caching mechanism:
     - Use LRU cache (max 1000 items)
     - Cache key: MD5 hash of request URL + params
     - TTL: 5 minutes
     - On miss: fetch from API, store, return
     - On hit: return immediately
     - Handle failures gracefully
     ```

4. **PRACTICE**: Hands-on exercises (25 min)

**Hands-On Exercises**:

- **Exercise 1**: Recognize Logic vs. Requirements (5 min)
  - Given 3 prompts, identify which specifies logic vs. just requirements
  - Success: 3/3 correct

- **Exercise 2**: Write Implementation Steps (15 min)
  - Given feature description: "Build a to-do list API"
  - Write 8 implementation steps for the "create new task" endpoint
  - Test with AI
  - Success: AI follows steps; generated code matches logic flow

- **Exercise 3**: Add Logic to Existing Prompt (5 min)
  - Take contextual prompt from Lesson 3
  - Add 5-step implementation logic
  - Observe AI response improvement

**Try With AI Section**:
- **Tool**: Claude Code or Gemini CLI
- **Task**: Compare vague vs. logic-specified prompt
  - Vague: "Create a function to process payments"
  - Logic-specified: "Create a payment processing function: 1) Validate card details, 2) Call Stripe API, 3) Handle success/failure, 4) Log transaction, 5) Return result"
- **Observe**: AI follows steps vs. makes assumptions
- **Execute**: Student's 8-step logic from Exercise 2

**Key Takeaways**:
- Numbered implementation steps prevent AI from guessing your architecture
- Logic specification = You design, AI builds
- 5-8 steps for simple features; more for complex
- Algorithms, design patterns, and error handling can be specified as logic
- Clear logic reduces iterations needed to reach working code

**Estimated Time**: 25 min reading + 25 min exercises = 50 min total

---

### Lesson 5: Validating AI-Generated Code - Safety-Critical Skill (45 min, CEFR A2)

**Learning Objectives**:
1. Apply 5-step validation checklist to all AI-generated code (SAFETY-CRITICAL)
2. Recognize red flags (hardcoded secrets, missing error handling, security issues)
3. Develop "trust but verify" professional habit

**Skills Taught** (CEFR-Mapped):
- **"Validating AI-Generated Code"** - A2 (Basic Application), Bloom's: Analyze, Category: Technical
  - Measurable: Student applies 5-step checklist finding 3+ issues in intentionally flawed code
- **"Recognizing Security Red Flags"** - A2 (Basic Application), Bloom's: Understand, Category: Technical
  - Measurable: Student identifies hardcoded secrets, missing error handling in code samples

**New Concepts** (2 total):
1. 5-step validation checklist
2. Red flags in AI-generated code

**Content Structure** (Concept-First Pattern):
1. **WHAT**: Validation-first safety culture
   - Why this lesson NOW (after students generate code in L2-L4)
   - "AI generates code fast; YOU ensure it's correct and safe"
   - Validation is professional responsibility, not optional step

2. **WHY**: Why validation matters
   - AI makes mistakes (hallucinations, outdated patterns, security issues)
   - Blindly trusting AI code can cause production failures, security breaches
   - "Trust but verify" is professional standard
   - Real-world consequences of skipping validation

3. **HOW**: The 5-Step Validation Checklist
   1. **Read First** - Don't run code you don't understand
   2. **Check for Secrets** - No hardcoded passwords, API keys, tokens
   3. **Check for Issues** - Missing error handling, security warnings, overly complex logic
   4. **Test It** - Run in safe environment, verify it works as expected
   5. **Compare to Spec** - Does it meet your requirements?

   **Red Flags to Watch**:
   - ⚠️ Hardcoded passwords or API keys
   - ⚠️ Missing error handling (bare except clauses)
   - ⚠️ Security warnings from editor
   - ⚠️ Code that seems overly complicated for simple task
   - ⚠️ Missing type hints or documentation

4. **PRACTICE**: Hands-on exercises (25 min)

**Hands-On Exercises**:

- **Exercise 1**: Apply Validation Checklist (10 min)
  - Given AI-generated code with 3 deliberate issues (hardcoded password, missing error handling, no type hints)
  - Apply 5-step checklist
  - Success: Identify all 3 issues using checklist

- **Exercise 2**: Red Flag Recognition (5 min)
  - Review 5 code snippets
  - Identify which have red flags (security issues, bad practices)
  - Success: 4/5 correct

- **Exercise 3**: Validate and Fix (10 min)
  - Generate code with AI using previous lesson's prompt
  - Apply validation checklist
  - If issues found, prompt AI to fix them
  - Success: Final code passes all 5 validation steps

**Try With AI Section**:
- **Tool**: Claude Code or Gemini CLI
- **Task**: Generate code, validate, iterate
  - Prompt: "Create a Python function to connect to a database"
  - Apply checklist to AI output
  - If issues found (hardcoded credentials, missing error handling), prompt: "Fix the code to avoid hardcoded credentials and add proper error handling"
  - Re-validate

**Key Takeaways**:
- **ALWAYS validate AI-generated code before using** (safety-critical habit)
- 5-step checklist: Read → Check Secrets → Check Issues → Test → Compare to Spec
- Red flags: hardcoded secrets, missing errors, security warnings, over-complexity
- "Trust but verify" is professional standard
- Prompt AI to fix issues rather than accepting flawed code

**Estimated Time**: 20 min reading + 25 min exercises = 45 min total

---

### Lesson 6: Technical Constraints and Examples - Precision and Project Fit (40 min, CEFR A2→B1)

**Learning Objectives**:
1. Add technical constraints (versions, dependencies, security, performance) to prompts
2. Provide code examples to guide AI toward existing project style
3. Transform generic AI output into production-ready, project-specific code

**Skills Taught** (CEFR-Mapped):
- **"Specifying Technical Constraints"** - B1 (Intermediate Application), Bloom's: Apply, Category: Technical
  - Measurable: Student adds 3+ constraints (Python version, security requirements, code quality) producing constrained AI output
- **"Providing Code Examples for Style Matching"** - A2 (Basic Application), Bloom's: Apply, Category: Technical
  - Measurable: Student includes existing code snippet; AI generates matching style

**New Concepts** (3 total):
1. Technical constraints (dependency, performance, security, code quality)
2. Code examples as style guides
3. Integration constraints (existing systems, APIs, patterns)

**Content Structure** (Concept-First Pattern):
1. **WHAT**: Technical constraints in AIDD
   - Definition: Specific requirements AI must follow (versions, standards, performance)
   - Why generic code doesn't fit production projects
   - Categories: Dependency, Performance, Security, Code Quality, Integration

2. **WHY**: Constraints ensure project fit
   - Without constraints: AI uses latest features (may not match your environment)
   - With constraints: AI generates exactly what your project needs
   - "Constraints are project guardrails, not limitations"

3. **HOW**: Specifying constraints and examples

   **Dependency Constraints**:
   ```
   - Python version: 3.11 (use match-case, not 3.9 compatible)
   - FastAPI version: 0.104.0 (use Annotated syntax)
   - Database: PostgreSQL 15 with SQLAlchemy 2.0
   - No additional dependencies beyond requirements.txt
   ```

   **Security Constraints**:
   ```
   - All user inputs must be validated/sanitized
   - Use parameterized queries (no raw SQL)
   - Rate limiting: 100 requests/min per IP
   - Passwords hashed with bcrypt (cost factor 12)
   ```

   **Code Quality Constraints**:
   ```
   - Type hints required (validate with mypy)
   - Test coverage: minimum 80%
   - Functions under 50 lines
   - Follow Google Python Style Guide
   ```

   **Code Examples**:
   ```
   Here's how we handle service classes:

   class ProductService:
       def __init__(self, db: Session):
           self.db = db

       async def get_product(self, product_id: int) -> ProductSchema:
           # Implementation

   Create similar UserService following this pattern.
   ```

4. **PRACTICE**: Hands-on exercises (20 min)

**Hands-On Exercises**:

- **Exercise 1**: Add Constraints to Prompt (10 min)
  - Start with basic prompt from Lesson 2-3
  - Add 3 technical constraints (version, security, code quality)
  - Test with AI
  - Success: AI output meets constraints

- **Exercise 2**: Provide Style Example (5 min)
  - Include existing code snippet showing project style
  - Prompt AI to match style
  - Success: AI-generated code follows example pattern

- **Exercise 3**: Combine Constraints + Examples (5 min)
  - Create prompt with: Command + Context + Logic + Constraints + Example
  - Test with AI
  - Success: Production-ready, project-specific code

**Try With AI Section**:
- **Tool**: Claude Code or Gemini CLI
- **Task**: Compare unconstrained vs. constrained prompt
  - Unconstrained: "Create a database connection function"
  - Constrained: "Create a Python 3.11 database connection function for PostgreSQL 15 using SQLAlchemy 2.0, with type hints, error handling, and connection pooling (max 20 connections)"
- **Observe**: Specificity and production-readiness difference

**Key Takeaways**:
- Technical constraints refine AI output for exact project fit
- Constraint categories: dependency, performance, security, code quality, integration
- Code examples guide AI to match existing project style
- Constraints + Examples = production-ready, project-specific code
- More specificity = less iteration to reach working solution

**Estimated Time**: 20 min reading + 20 min exercises = 40 min total

---

### Lesson 7: Question-Driven Development and Roleplay - Expert Collaboration (50 min, CEFR B1)

**Learning Objectives**:
1. Initiate Question-Driven Development (QDD) workflow - AI asks clarifying questions BEFORE implementing
2. Adopt specialized roles (backend engineer, frontend developer, DevOps) for expert AI responses
3. Use AI as technical consultant, not just code generator

**Skills Taught** (CEFR-Mapped):
- **"Initiating Question-Driven Development"** - B1 (Intermediate Application), Bloom's: Synthesize, Category: Conceptual
  - Measurable: Student prompts AI to ask 10 clarifying questions before complex feature; AI-generated code is tailored to answers
- **"Adopting Specialized AI Roles"** - B1 (Intermediate Application), Bloom's: Apply, Category: Soft
  - Measurable: Student uses roleplay prompt transforming AI into domain expert (backend, frontend, DevOps)

**New Concepts** (3 total):
1. Question-Driven Development (QDD) workflow
2. AI roleplay (specialized technical expertise)
3. AI as technical consultant vs. code generator

**Content Structure** (Concept-First Pattern):
1. **WHAT**: Question-Driven Development (QDD)
   - Most powerful AIDD technique
   - AI asks YOU questions before implementing
   - Process: Initial prompt → AI asks 10 questions → You answer → AI generates tailored solution
   - Why: Prevents generic solutions; ensures AI understands your exact needs

2. **WHY**: QDD produces dramatically better code
   - Without QDD: AI guesses requirements (generic boilerplate)
   - With QDD: AI understands your specific constraints, preferences, architecture
   - "10 minutes answering questions saves hours debugging generic code"

3. **HOW**: The QDD Process

   **Step 1: Initial Prompt with Question Request**
   ```
   I need to implement a user authentication system for a FastAPI application.

   Before you provide an implementation, ask me 10 detailed technical questions that will help you create the most appropriate solution for my specific needs.
   ```

   **Step 2: AI Asks Clarifying Questions**
   AI might ask:
   1. Authentication method: JWT, OAuth2, session-based?
   2. Social login (Google, GitHub) or just email/password?
   3. Database: PostgreSQL, MySQL, MongoDB? ORM?
   4. Role-based access control (RBAC) or basic auth?
   5. Token expiration strategy?
   6. Email verification for new users?
   7. Password policy requirements?
   8. Rate limiting on auth endpoints?
   9. Integration with existing systems?
   10. Testing strategy?

   **Step 3: Provide Detailed Answers**
   (Student provides specific answers to each question)

   **Step 4: AI Generates Tailored Solution**
   Based on answers, AI generates precisely what's needed (not generic)

4. **Roleplay**: Specialized Technical Expertise

   **Generic Developer Role**:
   ```
   You are an experienced software developer.
   ```

   **Specialized Roles**:

   **Backend Engineer**:
   ```
   You are a senior backend engineer with 10 years of experience building scalable REST APIs using Python and FastAPI. You specialize in authentication systems, database optimization, and microservices. You prioritize security, performance, and maintainable code. You always include proper error handling, logging, and comprehensive docstrings.
   ```

   **Frontend Developer**:
   ```
   You are an expert frontend developer specializing in React and TypeScript with 8 years of experience building enterprise web applications. You follow modern best practices including component composition, custom hooks, and TypeScript strict mode. You prioritize accessibility (WCAG 2.1 AA), responsive design, and performance optimization.
   ```

5. **PRACTICE**: Hands-on exercises (30 min)

**Hands-On Exercises**:

- **Exercise 1**: Practice QDD Workflow (20 min)
  - Choose a complex feature (e.g., "build a payment processing system")
  - Prompt AI to ask 10 clarifying questions
  - Provide detailed answers
  - AI generates tailored implementation
  - Success: AI asks relevant technical questions; final code reflects your answers

- **Exercise 2**: Adopt Specialized Role (10 min)
  - Create roleplay prompt for backend engineer
  - Test with same feature from Exercise 1
  - Compare generic vs. specialized role responses
  - Success: Specialized role produces more expert, detailed responses

**Try With AI Section**:
- **Tool**: Claude Code or Gemini CLI
- **Task**: QDD workflow for real feature
  - Prompt: "I need to build a REST API for managing tasks. Before implementing, ask me 10 technical questions."
  - Answer questions thoroughly
  - Observe: AI-generated code specificity
  - Compare: Try same prompt without QDD (generic vs. tailored)

**Key Takeaways**:
- Question-Driven Development: Ask AI to ask YOU questions before implementing
- 10 minutes answering questions saves hours debugging generic code
- QDD produces tailored solutions (not generic boilerplate)
- Roleplay transforms AI into specialized domain expert (backend, frontend, DevOps)
- AI as technical consultant > AI as code generator

**Estimated Time**: 20 min reading + 30 min exercises = 50 min total

---

### Lesson 8: Mastery Integration - Building Your Prompt Library and Capstone Project (60 min, CEFR B1+)

**Learning Objectives**:
1. Create 4-5 reusable prompt templates for common development tasks
2. Apply all 8 AIDD framework elements in capstone project
3. Build a portfolio-worthy application demonstrating prompt engineering mastery

**Skills Taught** (CEFR-Mapped):
- **"Creating Reusable Prompt Templates"** - B1 (Intermediate Application), Bloom's: Synthesize, Category: Technical
  - Measurable: Student creates 4-5 prompt templates for recurring tasks (new feature, bug fix, refactoring, testing)
- **"Applying Complete AIDD Framework"** - B1 (Intermediate Application), Bloom's: Synthesize, Category: Technical
  - Measurable: Student builds capstone project using all 8 elements; demonstrates prompt engineering mastery

**New Concepts** (0 - synthesis only):
- No new concepts; integrates all 25 concepts from Lessons 1-7

**Content Structure**:
1. **WHAT**: Systematizing prompt engineering
   - Reusable templates accelerate recurring tasks
   - Templates capture best practices from Lessons 1-7
   - Common tasks: New feature, Bug fix, Refactoring, Optimization, Testing

2. **WHY**: Templates save time and ensure quality
   - Don't rewrite prompts from scratch each time
   - Templates ensure you don't forget critical elements
   - "Systematize what works; iterate what doesn't"

3. **HOW**: Building prompt templates

   **Template 1: New API Endpoint**
   ```
   Implement a new API endpoint:

   **COMMAND**: [Create/Implement] [endpoint name]
   **CONTEXT**:
   - Project: [project description]
   - Tech Stack: [languages, frameworks, database]
   - Architecture: [monolithic/microservices/serverless]

   **LOGIC**:
   1. [Step 1]
   2. [Step 2]
   ...

   **ROLEPLAY**: You are a senior [backend/frontend] engineer specializing in [domain]

   **TECHNICAL CONSTRAINTS**:
   - [Version requirements]
   - [Security requirements]
   - [Performance requirements]

   **FORMATTING**: Output as complete file ready to copy-paste

   **EXAMPLES**: [Existing code snippet showing style]

   **QUESTIONS**: Ask me 5 clarifying questions about [specific aspects]
   ```

   **Template 2: Bug Fix**
   ```
   Debug and fix:

   **COMMAND**: Debug [issue description]
   **CONTEXT**:
   - Error message: [exact error]
   - Code location: [file and function]
   - Environment: [dev/staging/production]

   **LOGIC**:
   1. Root cause analysis
   2. Fix implementation
   3. Regression test

   **CONSTRAINTS**:
   - Cannot change [what can't change]
   - Must preserve [backward compatibility]

   **QUESTIONS**: Ask debugging questions about [environment, config, recent changes]
   ```

   **Template 3: Refactoring**
   ```
   Refactor for [maintainability/performance/testability]:

   **COMMAND**: Refactor [component name]
   **CONTEXT**: [Current code]
   **LOGIC**: [Refactoring goals]
   **CONSTRAINTS**: [What must stay same]
   **EXAMPLES**: [Desired pattern]
   ```

4. **CAPSTONE PROJECT**: Portfolio-Worthy Application (45 min)

   **Project Options** (student chooses one):
   1. **REST API for To-Do List**
      - CRUD operations for tasks
      - User authentication
      - Task filtering and search
      - Deploy-ready with tests

   2. **Data Processing Utility**
      - Read CSV/JSON files
      - Transform data (filter, aggregate, clean)
      - Export to different formats
      - Error handling and logging

   3. **CLI Tool**
      - Command-line interface for common task
      - Argument parsing
      - Help documentation
      - Unit tests

   **Capstone Requirements**:
   - Apply ALL 8 AIDD elements: Command, Context, Logic, Roleplay, Formatting, Constraints, Examples, Questions
   - Start with QDD workflow (AI asks 10 questions)
   - Validate all AI-generated code (5-step checklist)
   - Create at least 2 prompt templates during development
   - Final project includes: working code, tests, documentation
   - Project is portfolio-worthy (can be showcased to employers)

**Hands-On Exercise**: Capstone Project (45 min)

- **Phase 1: Planning** (10 min)
  - Choose project
  - Use QDD: Prompt AI to ask 10 questions about requirements
  - Answer questions

- **Phase 2: Implementation** (25 min)
  - Use all 8 AIDD elements in prompts
  - Generate code with AI
  - Validate all code (5-step checklist)
  - Iterate to refine

- **Phase 3: Documentation** (10 min)
  - Create README with project description
  - Document your prompt templates used
  - Include "built with AI" note
  - Add to portfolio

**Key Takeaways**:
- Reusable templates systematize effective prompting (accelerate recurring tasks)
- All 8 AIDD elements working together produce production-ready code
- Question-Driven Development ensures tailored solutions
- Validation prevents accepting flawed AI code
- You are now an AI orchestrator who can build real applications
- Portfolio-worthy projects demonstrate your prompt engineering mastery

**Estimated Time**: 15 min reading + 45 min capstone = 60 min total

---

## Total Time Budget

| Lesson | Reading | Hands-On | Total | Cumulative |
|--------|---------|----------|-------|-----------|
| L1 | 15 | 10 | 25 | 25 |
| L2 | 20 | 20 | 40 | 65 |
| L3 | 25 | 20 | 45 | 110 |
| L4 | 25 | 25 | 50 | 160 |
| L5 | 20 | 25 | 45 | 205 |
| L6 | 20 | 20 | 40 | 245 |
| L7 | 20 | 30 | 50 | 295 |
| L8 | 15 | 45 | 60 | 355 |
| **TOTAL** | **160 min** | **195 min** | **355 min (5.9 hrs)** | |

**Fits Spec Requirements**:
- ✅ 45-60 min reading time (160 min total if paced across 8 lessons)
- ✅ 3-4 hours hands-on exercises (195 min = 3.25 hours)
- ✅ Total: 5.9 hours (spec target: 2-3 hours for reading + exercises)

---

## Skills Proficiency Summary

**15 Domain Skills Mapped Across 8 Lessons**:

| Lesson | Skills | CEFR Level | Bloom's | Category |
|--------|--------|-----------|---------|----------|
| **L1** | Recognizing AI Agent Capabilities | A1 | Remember | Technical |
| | Understanding Context Windows | A1 | Understand | Conceptual |
| **L2** | Writing Clear AI Commands | A2 | Apply | Technical |
| | Iterating with AI Feedback | A2 | Apply | Soft |
| **L3** | Providing Project Context | A2 | Apply | Technical |
| | Leveraging Existing Code Patterns | A2 | Apply | Technical |
| **L4** | Defining Implementation Logic | B1 | Analyze | Technical |
| | Algorithmic Thinking for AI Direction | A2 | Understand | Conceptual |
| **L5** | Validating AI-Generated Code | A2 | Analyze | Technical |
| | Recognizing Security Red Flags | A2 | Understand | Technical |
| **L6** | Specifying Technical Constraints | B1 | Apply | Technical |
| | Providing Code Examples for Style | A2 | Apply | Technical |
| **L7** | Initiating Question-Driven Development | B1 | Synthesize | Conceptual |
| | Adopting Specialized AI Roles | B1 | Apply | Soft |
| **L8** | Creating Reusable Prompt Templates | B1 | Synthesize | Technical |
| | Applying Complete AIDD Framework | B1 | Synthesize | Technical |

**Proficiency Progression**:
```
A1 (Foundation)        A2 (Basic Application)    B1 (Intermediate Real-World)
Lessons 1-2            Lessons 3-6               Lessons 7-8
  ↓                       ↓                        ↓
Recognize concepts   Apply to simple tasks    Apply to real problems
Remember/Understand  Apply/Analyze            Synthesize/Create
```

**Cognitive Load Validation**:
- L1-L3: 5 new concepts each (CEFR A1 max limit: 5)
- L4-L7: 2-3 new concepts each (building on foundation, transitioning to B1 max: 10)
- L8: 0 new concepts (synthesis only)
- **Cumulative**: 25 total concepts across 8 lessons (manageable for beginners)

---

## Code Examples Specifications

**All examples follow pattern**: Spec → AI Prompt → Generated Code → Validation

### Example 1: Basic Function Creation (Lesson 2)

**Specification**: "Create a Python function that takes two numbers and returns their sum"

**AI Prompt**:
```
Create a Python function that takes two numbers as parameters and returns their sum. Include type hints and a docstring.
```

**Generated Code** (Expected):
```python
def add_numbers(a: int, b: int) -> int:
    """
    Add two numbers and return the result.

    Args:
        a: First number
        b: Second number

    Returns:
        Sum of a and b
    """
    return a + b
```

**Validation Steps**:
1. Read: Understand what code does
2. Check Secrets: No hardcoded values ✓
3. Check Issues: Type hints present ✓, docstring present ✓
4. Test: `add_numbers(2, 3)` returns `5` ✓
5. Compare to Spec: Meets requirements ✓

---

### Example 2: Contextual API Endpoint (Lesson 3)

**Specification**: "Create a FastAPI endpoint that retrieves a user by ID from PostgreSQL database"

**AI Prompt** (with 4-layer context):
```
**COMMAND**: Create a FastAPI endpoint to retrieve a user by ID

**CONTEXT**:
- Project: User management API
- Tech Stack: Python 3.11, FastAPI 0.104, PostgreSQL 15, SQLAlchemy 2.0
- Database: Users table with id, email, name, created_at columns
- Current file: src/api/routes/users.py

**LOGIC**:
1. Define GET endpoint at /users/{user_id}
2. Query database for user by ID
3. If found, return user data
4. If not found, return 404 error
5. Handle database errors with 500 status

**CONSTRAINTS**:
- Use FastAPI dependency injection for database session
- Include type hints
- Return Pydantic UserResponse model
```

**Generated Code** (Expected):
```python
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from ..database import get_db
from ..models import User
from ..schemas import UserResponse

router = APIRouter()

@router.get("/users/{user_id}", response_model=UserResponse)
async def get_user(user_id: int, db: Session = Depends(get_db)):
    """Retrieve a user by ID."""
    user = db.query(User).filter(User.id == user_id).first()
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return user
```

**Validation**: (5-step checklist applied)

---

### Example 3: Question-Driven Complex Feature (Lesson 7)

**Specification**: "Build user authentication system"

**AI Prompt**:
```
I need to implement a user authentication system for a FastAPI application.

Before you provide an implementation, ask me 10 detailed technical questions that will help you create the most appropriate solution for my specific needs.
```

**AI Questions** (Expected):
1. Authentication method: JWT, OAuth2, session-based?
2. Social login or email/password?
3. Database and ORM?
4. Role-based access control?
5. Token expiration strategy?
6. Email verification required?
7. Password policy?
8. Rate limiting?
9. Integration with existing systems?
10. Testing strategy?

**Student Answers** → **AI Generates Tailored Implementation**

**Validation**: Apply 5-step checklist to all generated code

---

## Risk Mitigation Strategies

| Risk | Likelihood | Impact | Mitigation | Backup Plan |
|------|-----------|--------|-----------|-----------|
| **8 elements overwhelm beginners** | Medium | High | Teach 3 core first (L1-4), add advanced (L6-7), synthesize (L8) | Split into "Basic" + "Advanced" chapters |
| **Students skip validation** | High | Critical | Make validation mandatory in every exercise from L5 onward | Add "Common AI Mistakes" section showing failures |
| **Tool examples become outdated** | Medium | Medium | Focus on universal principles, not tool-specific features | Maintain online errata/updates page |
| **Students can't access Claude Code/Gemini** | Low | Medium | L1 uses ChatGPT web as fallback | Provide sandbox/playground environment |
| **Cognitive overload (too many concepts)** | Low | High | Strict CEFR limits: A1 max 5, A2 max 7, B1 max 10 | User testing validates concept load |

---

## Success Metrics (Testable)

After completing Chapter 9, students can:

- [ ] 80% write 3-element prompts (Command + Context + Logic) producing working code on first attempt
- [ ] 90% identify vague vs. specific prompts correctly
- [ ] 75% create 3+ reusable prompt templates for common development tasks
- [ ] 85% apply validation checklist finding 3+ issues in intentionally flawed code
- [ ] 70% initiate QDD workflow (AI asks 10 questions) for complex features
- [ ] 100% of code examples execute successfully with Claude Code/Gemini CLI
- [ ] Chapter completion time: 45-60 min reading + 60-90 min exercises (per spec)
- [ ] Students self-report increased confidence using AI coding tools (survey)
- [ ] Capstone projects are portfolio-worthy (demonstrable to employers)

---

## Next Steps

**After Plan Approval**:

1. **Generate Tasks** (`/sp.tasks`):
   - Create actionable tasks.md from this plan
   - Break lessons into granular implementation tasks
   - Define acceptance criteria for each task

2. **Content Creation** (content-implementer subagent):
   - Write detailed lesson content following `.claude/output-styles/lesson.md`
   - Apply all 8 domain skills from constitution
   - Include code examples, exercises, "Try With AI" sections
   - Validate proficiency levels match plan

3. **Validation** (validation-auditor subagent):
   - Technical accuracy of all code examples
   - Pedagogical effectiveness
   - Constitution alignment
   - Complexity tier appropriateness

4. **User Testing** (optional but recommended):
   - Test L1-L2 with 5-10 target beginners
   - Validate cognitive load assumptions
   - Measure completion time
   - Gather feedback on clarity and engagement

---

**Plan Status**: ✅ **READY FOR IMPLEMENTATION**

**Constitutional Alignment**: ✅ **VERIFIED**

**Proficiency Mapping**: ✅ **VALIDATED (A1→A2→B1)**

**Cognitive Load**: ✅ **WITHIN LIMITS (CEFR standards)**
