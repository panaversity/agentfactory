# Chapter Specification: Pydantic and Generics

**Feature Branch**: `001-part-4-chapter-27`
**Created**: 2025-11-09
**Status**: Draft
**Chapter Number**: 27
**Part**: 4 (Python Fundamentals)
**File Name**: `27-pydantic-generics/`
**Complexity Tier**: Advanced (B1-B2)

**Input**: Chapter 27: "Pydantic and Generics" - Advanced type safety and data validation for AI-native Python development covering both AI-native use cases (LLM structured outputs, prompt validation) and general validation patterns (FastAPI, config files) with practical Generics patterns for type-safe containers and reusable functions.

---

## Success Evals (Measurable Success Criteria) *(mandatory - FIRST)*

### Comprehension Evals (How students know they understand)

- **EVAL-001**: 80%+ of students can explain the difference between runtime validation (Pydantic) and static type checking (Generics) when asked "Why do we need both?"
- **EVAL-002**: 75%+ of students can identify when to use Pydantic vs TypedDict vs dataclasses in a code review scenario
- **EVAL-003**: Students can describe 3 real-world use cases for Pydantic in AI-native development (LLM output validation, config management, API data modeling)

### Skill Acquisition Evals (What students can BUILD)

- **EVAL-004**: 90%+ of students can create a working Pydantic model with nested validation and custom validators from a data requirements description
- **EVAL-005**: 85%+ of students can write a generic function that works with multiple types using TypeVar and type hints
- **EVAL-006**: 80%+ of students can build a production-quality Config Manager combining Pydantic models + Generic containers (capstone project)
- **EVAL-007**: Students can validate LLM-generated JSON output using Pydantic models and handle validation errors gracefully

### Engagement Evals (Completion and interaction)

- **EVAL-008**: 75%+ chapter completion rate (students reach capstone lesson)
- **EVAL-009**: 80%+ of students complete at least 3 of 4 "Try With AI" prompts per lesson
- **EVAL-010**: Students submit working capstone Config Manager project for peer review

### Accessibility Eval (Reading level appropriate)

- **EVAL-011**: Content maintains Grade 10-11 reading level (advanced Python students with OOP background)
- **EVAL-012**: Technical terms (covariance, contravariance, validators) explained with analogies before first use

---

## Topic Summary *(mandatory)*

Pydantic is a Python library that brings runtime data validation and parsing to Python's type hints system. In AI-native development, Pydantic solves the critical problem of validating unstructured AI outputs (like LLM-generated JSON) and ensuring data integrity in production systems. When Claude Code generates JSON or when you build agent systems with FastAPI, Pydantic ensures the data matches your expectations BEFORE it causes runtime errors.

Generics in Python enable you to write type-safe, reusable code that works with multiple data types while preserving type information. Instead of writing separate functions for `list[int]`, `list[str]`, `list[User]`, you write ONE generic function that works with any type. Generics are the foundation of modern Python's typing system (introduced in PEP 484, enhanced in PEP 695) and essential for building maintainable AI agent systems where data flows through multiple layers.

Together, Pydantic and Generics form the type safety layer of professional Python development. Pydantic validates data AT RUNTIME (catching bad inputs from users, APIs, and AI). Generics provide type safety AT DESIGN TIME (helping your IDE catch bugs before you run code). This chapter teaches both patterns and shows how they work together in a production-quality Config Manager—a pattern you'll use in every AI agent project.

**Why this matters for AI-native development**:
- AI outputs are unpredictable → Pydantic validates them
- Agent systems have complex data flows → Generics keep them type-safe
- Production systems need reliability → Both prevent runtime failures
- FastAPI (the standard for AI agent APIs) is built on Pydantic
- This is NOT academic—every production Python/AI project uses these patterns

---

## Prerequisites *(mandatory)*

### Required Knowledge (Chapters 1-26)

**Essential Prerequisites**:
- **Chapter 24-25 (OOP Part I & II)**: Classes, inheritance, method overriding, special methods (`__init__`, `__str__`, `__eq__`)
- **Chapter 26 (Metaclasses and Dataclasses)**: Understanding of `@dataclass` decorator, field definitions, class customization
- **Chapter 14-16 (Type System)**: Type hints, annotations, `list[int]`, `dict[str, float]`, `X | None` syntax
- **Chapter 17 (Control Flow)**: Exception handling (try/except blocks for validation errors)

**Assumed Background**:
- Comfortable with Python's type hint syntax (`str`, `int`, `list[T]`, `dict[K, V]`, `X | None`)
- Understanding of data validation concepts (what is "valid email"? "valid JSON"?)
- Familiarity with JSON data structures (dicts, lists, nested objects)
- Basic understanding of APIs and configuration files (Chapters 5-8 covered AI tools that use these)

**New to This Chapter**:
- Pydantic library (installation, models, validators, settings)
- Generic type parameters (`TypeVar`, `Generic`, `Protocol`)
- Modern PEP 695 syntax for generics (Python 3.14+)
- Combining runtime validation with static type checking
- Production patterns for config management

---

## Learning Objectives (Measurable Outcomes Aligned with Evals) *(mandatory)*

By the end of Chapter 27, students will be able to:

### LO-1: Pydantic Data Validation (B1-B2)
**Aligned with EVAL-001, EVAL-004, EVAL-007**

Students can CREATE Pydantic models to validate complex, nested data structures with custom validation rules. They can APPLY Pydantic to validate LLM-generated JSON outputs and handle validation errors gracefully in production code.

**Measurable at this level (B1-B2)**:
- B1: Create basic Pydantic model with 3-5 fields and built-in validators
- B2: Design models with nested validation, custom validators, and error handling for real-world scenarios

### LO-2: Generic Type Safety Patterns (B1-B2)
**Aligned with EVAL-002, EVAL-005**

Students can WRITE generic functions and classes using `TypeVar` and `Generic` to create type-safe, reusable code. They can ANALYZE code to determine when Generics improve type safety vs when simpler approaches suffice.

**Measurable at this level (B1-B2)**:
- B1: Write generic function with single type parameter for simple container operations
- B2: Create generic classes with multiple type parameters and constraints (bounds)

### LO-3: Integration Patterns (B2)
**Aligned with EVAL-006**

Students can INTEGRATE Pydantic validation with Generic containers to build production-quality data pipelines. They can EVALUATE tradeoffs between Pydantic, TypedDict, and dataclasses for different use cases.

**Measurable at this level (B2)**:
- Design and implement Config Manager using Pydantic models + Generic containers
- Justify architectural choices (when Pydantic? when Generics? when both?)

### LO-4: AI-Native Application (B2)
**Aligned with EVAL-003, EVAL-007**

Students can APPLY Pydantic to validate AI agent outputs (LLM JSON, structured data from Claude Code). They can EXPLAIN how validation fits into the AI-native development workflow (describe intent → generate → validate → iterate).

**Measurable at this level (B2)**:
- Validate LLM-generated responses using Pydantic models
- Handle validation failures and request regeneration from AI with improved prompts

---

## Content Outline *(mandatory)*

### Section 1: Pydantic Fundamentals (Lessons 1-2)
**Cognitive Load**: Max 10 concepts per lesson (Advanced tier)

**Lesson 1: Introduction to Pydantic and Data Validation**
- What is data validation and why it matters (runtime safety)
- Installing Pydantic V2 with `uv` (modern Python tooling)
- Creating your first Pydantic model (`BaseModel` class)
- Basic field types and automatic validation
- Validation errors and error handling
- 💬 AI Colearning: "Ask your AI: Explain the difference between type hints and Pydantic validation"
- 🎓 Instructor Commentary: "Type hints are documentation—Pydantic makes them enforceable"

**Lesson 2: Advanced Pydantic Patterns**
- Nested models (models within models)
- Custom validators (`@field_validator`, `@model_validator`)
- Field constraints (`Field()` with min/max, regex, examples)
- Optional fields and default values (`X | None`, `Field(default=...)`)
- Settings management (`BaseSettings` for environment variables)
- 🚀 CoLearning Challenge: "Tell your AI: Create a User model with email validation and nested Address model"

### Section 2: Generics Fundamentals (Lessons 3-4)
**Cognitive Load**: Max 10 concepts per lesson

**Lesson 3: Introduction to Generics and Type Variables**
- What are Generics? (type-safe reusable code)
- Creating type variables (`TypeVar`)
- Generic functions (functions that work with any type)
- Modern PEP 695 syntax (`def func[T](x: T) -> T:`)
- Type safety in IDEs (autocomplete and error detection)
- 💬 AI Colearning: "Ask your AI: Why do we need Generics when Python is dynamically typed?"
- 🎓 Instructor Commentary: "Generics are for tools (IDEs, type checkers), not for Python runtime"

**Lesson 4: Generic Classes and Protocols**
- Generic classes (`Generic[T]`, multiple type parameters)
- Bounded type variables (constraints with `bound=`)
- Protocols (`Protocol` for structural subtyping)
- When to use Generics vs `Any` vs simple types
- Real-world examples (type-safe containers, API responses)
- 🚀 CoLearning Challenge: "Tell your AI: Create a Stack[T] class with push/pop methods that preserve types"

### Section 3: Integration and Production Patterns (Lessons 5-6)
**Cognitive Load**: Max 10 concepts (integration and synthesis)

**Lesson 5: Pydantic for AI-Native Development**
- Validating LLM outputs (structured generation with Claude)
- Prompt validation (ensuring AI follows your data requirements)
- FastAPI integration (automatic API validation)
- Error handling strategies for AI systems
- Iterative refinement (validation fails → improve prompt → regenerate)
- 💬 AI Colearning: "Ask your AI: Generate JSON for a User model, then validate it with Pydantic"
- ✨ Teaching Tip: "Use Claude Code's structured output mode with your Pydantic models"

**Lesson 6: Capstone - Type-Safe Config Manager**
- **BUILD PROJECT**: Production-quality configuration system
- Requirements: Load config from files, environment variables, CLI args
- Architecture: Pydantic models for validation + Generic containers for type safety
- Features: Nested config sections, environment-specific overrides, validation errors with helpful messages
- Testing: Unit tests for validation logic, integration tests for full config loading
- 🚀 CoLearning Challenge: "Tell your AI: Scaffold a ConfigManager with Settings, Database, and API config models"

### Common Mistakes Section
- Mixing up Pydantic validation (runtime) and type hints (static)
- Forgetting to handle ValidationError (always use try/except)
- Using `Any` instead of proper Generic constraints
- Not using Field() for advanced validation (regex, min/max)
- Overusing custom validators when built-in Field() constraints suffice

### AI Exercise Section (Try With AI)
- Every lesson ends with 4 prompts (Bloom's progression)
- Prompt 1: Remember/Understand (concept exploration)
- Prompt 2: Apply (code generation)
- Prompt 3: Analyze/Evaluate (tradeoff discussions)
- Prompt 4: Synthesize/Create (integration and project work)

---

## Code Examples (Specifications for AI Generation) *(mandatory)*

### Example Set 1: Pydantic Basics (Lesson 1)

**EX-001: Basic Pydantic Model**
- **Purpose**: Introduce BaseModel and automatic validation
- **Complexity**: B1 (simple model, 5 fields)
- **AI Prompt**: "Create a Pydantic model for a Book with title (str), author (str), year (int between 1000-2100), price (float >= 0), and optional ISBN (str | None). Include example usage showing both valid and invalid data."
- **Learning Focus**: Students see validation errors happen automatically

**EX-002: Nested Models**
- **Purpose**: Show model composition
- **Complexity**: B1-B2 (nested structure)
- **AI Prompt**: "Create Pydantic models for Author (name, bio) and Book (title, authors: list[Author], publication_date). Show how to create a Book with multiple authors and validate the nested structure."
- **Learning Focus**: Complex data structures remain type-safe

### Example Set 2: Custom Validation (Lesson 2)

**EX-003: Field Validators**
- **Purpose**: Demonstrate custom validation logic
- **Complexity**: B2 (validator methods)
- **AI Prompt**: "Create a User model with email field. Add a @field_validator that checks: email contains @, domain has at least one dot, no spaces allowed. Show validation passing and failing cases."
- **Learning Focus**: Go beyond built-in validation

**EX-004: Settings from Environment**
- **Purpose**: Production config pattern
- **Complexity**: B2 (BaseSettings integration)
- **AI Prompt**: "Create an AppSettings model using BaseSettings with database_url, api_key (secret), debug_mode (bool, default False). Show how it reads from environment variables and .env file. Include error handling for missing required variables."
- **Learning Focus**: Real-world config management

### Example Set 3: Generics Patterns (Lessons 3-4)

**EX-005: Generic Function**
- **Purpose**: Type-safe utility function
- **Complexity**: B1 (single type parameter)
- **AI Prompt**: "Create a generic function get_first_item[T](items: list[T]) -> T | None that returns the first item or None if empty. Show it working with list[int], list[str], and list[User]."
- **Learning Focus**: One function, many types, full type safety

**EX-006: Generic Container Class**
- **Purpose**: Type-safe data structure
- **Complexity**: B2 (Generic class)
- **AI Prompt**: "Create a Stack[T] class using Generic[T] with push(item: T), pop() -> T | None, and peek() -> T | None methods. Show usage with Stack[int] and Stack[Book] demonstrating type safety in IDE."
- **Learning Focus**: Custom generic classes

**EX-007: Bounded Generic**
- **Purpose**: Constraints on type parameters
- **Complexity**: B2 (type bounds)
- **AI Prompt**: "Create a generic function compare[T: Comparable](a: T, b: T) -> T that returns the larger of two items. Use a Protocol to define Comparable. Show it working with int, float, and custom classes."
- **Learning Focus**: Generics with constraints

### Example Set 4: AI-Native Integration (Lessons 5-6)

**EX-008: LLM Output Validation**
- **Purpose**: Validate AI-generated data
- **Complexity**: B2 (real-world AI pattern)
- **AI Prompt**: "Create a Recipe model (name, ingredients: list[str], steps: list[str], prep_time_minutes: int). Write code that asks Claude Code to generate a recipe, parses the JSON response, validates it with Pydantic, and handles ValidationError by asking for a corrected version."
- **Learning Focus**: AI → validate → iterate loop

**EX-009: Config Manager Capstone**
- **Purpose**: Production integration project
- **Complexity**: B2 (full system)
- **AI Prompt**: "Create a ConfigManager using Pydantic BaseSettings with nested models: DatabaseConfig (host, port, name), APIConfig (base_url, timeout, retry_count), AppConfig (debug, log_level). Add Generic[T] wrapper for typed config access. Include loading from YAML file, environment variables, and CLI args with precedence (CLI > env > file). Handle missing configs with helpful error messages."
- **Learning Focus**: Professional production pattern

---

## Acceptance Criteria (Quality Gates Referencing Evals) *(mandatory)*

### Technical Quality

- [ ] **AC-001**: All code examples run on Python 3.14+ without errors (validates EVAL-004, EVAL-005)
- [ ] **AC-002**: Type hints present on 100% of functions and class methods (validates EVAL-005)
- [ ] **AC-003**: All Pydantic models use Pydantic V2 syntax and features (validates EVAL-004)
- [ ] **AC-004**: Modern PEP 695 Generic syntax used throughout (not legacy `typing.Generic`) (validates EVAL-005)
- [ ] **AC-005**: All code examples tested and produce expected output (validates EVAL-004, EVAL-006)

### Pedagogical Quality

- [ ] **AC-006**: AI-Native Learning pattern applied (describe → explore → validate → learn from errors) throughout all lessons (validates EVAL-009)
- [ ] **AC-007**: Lesson closure pattern followed: ONLY "Try With AI" section at end, NO summaries/checklists after (constitutional requirement)
- [ ] **AC-008**: Each lesson includes 2-4 CoLearning elements (💬🎓🚀✨) positioned throughout content, NOT just at end (validates EVAL-009)
- [ ] **AC-009**: Conversational tone maintained (you, your, we), NOT documentation style (validates EVAL-008)
- [ ] **AC-010**: Exactly 4 "Try With AI" prompts per lesson with explicit expected outcomes and Bloom's progression (validates EVAL-009)

### Content Quality

- [ ] **AC-011**: No forward references to Chapters 30+ (SDD formally taught later) (constitutional requirement)
- [ ] **AC-012**: No "Specification-Driven Development" terminology used (Part 5 content); use "describe your data model" framing instead (validates EVAL-003)
- [ ] **AC-013**: Balanced coverage: both AI-native use cases AND general validation patterns demonstrated (validates EVAL-003)
- [ ] **AC-014**: Generics focused on practical patterns, NOT theoretical variance/contravariance (validates EVAL-002, EVAL-005)
- [ ] **AC-015**: Capstone Config Manager project is production-quality and portfolio-worthy (validates EVAL-006)

### Complexity and Proficiency Alignment

- [ ] **AC-016**: Content aligns with B1-B2 proficiency (independent application → analysis/design) (validates EVAL-001 through EVAL-007)
- [ ] **AC-017**: Max 10 concepts per lesson (advanced tier cognitive load limit) (validates EVAL-008)
- [ ] **AC-018**: Architectural discussions included (when Pydantic vs TypedDict vs dataclasses) (validates EVAL-002)
- [ ] **AC-019**: Production examples (config systems, API validation, LLM output validation) demonstrated (validates EVAL-003, EVAL-007)

### Evals Alignment

- [ ] **AC-020**: All 12 Success Evals (EVAL-001 through EVAL-012) are addressed by chapter content
- [ ] **AC-021**: Assessment opportunities built into lessons (observable via "Try With AI" prompts and capstone project)
- [ ] **AC-022**: Reading level maintains Grade 10-11 (advanced Python students) - validated via readability tools

---

## Assumptions *(optional - documented informed guesses)*

### Technical Assumptions

- **ASSUMPTION-001**: Students have Python 3.14+ installed (or can install via `uv` as taught in Chapter 12)
- **ASSUMPTION-002**: Students have access to Claude Code or Gemini CLI for "Try With AI" exercises (established in Part 2)
- **ASSUMPTION-003**: Students are familiar with JSON data structures from prior web/API exposure
- **ASSUMPTION-004**: Pydantic V2 is available via `uv add pydantic` (current as of 2024-2025)

### Pedagogical Assumptions

- **ASSUMPTION-005**: B1-B2 proficiency students can handle 10 concepts per lesson with proper scaffolding
- **ASSUMPTION-006**: Capstone project suitable for 2-3 hour completion time (based on prior capstone patterns in Ch 19, 21)
- **ASSUMPTION-007**: Students have 3.5-4 hours total for this chapter (industry-standard chapter length for advanced topics)

### Scope Assumptions

- **ASSUMPTION-008**: FastAPI integration mentioned but NOT deeply covered (that's for later agent framework chapters)
- **ASSUMPTION-009**: TypeScript validation libraries (Zod) mentioned as future connection but NOT taught (Part 8)
- **ASSUMPTION-010**: Production deployment patterns (Docker, K8s) mentioned as use cases but NOT taught here (Parts 10-13)

---

## Out of Scope *(optional - explicit exclusions)*

**Explicitly NOT Covered in Chapter 27**:

- **FastAPI framework deep dive** (agent framework chapters later)
- **Advanced Generic theory** (covariance, contravariance, variance rules) - prioritizing practical patterns per user preference
- **TypeScript validation libraries** (Zod, io-ts) - covered in Part 8 (Chapter 38-40)
- **Database ORMs using Pydantic** (SQLModel, Beanie) - database chapters in Part 11
- **Async validation patterns** - Asyncio is Chapter 28
- **Performance optimization** (Pydantic-core Rust implementation details) - advanced topic beyond scope
- **Form validation in web frameworks** - frontend chapters later
- **Migration from Pydantic V1 to V2** - assuming greenfield V2 usage

**Why these exclusions**:
- Keeps chapter focused on core Pydantic + Generics patterns
- Prevents scope creep into future chapters
- Maintains 3.5-4 hour completion time
- Allows depth on fundamentals over breadth on integrations

---

## Dependencies *(optional - external requirements)*

### Library Dependencies

- **Pydantic V2.0+**: Core validation library (`uv add pydantic`)
- **Python 3.14+**: Modern type system features (PEP 695 syntax)
- **python-dotenv** (optional): For `.env` file loading in Settings examples (`uv add python-dotenv`)

### Tool Dependencies

- **Claude Code or Gemini CLI**: For "Try With AI" exercises (taught in Part 2)
- **IDE with type checking**: VS Code with Pylance, PyCharm, or similar (for Generic type hints visualization)
- **uv package manager**: For dependency installation (taught in Chapter 12)

### Knowledge Dependencies

- **Prerequisite Chapters**: 1-26 (especially 24-26 OOP and type system foundations)
- **No external services required**: All examples run locally
- **No API keys needed**: LLM validation examples use local mocking (or optional Claude Code for exploration)

---

## Constraints *(optional - limitations and non-goals)*

### Technical Constraints

- **CONSTRAINT-001**: Must use Python 3.14+ syntax (no legacy compatibility code for 3.9 or earlier)
- **CONSTRAINT-002**: Pydantic V2 ONLY (V1 is deprecated, no migration guide needed)
- **CONSTRAINT-003**: All code must be testable locally without external services

### Pedagogical Constraints

- **CONSTRAINT-004**: Max 10 concepts per lesson (B1-B2 cognitive load limit)
- **CONSTRAINT-005**: No forward references to Chapters 30+ (SDD terminology forbidden)
- **CONSTRAINT-006**: Conversational tone required (not documentation style)
- **CONSTRAINT-007**: Exactly 4 "Try With AI" prompts per lesson (standardization)

### Scope Constraints

- **CONSTRAINT-008**: Chapter must complete in 3.5-4 hours total (industry standard)
- **CONSTRAINT-009**: Capstone project must be completable in 2-3 hours (based on prior chapters)
- **CONSTRAINT-010**: No deep dives into framework integrations (FastAPI, SQLModel) - overview only

### Content Constraints

- **CONSTRAINT-011**: Balanced coverage required: 50% Pydantic, 30% Generics, 20% integration (user preference)
- **CONSTRAINT-012**: AI-native use cases must be prominent (not just general validation)
- **CONSTRAINT-013**: Practical patterns prioritized over theoretical foundations for Generics

---

## Notes *(optional - additional context)*

### Design Decisions

**DECISION-001: Why Pydantic V2 Only?**
- V1 is deprecated (end-of-life announced)
- V2 has 5-50x performance improvements (Rust core)
- V2 API is cleaner and more Pythonic
- All modern projects use V2

**DECISION-002: Why PEP 695 Generic Syntax?**
- Python 3.14+ makes this the standard
- Simpler, more readable than legacy `typing.Generic`
- Future-proof (this is the direction Python is going)
- Better IDE support

**DECISION-003: Why Config Manager as Capstone?**
- Universal pattern (every project needs configuration)
- Combines both Pydantic AND Generics naturally
- Production-relevant (portfolio-worthy)
- Students will reuse this in their own projects

### Pedagogical Rationale

**Graduated Teaching Pattern Application**:
- **Tier 1 (Book Teaches)**: What is validation? What are Generics? Core concepts explained directly
- **Tier 2 (AI Companion)**: Complex model definitions, custom validators (AI generates from requirements)
- **Tier 3 (AI Orchestration)**: Full Config Manager scaffolding (AI creates multi-file structure)

**CoLearning Element Distribution**:
- 💬 AI Colearning Prompts: 2-3 per lesson (concept exploration)
- 🎓 Instructor Commentary: 2-3 per lesson ("syntax cheap, semantics gold" reinforcement)
- 🚀 CoLearning Challenges: 1-2 per lesson (hands-on practice WITH AI)
- ✨ Teaching Tips: 1-2 per lesson (effective tool usage)

**Why This Chapter Matters for Students**:
- **Aspiring Developers**: Learn industry-standard validation (every job uses Pydantic)
- **Professional Developers**: Upgrade skills to modern Python patterns
- **Technical Founders**: Build reliable AI products (bad data = broken products)

### Connection to Future Content

- **Part 5 (SDD)**: Pydantic models ARE specifications for data (formalize this later)
- **Part 8 (TypeScript)**: Zod validation mirrors Pydantic patterns (language-agnostic thinking)
- **Parts 10-13 (Production)**: Config Manager pattern used in all deployment examples
- **Agent Frameworks**: Pydantic is the foundation of FastAPI, LangChain, OpenAI SDK

---

## Validation Notes *(internal - checklist tracking)*

**Initial Spec Quality Check**:
- ✅ No implementation details (no specific framework code)
- ✅ Focused on learning outcomes and student value
- ✅ Written for educational content (adapted from software feature template)
- ✅ All mandatory sections completed
- ✅ Success Evals defined FIRST (before requirements)
- ✅ No [NEEDS CLARIFICATION] markers (all intelligent guesses documented in Assumptions)
- ✅ Evals are measurable (80%+, 75%+, etc.)
- ✅ Success Criteria are technology-agnostic (describe outcomes, not implementations)
- ✅ Requirements translated to Learning Objectives (appropriate for educational content)
- ✅ Dependencies and Constraints explicitly listed
- ✅ Scope boundaries clear (Out of Scope section)

**Ready for**: `/sp.clarify` (if needed) or `/sp.plan` (lesson breakdown)
