# Implementation Plan: Chapter 27 - Pydantic and Generics

**Branch**: `001-part-4-chapter-27` | **Date**: 2025-11-09 | **Spec**: [spec.md](./spec.md)
**Input**: Chapter specification from `/specs/001-part-4-chapter-27/spec.md`

**Note**: This plan provides detailed lesson-by-lesson breakdown with CEFR proficiency mapping, skills metadata, and CoLearning element placement for Chapter 27: "Pydantic and Generics" (Part 4, Advanced tier B1-B2).

---

## Summary

Chapter 27 teaches advanced type safety and data validation for AI-native Python development through Pydantic (runtime validation) and Generics (static type safety). Students learn to validate LLM outputs, build type-safe containers, and combine both patterns in a production-quality Config Manager. Balanced coverage: 50% Pydantic fundamentals, 30% Generics patterns, 20% integration. Complexity tier: B1-B2 (independent application → analysis/design), 6 lessons, 9 code examples, capstone project.

**Technical Approach**:
- Pydantic V2 for runtime validation (BaseModel, validators, Settings)
- Modern PEP 695 Generic syntax (Python 3.14+)
- AI-Native Learning pedagogy (describe → explore → validate → iterate)
- Capstone: Type-Safe Config Manager combining both patterns

---

## Technical Context

**Language/Version**: Python 3.14+ (PEP 695 Generic syntax, modern type system)
**Primary Dependencies**: Pydantic V2.0+, python-dotenv (optional for .env loading)
**Storage**: N/A (local file-based config examples)
**Testing**: pytest for code example validation, unit tests in capstone
**Target Platform**: Cross-platform Python development (local execution)
**Project Type**: Educational content (book chapter with interactive exercises)
**Performance Goals**: Code examples execute instantly (<100ms), appropriate for learning environment
**Constraints**: Max 10 concepts per lesson (B1-B2 cognitive load), 3.5-4 hour chapter completion time
**Scale/Scope**: 6 lessons, 9 code examples, 24 "Try With AI" prompts (4 per lesson), 1 capstone project

---

## Constitution Check

**GATE**: Educational content for AI-Native Python book - Constitution v3.0.0 compliance verified.

### ✅ Passed Constitution Requirements

- **Principle 12-13 (Graduated Complexity)**: Chapter uses Advanced tier (B1-B2) appropriate for Part 4, Chapters 24-29
- **Rule 8 (Standardized "Try With AI")**: 4 prompts per lesson, Bloom's progression, explicit expected outcomes
- **Rule 9 (AI-Native CoLearning)**: 2-4 CoLearning elements (💬🎓🚀✨) throughout each lesson, NOT just at end
- **Rule 6 (Lesson Closure)**: ONLY "Try With AI" section at end, NO summaries/checklists after
- **Rule 7 (Pedagogical Ordering)**: No forward references to Chapters 30+ (SDD formally introduced in Part 5)
- **Python Standards**: 3.14+ syntax, 100% type hints, PEP 695 Generics, Pydantic V2 only
- **Evals-First**: Success criteria (EVAL-001 through EVAL-012) defined in spec BEFORE planning
- **Complexity Management**: Max 10 concepts per lesson enforced

### 🚫 Explicitly Avoided

- ❌ No "Specification-Driven Development" terminology (Part 5 content)
- ❌ No forward references to FastAPI deep dives (later chapters)
- ❌ No theoretical Generic variance (practical patterns only)
- ❌ No Pydantic V1 migration content (greenfield V2 only)

---

## Chapter Structure

### Documentation (this chapter)

```text
specs/001-part-4-chapter-27/
├── spec.md              # Feature specification (COMPLETE)
├── plan.md              # This file (lesson-by-lesson breakdown)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT YET CREATED)
```

### Content Output (book repository)

```text
book-source/docs/04-Part-4-Python-Fundamentals/27-pydantic-generics/
├── 01-introduction-to-pydantic.md              # Lesson 1 (B1)
├── 02-advanced-pydantic-patterns.md            # Lesson 2 (B1-B2)
├── 03-introduction-to-generics.md              # Lesson 3 (B1)
├── 04-generic-classes-and-protocols.md         # Lesson 4 (B1-B2)
├── 05-pydantic-for-ai-native-development.md    # Lesson 5 (B1-B2 → B2)
└── 06-capstone-type-safe-config-manager.md     # Lesson 6 (B2)
```

**Structure Decision**: Standard Part 4 chapter layout following established patterns from Chapters 14-21. Each lesson is a standalone markdown file with YAML frontmatter, Docusaurus navigation, and "Try With AI" closure.

---

## Proficiency Progression Overview

### Chapter-Level Proficiency Arc (B1 → B2)

```
Lesson 1 (Pydantic Intro)      → B1 (Apply basic models with guidance)
Lesson 2 (Advanced Pydantic)   → B1-B2 (Design custom validators)
Lesson 3 (Generics Intro)      → B1 (Apply generic functions)
Lesson 4 (Generic Classes)     → B1-B2 (Create generic classes)
Lesson 5 (AI-Native Integration) → B1-B2 → B2 (Analyze validation strategies)
Lesson 6 (Capstone)            → B2 (Synthesize production system)
```

**Rationale**: Graduated progression from guided application (B1) through independent design (B1-B2) to professional synthesis (B2). Aligns with Bloom's taxonomy: Apply → Analyze → Evaluate → Create.

---

## Lesson-by-Lesson Breakdown

### Lesson 1: Introduction to Pydantic and Data Validation

**CEFR Proficiency Level**: B1 (Independent User - Application)
**Bloom's Cognitive Level**: Understand → Apply
**Estimated Time**: 35-40 minutes
**File**: `01-introduction-to-pydantic.md`

#### Learning Objectives (Lesson-Specific)

Students can:
1. **EXPLAIN** the difference between type hints (static documentation) and Pydantic validation (runtime enforcement)
2. **CREATE** basic Pydantic models with 3-5 fields using built-in validators
3. **HANDLE** ValidationError exceptions and understand error messages
4. **APPLY** Pydantic to validate simple data structures (user profiles, book records)

#### Skills Taught (Proficiency Metadata)

| Skill Name | CEFR Level | Category | Measurable at This Level |
|------------|------------|----------|-------------------------|
| Data Validation Concepts | B1 | Conceptual | Student can explain why runtime validation is needed and give 2-3 examples |
| Pydantic BaseModel Usage | B1 | Technical | Student can create a working model with 5 fields and validate input data |
| Exception Handling (ValidationError) | B1 | Technical | Student can catch ValidationError and display helpful error messages |
| Type Hints Application | B1 | Technical | Student can apply correct type hints (str, int, float, list[str]) to model fields |

#### Concepts Introduced (Max 10)

1. **Runtime vs Static Validation** - Type hints document, Pydantic enforces
2. **Pydantic BaseModel** - Base class for creating validated models
3. **Automatic Validation** - Field types automatically checked
4. **ValidationError** - Exception raised on invalid data
5. **Field Types** - str, int, float, bool, list[T], dict[K, V]
6. **Optional Fields** - `X | None` syntax for nullable fields
7. **Default Values** - `field: str = "default"` pattern
8. **Model Instantiation** - Creating instances with `Model(field=value)`
9. **Installing Pydantic** - `uv add pydantic` (connects to Ch 12)
10. **Error Messages** - Understanding Pydantic's helpful validation feedback

**Cognitive Load Check**: ✅ 10 concepts (at limit for B1-B2 tier)

#### Content Structure

**Introduction** (5 min)
- The validation problem: "How do you know your data is correct?"
- Real-world scenario: AI generates JSON, you need to validate it BEFORE using it
- Type hints say "this SHOULD be int" → Pydantic enforces "this MUST be int"

**Section 1: Your First Pydantic Model** (10 min)
- Installing Pydantic with `uv add pydantic`
- Creating Book model (title, author, year, price, ISBN)
- Running validation with valid and invalid data
- 💬 **AI Colearning Prompt**: "Ask your AI: What happens if I pass a string to an int field in Pydantic?"
- 🎓 **Instructor Commentary**: "Type hints are suggestions; Pydantic makes them laws"

**Section 2: Understanding Validation Errors** (10 min)
- Reading ValidationError messages
- Multiple validation errors at once
- Field-specific error details
- ✨ **Teaching Tip**: "Use try/except blocks to gracefully handle bad data in production"

**Section 3: Nested Models** (10 min)
- Composing models (Author → Book)
- Validating complex structures
- 🚀 **CoLearning Challenge**: "Tell your AI: Create an Author model and connect it to Book"

**Common Mistakes** (3 min)
- Forgetting to inherit from BaseModel
- Not handling ValidationError (crashes your program!)
- Mixing up type hints (list vs list[str])

**Try With AI** (Lesson Closure - 4 Prompts)

1. **Prompt 1** (Understand): "Ask your AI to explain the difference between Python's built-in type hints and Pydantic's validation. Request 2-3 examples."
   - **Expected Outcome**: Explanation distinguishing static (type hints) vs runtime (Pydantic) with examples like IDE autocomplete vs preventing bad API data.

2. **Prompt 2** (Apply): "Tell your AI: Create a Pydantic model for a Product with name, price (must be >= 0), quantity (integer), and optional description. Generate code that tests both valid and invalid inputs."
   - **Expected Outcome**: Working Product model with validation, test cases showing price validation and optional field handling.

3. **Prompt 3** (Analyze): "Ask your AI: When would you use Pydantic instead of just type hints? Give 3 scenarios where runtime validation is critical."
   - **Expected Outcome**: Scenarios like API input validation, config file parsing, LLM output validation with justification for each.

4. **Prompt 4** (Create): "Tell your AI: Build a UserProfile model with nested Address model (street, city, zip_code). Include validation that zip_code is exactly 5 digits. Create test data."
   - **Expected Outcome**: Nested UserProfile + Address models with custom zip_code validation, example data demonstrating nesting and validation.

#### Code Examples Assigned

- **EX-001: Basic Pydantic Model** (Book example)
- **EX-002: Nested Models** (Author + Book)

#### CoLearning Elements Positioned

- 💬 AI Colearning Prompts: 2 (mid-section concept exploration)
- 🎓 Instructor Commentary: 2 (key insights about validation philosophy)
- 🚀 CoLearning Challenges: 1 (hands-on nested model creation)
- ✨ Teaching Tips: 1 (error handling best practice)

---

### Lesson 2: Advanced Pydantic Patterns

**CEFR Proficiency Level**: B1-B2 (Transition - Independent Application → Design)
**Bloom's Cognitive Level**: Apply → Analyze
**Estimated Time**: 40-45 minutes
**File**: `02-advanced-pydantic-patterns.md`

#### Learning Objectives (Lesson-Specific)

Students can:
1. **CREATE** custom field validators using `@field_validator` decorator
2. **DESIGN** Pydantic models with Field() constraints (min/max, regex, examples)
3. **IMPLEMENT** settings management using BaseSettings for environment variables
4. **ANALYZE** when to use custom validators vs built-in Field() constraints

#### Skills Taught (Proficiency Metadata)

| Skill Name | CEFR Level | Category | Measurable at This Level |
|------------|------------|----------|-------------------------|
| Custom Validation Logic | B1-B2 | Technical | Student can write @field_validator for email format validation without template |
| Field Constraints Design | B1-B2 | Technical | Student can apply Field() with min/max/regex to enforce business rules |
| Environment Config Management | B2 | Technical | Student can create BaseSettings model that reads from .env and validates config |
| Validation Strategy Selection | B2 | Conceptual | Student can justify choosing custom validator vs Field() constraint for 3 scenarios |

#### Concepts Introduced (Max 10)

1. **@field_validator Decorator** - Custom validation functions
2. **Field() Function** - Advanced field configuration
3. **Field Constraints** - min_length, max_length, ge (>=), le (<=), regex
4. **Validator Mode** - 'before', 'after', 'wrap' validation timing
5. **Model Validators** - `@model_validator` for cross-field validation
6. **BaseSettings** - Environment variable integration
7. **.env Files** - Configuration management pattern
8. **Secret Fields** - Field(repr=False) for sensitive data
9. **Field Examples** - Field(examples=[...]) for API documentation
10. **Validation Context** - Passing context to validators

**Cognitive Load Check**: ✅ 10 concepts (at limit for B1-B2 tier)

#### Content Structure

**Introduction** (5 min)
- Beyond basic validation: business rules and real-world constraints
- When built-in validators aren't enough
- Production config management patterns

**Section 1: Custom Field Validators** (12 min)
- Creating @field_validator for email validation
- Validation logic: must contain @, domain has dot, no spaces
- Multiple validators on same field
- 💬 **AI Colearning Prompt**: "Ask your AI: Show me 3 different ways to validate an email in Pydantic, from simple to complex"
- 🎓 **Instructor Commentary**: "Custom validators = enforcing YOUR business rules, not just type correctness"

**Section 2: Field Constraints** (10 min)
- Using Field() for min/max values, lengths, regex patterns
- When Field() suffices vs when you need custom validators
- Combining Field() constraints with custom validators
- ✨ **Teaching Tip**: "Start with Field() constraints; add custom validators only when you need logic"

**Section 3: Model-Level Validation** (8 min)
- Cross-field validation with @model_validator
- Example: password confirmation matching
- Root validators for complex business rules
- 🚀 **CoLearning Challenge**: "Tell your AI: Create a PasswordChange model where new_password != old_password and new_password == confirm_password"

**Section 4: Settings Management** (10 min)
- BaseSettings for environment variables
- Loading from .env files
- Required vs optional config
- Secrets management (Field(repr=False))
- 💬 **AI Colearning Prompt**: "Ask your AI: Generate a DatabaseSettings model that reads host, port, username, password from environment"

**Common Mistakes** (3 min)
- Overusing custom validators when Field() suffices
- Not validating early (validate at boundaries!)
- Hardcoding secrets instead of using BaseSettings
- Forgetting mode parameter in @field_validator

**Try With AI** (Lesson Closure - 4 Prompts)

1. **Prompt 1** (Understand): "Ask your AI to explain when you'd use Field() constraints vs @field_validator. Request a decision tree or comparison table."
   - **Expected Outcome**: Comparison showing Field() for simple constraints (min/max), @field_validator for logic (format checking, external validation).

2. **Prompt 2** (Apply): "Tell your AI: Create a User model with email (@field_validator checking format), age (must be 13-120), and bio (max 500 chars). Use Field() where appropriate and custom validators where needed."
   - **Expected Outcome**: Working User model with mixed validation approaches, demonstrating correct tool selection.

3. **Prompt 3** (Analyze): "Ask your AI: Why is BaseSettings better than manually reading environment variables with os.getenv()? What problems does it solve?"
   - **Expected Outcome**: Explanation covering type validation, required field enforcement, .env file support, testing benefits.

4. **Prompt 4** (Create): "Tell your AI: Build an AppSettings model with DatabaseConfig (nested model), API_KEY (secret field), DEBUG (bool, default False), and LOG_LEVEL (must be 'DEBUG', 'INFO', 'WARNING', 'ERROR'). Include .env example."
   - **Expected Outcome**: Production-quality settings model with nesting, secrets, enums, and .env file example.

#### Code Examples Assigned

- **EX-003: Field Validators** (email validation)
- **EX-004: Settings from Environment** (BaseSettings pattern)

#### CoLearning Elements Positioned

- 💬 AI Colearning Prompts: 2 (concept exploration and code generation)
- 🎓 Instructor Commentary: 2 (validation philosophy and best practices)
- 🚀 CoLearning Challenges: 1 (cross-field validation exercise)
- ✨ Teaching Tips: 1 (when to use which validation approach)

---

### Lesson 3: Introduction to Generics and Type Variables

**CEFR Proficiency Level**: B1 (Independent User - Application)
**Bloom's Cognitive Level**: Understand → Apply
**Estimated Time**: 35-40 minutes
**File**: `03-introduction-to-generics.md`

#### Learning Objectives (Lesson-Specific)

Students can:
1. **EXPLAIN** what Generics are and why they enable type-safe reusable code
2. **CREATE** generic functions using TypeVar and modern PEP 695 syntax
3. **APPLY** generic functions to multiple data types while preserving type information
4. **UNDERSTAND** how Generics improve IDE autocomplete and error detection

#### Skills Taught (Proficiency Metadata)

| Skill Name | CEFR Level | Category | Measurable at This Level |
|------------|------------|----------|-------------------------|
| Generic Function Creation | B1 | Technical | Student can write a generic function that works with list[int], list[str], list[User] |
| TypeVar Usage | B1 | Technical | Student can create TypeVar and use it in function signatures |
| PEP 695 Syntax | B1 | Technical | Student can write `def func[T](x: T) -> T:` instead of legacy Generic syntax |
| Type Safety Understanding | B1 | Conceptual | Student can explain how Generics help IDEs catch bugs before running code |

#### Concepts Introduced (Max 10)

1. **Generics Definition** - Code that works with multiple types safely
2. **Type Variables (TypeVar)** - Placeholder for any type
3. **Generic Functions** - Functions that work with any type
4. **PEP 695 Syntax** - Modern `[T]` syntax (Python 3.14+)
5. **Type Preservation** - Generics maintain type information through function calls
6. **IDE Benefits** - Autocomplete and error detection with Generics
7. **Type Inference** - Python infers T from usage
8. **Legacy vs Modern** - typing.TypeVar vs PEP 695 syntax
9. **Type Safety** - Compile-time (IDE) vs runtime checking
10. **Reusability** - Write once, use with any type

**Cognitive Load Check**: ✅ 10 concepts (at limit for B1-B2 tier)

#### Content Structure

**Introduction** (5 min)
- The duplication problem: `get_first_int`, `get_first_str`, `get_first_user`... → ONE generic function
- What are Generics? Type-safe code that works with ANY type
- Why Python needs Generics (dynamic typing + static analysis tools)

**Section 1: Your First Generic Function** (12 min)
- Creating `get_first_item[T](items: list[T]) -> T | None`
- Testing with different types: list[int], list[str], list[Book]
- How IDEs use type information for autocomplete
- 💬 **AI Colearning Prompt**: "Ask your AI: What's the difference between using Generics and using `Any` type in Python?"
- 🎓 **Instructor Commentary**: "Generics preserve type information; `Any` throws it away. Generics = safety, Any = wild west"

**Section 2: Modern PEP 695 Syntax** (10 min)
- Legacy approach: `TypeVar('T')` from typing module
- Modern approach: `def func[T](x: T) -> T:`
- Why PEP 695 is cleaner and more readable
- ✨ **Teaching Tip**: "Always use PEP 695 syntax in Python 3.14+. It's simpler and future-proof"

**Section 3: Type Inference** (8 min)
- Python infers T from usage context
- Explicit type parameters (rarely needed)
- How type checkers (mypy, Pylance) use Generics
- 🚀 **CoLearning Challenge**: "Tell your AI: Create a generic function `get_last_item[T]` that returns the last element or None"

**Section 4: Generics vs Dynamic Typing** (5 min)
- Generics are for tools (IDEs), not runtime
- Python still dynamically typed at runtime
- Benefits: catch errors before running code
- 💬 **AI Colearning Prompt**: "Ask your AI: Create a generic function with a bug where types don't match, then show how an IDE would catch it"

**Common Mistakes** (3 min)
- Using `Any` when Generics are better
- Over-constraining with unnecessary type bounds
- Thinking Generics enforce runtime type checking (they don't!)
- Mixing legacy and modern syntax

**Try With AI** (Lesson Closure - 4 Prompts)

1. **Prompt 1** (Understand): "Ask your AI: Why do we need Generics when Python is dynamically typed? Explain with 2-3 concrete examples of bugs Generics help prevent."
   - **Expected Outcome**: Explanation showing IDE autocomplete benefits, type mismatch detection, and refactoring safety.

2. **Prompt 2** (Apply): "Tell your AI: Create a generic function `find_item[T](items: list[T], predicate: callable) -> T | None` that finds the first item matching a condition. Show usage with int and User types."
   - **Expected Outcome**: Working generic search function with examples showing type preservation across different types.

3. **Prompt 3** (Analyze): "Ask your AI: Compare using Generics vs using `Any` type. What are the tradeoffs? When would you use each?"
   - **Expected Outcome**: Comparison table showing Generics preserve type safety, Any provides flexibility; use cases for each.

4. **Prompt 4** (Create): "Tell your AI: Write a generic function `merge_sorted_lists[T](list1: list[T], list2: list[T]) -> list[T]` that combines two sorted lists. Include type hints and demonstrate with both numbers and strings."
   - **Expected Outcome**: Generic merge function with proper type preservation, examples showing it works identically with int and str.

#### Code Examples Assigned

- **EX-005: Generic Function** (get_first_item example)

#### CoLearning Elements Positioned

- 💬 AI Colearning Prompts: 2 (concept exploration)
- 🎓 Instructor Commentary: 2 (type safety philosophy)
- 🚀 CoLearning Challenges: 1 (hands-on function creation)
- ✨ Teaching Tips: 1 (modern syntax preference)

---

### Lesson 4: Generic Classes and Protocols

**CEFR Proficiency Level**: B1-B2 (Transition - Application → Design)
**Bloom's Cognitive Level**: Apply → Analyze → Evaluate
**Estimated Time**: 40-45 minutes
**File**: `04-generic-classes-and-protocols.md`

#### Learning Objectives (Lesson-Specific)

Students can:
1. **CREATE** generic classes using `Generic[T]` for type-safe containers
2. **IMPLEMENT** classes with multiple type parameters (e.g., `Generic[K, V]`)
3. **APPLY** bounded type variables to constrain what types are allowed
4. **DESIGN** Protocols for structural subtyping in generic contexts
5. **EVALUATE** when Generics add value vs when simpler approaches suffice

#### Skills Taught (Proficiency Metadata)

| Skill Name | CEFR Level | Category | Measurable at This Level |
|------------|------------|----------|-------------------------|
| Generic Class Design | B1-B2 | Technical | Student can create Stack[T] or Queue[T] class with full type safety |
| Multiple Type Parameters | B2 | Technical | Student can design Cache[K, V] with different key/value types |
| Bounded Generics | B2 | Technical | Student can constrain T to specific base classes or protocols |
| Protocol Definition | B2 | Technical | Student can create Protocol for structural typing (e.g., Comparable) |
| Architectural Decision-Making | B2 | Conceptual | Student can justify when to use Generics vs simpler alternatives |

#### Concepts Introduced (Max 10)

1. **Generic Classes** - Classes that work with parameterized types
2. **Generic[T] Base** - Inheriting from Generic to create generic classes
3. **Multiple Type Parameters** - `Generic[K, V]`, `Generic[T, U]`
4. **Type Bounds** - `T: SomeClass` or `bound=SomeClass`
5. **Protocols** - Structural subtyping without inheritance
6. **Protocol as Bound** - Using Protocols to constrain type variables
7. **Variance (Intro Only)** - Why Stack[Dog] isn't Stack[Animal] (mention only, no deep dive)
8. **Type Safety in Methods** - Methods preserve generic type
9. **Real-World Containers** - Stack, Queue, Cache patterns
10. **When NOT to Use Generics** - Overengineering warning

**Cognitive Load Check**: ✅ 10 concepts (at limit for B1-B2 tier)

#### Content Structure

**Introduction** (5 min)
- From generic functions to generic classes
- Real-world need: type-safe Stack[Book] vs Stack[User]
- How generic containers prevent mixing types accidentally

**Section 1: Creating a Generic Stack Class** (12 min)
- Defining `class Stack[T]:` with push/pop/peek
- Type safety: `Stack[int]` won't accept strings
- IDE autocomplete based on type parameter
- 💬 **AI Colearning Prompt**: "Ask your AI: Create a generic Queue[T] class with enqueue/dequeue methods. Show usage with both strings and custom objects."
- 🎓 **Instructor Commentary**: "Generic classes are blueprints with type parameters. Stack[int] and Stack[str] are different types to your IDE"

**Section 2: Multiple Type Parameters** (10 min)
- Creating `class Cache[K, V]:` with get/set
- Key type and value type are independent
- Real-world example: `Cache[str, User]`, `Cache[int, Book]`
- ✨ **Teaching Tip**: "Use descriptive names: K for key, V for value, T for general type"

**Section 3: Bounded Type Variables** (8 min)
- Constraining types with bounds
- Example: `T: Comparable` ensures T can be compared
- Using Protocols to define interfaces
- 🚀 **CoLearning Challenge**: "Tell your AI: Create a generic `find_max[T: Comparable](items: list[T]) -> T` function"

**Section 4: Protocols for Structural Typing** (10 min)
- What are Protocols? (duck typing formalized)
- Creating Comparable Protocol
- Why Protocols are better than inheritance for Generics
- 💬 **AI Colearning Prompt**: "Ask your AI: Explain the difference between Protocol and abstract base class. When would you use each?"
- 🎓 **Instructor Commentary**: "Protocols = 'acts like a duck'. Inheritance = 'is a duck'. Protocols are more flexible"

**Section 5: When NOT to Use Generics** (5 min)
- Overengineering warning: simple is better
- If you only have one type, don't genericize
- Balance type safety vs code complexity
- Real decision: "Will I actually use this with multiple types?"

**Common Mistakes** (3 min)
- Using Generics when you'll only ever use one type
- Not constraining when you need specific methods (missing bounds)
- Confusing Generic[T] (for defining) with T (for using)
- Overthinking variance (save for advanced study)

**Try With AI** (Lesson Closure - 4 Prompts)

1. **Prompt 1** (Understand): "Ask your AI: Explain how generic classes work in Python with a Stack[T] example. Show how type parameters flow through all methods."
   - **Expected Outcome**: Explanation with code showing how `push(item: T)` and `pop() -> T | None` maintain type consistency.

2. **Prompt 2** (Apply): "Tell your AI: Create a generic Repository[T] class with add(item: T), find_by_id(id: int) -> T | None, and get_all() -> list[T] methods. Show usage with User and Product types."
   - **Expected Outcome**: Working Repository class demonstrating CRUD operations with type safety across multiple types.

3. **Prompt 3** (Analyze): "Ask your AI: When should you add type bounds to a generic class? Give 3 scenarios where bounds are necessary and 2 where they're not."
   - **Expected Outcome**: Scenarios showing bounds needed for operations (comparison, serialization) vs bounds unnecessary for storage-only.

4. **Prompt 4** (Evaluate + Create): "Tell your AI: Design a generic PriorityQueue[T: Comparable] class. Justify why Comparable bound is needed. Implement with a concrete example using custom objects."
   - **Expected Outcome**: PriorityQueue implementation with Comparable Protocol, explanation of why bound is required for priority comparison, working example with custom class.

#### Code Examples Assigned

- **EX-006: Generic Container Class** (Stack[T] example)
- **EX-007: Bounded Generic** (Comparable Protocol example)

#### CoLearning Elements Positioned

- 💬 AI Colearning Prompts: 2 (concept exploration and architectural comparison)
- 🎓 Instructor Commentary: 2 (type philosophy and Protocol vs inheritance)
- 🚀 CoLearning Challenges: 1 (bounded generic function creation)
- ✨ Teaching Tips: 1 (naming conventions)

---

### Lesson 5: Pydantic for AI-Native Development

**CEFR Proficiency Level**: B1-B2 → B2 (Transition to Advanced Analysis)
**Bloom's Cognitive Level**: Apply → Analyze → Evaluate
**Estimated Time**: 40-45 minutes
**File**: `05-pydantic-for-ai-native-development.md`

#### Learning Objectives (Lesson-Specific)

Students can:
1. **APPLY** Pydantic to validate LLM-generated JSON outputs from Claude Code
2. **IMPLEMENT** iterative refinement: validation fails → improve prompt → regenerate
3. **ANALYZE** validation error patterns to improve AI prompts
4. **EVALUATE** when Pydantic validation adds value in AI pipelines
5. **INTEGRATE** Pydantic with FastAPI for automatic API validation (overview only)

#### Skills Taught (Proficiency Metadata)

| Skill Name | CEFR Level | Category | Measurable at This Level |
|------------|------------|----------|-------------------------|
| LLM Output Validation | B2 | Technical | Student can validate structured AI output, detect failures, request regeneration |
| Prompt Engineering for Validation | B2 | Technical | Student can improve prompts based on validation errors to get correct format |
| Error-Driven Iteration | B2 | Soft Skill | Student can analyze ValidationError, modify approach, and retry (AI-native loop) |
| AI Pipeline Design | B2 | Conceptual | Student can explain where validation fits in: prompt → generate → validate → use/retry |
| FastAPI Integration (Intro) | B1-B2 | Technical | Student can recognize how Pydantic enables automatic API validation (not full implementation) |

#### Concepts Introduced (Max 10)

1. **Structured AI Output** - Getting LLMs to return JSON matching your schema
2. **Validation-First Pattern** - ALWAYS validate AI output before using it
3. **Iterative Refinement Loop** - Generate → Validate → Fix Prompt → Regenerate
4. **Prompt Requirements** - How to tell AI what format you need
5. **Error Pattern Analysis** - Common LLM validation failures
6. **Graceful Degradation** - Handling validation failures in production
7. **FastAPI Integration** - Automatic request/response validation (intro)
8. **Pydantic as Specification** - Models are runnable specs (foreshadowing Part 5)
9. **AI-Native Development Workflow** - Validation as core skill, not afterthought
10. **Production Safety** - Never trust AI output without validation

**Cognitive Load Check**: ✅ 10 concepts (at limit for B1-B2 tier)

#### Content Structure

**Introduction** (5 min)
- The AI trust problem: LLMs are probabilistic, not deterministic
- Why validation is CRITICAL in AI systems (production safety)
- The AI-native loop: describe → generate → validate → iterate
- 🎓 **Instructor Commentary**: "AI is powerful but unpredictable. Pydantic is your safety net"

**Section 1: Validating LLM Outputs** (12 min)
- Creating a Recipe model (name, ingredients, steps, prep_time)
- Asking Claude Code to generate JSON matching the model
- Parsing and validating with Pydantic
- Handling ValidationError gracefully
- 💬 **AI Colearning Prompt**: "Ask your AI: Generate a Recipe as JSON, then validate it with Pydantic. Show what happens when the format is wrong."
- ✨ **Teaching Tip**: "Use `.model_validate_json(json_string)` for direct JSON parsing"

**Section 2: Iterative Refinement Pattern** (10 min)
- First attempt: vague prompt → validation fails
- Analyzing error: "expected int for prep_time, got string"
- Improved prompt: "prep_time must be an integer representing minutes"
- Success: validation passes
- 🚀 **CoLearning Challenge**: "Tell your AI to generate a User profile. If validation fails, improve the prompt and try again. Show both attempts."

**Section 3: Error Pattern Analysis** (8 min)
- Common LLM mistakes: wrong types, missing fields, extra fields
- Using Pydantic's `Field(examples=[...])` to guide LLMs
- Showing examples in prompts
- Why AI sometimes generates {"prep_time": "30 minutes"} instead of 30
- 💬 **AI Colearning Prompt**: "Ask your AI: What are the top 3 validation errors you see when validating LLM outputs? How do you prevent them?"

**Section 4: FastAPI Integration (Overview)** (7 min)
- FastAPI uses Pydantic for automatic validation
- Request bodies validated before your code runs
- Response models enforce output structure
- Preview of agent API development (later chapters)
- 🎓 **Instructor Commentary**: "FastAPI + Pydantic = automatic API contracts. You describe data, FastAPI validates it"

**Section 5: Production Patterns** (5 min)
- Always use try/except for ValidationError
- Log validation failures for debugging
- Retry with improved prompts (up to N times)
- Fallback to human intervention when AI can't produce valid output
- ✨ **Teaching Tip**: "In production, validation failures are EXPECTED. Design for them, don't just crash"

**Common Mistakes** (3 min)
- Using AI output without validation (dangerous!)
- Not giving LLM examples of correct format
- Giving up after first validation failure (iterate!)
- Overcomplicating prompts (start simple, add detail only when needed)

**Try With AI** (Lesson Closure - 4 Prompts)

1. **Prompt 1** (Understand): "Ask your AI: Explain the 'Generate → Validate → Iterate' loop in AI-native development. Why is validation essential rather than optional?"
   - **Expected Outcome**: Explanation of iterative refinement with safety rationale (AI is probabilistic, validation catches errors before they cause damage).

2. **Prompt 2** (Apply): "Tell your AI: Create a BlogPost model (title, author, content, tags: list[str], published_date). Ask AI to generate a sample blog post as JSON. Validate it with Pydantic and handle any errors."
   - **Expected Outcome**: BlogPost model, AI-generated JSON, validation code with error handling, working example.

3. **Prompt 3** (Analyze): "Ask your AI: You're validating LLM outputs for a Recipe model, and you keep getting ValidationError on prep_time (AI gives strings like '30 minutes' instead of integers). How would you modify your prompt to fix this?"
   - **Expected Outcome**: Analysis of the problem, improved prompt with explicit format requirements, possibly Field(examples=[30, 45, 60]) in model.

4. **Prompt 4** (Evaluate + Create): "Tell your AI: Design a validation strategy for an AI agent that generates structured data 1000 times per day. How would you handle failures? What metrics would you track? Implement a basic RetryValidator class that attempts validation up to 3 times with prompt improvements."
   - **Expected Outcome**: Production-quality retry logic, error tracking strategy, RetryValidator implementation, discussion of monitoring (success rate, common errors).

#### Code Examples Assigned

- **EX-008: LLM Output Validation** (Recipe generation and validation)

#### CoLearning Elements Positioned

- 💬 AI Colearning Prompts: 2 (hands-on validation and error analysis)
- 🎓 Instructor Commentary: 2 (AI safety philosophy and FastAPI preview)
- 🚀 CoLearning Challenges: 1 (iterative refinement exercise)
- ✨ Teaching Tips: 2 (parsing method and production error handling)

---

### Lesson 6: Capstone - Type-Safe Config Manager

**CEFR Proficiency Level**: B2 (Advanced Independent User - Synthesis)
**Bloom's Cognitive Level**: Evaluate → Create
**Estimated Time**: 60-90 minutes (extended capstone)
**File**: `06-capstone-type-safe-config-manager.md`

#### Learning Objectives (Lesson-Specific)

Students can:
1. **SYNTHESIZE** Pydantic validation + Generic containers into production-quality system
2. **DESIGN** multi-source config loading with precedence (file < env < CLI)
3. **EVALUATE** architectural tradeoffs (Pydantic vs TypedDict vs dataclasses)
4. **CREATE** portfolio-worthy project demonstrating professional config management
5. **JUSTIFY** design decisions with reference to production requirements

#### Skills Taught (Proficiency Metadata)

| Skill Name | CEFR Level | Category | Measurable at This Level |
|------------|------------|----------|-------------------------|
| System Architecture Design | B2 | Conceptual | Student can design multi-component system with clear responsibilities |
| Pydantic + Generics Integration | B2 | Technical | Student can combine Pydantic models with Generic containers for type-safe access |
| Configuration Precedence | B2 | Technical | Student can implement layered config: defaults < file < env < CLI |
| Production Code Quality | B2 | Technical | Student can write code with error handling, logging, tests, documentation |
| Architectural Justification | B2 | Soft Skill | Student can explain WHY each design decision was made with reference to requirements |

#### Concepts Introduced (Max 10)

1. **Config Manager Pattern** - Centralized configuration system
2. **Multi-Source Loading** - YAML files, environment variables, CLI args
3. **Precedence Rules** - CLI > env > file > defaults
4. **Nested Config Models** - DatabaseConfig, APIConfig, AppConfig
5. **Type-Safe Access** - Generic[T] wrapper for typed config retrieval
6. **Graceful Defaults** - Sensible fallbacks for missing config
7. **Validation at Boundaries** - Validate config on load, fail fast
8. **Environment-Specific Overrides** - dev vs staging vs production configs
9. **Testing Config Systems** - Unit tests with mock files and env vars
10. **Production Patterns** - Error messages, logging, documentation

**Cognitive Load Check**: ✅ 10 concepts (at limit for B1-B2 tier, appropriate for capstone synthesis)

#### Content Structure

**Introduction** (7 min)
- The configuration problem: every app needs settings
- Requirements: multiple sources, validation, type safety, testability
- What you'll build: Production-quality ConfigManager
- 🎓 **Instructor Commentary**: "Config management is unglamorous but essential. This pattern will save you hours on every project"

**Section 1: Requirements and Architecture** (10 min)
- Functional requirements: load from files, env, CLI
- Non-functional requirements: type-safe, validated, tested
- Architecture diagram: ConfigLoader → Pydantic Validation → Generic Access
- Design decisions: why Pydantic? why BaseSettings? why Generic wrapper?
- 💬 **AI Colearning Prompt**: "Ask your AI: Compare Pydantic BaseSettings vs manually reading environment variables. What are the tradeoffs?"

**Section 2: Defining Config Models** (15 min)
- Creating DatabaseConfig (host, port, name, user, password)
- Creating APIConfig (base_url, timeout, retry_count)
- Creating AppConfig (debug, log_level, database, api)
- Nested model composition
- 🚀 **CoLearning Challenge**: "Tell your AI: Scaffold the three config models with realistic defaults and validation constraints"

**Section 3: Multi-Source Loading** (15 min)
- Loading from YAML file (PyYAML or TOML)
- Loading from environment variables
- Loading from CLI arguments (argparse or typer)
- Implementing precedence: merge configs with priority
- ✨ **Teaching Tip**: "Use BaseSettings' env_prefix to namespace environment variables: APP_DATABASE_HOST"

**Section 4: Generic Type-Safe Access** (12 min)
- Creating `ConfigValue[T]` wrapper for typed access
- Example: `config.get[DatabaseConfig]("database")`
- Why generics ensure type safety
- IDE autocomplete based on type parameter
- 💬 **AI Colearning Prompt**: "Ask your AI: Why is `config.get[DatabaseConfig]('database')` better than `config['database']`? Show the type safety difference."

**Section 5: Error Handling and Validation** (10 min)
- Validation errors: missing required fields
- Helpful error messages for users
- Failing fast on startup (not during runtime)
- Logging config sources for debugging
- 🎓 **Instructor Commentary**: "Config errors should crash the app at startup, not 3 hours into production"

**Section 6: Testing** (12 min)
- Unit tests with mock YAML files
- Testing environment variable overrides
- Testing precedence rules
- Integration tests for full config loading
- ✨ **Teaching Tip**: "Use pytest fixtures to create temporary config files for tests"

**Section 7: Production Enhancements** (Optional - 10 min)
- Secrets from secret managers (AWS Secrets Manager, HashiCorp Vault)
- Hot reloading configuration (watchdog library)
- Config validation on change
- Remote config sources (S3, etcd, Consul)

**Project Deliverables** (10 min)
- Working ConfigManager module
- Example configs (dev.yaml, prod.yaml)
- Unit and integration tests
- README with usage examples
- 🚀 **CoLearning Challenge**: "Tell your AI: Generate a complete project structure with all files, tests, and documentation"

**Common Mistakes** (3 min)
- Not validating config on startup (fails in production hours later)
- Hardcoding defaults in code instead of config files
- Not documenting precedence rules (users confused about why values aren't applied)
- Overcomplicating with too many config sources

**Try With AI** (Lesson Closure - 4 Prompts)

1. **Prompt 1** (Understand): "Ask your AI: Explain the architecture of a production config system. Why do we need multiple sources (file, env, CLI) and precedence rules?"
   - **Expected Outcome**: Explanation covering flexibility (different envs), security (secrets in env), debugging (CLI overrides), with concrete use cases.

2. **Prompt 2** (Apply): "Tell your AI: Implement ConfigManager with DatabaseConfig, APIConfig, and AppConfig models. Show loading from a YAML file with environment variable overrides. Include error handling."
   - **Expected Outcome**: Working ConfigManager with multi-source loading, validation, precedence implementation.

3. **Prompt 3** (Analyze + Evaluate): "Ask your AI: Compare three approaches to config management: 1) Pydantic BaseSettings, 2) TypedDict, 3) dataclasses. What are the tradeoffs? When would you use each?"
   - **Expected Outcome**: Comparison table covering validation (Pydantic wins), simplicity (TypedDict), immutability (dataclasses), with justified recommendations per use case.

4. **Prompt 4** (Create): "Tell your AI: Build a complete, production-ready ConfigManager project. Include: nested config models, YAML + env + CLI loading, precedence handling, validation, tests, and a demo app that uses the config. Make it portfolio-worthy."
   - **Expected Outcome**: Complete project with all components, professional code quality, tests passing, README with examples, ready for GitHub portfolio.

#### Code Examples Assigned

- **EX-009: Config Manager Capstone** (full production system)

#### CoLearning Elements Positioned

- 💬 AI Colearning Prompts: 2 (architectural comparison and type safety benefits)
- 🎓 Instructor Commentary: 2 (config philosophy and production wisdom)
- 🚀 CoLearning Challenges: 2 (scaffolding and full project generation)
- ✨ Teaching Tips: 2 (environment variable namespacing and testing setup)

---

## Code Example Distribution Summary

| Example ID | Lesson | Complexity | Pedagogical Purpose |
|------------|--------|------------|---------------------|
| EX-001 | Lesson 1 | B1 | Basic Pydantic model with automatic validation |
| EX-002 | Lesson 1 | B1-B2 | Nested models for composition |
| EX-003 | Lesson 2 | B2 | Custom field validators |
| EX-004 | Lesson 2 | B2 | BaseSettings for environment config |
| EX-005 | Lesson 3 | B1 | Generic function with type preservation |
| EX-006 | Lesson 4 | B2 | Generic class (Stack[T]) |
| EX-007 | Lesson 4 | B2 | Bounded generics with Protocol |
| EX-008 | Lesson 5 | B2 | LLM output validation and iteration |
| EX-009 | Lesson 6 | B2 | Production Config Manager (capstone) |

**Coverage**: 50% Pydantic (EX-001 to EX-004, EX-008), 30% Generics (EX-005 to EX-007), 20% Integration (EX-009)

---

## CoLearning Element Distribution

### Total CoLearning Elements: 38 across 6 lessons

| Element Type | Lesson 1 | Lesson 2 | Lesson 3 | Lesson 4 | Lesson 5 | Lesson 6 | Total |
|--------------|----------|----------|----------|----------|----------|----------|-------|
| 💬 AI Colearning Prompts | 2 | 2 | 2 | 2 | 2 | 2 | **12** |
| 🎓 Instructor Commentary | 2 | 2 | 2 | 2 | 2 | 2 | **12** |
| 🚀 CoLearning Challenges | 1 | 1 | 1 | 1 | 1 | 2 | **7** |
| ✨ Teaching Tips | 1 | 1 | 1 | 1 | 2 | 2 | **8** |
| **Lesson Total** | **6** | **6** | **6** | **6** | **7** | **8** | **39** |

**Distribution Strategy**: Elements positioned THROUGHOUT lessons (not just at end), with higher density in complex sections. Capstone (Lesson 6) has more elements due to extended format.

---

## Assessment Strategy

### Formative Assessment (During Learning)

**"Try With AI" Prompts** (24 total, 4 per lesson):
- Bloom's Level 1-2 (Understand/Apply): Prompts 1-2 each lesson
- Bloom's Level 3-4 (Analyze/Evaluate): Prompt 3 each lesson
- Bloom's Level 5-6 (Evaluate/Create): Prompt 4 each lesson

**CoLearning Challenges** (7 total):
- Hands-on practice with AI assistance
- Immediate feedback through code execution
- Scaffolded complexity (B1 challenges in early lessons → B2 in later lessons)

### Summative Assessment (Chapter Completion)

**Capstone Project (Lesson 6)**:
- Type-Safe Config Manager with Pydantic + Generics
- Multi-source loading with validation
- Production code quality (tests, docs, error handling)
- Portfolio-worthy deliverable

**Success Criteria** (From Spec):
- EVAL-006: 80%+ build working Config Manager
- EVAL-010: Students submit capstone for peer review
- EVAL-008: 75%+ complete all 6 lessons

### Competency Mapping (CEFR Proficiency)

| CEFR Level | Lessons | Observable Behaviors |
|------------|---------|---------------------|
| **B1** | 1, 3 | Student can apply Pydantic models and generic functions with guidance from examples |
| **B1-B2** | 2, 4 | Student can design custom validators and generic classes independently for familiar problems |
| **B2** | 5, 6 | Student can analyze validation strategies, evaluate architectural tradeoffs, synthesize production systems |

---

## Next Steps

### Immediate (Phase 2 - Planning Complete)

✅ **plan.md created** with:
- Detailed lesson breakdowns (6 lessons)
- CEFR proficiency mapping (B1 → B2 progression)
- Skills metadata for each lesson
- CoLearning element positioning
- Code example assignments
- Bloom's taxonomy alignment
- Cognitive load validation (max 10 concepts per lesson)

### Phase 3: Tasks Generation

**Next Command**: `/sp.tasks` to create `tasks.md`

**Expected Output**:
- Actionable implementation checklist
- Acceptance criteria per task
- Dependency ordering
- Validation gates

### Phase 4: Implementation

**After Tasks Approved**: Invoke `/sp.implement` with content-implementer subagent

**Implementation Order**:
1. Lesson 1: Introduction to Pydantic and Data Validation
2. Lesson 2: Advanced Pydantic Patterns
3. Lesson 3: Introduction to Generics and Type Variables
4. Lesson 4: Generic Classes and Protocols
5. Lesson 5: Pydantic for AI-Native Development
6. Lesson 6: Capstone - Type-Safe Config Manager

**Review Gates**: Human review after each lesson before proceeding to next

### Phase 5: Validation

**After All Lessons Complete**: Invoke validation-auditor for validation

**Validation Checks**:
- Technical correctness (Python 3.14+, Pydantic V2, PEP 695 syntax)
- Constitutional compliance (no SDD references, lesson closure pattern, CoLearning elements)
- Pedagogical quality (CEFR alignment, Bloom's progression, cognitive load)
- Code quality (all examples tested, type hints 100%)

---

## Validation Checklist (Pre-Implementation)

### Planning Quality Gates

- [x] **6 lessons defined** with clear learning objectives
- [x] **CEFR proficiency levels assigned** (B1 → B1-B2 → B2 progression)
- [x] **Skills metadata included** for each lesson (name, level, category, measurability)
- [x] **Cognitive load validated** (max 10 concepts per lesson)
- [x] **CoLearning elements positioned** throughout lessons (not just at end)
- [x] **Code examples distributed** (9 total, balanced across Pydantic/Generics/Integration)
- [x] **"Try With AI" prompts structured** (4 per lesson, Bloom's progression)
- [x] **Capstone project defined** (Type-Safe Config Manager, production-quality)
- [x] **No forward references** to Chapters 30+ (SDD in Part 5)
- [x] **Lesson closure pattern specified** (ONLY "Try With AI", NO summaries after)

### Ready for Next Phase

✅ **Plan approved** → Proceed to `/sp.tasks` for implementation checklist generation

---
