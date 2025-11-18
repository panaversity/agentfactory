# documentation-exploration

**Description**: Systematic framework for extracting architectural patterns and design decisions from technical documentation, applicable to any framework, library, or system.

**When to use this skill**: Apply when evaluating new frameworks, libraries, databases, or systems to build deep conceptual understanding (not just feature awareness).

---

## Persona

"Think like a technical researcher extracting architectural patterns and design decisions from documentation, not just feature lists"

**Cognitive stance**: Documentation exploration is NOT reading linearly or memorizing features. It's extracting: (1) What problem this solves, (2) How it's designed differently, (3) What mental models matter, (4) What trade-offs exist. Prioritize conceptual understanding over implementation details.

---

## Questions

Apply these 5 questions sequentially when exploring any technical documentation:

### 1. What problem does this framework/library solve?

**Purpose**: Understand the fundamental purpose, not the feature list.

**Ask**:
- What pain point or challenge motivated this project's creation?
- What existing solutions were inadequate? Why?
- What specific use cases does this serve best?
- What problem does this NOT solve? (Clarify boundaries)

**Bad answer**: "FastAPI is a web framework"
**Good answer**: "FastAPI solves building high-performance async web APIs with automatic request/response validation using Python type hints, addressing the gap between Flask's simplicity and the need for production-grade type safety"

**Focus**: Problem → Solution, not Features → Capabilities

---

### 2. What design decisions differentiate it from alternatives?

**Purpose**: Identify architectural choices that make this unique.

**Ask**:
- What design choices distinguish this from similar tools?
- What trade-offs did designers make? (What did they optimize for?)
- What architectural patterns are central? (Event-driven, layered, microkernel)
- How does philosophy differ from alternatives? (Opinionated vs flexible, batteries-included vs minimal)

**Example comparisons**:
- **FastAPI vs Flask**: Type hints + async vs simplicity + sync
- **Django vs Flask**: Batteries-included + ORM vs minimal + flexibility
- **React vs Vue**: JSX + component composition vs template-driven + directives

**Focus**: Design rationale (WHY these choices), not just what differs

---

### 3. What are the core abstractions?

**Purpose**: Build mental models of key concepts.

**Ask**:
- What are the 3-5 fundamental concepts/abstractions?
- How do these abstractions relate to each other?
- What's the conceptual hierarchy? (Low-level primitives → high-level composition)
- Can I draw a diagram showing relationships?

**Example (FastAPI)**:
- **Path Operations**: Route definitions (@app.get, @app.post)
- **Pydantic Models**: Request/response data structures with validation
- **Dependency Injection**: Shared logic (auth, DB connections) via Depends()
- **Async/Await**: Non-blocking I/O for high concurrency

**Mental model**: Path operations receive Pydantic models, use dependencies, run async

---

### 4. What constraints or limitations exist?

**Purpose**: Understand trade-offs and when NOT to use this.

**Ask**:
- What are the known limitations? (Performance, scalability, compatibility)
- What scenarios is this NOT suited for?
- What's the learning curve? (Simple for beginners? Requires advanced knowledge?)
- What dependencies or runtime requirements exist? (Python version, OS, libraries)
- What's the cost of adoption? (Migration effort, team training, ecosystem maturity)

**Example (Django)**:
- **Constraint**: Opinionated structure (harder to customize than Flask)
- **Limitation**: Monolithic (not ideal for microservices)
- **Requirement**: Python 3.8+, PostgreSQL/MySQL for full ORM features
- **Learning curve**: Moderate (need to learn MTV pattern, ORM, admin)

**Focus**: Trade-offs, not just benefits (every design choice has costs)

---

### 5. How would I evaluate if this fits my use case?

**Purpose**: Create decision framework for adoption.

**Ask**:
- What criteria matter for my project? (Performance, simplicity, ecosystem, team expertise)
- How does this framework score on those criteria?
- What are the dealbreakers? (Missing features, incompatible constraints)
- What's the "sweet spot"? (Scenarios where this excels)
- What would make me choose an alternative instead?

**Decision framework example**:
```
Choose FastAPI if:
- ✅ Need high-performance async APIs
- ✅ Team values type safety (mypy, IDE autocomplete)
- ✅ Modern Python (3.7+) is acceptable

Choose Django if:
- ✅ Need full-stack web app with admin interface
- ✅ Prefer batteries-included (ORM, auth, migrations)
- ✅ Rapid prototyping more important than performance

Choose Flask if:
- ✅ Want maximum control over architecture
- ✅ Building lightweight APIs or microservices
- ✅ Team prefers minimal framework, custom extensions
```

---

## Principles

These are decision frameworks, not rigid rules. Apply judgment to context.

### Principle 1: Conceptual Before Implementation

**Framework**: "Understand WHY the system is designed this way before learning HOW to use it. Mental models before syntax."

**What this means**:
- Read philosophy/overview sections first (not tutorials)
- Ask "What problem does this solve?" before "How do I use this feature?"
- Build conceptual understanding (request lifecycle, data flow) before writing code

**Example**:
- ❌ Bad: Jump straight to tutorial ("Create your first app"), copy-paste code
- ✅ Good: Read "Philosophy" and "Architecture Overview" first, understand request lifecycle, THEN tutorial

---

### Principle 2: Decision Frameworks Over Absolutes

**Framework**: "Seek 'when to use X vs Y' frameworks, not 'X is better than Y' absolutes. Context determines quality."

**What this means**:
- Look for comparison sections (FastAPI vs Django vs Flask)
- Extract trade-offs (what do you optimize for, what do you sacrifice?)
- Build decision criteria (if performance matters → X, if admin matters → Y)

**Avoid**: Claims like "FastAPI is the best framework" (best for WHAT scenario?)

---

### Principle 3: Falsifiable Understanding

**Framework**: "Can you explain trade-offs? If you only know benefits, understanding is incomplete."

**What this means**:
- For every strength, identify the cost (async = performance, but complexity)
- Challenge claims ("If FastAPI is so fast, when would I choose Django instead?")
- Test understanding: Can you recommend when NOT to use this?

**Validation**: If you can't articulate limitations, explore constraints section.

---

### Principle 4: Progressive Layers (4-Layer Context Model)

**Framework**: "Explore documentation in layers: Overview → Core Concepts → Comparisons → Use Cases. Don't read linearly."

**What this means**:

**Layer 1 (Overview)**: What is this? Why does it exist? (5-10 minutes)
**Layer 2 (Core Concepts)**: What are key abstractions? (15-20 minutes)
**Layer 3 (Comparisons)**: How does this differ from alternatives? (10-15 minutes)
**Layer 4 (Use Cases)**: When should I use this? (10 minutes)

**Total**: 40-55 minutes for deep conceptual understanding (not hours reading tutorials)

---

### Principle 5: Context Chunking for Large Docs

**Framework**: "For documentation >50 pages, chunk into manageable sections. Synthesize after each chunk, not after reading everything."

**What this means**:
- Read one section (e.g., Models in Django)
- Synthesize: What's the key insight from this section?
- Connect: How does this relate to previous sections?
- Then move to next section

**Avoid**: Reading 100 pages linearly without pausing to synthesize (information overload, no retention)

---

## Reusability

**This skill transfers across**:
- **Python frameworks** (FastAPI, Django, Flask, Pyramid)
- **JavaScript frameworks** (React, Vue, Angular, Svelte)
- **Databases** (PostgreSQL, MongoDB, Redis, Cassandra)
- **Infrastructure** (Docker, Kubernetes, Terraform, Ansible)
- **Libraries** (Pandas, NumPy, TensorFlow, PyTorch)
- **Any technical system with documentation**

**Evidence of reusability** (tested in Chapter 10, Lesson 7):
- ✅ Explored FastAPI (async web framework)
- ✅ Explored Django (full-stack framework)
- ✅ Explored Flask (minimalist framework)
- Same 5 questions, same process, different insights per framework

**Future application** (Part 4+):
- Evaluate Python libraries for data processing (Pandas vs Polars)
- Compare testing frameworks (pytest vs unittest)
- Assess ORMs (SQLAlchemy vs Django ORM vs Tortoise)
- Protocol is domain-agnostic (reasoning pattern, not tool-specific)

---

## Usage Example

**Scenario**: Evaluating PostgreSQL vs MongoDB for new project

**Apply documentation-exploration**:

**Question 1: What problem does each solve?**
- PostgreSQL: Relational data with ACID guarantees, complex queries
- MongoDB: Document storage with flexible schema, horizontal scaling

**Question 2: What design decisions differentiate them?**
- PostgreSQL: Schema-first, normalized tables, SQL queries
- MongoDB: Schema-less, denormalized documents, JSON-like queries

**Question 3: What are core abstractions?**
- PostgreSQL: Tables, Rows, Columns, Foreign Keys, Transactions
- MongoDB: Collections, Documents, Fields, Embedded Documents, Aggregation Pipeline

**Question 4: What constraints exist?**
- PostgreSQL: Harder to scale horizontally, schema migrations required
- MongoDB: No ACID across documents (pre-4.0), less mature for complex joins

**Question 5: How to evaluate fit?**
```
Choose PostgreSQL if:
- ✅ Relational data with foreign keys
- ✅ Need complex joins and transactions
- ✅ Schema is well-defined upfront

Choose MongoDB if:
- ✅ Document-oriented data (JSON-like)
- ✅ Need flexible schema (evolving requirements)
- ✅ Horizontal scaling priority
```

**Decision**: PostgreSQL for transactional systems (e-commerce), MongoDB for content management (flexible schema)

---

---

## Transfer Validation

**This skill claims to transfer to Python development (Part 4, Chapters 12-29).**

**Validation checkpoint**: When Part 4 is implemented, validate that the Persona + Questions + Principles pattern works for Python library documentation review without modification:

**Test cases**:
1. **Python library comparison**: Does Question 2 (What design decisions differentiate it?) compare Pandas vs. Polars vs. Dask effectively?
2. **API documentation**: Does Question 3 (What are core abstractions?) extract key concepts from FastAPI, Flask, Django docs?
3. **Type system libraries**: Does Question 4 (What constraints exist?) identify limitations in Pydantic, TypedDict, dataclasses?
4. **Standard library**: Does the framework work for exploring asyncio, pathlib, logging documentation?
5. **Decision frameworks**: Does Question 5 (How to evaluate fit?) produce actionable "Choose X if..." criteria for Python libraries?

**Expected outcome**: Framework works without Python-specific modifications. Questions are domain-agnostic and apply to Python library docs as well as markdown/git/bash documentation.

**Validation date**: [To be completed when Part 4 Python chapters are implemented]

---

## Version

**Version**: 1.0.0
**Created**: 2025-01-18 (Chapter 10, Lesson 7)
**Domain**: Cross-domain documentation exploration
**Pattern**: Persona + Questions + Principles (reasoning-activated)
**Constitution**: v6.0.0 Compliance
