# Chapter 25 Specification: Object-Oriented Programming Part II

**Feature Branch**: `020-oop-part-1-2`
**Created**: 2025-11-09
**Status**: Draft
**Part**: 4 (Python Fundamentals)
**Chapter**: 25 of 57
**Prerequisites**: Chapters 1-24 (especially Ch24: OOP Part I)
**Complexity Tier**: Advanced (B1-B2, max 10 concepts/lesson)

## Success Evals *(defined BEFORE specification)*

These business-goal-aligned success criteria define what "good" looks like:

### Comprehension Evals
- **EVAL-001**: 80%+ can explain inheritance and when to prefer composition over inheritance
- **EVAL-002**: 75%+ can explain Method Resolution Order (MRO) and C3 linearization
- **EVAL-003**: 70%+ can explain polymorphism and duck typing with code examples
- **EVAL-004**: 75%+ can explain at least 3 special methods (__str__, __repr__, __add__) and their use cases

### Skill Acquisition Evals
- **EVAL-005**: 85%+ can create a class hierarchy using inheritance with proper super() usage
- **EVAL-006**: 80%+ can implement polymorphic behavior using abstract base classes
- **EVAL-007**: 75%+ can implement at least 2 design patterns (Singleton, Factory, Observer, or Strategy)
- **EVAL-008**: 70%+ can override special methods to customize object behavior

### Engagement Evals
- **EVAL-009**: 90%+ complete all 5 lessons
- **EVAL-010**: 85%+ complete the Design Patterns capstone (implementing all 4 patterns)
- **EVAL-011**: 75%+ engage with "Try With AI" prompts

### Accessibility Evals
- **EVAL-012**: Flesch-Kincaid Grade Level: 10-12 (B1-B2 CEFR)
- **EVAL-013**: All code examples run on Python 3.14+
- **EVAL-014**: Average lesson completion time: 50-70 minutes (slightly longer than Part I due to complexity)

## Topic Summary

**Object-Oriented Programming Part II** builds on Chapter 24 foundations to teach advanced OOP patterns: inheritance hierarchies, polymorphism, composition vs inheritance, special methods, and professional design patterns (Singleton, Factory, Observer, Strategy).

This chapter transforms students from basic OOP practitioners into professional object-oriented designers. Students learn to evaluate design tradeoffs, recognize when to use inheritance vs composition, and apply battle-tested design patterns from real-world software engineering.

**Why this matters**: Modern AI agent systems are built using these advanced patterns. Multi-agent systems use inheritance for agent types, polymorphism for flexible behavior, and design patterns like Factory (creating agents dynamically) and Observer (event-driven agent communication). This chapter prepares students for professional AI engineering.

## Learning Objectives *(aligned with evals)*

By the end of Chapter 25, students will be able to:

1. **LO-001**: Create inheritance hierarchies using super(), explain Method Resolution Order (MRO), and identify when inheritance is appropriate vs composition *(aligns with EVAL-001, EVAL-002, EVAL-005)*

2. **LO-002**: Implement polymorphic behavior using abstract base classes, duck typing, and method overriding *(aligns with EVAL-003, EVAL-006)*

3. **LO-003**: Design object systems using composition over inheritance when appropriate, and explain the tradeoffs *(aligns with EVAL-001)*

4. **LO-004**: Override special methods (__str__, __repr__, __add__, __len__, __getitem__, etc.) to customize object behavior *(aligns with EVAL-004, EVAL-008)*

5. **LO-005**: Implement 4 design patterns (Singleton, Factory, Observer, Strategy) and explain when each is appropriate *(aligns with EVAL-007, EVAL-010)*

6. **LO-006**: Organize OOP code into modules and packages for scalability *(extends Ch24 knowledge)*

## User Scenarios & Testing *(for educational content)*

### Learning Journey 1 - Mastering Inheritance and MRO (Priority: P1)

**Scenario**: Student has built basic classes (Ch24) and now needs to create class hierarchies to model "is-a" relationships and reuse code through inheritance.

**Why this priority**: Inheritance is foundational for polymorphism and design patterns. Without understanding MRO, students cannot debug diamond inheritance or super() calls.

**Independent Test**: Student can create a 3-level inheritance hierarchy with proper super() usage.

**Acceptance Scenarios**:

1. **Given** student understands classes from Ch24, **When** they learn single inheritance (Lesson 1), **Then** they can create parent-child class relationships
2. **Given** student can create single inheritance, **When** they learn super() function, **Then** they can properly call parent methods
3. **Given** student uses super(), **When** they learn MRO, **Then** they can explain which parent method gets called in diamond inheritance
4. **Given** student understands MRO, **When** they use mro() method, **Then** they can inspect and debug inheritance chains

---

### Learning Journey 2 - Implementing Polymorphism (Priority: P2)

**Scenario**: Student needs to write code that works with different object types through a common interface (polymorphism) and understands Python's duck typing philosophy.

**Why this priority**: Polymorphism is the power of OOP. Without it, students miss the main benefit of inheritance and abstract base classes.

**Independent Test**: Student can create polymorphic code that works with multiple class types.

**Acceptance Scenarios**:

1. **Given** student knows inheritance, **When** they learn method overriding (Lesson 2), **Then** they can create child classes with specialized behavior
2. **Given** student can override methods, **When** they learn abstract base classes deeply, **Then** they can enforce implementation contracts
3. **Given** student uses ABC, **When** they learn duck typing, **Then** they can write polymorphic code without inheritance
4. **Given** student understands polymorphism, **When** they see real AI agent examples, **Then** they recognize polymorphic patterns in production code

---

### Learning Journey 3 - Choosing Composition Over Inheritance (Priority: P3)

**Scenario**: Student needs to decide between inheritance ("is-a") and composition ("has-a") when designing object systems, and organize code into modules.

**Why this priority**: Professional developers prefer composition for flexibility. This journey teaches design judgment.

**Independent Test**: Student can refactor an inheritance-based design to use composition and explain why it's better.

**Acceptance Scenarios**:

1. **Given** student knows inheritance, **When** they learn composition (Lesson 3), **Then** they can model "has-a" relationships
2. **Given** student can use both patterns, **When** they learn "composition over inheritance" principle, **Then** they can explain tradeoffs
3. **Given** student understands tradeoffs, **When** they see a design problem, **Then** they can choose the appropriate pattern
4. **Given** student builds multi-class systems, **When** they learn modules/packages, **Then** they can organize code into scalable structure

---

### Learning Journey 4 - Customizing Objects with Special Methods (Priority: P4)

**Scenario**: Student needs to make custom objects behave like built-in types (support +, len(), str(), iteration, etc.) using special methods.

**Why this priority**: Special methods make Python objects Pythonic. This is advanced but essential for professional code.

**Independent Test**: Student can implement 5+ special methods to customize object behavior.

**Acceptance Scenarios**:

1. **Given** student has created classes, **When** they learn __str__ and __repr__ (Lesson 4), **Then** they can control object display
2. **Given** student controls display, **When** they learn operator overloading (__add__, __sub__, etc.), **Then** custom objects work with operators
3. **Given** student can overload operators, **When** they learn container protocols (__len__, __getitem__, __iter__), **Then** objects behave like lists/dicts
4. **Given** student masters special methods, **When** they see Python source code, **Then** they recognize how built-in types work

---

### Learning Journey 5 - Applying Design Patterns (Priority: P5 - Capstone)

**Scenario**: Student applies all OOP Part II knowledge to implement 4 professional design patterns used in real AI systems.

**Why this priority**: Synthesis of all learning. Design patterns are the vocabulary of professional software architecture.

**Independent Test**: Student completes capstone implementing Singleton, Factory, Observer, and Strategy patterns.

**Acceptance Scenarios**:

1. **Given** student completed Lessons 1-4, **When** they start capstone (Lesson 5), **Then** they can identify which pattern solves which problem
2. **Given** student understands pattern selection, **When** they implement Singleton pattern, **Then** they ensure only one instance exists
3. **Given** student implements Singleton, **When** they implement Factory pattern, **Then** they create objects without specifying exact class
4. **Given** student implements Factory, **When** they implement Observer pattern, **Then** they create event-driven systems
5. **Given** student implements Observer, **When** they implement Strategy pattern, **Then** they can switch algorithms at runtime
6. **Given** student implements all 4 patterns, **When** they integrate them into one system, **Then** they demonstrate professional OOP architecture

---

### Edge Cases

- What happens when you have diamond inheritance (A <- B, A <- C, D <- B & C)? *(Teach MRO and C3 linearization)*
- How do you prevent inheritance when you want composition? *(Teach when NOT to use inheritance)*
- What if a subclass doesn't override an abstract method? *(Show TypeError at instantiation)*
- How do special methods like __add__ handle different types? *(Teach type checking and NotImplemented)*
- What if multiple patterns solve the same problem? *(Teach pattern selection criteria)*

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Chapter MUST teach single inheritance, multiple inheritance, and diamond inheritance with MRO explanations

- **FR-002**: Chapter MUST demonstrate super() function usage in constructors and method calls

- **FR-003**: Chapter MUST teach method overriding and polymorphism with abstract base classes (ABC deep dive)

- **FR-004**: Chapter MUST explain duck typing and when it's preferred over inheritance

- **FR-005**: Chapter MUST teach composition and explain "composition over inheritance" principle with concrete examples

- **FR-006**: Chapter MUST demonstrate organizing classes into modules and packages

- **FR-007**: Chapter MUST teach at least 8 special methods: __str__, __repr__, __add__, __len__, __getitem__, __iter__, __eq__, __call__

- **FR-008**: Chapter MUST implement all 4 design patterns: Singleton, Factory, Observer, Strategy

- **FR-009**: All code examples MUST use Python 3.14+ with modern type hints

- **FR-010**: Chapter MUST apply AI-Native CoLearning pedagogy with 💬🎓🚀✨ elements throughout

- **FR-011**: All lessons MUST end with "Try With AI" section ONLY (4 prompts, progressive Bloom's levels)

- **FR-012**: Chapter MUST avoid forward references (Lesson N uses only concepts from Ch24 + Lessons 1 to N)

### Key Entities *(educational concepts)*

- **Inheritance**: "is-a" relationship; subclass inherits from superclass
- **super()**: Function to call parent class methods
- **Method Resolution Order (MRO)**: Order Python searches for methods in inheritance hierarchy
- **C3 Linearization**: Algorithm Python uses to compute MRO
- **Method Overriding**: Subclass provides specific implementation of parent method
- **Polymorphism**: Objects of different types respond to same method call
- **Duck Typing**: "If it walks like a duck and quacks like a duck, it's a duck" - behavior over type
- **Composition**: "has-a" relationship; object contains another object
- **Special Methods (Magic/Dunder Methods)**: __method__ that Python calls implicitly
- **Design Pattern**: Reusable solution to common problem in software design
- **Singleton Pattern**: Ensure only one instance of a class exists
- **Factory Pattern**: Create objects without specifying exact class
- **Observer Pattern**: One-to-many dependency for event notification
- **Strategy Pattern**: Encapsulate algorithms and make them interchangeable

## Content Outline

### Section 1: Inheritance and Method Resolution Order

**Concepts to Cover** (8 concepts):
1. Single inheritance (parent-child relationship)
2. super() function (calling parent methods)
3. Multiple inheritance (class inherits from 2+ parents)
4. Diamond inheritance problem
5. Method Resolution Order (MRO)
6. C3 linearization algorithm
7. mro() method (inspecting inheritance chain)
8. When to use inheritance vs when to avoid

**Lesson 1**: Inheritance and MRO
- 💬 AI Colearning Prompt: "Ask your AI: Why does Python use C3 linearization instead of simpler depth-first or breadth-first search?"
- 🎓 Instructor Commentary: "In AI-native development, inheritance hierarchies model agent types. Understanding MRO prevents subtle bugs in multi-agent systems."
- 🚀 CoLearning Challenge: "Create a diamond inheritance hierarchy (Vehicle <- Car, Vehicle <- Boat, AmphibiousCar <- Car & Boat). Ask AI to explain MRO."
- ✨ Teaching Tip: "Use Claude Code to visualize MRO: 'Draw the inheritance tree for this class and show method lookup order'"

---

### Section 2: Polymorphism and Duck Typing

**Concepts to Cover** (7 concepts):
1. Method overriding (subclass replaces parent method)
2. Polymorphic behavior (same interface, different implementation)
3. Abstract Base Classes (ABC) - deep dive
4. @abstractmethod decorator
5. Abstract properties and class methods
6. Duck typing philosophy
7. Structural subtyping vs nominal subtyping

**Lesson 2**: Polymorphism and Duck Typing
- 💬 AI Colearning Prompt: "Explain why Python prefers duck typing over static type checking. What are the tradeoffs?"
- 🎓 Instructor Commentary: "In AI agent systems, polymorphism lets different agent types (ChatAgent, CodeAgent, DataAgent) share a common interface (process_message). Duck typing makes this flexible."
- 🚀 CoLearning Challenge: "Create an abstract Agent base class with @abstractmethod process(). Then create 3 agent types that implement it differently."

---

### Section 3: Composition Over Inheritance + Modules

**Concepts to Cover** (6 concepts):
1. Composition ("has-a" relationship)
2. Aggregation vs composition
3. When to use composition vs inheritance
4. "Composition over inheritance" principle
5. Organizing classes into modules
6. Creating packages with __init__.py

**Lesson 3**: Composition vs Inheritance and Module Organization
- 💬 AI Colearning Prompt: "Ask your AI: Why do professional developers prefer 'composition over inheritance'? Give real-world examples."
- 🚀 CoLearning Challenge: "Refactor a Car class that inherits from Engine to use composition (Car has-an Engine). Explain why this is better."
- ✨ Teaching Tip: "Use AI to explore Gang of Four book: 'Explain the composition pattern from Design Patterns and show Python example'"

---

### Section 4: Special Methods (Magic/Dunder Methods)

**Concepts to Cover** (10 concepts):
1. __str__ and __repr__ (string representation)
2. __add__, __sub__, __mul__ (operator overloading)
3. __len__ (length protocol)
4. __getitem__, __setitem__, __delitem__ (indexing protocol)
5. __iter__, __next__ (iteration protocol)
6. __eq__, __lt__, __hash__ (comparison and hashing)
7. __call__ (making objects callable)
8. __enter__, __exit__ (context manager protocol - brief intro)
9. __new__ (object creation - brief intro)
10. When to override which special method

**Lesson 4**: Special Methods and Python Protocols
- 💬 AI Colearning Prompt: "Explain how Python's built-in list implements __getitem__, __len__, and __iter__. What makes an object 'list-like'?"
- 🎓 Instructor Commentary: "Special methods are Python's secret sauce. They make custom objects feel native. In AI systems, overriding __call__ makes agent objects callable like functions."
- 🚀 CoLearning Challenge: "Create a Vector class that supports +, -, *, len(), and iteration using special methods."

---

### Section 5: Design Patterns Capstone

**Lesson 5**: Implementing Design Patterns

**Patterns to Implement**:

1. **Singleton Pattern** (Configuration, Logger, Database Connection)
   - Ensure only one instance exists
   - Thread-safe implementation
   - Use case: Global configuration manager

2. **Factory Pattern** (Agent creation, Object creation)
   - Create objects without specifying exact class
   - Product hierarchy (AbstractProduct, ConcreteProducts)
   - Use case: Creating different agent types dynamically

3. **Observer Pattern** (Event systems, Pub-Sub)
   - One-to-many dependency
   - Subject notifies observers when state changes
   - Use case: Event-driven agent communication

4. **Strategy Pattern** (Algorithm selection)
   - Encapsulate algorithms in separate classes
   - Switch strategies at runtime
   - Use case: Different AI model selection strategies

**Integration Example**: Build an AI Agent System combining all 4 patterns:
- Singleton: AgentManager (single instance)
- Factory: AgentFactory creates different agent types
- Observer: Agents observe events and react
- Strategy: Agents use different reasoning strategies

**🚀 CoLearning Challenge**:
"Design a multi-agent system where agents communicate via Observer pattern, are created by Factory, managed by Singleton, and use Strategy for decision-making. Ask AI to help architect this."

---

### Common Mistakes

1. **Forgetting super() in __init__**: Child class doesn't call parent constructor
2. **Diamond inheritance without understanding MRO**: Unexpected method calls
3. **Overusing inheritance**: Creating deep hierarchies when composition is better
4. **Not implementing all abstract methods**: TypeError at instantiation
5. **Implementing __eq__ without __hash__**: Objects can't be used in sets/dicts
6. **Circular imports in modules**: Poor package organization

---

### AI Exercise Integration

Throughout all lessons, students use Claude Code or Gemini CLI to:
- Explore advanced patterns through AI dialogue
- Generate design pattern implementations
- Refactor inheritance to composition
- Debug MRO issues
- Visualize class hierarchies

**Framing** (Part 4 appropriate):
- ✅ "Describe the agent system you want to build..."
- ✅ "Ask your AI which pattern fits this problem..."
- ✅ "Validate your MRO by testing with mro() method..."
- ❌ NO "Write formal specifications" (that's Part 5)

## Code Examples *(specifications for 12-15 examples)*

### Example 1: Single Inheritance with super()
**Purpose**: Demonstrate basic inheritance and parent method calling
**Complexity**: Intermediate (B1)
**Prompt**: "Create Animal parent class with speak(). Create Dog child class that overrides speak() but calls super().speak() first."
**Key Teaching Point**: super() usage

### Example 2: Diamond Inheritance and MRO
**Purpose**: Show diamond problem and C3 linearization
**Complexity**: Advanced (B2)
**Prompt**: "Create A <- B, A <- C, D <- B & C diamond. Call method in A from D and explain MRO."
**Key Teaching Point**: Understanding MRO

### Example 3: Abstract Base Class Deep Dive
**Purpose**: Enforce implementation contracts
**Complexity**: Advanced (B2)
**Prompt**: "Create abstract Vehicle with abstract start(), stop(), refuel(). Create Car and Bike subclasses."
**Key Teaching Point**: ABC for polymorphism

### Example 4: Duck Typing
**Purpose**: Show polymorphism without inheritance
**Complexity**: Intermediate (B1)
**Prompt**: "Create FileReader and URLReader that both have read() method. Write process() function that works with both without inheritance."
**Key Teaching Point**: Duck typing philosophy

### Example 5: Composition vs Inheritance
**Purpose**: Show "has-a" vs "is-a"
**Complexity**: Advanced (B2)
**Prompt**: "Refactor a Car(Engine) inheritance to Car with Engine composition. Show why composition is more flexible."
**Key Teaching Point**: Composition over inheritance

### Example 6: Module Organization
**Purpose**: Organize classes into packages
**Complexity**: Intermediate (B1)
**Prompt**: "Create animals package with mammals.py, birds.py modules. Show proper __init__.py imports."
**Key Teaching Point**: Scalable code organization

### Example 7-16: Special Methods
**Purpose**: Customize object behavior
**Complexity**: Advanced (B2)
**Prompts**:
- Example 7: __str__ and __repr__
- Example 8: __add__, __sub__ (operator overloading)
- Example 9: __len__, __getitem__ (container protocol)
- Example 10: __iter__, __next__ (iteration protocol)
- Example 11: __eq__, __hash__ (comparison protocol)
- Example 12: __call__ (callable objects)
**Key Teaching Point**: Making custom objects Pythonic

### Example 13-16: Design Patterns (Capstone)
**Prompts**:
- Example 13: Singleton pattern (AgentManager)
- Example 14: Factory pattern (AgentFactory)
- Example 15: Observer pattern (Event system)
- Example 16: Strategy pattern (Decision strategies)
**Key Teaching Point**: Professional architecture patterns

## Acceptance Criteria *(references evals)*

### Content Quality
- [ ] Inheritance, MRO, and polymorphism explained clearly *(references EVAL-001, EVAL-002, EVAL-003)*
- [ ] All 4 design patterns implemented with working code *(references EVAL-007, EVAL-010)*
- [ ] No forward references to Chapter 26+ topics (metaclasses, dataclasses)
- [ ] All code uses Python 3.14+ with type hints
- [ ] 💬🎓🚀✨ CoLearning elements present throughout *(references EVAL-011)*
- [ ] Conversational tone (not documentation)

### Lesson Structure
- [ ] 5 lessons total (4 foundational + 1 capstone)
- [ ] Each lesson ends with "Try With AI" ONLY (4 prompts) *(references EVAL-009)*
- [ ] No summaries/checklists after Try With AI
- [ ] Progression: Concept → Code → Try → Why it matters

### Pedagogical Compliance
- [ ] Lesson N uses only Ch24 + Lessons 1 to N (no forward references)
- [ ] All concepts introduced before use
- [ ] Max 10 concepts per lesson (B1-B2 cognitive load)
- [ ] MRO examples tested and output verified

### Capstone Quality
- [ ] Design Patterns capstone integrates all 4 patterns *(references EVAL-010)*
- [ ] Working code students can extend
- [ ] Demonstrates inheritance, polymorphism, composition, special methods
- [ ] Real AI agent system example

### Validation Against Evals
- [ ] Content enables 80%+ explaining inheritance/composition *(EVAL-001)*
- [ ] Exercises enable 85%+ creating inheritance hierarchies *(EVAL-005)*
- [ ] Capstone enables 75%+ implementing design patterns *(EVAL-007)*
- [ ] All code tested and working *(EVAL-013)*
- [ ] Flesch-Kincaid Grade Level 10-12 *(EVAL-012)*

### AI-Native Learning Pattern
- [ ] "Describe intent" framing (NOT formal SDD)
- [ ] AI partnership throughout
- [ ] Validation-first (test understanding)
- [ ] Error literacy (learn from mistakes)

## Assumptions

1. **Prerequisites**: Students completed Chapter 24 (OOP Part I) and understand classes, objects, encapsulation, methods

2. **AI Tool Access**: Claude Code or Gemini CLI available for CoLearning

3. **Time Allocation**: 50-70 minutes per lesson; total chapter: 5-6 hours

4. **Python Version**: Python 3.14+ with modern type hints

5. **Reading Level**: B1-B2 CEFR (Flesch-Kincaid 10-12)

6. **Design Patterns**: Focus on 4 patterns most relevant to AI systems (Singleton, Factory, Observer, Strategy)

7. **Code Testing**: All examples tested before publication

8. **MRO Examples**: Verified with mro() method output

## Out of Scope (For Later Chapters)

- Metaclasses (Chapter 26)
- Dataclasses (Chapter 26)
- Pydantic models (Chapter 27)
- Generics and type parameters (Chapter 27)
- Async/await patterns (Chapter 28)
- Advanced decorators beyond @property (mentioned briefly)
- Context managers (brief __enter__/__exit__ intro only)

## Dependencies

- **Chapter 24 (OOP Part I)**: Absolute prerequisite - students must understand classes, objects, encapsulation
- **Chapter 20 (Functions)**: For understanding method vs function
- **Chapter 21 (Exception Handling)**: For robust pattern implementations

## Success Criteria *(technology-agnostic, measurable)*

### Learning Outcomes
- **SC-001**: 80% can create a 3-level inheritance hierarchy with proper super() *(aligns with EVAL-005)*
- **SC-002**: 75% can explain MRO for a diamond inheritance example *(aligns with EVAL-002)*
- **SC-003**: 70% can implement polymorphic behavior using ABC *(aligns with EVAL-006)*
- **SC-004**: 75% can implement 2+ design patterns from scratch *(aligns with EVAL-007)*

### Engagement Metrics
- **SC-005**: 90% complete all 5 lessons *(aligns with EVAL-009)*
- **SC-006**: 85% complete design patterns capstone *(aligns with EVAL-010)*
- **SC-007**: 75% engage with 15+ out of 20 "Try With AI" prompts *(aligns with EVAL-011)*

### Content Quality
- **SC-008**: Flesch-Kincaid readability 10-12 *(aligns with EVAL-012)*
- **SC-009**: Zero critical code errors on Python 3.14+ *(aligns with EVAL-013)*
- **SC-010**: Student difficulty rating 4.0-4.5 out of 5 (appropriately challenging for advanced)

### Skill Retention
- **SC-011**: 70% can apply design patterns in Chapter 26+ without review
- **SC-012**: 65% can refactor inheritance-based code to composition in under 60 minutes

## Notes

- This is **advanced OOP** building on Chapter 24 foundations
- Success measured by **student mastery of patterns**, not memorization
- Design patterns are **teaching tools** for architecture thinking
- AI CoLearning is **integral**, not optional
- Chapter 25 prepares students for **professional AI engineering** (Parts 5-13)
