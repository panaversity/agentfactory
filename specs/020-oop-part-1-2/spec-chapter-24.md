# Chapter 24 Specification: Object-Oriented Programming Part I

**Feature Branch**: `020-oop-part-1-2`
**Created**: 2025-11-09
**Status**: Draft
**Part**: 4 (Python Fundamentals)
**Chapter**: 24 of 57
**Prerequisites**: Chapters 1-23 (especially Ch20: Module and Functions, Ch21: Exception Handling)
**Complexity Tier**: Advanced (B1-B2, max 10 concepts/lesson)

## Success Evals *(defined BEFORE specification)*

These business-goal-aligned success criteria define what "good" looks like:

### Comprehension Evals
- **EVAL-001**: 80%+ of students can explain all 4 OOP pillars (Encapsulation, Abstraction, Inheritance, Polymorphism) in their own words
- **EVAL-002**: 75%+ can identify when to use class vs instance attributes in code review scenarios
- **EVAL-003**: 70%+ can explain the difference between public, protected, and private attributes

### Skill Acquisition Evals
- **EVAL-004**: 85%+ can create a class with proper constructor, attributes, and methods from a written description
- **EVAL-005**: 80%+ can implement encapsulation using public/protected/private attributes correctly
- **EVAL-006**: 75%+ can build a simple object system (3+ interacting classes) for a real-world scenario

### Engagement Evals
- **EVAL-007**: 90%+ complete all 5 lessons
- **EVAL-008**: 85%+ complete the Game Character System capstone project
- **EVAL-009**: 75%+ engage with "Try With AI" prompts (measured by AI interaction logs if available)

### Accessibility Evals
- **EVAL-010**: Flesch-Kincaid Grade Level: 10-12 (appropriate for B1-B2 CEFR)
- **EVAL-011**: All code examples run successfully on Python 3.14+
- **EVAL-012**: Average lesson completion time: 45-60 minutes

## Topic Summary

**Object-Oriented Programming Part I** introduces students to the foundational concepts of OOP in Python: classes, objects, encapsulation, and methods. This chapter teaches students to think in terms of real-world entities modeled as objects with attributes (data) and methods (behavior).

In AI-native development, OOP is essential because AI agents themselves are objects. Understanding how to design classes, encapsulate data, and define methods prepares students to build agentic systems where each agent is a well-designed object with clear responsibilities.

**Why this matters**: Professional AI applications are built using OOP principles. Whether creating chatbot agents, data processing pipelines, or multi-agent systems, students need to model complex systems as interacting objects. This chapter lays the foundation for that professional skill.

## Learning Objectives *(aligned with evals)*

By the end of Chapter 24, students will be able to:

1. **LO-001**: Explain the four pillars of OOP (Encapsulation, Abstraction, Inheritance, Polymorphism) and identify when each is appropriate *(aligns with EVAL-001)*

2. **LO-002**: Create Python classes with constructors (__init__), destructors (__del__), and properly scoped attributes (public, protected, private) *(aligns with EVAL-004, EVAL-005)*

3. **LO-003**: Implement three types of methods (instance methods, class methods with @classmethod, static methods with @staticmethod) and explain when to use each *(aligns with EVAL-002)*

4. **LO-004**: Apply encapsulation principles to protect data and control access using getters/setters and property decorators *(aligns with EVAL-005)*

5. **LO-005**: Build a multi-class object system that models a real-world scenario (Game Character System) using proper OOP design *(aligns with EVAL-006, EVAL-008)*

## User Scenarios & Testing *(for educational content)*

### Learning Journey 1 - Understanding OOP Fundamentals (Priority: P1)

**Scenario**: A beginner Python developer has mastered functions, loops, and data structures (Chapters 12-23) and now needs to organize complex programs using OOP to build maintainable AI applications.

**Why this priority**: OOP fundamentals are the foundation for everything else in Part I. Without understanding classes vs objects and the 4 pillars, students cannot progress.

**Independent Test**: Student can explain OOP pillars and create a simple class after Lesson 1-2.

**Acceptance Scenarios**:

1. **Given** student has completed Chapters 1-23, **When** they read Lesson 1 (What is OOP?), **Then** they can list the 4 OOP pillars and explain why OOP exists
2. **Given** student understands OOP concept, **When** they complete Lesson 2 (Classes and Objects), **Then** they can create a class with __init__ and create instances
3. **Given** student has created basic classes, **When** they ask AI "Why do we use self?", **Then** AI explains instance reference pattern

---

### Learning Journey 2 - Mastering Constructors and Attributes (Priority: P2)

**Scenario**: Student needs to initialize objects with specific data and manage attribute visibility (public/protected/private) to build robust classes.

**Why this priority**: Proper initialization and attribute management are essential for professional code. This builds on Journey 1 foundations.

**Independent Test**: Student can create a class with parameterized constructor and control attribute access.

**Acceptance Scenarios**:

1. **Given** student knows basic classes, **When** they learn constructors (Lesson 3), **Then** they can create classes with parameterized __init__ methods
2. **Given** student understands constructors, **When** they learn class vs instance attributes, **Then** they can identify which attributes to make class-level vs instance-level
3. **Given** student manages attributes, **When** they use protected (_) and private (__) naming, **Then** they can explain when to use each

---

### Learning Journey 3 - Implementing Encapsulation and Methods (Priority: P3)

**Scenario**: Student needs to organize behavior into methods, control access to data, and use different method types (@classmethod, @staticmethod) appropriately.

**Why this priority**: Builds on Journeys 1-2 to create fully functional, encapsulated classes.

**Independent Test**: Student can create a class with all three method types and proper encapsulation.

**Acceptance Scenarios**:

1. **Given** student has built classes with attributes, **When** they learn instance methods (Lesson 4), **Then** they can add behavior to objects
2. **Given** student uses instance methods, **When** they learn @classmethod and @staticmethod, **Then** they can choose the appropriate method type for different scenarios
3. **Given** student has all method types, **When** they apply encapsulation, **Then** they can protect data using getters/setters and property decorators

---

### Learning Journey 4 - Building Real-World Object Systems (Priority: P4)

**Scenario**: Student applies all OOP Part I concepts to build a Game Character System with multiple interacting classes (Player, Enemy, Inventory, Actions).

**Why this priority**: Synthesis of all prior learning into a complete, testable system.

**Independent Test**: Student completes capstone project successfully.

**Acceptance Scenarios**:

1. **Given** student has learned all concepts (Lessons 1-4), **When** they start the capstone (Lesson 5), **Then** they can design a multi-class system
2. **Given** student designs the system, **When** they implement Player and Enemy classes, **Then** classes interact correctly (combat, health management)
3. **Given** student implements core classes, **When** they add Inventory and Actions, **Then** the complete system models game mechanics using proper OOP

---

### Edge Cases

- What happens when a student tries to access a private attribute directly? *(Should raise AttributeError or demonstrate name mangling)*
- How does a student handle cases where class methods need to create instances? *(Teach factory pattern basics)*
- What if a student confuses class attributes with instance attributes and modifies the wrong one? *(Teach debugging with __dict__ inspection)*
- How does @property decorator differ from regular getter methods? *(Show both approaches, explain when each is preferred)*

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Chapter MUST introduce all 4 OOP pillars (Encapsulation, Abstraction, Inheritance, Polymorphism) with clear definitions *(Note: Part I focuses on Encapsulation and Abstraction intro; Inheritance and Polymorphism covered in Part II)*

- **FR-002**: Chapter MUST teach class creation syntax with __init__ constructor and self keyword

- **FR-003**: Chapter MUST demonstrate the difference between class attributes and instance attributes using code examples

- **FR-004**: Chapter MUST teach all three method types: instance methods, @classmethod, @staticmethod with use cases for each

- **FR-005**: Chapter MUST explain encapsulation using public, protected (_name), and private (__name) attribute naming conventions

- **FR-006**: Chapter MUST include Abstract Base Classes (ABC) as a brief introduction (concept only; deep dive in Chapter 25)

- **FR-007**: All code examples MUST use Python 3.14+ syntax with modern type hints (list[int], dict[str, float], X | None)

- **FR-008**: Chapter MUST include a Game Character System capstone that integrates all Part I concepts

- **FR-009**: Chapter MUST apply AI-Native CoLearning pedagogy throughout with 💬🎓🚀✨ elements in every lesson

- **FR-010**: All lessons MUST end with "Try With AI" section ONLY (4 prompts with progressive Bloom's levels)

- **FR-011**: Chapter MUST avoid forward references (Lesson N uses only concepts from Lessons 1 to N)

- **FR-012**: Chapter MUST distinguish built-in functions (len, type, isinstance) from methods (.upper, .split) in explanations

### Key Entities *(educational concepts, not data entities)*

- **Class**: Blueprint/template for creating objects; defines attributes and methods
- **Object**: Instance of a class with specific attribute values
- **Constructor (__init__)**: Special method that initializes new objects
- **Destructor (__del__)**: Special method called when object is destroyed
- **self**: Reference to the current instance within methods
- **Class Attribute**: Shared across all instances of a class
- **Instance Attribute**: Unique to each object
- **Instance Method**: Operates on instance data (takes self)
- **Class Method (@classmethod)**: Operates on class data (takes cls)
- **Static Method (@staticmethod)**: Utility function within class scope
- **Encapsulation**: Bundling data and methods; restricting access with public/protected/private
- **Property Decorator (@property)**: Pythonic way to create getters/setters
- **Abstract Base Class (ABC)**: Conceptual introduction (enforces method implementation in subclasses)

## Content Outline

### Section 1: OOP Fundamentals - Why Objects Matter

**Concepts to Cover** (5 concepts):
1. What is OOP? (paradigm shift from procedural to object-oriented)
2. Why use OOP? (Modularity, Reusability, Maintainability, Scalability)
3. The 4 Pillars Overview (Encapsulation, Abstraction, Inheritance, Polymorphism)
4. Real-world modeling (mapping entities to classes)
5. OOP in AI-native development (agents as objects)

**Lesson 1**: What is OOP? Why OOP?
- 💬 AI Colearning Prompt: "Ask your AI: Why did Python adopt OOP when it could have stayed purely procedural?"
- 🎓 Instructor Commentary: "In AI-driven development, OOP isn't about memorizing syntax—it's about thinking in systems of interacting entities"
- 🚀 CoLearning Challenge: "Ask your AI Co-Teacher: Generate a simple class that models a thermometer. Explain why we'd use a class instead of just functions."
- ✨ Teaching Tip: "Use Claude Code to explore OOP history: 'How did OOP evolve from Simula to Python?'"

---

### Section 2: Classes, Objects, and Construction

**Concepts to Cover** (7 concepts):
1. Class definition syntax (class keyword, naming conventions)
2. __init__ constructor method
3. self keyword (instance reference)
4. Creating objects (instantiation)
5. Parameterized vs default constructors
6. __del__ destructor (cleanup)
7. Class vs instance attributes (__dict__ inspection)

**Lesson 2**: Classes and Objects Basics
- Teaching pattern: What it is → Code → Try → Why it matters
- 💬 AI Colearning Prompt: "Ask your AI: What happens in memory when we create an object?"
- 🚀 CoLearning Challenge: "Generate a Vehicle class with brand, model, year. Then explain the difference between the class and an instance."

**Lesson 3**: Constructors, Destructors, and Attributes
- Parameterized constructors with type hints
- When to use __del__ (file handles, connections)
- Class attributes shared across instances
- Instance attributes unique to each object
- 💬 AI Colearning Prompt: "Explain why modifying a class attribute through an instance creates a new instance attribute"

---

### Section 3: Encapsulation and Methods

**Concepts to Cover** (8 concepts):
1. Public attributes (no prefix)
2. Protected attributes (_single_underscore convention)
3. Private attributes (__double_underscore name mangling)
4. Instance methods (operate on self)
5. @classmethod decorator (operate on cls)
6. @staticmethod decorator (utility functions)
7. @property decorator (getters)
8. @property.setter (setters)

**Lesson 4**: Encapsulation and Method Types
- Access control patterns
- When to use each method type
- Property decorators vs manual getters/setters
- 💬 AI Colearning Prompt: "Why does Python use naming conventions (_name, __name) instead of true access modifiers like private/public keywords?"
- 🚀 CoLearning Challenge: "Create a BankAccount class with private __balance. Use @property to control access and validate withdrawals."

---

### Section 4: Abstract Base Classes (Brief Introduction)

**Concepts to Cover** (3 concepts):
1. What is abstraction? (hiding complexity, showing only essentials)
2. ABC module (abc.ABC, @abstractmethod)
3. Why abstract classes exist (enforce implementation contracts)

**Note**: This is a BRIEF introduction only. Deep dive with polymorphism patterns in Chapter 25.

**In Lesson 4**: Add subsection after methods:
- Show simple ABC example (Shape with area() abstract method)
- Explain "contract" concept (all shapes MUST implement area())
- 💬 AI Colearning Prompt: "Ask your AI: Why would we create a class we can't instantiate? What problem does ABC solve?"

---

### Section 5: Capstone - Game Character System

**Lesson 5**: Building a Game Character System

**Classes to Build**:
1. **Character** (base class with health, name, level)
2. **Player** (extends Character with inventory, experience)
3. **Enemy** (extends Character with loot, difficulty)
4. **Inventory** (manages items, weight limits)
5. **Action** (attack, defend, use_item methods)

**Concepts Applied**:
- Multiple interacting classes
- Encapsulation (private health with getter/setter)
- Class methods (factory for creating character types)
- Static methods (damage calculations)
- Property decorators (computed attributes like is_alive)

**🚀 CoLearning Challenge**:
"Ask your AI to design the class structure for a turn-based combat system. Then implement it together, explaining each design choice."

---

### Common Mistakes

1. **Confusing class and instance attributes**: Modifying class attribute creates instance shadow
2. **Forgetting self parameter**: Instance methods must have self as first parameter
3. **Accessing private attributes directly**: Use getters or property decorators
4. **Overusing @staticmethod**: If it doesn't need self or cls, maybe it shouldn't be in the class
5. **Skipping type hints**: Modern Python requires type hints for clarity and AI collaboration

---

### AI Exercise Integration

Throughout all lessons, students use Claude Code or Gemini CLI to:
- Explore concepts through dialogue
- Generate code from descriptions
- Validate understanding through questions
- Debug errors with AI assistance
- Extend examples beyond chapter scope

**Framing** (Part 4 appropriate):
- ✅ "Describe what your class should do..."
- ✅ "Ask your AI to explain..."
- ✅ "Validate your code works by testing..."
- ❌ NO "Write a specification" (that's Part 5)
- ❌ NO "Specification-Driven Development" (formal SDD is Chapters 30+)

## Code Examples *(specifications for 8-12 examples)*

### Example 1: Simple Class with Constructor
**Purpose**: Demonstrate basic class creation and object instantiation
**Complexity**: Beginner (A2)
**Prompt**: "Create a Dog class with name and breed attributes. Add a bark() method."
**Key Teaching Point**: Class as blueprint, object as instance

### Example 2: Parameterized Constructor
**Purpose**: Show __init__ with parameters and type hints
**Complexity**: Beginner (A2)
**Prompt**: "Create a Car class with make: str, model: str, year: int. Add a display_info() method."
**Key Teaching Point**: Type hints from the start

### Example 3: Class vs Instance Attributes
**Purpose**: Demonstrate shared class attributes vs unique instance attributes
**Complexity**: Intermediate (B1)
**Prompt**: "Create a BankAccount class with a class attribute interest_rate and instance attribute balance."
**Key Teaching Point**: When to use class-level data

### Example 4: Instance, Class, and Static Methods
**Purpose**: Show all three method types in one class
**Complexity**: Intermediate (B1)
**Prompt**: "Create a Temperature class with instance method to_fahrenheit(), class method from_fahrenheit(), and static method is_boiling()."
**Key Teaching Point**: Choose the right method type for the task

### Example 5: Encapsulation with Private Attributes
**Purpose**: Demonstrate public/protected/private naming
**Complexity**: Intermediate (B1)
**Prompt**: "Create a Person class with public name, protected _age, private __ssn. Add getters for age and ssn."
**Key Teaching Point**: Data protection patterns

### Example 6: Property Decorators
**Purpose**: Show @property, @setter, @deleter
**Complexity**: Advanced (B2)
**Prompt**: "Create a Circle class with private __radius. Use @property for radius getter and setter with validation (radius > 0)."
**Key Teaching Point**: Pythonic getters/setters

### Example 7: Abstract Base Class (Brief)
**Purpose**: Introduce ABC concept
**Complexity**: Advanced (B2)
**Prompt**: "Create an abstract Shape class with abstract area() method. Then create Circle and Square subclasses."
**Key Teaching Point**: Enforcing implementation contracts

### Example 8: Destructor Usage
**Purpose**: Show __del__ for cleanup
**Complexity**: Advanced (B2)
**Prompt**: "Create a FileHandler class that opens a file in __init__ and closes it in __del__."
**Key Teaching Point**: Resource cleanup

### Example 9-12: Game Character System (Capstone)
**Purpose**: Integrate all Part I concepts
**Complexity**: Advanced (B2)
**Prompts**:
- Example 9: "Create Character base class with health, name, level"
- Example 10: "Create Player class with inventory and experience"
- Example 11: "Create Enemy class with loot and difficulty"
- Example 12: "Implement combat system with Action class"
**Key Teaching Point**: Real-world multi-class design

## Acceptance Criteria *(references evals)*

### Content Quality
- [ ] All 4 OOP pillars introduced clearly *(references EVAL-001)*
- [ ] No forward references to Chapter 25 topics (inheritance patterns, polymorphism, MRO)
- [ ] All code examples use Python 3.14+ with type hints
- [ ] 💬🎓🚀✨ CoLearning elements present in every lesson *(references EVAL-009)*
- [ ] Conversational tone (not documentation style)
- [ ] AI positioned as co-reasoning partner

### Lesson Structure
- [ ] 5 lessons total (4 foundational + 1 capstone)
- [ ] Each lesson ends with "Try With AI" section ONLY (4 prompts, progressive) *(references EVAL-007)*
- [ ] No summaries, checklists, or "what's next" after Try With AI
- [ ] Lesson progression: Concept → Code → Try → Why it matters

### Pedagogical Compliance
- [ ] Lesson N uses only concepts from Lessons 1 to N (no forward references)
- [ ] All methods/functions introduced BEFORE first use
- [ ] Built-ins distinguished from methods in explanations
- [ ] Type hints introduced from Example 1 onward
- [ ] Max 10 concepts per lesson (B1-B2 cognitive load limit)

### Capstone Quality
- [ ] Game Character System integrates all Part I concepts *(references EVAL-006, EVAL-008)*
- [ ] At least 4 interacting classes (Character, Player, Enemy, Inventory/Action)
- [ ] Demonstrates encapsulation, constructors, all method types
- [ ] Working code students can extend

### Validation Against Evals
- [ ] Content supports 80%+ explaining OOP pillars *(EVAL-001)*
- [ ] Exercises enable 85%+ creating classes from descriptions *(EVAL-004)*
- [ ] Capstone enables 75%+ building multi-class systems *(EVAL-006)*
- [ ] All code examples tested and working *(EVAL-011)*
- [ ] Flesch-Kincaid Grade Level 10-12 *(EVAL-010)*

### AI-Native Learning Pattern
- [ ] "Describe intent" framing (NOT "write specifications")
- [ ] AI partnership emphasized throughout
- [ ] Validation-first approach (test understanding before moving on)
- [ ] Error literacy (learning from mistakes with AI help)

## Assumptions

1. **Student Background**: Students have completed Chapters 1-23 and are comfortable with functions, loops, data structures, and exception handling

2. **AI Tool Access**: Students have access to Claude Code or Gemini CLI for CoLearning prompts

3. **Time Allocation**: Each lesson takes 45-60 minutes; total chapter completion: 4-5 hours

4. **Part I vs Part II Split**:
   - Part I (Ch24): OOP foundations (classes, objects, encapsulation, methods, ABC intro)
   - Part II (Ch25): Advanced patterns (inheritance, polymorphism, MRO, composition, design patterns, deep ABC usage)

5. **ABC Placement**: Brief introduction in Chapter 24 (what it is, why it exists) with one simple example. Deep dive with polymorphism in Chapter 25.

6. **Python Version**: All examples target Python 3.14+ (latest stable release)

7. **Reading Level**: B1-B2 CEFR (Flesch-Kincaid Grade Level 10-12)

8. **Code Testing**: All examples are tested in Python 3.14+ environment before publication

## Out of Scope (For Chapter 25)

- Inheritance hierarchies and super()
- Multiple inheritance and Method Resolution Order (MRO)
- Polymorphism and duck typing
- Composition vs inheritance design decisions
- Special/magic methods (__str__, __repr__, __add__, etc.)
- Design patterns (Singleton, Factory, Observer, Strategy)
- Metaclasses
- Deep dive into Abstract Base Classes with polymorphism
- Decorators beyond @property (covered in Chapter 25)

## Dependencies

- **Chapter 20 (Module and Functions)**: Students need function knowledge for methods
- **Chapter 21 (Exception Handling)**: Error handling in constructors/methods
- **Chapter 22 (IO and File Handling)**: For FileHandler destructor example
- **Chapter 23 (Math, Date Time Calendar)**: For practical class examples

## Success Criteria *(technology-agnostic, measurable)*

### Learning Outcomes
- **SC-001**: 80% of students can create a class from a written description without AI assistance *(aligns with EVAL-004)*
- **SC-002**: 75% can identify and fix encapsulation violations in code review exercises *(aligns with EVAL-005)*
- **SC-003**: 70% can explain when to use @classmethod vs @staticmethod vs instance method *(aligns with EVAL-003)*
- **SC-004**: 85% complete the Game Character System capstone successfully *(aligns with EVAL-008)*

### Engagement Metrics
- **SC-005**: Average lesson completion rate of 90% or higher *(aligns with EVAL-007)*
- **SC-006**: 75% of students engage with at least 12 out of 20 "Try With AI" prompts *(aligns with EVAL-009)*
- **SC-007**: Capstone project submission rate of 80% or higher

### Content Quality
- **SC-008**: Flesch-Kincaid readability score between 10-12 (appropriate for B1-B2) *(aligns with EVAL-010)*
- **SC-009**: Zero critical errors in code examples when tested on Python 3.14+ *(aligns with EVAL-011)*
- **SC-010**: Average student-reported difficulty rating of 3.5-4.0 out of 5 (appropriately challenging)

### Skill Retention
- **SC-011**: 70% of students can apply OOP concepts in Chapter 25 (Part II) without reviewing Chapter 24
- **SC-012**: 65% can build a new multi-class system (different from capstone) in under 90 minutes

## Notes

- This specification describes **educational content**, not a software feature
- Success is measured by **student learning outcomes**, not system performance
- The capstone (Game Character System) is a **teaching tool**, not a production game
- AI CoLearning is **integral** to the pedagogy, not optional
- Chapter 24 and Chapter 25 are **sequential dependencies** (Part I must be completed before Part II)
