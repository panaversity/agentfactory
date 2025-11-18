# Implementation Tasks: Chapters 24 & 25 (OOP Part I & II)

**Feature**: Object-Oriented Programming (Chapters 24-25)
**Feature Branch**: `020-oop-part-1-2`
**Created**: 2025-11-09
**Status**: Ready for Implementation

## Overview

This document provides actionable implementation tasks for creating Chapters 24 and 25 of the AI-Native Software Development book. Tasks are organized by learning journey (user story equivalent for educational content) to enable independent, incremental implementation.

**Total Lessons**: 10 (5 per chapter)
**Total Duration**: ~10 hours of student instruction
**Target Proficiency**: A2 → B2 (CEFR)

---

## Implementation Strategy

### MVP Approach (Minimum Viable Chapter)

**Chapter 24 MVP**: Learning Journey 1 + 2 (Lessons 1-3)
- Delivers: OOP fundamentals, classes/objects, constructors/attributes
- Student can: Explain OOP pillars, create basic classes, understand attributes
- Independent test: Create a class with constructor and attributes

**Chapter 25 MVP**: Learning Journey 1 + 2 (Lessons 1-2)
- Prerequisite: Chapter 24 complete
- Delivers: Inheritance/MRO, polymorphism/duck typing
- Student can: Create inheritance hierarchies, implement polymorphic code
- Independent test: Create ABC-based polymorphic system

### Incremental Delivery

1. **Phase 1-2**: Setup and foundational tasks (both chapters)
2. **Phase 3**: Chapter 24 Learning Journey 1 (Lessons 1-2)
3. **Phase 4**: Chapter 24 Learning Journey 2 (Lesson 3)
4. **Phase 5**: Chapter 24 Learning Journey 3 (Lesson 4)
5. **Phase 6**: Chapter 24 Learning Journey 4 (Lesson 5 - Capstone)
6. **Phase 7**: Chapter 25 Learning Journey 1 (Lesson 1)
7. **Phase 8**: Chapter 25 Learning Journey 2 (Lesson 2)
8. **Phase 9**: Chapter 25 Learning Journey 3 (Lesson 3)
9. **Phase 10**: Chapter 25 Learning Journey 4 (Lesson 4)
10. **Phase 11**: Chapter 25 Learning Journey 5 (Lesson 5 - Capstone)
11. **Phase 12**: Polish & cross-cutting concerns

---

## Phase 1: Setup & Project Initialization

**Goal**: Establish directory structure, create chapter readme files, set up validation environment

**Independent Test**: Directory structure exists, Python 3.14+ environment ready

### Tasks

- [ ] T001 Create Chapter 24 directory: `book-source/docs/04-Part-4-Python-Fundamentals/24-oop-part-1/`
- [ ] T002 Create Chapter 25 directory: `book-source/docs/04-Part-4-Python-Fundamentals/25-oop-part-2/`
- [ ] T003 [P] Create Chapter 24 readme.md with chapter overview, navigation, and learning objectives
- [ ] T004 [P] Create Chapter 25 readme.md with chapter overview, navigation, and prerequisites note
- [ ] T005 Verify Python 3.14+ environment for code testing

---

## Phase 2: Foundational Tasks (Blocking Prerequisites)

**Goal**: Create reusable templates, establish CoLearning pedagogy patterns, set up code validation

**Independent Test**: Lesson template exists with all required sections, code validator ready

### Tasks

- [ ] T006 [P] Document "Try With AI" section template (4 prompts: Recall → Understand → Apply → Analyze)
- [ ] T007 [P] Document CoLearning element patterns (💬🎓🚀✨ placement guidelines)
- [ ] T008 Create Python code validation script for testing all examples on 3.14+
- [ ] T009 Document pedagogical ordering checklist (no forward references validation)

---

## Phase 3: Chapter 24 Learning Journey 1 - OOP Fundamentals (Lessons 1-2)

**Priority**: P1 (Foundation for all other chapters)
**Goal**: Student understands OOP pillars and can create basic classes
**Independent Test**: Student can explain 4 OOP pillars and create a class with `__init__`

**Story Context**: Learning Journeys 1 from spec-chapter-24.md

### Tasks

#### Lesson 1: OOP Fundamentals

- [ ] T010 [LJ1-CH24] Write Lesson 1 introduction: "What is OOP? Why OOP?" in `24-oop-part-1/01-oop-fundamentals.md`
- [ ] T011 [LJ1-CH24] Write "What is OOP?" section (10min): definition, procedural vs OOP, blueprint analogy
- [ ] T012 [LJ1-CH24] Write "The Four Pillars" section (15min): Encapsulation, Abstraction, Inheritance preview, Polymorphism preview
- [ ] T013 [LJ1-CH24] Add 💬 AI Colearning Prompt: "Why did Python adopt OOP when it could have stayed procedural?"
- [ ] T014 [LJ1-CH24] Add 🎓 Instructor Commentary: "OOP isn't about memorizing syntax—it's about thinking in systems"
- [ ] T015 [LJ1-CH24] Write "Why Use OOP?" section (10min): Modularity, Reusability, Maintainability, Scalability
- [ ] T016 [LJ1-CH24] Write "OOP in AI-Native Development" section (10min): agents as objects, multi-agent systems
- [ ] T017 [LJ1-CH24] Add 🚀 CoLearning Challenge: "Generate a thermometer class, explain why class vs functions"
- [ ] T018 [LJ1-CH24] Add ✨ Teaching Tip: "Use Claude Code to explore OOP history"
- [ ] T019 [LJ1-CH24] Write "Try With AI" section with 4 prompts (Recall → Understand → Apply → Analyze)
- [ ] T020 [LJ1-CH24] Validate Lesson 1: No forward references, conversational tone, CEFR A2-B1 appropriate

#### Lesson 2: Classes and Objects

- [ ] T021 [LJ1-CH24] Write Lesson 2 introduction: "Classes and Objects Basics" in `24-oop-part-1/02-classes-and-objects.md`
- [ ] T022 [LJ1-CH24] Write "Defining a Class" section (10min): `class` keyword, naming conventions, empty class
- [ ] T023 [LJ1-CH24] Add Code Example 1: Empty Dog class with `pass`
- [ ] T024 [LJ1-CH24] Write "The __init__ Constructor" section (15min): special method, self parameter, type hints
- [ ] T025 [LJ1-CH24] Add Code Example 2: Dog class with parameterized constructor
- [ ] T026 [LJ1-CH24] Add 💬 AI Colearning Prompt: "What happens in memory when we call Dog('Max', 'Labrador')?"
- [ ] T027 [LJ1-CH24] Write "Creating Objects (Instantiation)" section (10min): creating instances, accessing attributes
- [ ] T028 [LJ1-CH24] Add Code Example 3: Creating dog1 and dog2 objects
- [ ] T029 [LJ1-CH24] Write "Understanding self" section (10min): instance reference, why Python requires it
- [ ] T030 [LJ1-CH24] Add 🎓 Instructor Commentary: "self means 'this specific object' in AI agent context"
- [ ] T031 [LJ1-CH24] Write "Adding Simple Methods" section (5min): bark() method example
- [ ] T032 [LJ1-CH24] Add 🚀 CoLearning Challenge: "Generate Vehicle class with type hints, explain why type hints matter"
- [ ] T033 [LJ1-CH24] Add ✨ Teaching Tip: "Explore how Python's built-in list class is defined"
- [ ] T034 [LJ1-CH24] Write "Try With AI" section with 4 prompts
- [ ] T035 [LJ1-CH24] Test all code examples in Lessons 1-2 on Python 3.14+
- [ ] T036 [LJ1-CH24] Validate Lessons 1-2: Pedagogical ordering (L2 uses only L1+L2 concepts), conversational tone

---

## Phase 4: Chapter 24 Learning Journey 2 - Constructors & Attributes (Lesson 3)

**Priority**: P2 (Builds on LJ1)
**Goal**: Student can initialize objects and manage attribute visibility
**Independent Test**: Student creates class with parameterized constructor and public/protected/private attributes

**Story Context**: Learning Journey 2 from spec-chapter-24.md

### Tasks

- [ ] T037 [LJ2-CH24] Write Lesson 3 introduction: "Constructors, Destructors, and Attributes" in `24-oop-part-1/03-constructors-attributes.md`
- [ ] T038 [LJ2-CH24] Write "Parameterized vs Default Constructors" section (15min): with/without defaults
- [ ] T039 [LJ2-CH24] Add Code Example 4: Car class with default parameters
- [ ] T040 [LJ2-CH24] Add 💬 AI Colearning Prompt: "When should we use default parameters? 3 scenarios + 1 danger"
- [ ] T041 [LJ2-CH24] Write "Class Attributes vs Instance Attributes" section (20min): shared vs unique, shadowing behavior
- [ ] T042 [LJ2-CH24] Add Code Example 5: BankAccount with class attribute interest_rate
- [ ] T043 [LJ2-CH24] Add 🎓 Instructor Commentary: "Class vs instance critical for AI agents - config vs conversation history"
- [ ] T044 [LJ2-CH24] Add 💬 AI Colearning Prompt: "Why does modifying class attribute through instance create instance attribute?"
- [ ] T045 [LJ2-CH24] Write "The __dict__ Attribute (Inspection)" section (10min): debugging with __dict__
- [ ] T046 [LJ2-CH24] Add Code Example 6: Person class showing __dict__ inspection
- [ ] T047 [LJ2-CH24] Add ✨ Teaching Tip: "Use __dict__ to debug attribute issues with Claude"
- [ ] T048 [LJ2-CH24] Write "Destructors (__del__)" section (10min): cleanup pattern, when reliable/unreliable
- [ ] T049 [LJ2-CH24] Add Code Example 7: FileHandler with __del__ for file closing
- [ ] T050 [LJ2-CH24] Add 🚀 CoLearning Challenge: "Create DatabaseConnection class, explain __del__ unreliability"
- [ ] T051 [LJ2-CH24] Write "Try With AI" section with 4 prompts
- [ ] T052 [LJ2-CH24] Test all code examples in Lesson 3 on Python 3.14+
- [ ] T053 [LJ2-CH24] Validate Lesson 3: Uses only L1-L3 concepts, no forward references to methods/encapsulation

---

## Phase 5: Chapter 24 Learning Journey 3 - Encapsulation & Methods (Lesson 4)

**Priority**: P3 (Builds on LJ1-2)
**Goal**: Student can implement encapsulation and use all three method types
**Independent Test**: Student creates class with public/protected/private attributes, instance/class/static methods, and property decorators

**Story Context**: Learning Journey 3 from spec-chapter-24.md

### Tasks

- [ ] T054 [LJ3-CH24] Write Lesson 4 introduction: "Encapsulation and Method Types" in `24-oop-part-1/04-encapsulation-methods.md`
- [ ] T055 [LJ3-CH24] Write "Encapsulation - Public, Protected, Private" section (20min): naming conventions, philosophy
- [ ] T056 [LJ3-CH24] Add Code Example 8: Person class with public name, protected _age, private __ssn
- [ ] T057 [LJ3-CH24] Add 🎓 Instructor Commentary: "Python's encapsulation is convention, not enforcement - 'we're all consenting adults'"
- [ ] T058 [LJ3-CH24] Add 💬 AI Colearning Prompt: "Why naming conventions vs true private keywords? Tradeoffs?"
- [ ] T059 [LJ3-CH24] Write "Instance Methods" section (10min): operates on self
- [ ] T060 [LJ3-CH24] Add Code Example 9: Circle class with area() and circumference() methods
- [ ] T061 [LJ3-CH24] Write "Class Methods (@classmethod)" section (12min): factory pattern, operates on cls
- [ ] T062 [LJ3-CH24] Add Code Example 10: Temperature class with from_fahrenheit() and from_kelvin() factories
- [ ] T063 [LJ3-CH24] Add 🚀 CoLearning Challenge: "Create Date class with from_string() class method"
- [ ] T064 [LJ3-CH24] Write "Static Methods (@staticmethod)" section (10min): utility functions, no self/cls needed
- [ ] T065 [LJ3-CH24] Add Code Example 11: MathUtils class with is_prime() and factorial() static methods
- [ ] T066 [LJ3-CH24] Add 💬 AI Colearning Prompt: "When @staticmethod vs regular function outside class?"
- [ ] T067 [LJ3-CH24] Write "Property Decorators (@property)" section (18min): getters, setters, computed properties
- [ ] T068 [LJ3-CH24] Add Code Example 12: Circle class with radius property (getter/setter with validation)
- [ ] T069 [LJ3-CH24] Add 🎓 Instructor Commentary: "Property decorators are Pythonic encapsulation - powerful for AI agents"
- [ ] T070 [LJ3-CH24] Add ✨ Teaching Tip: "Explore built-in property usage with Claude"
- [ ] T071 [LJ3-CH24] Write "Abstract Base Classes (Brief Introduction)" section (10min): concept only, contract enforcement
- [ ] T072 [LJ3-CH24] Add Code Example 13: Shape ABC with abstract area() and perimeter(), Rectangle implementation
- [ ] T073 [LJ3-CH24] Add 🎓 Instructor Commentary: "ABCs enforce contracts - brief intro, deep dive in Ch25"
- [ ] T074 [LJ3-CH24] Add 💬 AI Colearning Prompt: "Why create a class we can't instantiate? What problem does ABC solve?"
- [ ] T075 [LJ3-CH24] Write "Try With AI" section with 4 prompts
- [ ] T076 [LJ3-CH24] Test all code examples in Lesson 4 on Python 3.14+
- [ ] T077 [LJ3-CH24] Validate Lesson 4: Uses only L1-L4 concepts, max 10 concepts (B2 limit), conversational tone

---

## Phase 6: Chapter 24 Learning Journey 4 - Game Character Capstone (Lesson 5)

**Priority**: P4 (Synthesis of LJ1-3)
**Goal**: Student builds multi-class object system integrating all Chapter 24 concepts
**Independent Test**: Student completes working Game Character System with 4+ interacting classes

**Story Context**: Learning Journey 4 from spec-chapter-24.md

### Tasks

- [ ] T078 [LJ4-CH24] Write Lesson 5 introduction: "Game Character System (Capstone)" in `24-oop-part-1/05-game-character-capstone.md`
- [ ] T079 [LJ4-CH24] Write "Project Overview" section: turn-based combat, class responsibilities
- [ ] T080 [LJ4-CH24] Write "Part 1: Design Phase" (15min): AI-guided design, class structure planning
- [ ] T081 [LJ4-CH24] Add 💬 AI Colearning Prompt: "Ask me clarifying questions about game requirements, suggest class structure"
- [ ] T082 [LJ4-CH24] Write "Part 2: Character Base Class" (15min): health, name, level, is_alive property, ABC pattern
- [ ] T083 [LJ4-CH24] Add Code Example 14: Character abstract base class with properties and abstract attack()
- [ ] T084 [LJ4-CH24] Add 🎓 Instructor Commentary: "Notice encapsulation + properties + computed properties + ABC - professional OOP"
- [ ] T085 [LJ4-CH24] Write "Part 3: Player Class" (15min): inventory, experience, level_up system
- [ ] T086 [LJ4-CH24] Add Code Example 15: Player class with Item, add_item(), use_item(), gain_experience()
- [ ] T087 [LJ4-CH24] Add 🚀 CoLearning Challenge: "Add mana system to Player with magic_attack() and validation"
- [ ] T088 [LJ4-CH24] Write "Part 4: Enemy Class" (10min): loot system, difficulty scaling
- [ ] T089 [LJ4-CH24] Add Code Example 16: Enemy class with difficulty-based damage multiplier
- [ ] T090 [LJ4-CH24] Write "Part 5: Combat System" (15min): static methods for battle management
- [ ] T091 [LJ4-CH24] Add Code Example 17: Combat class with battle() and calculate_critical_hit() static methods
- [ ] T092 [LJ4-CH24] Write "Part 6: Integration" (10min): putting it all together, example gameplay
- [ ] T093 [LJ4-CH24] Add Code Example 18: Complete game flow (create hero, enemies, battles)
- [ ] T094 [LJ4-CH24] Add 🚀 CoLearning Challenge: "Extend with Boss, Shop, save/load JSON - design architecture with AI"
- [ ] T095 [LJ4-CH24] Write "Try With AI" section with 4 prompts (includes design review, tradeoffs, extending, AI agents connection)
- [ ] T096 [LJ4-CH24] Test complete Game Character System on Python 3.14+
- [ ] T097 [LJ4-CH24] Validate Lesson 5: All Ch24 concepts integrated, working code, capstone quality

---

## Phase 7: Chapter 25 Learning Journey 1 - Inheritance & MRO (Lesson 1)

**Priority**: P1 (Foundation for Ch25, requires Ch24 complete)
**Goal**: Student masters inheritance hierarchies and Method Resolution Order
**Independent Test**: Student creates 3-level inheritance hierarchy with proper super() usage, explains MRO

**Story Context**: Learning Journey 1 from spec-chapter-25.md
**Prerequisite**: Chapter 24 complete

### Tasks

- [ ] T098 [LJ1-CH25] Write Lesson 1 introduction: "Inheritance and Method Resolution Order" in `25-oop-part-2/01-inheritance-mro.md`
- [ ] T099 [LJ1-CH25] Write "Single Inheritance" section (15min): parent-child relationships
- [ ] T100 [LJ1-CH25] Add Code Example 19: Animal parent, Dog child with speak() override
- [ ] T101 [LJ1-CH25] Write "The super() Function" section (10min): calling parent methods, why super() vs direct call
- [ ] T102 [LJ1-CH25] Add 💬 AI Colearning Prompt: "Explain super().__init__(name). What if I forget it? Memory state?"
- [ ] T103 [LJ1-CH25] Write "Multiple Inheritance" section (15min): inheriting from 2+ parents
- [ ] T104 [LJ1-CH25] Add Code Example 20: Duck(Flyer, Swimmer) with fly(), swim(), quack()
- [ ] T105 [LJ1-CH25] Write "Diamond Inheritance Problem" section (10min): diamond hierarchy challenge
- [ ] T106 [LJ1-CH25] Add Code Example 21: Diamond (A, B(A), C(A), D(B,C)) with greet() method
- [ ] T107 [LJ1-CH25] Write "Method Resolution Order (MRO)" section (10min): C3 linearization algorithm
- [ ] T108 [LJ1-CH25] Add 🎓 Instructor Commentary: "C3 Linearization ensures: subclasses before parents, order preserved, no double visits"
- [ ] T109 [LJ1-CH25] Add Code Example 22: MRO visualization for diamond hierarchy
- [ ] T110 [LJ1-CH25] Add 🚀 CoLearning Challenge: "Create Vehicle diamond hierarchy, ask AI to explain MRO"
- [ ] T111 [LJ1-CH25] Write "Try With AI" section with 4 prompts (includes when NOT to use inheritance)
- [ ] T112 [LJ1-CH25] Test all code examples in Lesson 1 on Python 3.14+
- [ ] T113 [LJ1-CH25] Validate Lesson 1: Uses Ch24 + L1 concepts only, B1-B2 appropriate

---

## Phase 8: Chapter 25 Learning Journey 2 - Polymorphism (Lesson 2)

**Priority**: P2 (Builds on LJ1-CH25)
**Goal**: Student implements polymorphic behavior using ABC and duck typing
**Independent Test**: Student creates polymorphic code with ABC and duck typing examples

**Story Context**: Learning Journey 2 from spec-chapter-25.md

### Tasks

- [ ] T114 [LJ2-CH25] Write Lesson 2 introduction: "Polymorphism and Duck Typing" in `25-oop-part-2/02-polymorphism-duck-typing.md`
- [ ] T115 [LJ2-CH25] Write "Method Overriding" section (10min): subclass replaces parent method
- [ ] T116 [LJ2-CH25] Add Code Example 23: Shape parent, Circle/Rectangle override area()
- [ ] T117 [LJ2-CH25] Write "Abstract Base Classes (Deep Dive)" section (20min): enforcing contracts with ABC
- [ ] T118 [LJ2-CH25] Add Code Example 24: Agent ABC with abstract process() and get_capabilities(), ChatAgent/CodeAgent implementations
- [ ] T119 [LJ2-CH25] Add 🎓 Instructor Commentary: "ABCs enforce contracts at class level - CRITICAL for AI agents"
- [ ] T120 [LJ2-CH25] Add 💬 AI Colearning Prompt: "ABC with @abstractmethod vs base class with NotImplementedError - which better?"
- [ ] T121 [LJ2-CH25] Write "Abstract Properties and Class Methods" section (10min): @property + @abstractmethod
- [ ] T122 [LJ2-CH25] Add Code Example 25: DataSource ABC with abstract connection_string property and from_config() factory
- [ ] T123 [LJ2-CH25] Write "Duck Typing" section (15min): polymorphism without inheritance
- [ ] T124 [LJ2-CH25] Add Code Example 26: FileReader, URLReader, DatabaseReader - no inheritance, all have read()
- [ ] T125 [LJ2-CH25] Add 🎓 Instructor Commentary: "Duck typing: 'If it walks like a duck...' - Python prefers this"
- [ ] T126 [LJ2-CH25] Add 🚀 CoLearning Challenge: "Design PaymentProcessor ABC, implement 3 processors, show duck typing alternative"
- [ ] T127 [LJ2-CH25] Write "Try With AI" section with 4 prompts (includes Protocol vs ABC - advanced)
- [ ] T128 [LJ2-CH25] Test all code examples in Lesson 2 on Python 3.14+
- [ ] T129 [LJ2-CH25] Validate Lesson 2: Uses Ch24 + L1-L2 concepts, B2 proficiency

---

## Phase 9: Chapter 25 Learning Journey 3 - Composition & Modules (Lesson 3)

**Priority**: P3 (Builds on LJ1-2-CH25)
**Goal**: Student chooses composition over inheritance appropriately and organizes code into modules
**Independent Test**: Student refactors inheritance to composition and creates package structure

**Story Context**: Learning Journey 3 from spec-chapter-25.md

### Tasks

- [ ] T130 [LJ3-CH25] Write Lesson 3 introduction: "Composition Over Inheritance + Modules" in `25-oop-part-2/03-composition-modules.md`
- [ ] T131 [LJ3-CH25] Write "Composition ('has-a')" section (15min): Car HAS-AN Engine pattern
- [ ] T132 [LJ3-CH25] Add Code Example 27: Engine class, Car class with composition
- [ ] T133 [LJ3-CH25] Write "Aggregation vs Composition" section (10min): strong vs weak ownership, lifecycle
- [ ] T134 [LJ3-CH25] Add Code Example 28: Car composition (owns Engine) vs University aggregation (references Departments)
- [ ] T135 [LJ3-CH25] Add 🎓 Instructor Commentary: "Composition: container owns. Aggregation: container references"
- [ ] T136 [LJ3-CH25] Write "Composition Over Inheritance Principle" section (15min): when composition is better
- [ ] T137 [LJ3-CH25] Add Code Example 29: Bad inheritance (Car IS-AN Engine) vs Good composition (Car HAS-AN Engine)
- [ ] T138 [LJ3-CH25] Add 💬 AI Colearning Prompt: "Why 'composition over inheritance'? 5 reasons with code examples"
- [ ] T139 [LJ3-CH25] Add 🚀 CoLearning Challenge: "Refactor Bird inheritance (Penguin can't fly) to composition"
- [ ] T140 [LJ3-CH25] Write "Organizing Classes into Modules" section (15min): directory structure, __init__.py
- [ ] T141 [LJ3-CH25] Add Code Example 30: Package structure (animals/, vehicles/) with proper imports
- [ ] T142 [LJ3-CH25] Add ✨ Teaching Tip: "Explore Django package organization with Claude"
- [ ] T143 [LJ3-CH25] Write "Try With AI" section with 4 prompts (includes Zoo design, refactoring, e-commerce structure)
- [ ] T144 [LJ3-CH25] Test all code examples in Lesson 3 on Python 3.14+
- [ ] T145 [LJ3-CH25] Validate Lesson 3: Uses Ch24 + L1-L3 concepts, design judgment encouraged

---

## Phase 10: Chapter 25 Learning Journey 4 - Special Methods (Lesson 4)

**Priority**: P4 (Builds on LJ1-3-CH25)
**Goal**: Student customizes object behavior with special/magic methods
**Independent Test**: Student implements 5+ special methods to make custom objects Pythonic

**Story Context**: Learning Journey 4 from spec-chapter-25.md

### Tasks

- [ ] T146 [LJ4-CH25] Write Lesson 4 introduction: "Special Methods (Magic/Dunder Methods)" in `25-oop-part-2/04-special-methods.md`
- [ ] T147 [LJ4-CH25] Write "__str__ and __repr__" section (10min): user-friendly vs debug strings
- [ ] T148 [LJ4-CH25] Add Code Example 31: Person class with __str__ and __repr__
- [ ] T149 [LJ4-CH25] Write "Operator Overloading" section (15min): __add__, __sub__, __mul__
- [ ] T150 [LJ4-CH25] Add Code Example 32: Vector class with operator overloading
- [ ] T151 [LJ4-CH25] Add 💬 AI Colearning Prompt: "Show all operator overload methods. When __radd__ vs __add__?"
- [ ] T152 [LJ4-CH25] Write "Container Protocol" section (12min): __len__, __getitem__, __setitem__
- [ ] T153 [LJ4-CH25] Add Code Example 33: Playlist class with container protocol
- [ ] T154 [LJ4-CH25] Write "Iteration Protocol" section (12min): __iter__, __next__
- [ ] T155 [LJ4-CH25] Add Code Example 34: Countdown class with iteration protocol
- [ ] T156 [LJ4-CH25] Add 🚀 CoLearning Challenge: "Create Range class mimicking built-in range()"
- [ ] T157 [LJ4-CH25] Write "Comparison and Hashing" section (10min): __eq__, __lt__, __hash__, @total_ordering
- [ ] T158 [LJ4-CH25] Add Code Example 35: Person class with comparison and hashing for set/dict usage
- [ ] T159 [LJ4-CH25] Add 🎓 Instructor Commentary: "__eq__ without __hash__ breaks sets/dicts - equal objects MUST have equal hashes"
- [ ] T160 [LJ4-CH25] Write "Callable Objects (__call__)" section (6min): making instances callable
- [ ] T161 [LJ4-CH25] Add Code Example 36: Multiplier class with __call__
- [ ] T162 [LJ4-CH25] Add 💬 AI Colearning Prompt: "How does Python's list use special methods? len(), indexing, iteration?"
- [ ] T163 [LJ4-CH25] Write "Try With AI" section with 4 prompts (includes custom collection synthesis)
- [ ] T164 [LJ4-CH25] Test all code examples in Lesson 4 on Python 3.14+
- [ ] T165 [LJ4-CH25] Validate Lesson 4: Uses Ch24 + L1-L4 concepts, max 10 concepts (B2 limit)

---

## Phase 11: Chapter 25 Learning Journey 5 - Design Patterns Capstone (Lesson 5)

**Priority**: P5 (Synthesis of all Ch24-25)
**Goal**: Student implements 4 professional design patterns in integrated AI agent system
**Independent Test**: Student completes capstone with Singleton, Factory, Observer, Strategy patterns working together

**Story Context**: Learning Journey 5 from spec-chapter-25.md

### Tasks

- [ ] T166 [LJ5-CH25] Write Lesson 5 introduction: "Design Patterns (Capstone)" in `25-oop-part-2/05-design-patterns-capstone.md`
- [ ] T167 [LJ5-CH25] Write "Project Overview" section: AI Agent System combining all 4 patterns
- [ ] T168 [LJ5-CH25] Write "Part 1: Singleton Pattern" (15min): AgentManager ensuring single instance
- [ ] T169 [LJ5-CH25] Add Code Example 37: Singleton AgentManager with __new__ implementation
- [ ] T170 [LJ5-CH25] Add 🎓 Instructor Commentary: "Singletons controversial - use for config, logging, not testable state"
- [ ] T171 [LJ5-CH25] Write "Part 2: Factory Pattern" (20min): AgentFactory creating different agent types
- [ ] T172 [LJ5-CH25] Add Code Example 38: Agent ABC, ChatAgent/CodeAgent/DataAgent, AgentFactory.create_agent()
- [ ] T173 [LJ5-CH25] Add 🚀 CoLearning Challenge: "Extend Factory with registry pattern - self-registering agents"
- [ ] T174 [LJ5-CH25] Write "Part 3: Observer Pattern" (20min): EventBus for agent communication
- [ ] T175 [LJ5-CH25] Add Code Example 39: Subject/Observer protocol, EventBus.publish(), Agent.update()
- [ ] T176 [LJ5-CH25] Add 💬 AI Colearning Prompt: "Observer vs polling - why event-driven better for AI agents?"
- [ ] T177 [LJ5-CH25] Write "Part 4: Strategy Pattern" (15min): DecisionStrategy for runtime algorithm switching
- [ ] T178 [LJ5-CH25] Add Code Example 40: DecisionStrategy ABC, AggressiveStrategy/DefensiveStrategy/BalancedStrategy, Agent.set_strategy()
- [ ] T179 [LJ5-CH25] Write "Part 5: Integrated System" (10min): bringing all 4 patterns together
- [ ] T180 [LJ5-CH25] Add Code Example 41: Complete AI Agent System (Singleton manager + Factory creation + Observer events + Strategy decisions)
- [ ] T181 [LJ5-CH25] Add 🚀 CoLearning Challenge: "Add Mediator + Command patterns, design with AI guidance"
- [ ] T182 [LJ5-CH25] Write "Try With AI" section with 4 prompts (pattern recognition, tradeoffs, Command pattern, multi-pattern synthesis)
- [ ] T183 [LJ5-CH25] Test complete Design Patterns system on Python 3.14+
- [ ] T184 [LJ5-CH25] Validate Lesson 5: All Ch24-25 concepts integrated, professional architecture demonstrated

---

## Phase 12: Polish & Cross-Cutting Concerns

**Goal**: Final quality checks, cross-chapter consistency, metadata, documentation

### Tasks

- [ ] T185 Update Chapter 24 readme.md with final lesson links and completion checklist
- [ ] T186 Update Chapter 25 readme.md with final lesson links and completion checklist
- [ ] T187 [P] Run Flesch-Kincaid readability check on all 10 lessons (target: Grade 10-12)
- [ ] T188 [P] Verify all 35 code examples run successfully on Python 3.14+
- [ ] T189 [P] Validate all 10 lessons end with "Try With AI" section ONLY (no summaries/checklists after)
- [ ] T190 [P] Validate all 108 CoLearning elements (💬🎓🚀✨) are present and positioned correctly
- [ ] T191 [P] Check pedagogical ordering compliance across all lessons (no forward references)
- [ ] T192 [P] Verify CEFR proficiency progression (Ch24: A2→B2, Ch25: B2 maintained)
- [ ] T193 Cross-chapter consistency check: Ch25 L1 builds on Ch24 L5 correctly
- [ ] T194 Add YAML frontmatter to all 10 lessons with skills metadata (CEFR levels)
- [ ] T195 Generate lesson navigation (prev/next links) in all lesson files

---

## Dependency Graph

### Chapter 24 Dependencies

```
Setup (T001-T005) ──┐
                    ├─→ Foundational (T006-T009) ──┐
                                                     ├─→ LJ1 L1-2 (T010-T036) ──┐
                                                     │                          ├─→ LJ2 L3 (T037-T053) ──┐
                                                     │                          │                        ├─→ LJ3 L4 (T054-T077) ──┐
                                                     │                          │                        │                         ├─→ LJ4 L5 (T078-T097)
```

### Chapter 25 Dependencies

```
Chapter 24 Complete ──→ LJ1 L1 (T098-T113) ──┐
                                              ├─→ LJ2 L2 (T114-T129) ──┐
                                              │                        ├─→ LJ3 L3 (T130-T145) ──┐
                                              │                        │                        ├─→ LJ4 L4 (T146-T165) ──┐
                                              │                        │                        │                         ├─→ LJ5 L5 (T166-T184)
```

### Cross-Chapter Dependency

```
Chapter 24 Lesson 5 Complete ──→ Chapter 25 can begin
```

---

## Parallel Execution Opportunities

### Phase 3 (Ch24 LJ1):
- T010-T020 (Lesson 1) and T021-T034 (Lesson 2) can be written in parallel by different authors
- T035-T036 (testing & validation) must be sequential after both lessons complete

### Phase 6 (Ch24 Capstone):
- T082-T084 (Character class), T085-T087 (Player class), T088-T089 (Enemy class), T090-T091 (Combat class) can be written in parallel
- T092-T097 (integration & validation) must be sequential

### Phase 12 (Polish):
- T185-T192 (all validation tasks marked [P]) can run in parallel
- T193-T195 (cross-chapter tasks) must be sequential

---

## Task Summary

**Total Tasks**: 195
**Parallelizable Tasks**: 23 (marked with [P])
**Sequential Tasks**: 172

### Tasks by Phase:
- Phase 1 (Setup): 5 tasks
- Phase 2 (Foundational): 4 tasks
- Phase 3 (Ch24 LJ1): 27 tasks
- Phase 4 (Ch24 LJ2): 17 tasks
- Phase 5 (Ch24 LJ3): 24 tasks
- Phase 6 (Ch24 LJ4): 20 tasks
- Phase 7 (Ch25 LJ1): 16 tasks
- Phase 8 (Ch25 LJ2): 16 tasks
- Phase 9 (Ch25 LJ3): 16 tasks
- Phase 10 (Ch25 LJ4): 20 tasks
- Phase 11 (Ch25 LJ5): 19 tasks
- Phase 12 (Polish): 11 tasks

### Tasks by Learning Journey:
- Ch24 LJ1 (P1): 27 tasks → **MVP Core**
- Ch24 LJ2 (P2): 17 tasks → **MVP Core**
- Ch24 LJ3 (P3): 24 tasks
- Ch24 LJ4 (P4 Capstone): 20 tasks
- Ch25 LJ1 (P1): 16 tasks → **Ch25 MVP Core**
- Ch25 LJ2 (P2): 16 tasks → **Ch25 MVP Core**
- Ch25 LJ3 (P3): 16 tasks
- Ch25 LJ4 (P4): 20 tasks
- Ch25 LJ5 (P5 Capstone): 19 tasks

---

## Validation Checklist

After implementation, verify:

- [ ] All 10 lessons exist in correct directories
- [ ] All 35 code examples tested on Python 3.14+
- [ ] All 20 "Try With AI" sections have exactly 4 prompts each
- [ ] All 108 CoLearning elements (💬🎓🚀✨) present
- [ ] All lessons end with "Try With AI" ONLY (no summaries)
- [ ] Pedagogical ordering compliant (no forward references)
- [ ] CEFR proficiency progression validated
- [ ] Flesch-Kincaid Grade Level 10-12
- [ ] Conversational tone throughout (not documentation style)
- [ ] Type hints in all code examples
- [ ] Chapter 24 complete before Chapter 25 begins

---

## Policy Note for Lesson Authors

**Lesson Closure Pattern** (Constitutional Requirement):

Within this chapter, each lesson MUST end with a single final section titled **"Try With AI"** containing 4 prompts with progressive Bloom's levels (Recall → Understand → Apply → Analyze/Synthesize).

**DO NOT include after "Try With AI"**:
- ❌ "Key Takeaways" or "Summary"
- ❌ "What's Next"
- ❌ "Completion Checklist"
- ❌ Any other closure content

**AI Tool Guidance**:
- In Part 4 (these chapters), students use their preferred AI companion tool (Claude Code, Gemini CLI, or ChatGPT web)
- Phrase prompts generically: "Ask your AI companion..." or "Use your AI tool..."
- Optionally provide CLI and web variants for complex prompts

---

**Tasks Status**: ✅ READY FOR IMPLEMENTATION (195 tasks defined)
