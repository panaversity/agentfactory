---
title: "Composition Over Inheritance and Code Organization"
chapter: 25
lesson: 3
duration_minutes: 55

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Composition Design"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain 'has-a' relationships and when composition is more flexible than inheritance"

  - name: "Has-A vs Is-A Relationships"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can evaluate design choices between inheritance and composition in real problems"

  - name: "Aggregation vs Composition"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can distinguish between strong and weak object coupling"

  - name: "Module Organization"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can structure multi-file projects with logical module organization"

  - name: "Package Creation with __init__.py"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create packages and manage public APIs with __init__.py"

  - name: "AI-Assisted Architecture Design"
    proficiency_level: "B2"
    category: "Soft"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use AI to evaluate and refine project structure decisions"

learning_objectives:
  - objective: "Design object systems using composition when inheritance would create rigid hierarchies"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Refactoring an inheritance-based design to use composition with explanation"

  - objective: "Organize OOP code into modules and packages for scalability and maintainability"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Creating a multi-module project with proper __init__.py usage"

  - objective: "Evaluate composition vs inheritance tradeoffs and explain design decisions to others"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Analyzing code examples and justifying architectural choices"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (Composition, Has-A vs Is-A, Aggregation vs Composition, Module organization, __init__.py, Package imports) within B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore circular import prevention; design plugin architectures; analyze Gang of Four patterns that use composition"
  remedial_for_struggling: "Start with simple composition examples (Car has Engine) before aggregation; use visual diagrams for module relationships"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/020-oop-part-1-2/spec-chapter-25.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Composition Over Inheritance and Code Organization

In Lessons 1 and 2, you learned that inheritance is powerful—it lets you build hierarchies of classes where specialized types inherit from general types. Dog **is-a** Animal. ElectricCar **is-a** Car. Inheritance feels natural for modeling these relationships.

But here's the professional secret: **most of the time, composition is better than inheritance.**

This might seem contradictory. You just spent a lesson mastering inheritance! But professional developers have learned through hard experience that inheritance creates rigid class hierarchies that are difficult to change. Composition—the "has-a" relationship—creates flexible, decoupled designs that evolve as requirements change.

In this lesson, you'll learn when to choose composition over inheritance, how to design systems that are easy to modify, and how to organize your code into modules and packages for real-world projects. By the end, you'll understand why experienced architects prefer composition and how to structure Python projects professionally.

---

## Composition: The "Has-A" Relationship

**Composition** means building classes by combining other objects. Instead of inheriting behavior, a class *contains* other objects and delegates work to them.

Let's start with a concrete example. Imagine designing a `Car` class:

```python
class Engine:
    """A Car has-an Engine"""

    def __init__(self, horsepower: int) -> None:
        self.horsepower = horsepower

    def start(self) -> str:
        return f"Engine with {self.horsepower}hp started"

    def stop(self) -> str:
        return "Engine stopped"


class Car:
    """A Car has-an Engine, not IS-AN Engine"""

    def __init__(self, make: str, engine: Engine) -> None:
        self.make = make
        self.engine = engine  # Composition: Car HAS-AN Engine

    def start(self) -> str:
        return f"{self.make}: {self.engine.start()}"

    def stop(self) -> str:
        return f"{self.make}: {self.engine.stop()}"


# Create an engine and give it to a car
engine = Engine(200)
car = Car("Toyota", engine)
print(car.start())  # Toyota: Engine with 200hp started
print(car.stop())   # Toyota: Engine stopped
```

Notice: `Car` doesn't inherit from `Engine`. Instead, `Car` has an `Engine` as an attribute. When you call `car.start()`, the car delegates to its engine's `start()` method.

#### 🎓 Instructor Commentary

> In AI-native development, composition is the default pattern. Multi-agent systems use composition—an orchestrator agent *has* specialized sub-agents. Understanding composition is more critical than mastering inheritance hierarchies.

---

## Why Composition Wins Over Inheritance

Let's see why professionals prefer composition. Consider the design problem: **A penguin is a bird, but penguins can't fly.**

**Using inheritance (the wrong way):**

```python
class Bird:
    def fly(self) -> str:
        return "Flying high!"


class Penguin(Bird):
    # Problem: Penguin inherits fly() but can't actually fly!
    pass


penguin = Penguin()
print(penguin.fly())  # This is wrong - penguins don't fly!
```

This breaks the Liskov Substitution Principle: a subclass should be usable wherever the parent is used. But a Penguin can't reliably replace a Bird—it can't fly!

**Using composition (the right way):**

```python
class Flyer:
    """Capability: ability to fly"""
    def fly(self) -> str:
        return "Flying!"


class Swimmer:
    """Capability: ability to swim"""
    def swim(self) -> str:
        return "Swimming!"


class Bird:
    """A bird has capabilities"""
    def __init__(self, name: str, flyer: Flyer | None = None, swimmer: Swimmer | None = None) -> None:
        self.name = name
        self.flyer = flyer
        self.swimmer = swimmer

    def move(self) -> str:
        if self.flyer:
            return f"{self.name}: {self.flyer.fly()}"
        elif self.swimmer:
            return f"{self.name}: {self.swimmer.swim()}"
        return f"{self.name}: Walking"


class Penguin(Bird):
    """Penguins have swimming but not flying"""
    def __init__(self, name: str) -> None:
        super().__init__(name, flyer=None, swimmer=Swimmer())


# This is correct - penguin swims, doesn't fly
penguin = Penguin("Pingu")
print(penguin.move())  # Pingu: Swimming!
```

Now the design is flexible. Penguins and eagles have different capabilities without forcing an inheritance hierarchy.

#### 💬 AI Colearning Prompt

> "Ask your AI: Why do professional developers prefer 'composition over inheritance'? Give me 5 concrete reasons with code examples showing when inheritance fails and composition succeeds."

---

## Aggregation vs Composition: Understanding Coupling

Both composition and aggregation are "has-a" relationships, but they differ in **coupling**—how tightly objects are bound together.

**Composition: Strong Ownership**

In composition, the container **creates and owns** the contained object. When the container dies, the contained object dies with it.

```python
class Car:
    def __init__(self, make: str, horsepower: int) -> None:
        self.make = make
        self.engine = Engine(horsepower)  # Car CREATES and OWNS engine

    # If Car is destroyed, Engine is destroyed


# The engine's lifetime is tied to the car's
car = Car("Toyota", 200)
# If car is deleted, engine is also deleted (both go away together)
```

**Aggregation: Weak Relationship**

In aggregation, the container **references but doesn't own** the contained object. The contained object can exist independently.

```python
class Department:
    def __init__(self, name: str) -> None:
        self.name = name


class University:
    def __init__(self, name: str) -> None:
        self.name = name
        self.departments: list[Department] = []

    def add_department(self, dept: Department) -> None:
        self.departments.append(dept)  # University REFERENCES, doesn't create


# Department can exist independently
cs_dept = Department("Computer Science")
university = University("Tech University")
university.add_department(cs_dept)

# Even if university closes, cs_dept can continue to exist
# (The relationship is weak)
```

#### 🚀 CoLearning Challenge

> Ask your AI: "I have a Library with Books. Should I use composition or aggregation? Books are created in the library's catalog system but might be shared or archived. What pattern fits?"

---

## Organizing Code into Modules and Packages

As your projects grow beyond single files, you need to organize classes into logical modules. Python's **module system** lets you split code across files and organize files into **packages** (directories with `__init__.py`).

### Module: A File with Classes

Create a file `animals.py` with animal-related classes:

```python
# animals.py
class Dog:
    def __init__(self, name: str) -> None:
        self.name = name

    def speak(self) -> str:
        return f"{self.name} says: Woof!"


class Cat:
    def __init__(self, name: str) -> None:
        self.name = name

    def speak(self) -> str:
        return f"{self.name} says: Meow!"
```

In another file, import and use these classes:

```python
# main.py
from animals import Dog, Cat

dog = Dog("Buddy")
cat = Cat("Whiskers")

print(dog.speak())  # Buddy says: Woof!
print(cat.speak())  # Whiskers says: Meow!
```

### Package: A Directory with `__init__.py`

For larger projects, organize modules into packages:

```
my_project/
├── animals/
│   ├── __init__.py
│   ├── mammals.py
│   └── birds.py
├── vehicles/
│   ├── __init__.py
│   └── cars.py
└── main.py
```

**animals/mammals.py**:
```python
class Dog:
    def speak(self) -> str:
        return "Woof!"


class Cat:
    def speak(self) -> str:
        return "Meow!"
```

**animals/__init__.py** (controls public API):
```python
from .mammals import Dog, Cat
from .birds import Parrot

# __all__ controls what gets imported with "from animals import *"
__all__ = ['Dog', 'Cat', 'Parrot']
```

**main.py** (uses the package):
```python
from animals import Dog, Cat, Parrot

dog = Dog()
cat = Cat()
parrot = Parrot()
```

The `__init__.py` file is critical—it tells Python "this directory is a package" and controls which classes are publicly available through the package name.

#### ✨ Teaching Tip

> Use Claude Code to explore real projects: "Show me how Django organizes its apps, models, views, and templates. How does it use packages and __init__.py to create a scalable architecture?"

---

## Refactoring Inheritance to Composition: A Real Example

Let's see how to recognize when inheritance is wrong and refactor to composition.

**Problematic Inheritance Design:**

```python
# This inheritance hierarchy is rigid
class Employee:
    def __init__(self, name: str, salary: float) -> None:
        self.name = name
        self.salary = salary


class Manager(Employee):
    """Manager IS-AN Employee with extra responsibilities"""
    def __init__(self, name: str, salary: float, team_size: int) -> None:
        super().__init__(name, salary)
        self.team_size = team_size


class Printer(Employee):
    """Wait... Employee IS-A Printer? This makes no sense!"""
    def print_report(self) -> str:
        return f"Printing {self.name}'s report"


# The design is confused. Not all employees print reports.
```

**Refactored with Composition:**

```python
class Employee:
    def __init__(self, name: str, salary: float) -> None:
        self.name = name
        self.salary = salary


class ReportPrinter:
    """A tool for printing reports"""
    def print_report(self, employee: Employee) -> str:
        return f"Printing {employee.name}'s report"


class Manager(Employee):
    """Manager IS-AN Employee with team oversight"""
    def __init__(self, name: str, salary: float, team_size: int) -> None:
        super().__init__(name, salary)
        self.team_size = team_size


# Optional: Manager has a printer tool
manager = Manager("Alice", 100000, 5)
printer = ReportPrinter()
print(printer.print_report(manager))  # Printing Alice's report
```

Now the design makes sense:
- `Manager` IS-AN `Employee` (inheritance for real "is-a" relationships)
- `Manager` HAS-A `Printer` (composition for optional capabilities)

#### 🎓 Instructor Commentary

> The rule: **Inheritance models unchanging identity ("is-a"), composition models changeable capabilities ("has-a")**. An object's type rarely changes, but its capabilities often do.

---

## Real-World Project Structure: Multi-Agent System

Here's how a professional project organizing multiple agent types might look:

```
ai_agent_system/
├── agents/
│   ├── __init__.py
│   ├── base.py              # Abstract Agent base class
│   ├── chat_agent.py        # ChatAgent (HAS-A reasoning engine)
│   └── code_agent.py        # CodeAgent (HAS-A syntax checker)
├── engines/
│   ├── __init__.py
│   ├── reasoning.py         # Reasoning engine (composition)
│   └── syntax_checker.py    # Syntax validation (composition)
├── events/
│   ├── __init__.py
│   └── bus.py              # Event bus for agent communication
└── main.py                  # Orchestration
```

**agents/base.py** (abstract base):
```python
from abc import ABC, abstractmethod


class Agent(ABC):
    """All agents share this interface"""

    def __init__(self, name: str) -> None:
        self.name = name

    @abstractmethod
    def process(self, message: str) -> str:
        pass
```

**agents/chat_agent.py** (uses composition):
```python
from agents.base import Agent
from engines.reasoning import ReasoningEngine


class ChatAgent(Agent):
    """ChatAgent HAS-A ReasoningEngine (composition)"""

    def __init__(self, name: str, reasoning_engine: ReasoningEngine) -> None:
        super().__init__(name)
        self.reasoning_engine = reasoning_engine  # Composition!

    def process(self, message: str) -> str:
        reasoning = self.reasoning_engine.reason(message)
        return f"{self.name}: {reasoning}"
```

**agents/__init__.py** (public API):
```python
from .base import Agent
from .chat_agent import ChatAgent
from .code_agent import CodeAgent

__all__ = ['Agent', 'ChatAgent', 'CodeAgent']
```

**main.py** (orchestration):
```python
from agents import ChatAgent, CodeAgent
from engines.reasoning import ReasoningEngine
from engines.syntax_checker import SyntaxChecker

# Create engines (shared across agents or individual)
reasoning = ReasoningEngine()
syntax = SyntaxChecker()

# Create agents with their tools (composition)
chat = ChatAgent("ChatBot", reasoning)
code = CodeAgent("CodeHelper", syntax)

# Use agents
print(chat.process("Explain Python"))
print(code.process("Check this function"))
```

This design is flexible: agents are composed from interchangeable engines. You can swap engines, test with mock engines, and add new agent types without modifying existing code.

---

## Key Design Principles: When to Use Each Pattern

**Use Inheritance for:**
- Permanent, fundamental "is-a" relationships
- Examples: `Dog` is-an `Animal`, `Circle` is-a `Shape`, `Manager` is-an `Employee`
- The relationship doesn't change

**Use Composition for:**
- Changeable capabilities and relationships
- Examples: `Car` has-an `Engine`, `Agent` has-a `ReasoningEngine`, `Employee` has-a `Printer`
- The relationship can be modified, swapped, or removed

**The Liskov Substitution Principle Test:**
- If a subclass can't reliably replace the parent (like `Penguin` can't replace `Bird`), use composition instead

#### 💬 AI Colearning Prompt

> "Ask your AI: Design a game with Players, Weapons, and Armor. Should Player inherit from Weapon? Should Player have-a Weapon? Show me the composition design and explain why it's more flexible than inheritance."

---

## Try With AI

**Prompt 1: Recall - Composition Syntax**

Create a `Computer` class that has-a `Processor`, `Memory`, and `Storage` (each as separate classes). Show composition, not inheritance. Include type hints.

**Prompt 2: Understand - When to Choose Composition**

I'm designing a Zoo system where animals can be organized by habitat. Should I use inheritance (`class Penguin(ArcticHabitat)`) or composition? Explain the tradeoffs and which you'd choose.

**Prompt 3: Apply - Refactoring to Composition**

Here's problematic code: `class Manager(Employee, Printer)`. The Printer shouldn't be a parent class. Refactor to use composition instead, explaining why this is better.

**Prompt 4: Synthesize - Module Organization**

Design a package structure for an e-commerce system with products, orders, payments, and users. Create the directory structure, __init__.py files, and show imports. Explain your organizational decisions.

