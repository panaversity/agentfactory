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
generated_by: "content-implementer v3.0.0"
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

#### 🎓 Expert Insight

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

#### 🎓 Expert Insight

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

## Challenge: Building Flexible Agent Architectures with Composition

In this challenge, you'll discover why inheritance creates rigid designs, learn how composition provides flexibility, challenge AI with design questions, and build a production agent system.

---

## Part 1: Student Discovers Inheritance Rigidity in Agent Design

**Your Role**: System architect identifying design constraints

### Discovery Exercise: Build an Agent System with Inheritance (the Wrong Way)

Imagine you're building an agent framework where agents need different capabilities. You try to model this with inheritance.

**Stage 1: The Inheritance Hierarchy Problem**

Create a rigid hierarchy of agent classes:

```python
# agents_inheritance.py - PROBLEMATIC
class BaseAgent:
    """Base for all agents"""
    def __init__(self, name: str):
        self.name = name


class LLMAgent(BaseAgent):
    """Agent with LLM capability"""
    def reason(self, question: str) -> str:
        return f"LLM reasoning: {question}"


class DatabaseAgent(BaseAgent):
    """Agent with database capability"""
    def query(self, query: str) -> str:
        return f"Database result: {query}"


class CombinedAgent(LLMAgent, DatabaseAgent):
    """Agent with BOTH capabilities - multiple inheritance!"""
    pass


# Test
agent = CombinedAgent("MultiAgent")
print(agent.reason("What is the answer?"))
print(agent.query("SELECT * FROM users"))
```

**Your task 1**: Copy this code and document in `inheritance_rigidity_analysis.md`:
- How many parent classes does CombinedAgent have?
- If you need an agent with LLM + database + web-search, what inheritance structure would you need?
- What happens with three capabilities? Five?
- What pattern do you notice?

**Stage 2: The Capability Explosion**

**Your task 2**: Try to design an agent that has:
- LLM reasoning
- Database access
- Web search
- File I/O
- Code execution

Document:
- How many inheritance paths would be needed?
- Which capabilities would conflict?
- Why is inheritance a poor fit for "mixing capabilities"?
- What would a better design look like?

### Your Discovery Document

Create `composition_vs_inheritance_problem.md` with:

1. **The Rigidity Problem**: Inheritance locks in capability combinations at compile time
2. **The Explosion Problem**: Each new capability combination requires a new class
3. **The Modification Problem**: Adding a new capability to existing agents is hard
4. **Your Prediction**: What design pattern would allow mixing capabilities dynamically?

---

## Part 2: AI Teaches Composition as the Flexible Solution

**Your Role**: Student learning from AI Teacher

### AI Teaching Prompt

Ask your AI companion:

> "I'm building an agent framework. Agents need different capabilities: some have LLM reasoning, some have database access, some have web search. I tried inheritance (LLMAgent, DatabaseAgent, LLMDatabaseAgent, etc.) but it explodes combinatorially.
>
> Explain:
> 1. What is composition? How is it different from inheritance?
> 2. Design an agent system where agents are composed from capability objects (ReasoningEngine, DatabaseEngine, SearchEngine)
> 3. Show me how to add a new agent with new capabilities WITHOUT creating a new class
> 4. What are the trade-offs between inheritance and composition?"

### Expected AI Response Summary

AI will explain:
- **Composition**: Objects contain other objects; capabilities come from components, not class hierarchy
- **Flexible design**: New agents are created by combining engines, not creating new subclasses
- **Dynamic capabilities**: Add/remove capabilities at runtime by adding/removing components
- **Scalability**: 5 engines can combine into unlimited agent types without creating new classes
- **Trade-off**: Inheritance enforces contracts; composition requires more careful design

**AI will show code like**:

```python
# SOLUTION: Composition
class ReasoningEngine:
    def reason(self, question: str) -> str:
        return f"Reasoning: {question}"

class DatabaseEngine:
    def query(self, query: str) -> str:
        return f"Query result: {query}"

class Agent:
    def __init__(self, name: str, engines: dict):
        self.name = name
        self.engines = engines  # HAS-A engines, not IS-A subclass

    def get_capability(self, capability_name: str):
        return self.engines.get(capability_name)

# Create agents with different capability combinations
reasoning_agent = Agent("Reasoner", {
    "reasoning": ReasoningEngine()
})

combined_agent = Agent("Combined", {
    "reasoning": ReasoningEngine(),
    "database": DatabaseEngine()
})
```

### Convergence Activity

After AI explains, verify understanding:

> "Show me how this composition design makes it easy to add 5 new agents with different capability combinations. Explain why inheritance couldn't handle this as elegantly. What happens if I want agents to share the same engine instance?"

### Deliverable

Write 1-paragraph summary: "How Composition Replaces Inheritance in Agent Design" explaining the core insight about flexibility and scalability.

---

## Part 3: Student Challenges AI with Architecture Edge Cases

**Your Role**: Student testing AI's understanding

### Challenge Design Scenarios

Ask AI to handle these cases:

#### Challenge 1: Shared Engines

> "If multiple agents share the same ReasoningEngine instance, what happens when one agent modifies engine state? How is this different from inheritance where each subclass might override methods? Which is safer?"

**Expected learning**: AI explains state management in composition vs method overriding in inheritance.

#### Challenge 2: Engine Dependency Chain

> "What if ReasoningEngine depends on DatabaseEngine? I have Agent A with both, and Agent B with only DatabaseEngine. Show me how to wire dependencies correctly. What problems could occur?"

**Expected learning**: AI explains dependency injection and how composition manages complex dependencies.

#### Challenge 3: Module Organization

> "I have an e-commerce system: products, orders, payments, shipping. Should some of these be capabilities (composition) and others base classes (inheritance)? Design the module structure with __init__.py files."

**Expected learning**: AI shows how to mix inheritance (for stable hierarchies) with composition (for flexible capabilities) and organize into packages.

### Deliverable

Document your three challenges, AI's responses, and analysis of when composition beats inheritance and how to organize large systems with modules.

---

## Part 4: Build Flexible Agent System with Composition and Modules

**Your Role**: Knowledge synthesizer creating reusable code

### Your Modular Agent System

Create a professional project structure:

```
agent_system/
├── agents/
│   ├── __init__.py
│   └── agent.py          # Flexible Agent class
├── engines/
│   ├── __init__.py
│   ├── reasoning.py      # ReasoningEngine (composition)
│   ├── database.py       # DatabaseEngine (composition)
│   └── search.py         # SearchEngine (composition)
├── config.py             # Agent configurations
└── main.py               # Usage examples
```

**engines/reasoning.py**:
```python
class ReasoningEngine:
    """Provides reasoning capability - composed into agents"""

    def reason(self, question: str) -> str:
        """Process a reasoning question"""
        return f"Reasoning about: {question}"


class DatabaseEngine:
    """Provides database capability - composed into agents"""

    def query(self, sql: str) -> str:
        """Execute database query"""
        return f"Query executed: {sql}"


class SearchEngine:
    """Provides web search capability - composed into agents"""

    def search(self, query: str) -> str:
        """Search the web"""
        return f"Search results for: {query}"
```

**agents/agent.py**:
```python
from typing import Any


class Agent:
    """Flexible agent class composed from capability engines"""

    def __init__(self, name: str, engines: dict[str, Any]) -> None:
        """
        Create an agent with specific engines

        Args:
            name: Agent name
            engines: Dict mapping capability names to engine instances
        """
        self.name = name
        self.engines = engines

    def has_capability(self, capability: str) -> bool:
        """Check if agent has a capability"""
        return capability in self.engines

    def execute_reasoning(self, question: str) -> str:
        """Execute reasoning if available"""
        if self.has_capability("reasoning"):
            return self.engines["reasoning"].reason(question)
        raise AttributeError(f"{self.name} has no reasoning capability")

    def execute_query(self, sql: str) -> str:
        """Execute database query if available"""
        if self.has_capability("database"):
            return self.engines["database"].query(sql)
        raise AttributeError(f"{self.name} has no database capability")

    def execute_search(self, query: str) -> str:
        """Execute web search if available"""
        if self.has_capability("search"):
            return self.engines["search"].search(query)
        raise AttributeError(f"{self.name} has no search capability")

    def get_capabilities(self) -> list[str]:
        """List all capabilities"""
        return list(self.engines.keys())
```

**agents/__init__.py**:
```python
from .agent import Agent

__all__ = ['Agent']
```

**engines/__init__.py**:
```python
from .reasoning import ReasoningEngine
from .database import DatabaseEngine
from .search import SearchEngine

__all__ = ['ReasoningEngine', 'DatabaseEngine', 'SearchEngine']
```

**config.py**:
```python
"""Pre-configured agent types"""
from agents import Agent
from engines import ReasoningEngine, DatabaseEngine, SearchEngine


def create_reasoning_agent(name: str) -> Agent:
    """Agent with reasoning only"""
    return Agent(name, {
        "reasoning": ReasoningEngine()
    })


def create_database_agent(name: str) -> Agent:
    """Agent with database access only"""
    return Agent(name, {
        "database": DatabaseEngine()
    })


def create_full_agent(name: str) -> Agent:
    """Agent with all capabilities"""
    return Agent(name, {
        "reasoning": ReasoningEngine(),
        "database": DatabaseEngine(),
        "search": SearchEngine()
    })
```

**main.py**:
```python
from config import create_reasoning_agent, create_database_agent, create_full_agent

# Create agents with different capability combinations
reasoner = create_reasoning_agent("Thinker")
analyst = create_database_agent("Analyst")
researcher = create_full_agent("Researcher")

# Each agent has only its needed capabilities
print(f"Reasoner capabilities: {reasoner.get_capabilities()}")
print(f"Analyst capabilities: {analyst.get_capabilities()}")
print(f"Researcher capabilities: {researcher.get_capabilities()}")

# Execute capability-specific operations
print(f"\nReasoner: {reasoner.execute_reasoning('What is AI?')}")
print(f"Analyst: {analyst.execute_query('SELECT * FROM data')}")
print(f"Researcher: {researcher.execute_search('latest AI trends')}")

# Adding new agents is trivial - just combine existing engines!
```

**Your task**: Expand this system with:
1. Add 2-3 more engine types (FileEngine, APIEngine, CodeExecutionEngine)
2. Create new agent configurations combining different engines
3. Add a comparison document: `composition_vs_inheritance_guide.md`
4. Create a module organization guide: `project_structure.md`

### Validation Checklist

- ✅ Agents are composed from engines, not inherited
- ✅ New agent types require only new configurations, not new classes
- ✅ Engines are reusable across agents
- ✅ Adding a new engine doesn't require modifying Agent or existing engines
- ✅ Module structure is clear with logical packages
- ✅ __init__.py files define public APIs

### Deliverable

Complete agent system with:
- Modular organization (agents/, engines/, config.py)
- Flexible Agent class using composition
- Multiple engine types demonstrating capability-based design
- Configuration functions for common agent types
- Documentation explaining composition benefits

---

## Summary: Bidirectional Learning in Action

**Part 1 (Student discovers)**: You found that inheritance creates rigid, combinatorial designs that don't scale

**Part 2 (AI teaches)**: AI explained how composition provides flexibility—mix capabilities without creating new classes

**Part 3 (Student teaches)**: You challenged AI with edge cases about state management, dependencies, and module organization

**Part 4 (Knowledge synthesis)**: You built a production agent system with flexible composition and professional module organization

### What You've Built

1. `inheritance_rigidity_analysis.md` — Problem analysis
2. `composition_vs_inheritance_problem.md` — Clear problem statement
3. Challenge documentation — Three architectural edge cases
4. `agent_system/` — Production modular project with composition
5. `composition_vs_inheritance_guide.md` — When to use each pattern
6. `project_structure.md` — Module organization best practices

### Next Steps

Lesson 4 teaches special methods that make composed objects feel like built-in Python types. You'll use this flexible agent system and add Python protocol support.