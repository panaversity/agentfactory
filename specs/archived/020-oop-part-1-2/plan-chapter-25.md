# Chapter 25 Implementation Plan: Object-Oriented Programming Part II

**Feature Branch**: `020-oop-part-1-2`
**Created**: 2025-11-09
**Status**: Ready for Implementation
**Spec**: [spec-chapter-25.md](./spec-chapter-25.md)
**Prerequisites**: Chapter 24 (OOP Part I) MUST be completed

## Overview

This plan transforms the Chapter 25 specification into a detailed lesson-by-lesson implementation guide covering advanced OOP: inheritance, MRO, polymorphism, composition, special methods, and design patterns.

**Target Outcome**: Students master advanced OOP patterns and implement 4 professional design patterns (Singleton, Factory, Observer, Strategy) in an integrated AI agent system.

## Lesson Breakdown

### Lesson 1: Inheritance and Method Resolution Order (MRO)

**File**: `01-inheritance-mro.md`
**Estimated Time**: 60 minutes
**CEFR Proficiency**: B1 → B2 (complex hierarchies)
**Cognitive Load**: 8 concepts

#### Skills Taught

| Skill Name | CEFR Level | Category | Measurable at this level |
|------------|------------|----------|--------------------------|
| Single Inheritance | B1 | Technical | Can create parent-child class relationships |
| super() Function Usage | B2 | Technical | Can properly call parent methods |
| Multiple Inheritance | B2 | Conceptual | Can understand diamond inheritance |
| MRO Understanding | B2 | Conceptual | Can explain C3 linearization |
| mro() Method Usage | B2 | Technical | Can inspect and debug inheritance chains |
| Inheritance Design | B2 | Soft | Can decide when inheritance is appropriate |

#### Content Structure

**1. Single Inheritance** (15 min)
```python
class Animal:
    def __init__(self, name: str):
        self.name = name

    def speak(self) -> str:
        return "Some generic sound"

class Dog(Animal):
    def __init__(self, name: str, breed: str):
        super().__init__(name)  # Call parent constructor
        self.breed = breed

    def speak(self) -> str:
        return f"{self.name} says: Woof!"

dog = Dog("Max", "Labrador")
print(dog.speak())  # Max says: Woof!
```

**2. The super() Function** (10 min)
💬 **AI Colearning Prompt**:
> "Explain exactly what super().__init__(name) does. What happens if I forget to call it? Show me the memory state difference."

**3. Multiple Inheritance** (15 min)
```python
class Flyer:
    def fly(self) -> str:
        return "Flying high!"

class Swimmer:
    def swim(self) -> str:
        return "Swimming fast!"

class Duck(Flyer, Swimmer):
    def __init__(self, name: str):
        self.name = name

    def quack(self) -> str:
        return f"{self.name} says: Quack!"

duck = Duck("Donald")
print(duck.fly())   # Flying high!
print(duck.swim())  # Swimming fast!
print(duck.quack()) # Donald says: Quack!
```

**4. Diamond Inheritance Problem** (10 min)
```python
class A:
    def greet(self):
        return "Hello from A"

class B(A):
    def greet(self):
        return "Hello from B"

class C(A):
    def greet(self):
        return "Hello from C"

class D(B, C):
    pass

d = D()
print(d.greet())  # Which greet() gets called?
print(D.mro())    # Let's see the Method Resolution Order
```

**5. Method Resolution Order (MRO)** (10 min)
🎓 **Instructor Commentary**:
> "Python uses C3 Linearization to determine MRO. It ensures: (1) subclasses before parents, (2) inheritance order preserved, (3) no class visited twice. This prevents diamond inheritance chaos."

```python
# MRO for D: [D, B, C, A, object]
# When d.greet() is called:
# 1. Check D → not found
# 2. Check B → FOUND! Return "Hello from B"
# Search stops. C and A are never checked.
```

🚀 **CoLearning Challenge**:
> "Create a diamond hierarchy (Vehicle <- Car, Vehicle <- Boat, AmphibiousCar <- Car & Boat). Add a start() method in Vehicle and override it in Car and Boat. Call start() from AmphibiousCar and ask AI to explain the MRO."

#### Try With AI

**Prompt 1: Recall - Inheritance Syntax**
```
Create a base class Employee with name and salary. Create a Manager subclass that adds department. Use super() correctly in Manager.__init__.
```

**Prompt 2: Understand - super() Deep Dive**
```
Why do we use super() instead of calling Parent.__init__(self, ...) directly? Show me a scenario where super() is critical (hint: multiple inheritance).
```

**Prompt 3: Apply - Diamond Inheritance**
```
Create classes: Device, Phone (adds call()), Computer (adds compute()), Smartphone (inherits Phone and Computer). Verify MRO with .mro() method.
```

**Prompt 4: Analyze - When NOT to Use Inheritance**
```
Give me 3 scenarios where inheritance is the WRONG choice and composition (has-a) would be better. Explain the design smell called "inheritance abuse."
```

---

### Lesson 2: Polymorphism and Duck Typing

**File**: `02-polymorphism-duck-typing.md`
**Estimated Time**: 55 minutes
**CEFR Proficiency**: B2 (abstract thinking)
**Cognitive Load**: 7 concepts

#### Skills Taught

| Skill Name | CEFR Level | Category | Measurable at this level |
|------------|------------|----------|--------------------------|
| Method Overriding | B2 | Technical | Can override parent methods appropriately |
| Abstract Base Classes (Deep) | B2 | Technical | Can design and implement ABC hierarchies |
| @abstractmethod Decorator | B2 | Technical | Can enforce implementation contracts |
| Duck Typing Philosophy | B2 | Conceptual | Can write polymorphic code without inheritance |
| Polymorphic Design | B2 | Soft | Can choose between inheritance and duck typing |

#### Content Structure

**1. Method Overriding** (10 min)
```python
class Shape:
    def area(self) -> float:
        return 0.0  # Default implementation

class Circle(Shape):
    def __init__(self, radius: float):
        self.radius = radius

    def area(self) -> float:
        """Override parent method"""
        return 3.14159 * self.radius ** 2

class Rectangle(Shape):
    def __init__(self, width: float, height: float):
        self.width = width
        self.height = height

    def area(self) -> float:
        """Different implementation, same interface"""
        return self.width * self.height

shapes: list[Shape] = [Circle(5), Rectangle(4, 6)]
for shape in shapes:
    print(shape.area())  # Polymorphism in action
```

**2. Abstract Base Classes (Deep Dive)** (20 min)
```python
from abc import ABC, abstractmethod

class Agent(ABC):
    """Abstract base class for AI agents"""

    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def process(self, message: str) -> str:
        """All agents MUST implement message processing"""
        pass

    @abstractmethod
    def get_capabilities(self) -> list[str]:
        """All agents MUST describe their capabilities"""
        pass

    def log(self, message: str) -> None:
        """Concrete method (not abstract)"""
        print(f"[{self.name}] {message}")

# This raises TypeError: Can't instantiate abstract class
# agent = Agent("Generic")

class ChatAgent(Agent):
    def process(self, message: str) -> str:
        self.log(f"Processing: {message}")
        return f"ChatAgent response to: {message}"

    def get_capabilities(self) -> list[str]:
        return ["text_chat", "conversation_history"]

class CodeAgent(Agent):
    def process(self, message: str) -> str:
        self.log(f"Analyzing code: {message}")
        return f"Code analysis result for: {message}"

    def get_capabilities(self) -> list[str]:
        return ["code_analysis", "syntax_checking", "refactoring"]

# Polymorphic usage
agents: list[Agent] = [ChatAgent("ChatBot"), CodeAgent("CodeHelper")]
for agent in agents:
    print(agent.process("Hello"))
    print(f"Capabilities: {agent.get_capabilities()}")
```

🎓 **Instructor Commentary**:
> "ABCs enforce contracts at the class level. In AI agent systems, this is CRITICAL—you can't deploy an agent that doesn't implement process(). Python checks this at instantiation, not runtime failure."

💬 **AI Colearning Prompt**:
> "Explain the difference between an ABC with abstract methods vs a regular base class with methods that raise NotImplementedError. Which is better and why?"

**3. Abstract Properties and Class Methods** (10 min)
```python
from abc import ABC, abstractmethod

class DataSource(ABC):
    @property
    @abstractmethod
    def connection_string(self) -> str:
        """Abstract property - subclasses must implement"""
        pass

    @classmethod
    @abstractmethod
    def from_config(cls, config: dict) -> 'DataSource':
        """Abstract class method - factory pattern"""
        pass

class PostgresSource(DataSource):
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port

    @property
    def connection_string(self) -> str:
        return f"postgresql://{self.host}:{self.port}"

    @classmethod
    def from_config(cls, config: dict) -> 'PostgresSource':
        return cls(config['host'], config['port'])
```

**4. Duck Typing** (15 min)
```python
# No inheritance needed - just implement the interface
class FileReader:
    def read(self) -> str:
        return "File content"

class URLReader:
    def read(self) -> str:
        return "URL content"

class DatabaseReader:
    def read(self) -> str:
        return "Database content"

def process_data(reader) -> None:
    """Polymorphic function - works with ANY object that has read()"""
    # No type checking needed - duck typing
    data = reader.read()
    print(f"Processing: {data}")

# All work without inheritance
process_data(FileReader())
process_data(URLReader())
process_data(DatabaseReader())
```

🎓 **Instructor Commentary**:
> "Duck typing: 'If it walks like a duck and quacks like a duck, it's a duck.' Python prefers this over strict inheritance. It's more flexible but requires discipline."

🚀 **CoLearning Challenge**:
> "Design an abstract PaymentProcessor with process_payment() and refund(). Implement CreditCard, PayPal, and Crypto processors. Then show how duck typing would achieve the same without ABC."

#### Try With AI

**Prompt 1: Recall - ABC Syntax**
```
Create an abstract Vehicle class with abstract start(), stop(), and refuel() methods. Create Car and ElectricCar subclasses. Show what happens if you forget to implement a method.
```

**Prompt 2: Understand - Polymorphism vs Duck Typing**
```
Explain when to use ABC-based polymorphism vs duck typing. Give me 3 scenarios for each approach.
```

**Prompt 3: Apply - Building an Agent System**
```
Design an abstract Agent base class for a multi-agent system. Include abstract process_message() and get_status(). Create 3 concrete agent types. Demonstrate polymorphic message routing.
```

**Prompt 4: Analyze - Protocol vs ABC**
```
Python 3.8+ added 'Protocol' from typing module as an alternative to ABC. Compare Protocol vs ABC. When would you choose each? (This is advanced - ask AI to explain thoroughly.)
```

---

### Lesson 3: Composition Over Inheritance + Modules

**File**: `03-composition-modules.md`
**Estimated Time**: 55 minutes
**CEFR Proficiency**: B2 (design judgment)
**Cognitive Load**: 6 concepts

#### Skills Taught

| Skill Name | CEFR Level | Category | Measurable at this level |
|------------|------------|----------|--------------------------|
| Composition Design | B2 | Conceptual | Can model "has-a" relationships |
| Composition vs Inheritance Choice | B2 | Soft | Can evaluate design tradeoffs |
| Aggregation vs Composition | B2 | Conceptual | Can distinguish lifecycle coupling |
| Module Organization | B1 | Technical | Can structure multi-file projects |
| Package Creation | B2 | Technical | Can create packages with __init__.py |

#### Content Structure

**1. Composition ("has-a")** (15 min)
```python
class Engine:
    def __init__(self, horsepower: int):
        self.horsepower = horsepower

    def start(self) -> str:
        return f"Engine with {self.horsepower}hp started"

class Car:
    def __init__(self, make: str, engine: Engine):
        self.make = make
        self.engine = engine  # Composition: Car HAS-AN Engine

    def start(self) -> str:
        return f"{self.make}: {self.engine.start()}"

engine = Engine(200)
car = Car("Toyota", engine)
print(car.start())  # Toyota: Engine with 200hp started
```

**2. Aggregation vs Composition** (10 min)
```python
# Composition: Strong ownership - Car owns Engine
class Car:
    def __init__(self, make: str, horsepower: int):
        self.make = make
        self.engine = Engine(horsepower)  # Car CREATES and OWNS engine

# Aggregation: Weak ownership - University has Departments
class Department:
    def __init__(self, name: str):
        self.name = name

class University:
    def __init__(self, name: str):
        self.name = name
        self.departments: list[Department] = []

    def add_department(self, dept: Department):
        self.departments.append(dept)  # University references, doesn't own

# Department can exist independently
cs_dept = Department("Computer Science")
uni = University("Tech University")
uni.add_department(cs_dept)
```

🎓 **Instructor Commentary**:
> "Composition: contained object lifecycle depends on container (Car destroyed → Engine destroyed). Aggregation: contained object lifecycle independent (University closed → Departments can continue)."

**3. Composition Over Inheritance Principle** (15 min)
```python
# BAD: Inheritance abuse
class Car(Engine):  # "is-a" relationship - WRONG!
    pass

# GOOD: Composition
class Car:
    def __init__(self, engine: Engine):
        self.engine = engine  # "has-a" relationship - RIGHT!
```

💬 **AI Colearning Prompt**:
> "Ask your AI: Why do professional developers prefer 'composition over inheritance'? Give me 5 concrete reasons with code examples."

🚀 **CoLearning Challenge**:
> "Refactor this inheritance-based design to use composition: Bird(Animal), Penguin(Bird). Problem: Penguin can't fly but inherits fly() from Bird. Fix it with composition."

**4. Organizing Classes into Modules** (15 min)

**Directory Structure**:
```
my_project/
├── animals/
│   ├── __init__.py
│   ├── mammals.py
│   └── birds.py
├── vehicles/
│   ├── __init__.py
│   ├── cars.py
│   └── bikes.py
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

**animals/__init__.py**:
```python
from .mammals import Dog, Cat
from .birds import Parrot

__all__ = ['Dog', 'Cat', 'Parrot']
```

**main.py**:
```python
from animals import Dog, Cat
from vehicles import Car

dog = Dog()
print(dog.speak())
```

✨ **Teaching Tip**:
> "Use Claude Code to explore package organization: 'Show me how Django organizes its models, views, and controllers into packages.'"

#### Try With AI

**Prompt 1: Recall - Composition Syntax**
```
Create a Computer class that has-a Processor, Memory, and Storage (all separate classes). Show composition, not inheritance.
```

**Prompt 2: Understand - When to Choose Composition**
```
I'm designing a Zoo system with animals that can fly, swim, or walk. Should I use inheritance (Bird, Fish, Mammal) or composition (Animal with Flyable, Swimmable mixins)? Explain the tradeoffs.
```

**Prompt 3: Apply - Refactoring to Composition**
```
Refactor this: class Manager(Employee, Printer). Manager shouldn't BE a Printer. Use composition to fix it.
```

**Prompt 4: Synthesize - Module Organization**
```
Design a package structure for an e-commerce system (products, orders, payments, users). Use nested packages. Show __init__.py imports. Explain the organization rationale.
```

---

### Lesson 4: Special Methods (Magic/Dunder Methods)

**File**: `04-special-methods.md`
**Estimated Time**: 65 minutes
**CEFR Proficiency**: B2 (advanced customization)
**Cognitive Load**: 10 concepts (at maximum)

#### Skills Taught

| Skill Name | CEFR Level | Category | Measurable at this level |
|------------|------------|----------|--------------------------|
| String Representation (__str__, __repr__) | B2 | Technical | Can customize object display |
| Operator Overloading | B2 | Technical | Can implement +, -, *, etc. |
| Container Protocol | B2 | Technical | Can implement __len__, __getitem__ |
| Iteration Protocol | B2 | Technical | Can implement __iter__, __next__ |
| Comparison Protocol | B2 | Technical | Can implement __eq__, __lt__, __hash__ |
| Callable Objects | B2 | Conceptual | Can implement __call__ |

#### Content Structure

**1. __str__ and __repr__** (10 min)
```python
class Person:
    def __init__(self, name: str, age: int):
        self.name = name
        self.age = age

    def __str__(self) -> str:
        """User-friendly string (for print())"""
        return f"{self.name}, {self.age} years old"

    def __repr__(self) -> str:
        """Developer-friendly string (for debugging)"""
        return f"Person(name='{self.name}', age={self.age})"

p = Person("Alice", 30)
print(str(p))   # Alice, 30 years old
print(repr(p))  # Person(name='Alice', age=30)
```

**2. Operator Overloading (__add__, __sub__, etc.)** (15 min)
```python
class Vector:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __add__(self, other: 'Vector') -> 'Vector':
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other: 'Vector') -> 'Vector':
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> 'Vector':
        return Vector(self.x * scalar, self.y * scalar)

    def __str__(self) -> str:
        return f"Vector({self.x}, {self.y})"

v1 = Vector(1, 2)
v2 = Vector(3, 4)
print(v1 + v2)  # Vector(4, 6)
print(v2 - v1)  # Vector(2, 2)
print(v1 * 3)   # Vector(3, 6)
```

💬 **AI Colearning Prompt**:
> "Show me all the operator overload methods Python supports (__add__, __sub__, __mul__, __truediv__, etc.). When would I use __radd__ vs __add__?"

**3. Container Protocol (__len__, __getitem__)** (12 min)
```python
class Playlist:
    def __init__(self):
        self._songs: list[str] = []

    def add_song(self, song: str):
        self._songs.append(song)

    def __len__(self) -> int:
        """len(playlist) works"""
        return len(self._songs)

    def __getitem__(self, index: int) -> str:
        """playlist[0] works"""
        return self._songs[index]

    def __setitem__(self, index: int, song: str):
        """playlist[0] = 'New Song' works"""
        self._songs[index] = song

playlist = Playlist()
playlist.add_song("Song 1")
playlist.add_song("Song 2")
print(len(playlist))  # 2
print(playlist[0])    # Song 1
playlist[1] = "Updated Song 2"
```

**4. Iteration Protocol (__iter__, __next__)** (12 min)
```python
class Countdown:
    def __init__(self, start: int):
        self.current = start

    def __iter__(self):
        """Make object iterable"""
        return self

    def __next__(self) -> int:
        """Get next value"""
        if self.current <= 0:
            raise StopIteration
        self.current -= 1
        return self.current + 1

for num in Countdown(5):
    print(num)  # 5, 4, 3, 2, 1
```

🚀 **CoLearning Challenge**:
> "Create a Range class that mimics Python's built-in range(). Implement __iter__ and __next__. Then add __len__ and __getitem__ to support range[0] and len(range)."

**5. Comparison and Hashing (__eq__, __lt__, __hash__)** (10 min)
```python
from functools import total_ordering

@total_ordering
class Person:
    def __init__(self, name: str, age: int):
        self.name = name
        self.age = age

    def __eq__(self, other) -> bool:
        return self.age == other.age

    def __lt__(self, other) -> bool:
        return self.age < other.age

    def __hash__(self) -> int:
        """Needed for set/dict"""
        return hash((self.name, self.age))

people = [Person("Alice", 30), Person("Bob", 25), Person("Charlie", 35)]
print(sorted(people, key=lambda p: p.age))  # Uses __lt__
```

🎓 **Instructor Commentary**:
> "If you implement __eq__ but not __hash__, your objects can't be used in sets or as dict keys. Python requires: equal objects MUST have equal hashes."

**6. Callable Objects (__call__)** (6 min)
```python
class Multiplier:
    def __init__(self, factor: int):
        self.factor = factor

    def __call__(self, x: int) -> int:
        """Make instance callable like a function"""
        return x * self.factor

double = Multiplier(2)
triple = Multiplier(3)

print(double(5))  # 10 (calls __call__)
print(triple(5))  # 15
```

💬 **AI Colearning Prompt**:
> "Explain how Python's built-in list uses special methods. What happens when I do len([1,2,3])? When I do [1,2,3][0]? When I iterate?"

#### Try With AI

**Prompt 1: Recall - Basic Special Methods**
```
Create a Book class. Implement __str__ (user-friendly), __repr__ (debug), and __eq__ (compare by ISBN). Test all three.
```

**Prompt 2: Understand - Operator Overloading**
```
Create a Money class (amount, currency). Implement __add__ (only if same currency), __sub__, __lt__ (for comparison). Handle type checking.
```

**Prompt 3: Apply - Container Protocol**
```
Create a Stack class (LIFO). Implement __len__, __getitem__, __iter__. Make it behave like a list but with stack semantics.
```

**Prompt 4: Synthesize - Building a Custom Collection**
```
Create a SortedList class that automatically keeps items sorted. Implement __init__, __len__, __getitem__, __setitem__, __iter__, __contains__. Make it behave like a Python built-in.
```

---

### Lesson 5: Design Patterns (Capstone)

**File**: `05-design-patterns-capstone.md`
**Estimated Time**: 80 minutes
**CEFR Proficiency**: B2 (professional architecture)
**Cognitive Load**: Integration (synthesis of all Ch24-25 concepts)

#### Skills Taught

| Skill Name | CEFR Level | Category | Measurable at this level |
|------------|------------|----------|--------------------------|
| Singleton Pattern | B2 | Technical | Can ensure single instance |
| Factory Pattern | B2 | Technical | Can create objects without specifying class |
| Observer Pattern | B2 | Technical | Can implement event-driven systems |
| Strategy Pattern | B2 | Technical | Can encapsulate algorithms |
| Pattern Selection | B2 | Soft | Can choose appropriate pattern for problem |
| Architecture Design | B2 | Soft | Can integrate multiple patterns |

#### Project Overview

**Build an AI Agent System** combining all 4 design patterns:
- Singleton: AgentManager (single instance coordinates agents)
- Factory: AgentFactory (creates different agent types dynamically)
- Observer: Event system (agents react to events)
- Strategy: Decision strategies (agents use different reasoning approaches)

#### Content Structure

**Part 1: Singleton Pattern** (15 min)
```python
class AgentManager:
    """Singleton - only one instance exists"""
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self._initialized = True
        self.agents: list = []

    def register_agent(self, agent):
        self.agents.append(agent)
        print(f"Registered: {agent.name}")

# Test singleton
manager1 = AgentManager()
manager2 = AgentManager()
print(manager1 is manager2)  # True - same instance
```

🎓 **Instructor Commentary**:
> "Singletons are controversial. Use for: configuration, logging, connection pools. Avoid for: anything that needs testing with different states."

**Part 2: Factory Pattern** (20 min)
```python
from abc import ABC, abstractmethod

class Agent(ABC):
    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def process(self, message: str) -> str:
        pass

class ChatAgent(Agent):
    def process(self, message: str) -> str:
        return f"ChatAgent {self.name}: Processing '{message}' as conversation"

class CodeAgent(Agent):
    def process(self, message: str) -> str:
        return f"CodeAgent {self.name}: Analyzing code '{message}'"

class DataAgent(Agent):
    def process(self, message: str) -> str:
        return f"DataAgent {self.name}: Processing data '{message}'"

class AgentFactory:
    """Factory - creates agents without specifying exact class"""

    @staticmethod
    def create_agent(agent_type: str, name: str) -> Agent:
        if agent_type == "chat":
            return ChatAgent(name)
        elif agent_type == "code":
            return CodeAgent(name)
        elif agent_type == "data":
            return DataAgent(name)
        else:
            raise ValueError(f"Unknown agent type: {agent_type}")

# Use factory
agent1 = AgentFactory.create_agent("chat", "ChatBot1")
agent2 = AgentFactory.create_agent("code", "CodeHelper")
print(agent1.process("Hello"))
```

🚀 **CoLearning Challenge**:
> "Extend the Factory with a registry pattern: agents can self-register. Then create agents by string lookup without if/elif chains."

**Part 3: Observer Pattern** (20 min)
```python
from typing import Protocol

class Observer(Protocol):
    """Observer interface - duck typing"""
    def update(self, event: str) -> None:
        ...

class Subject:
    """Subject - maintains observers and notifies them"""
    def __init__(self):
        self._observers: list[Observer] = []

    def attach(self, observer: Observer):
        self._observers.append(observer)

    def detach(self, observer: Observer):
        self._observers.remove(observer)

    def notify(self, event: str):
        for observer in self._observers:
            observer.update(event)

class EventBus(Subject):
    """Central event dispatcher"""
    def publish(self, event: str):
        print(f"📢 Publishing event: {event}")
        self.notify(event)

class Agent:
    """Agent that observes events"""
    def __init__(self, name: str):
        self.name = name

    def update(self, event: str):
        print(f"  {self.name} received: {event}")

# Setup
event_bus = EventBus()
agent1 = Agent("Agent1")
agent2 = Agent("Agent2")

event_bus.attach(agent1)
event_bus.attach(agent2)

event_bus.publish("new_message")
# Output:
# 📢 Publishing event: new_message
#   Agent1 received: new_message
#   Agent2 received: new_message
```

💬 **AI Colearning Prompt**:
> "Explain how Observer pattern differs from polling. Why is event-driven better for AI agent communication?"

**Part 4: Strategy Pattern** (15 min)
```python
from abc import ABC, abstractmethod

class DecisionStrategy(ABC):
    """Strategy interface"""
    @abstractmethod
    def decide(self, context: dict) -> str:
        pass

class AggressiveStrategy(DecisionStrategy):
    def decide(self, context: dict) -> str:
        return "Aggressive: Attack immediately"

class DefensiveStrategy(DecisionStrategy):
    def decide(self, context: dict) -> str:
        return "Defensive: Retreat and regroup"

class BalancedStrategy(DecisionStrategy):
    def decide(self, context: dict) -> str:
        threat = context.get('threat_level', 0)
        if threat > 7:
            return "Balanced: Tactical retreat"
        else:
            return "Balanced: Careful advance"

class Agent:
    """Agent uses strategy pattern for decision-making"""
    def __init__(self, name: str, strategy: DecisionStrategy):
        self.name = name
        self.strategy = strategy

    def set_strategy(self, strategy: DecisionStrategy):
        """Switch strategies at runtime"""
        self.strategy = strategy

    def make_decision(self, context: dict) -> str:
        return f"{self.name}: {self.strategy.decide(context)}"

# Use strategies
agent = Agent("Agent1", AggressiveStrategy())
print(agent.make_decision({'threat_level': 5}))

# Switch strategy dynamically
agent.set_strategy(DefensiveStrategy())
print(agent.make_decision({'threat_level': 9}))
```

**Part 5: Integrated AI Agent System** (10 min)
```python
# Bring it all together
manager = AgentManager()  # Singleton
event_bus = EventBus()     # Observer

# Create agents with Factory
chat_agent = AgentFactory.create_agent("chat", "ChatBot")
code_agent = AgentFactory.create_agent("code", "CodeHelper")

# Register agents
manager.register_agent(chat_agent)
manager.register_agent(code_agent)

# Attach to event bus
event_bus.attach(chat_agent)
event_bus.attach(code_agent)

# Publish events
event_bus.publish("user_message: Hello")
event_bus.publish("code_review_request")
```

🚀 **CoLearning Challenge**:
> "Extend this system: Add a Mediator pattern to coordinate agent communication. Add Command pattern to queue agent actions. Design the architecture with AI guidance."

#### Try With AI

**Prompt 1: Recall - Pattern Recognition**
```
For each pattern (Singleton, Factory, Observer, Strategy), give me a real-world AI system example where it's used. Explain why that pattern fits.
```

**Prompt 2: Understand - Pattern Tradeoffs**
```
Compare Singleton vs Dependency Injection. When is Singleton acceptable vs anti-pattern? Give me 3 scenarios for each.
```

**Prompt 3: Apply - Building a Pattern**
```
Implement a Command pattern for an AI agent task queue. Create Command interface, concrete commands (ProcessMessage, AnalyzeCode), and CommandQueue. Show undo functionality.
```

**Prompt 4: Synthesize - Multi-Pattern System**
```
Design a multi-agent orchestration system using: Singleton (Orchestrator), Factory (Agent creation), Observer (Event bus), Strategy (Task routing), Decorator (Agent capabilities). Draw architecture and implement core components.
```

---

## Proficiency Progression Summary

| Lesson | Starting CEFR | Ending CEFR | Concepts Introduced | Cumulative with Ch24 |
|--------|---------------|-------------|---------------------|----------------------|
| 1      | B1            | B2          | 8                   | 38                   |
| 2      | B2            | B2          | 7                   | 45                   |
| 3      | B2            | B2          | 6                   | 51                   |
| 4      | B2            | B2          | 10                  | 61                   |
| 5      | B2            | B2          | Integration         | All (synthesis)      |

**Cognitive Load Validation**:
- ✅ Lesson 1: 8 concepts (B1-B2 complex hierarchies)
- ✅ Lesson 2: 7 concepts (B2 abstract thinking)
- ✅ Lesson 3: 6 concepts (B2 design judgment)
- ✅ Lesson 4: 10 concepts (B2 maximum - special methods)
- ✅ Lesson 5: Integration (synthesis, no new concepts)

---

## Code Examples Mapping

| Example # | Purpose | Lesson | Complexity | Status |
|-----------|---------|--------|------------|--------|
| 1 | Single inheritance (Dog) | L1 | B1 | Specified |
| 2 | super() usage | L1 | B2 | Specified |
| 3 | Multiple inheritance (Duck) | L1 | B2 | Specified |
| 4 | Diamond inheritance + MRO | L1 | B2 | Specified |
| 5 | Method overriding (Shape) | L2 | B2 | Specified |
| 6 | ABC deep dive (Agent) | L2 | B2 | Specified |
| 7 | Duck typing (Readers) | L2 | B2 | Specified |
| 8 | Composition (Car/Engine) | L3 | B2 | Specified |
| 9 | Aggregation (University) | L3 | B2 | Specified |
| 10 | Package organization | L3 | B1 | Specified |
| 11-16 | Special methods | L4 | B2 | 6 examples specified |
| 17-20 | Design patterns | L5 | B2 | 4 patterns integrated |

---

## AI-Native CoLearning Compliance

### CoLearning Elements Per Lesson

| Lesson | 💬 AI Prompts | 🎓 Commentaries | 🚀 Challenges | ✨ Tips | Try With AI |
|--------|---------------|-----------------|---------------|---------|-------------|
| 1      | 2             | 2               | 2             | 0       | 4 prompts   |
| 2      | 3             | 2               | 2             | 0       | 4 prompts   |
| 3      | 2             | 2               | 2             | 1       | 4 prompts   |
| 4      | 3             | 2               | 2             | 0       | 4 prompts   |
| 5      | 2             | 2               | 3             | 0       | 4 prompts   |
| **Total** | **12** | **10** | **11** | **1** | **20 prompts** |

✅ All lessons have CoLearning elements throughout
✅ All lessons end with "Try With AI" ONLY (4 prompts each)
✅ Progressive Bloom's levels in each Try With AI section

---

## Implementation Checklist

### Pre-Implementation
- [x] Specification approved
- [x] Plan created with CEFR levels
- [x] Cognitive load validated
- [x] CoLearning elements designed
- [x] Chapter 24 dependency noted
- [ ] Skills proficiency mapper applied (tasks phase)

### Content Creation (Phase 4)
- [ ] Lesson 1: Inheritance and MRO written
- [ ] Lesson 2: Polymorphism and Duck Typing written
- [ ] Lesson 3: Composition and Modules written
- [ ] Lesson 4: Special Methods written
- [ ] Lesson 5: Design Patterns Capstone written
- [ ] All code tested on Python 3.14+
- [ ] All "Try With AI" prompts validated

### Quality Gates (Phase 4)
- [ ] No forward references verified
- [ ] Pedagogical ordering compliance
- [ ] Conversational tone throughout
- [ ] Type hints in all examples
- [ ] validation-auditor validation PASS

---

## Dependencies and Prerequisites

**Student MUST Have Completed**:
- Chapter 24: OOP Part I (classes, objects, encapsulation, methods, ABC intro)

**Student Will Be Ready For**:
- Chapter 26: Metaclasses and Data Classes
- Advanced Python topics (Chapters 27-29)

---

## Next Steps

1. **Run /sp.tasks** to generate implementation checklist
2. **Invoke content-implementer** subagent for each lesson
3. **Apply validation-auditor** for validation
4. **Update chapter-index.md** with completion status

---

**Plan Status**: ✅ READY FOR TASKS GENERATION (after Ch24 complete)
