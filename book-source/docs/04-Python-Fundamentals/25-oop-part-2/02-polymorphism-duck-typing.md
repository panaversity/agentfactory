---
title: "Polymorphism and Duck Typing"
chapter: 25
lesson: 2
duration_minutes: 55

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Method Overriding"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can override parent methods appropriately and implement specialized behavior in subclasses"

  - name: "Abstract Base Classes (Deep Dive)"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can design and implement ABC hierarchies to enforce implementation contracts"

  - name: "@abstractmethod Decorator"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can use @abstractmethod to create enforced method signatures"

  - name: "Duck Typing Philosophy"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain duck typing and recognize when to apply it over inheritance"

  - name: "Polymorphic Design"
    proficiency_level: "B2"
    category: "Soft"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can choose between inheritance-based and duck-typing approaches based on design requirements"

  - name: "Protocol-Based Programming"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can define and use Python protocols for structural subtyping"

  - name: "Type Checking vs Duck Typing Trade-offs"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can evaluate design tradeoffs between static typing and duck typing"

learning_objectives:
  - objective: "Implement method overriding to create polymorphic behavior in inheritance hierarchies"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Code exercises creating specialized subclass methods"

  - objective: "Design abstract base classes with @abstractmethod to enforce implementation contracts"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Creating ABC hierarchies for real-world problems"

  - objective: "Apply duck typing principles to write flexible, type-agnostic code"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Refactoring inheritance-based code to duck typing"

  - objective: "Analyze design tradeoffs between ABC-based polymorphism and duck typing"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Explaining when each approach is appropriate"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (method overriding, ABC, @abstractmethod, duck typing, protocols, type checking, trade-offs) within B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore typing.Protocol in depth; implement structural subtyping; compare Protocol vs ABC performance characteristics"
  remedial_for_struggling: "Focus on duck typing examples first (simpler conceptually); build to ABC as enforcement mechanism; use shape example as primary case study"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/020-oop-part-1-2/spec-chapter-25.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 2: Polymorphism and Duck Typing

Polymorphism is the power of object-oriented programming. After mastering inheritance in Lesson 1, you now face a critical design question: **Should objects be related through inheritance, or should they simply share a common interface?** This lesson teaches you both paths—and when to use each one.

In professional AI systems, polymorphism enables multi-agent architectures where different agent types (ChatAgent, CodeAgent, DataAgent) respond to the same `process()` call with specialized behavior. Understanding polymorphism and duck typing is essential for building flexible, maintainable systems.

**What you'll learn**: Method overriding, Abstract Base Classes (deep dive), duck typing philosophy, and the critical design trade-offs between enforcing contracts through ABC versus relying on shared behavior through duck typing.

---

## Method Overriding: Replacing Parent Behavior

Method overriding is the foundation of polymorphism. A subclass provides its own implementation of a parent method, replacing the parent's version entirely. The key insight: **the same method call produces different behavior depending on the object's actual type.**

Let's start with shapes:

```python
class Shape:
    """Base class for all shapes"""

    def area(self) -> float:
        """Default implementation returns 0"""
        return 0.0

    def describe(self) -> str:
        """Generic description"""
        return "I am a shape"


class Circle(Shape):
    """Circle overrides area() and describe()"""

    def __init__(self, radius: float):
        self.radius = radius

    def area(self) -> float:
        """Circle-specific area calculation"""
        import math
        return math.pi * self.radius ** 2

    def describe(self) -> str:
        """Circle-specific description"""
        return f"Circle with radius {self.radius}"


class Rectangle(Shape):
    """Rectangle with different area calculation"""

    def __init__(self, width: float, height: float):
        self.width = width
        self.height = height

    def area(self) -> float:
        """Rectangle-specific area calculation"""
        return self.width * self.height

    def describe(self) -> str:
        """Rectangle-specific description"""
        return f"Rectangle {self.width}×{self.height}"


# Polymorphism in action
shapes: list[Shape] = [Circle(5), Rectangle(4, 6)]

for shape in shapes:
    print(f"{shape.describe()}: {shape.area():.2f} square units")
# Output:
# Circle with radius 5: 78.54 square units
# Rectangle 4×6: 24.00 square units
```

**Key insight**: Notice that `shapes` is a list of `Shape` objects, but each `shape.area()` call invokes the **correct implementation** based on the actual object type. Python looks up the method in the object's class first, then walks up the inheritance tree if needed. This is **polymorphism**—the same interface (`area()`), different implementations.

#### 💬 AI Colearning Prompt

> "Explain how Python determines which `area()` method to call when we iterate through the shapes list. What's the underlying mechanism that makes polymorphism work?"

This question pushes you from "it works" to "I understand why it works"—method resolution order determines which version of a method gets called.

---

## Abstract Base Classes: Enforcing Contracts

Now we reach a critical decision point. The `Shape` class above has a default `area()` that returns 0.0—but that's misleading. A Shape **should** have an area, and subclasses **must** implement it. Python's Abstract Base Classes (ABC) enforce this at the class level.

With Abstract Base Classes, you can declare that certain methods **must** be implemented by subclasses. If a subclass forgets to implement an abstract method, Python raises a `TypeError` at instantiation—catching the error early, not at runtime.

```python
from abc import ABC, abstractmethod


class Agent(ABC):
    """Abstract base class for AI agents - defines the contract all agents must fulfill"""

    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def process(self, message: str) -> str:
        """All agents MUST implement message processing

        Args:
            message: Input message to process

        Returns:
            Response from the agent
        """
        pass  # Abstract methods have no implementation

    @abstractmethod
    def get_capabilities(self) -> list[str]:
        """All agents MUST describe what they can do

        Returns:
            List of capability strings
        """
        pass

    def log(self, message: str) -> None:
        """Concrete method: all agents inherit this implementation"""
        print(f"[{self.name}] {message}")


# This raises TypeError: Can't instantiate abstract class Agent
# agent = Agent("Generic")  # ❌ ERROR


class ChatAgent(Agent):
    """Concrete implementation: ChatAgent"""

    def process(self, message: str) -> str:
        """ChatAgent processes natural language"""
        self.log(f"Processing: {message}")
        return f"ChatAgent response to: {message}"

    def get_capabilities(self) -> list[str]:
        """ChatAgent capabilities"""
        return ["text_chat", "conversation_history", "context_memory"]


class CodeAgent(Agent):
    """Concrete implementation: CodeAgent"""

    def process(self, message: str) -> str:
        """CodeAgent analyzes code"""
        self.log(f"Analyzing code: {message}")
        return f"Code analysis result for: {message}"

    def get_capabilities(self) -> list[str]:
        """CodeAgent capabilities"""
        return ["code_analysis", "syntax_checking", "refactoring"]


# Now we can instantiate concrete subclasses
chat_agent = ChatAgent("ChatBot")
code_agent = CodeAgent("CodeHelper")

# Polymorphic usage: same interface, different implementations
agents: list[Agent] = [chat_agent, code_agent]

for agent in agents:
    print(f"\n{agent.name}:")
    print(f"  Capabilities: {agent.get_capabilities()}")
    print(f"  Processing: {agent.process('Hello')}")

# Output:
# ChatBot:
#   Capabilities: ['text_chat', 'conversation_history', 'context_memory']
#   Processing: ChatAgent response to: Hello
#
# CodeHelper:
#   Capabilities: ['code_analysis', 'syntax_checking', 'refactoring']
#   Processing: Code analysis result for: Hello
```

#### 🎓 Instructor Commentary

> In AI-native development, ABCs are **critical**. You cannot deploy an agent that doesn't implement `process()`. Python checks this at instantiation (when you create the object), not at runtime (when you call the method). This shifts errors from "oh no, the production system crashed" to "oops, my agent class isn't complete yet."

#### 💬 AI Colearning Prompt

> "Explain the difference between an ABC with `@abstractmethod` versus a regular base class with methods that `raise NotImplementedError()`. Which approach is better and why?"

The answer teaches you about **explicit contract enforcement** versus **runtime error handling**—a fundamental design difference.

---

## Abstract Properties and Class Methods

Abstract Base Classes aren't limited to regular methods. You can also make properties and class methods abstract, forcing subclasses to define specific attributes or factory methods.

```python
from abc import ABC, abstractmethod


class DataSource(ABC):
    """Abstract data source - defines what any data source must provide"""

    @property
    @abstractmethod
    def connection_string(self) -> str:
        """Every data source must provide a connection string

        Returns:
            Connection string specific to this data source type
        """
        pass

    @classmethod
    @abstractmethod
    def from_config(cls, config: dict) -> 'DataSource':
        """Factory method - create instance from configuration

        Args:
            config: Configuration dictionary

        Returns:
            Instance of the data source
        """
        pass

    @abstractmethod
    def query(self, sql: str) -> list[dict]:
        """Execute a query

        Args:
            sql: SQL query string

        Returns:
            List of result rows as dictionaries
        """
        pass


class PostgresSource(DataSource):
    """PostgreSQL implementation"""

    def __init__(self, host: str, port: int, database: str):
        self.host = host
        self.port = port
        self.database = database

    @property
    def connection_string(self) -> str:
        """PostgreSQL connection string"""
        return f"postgresql://{self.host}:{self.port}/{self.database}"

    @classmethod
    def from_config(cls, config: dict) -> 'PostgresSource':
        """Create PostgresSource from configuration"""
        return cls(
            host=config['host'],
            port=config['port'],
            database=config['database']
        )

    def query(self, sql: str) -> list[dict]:
        """Execute PostgreSQL query (simplified for example)"""
        return [{"result": f"PostgreSQL executed: {sql}"}]


# Create from config
config = {'host': 'localhost', 'port': 5432, 'database': 'mydb'}
pg_source = PostgresSource.from_config(config)
print(pg_source.connection_string)  # postgresql://localhost:5432/mydb
print(pg_source.query("SELECT * FROM users"))
```

**Specification → AI Prompt Used → Generated Concepts → Validation**

At this point, you understand:
- ✅ Inheritance creates parent-child relationships
- ✅ Method overriding replaces parent implementations
- ✅ ABC enforces that subclasses implement specific methods/properties
- ✅ Abstract properties and class methods enforce richer contracts

Now comes a paradigm shift: **What if you don't need inheritance at all?**

---

## Duck Typing: The Pythonic Way

Here's a question that separates Python from languages like Java or C++: **Does an object need to inherit from a specific class to be polymorphic?**

Python's answer: **No. If it walks like a duck and quacks like a duck, it's a duck.**

This is **duck typing**—focusing on what an object can **do** (its interface) rather than what it **is** (its class). You don't need a common parent class; you just need objects that implement the same methods.

Consider payment processing. Instead of forcing all processors to inherit from a `PaymentProcessor` base class, why not just require them to have a `process_payment()` method?

```python
class CreditCardProcessor:
    """Process payments via credit card - no inheritance needed"""

    def process_payment(self, amount: float, card_number: str) -> bool:
        """Process credit card payment

        Args:
            amount: Payment amount
            card_number: Card number

        Returns:
            True if successful, False otherwise
        """
        print(f"Processing ${amount} via credit card {card_number[-4:]}")
        return True


class PayPalProcessor:
    """Process payments via PayPal - no inheritance needed"""

    def process_payment(self, amount: float, email: str) -> bool:
        """Process PayPal payment

        Args:
            amount: Payment amount
            email: PayPal email

        Returns:
            True if successful, False otherwise
        """
        print(f"Processing ${amount} via PayPal ({email})")
        return True


class CryptoProcessor:
    """Process payments via cryptocurrency - no inheritance needed"""

    def process_payment(self, amount: float, wallet_address: str) -> bool:
        """Process crypto payment

        Args:
            amount: Payment amount
            wallet_address: Wallet address

        Returns:
            True if successful, False otherwise
        """
        print(f"Processing ${amount} via crypto to {wallet_address}")
        return True


def checkout(processor, amount: float, **kwargs) -> None:
    """Process a checkout with any processor that has process_payment()

    Args:
        processor: Any object with a process_payment() method
        amount: Payment amount
        **kwargs: Additional arguments (card_number, email, wallet_address, etc.)
    """
    if processor.process_payment(amount, list(kwargs.values())[0]):
        print(f"✓ Payment of ${amount} successful\n")
    else:
        print(f"✗ Payment of ${amount} failed\n")


# All three processors work without inheritance or a common base class!
cc_processor = CreditCardProcessor()
paypal_processor = PayPalProcessor()
crypto_processor = CryptoProcessor()

checkout(cc_processor, 99.99, card_number="1234-5678-9012-3456")
checkout(paypal_processor, 49.99, email="user@example.com")
checkout(crypto_processor, 29.99, wallet_address="0x123abc456def789...")

# Output:
# Processing $99.99 via credit card 3456
# ✓ Payment of $99.99 successful
#
# Processing $49.99 via PayPal (user@example.com)
# ✓ Payment of $49.99 successful
#
# Processing $29.99 via crypto to 0x123abc456def789...
# ✓ Payment of $29.99 successful
```

**No `Payment` base class. No `@abstractmethod` decorators. Just objects that do what we need them to do.**

#### 🎓 Instructor Commentary

> Duck typing is the heart of Python's philosophy. It's more flexible than inheritance because you don't need to plan a hierarchy ahead of time. If you need a new processor, just implement `process_payment()` and it works. No inheritance chain, no complex base class design—just implement the interface you need.

#### 🚀 CoLearning Challenge

> Ask your AI: "Design an abstract `PaymentProcessor` base class with `@abstractmethod process_payment()`. Then show how the same system works with duck typing (no inheritance). Compare the code complexity and flexibility of each approach."

**Expected outcome**: You'll see that duck typing requires fewer lines of code and creates looser coupling between classes. But you'll also see that ABC provides explicit contracts (which can be valuable).

---

## When to Use ABC vs Duck Typing

This is where design judgment matters. Both approaches work. **Choosing correctly depends on your specific problem.**

### Use ABC When:

1. **You need explicit enforcement** — You want Python to reject incomplete subclasses at instantiation time
2. **You're building a framework** — Other developers will extend your classes, and you want to enforce their implementations
3. **You need documentation** — The ABC clearly documents what a subclass must implement
4. **You want static type checking** — Tools like `mypy` understand ABC contracts better than duck typing

**Example**: Building an agent framework where you want to guarantee every agent has `process()`, `get_status()`, and `shutdown()` methods.

### Use Duck Typing When:

1. **You're writing application code** — Not a framework; you control all the implementations
2. **You need flexibility** — New implementations might not fit a predefined interface
3. **The interface is simple** — Just one or two methods (like `read()` or `process()`)
4. **You want loose coupling** — Objects don't need to know about each other's class hierarchy

**Example**: Writing a data processing pipeline where readers might be files, URLs, databases, or API endpoints. Each just needs a `read()` method.

#### 💬 AI Colearning Prompt

> "Explain when to use ABC vs duck typing. Give me 3 scenarios for each approach and explain why each is appropriate."

---

## Real-World Example: File Readers

Let's see both approaches solving the same problem:

```python
from abc import ABC, abstractmethod
from typing import Protocol


# ============ APPROACH 1: ABC-Based ============

class Reader(ABC):
    """Abstract reader - enforces read() implementation"""

    @abstractmethod
    def read(self) -> str:
        """Read content

        Returns:
            Content as string
        """
        pass


class FileReader(Reader):
    """Read from file"""

    def __init__(self, filepath: str):
        self.filepath = filepath

    def read(self) -> str:
        with open(self.filepath, 'r') as f:
            return f.read()


class URLReader(Reader):
    """Read from URL"""

    def __init__(self, url: str):
        self.url = url

    def read(self) -> str:
        # Simplified - in reality would use requests library
        return f"Content from {self.url}"


def process_with_abc(reader: Reader) -> None:
    """Process content using ABC reader

    Args:
        reader: Must be a Reader subclass instance
    """
    content = reader.read()
    print(f"Processing {len(content)} characters")


# ============ APPROACH 2: Duck Typing ============

class DuckFileReader:
    """Read from file - no inheritance"""

    def __init__(self, filepath: str):
        self.filepath = filepath

    def read(self) -> str:
        with open(self.filepath, 'r') as f:
            return f.read()


class DuckURLReader:
    """Read from URL - no inheritance"""

    def __init__(self, url: str):
        self.url = url

    def read(self) -> str:
        return f"Content from {self.url}"


def process_with_duck(reader) -> None:
    """Process content using duck typing

    Args:
        reader: Any object with a read() method
    """
    content = reader.read()
    print(f"Processing {len(content)} characters")


# Both approaches work identically!
file_reader = FileReader("test.txt")
url_reader = URLReader("https://example.com")

process_with_abc(file_reader)
process_with_abc(url_reader)

duck_file = DuckFileReader("test.txt")
duck_url = DuckURLReader("https://example.com")

process_with_duck(duck_file)
process_with_duck(duck_url)
```

**Which is better?** For this simple case, duck typing is simpler and requires less code. For a complex framework with many implementations, ABC provides valuable explicit contracts.

---

## Polymorphic Collections

One of polymorphism's greatest powers: **You can store objects of different types in a single collection, as long as they share a common interface.**

```python
from abc import ABC, abstractmethod


class Agent(ABC):
    """Base agent - all agents respond to process()"""

    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def process(self, task: str) -> str:
        """Process a task

        Args:
            task: Task description

        Returns:
            Result of processing
        """
        pass


class ChatAgent(Agent):
    def process(self, task: str) -> str:
        return f"ChatAgent {self.name}: Conversing about '{task}'"


class CodeAgent(Agent):
    def process(self, task: str) -> str:
        return f"CodeAgent {self.name}: Analyzing code for '{task}'"


class ResearchAgent(Agent):
    def process(self, task: str) -> str:
        return f"ResearchAgent {self.name}: Researching '{task}'"


# Single collection of different agent types
agents: list[Agent] = [
    ChatAgent("Claude-Chat"),
    CodeAgent("Claude-Code"),
    ResearchAgent("Claude-Research")
]

# Route a task to all agents
task = "explain machine learning"

print(f"Task: {task}\n")
for agent in agents:
    result = agent.process(task)
    print(f"  {result}")

# Output:
# Task: explain machine learning
#
#   ChatAgent Claude-Chat: Conversing about 'explain machine learning'
#   CodeAgent Claude-Code: Analyzing code for 'explain machine learning'
#   ResearchAgent Claude-Research: Researching 'explain machine learning'
```

This pattern is **foundational for multi-agent systems**. You have a collection of heterogeneous agents that all respond to the same interface. Each agent specializes in its domain, but they can all be treated uniformly.

---

## Try With AI

Use your AI companion (Claude Code, Gemini CLI, or ChatGPT) to explore polymorphism and duck typing in depth.

### Prompt 1: Recall - ABC Syntax

Create an abstract `Vehicle` class with abstract methods `start()`, `stop()`, and `refuel()`. Create two concrete subclasses: `Car` and `ElectricCar`. Show what happens if you forget to implement a method in `ElectricCar`.

**Expected outcome**: You'll see the `TypeError` when trying to instantiate a class that doesn't implement all abstract methods.

---

### Prompt 2: Understand - Polymorphism vs Duck Typing

Explain when to use abstract base classes versus duck typing. Give me three scenarios where ABC is the right choice and three where duck typing is better. For each, explain the reasoning.

**Expected outcome**: You'll develop design judgment about when to enforce contracts (ABC) versus relying on behavior (duck typing).

---

### Prompt 3: Apply - Building an Agent System

Design an abstract `Agent` base class for a multi-agent system. Include abstract methods `process_message()` and `get_status()`. Create three concrete agent types (ChatAgent, CodeAgent, DataAgent). Demonstrate polymorphic message routing where different agent types handle messages differently.

**Expected outcome**: You'll build a realistic multi-agent architecture where all agents share a common interface but specialize in different domains.

---

### Prompt 4: Analyze - Protocol vs ABC

Python 3.8+ introduced `Protocol` from the `typing` module as an alternative to ABC. Compare `Protocol` versus `ABC`:

1. How do they differ in how they define contracts?
2. When would you choose `Protocol` over `ABC`?
3. Show a code example of the same system using both approaches.

**Expected outcome**: You'll understand structural subtyping (Protocol) versus nominal subtyping (ABC)—advanced concepts that enable you to write even more flexible code.

---

**Safety & Ethics Note**: When designing polymorphic systems with AI, remember: **The contract matters**. If an agent claims to implement `process()`, it must actually do what users expect. Use ABC to enforce contracts in production systems; use duck typing when you control all implementations and want flexibility during development.

