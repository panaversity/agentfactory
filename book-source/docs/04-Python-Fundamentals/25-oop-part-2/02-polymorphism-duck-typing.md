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
generated_by: "content-implementer v3.0.0"
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

#### 🎓 Expert Insight

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

#### 🎓 Expert Insight

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

## Challenge: Building a Polymorphic Agent Dispatcher

In this challenge, you'll discover why polymorphism matters, learn how both ABC and duck typing work, test your understanding against AI, and build a production-ready system.

---

## Part 1: Student Discovers Type-Checking Problems in Agent Dispatch

**Your Role**: System architect identifying design gaps

### Discovery Exercise: Build an Agent Dispatcher Without Polymorphism

Imagine you're building a multi-agent dispatcher that routes messages to different agent types. Without polymorphism, you must check types explicitly.

**Stage 1: The Type-Checking Problem**

Create three agent classes and a dispatcher that checks type explicitly:

```python
# agents.py - Type-checking approach (PROBLEMATIC)
class ChatAgent:
    def __init__(self, name: str):
        self.name = name

    def process(self, message: str) -> str:
        return f"Chat: {message}"


class CodeAgent:
    def __init__(self, name: str):
        self.name = name

    def process(self, message: str) -> str:
        return f"Code Analysis: {message}"


class DataAgent:
    def __init__(self, name: str):
        self.name = name

    def process(self, message: str) -> str:
        return f"Data Processing: {message}"


# dispatcher.py - Without polymorphism
def dispatch_message(agent, message: str) -> str:
    """Route message to agent - REQUIRES TYPE CHECKING"""
    if isinstance(agent, ChatAgent):
        return agent.process(message)
    elif isinstance(agent, CodeAgent):
        return agent.process(message)
    elif isinstance(agent, DataAgent):
        return agent.process(message)
    else:
        raise ValueError(f"Unknown agent type: {type(agent)}")
```

**Your task 1**: Copy this code and document in `dispatcher_type_checking_analysis.md`:
- How many `isinstance()` checks exist?
- If you add a 4th agent type (ImageAgent), what must change in `dispatch_message()`?
- What happens if you forget to add the check?
- What pattern do you notice?

**Stage 2: Adding New Agents Breaks Existing Code**

**Your task 2**: Try adding `ImageAgent` without modifying `dispatch_message()`. Document:
- The dispatcher still works, but does it route correctly?
- How would you even know it's broken?
- What architectural problem does this reveal?

### Your Discovery Document

Create `polymorphism_problem_statement.md` with:

1. **The Type-Checking Problem**: Dispatcher code is coupled to specific agent types
2. **The Scaling Problem**: What happens at 20 agent types? 100?
3. **The Maintenance Risk**: How risky is adding a new agent type?
4. **Your Prediction**: What language feature would eliminate this coupling?

---

## Part 2: AI Teaches Polymorphism and Duck Typing as Solutions

**Your Role**: Student learning from AI Teacher

### AI Teaching Prompt

Ask your AI companion:

> "I built an agent dispatcher with type checking. ChatAgent, CodeAgent, DataAgent each have a `process(message)` method. The dispatcher uses `isinstance()` to route messages:
>
> ```python
> if isinstance(agent, ChatAgent):
>     return agent.process(message)
> elif isinstance(agent, CodeAgent):
>     return agent.process(message)
> # ... more checks
> ```
>
> This is fragile. If I add ImageAgent, I must modify the dispatcher.
>
> Explain:
> 1. What is polymorphism? How does it eliminate type checking?
> 2. Show me two solutions: ABC with @abstractmethod, and duck typing without inheritance
> 3. In each approach, what happens when I add a new agent type? Must I modify the dispatcher?
> 4. When would you choose ABC vs duck typing for this problem?"

### Expected AI Response Summary

AI will explain:
- **Polymorphism**: Same interface, different implementations—Python calls the right method based on object type
- **ABC approach**: Inherit from Agent base class with @abstractmethod process(); dispatcher accepts Agent, no type checking
- **Duck typing approach**: No inheritance; any object with process() method works; dispatcher is even simpler
- **Adding new agents**: Both approaches allow adding agents WITHOUT modifying the dispatcher
- **Design trade-off**: ABC enforces contracts explicitly; duck typing gives flexibility

**AI will show code like**:

```python
# SOLUTION 1: ABC Polymorphism
from abc import ABC, abstractmethod

class Agent(ABC):
    @abstractmethod
    def process(self, message: str) -> str:
        pass

class ChatAgent(Agent):
    def process(self, message: str) -> str:
        return f"Chat: {message}"

# Dispatcher is simple - no type checking!
def dispatch_message(agent: Agent, message: str) -> str:
    return agent.process(message)  # Works for any Agent subclass


# SOLUTION 2: Duck Typing (No Inheritance)
class ChatAgent:  # No inheritance!
    def process(self, message: str) -> str:
        return f"Chat: {message}"

# Same dispatcher - even simpler!
def dispatch_message(agent, message: str) -> str:
    return agent.process(message)  # Works if agent has process()
```

### Convergence Activity

After AI explains, verify understanding:

> "Explain what happens in the ABC approach when I create 100 different agent types. Do I need to modify the dispatcher each time? Walk me through the duck typing approach and explain why it's even more flexible."

### Deliverable

Write 1-paragraph summary: "How Polymorphism Eliminates Type Checking" explaining the core insight and trade-offs between ABC and duck typing.

---

## Part 3: Student Challenges AI with Design Edge Cases

**Your Role**: Student testing AI's understanding

### Challenge Design Scenarios

Ask AI to handle these cases:

#### Challenge 1: ABC Contract Enforcement

> "Show me what happens if I create a ChatAgent that inherits from Agent but forgets to implement process(). Then show what happens if I use duck typing and forget to implement process(). What's different about when these errors are caught?"

**Expected learning**: AI explains that ABC catches errors at instantiation time; duck typing catches them at call time.

#### Challenge 2: Adding Capability to All Agents

> "I want to add logging to all agents. With ABC, I add a log() method to the Agent base class. All agents inherit it automatically. How would duck typing handle this? What's the problem?"

**Expected learning**: AI explains that duck typing requires manual updates to each class, showing the trade-off.

#### Challenge 3: Runtime Behavior Discovery

> "I have a duck-typed processor function that accepts any object with process(). I pass in an object that has process() but it crashes. How is this different from ABC, which would have caught the problem earlier?"

**Expected learning**: AI explains structural vs nominal typing, and when each catches errors.

### Deliverable

Document your three challenges, AI's responses, and analysis of when each approach (ABC vs duck typing) is appropriate.

---

## Part 4: Build Polymorphic Agent Dispatcher for Production

**Your Role**: Knowledge synthesizer creating reusable code

### Your Agent Dispatcher System

Create `agent_dispatcher.py` with both approaches, demonstrating when each is appropriate:

```python
from abc import ABC, abstractmethod
from typing import Protocol


# ============ APPROACH 1: ABC Polymorphism ============

class AgentABC(ABC):
    """Contract-based polymorphism - enforces implementation"""

    def __init__(self, name: str) -> None:
        self.name = name

    @abstractmethod
    def process(self, message: str) -> str:
        """All agents MUST implement message processing"""
        pass

    def log(self, message: str) -> None:
        """Shared logging - inherited by all agents"""
        print(f"[{self.name}] {message}")


class ChatAgentABC(AgentABC):
    """Concrete chat agent using ABC"""

    def process(self, message: str) -> str:
        self.log(f"Processing: {message}")
        return f"Chat response to: {message}"


class CodeAgentABC(AgentABC):
    """Concrete code agent using ABC"""

    def process(self, message: str) -> str:
        self.log(f"Analyzing: {message}")
        return f"Code analysis for: {message}"


def dispatch_with_abc(agent: AgentABC, message: str) -> str:
    """Polymorphic dispatcher using ABC - no type checking!"""
    return agent.process(message)


# ============ APPROACH 2: Duck Typing ============

class ChatAgentDuck:
    """Chat agent using duck typing - no inheritance"""

    def __init__(self, name: str) -> None:
        self.name = name

    def process(self, message: str) -> str:
        return f"Chat response to: {message}"


class CodeAgentDuck:
    """Code agent using duck typing - no inheritance"""

    def __init__(self, name: str) -> None:
        self.name = name

    def process(self, message: str) -> str:
        return f"Code analysis for: {message}"


def dispatch_with_duck(agent, message: str) -> str:
    """Polymorphic dispatcher using duck typing - even simpler!"""
    return agent.process(message)


# ============ APPROACH 3: Protocol (Structural Typing) ============

class ProcessorProtocol(Protocol):
    """Structural protocol - defines what agents must do"""

    def process(self, message: str) -> str:
        """Any object implementing this interface works"""
        ...


def dispatch_with_protocol(agent: ProcessorProtocol, message: str) -> str:
    """Polymorphic dispatcher using Protocol - type-safe but flexible"""
    return agent.process(message)


# Test all three approaches
if __name__ == "__main__":
    print("=== ABC Approach ===")
    chat_abc = ChatAgentABC("ChatBot")
    code_abc = CodeAgentABC("CodeHelper")

    print(dispatch_with_abc(chat_abc, "hello"))
    print(dispatch_with_abc(code_abc, "debug this"))

    print("\n=== Duck Typing Approach ===")
    chat_duck = ChatAgentDuck("ChatBot")
    code_duck = CodeAgentDuck("CodeHelper")

    print(dispatch_with_duck(chat_duck, "hello"))
    print(dispatch_with_duck(code_duck, "debug this"))

    # Add a completely new agent type without modifying dispatcher
    class ImageAgentDuck:
        def __init__(self, name: str) -> None:
            self.name = name

        def process(self, message: str) -> str:
            return f"Image processing for: {message}"

    print("\n=== New Agent (Duck Typing) ===")
    image = ImageAgentDuck("ImageBot")
    print(dispatch_with_duck(image, "analyze image"))
```

**Your task**: Expand this system with:
1. Add 2-3 more specialized agent types (SearchAgent, ResearchAgent)
2. Create a dispatcher registry that tracks all agent types
3. Add a comparison document: `abc_vs_duck_typing.md` explaining when to use each
4. Show that adding new agent types requires NO changes to dispatcher code

### Validation Checklist

- ✅ ABC approach works with inherited agents
- ✅ Duck typing approach works with unrelated classes
- ✅ Protocol approach provides static type safety
- ✅ Dispatcher code never uses isinstance() or type checks
- ✅ Adding new agent types requires changing only the agent class
- ✅ All three approaches produce identical behavior

### Deliverable

Complete `agent_dispatcher.py` with all three polymorphic approaches and documentation explaining:
- How polymorphism eliminates type checking
- When ABC provides value (contracts, inheritance of shared methods)
- When duck typing wins (flexibility, loose coupling)
- When Protocol is useful (static type checking without inheritance)

---

## Summary: Bidirectional Learning in Action

**Part 1 (Student discovers)**: You identified the type-checking and coupling problems with explicit isinstance checks

**Part 2 (AI teaches)**: AI explained how polymorphism—both ABC and duck typing—eliminates this coupling

**Part 3 (Student teaches)**: You challenged AI with edge cases about when each approach catches errors

**Part 4 (Knowledge synthesis)**: You built a production dispatcher demonstrating all three polymorphic approaches

### What You've Built

1. `dispatcher_type_checking_analysis.md` — Problem analysis
2. `polymorphism_problem_statement.md` — Clear problem statement
3. Challenge documentation — Three edge cases you posed to AI
4. `agent_dispatcher.py` — Production dispatcher with ABC, duck typing, and Protocol approaches
5. `abc_vs_duck_typing.md` — Comparison guide for design decisions

### Next Steps

Lesson 3 shows how composition and modules organize agent systems. You'll use this polymorphic agent framework to build modular, scalable architectures.

