---
title: "Design Patterns (Capstone)"
chapter: 25
lesson: 5
duration_minutes: 80

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Singleton Pattern"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement Singleton pattern to ensure single instance existence with thread-safe initialization"

  - name: "Factory Pattern"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement Factory pattern to create objects without specifying exact classes, enabling dynamic creation"

  - name: "Observer Pattern"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement Observer pattern for event-driven systems with pub-sub communication"

  - name: "Strategy Pattern"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement Strategy pattern to encapsulate and switch algorithms at runtime"

  - name: "Design Pattern Selection"
    proficiency_level: "B2"
    category: "Soft"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can analyze problems and select appropriate design patterns based on architectural requirements"

  - name: "Multi-Pattern Integration"
    proficiency_level: "B2"
    category: "Soft"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can integrate multiple patterns into cohesive architectures for complex systems like multi-agent systems"

learning_objectives:
  - objective: "Implement Singleton pattern to ensure single instance and prevent multiple instantiations"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create AgentManager using Singleton with proper initialization guards"

  - objective: "Implement Factory pattern to decouple object creation from usage"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create AgentFactory with registry of agent types and dynamic creation"

  - objective: "Implement Observer pattern for event-driven agent communication"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create event bus with attach/detach/notify and multiple observers"

  - objective: "Implement Strategy pattern for runtime algorithm selection"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create agents with swappable decision strategies using Strategy pattern"

  - objective: "Analyze design problems and select appropriate patterns"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Given a system requirement, choose and justify pattern selection"

  - objective: "Integrate multiple patterns into cohesive AI agent architectures"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Build complete multi-agent system using all 4 patterns working together"

cognitive_load:
  new_concepts: 0
  assessment: "CAPSTONE LESSON - Integration phase. No new concepts introduced. All concepts from Ch24-25 (Lessons 1-4) synthesized and applied. Maximum synthesis at B2 proficiency level ✓"

differentiation:
  extension_for_advanced: "Implement additional patterns (Mediator for agent coordination, Command for task queuing, Decorator for capability composition). Explore design pattern combinations and anti-patterns."
  remedial_for_struggling: "Start with individual pattern implementation first; build integrated system step-by-step; use provided code templates as scaffolding; focus on understanding pattern intent before optimization"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/020-oop-part-1-2/spec-chapter-27.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Design Patterns (Capstone)

## Introduction: Architectural Thinking at Scale

You've spent Lessons 1-4 mastering the fundamental tools of object-oriented design: inheritance hierarchies, polymorphism, composition, and special methods. Now comes the moment where these tools become architecture.

**Design patterns** are reusable solutions to common architectural problems. They're the vocabulary of professional software engineering—when you tell another developer "we'll use the Observer pattern for agent communication," they immediately understand your architecture without you explaining details.

In this capstone lesson, you'll implement four industry-standard patterns in a real AI agent system. Each pattern solves a specific problem:

- **Singleton**: Ensure only one manager coordinates all agents
- **Factory**: Create different agent types dynamically without hardcoding classes
- **Observer**: Enable event-driven agent communication
- **Strategy**: Let agents select different decision-making approaches at runtime

By the end, you'll have built a professional multi-agent architecture that integrates all 4 patterns—the kind of system you'd encounter in production AI applications.

**Why this matters**: Design patterns separate junior developers from architects. Junior developers write code that works. Architects design systems that scale, adapt, and integrate. This lesson transitions you from code-writing to architectural thinking.

---

## Part 1: Singleton Pattern — Global State Done Right

### The Problem: Too Many Instances

Imagine you're building an agent management system. Every part of your code needs access to *the same* agent manager—no duplicates, no inconsistency. How do you ensure only one instance exists?

```python
# ❌ WRONG: Each call creates new instance
manager1 = AgentManager()
manager2 = AgentManager()
manager1.register_agent(agent1)
manager2.register_agent(agent2)
# Now agents are split between two managers—inconsistent state!
```

The **Singleton pattern** solves this: guarantee exactly one instance, globally accessible.

### The Solution: Singleton Implementation

```python
class AgentManager:
    """Singleton pattern - ensures only one instance exists globally"""

    _instance: 'AgentManager' | None = None
    _initialized: bool = False

    def __new__(cls) -> 'AgentManager':
        """Control instance creation at the class level"""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self) -> None:
        """Initialize only once, even if __new__ is called multiple times"""
        if self._initialized:
            return
        self._initialized = True
        self.agents: list[dict] = []
        self.agent_map: dict[str, dict] = {}

    def register_agent(self, agent_id: str, agent_data: dict) -> None:
        """Register an agent with the manager"""
        self.agents.append(agent_data)
        self.agent_map[agent_id] = agent_data
        print(f"✅ Registered agent: {agent_id}")

    def get_agent(self, agent_id: str) -> dict | None:
        """Retrieve an agent by ID"""
        return self.agent_map.get(agent_id)

    def list_agents(self) -> list[str]:
        """Get all registered agent IDs"""
        return list(self.agent_map.keys())


# Test singleton behavior
manager1 = AgentManager()
manager2 = AgentManager()

print(f"Same instance? {manager1 is manager2}")  # True!
print(f"manager1 id: {id(manager1)}")
print(f"manager2 id: {id(manager2)}")

manager1.register_agent("chat_bot", {"name": "ChatBot", "type": "chat"})
manager2.register_agent("code_bot", {"name": "CodeBot", "type": "code"})

# Both see the same agents because they're the same instance
print(f"Agents in manager1: {manager1.list_agents()}")  # ['chat_bot', 'code_bot']
print(f"Agents in manager2: {manager2.list_agents()}")  # ['chat_bot', 'code_bot']
```

**How it works**:

1. `__new__()` controls instance creation. First call creates instance and saves it in `_instance`. Subsequent calls return the same instance.
2. `__init__()` might be called multiple times (once per reference), so we guard initialization with `_initialized` flag.
3. Result: `AgentManager()` always returns the exact same object, no matter how many times you call it.

#### 🎓 Expert Insight

> Singletons are controversial in software engineering. **Use for**: configuration managers, loggers, connection pools—stateless or simple-state resources. **Avoid for**: anything that needs different state in tests or multiple independent instances. The controversy exists because global state makes testing and reasoning about code harder. But for true global coordination points (like the agent manager), Singleton is appropriate.

#### 💬 AI Colearning Prompt

> "Explain the difference between Singleton pattern and a module-level variable. Why use Singleton instead of just `manager = AgentManager()` at module level? What problem does Singleton solve that module-level variables don't?"

---

## Part 2: Factory Pattern — Decoupling Object Creation

### The Problem: Tight Coupling to Concrete Classes

As your system grows, you have multiple agent types:

```python
# ❌ WRONG: Hardcoded creation, tightly coupled
if agent_type == "chat":
    agent = ChatAgent(name)
elif agent_type == "code":
    agent = CodeAgent(name)
elif agent_type == "data":
    agent = DataAgent(name)
else:
    raise ValueError("Unknown agent type")
```

This is fragile. Adding a new agent type requires changing this code everywhere. The **Factory pattern** decouples object creation from usage.

### The Solution: Factory Implementation

```python
from abc import ABC, abstractmethod

class Agent(ABC):
    """Abstract base class for all agents"""

    def __init__(self, name: str) -> None:
        self.name = name

    @abstractmethod
    def process(self, message: str) -> str:
        """All agents must implement message processing"""
        pass

    @abstractmethod
    def get_capabilities(self) -> list[str]:
        """All agents must declare their capabilities"""
        pass


class ChatAgent(Agent):
    """Agent specialized for conversational interaction"""

    def process(self, message: str) -> str:
        return f"ChatAgent {self.name}: Processing conversation '{message}'"

    def get_capabilities(self) -> list[str]:
        return ["conversation", "context_understanding", "multi-turn_dialogue"]


class CodeAgent(Agent):
    """Agent specialized for code analysis and generation"""

    def process(self, message: str) -> str:
        return f"CodeAgent {self.name}: Analyzing code '{message}'"

    def get_capabilities(self) -> list[str]:
        return ["code_analysis", "syntax_checking", "refactoring", "testing"]


class DataAgent(Agent):
    """Agent specialized for data processing"""

    def process(self, message: str) -> str:
        return f"DataAgent {self.name}: Processing data '{message}'"

    def get_capabilities(self) -> list[str]:
        return ["data_analysis", "visualization", "cleaning", "aggregation"]


class AgentFactory:
    """Factory pattern - creates agents without specifying concrete classes"""

    # Registry maps agent type strings to classes
    _registry: dict[str, type[Agent]] = {
        "chat": ChatAgent,
        "code": CodeAgent,
        "data": DataAgent,
    }

    @classmethod
    def create_agent(cls, agent_type: str, name: str) -> Agent:
        """Create an agent by type name"""
        agent_class = cls._registry.get(agent_type)
        if agent_class is None:
            raise ValueError(
                f"Unknown agent type: {agent_type}. "
                f"Available: {list(cls._registry.keys())}"
            )
        return agent_class(name)

    @classmethod
    def register_agent_type(cls, type_name: str, agent_class: type[Agent]) -> None:
        """Register a new agent type (enables extension without modifying factory)"""
        cls._registry[type_name] = agent_class

    @classmethod
    def available_types(cls) -> list[str]:
        """List all registered agent types"""
        return list(cls._registry.keys())


# Usage: Create agents without knowing concrete classes
agents: list[Agent] = [
    AgentFactory.create_agent("chat", "ChatBot1"),
    AgentFactory.create_agent("code", "CodeHelper"),
    AgentFactory.create_agent("data", "DataAnalyzer"),
]

# Process messages polymorphically
for agent in agents:
    print(agent.process("Hello"))
    print(f"Capabilities: {agent.get_capabilities()}")
    print()
```

**How it works**:

1. `Agent` is an abstract interface. Concrete agents (ChatAgent, CodeAgent, DataAgent) implement it.
2. `AgentFactory.create_agent()` takes a type string and returns the appropriate agent.
3. The registry pattern allows new agent types to be added without modifying the factory method.
4. Code that uses agents doesn't care what concrete type it is—it just calls the `Agent` interface.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "Show me how to extend this factory with a registry pattern where agents can self-register their types. Then demonstrate creating a custom `RAGAgent` that automatically registers itself without modifying `AgentFactory` code."

**Expected Outcome**: You'll understand how to decouple class registration from class creation, enabling true plugin-like architecture.

#### 🎓 Expert Insight

> The Factory pattern is critical in AI systems. Your agent creation might be dynamic—loading from configuration, database, or user choice. Factory decouples "which agent type" (decided at runtime) from "how to create it" (stateless factory code). This is professional architecture.

---

## Part 3: Observer Pattern — Event-Driven Architecture

### The Problem: Tight Coupling Between Components

Imagine agents need to react to events:

```python
# ❌ WRONG: Direct coupling, hard to extend
agent1.receive_message("event")
agent2.receive_message("event")
agent3.receive_message("event")
# What if you add agent4? Modify all code?
# What if agents need different reactions? Hard to manage
```

The **Observer pattern** decouples senders from receivers using an event bus.

### The Solution: Observer Implementation

```python
from typing import Protocol

class Observer(Protocol):
    """Observer interface - any object with update() method can observe"""

    def update(self, event_type: str, data: dict) -> None:
        """Called when subject publishes an event"""
        ...


class EventBus:
    """Subject - manages observers and publishes events"""

    def __init__(self) -> None:
        self._observers: list[Observer] = []
        self._event_history: list[dict] = []

    def attach(self, observer: Observer) -> None:
        """Register an observer"""
        if observer not in self._observers:
            self._observers.append(observer)
            print(f"✅ Observer attached")

    def detach(self, observer: Observer) -> None:
        """Unregister an observer"""
        if observer in self._observers:
            self._observers.remove(observer)
            print(f"✅ Observer detached")

    def notify(self, event_type: str, data: dict) -> None:
        """Notify all observers of an event"""
        event = {"type": event_type, "data": data}
        self._event_history.append(event)

        print(f"📢 Publishing event: {event_type}")
        for observer in self._observers:
            observer.update(event_type, data)

    def get_history(self) -> list[dict]:
        """Retrieve event history"""
        return self._event_history


class Agent:
    """Agent that observes events"""

    def __init__(self, name: str) -> None:
        self.name = name

    def update(self, event_type: str, data: dict) -> None:
        """React to published events"""
        print(f"  → {self.name} received {event_type}: {data}")


class TaskQueue:
    """Another observer type - queue incoming tasks"""

    def __init__(self) -> None:
        self.tasks: list[dict] = []

    def update(self, event_type: str, data: dict) -> None:
        """Add tasks from events to queue"""
        if event_type == "task_created":
            self.tasks.append(data)
            print(f"  → TaskQueue queued: {data.get('task_id')}")


class Logger:
    """Another observer - logs all events"""

    def __init__(self, filename: str) -> None:
        self.filename = filename
        self.logs: list[str] = []

    def update(self, event_type: str, data: dict) -> None:
        """Log event to file (simulated)"""
        log_entry = f"[{event_type}] {data}"
        self.logs.append(log_entry)
        print(f"  → Logger recorded: {event_type}")


# Build an event-driven system
event_bus = EventBus()

agent1 = Agent("Agent1")
agent2 = Agent("Agent2")
task_queue = TaskQueue()
logger = Logger("system.log")

# All observers attach to the same event bus
event_bus.attach(agent1)
event_bus.attach(agent2)
event_bus.attach(task_queue)
event_bus.attach(logger)

# Publish events - all observers react automatically
event_bus.notify("user_message", {"user_id": "user123", "message": "Hello"})
event_bus.notify("task_created", {"task_id": "task001", "priority": "high"})
event_bus.notify("agent_status_changed", {"agent_id": "agent1", "status": "busy"})

print(f"Event history: {event_bus.get_history()}")
print(f"Queued tasks: {task_queue.tasks}")
print(f"Log entries: {logger.logs}")
```

**How it works**:

1. `Observer` is a protocol (interface) - any object with `update()` can observe.
2. `EventBus` maintains a list of observers and publishes events to all.
3. Observers (`Agent`, `TaskQueue`, `Logger`) react independently when events are published.
4. Adding new observer types requires no changes to EventBus.

#### 💬 AI Colearning Prompt

> "Explain how the Observer pattern differs from polling. In polling, an agent repeatedly checks 'Has an event happened?' In Observer, the event bus notifies agents. Why is event-driven better for AI agent systems that need to react in real-time?"

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "Extend this system with event filtering: observers should only receive events they care about. Implement a subscription system where agents subscribe to specific event types, and EventBus only notifies relevant observers."

**Expected Outcome**: You'll understand selective notification, reducing unnecessary processing.

---

## Part 4: Strategy Pattern — Runtime Algorithm Selection

### The Problem: Hardcoded Decision Logic

Agents need different decision-making strategies depending on context:

```python
# ❌ WRONG: Logic embedded in agent, hard to change
class Agent:
    def make_decision(self, context):
        if context['threat_level'] > 7:
            return "retreat"
        else:
            return "advance"
        # What if we need different logic? Modify Agent?
```

The **Strategy pattern** encapsulates algorithms so they're interchangeable.

### The Solution: Strategy Implementation

```python
from abc import ABC, abstractmethod

class DecisionStrategy(ABC):
    """Strategy interface - all strategies must implement decide()"""

    @abstractmethod
    def decide(self, context: dict) -> str:
        """Make a decision based on context"""
        pass


class AggressiveStrategy(DecisionStrategy):
    """Always attack"""

    def decide(self, context: dict) -> str:
        return "🔥 Aggressive: Attack immediately"


class DefensiveStrategy(DecisionStrategy):
    """Prioritize safety"""

    def decide(self, context: dict) -> str:
        return "🛡️ Defensive: Retreat and regroup"


class BalancedStrategy(DecisionStrategy):
    """Adapt based on threat level"""

    def decide(self, context: dict) -> str:
        threat = context.get('threat_level', 5)

        if threat > 8:
            return "⚠️ Balanced: Tactical retreat"
        elif threat > 5:
            return "⚠️ Balanced: Cautious advance"
        else:
            return "⚠️ Balanced: Confident push forward"


class AdaptiveStrategy(DecisionStrategy):
    """Learn from history"""

    def __init__(self) -> None:
        self.success_count: int = 0
        self.failure_count: int = 0

    def decide(self, context: dict) -> str:
        # Simplified: attack if we've been successful
        if self.success_count > self.failure_count:
            return "📈 Adaptive: Attack (success history favors it)"
        else:
            return "📈 Adaptive: Defend (learn from failures)"

    def record_success(self) -> None:
        """Update success history"""
        self.success_count += 1

    def record_failure(self) -> None:
        """Update failure history"""
        self.failure_count += 1


class Agent:
    """Agent uses Strategy pattern for decision-making"""

    def __init__(self, name: str, strategy: DecisionStrategy) -> None:
        self.name = name
        self.strategy = strategy

    def set_strategy(self, strategy: DecisionStrategy) -> None:
        """Switch strategies at runtime"""
        self.strategy = strategy
        print(f"  → {self.name} switched strategy")

    def make_decision(self, context: dict) -> str:
        """Delegate decision to strategy"""
        return f"{self.name}: {self.strategy.decide(context)}"


# Create agents with different strategies
context = {'threat_level': 6, 'resources': 100}

agent = Agent("Agent1", BalancedStrategy())

print("Initial strategy (Balanced):")
print(agent.make_decision(context))

# Switch to aggressive strategy
agent.set_strategy(AggressiveStrategy())
print("After switching to Aggressive:")
print(agent.make_decision(context))

# Switch to defensive strategy
agent.set_strategy(DefensiveStrategy())
print("After switching to Defensive:")
print(agent.make_decision(context))

# Use adaptive strategy that learns
adaptive = AdaptiveStrategy()
agent.set_strategy(adaptive)
print("Adaptive strategy (initially neutral):")
print(agent.make_decision(context))

adaptive.record_success()
adaptive.record_success()
print("After 2 successes:")
print(agent.make_decision(context))
```

**How it works**:

1. `DecisionStrategy` defines the interface for all strategies.
2. Concrete strategies (Aggressive, Defensive, Balanced, Adaptive) implement different algorithms.
3. `Agent` holds a reference to a strategy and delegates decisions to it.
4. `set_strategy()` switches strategies at runtime without changing Agent code.

#### 🎓 Expert Insight

> Strategy pattern is essential in AI systems. Different models, different reasoning approaches, different risk profiles—all are strategies. By encapsulating them, agents adapt their behavior without core logic changes. This is how real AI systems handle A/B testing and experimentation.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:

> "Design a CompoundStrategy that combines multiple strategies (like an ensemble). Have an agent use multiple strategies and aggregate their decisions. Show how this pattern enables collaborative decision-making."

**Expected Outcome**: You'll understand strategy composition, a technique used in real ensemble AI systems.

---

## Part 5: Bringing It All Together — The Multi-Agent Architecture

Now you'll integrate all 4 patterns into a cohesive system:

```python
# ============================================================
# CAPSTONE: Integrated Multi-Agent System
# Singleton + Factory + Observer + Strategy
# ============================================================

# 1. SINGLETON: Global agent manager
manager = AgentManager()

# 2. FACTORY: Create agents dynamically
print("Creating agents with Factory pattern:")
agents = [
    AgentFactory.create_agent("chat", "ChatBot"),
    AgentFactory.create_agent("code", "CodeHelper"),
    AgentFactory.create_agent("data", "DataAnalyzer"),
]

for agent_obj in agents:
    manager.register_agent(agent_obj.name, {"instance": agent_obj})

# 3. OBSERVER: Event-driven communication
print("Setting up event-driven communication:")
event_bus = EventBus()

# Wrap agents as observers
class AgentObserver:
    """Adapts Agent to Observer protocol"""
    def __init__(self, agent: Agent) -> None:
        self.agent = agent

    def update(self, event_type: str, data: dict) -> None:
        """Agent reacts to events"""
        message = data.get('message', '')
        response = self.agent.process(message)
        print(f"    {response}")

# Attach all agents to event bus
for agent_obj in agents:
    observer = AgentObserver(agent_obj)
    event_bus.attach(observer)

# 4. STRATEGY: Agents with decision strategies
print("Adding Strategy pattern for agent decisions:")
strategies = {
    agents[0]: AggressiveStrategy(),  # ChatBot is aggressive
    agents[1]: BalancedStrategy(),    # CodeHelper is balanced
    agents[2]: DefensiveStrategy(),   # DataAnalyzer is defensive
}

# Simulate agent decision-making
context = {'threat_level': 6, 'complexity': 'high'}
for agent_obj, strategy in strategies.items():
    decision = strategy.decide(context)
    print(f"  {agent_obj.name} decides: {decision}")

# 5. ORCHESTRATION: Events trigger agent actions
print("Publishing events (triggers all agents via Observer):")
event_bus.notify("user_request", {
    "message": "Please help me understand this error",
    "user_id": "user123"
})

# Check manager state (Singleton)
print(f"Agents registered with Singleton manager: {manager.list_agents()}")
```

**What you've accomplished**:

1. **Singleton** (`AgentManager`): Single coordination point for all agents
2. **Factory** (`AgentFactory`): Dynamic agent creation by type
3. **Observer** (`EventBus`): Event-driven agent communication
4. **Strategy**: Each agent uses a decision strategy

These patterns work together seamlessly because each solves one specific problem:
- Singleton handles *global state*
- Factory handles *object creation*
- Observer handles *loose coupling communication*
- Strategy handles *algorithm selection*

#### 🎓 Expert Insight

> This integrated system represents professional architecture. In production AI systems:
> - Singleton coordinates resources (expensive to create, need single instance)
> - Factory enables dynamic agent creation from config or user input
> - Observer enables asynchronous, loosely-coupled agent communication
> - Strategy enables experimentation with different reasoning approaches
>
> Learning to combine patterns is what separates architects from code writers.

---

## Challenge: Building Complex AI Systems with Design Patterns

In this capstone challenge, you'll integrate all four design patterns (plus additional ones) into a complete agent orchestration system. This is where theory meets practice—designing production-ready architectures.

---

## Part 1: Plan Multi-Pattern System with AI

**Your Role**: System architect planning integration of 4 design patterns with AI collaboration

### Planning Exercise: Design Multi-Pattern Agent Architecture

You're building a customer service platform that needs all 4 design patterns working together. Instead of manually typing 200+ lines of pattern code, you'll PLAN the architecture with AI, then have AI generate it.

#### 💬 AI CoLearning Prompt - Understanding Pattern Integration Needs

> "I'm building a customer service agent system with these requirements:
>
> **Requirement 1: Single Manager**
> - Only ONE AgentManager should exist globally (not multiple instances)
> - All parts of the system access the SAME manager instance
>
> **Requirement 2: Dynamic Agent Creation**
> - Create ChatAgent, TicketAgent, EscalationAgent by type name
> - Add new agent types without modifying creation code
>
> **Requirement 3: Event-Driven Communication**
> - Agents notify each other via events (not direct calls)
> - Adding new agents doesn't require modifying event system
>
> **Requirement 4: Swappable Routing Logic**
> - Route messages using different strategies (sentiment-based, round-robin)
> - Switch strategies at runtime without code changes
>
> For EACH requirement, tell me:
> 1. Which design pattern solves this problem?
> 2. What does that pattern do?
> 3. How does it solve the problem?
> 4. Give me a simple plain-language architecture description (no code yet!)"

**Expected Understanding**: AI will map:
- Requirement 1 → **Singleton pattern** (ensures single instance)
- Requirement 2 → **Factory pattern** (decouples creation from types)
- Requirement 3 → **Observer pattern** (event-driven, loose coupling)
- Requirement 4 → **Strategy pattern** (encapsulates algorithms)

You'll understand WHAT each pattern does BEFORE seeing implementation code.

#### 💬 AI CoLearning Prompt - Planning Pattern Interactions

> "Now that I know the 4 patterns, help me plan how they work TOGETHER:
>
> **My Architecture Plan:**
> - AgentManager (Singleton) coordinates everything
> - AgentFactory (Factory) creates agents
> - EventBus (Observer) handles communication
> - RoutingStrategy (Strategy) decides which agent processes messages
>
> Help me think through the interactions:
> 1. When a message arrives, what's the flow? (Manager → Strategy → Agent → EventBus?)
> 2. How does the Manager use the Factory? (Does it call Factory.create()?)
> 3. How do agents register with EventBus? (Manager does it, or agents do it themselves?)
> 4. Can I swap the RoutingStrategy after the system starts?
> 5. What's the initialization order? (Create patterns in what sequence?)
>
> Give me a step-by-step flow diagram in plain language showing how a message travels through all 4 patterns."

**Expected Understanding**: AI will explain the interaction flow:
1. Manager receives message
2. Manager asks RoutingStrategy which agent to use
3. Manager uses Factory to create agent if needed (or retrieves existing)
4. Agent processes message
5. Agent publishes result to EventBus
6. Other agents subscribed to EventBus react to event

You'll understand the DEPENDENCIES between patterns before implementing.

---

## Part 2: Generate System and Validate Design

**Your Role**: Design validator working with AI to generate and refine the multi-pattern system

### AI-Assisted Implementation

Instead of typing 300+ lines of pattern code manually, you'll have AI generate the complete system based on your architecture plan from Part 1, then YOU validate it.

#### 💬 AI CoLearning Prompt - Generate Complete Multi-Pattern System

> "Based on my architecture plan (4 patterns working together), generate the complete Python code for my agent system:
>
> **Requirements:**
>
> 1. **AgentManager (Singleton pattern)**:
>    - `__new__()` ensures only one instance
>    - `_initialized` flag prevents re-initialization
>    - `register_agent()`, `route_message()` methods
>    - Holds references to EventBus and RoutingStrategy
>
> 2. **AgentFactory (Factory pattern)**:
>    - `_registry` dict mapping type names to classes
>    - `create(agent_type, name)` creates agents dynamically
>    - `register(type_name, agent_class)` adds new types
>    - Supports ChatAgent, TicketAgent, EscalationAgent
>
> 3. **EventBus (Observer pattern)**:
>    - `_observers` list of subscribers
>    - `attach(observer)`, `detach(observer)`, `notify(event)` methods
>    - Supports any object with `update(event_type, data)` method
>
> 4. **RoutingStrategy (Strategy pattern)**:
>    - Abstract base with `route(message)` method
>    - SentimentBasedStrategy (routes by keywords like 'urgent', 'ticket')
>    - RoundRobinStrategy (cycles through agents)
>
> 5. **Agent base class**:
>    - Abstract with `process(message)` method
>    - ChatAgent, TicketAgent, EscalationAgent concrete implementations
>
> **Important:**
> - Add type hints to everything
> - Include docstrings explaining each pattern's role
> - Show main.py demonstrating all patterns working together
> - Use proper module structure (patterns/, agents/, main.py)
>
> Generate the complete code. I'll validate it against my architecture plan."

**Expected AI Output**: ~150-200 lines of complete, working code implementing all 4 patterns with proper integration.

---

### Validation Exercise: Check AI's Pattern Implementation

**Your task**: Read the AI-generated code and validate these architectural decisions:

**✅ Validation Checklist:**

1. **Singleton Pattern**:
   - Does AgentManager have `_instance` class variable? ✓
   - Does `__new__()` return same instance every time? ✓
   - Does `__init__()` guard against re-initialization? ✓
   - Test: Create two references - are they the same object (same id)? ✓

2. **Factory Pattern**:
   - Does AgentFactory have a registry dict? ✓
   - Can I create agents without importing concrete classes? ✓
   - Can I register new agent types without modifying factory code? ✓
   - Test: Add new agent type - does it work without editing Factory? ✓

3. **Observer Pattern**:
   - Does EventBus maintain list of observers? ✓
   - Can observers attach/detach dynamically? ✓
   - Does notify() call update() on all observers? ✓
   - Test: Add 3 agents as observers, publish event - do all receive it? ✓

4. **Strategy Pattern**:
   - Is RoutingStrategy an abstract base class? ✓
   - Can I create multiple concrete strategies? ✓
   - Can AgentManager's strategy be swapped at runtime? ✓
   - Test: Switch from SentimentBased to RoundRobin - does routing change? ✓

5. **Pattern Integration**:
   - Does AgentManager use AgentFactory to create agents? ✓
   - Does AgentManager use RoutingStrategy to route messages? ✓
   - Do agents publish events to EventBus? ✓
   - Does the system work end-to-end (message in → routed → processed → event out)? ✓

#### 🤝 CoLearning Convergence - Refining the Design

After validating the basic implementation, ask AI to enhance it:

> "The basic 4-pattern system works! Now enhance it with these real-world features:
> 1. Add logging to AgentManager showing which pattern is used for each operation
> 2. Add error handling: what if Factory can't create agent type?
> 3. Add metrics: track how many messages each strategy routes
> 4. Add event filtering: agents subscribe only to specific event types
>
> Show me the enhanced code focusing on production-readiness."

**Expected Outcome**: AI adds production features while maintaining the 4-pattern architecture. You'll see how patterns compose into robust systems.

### Deliverable

Write 2-paragraph summary: "How I Validated and Enhanced a Multi-Pattern System" explaining:
- What architecture decisions you validated
- What production features you added
- Why pattern composition enables extensibility

---

## Part 3: Challenge AI with Complex Architecture Questions

**Your Role**: Student testing AI's understanding of pattern interactions

### Challenge Design Scenarios

Ask AI to handle these cases:

#### Challenge 1: Pattern Conflicts

> "If I use Singleton for AgentManager and Dependency Injection to pass the manager to agents, which is better? Can I use both? What problems could arise if I mix them?"

**Expected learning**: AI explains trade-offs between patterns and when to mix/match approaches.

#### Challenge 2: Pattern Composition

> "I want to add three more patterns: Mediator (for agent communication), Command (for queuing tasks), Decorator (for dynamic capabilities). How do they interact with Singleton, Factory, Observer, and Strategy? Show the architecture diagram."

**Expected learning**: AI shows how multiple patterns compose into larger systems without conflicting.

#### Challenge 3: Testing and Patterns

> "With Singleton and Observer, testing becomes harder (global state, hidden dependencies). How would you design these patterns to be testable? Should I use Dependency Injection instead?"

**Expected learning**: AI explains testability implications of patterns and how to design for both.

### Deliverable

Document your three challenges, AI's responses, and your understanding of pattern trade-offs and composition.

---

## Part 4: Build Complete Multi-Pattern Agent Orchestration System

**Your Role**: Architect designing production system

Create a professional multi-agent system integrating all four patterns:

```
agent_system/
├── patterns/
│   ├── __init__.py
│   ├── singleton.py       # AgentManager (Singleton)
│   ├── factory.py         # AgentFactory (Factory)
│   ├── observer.py        # EventBus (Observer)
│   └── strategy.py        # RoutingStrategy (Strategy)
├── agents/
│   ├── __init__.py
│   ├── base.py            # Agent base class
│   ├── chat_agent.py      # Concrete agent
│   ├── ticket_agent.py    # Concrete agent
│   └── escalation_agent.py
├── events/
│   ├── __init__.py
│   └── events.py          # Event definitions
└── main.py                # Integration
```

**patterns/singleton.py**:
```python
from typing import Dict, List, Optional


class AgentManager:
    """Singleton pattern - ensures single manager instance"""

    _instance: Optional['AgentManager'] = None
    _initialized = False

    def __new__(cls) -> 'AgentManager':
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self) -> None:
        if not AgentManager._initialized:
            self.agents: Dict[str, 'Agent'] = {}
            self.event_bus: Optional['EventBus'] = None
            self.routing_strategy: Optional['RoutingStrategy'] = None
            AgentManager._initialized = True

    def register_agent(self, name: str, agent: 'Agent') -> None:
        self.agents[name] = agent

    def route_message(self, message: str) -> str:
        if not self.routing_strategy:
            raise RuntimeError("Routing strategy not set")
        agent_name = self.routing_strategy.route(message)
        return self.agents[agent_name].process(message)
```

**patterns/factory.py**:
```python
from typing import Dict, Type, Optional


class AgentFactory:
    """Factory pattern - creates agents without exposing types"""

    def __init__(self) -> None:
        self._registry: Dict[str, Type] = {}

    def register(self, agent_type: str, agent_class: Type) -> None:
        self._registry[agent_type] = agent_class

    def create(self, agent_type: str, name: str) -> 'Agent':
        if agent_type not in self._registry:
            raise ValueError(f"Unknown agent type: {agent_type}")
        return self._registry[agent_type](name)

    def get_available_types(self) -> list[str]:
        return list(self._registry.keys())
```

**patterns/observer.py**:
```python
from typing import List, Callable, Any


class EventBus:
    """Observer pattern - publish-subscribe event system"""

    def __init__(self) -> None:
        self._observers: List[Callable] = []

    def attach(self, observer: Callable) -> None:
        self._observers.append(observer)

    def detach(self, observer: Callable) -> None:
        if observer in self._observers:
            self._observers.remove(observer)

    def notify(self, event: Any) -> None:
        for observer in self._observers:
            observer(event)
```

**patterns/strategy.py**:
```python
from abc import ABC, abstractmethod


class RoutingStrategy(ABC):
    """Strategy pattern - encapsulates routing algorithms"""

    @abstractmethod
    def route(self, message: str) -> str:
        pass


class SentimentBasedStrategy(RoutingStrategy):
    """Route based on message sentiment"""

    def route(self, message: str) -> str:
        if "urgent" in message.lower():
            return "escalation_agent"
        elif "ticket" in message.lower():
            return "ticket_agent"
        return "chat_agent"


class RoundRobinStrategy(RoutingStrategy):
    """Route in round-robin fashion"""

    def __init__(self) -> None:
        self.counter = 0
        self.agents = ["chat_agent", "ticket_agent", "escalation_agent"]

    def route(self, message: str) -> str:
        agent = self.agents[self.counter % len(self.agents)]
        self.counter += 1
        return agent
```

**agents/base.py**:
```python
from abc import ABC, abstractmethod


class Agent(ABC):
    """Base agent class"""

    def __init__(self, name: str) -> None:
        self.name = name

    @abstractmethod
    def process(self, message: str) -> str:
        pass
```

**agents/chat_agent.py**:
```python
from agents.base import Agent


class ChatAgent(Agent):
    """Concrete chat agent"""

    def process(self, message: str) -> str:
        return f"[ChatAgent {self.name}] Processing: {message}"
```

**main.py**:
```python
from patterns.singleton import AgentManager
from patterns.factory import AgentFactory
from patterns.observer import EventBus
from patterns.strategy import SentimentBasedStrategy, RoundRobinStrategy
from agents.chat_agent import ChatAgent

# Initialize patterns
manager = AgentManager()  # Singleton - same instance always
factory = AgentFactory()  # Factory - creates agents
event_bus = EventBus()    # Observer - event system
strategy = SentimentBasedStrategy()  # Strategy - routing logic

# Register agent types with factory
factory.register("chat", ChatAgent)

# Create agents via factory
chat = factory.create("chat", "Helper")

# Register agents with manager
manager.register_agent("chat_agent", chat)

# Set up event system and routing
manager.event_bus = event_bus
manager.routing_strategy = strategy

# Use the system
result = manager.route_message("I need help")
print(result)
```

**Your task**: Expand this system with:
1. Add TicketAgent and EscalationAgent
2. Implement EventBus with agent event handlers
3. Add Mediator pattern for complex agent communication
4. Create comprehensive test suite for all patterns
5. Write architecture guide: `design_patterns_integration.md`

### Validation Checklist

- ✅ AgentManager is true Singleton (same instance everywhere)
- ✅ AgentFactory creates agents without revealing classes
- ✅ EventBus allows pub-sub between agents
- ✅ RoutingStrategy can be swapped at runtime
- ✅ All patterns work together seamlessly
- ✅ Adding new agent type requires only Factory registration
- ✅ Adding new routing strategy needs only new Strategy class

### Deliverable

Complete multi-agent system with:
- All 4 core patterns (Singleton, Factory, Observer, Strategy)
- At least 3 concrete agent types
- Multiple routing strategies
- Event system connecting agents
- Comprehensive test suite
- Architecture documentation

---

## Part 5: Reflect on All Chapter 30 Concepts

**Your Role**: Educational synthesist integrating all learning

This is unique to the capstone—you'll reflect on what you've learned across all five lessons.

### Reflection Prompt

Write a comprehensive reflection document: `chapter_25_integration_reflection.md` with sections:

#### Section 1: Inheritance and MRO (Lesson 1)
Reflect on:
- How did understanding Method Resolution Order change your thinking about class hierarchies?
- When would you use single vs multiple inheritance?
- How did inheritance patterns appear in later lessons?

#### Section 2: Polymorphism and Duck Typing (Lesson 2)
Reflect on:
- What's the key insight that separates polymorphism from duck typing?
- In your multi-agent system, where did you use each approach? Why?
- How would your design differ if you HAD to use duck typing everywhere?

#### Section 3: Composition Over Inheritance (Lesson 3)
Reflect on:
- Your agents are composed from engines. How is this better than inheritance?
- What would the codebase look like if agents inherited from TicketAgent, ChatAgent, etc.?
- How did composition principles appear in your strategy and observer implementations?

#### Section 4: Special Methods and Protocols (Lesson 4)
Reflect on:
- How did special methods make your custom types feel like built-ins?
- In an agent system, where might you use __call__, __enter__/__exit__, or __iter__?
- What's the relationship between protocols and polymorphism?

#### Section 5: Design Patterns Integration (Lesson 5)
Reflect on:
- How do Singleton, Factory, Observer, and Strategy solve fundamental OOP problems?
- Could you design your system with only some patterns? What would you lose?
- What's the relationship between design patterns and the OOP concepts from Lessons 1-4?

#### Section 6: Architectural Synthesis
Reflect on:
- In your system, inheritance is used for agent types. Composition is used for agent capabilities. Polymorphism makes routing work. Special methods make events feel natural.
- How are all these concepts working together?
- Can you trace a message through your system and identify which OOP concept is responsible at each step?
- What would break if you removed each concept?

### Deliverable

Complete `chapter_25_integration_reflection.md` with thoughtful answers to all reflection questions, demonstrating deep understanding of how all Chapter 30 concepts interconnect.

---

## Try With AI

How do you coordinate Singleton (AgentManager), Factory (agent creation), Observer (events), and Strategy (routing) in one system?

**🔍 Explore Pattern Integration:**
> "Show me how AgentManager (Singleton) uses AgentFactory (Factory) to create agents, EventBus (Observer) to broadcast events, and RoutingStrategy (Strategy) to route messages. Trace a message through all 4 patterns."

**🎯 Practice Pattern Composition:**
> "Design a system where Factory creates agents, Singleton manages them, Observer notifies on state changes, and Strategy determines behavior. Explain dependency flow and initialization order."

**🧪 Test Pattern Interactions:**
> "What happens when AgentFactory creates an agent that registers with EventBus and AgentManager simultaneously? Show the sequence of method calls and explain potential race conditions."

**🚀 Apply to Production Architecture:**
> "Build a complete multi-agent system using all Chapter 30 concepts: inheritance (agent types), polymorphism (dispatch), composition (capabilities), special methods (Event __eq__), and all 4 patterns. Explain design decisions."

---
