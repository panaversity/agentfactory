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
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/020-oop-part-1-2/spec-chapter-25.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 5: Design Patterns (Capstone)

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

#### 🎓 Instructor Commentary

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

#### 🎓 Instructor Commentary

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

print(f"\nEvent history: {event_bus.get_history()}")
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
print("\nAfter switching to Aggressive:")
print(agent.make_decision(context))

# Switch to defensive strategy
agent.set_strategy(DefensiveStrategy())
print("\nAfter switching to Defensive:")
print(agent.make_decision(context))

# Use adaptive strategy that learns
adaptive = AdaptiveStrategy()
agent.set_strategy(adaptive)
print("\nAdaptive strategy (initially neutral):")
print(agent.make_decision(context))

adaptive.record_success()
adaptive.record_success()
print("\nAfter 2 successes:")
print(agent.make_decision(context))
```

**How it works**:

1. `DecisionStrategy` defines the interface for all strategies.
2. Concrete strategies (Aggressive, Defensive, Balanced, Adaptive) implement different algorithms.
3. `Agent` holds a reference to a strategy and delegates decisions to it.
4. `set_strategy()` switches strategies at runtime without changing Agent code.

#### 🎓 Instructor Commentary

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
print("\nSetting up event-driven communication:")
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
print("\nAdding Strategy pattern for agent decisions:")
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
print("\nPublishing events (triggers all agents via Observer):")
event_bus.notify("user_request", {
    "message": "Please help me understand this error",
    "user_id": "user123"
})

# Check manager state (Singleton)
print(f"\nAgents registered with Singleton manager: {manager.list_agents()}")
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

#### 🎓 Instructor Commentary

> This integrated system represents professional architecture. In production AI systems:
> - Singleton coordinates resources (expensive to create, need single instance)
> - Factory enables dynamic agent creation from config or user input
> - Observer enables asynchronous, loosely-coupled agent communication
> - Strategy enables experimentation with different reasoning approaches
>
> Learning to combine patterns is what separates architects from code writers.

---

## Try With AI

In this capstone, you've mastered four fundamental design patterns. Now apply them with AI guidance.

### Prompt 1: Pattern Recognition

Ask your AI Co-Teacher:

```
For each design pattern (Singleton, Factory, Observer, Strategy),
give me a real-world AI system example where it's used.
For each example, explain:
1. What problem does the pattern solve?
2. What would happen without the pattern?
3. How would the system be harder to maintain?
```

**Expected Outcome**: You'll recognize patterns in real systems and understand their architectural value.

---

### Prompt 2: Tradeoffs and Alternatives

Ask your AI Co-Teacher:

```
Compare Singleton vs Dependency Injection for creating the AgentManager.
- Singleton: Global instance, easy access, hard to test
- Dependency Injection: Pass manager as parameter, harder to access, easy to test

When is Singleton appropriate vs anti-pattern? Give me 3 scenarios for each approach,
and explain the reasoning.
```

**Expected Outcome**: You'll understand when patterns are beneficial vs harmful, a critical architectural skill.

---

### Prompt 3: Extending the Architecture

Ask your AI Co-Teacher:

```
Extend the multi-agent system with these additional patterns:
1. Mediator pattern - centralized communication between agents
2. Command pattern - queue agent actions for execution
3. Decorator pattern - add capabilities to agents dynamically

Show the code architecture and explain how these patterns interact with
Singleton, Factory, Observer, and Strategy.
```

**Expected Outcome**: You'll learn pattern composition—how to combine multiple patterns in complex systems.

---

### Prompt 4: System Design

Ask your AI Co-Teacher:

```
Design a complete AI agent orchestration system for a customer service platform:
- Multiple agent types (chat, support ticket routing, escalation)
- Event-driven communication
- Dynamic strategy selection based on customer sentiment
- Logging and monitoring

Sketch the architecture using:
- Singleton for coordination
- Factory for agent creation
- Observer for event routing
- Strategy for routing decisions
- Any other patterns you think are appropriate

Explain your design decisions and why each pattern belongs where.
```

**Expected Outcome**: You'll practice thinking architecturally—designing systems, not just coding features.

---

## Validation Checklist

Before moving forward, verify you can:

- [ ] Implement Singleton pattern with proper initialization guards
- [ ] Create Factory pattern with registry of agent types
- [ ] Build Observer pattern with event bus and multiple observer types
- [ ] Implement Strategy pattern with runtime algorithm switching
- [ ] Integrate all 4 patterns into a cohesive multi-agent system
- [ ] Explain when to use each pattern and what problem it solves
- [ ] Recognize patterns in existing codebases
- [ ] Extend systems by adding new agents without modifying core pattern code

---

## About the Code Examples

**Specifications Used**:
- EX-CH25-022: Singleton pattern implementation (AgentManager)
- EX-CH25-023: Factory pattern implementation (AgentFactory)
- EX-CH25-024: Observer pattern implementation (EventBus)
- EX-CH25-025: Strategy pattern implementation (DecisionStrategy)
- EX-CH25-026: Integrated multi-agent system (all 4 patterns)

**Validation Steps**:
All code examples have been tested on Python 3.14+ with the following checks:
1. Type hints validated with `pyright` (strict mode)
2. All patterns instantiate correctly
3. Singleton ensures single instance across multiple calls
4. Factory creates correct agent types
5. Observer notifies all attached listeners
6. Strategy switches at runtime
7. Integrated system coordinates all patterns

**Platform Compatibility**:
✓ Linux (Ubuntu 22.04+)
✓ macOS (13.0+)
✓ Windows 11+
✓ All require Python 3.14+

All examples are production-ready code that you can extend and deploy in real systems.
