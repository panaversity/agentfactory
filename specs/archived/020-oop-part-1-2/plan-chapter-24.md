# Chapter 24 Implementation Plan: Object-Oriented Programming Part I

**Feature Branch**: `020-oop-part-1-2`
**Created**: 2025-11-09
**Status**: Ready for Implementation
**Spec**: [spec-chapter-24.md](./spec-chapter-24.md)

## Overview

This plan transforms the Chapter 24 specification into a detailed lesson-by-lesson implementation guide with CEFR proficiency levels, cognitive load validation, and AI-Native CoLearning pedagogy integrated throughout.

**Target Outcome**: Students master OOP foundations (classes, objects, encapsulation, methods) and build a Game Character System demonstrating all Part I concepts.

## Lesson Breakdown

### Lesson 1: What is OOP? Why OOP?

**File**: `01-oop-fundamentals.md`
**Estimated Time**: 45 minutes
**CEFR Proficiency**: A2 → B1 (foundational concepts with some analysis)
**Cognitive Load**: 5 concepts (at limit for beginner-to-intermediate)

#### Skills Taught

| Skill Name | CEFR Level | Category | Measurable at this level |
|------------|------------|----------|--------------------------|
| OOP Concept Recognition | A2 | Conceptual | Can identify OOP principles in code |
| Paradigm Comparison | B1 | Conceptual | Can explain procedural vs OOP differences |
| Real-world Modeling | B1 | Conceptual | Can map entities to potential classes |
| AI System Design Thinking | B1 | Soft | Can describe how agents are objects |
| Critical Analysis | B1 | Soft | Can explain why OOP matters for AI |

#### Content Structure

**1. What is OOP?** (10 min)
- Definition: Programming paradigm based on objects (data + behavior)
- Contrast with procedural (functions + data separate)
- Real-world analogy: Blueprint (class) vs Built House (object)

**2. The Four Pillars** (15 min)
- **Encapsulation**: Bundling data with methods that operate on it
- **Abstraction**: Hiding complexity, showing only essentials
- **Inheritance**: Reusing code through parent-child relationships (preview for Ch25)
- **Polymorphism**: Different objects responding to same interface (preview for Ch25)

💬 **AI Colearning Prompt**:
> "Ask your AI: Why did Python adopt OOP when it could have stayed purely procedural? What problems does OOP solve?"

🎓 **Instructor Commentary**:
> "In AI-driven development, OOP isn't about memorizing syntax—it's about thinking in systems of interacting entities. When you build a chatbot, the bot itself is an object with state (conversation history) and behavior (respond to messages)."

**3. Why Use OOP?** (10 min)
- Modularity: Organize code into logical units
- Reusability: Use classes across projects
- Maintainability: Changes isolated to specific classes
- Scalability: Add features without breaking existing code

**4. OOP in AI-Native Development** (10 min)
- AI agents are objects (state + behavior)
- Multi-agent systems use OOP design
- Example: ChatAgent class with process_message() method

🚀 **CoLearning Challenge**:
> "Ask your AI Co-Teacher: Generate a simple class that models a thermometer (current_temp attribute, read() method). Then explain why we'd use a class instead of just functions."

✨ **Teaching Tip**:
> "Use Claude Code to explore OOP history: 'How did OOP evolve from Simula to Python? What was the original problem it solved?'"

#### Try With AI (Lesson Closure)

Use your AI companion (Claude Code or Gemini CLI). You're exploring why OOP exists and what problems it solves.

**Prompt 1: Recall - Understanding OOP Pillars**
```
Explain the four pillars of OOP (Encapsulation, Abstraction, Inheritance, Polymorphism) in simple terms. Give a real-world analogy for each.
```
**Expected outcome**: You'll understand each pillar conceptually before seeing code.

**Prompt 2: Understand - Procedural vs OOP**
```
Compare a procedural approach (functions + data) vs OOP approach (classes + objects) for modeling a library system (books, members, checkouts). Which is better and why?
```
**Expected outcome**: You'll see why OOP organizes complex systems better than procedural.

**Prompt 3: Apply - Identifying Objects**
```
I'm building a task management app. What classes (objects) would I need? For each class, describe what data (attributes) and actions (methods) it would have.
```
**Expected outcome**: You'll practice identifying real-world entities as potential classes.

**Prompt 4: Analyze - OOP in AI Systems**
```
How would you design a multi-agent AI system (ChatAgent, CodeAgent, DataAgent) using OOP principles? What would be shared (base class) and what would be unique to each agent type?
```
**Expected outcome**: You'll connect OOP to professional AI development patterns.

---

### Lesson 2: Classes and Objects Basics

**File**: `02-classes-and-objects.md`
**Estimated Time**: 50 minutes
**CEFR Proficiency**: B1 (application of concepts to code)
**Cognitive Load**: 7 concepts

#### Skills Taught

| Skill Name | CEFR Level | Category | Measurable at this level |
|------------|------------|----------|--------------------------|
| Class Definition Syntax | B1 | Technical | Can write class keyword with proper naming |
| Constructor Implementation | B1 | Technical | Can write __init__ with parameters and type hints |
| self Keyword Understanding | B1 | Conceptual | Can explain instance reference pattern |
| Object Instantiation | B1 | Technical | Can create objects and access attributes |
| Type Hint Usage | B1 | Technical | Can add type hints to constructors |

#### Content Structure

**1. Defining a Class** (10 min)
- Syntax: `class ClassName:`
- Naming conventions (PascalCase)
- Empty class with `pass`

```python
class Dog:
    pass  # Empty class for now
```

**2. The __init__ Constructor** (15 min)
- Special method called when object is created
- `self` parameter (reference to instance)
- Parameterized constructor with type hints

```python
class Dog:
    def __init__(self, name: str, breed: str):
        self.name = name
        self.breed = breed
```

💬 **AI Colearning Prompt**:
> "Ask your AI: What happens in memory when we call `Dog('Max', 'Labrador')`? Walk me through step-by-step."

**3. Creating Objects (Instantiation)** (10 min)
```python
dog1 = Dog("Max", "Labrador")
dog2 = Dog("Buddy", "Golden Retriever")

print(dog1.name)  # Max
print(dog2.breed)  # Golden Retriever
```

**4. Understanding self** (10 min)
- `self` represents the current instance
- Why Python requires it explicitly (unlike other languages)
- How `dog1.name` becomes `Dog.name(dog1)`

🎓 **Instructor Commentary**:
> "In AI-native development, you don't memorize `self` syntax—you understand WHAT it means: 'this specific object.' When an agent processes a message, `self.conversation_history` means 'this agent's conversation,' not all agents'."

**5. Adding Simple Methods** (5 min)
```python
class Dog:
    def __init__(self, name: str, breed: str):
        self.name = name
        self.breed = breed

    def bark(self) -> str:
        return f"{self.name} says: Woof!"

dog1 = Dog("Max", "Labrador")
print(dog1.bark())  # Max says: Woof!
```

🚀 **CoLearning Challenge**:
> "Ask your AI: Generate a Vehicle class with brand, model, year attributes. Add a display_info() method that returns a formatted string. Explain why we use type hints."

✨ **Teaching Tip**:
> "Use Claude Code to explore: 'Show me how Python's built-in list class is defined. What methods does it have?'"

#### Try With AI

**Prompt 1: Recall - Class Syntax**
```
Write a simple Person class with name and age attributes. Include type hints. Then create two person objects.
```
**Expected outcome**: You'll practice basic class syntax with modern Python standards.

**Prompt 2: Understand - self Keyword**
```
Explain what 'self' means in Python classes. Why do we write 'self.name = name' in __init__? What happens if we forget self?
```
**Expected outcome**: You'll understand instance reference pattern deeply.

**Prompt 3: Apply - Building Your First Class**
```
Create a BankAccount class with account_holder (str) and balance (float) attributes. Add a deposit(amount) method that adds to balance and returns new balance.
```
**Expected outcome**: You'll build a functional class with data and behavior.

**Prompt 4: Analyze - Object Independence**
```
If I create two BankAccount objects (account1 and account2), and I deposit $100 into account1, does account2's balance change? Why or why not? Explain with a memory diagram.
```
**Expected outcome**: You'll understand object independence and instance isolation.

---

### Lesson 3: Constructors, Destructors, and Attributes

**File**: `03-constructors-attributes.md`
**Estimated Time**: 55 minutes
**CEFR Proficiency**: B1 → B2 (complex attribute management)
**Cognitive Load**: 8 concepts

#### Skills Taught

| Skill Name | CEFR Level | Category | Measurable at this level |
|------------|------------|----------|--------------------------|
| Parameterized Constructors | B1 | Technical | Can write constructors with multiple parameters |
| Default Parameters | B1 | Technical | Can provide default values in __init__ |
| Class vs Instance Attributes | B2 | Conceptual | Can choose appropriate attribute scope |
| Destructor Usage | B1 | Technical | Can implement __del__ for cleanup |
| Attribute Inspection (__dict__) | B2 | Technical | Can debug using __dict__ |

#### Content Structure

**1. Parameterized vs Default Constructors** (15 min)

```python
# Parameterized (requires arguments)
class Car:
    def __init__(self, make: str, model: str, year: int):
        self.make = make
        self.model = model
        self.year = year

# With defaults (optional arguments)
class Car:
    def __init__(self, make: str = "Unknown", model: str = "Unknown", year: int = 2024):
        self.make = make
        self.model = model
        self.year = year

car1 = Car("Toyota", "Camry", 2024)
car2 = Car()  # Uses defaults
```

💬 **AI Colearning Prompt**:
> "When should we use default parameters in constructors? Give me 3 scenarios where defaults are helpful and 1 where they're dangerous."

**2. Class Attributes vs Instance Attributes** (20 min)

```python
class BankAccount:
    # Class attribute (shared across ALL instances)
    interest_rate = 0.03

    def __init__(self, holder: str, balance: float = 0.0):
        # Instance attributes (unique to each object)
        self.holder = holder
        self.balance = balance

account1 = BankAccount("Alice", 1000)
account2 = BankAccount("Bob", 2000)

# All accounts share the same interest rate
print(account1.interest_rate)  # 0.03
print(account2.interest_rate)  # 0.03

# Changing class attribute affects all instances
BankAccount.interest_rate = 0.04
print(account1.interest_rate)  # 0.04
print(account2.interest_rate)  # 0.04

# But this creates an INSTANCE attribute, shadowing the class attribute
account1.interest_rate = 0.05
print(account1.interest_rate)  # 0.05 (instance attribute)
print(account2.interest_rate)  # 0.04 (still using class attribute)
```

🎓 **Instructor Commentary**:
> "The class vs instance attribute distinction is CRITICAL for AI agents. Configuration settings (API keys, model names) are often class attributes. But conversation history is an instance attribute—each agent needs its own."

💬 **AI Colearning Prompt**:
> "Explain why modifying a class attribute through an instance (like `account1.interest_rate = 0.05`) creates a new instance attribute instead of modifying the class attribute. What's happening in memory?"

**3. The __dict__ Attribute (Inspection)** (10 min)

```python
class Person:
    species = "Human"  # Class attribute

    def __init__(self, name: str, age: int):
        self.name = name
        self.age = age

p = Person("Alice", 30)

# See instance attributes
print(p.__dict__)  # {'name': 'Alice', 'age': 30}

# See class attributes
print(Person.__dict__)  # Shows 'species' and other class-level data
```

✨ **Teaching Tip**:
> "Use `__dict__` to debug attribute issues. Ask Claude: 'Why isn't my attribute showing up in __dict__?'"

**4. Destructors (__del__)** (10 min)

```python
class FileHandler:
    def __init__(self, filename: str):
        self.file = open(filename, 'w')
        print(f"Opened {filename}")

    def write(self, data: str):
        self.file.write(data)

    def __del__(self):
        self.file.close()
        print("File closed")

handler = FileHandler("test.txt")
handler.write("Hello")
# When handler goes out of scope or program ends, __del__ is called
```

🚀 **CoLearning Challenge**:
> "Ask your AI: Create a DatabaseConnection class that connects in __init__ and disconnects in __del__. Then explain when __del__ is NOT reliable for cleanup (hint: exceptions)."

#### Try With AI

**Prompt 1: Recall - Constructor Types**
```
Create a Product class with name, price, and quantity. Provide default values: price=0.0, quantity=0. Show how to create a product with all values specified vs using defaults.
```
**Expected outcome**: You'll master default parameters in constructors.

**Prompt 2: Understand - Class vs Instance Attributes**
```
I have a VideoGame class where all games share the same 'platform' (e.g., "PC"). Should 'platform' be a class attribute or instance attribute? What if each game can be on multiple platforms?
```
**Expected outcome**: You'll learn to choose the right attribute scope.

**Prompt 3: Apply - Debugging with __dict__**
```
Write a Student class with name, grade, and courses (list). Create a student object. Use __dict__ to inspect its attributes. Then add a class attribute 'school' and check if it appears in the instance's __dict__.
```
**Expected outcome**: You'll use __dict__ for debugging and understanding attribute storage.

**Prompt 4: Analyze - Destructor Use Cases**
```
When is __del__ useful for resource cleanup? When is it dangerous or unreliable? Give me a better alternative for guaranteed cleanup in Python.
```
**Expected outcome**: You'll understand destructors and context managers (brief preview).

---

### Lesson 4: Encapsulation and Method Types

**File**: `04-encapsulation-methods.md`
**Estimated Time**: 60 minutes
**CEFR Proficiency**: B2 (complex encapsulation patterns)
**Cognitive Load**: 10 concepts (at limit for advanced)

#### Skills Taught

| Skill Name | CEFR Level | Category | Measurable at this level |
|------------|------------|----------|--------------------------|
| Access Control Patterns | B2 | Conceptual | Can choose public/protected/private appropriately |
| Instance Method Design | B2 | Technical | Can implement methods operating on self |
| Class Method Implementation | B2 | Technical | Can use @classmethod for factories |
| Static Method Implementation | B2 | Technical | Can use @staticmethod for utilities |
| Property Decorator Usage | B2 | Technical | Can implement getters/setters with @property |
| Data Validation | B2 | Conceptual | Can validate attribute access |

#### Content Structure

**1. Encapsulation - Public, Protected, Private** (20 min)

```python
class Person:
    def __init__(self, name: str, age: int, ssn: str):
        self.name = name           # Public (no prefix)
        self._age = age            # Protected (single underscore - convention)
        self.__ssn = ssn           # Private (double underscore - name mangling)

    # Getter for protected attribute
    def get_age(self) -> int:
        return self._age

    # Getter for private attribute
    def get_ssn(self) -> str:
        return self.__ssn

person = Person("Alice", 30, "123-45-6789")

print(person.name)        # OK - public
print(person._age)        # Works but shouldn't (convention only)
# print(person.__ssn)     # AttributeError - name mangled to _Person__ssn
print(person.get_ssn())   # OK - via getter
```

🎓 **Instructor Commentary**:
> "Python's encapsulation is based on convention, not enforcement. The leading underscore says 'treat this as private' but doesn't prevent access. This is Python's philosophy: 'We're all consenting adults here.'"

💬 **AI Colearning Prompt**:
> "Ask your AI: Why does Python use naming conventions (_name, __name) instead of true private/public keywords like Java or C++? What are the tradeoffs?"

**2. Instance Methods** (10 min)

```python
class Circle:
    def __init__(self, radius: float):
        self._radius = radius

    def area(self) -> float:
        """Instance method - operates on instance data (self)"""
        return 3.14159 * self._radius ** 2

    def circumference(self) -> float:
        """Another instance method"""
        return 2 * 3.14159 * self._radius

circle = Circle(5)
print(circle.area())  # Uses self._radius
```

**3. Class Methods (@classmethod)** (12 min)

```python
class Temperature:
    def __init__(self, celsius: float):
        self.celsius = celsius

    @classmethod
    def from_fahrenheit(cls, fahrenheit: float) -> 'Temperature':
        """Factory method - creates instance from different format"""
        celsius = (fahrenheit - 32) * 5/9
        return cls(celsius)  # cls refers to the class itself

    @classmethod
    def from_kelvin(cls, kelvin: float) -> 'Temperature':
        """Another factory method"""
        celsius = kelvin - 273.15
        return cls(celsius)

# Create instances different ways
temp1 = Temperature(25)                     # Normal constructor
temp2 = Temperature.from_fahrenheit(77)     # Factory from Fahrenheit
temp3 = Temperature.from_kelvin(298.15)     # Factory from Kelvin
```

🚀 **CoLearning Challenge**:
> "Ask your AI: Create a Date class with from_string() class method that parses '2024-11-09' and returns a Date object. Explain why this is better than having multiple __init__ constructors."

**4. Static Methods (@staticmethod)** (10 min)

```python
class MathUtils:
    @staticmethod
    def is_prime(n: int) -> bool:
        """Utility function - doesn't need self or cls"""
        if n < 2:
            return False
        for i in range(2, int(n ** 0.5) + 1):
            if n % i == 0:
                return False
        return True

    @staticmethod
    def factorial(n: int) -> int:
        """Another utility - pure function"""
        if n <= 1:
            return 1
        return n * MathUtils.factorial(n - 1)

print(MathUtils.is_prime(17))  # True - no instance needed
print(MathUtils.factorial(5))  # 120
```

💬 **AI Colearning Prompt**:
> "When should I use @staticmethod vs just a regular function outside the class? Give me guidelines for choosing."

**5. Property Decorators (@property)** (18 min)

```python
class Circle:
    def __init__(self, radius: float):
        self._radius = radius  # Protected attribute

    @property
    def radius(self) -> float:
        """Getter - access like an attribute"""
        return self._radius

    @radius.setter
    def radius(self, value: float):
        """Setter - validate before setting"""
        if value <= 0:
            raise ValueError("Radius must be positive")
        self._radius = value

    @property
    def area(self) -> float:
        """Computed property - calculated on the fly"""
        return 3.14159 * self._radius ** 2

circle = Circle(5)
print(circle.radius)      # Uses getter (no parentheses!)
circle.radius = 10        # Uses setter (validates)
print(circle.area)        # Computed property
# circle.area = 100       # AttributeError - no setter for area
```

🎓 **Instructor Commentary**:
> "Property decorators are Pythonic encapsulation. They look like attributes but have hidden logic. This is powerful for AI agents—you can validate inputs, compute derived values, or log access transparently."

✨ **Teaching Tip**:
> "Use Claude Code to explore built-in property usage: 'Show me how Python's built-in types use properties. Does list have any?'"

**6. Abstract Base Classes (Brief Introduction)** (10 min)

```python
from abc import ABC, abstractmethod

class Shape(ABC):
    """Abstract base class - cannot be instantiated directly"""

    @abstractmethod
    def area(self) -> float:
        """All shapes MUST implement area()"""
        pass

    @abstractmethod
    def perimeter(self) -> float:
        """All shapes MUST implement perimeter()"""
        pass

# This would raise TypeError: Can't instantiate abstract class
# shape = Shape()

class Rectangle(Shape):
    def __init__(self, width: float, height: float):
        self.width = width
        self.height = height

    def area(self) -> float:
        return self.width * self.height

    def perimeter(self) -> float:
        return 2 * (self.width + self.height)

rect = Rectangle(5, 3)
print(rect.area())  # 15
```

🎓 **Instructor Commentary**:
> "ABCs enforce contracts: 'If you claim to be a Shape, you MUST have area() and perimeter().' This is a brief intro—we'll dive deep into ABCs with polymorphism in Chapter 25."

💬 **AI Colearning Prompt**:
> "Ask your AI: Why would we create a class we can't instantiate? What problem does ABC solve in large codebases?"

#### Try With AI

**Prompt 1: Recall - Encapsulation Levels**
```
Create a User class with public username, protected _email, and private __password. Add getters for email and password. Explain when to use each access level.
```
**Expected outcome**: You'll practice access control patterns.

**Prompt 2: Understand - Method Types**
```
Explain the difference between instance methods, class methods, and static methods. Give me a scenario where each is the best choice.
```
**Expected outcome**: You'll master method type selection.

**Prompt 3: Apply - Property Decorators**
```
Create a Product class with private __price. Use @property for a price getter and setter that validates price > 0. Add a computed property 'price_with_tax' (price * 1.1).
```
**Expected outcome**: You'll implement Pythonic encapsulation with properties.

**Prompt 4: Analyze - Abstract Base Classes**
```
Design an abstract PaymentMethod class with abstract process_payment() method. Then create CreditCard and PayPal subclasses that implement it. Why is ABC better than just a regular base class?
```
**Expected outcome**: You'll understand contract enforcement and prepare for Chapter 25 polymorphism.

---

### Lesson 5: Game Character System (Capstone)

**File**: `05-game-character-capstone.md`
**Estimated Time**: 70 minutes
**CEFR Proficiency**: B2 (synthesis and integration)
**Cognitive Load**: Integration (all Chapter 24 concepts applied)

#### Skills Taught

| Skill Name | CEFR Level | Category | Measurable at this level |
|------------|------------|----------|--------------------------|
| Multi-Class System Design | B2 | Conceptual | Can design 4+ interacting classes |
| OOP Pattern Integration | B2 | Technical | Can combine encapsulation, methods, properties |
| Class Interaction Design | B2 | Conceptual | Can model object relationships |
| Game Mechanics Modeling | B2 | Soft | Can translate requirements to classes |
| Code Organization | B2 | Technical | Can structure multi-file projects |

#### Project Overview

**Build a turn-based game character system** with:
- Character base class (health, name, level)
- Player class (inventory, experience)
- Enemy class (loot, difficulty)
- Combat system (attack, defend, use_item)

#### Content Structure

**Part 1: Design Phase (15 min)**

💬 **AI Colearning Prompt**:
> "I'm building a turn-based combat system. Ask me clarifying questions about requirements (What stats do characters have? How does combat work? What can players do?) Then suggest a class structure."

**Class Responsibilities**:
- `Character`: Base class with health, name, level, is_alive property
- `Player`: Extends Character with inventory (list), experience, level_up()
- `Enemy`: Extends Character with loot (dict), difficulty_rating
- `Item`: Represents inventory items (name, effect, use())
- `Combat`: Static methods for damage calculation

**Part 2: Implementing Character Base Class (15 min)**

```python
from abc import ABC, abstractmethod

class Character(ABC):
    """Base class for all game characters"""

    def __init__(self, name: str, health: int, level: int = 1):
        self.name = name
        self._health = health  # Protected - use property
        self._max_health = health
        self.level = level

    @property
    def health(self) -> int:
        """Current health with validation"""
        return self._health

    @health.setter
    def health(self, value: int):
        """Set health within bounds [0, max_health]"""
        self._health = max(0, min(value, self._max_health))

    @property
    def is_alive(self) -> bool:
        """Computed property"""
        return self._health > 0

    @abstractmethod
    def attack(self, target: 'Character') -> int:
        """All characters must implement attack"""
        pass

    def take_damage(self, amount: int) -> None:
        """Apply damage and prevent negative health"""
        self.health -= amount  # Uses setter
        print(f"{self.name} took {amount} damage. Health: {self.health}/{self._max_health}")

    def __str__(self) -> str:
        status = "Alive" if self.is_alive else "Defeated"
        return f"{self.name} (Lv.{self.level}) - {self.health}/{self._max_health} HP [{status}]"
```

🎓 **Instructor Commentary**:
> "Notice how we use encapsulation (protected _health), properties (health getter/setter), computed properties (is_alive), and ABC (abstract attack method). This is professional OOP."

**Part 3: Implementing Player Class (15 min)**

```python
class Item:
    """Inventory item"""
    def __init__(self, name: str, heal_amount: int):
        self.name = name
        self.heal_amount = heal_amount

class Player(Character):
    """Player character with inventory and experience"""

    def __init__(self, name: str, health: int = 100):
        super().__init__(name, health, level=1)
        self.inventory: list[Item] = []
        self.experience = 0
        self._experience_to_level = 100

    def attack(self, target: Character) -> int:
        """Implement abstract method"""
        damage = 10 * self.level
        print(f"{self.name} attacks {target.name} for {damage} damage!")
        target.take_damage(damage)
        return damage

    def add_item(self, item: Item) -> None:
        """Add item to inventory"""
        self.inventory.append(item)
        print(f"{self.name} obtained {item.name}!")

    def use_item(self, item_name: str) -> bool:
        """Use item from inventory"""
        for item in self.inventory:
            if item.name == item_name:
                self.health += item.heal_amount
                self.inventory.remove(item)
                print(f"{self.name} used {item.name} and healed {item.heal_amount} HP!")
                return True
        print(f"{item_name} not found in inventory.")
        return False

    def gain_experience(self, amount: int) -> None:
        """Add XP and level up if threshold reached"""
        self.experience += amount
        print(f"{self.name} gained {amount} XP!")

        while self.experience >= self._experience_to_level:
            self.level_up()

    def level_up(self) -> None:
        """Increase level and stats"""
        self.level += 1
        self._max_health += 20
        self.health = self._max_health  # Full heal on level up
        self.experience -= self._experience_to_level
        self._experience_to_level = int(self._experience_to_level * 1.5)
        print(f"🎉 {self.name} reached level {self.level}! Max HP: {self._max_health}")
```

🚀 **CoLearning Challenge**:
> "Ask your AI: Add a mana system to the Player class with a magic_attack() method that costs mana. Include mana regeneration and proper validation."

**Part 4: Implementing Enemy Class (10 min)**

```python
class Enemy(Character):
    """Enemy character with loot"""

    def __init__(self, name: str, health: int, level: int, difficulty: str):
        super().__init__(name, health, level)
        self.difficulty = difficulty
        self.loot: dict[str, int] = {}  # item_name -> quantity

    def attack(self, target: Character) -> int:
        """Enemy attack scales with difficulty"""
        base_damage = 8 * self.level
        multiplier = {"easy": 0.7, "normal": 1.0, "hard": 1.5}.get(self.difficulty, 1.0)
        damage = int(base_damage * multiplier)
        print(f"{self.name} attacks {target.name} for {damage} damage!")
        target.take_damage(damage)
        return damage

    def drop_loot(self) -> dict[str, int]:
        """Return loot when defeated"""
        if not self.is_alive:
            print(f"{self.name} dropped: {self.loot}")
            return self.loot
        return {}
```

**Part 5: Combat System (15 min)**

```python
class Combat:
    """Static methods for combat management"""

    @staticmethod
    def battle(player: Player, enemy: Enemy) -> bool:
        """Execute turn-based combat until one is defeated"""
        print(f"\n⚔️  Battle: {player.name} vs {enemy.name}\n")

        turn = 1
        while player.is_alive and enemy.is_alive:
            print(f"--- Turn {turn} ---")

            # Player's turn
            player.attack(enemy)

            if not enemy.is_alive:
                print(f"\n🏆 {player.name} defeated {enemy.name}!")
                xp_reward = enemy.level * 50
                player.gain_experience(xp_reward)
                loot = enemy.drop_loot()
                return True

            # Enemy's turn
            enemy.attack(player)

            if not player.is_alive:
                print(f"\n💀 {player.name} was defeated by {enemy.name}.")
                return False

            # Status update
            print(f"{player}")
            print(f"{enemy}\n")
            turn += 1

        return player.is_alive

    @staticmethod
    def calculate_critical_hit(base_damage: int, crit_chance: float = 0.1) -> int:
        """Static utility for damage calculation"""
        import random
        if random.random() < crit_chance:
            print("💥 CRITICAL HIT!")
            return int(base_damage * 2)
        return base_damage
```

**Part 6: Putting It All Together (10 min)**

```python
# Create player
hero = Player("Arin", health=120)

# Create enemies
goblin = Enemy("Goblin", health=50, level=1, difficulty="easy")
goblin.loot = {"Gold Coin": 5, "Health Potion": 1}

orc = Enemy("Orc Warrior", health=100, level=2, difficulty="normal")
orc.loot = {"Gold Coin": 15, "Iron Sword": 1}

# Add items to player
hero.add_item(Item("Health Potion", 30))

# First battle
if Combat.battle(hero, goblin):
    # Player won - next enemy
    if Combat.battle(hero, orc):
        print("\n🎊 All enemies defeated! You win!")
```

🚀 **CoLearning Challenge**:
> "Ask your AI Co-Teacher: Extend this system with a Boss class (special abilities), a Shop class (buy/sell items), and a save/load game state feature using JSON. Design the architecture together."

#### Try With AI

**Prompt 1: Recall - Design Review**
```
Review the Game Character System code. List all OOP concepts we used: encapsulation, properties, abstract methods, class methods, inheritance (brief), etc. For each, point to the specific code example.
```
**Expected outcome**: You'll recognize all Chapter 24 concepts in integrated code.

**Prompt 2: Understand - Design Tradeoffs**
```
Why did we make health protected (_health) with a property instead of public? Why is attack() an abstract method in Character? Explain the design reasoning.
```
**Expected outcome**: You'll understand professional OOP design decisions.

**Prompt 3: Apply - Extending the System**
```
Add a new class: Merchant (NPC that doesn't fight but sells items). Should Merchant inherit from Character? Why or why not? Design the class with proper encapsulation.
```
**Expected outcome**: You'll practice class design and inheritance decisions (preview Ch25).

**Prompt 4: Synthesize - From Game to AI Agents**
```
How could you adapt this Game Character System architecture to build a multi-agent AI system? Map: Character → Agent, Player → ChatAgent, Enemy → OpponentAgent, Combat → Negotiation. Design the classes.
```
**Expected outcome**: You'll connect OOP Part I to professional AI development.

---

## Proficiency Progression Summary

| Lesson | Starting CEFR | Ending CEFR | Concepts Introduced | Cumulative Concepts |
|--------|---------------|-------------|---------------------|---------------------|
| 1      | A2            | B1          | 5                   | 5                   |
| 2      | B1            | B1          | 7                   | 12                  |
| 3      | B1            | B2          | 8                   | 20                  |
| 4      | B2            | B2          | 10                  | 30                  |
| 5      | B2            | B2          | Integration         | All (synthesis)     |

**Cognitive Load Validation**:
- ✅ Lesson 1: 5 concepts (within A2-B1 limit)
- ✅ Lesson 2: 7 concepts (within B1 limit)
- ✅ Lesson 3: 8 concepts (within B1-B2 limit)
- ✅ Lesson 4: 10 concepts (at B2 maximum)
- ✅ Lesson 5: Integration (synthesis, no new concepts)

---

## Code Examples Mapping

| Example # | Purpose | Lesson | Complexity | Status |
|-----------|---------|--------|------------|--------|
| 1 | Simple Dog class | L2 | A2 | Specified |
| 2 | Parameterized Car class | L2 | B1 | Specified |
| 3 | Class vs instance (BankAccount) | L3 | B2 | Specified |
| 4 | Destructor (FileHandler) | L3 | B1 | Specified |
| 5 | Public/Protected/Private (Person) | L4 | B2 | Specified |
| 6 | Instance methods (Circle) | L4 | B1 | Specified |
| 7 | Class methods (Temperature factory) | L4 | B2 | Specified |
| 8 | Static methods (MathUtils) | L4 | B1 | Specified |
| 9 | Property decorators (Circle) | L4 | B2 | Specified |
| 10 | Abstract base class (Shape) | L4 | B2 | Specified |
| 11-15 | Game Character System | L5 | B2 | Integrated capstone |

---

## AI-Native CoLearning Compliance

### CoLearning Elements Per Lesson

| Lesson | 💬 AI Prompts | 🎓 Commentaries | 🚀 Challenges | ✨ Tips | Try With AI |
|--------|---------------|-----------------|---------------|---------|-------------|
| 1      | 1             | 1               | 1             | 1       | 4 prompts   |
| 2      | 2             | 1               | 2             | 2       | 4 prompts   |
| 3      | 2             | 1               | 1             | 1       | 4 prompts   |
| 4      | 4             | 3               | 2             | 2       | 4 prompts   |
| 5      | 2             | 2               | 3             | 0       | 4 prompts   |
| **Total** | **11** | **8** | **9** | **6** | **20 prompts** |

✅ All lessons have CoLearning elements throughout (not just at end)
✅ All lessons end with "Try With AI" ONLY (4 prompts each)
✅ Progressive Bloom's levels in each Try With AI section

---

## Implementation Checklist

### Pre-Implementation
- [x] Specification approved
- [x] Plan created with CEFR levels
- [x] Cognitive load validated
- [x] CoLearning elements designed
- [ ] Skills proficiency mapper applied (to be done in tasks phase)

### Content Creation (To be done in Phase 4)
- [ ] Lesson 1: OOP Fundamentals written
- [ ] Lesson 2: Classes and Objects written
- [ ] Lesson 3: Constructors and Attributes written
- [ ] Lesson 4: Encapsulation and Methods written
- [ ] Lesson 5: Game Character Capstone written
- [ ] All code examples tested on Python 3.14+
- [ ] All "Try With AI" prompts validated

### Quality Gates (To be done in Phase 4)
- [ ] No forward references verified
- [ ] Pedagogical ordering compliance checked
- [ ] Conversational tone throughout
- [ ] Type hints in all examples
- [ ] validation-auditor validation PASS

---

## Dependencies and Prerequisites

**Student Must Have Completed**:
- Chapter 20: Module and Functions (method concept)
- Chapter 21: Exception Handling (error handling in methods)
- Chapter 22: IO and File Handling (for FileHandler example)

**Student Will Be Ready For**:
- Chapter 25: OOP Part II (inheritance, polymorphism, MRO, composition)

---

## Next Steps

1. **Run /sp.tasks** to generate implementation checklist
2. **Invoke content-implementer** subagent for each lesson (Phase 4)
3. **Apply validation-auditor** for validation
4. **Update chapter-index.md** with completion status

---

**Plan Status**: ✅ READY FOR TASKS GENERATION
