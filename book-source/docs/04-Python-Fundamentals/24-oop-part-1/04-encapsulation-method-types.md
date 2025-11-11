---
title: "Encapsulation and Method Types"
chapter: 24
lesson: 4
duration_minutes: 60

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Access Control Patterns"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can choose appropriate access levels (public/protected/private) for attributes in design scenarios"

  - name: "Instance Method Design"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement methods operating on instance state (self) for various use cases"

  - name: "Class Method Implementation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement @classmethod for factory patterns and class-level operations"

  - name: "Static Method Implementation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement @staticmethod for utility functions within class scope"

  - name: "Property Decorator Usage"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement getters, setters, and computed properties using @property decorator"

  - name: "Data Validation"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Apply"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can validate attribute access through properties and setters"

  - name: "Method Type Selection"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can choose between instance methods, class methods, and static methods based on context"

  - name: "Encapsulation Through Properties"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design encapsulation strategies using properties and naming conventions"

  - name: "Getter/Setter Validation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can implement validation logic in property setters"

  - name: "AI-Assisted API Design"
    proficiency_level: "B2"
    category: "Soft"
    bloom_level: "Analyze"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student can describe API design choices to AI and refine implementation together"

learning_objectives:
  - objective: "Evaluate access control patterns and choose appropriate visibility levels (public/protected/private) for attributes"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Design review exercise choosing access levels for real-world scenarios"

  - objective: "Implement all three method types (instance, class, static) and explain when each is appropriate"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Implementation exercise with all three method types in one class"

  - objective: "Apply property decorators to implement Pythonic getters, setters, and computed attributes"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Property implementation with validation and computed properties"

  - objective: "Design encapsulation strategies using properties for data protection and validation"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Design exercise implementing validated BankAccount class"

cognitive_load:
  new_concepts: 10
  assessment: "10 concepts (public/protected/private, instance methods, @classmethod, @staticmethod, @property, @setter, computed properties, data validation, encapsulation strategy, ABC contracts) at B2 maximum. At limit ✓"

differentiation:
  extension_for_advanced: "Research Python's @abstractmethod with @property. Design an abstract DataStore with property-based interface. Explore Protocol typing (PEP 544) as alternative to ABC for duck typing enforcement."
  remedial_for_struggling: "Focus on one method type at a time. Start with instance methods, then add class methods, then static methods. Practice property decorator on simple cases (age validation) before computed properties."

# Generation metadata
generated_by: "lesson-writer v1.0.0"
source_spec: "specs/020-oop-part-1-2/spec-chapter-24.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Encapsulation and Method Types

In Lessons 2 and 3, you learned how to create classes, initialize objects, and manage attributes. Now you'll discover how to protect those attributes and organize behavior using three different method types. This lesson brings everything together: strategic access control, multiple method types, and professional-grade encapsulation.

This is where your classes become truly professional. You'll learn to **describe your intent** (public data, protected configuration, private secrets) and let Python enforce those patterns through naming conventions and decorators.

## Encapsulation: Controlling Access to Attributes

When you build a class, not all attributes should be treated equally. Some are safe for users to access directly. Others need protection—they require validation or careful control.

### Public Attributes (No Prefix)

The simplest pattern: any attribute without a leading underscore is **public**. Users can read and modify it freely.

```python
class Person:
    def __init__(self, name: str, age: int, ssn: str):
        self.name = name           # Public - direct access encouraged
        self._age = age            # Protected - convention only
        self.__ssn = ssn           # Private - name mangled
```

Public attributes work fine for simple data that doesn't need validation:

```python
person = Person("Alice", 30, "123-45-6789")
print(person.name)  # Alice - direct access OK
person.name = "Bob"  # Direct modification OK
```

#### 💬 AI Colearning Prompt

> "Ask your AI: In a professional API, when should an attribute be public vs protected? Give me 3 criteria for deciding."

### Protected Attributes (_single_underscore)

A single leading underscore signals: "This is part of the internal implementation. You *can* access it, but you probably shouldn't."

It's a **convention**, not enforcement. Python doesn't prevent access—the underscore says "treat this as protected."

```python
class BankAccount:
    def __init__(self, holder: str, balance: float):
        self.holder = holder      # Public
        self._balance = balance   # Protected - implementation detail

    def get_balance(self) -> float:
        """Use this method instead of accessing _balance directly"""
        return self._balance
```

Why protect `_balance`? Because direct modification breaks accounting:

```python
account = BankAccount("Alice", 1000)
account._balance = 999999  # Oops! No validation, no audit trail
```

#### 🎓 Instructor Commentary

> In AI-native development, protected attributes are common in agents. You might have `_conversation_history` (users shouldn't modify it directly) but expose `get_recent_messages()` method that returns safe copies. Encapsulation builds trust.

### Private Attributes (__double_underscore)

A double leading underscore triggers **name mangling**. Python renames the attribute to `_ClassName__attribute`, making direct access impossible (but not completely preventing determined hackers).

```python
class Person:
    def __init__(self, name: str, ssn: str):
        self.name = name        # Public
        self.__ssn = ssn        # Private - name mangled to _Person__ssn

person = Person("Alice", "123-45-6789")
print(person.name)      # Alice - OK
# print(person.__ssn)   # AttributeError: no attribute '__ssn'
print(person._Person__ssn)  # Works, but ugly - don't do this!
```

Provide a getter for private attributes:

```python
class Person:
    def __init__(self, name: str, ssn: str):
        self.name = name
        self.__ssn = ssn

    def get_ssn(self) -> str:
        """Safe accessor for sensitive data"""
        return self.__ssn

person = Person("Alice", "123-45-6789")
print(person.get_ssn())  # 123-45-6789
```

#### 💬 AI Colearning Prompt

> "Ask your AI: Why does Python use naming conventions (_name, __name) instead of true private/public keywords like Java or C++? What are the tradeoffs?"

## Instance Methods: The Workhorses

Instance methods operate on **instance data** (accessed via `self`). Most methods you write are instance methods.

```python
class BankAccount:
    def __init__(self, holder: str, balance: float):
        self._holder = holder
        self._balance = balance

    def deposit(self, amount: float) -> float:
        """Instance method - modifies this specific account"""
        if amount <= 0:
            raise ValueError("Deposit must be positive")
        self._balance += amount
        return self._balance

    def withdraw(self, amount: float) -> float:
        """Instance method - operates on self"""
        if amount > self._balance:
            raise ValueError("Insufficient funds")
        self._balance -= amount
        return self._balance

# Each account has its own balance
account1 = BankAccount("Alice", 1000)
account2 = BankAccount("Bob", 500)

account1.deposit(100)   # Alice: 1100
account2.withdraw(50)   # Bob: 450 (different instance!)
```

**Rule**: If a method needs `self` to operate, it's an instance method. That's the default.

#### ✨ Teaching Tip

> "Instance methods are so common that when you write `def method(self, ...)`, Python automatically knows it's an instance method. The @classmethod and @staticmethod decorators are for the exceptions."

## Class Methods: Factories and Class Operations

Sometimes you need to operate on the **class itself**, not individual instances. That's where `@classmethod` comes in.

A class method receives `cls` (the class) instead of `self` (an instance). It can't access instance data, but it can access or modify class data.

### Factory Pattern with @classmethod

The most common use: create instances in different ways.

```python
class Temperature:
    """Temperature object storing celsius internally"""

    def __init__(self, celsius: float):
        self.celsius = celsius

    @classmethod
    def from_fahrenheit(cls, fahrenheit: float) -> 'Temperature':
        """Factory method: create from Fahrenheit"""
        celsius = (fahrenheit - 32) * 5/9
        return cls(celsius)  # cls refers to Temperature class

    @classmethod
    def from_kelvin(cls, kelvin: float) -> 'Temperature':
        """Factory method: create from Kelvin"""
        celsius = kelvin - 273.15
        return cls(celsius)

    def to_fahrenheit(self) -> float:
        """Instance method - uses self"""
        return (self.celsius * 9/5) + 32

# Create in multiple ways
temp1 = Temperature(25)                       # Direct
temp2 = Temperature.from_fahrenheit(77)       # From Fahrenheit
temp3 = Temperature.from_kelvin(298.15)       # From Kelvin

print(temp1.celsius)  # 25.0
print(temp2.celsius)  # 25.0 (same temperature, different input)
```

Why use `cls` instead of `Temperature`? Because subclasses can override it:

```python
class ExtendedTemperature(Temperature):
    """Subclass with additional features"""
    pass

# This automatically creates an ExtendedTemperature, not Temperature
temp = ExtendedTemperature.from_fahrenheit(77)  # Returns ExtendedTemperature instance
```

#### 🚀 CoLearning Challenge

> "Ask your AI Co-Teacher: Create a Date class with a from_string() class method that parses '2024-11-09' and returns a Date object. Then explain why this is better than having multiple __init__ constructors."

## Static Methods: Utility Functions

Static methods are functions that **belong to a class logically** but don't need `self` or `cls`. They're utilities grouped by topic.

```python
class MathUtils:
    @staticmethod
    def is_prime(n: int) -> bool:
        """Pure function - no self, no cls"""
        if n < 2:
            return False
        for i in range(2, int(n ** 0.5) + 1):
            if n % i == 0:
                return False
        return True

    @staticmethod
    def factorial(n: int) -> int:
        """Another pure utility function"""
        if n <= 1:
            return 1
        return n * MathUtils.factorial(n - 1)

# Call directly on class
print(MathUtils.is_prime(17))    # True
print(MathUtils.factorial(5))     # 120
```

Static methods don't access instance data, so they could be standalone functions. But grouping them in a class provides organization:

```python
# Good: Organized by domain
calculator.add(5, 3)

# Okay: Utility function alone
add(5, 3)

# Best: Organized in a class
MathUtils.add(5, 3)
```

#### 💬 AI Colearning Prompt

> "When should I use @staticmethod vs just a regular function outside the class? Give me guidelines for choosing."

#### 🚀 CoLearning Challenge

> "Ask your AI: Create a StringUtils class with @staticmethod methods: reverse(), is_palindrome(), word_count(). Then explain why these utilities live in a class rather than as standalone functions."

## Property Decorators: Pythonic Getters and Setters

Properties let you hide implementation details while looking like simple attributes.

### @property: The Getter

A property getter lets you read an attribute through a method, but the code looks like attribute access:

```python
class Circle:
    def __init__(self, radius: float):
        self._radius = radius  # Protected - use property to access

    @property
    def radius(self) -> float:
        """Property getter - looks like attribute, works like method"""
        return self._radius

circle = Circle(5)
print(circle.radius)  # Looks like attribute access, calls method!
```

This is cleaner than `circle.get_radius()`. Properties make your class API simpler.

### @property.setter: The Setter with Validation

A setter lets you validate data before accepting it:

```python
class Circle:
    def __init__(self, radius: float):
        self._radius = 0
        self.radius = radius  # Uses setter for validation

    @property
    def radius(self) -> float:
        """Getter"""
        return self._radius

    @radius.setter
    def radius(self, value: float):
        """Setter with validation"""
        if value <= 0:
            raise ValueError("Radius must be positive")
        self._radius = value

circle = Circle(5)
print(circle.radius)  # 5

circle.radius = 10    # Validates and accepts
circle.radius = -5    # ValueError: Radius must be positive
```

#### 🎓 Instructor Commentary

> "Property decorators are Pythonic encapsulation. They look like attributes but have hidden logic. This is powerful for AI agents—you can validate inputs, log access, compute derived values, all transparently."

### Computed Properties: No Backing Attribute

A property doesn't need a backing attribute. It can compute values on the fly:

```python
class Circle:
    def __init__(self, radius: float):
        self._radius = radius

    @property
    def radius(self) -> float:
        return self._radius

    @property
    def area(self) -> float:
        """Computed property - calculated, not stored"""
        return 3.14159 * self._radius ** 2

    @property
    def circumference(self) -> float:
        """Another computed property"""
        return 2 * 3.14159 * self._radius

circle = Circle(5)
print(circle.area)           # 78.53975 - computed
print(circle.circumference)  # 31.4159 - computed

# These look like attributes but are calculated!
```

Computed properties are perfect for **derived data**. Users don't care if it's stored or calculated—they just want the value.

#### ✨ Teaching Tip

> "Use Claude Code to explore built-in property usage: 'Show me how Python's built-in types use properties. Does the `list` class have any computed properties?'"

## Choosing the Right Method Type

By now you've seen three method types. When do you use each?

### Decision Matrix

| Needs `self`? | Needs `cls`? | Type | Use Case | Example |
|---|---|---|---|---|
| Yes | No | Instance | Operate on object state | `account.deposit(100)` |
| No | Yes | Class | Create instances or modify class data | `Temperature.from_fahrenheit(77)` |
| No | No | Static | Pure utility functions | `MathUtils.is_prime(17)` |

**Real-world example**: A User authentication system

```python
class User:
    # Class data - shared across all users
    total_users = 0
    admin_list: list[str] = []

    def __init__(self, username: str, password: str):
        self.username = username
        self._password = password
        User.total_users += 1

    def login(self, password_attempt: str) -> bool:
        """Instance method - checks this user's password"""
        return password_attempt == self._password

    @classmethod
    def from_admin(cls, username: str, admin_password: str) -> 'User':
        """Factory - only admins can create new admins"""
        # Verify admin_password, then create
        user = cls(username, "admin_secret_" + admin_password)
        cls.admin_list.append(username)
        return user

    @staticmethod
    def validate_username(username: str) -> bool:
        """Pure utility - validates format"""
        return len(username) > 3 and username.isalnum()

    @staticmethod
    def hash_password(password: str) -> str:
        """Pure utility - hashes password"""
        import hashlib
        return hashlib.md5(password.encode()).hexdigest()

# Usage
if User.validate_username("alice"):           # Static method
    user1 = User("alice", User.hash_password("secret"))  # Instance + static
    user2 = User.from_admin("admin", "admin_key")  # Class method

print(User.total_users)  # 2 - accessed through class
```

## Abstract Base Classes (Brief Introduction)

You briefly saw ABC in Lesson 2. Now let's understand why they exist.

An **Abstract Base Class** (ABC) enforces a **contract**: if you inherit from this class, you MUST implement certain methods.

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

# This fails:
# shape = Shape()  # TypeError: Can't instantiate abstract class

# But this works - Rectangle implements the contract:
class Rectangle(Shape):
    def __init__(self, width: float, height: float):
        self.width = width
        self.height = height

    def area(self) -> float:
        return self.width * self.height

    def perimeter(self) -> float:
        return 2 * (self.width + self.height)

# Now we can create instances
rect = Rectangle(5, 3)
print(rect.area())      # 15
print(rect.perimeter()) # 16

# This fails - no area() implementation:
# class Triangle(Shape):
#     def perimeter(self) -> float:
#         pass
# TypeError: Can't instantiate abstract class Triangle with abstract method area
```

#### 🎓 Instructor Commentary

> "ABCs enforce contracts: 'If you claim to be a Shape, you MUST have area() and perimeter().' This is a brief introduction—we'll dive deep into ABCs with polymorphism and inheritance in Chapter 25."

#### 💬 AI Colearning Prompt

> "Ask your AI: Why would we create a class we can't instantiate? What problem does ABC solve in large codebases?"

---

## Try With AI

Use your AI companion (Claude Code, Gemini CLI, or ChatGPT web). You're mastering encapsulation strategies and method selection.

**Prompt 1: Recall - Encapsulation Levels**

```
Create a User class with:
- public username
- protected _email
- private __password

Add a get_password() method that returns the password.

Explain when to use each access level and why we provide a getter for private data.
```

**Expected outcome**: You'll practice access control patterns and understand the philosophy behind each level.

**Prompt 2: Understand - Method Types**

```
Explain the difference between instance methods, class methods, and static methods.
For each, give a scenario where it's the best choice.

Then explain: Why can't a static method access self?
```

**Expected outcome**: You'll distinguish between method types and understand their constraints and use cases.

**Prompt 3: Apply - Property Decorators**

```
Create a Product class with:
- private __price
- @property getter for price
- @property.setter that validates price > 0
- computed property 'price_with_tax' (price * 1.1)

Show how a user would access these as if they were attributes.
```

**Expected outcome**: You'll implement Pythonic encapsulation with properties and validation.

**Prompt 4: Analyze - Design Review**

```
I'm designing a BankAccount class. Here are my choices:

1. public balance (users can modify it directly)
2. protected _balance with deposit()/withdraw() methods
3. private __balance with @property (getter/setter with validation)

Analyze each approach: What breaks? What's safe? Which is most professional?
Then design a BankAccount using properties with validation.
```

**Expected outcome**: You'll understand professional encapsulation design and prepare to build the Game Character System in Lesson 5.
