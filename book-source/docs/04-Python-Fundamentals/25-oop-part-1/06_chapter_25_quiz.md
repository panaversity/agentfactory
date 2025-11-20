---
sidebar_position: 6
title: "Chapter 25: Object-Oriented Programming - Part 1 Quiz"
---

# Chapter 25: Object-Oriented Programming - Part 1 Quiz

Test your understanding of object-oriented programming fundamentals, including classes, objects, constructors, encapsulation, and method types.

<Quiz
  title="Chapter 25: Object-Oriented Programming - Part 1 Assessment"
  questions={[
    {
      question: "You're building a banking system and need 1,000 account objects. What's the core advantage of OOP over procedural programming for this scenario?",
      options: [
        "Define account logic once, create 1,000 independent instances automatically",
        "OOP runs faster than procedural code for large datasets",
        "Classes use less memory than separate variables and functions",
        "OOP prevents bugs that procedural programming cannot catch"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. OOP's core advantage is defining the blueprint (class) once and creating as many independent instances as needed. With procedural code, you'd need 1,000 sets of variables and duplicate functions. OOP doesn't inherently run faster (option 2), use less memory (option 3), or prevent all bugs (option 4)—its power lies in code reusability and organization through the class-instance pattern. When describing this to AI, you'd say 'create a BankAccount class with balance and deposit/withdraw methods,' and AI knows one definition serves infinite instances. This is covered in Lesson 1: What is OOP?",
      source: "Lesson 1: What is OOP? Why OOP?"
    },
    {
      question: "What problem does Object-Oriented Programming primarily solve compared to procedural programming?",
      options: [
        "Scaling and organizing code for multiple similar entities",
        "Making programs run significantly faster through optimization",
        "Preventing all possible runtime errors through type safety",
        "Reducing the total lines of code needed"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. OOP solves the scaling and organization problem—when you have many entities (users, products, agents) with similar data and behavior, OOP lets you define structure once and create unlimited instances. Option 2 is wrong (OOP isn't primarily about speed). Option 3 is wrong (OOP doesn't prevent all errors). Option 4 is misleading (OOP can actually increase code for simple tasks). The real power emerges when managing multiple objects: with procedural code, 100 entities = 200+ variables and duplicate functions; with OOP, it's 1 class definition and 100 instances. Lesson 1 demonstrates this through the banking system example.",
      source: "Lesson 1: What is OOP? Why OOP?"
    },
    {
      question: "Which scenario makes OOP the right architectural choice over procedural programming?",
      options: [
        "You have 5+ entities with similar data structure and operations",
        "Your script calculates a mathematical formula with 3 variables",
        "You're writing a one-time data migration script",
        "The program needs to run as fast as possible"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. OOP becomes valuable when you have multiple similar entities that share structure and behavior. This is the decision framework from Lesson 1: if you're managing many instances of the same concept, use OOP. Option 2 (math formula) works fine procedurally. Option 3 (one-time script) doesn't benefit from OOP's reusability. Option 4 (performance) is wrong—OOP vs procedural doesn't inherently determine speed. When working with AI, describing 'multiple similar objects' signals that AI should suggest class-based architecture. The decision tree in Lesson 1 guides this: similar entities + repeated operations + growth potential = use OOP.",
      source: "Lesson 1: What is OOP? Why OOP?"
    },
    {
      question: "In OOP, what are the 'four pillars' that define the paradigm's core principles?",
      options: [
        "Encapsulation, Abstraction, Inheritance, Polymorphism",
        "Classes, Objects, Methods, Attributes",
        "Constructors, Destructors, Getters, Setters",
        "Public, Private, Protected, Static"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. The four pillars of OOP are Encapsulation (bundling data and methods), Abstraction (hiding implementation details), Inheritance (reusing code through parent-child relationships), and Polymorphism (different objects responding to same interface). Option 2 lists implementation concepts, not principles. Option 3 lists specific methods. Option 4 lists access modifiers. These four pillars represent design philosophies: Encapsulation protects data, Abstraction simplifies interfaces, Inheritance promotes reuse, Polymorphism enables flexibility. Understanding these helps you describe design intent to AI: 'encapsulate user data' or 'inherit from base agent class.' Lesson 1 introduces these conceptually before diving into implementation in later lessons.",
      source: "Lesson 1: What is OOP? Why OOP?"
    },
    {
      question: "You asked AI to 'create a User class.' What does AI understand this request to mean?",
      options: [
        "Create a blueprint that can produce multiple user objects",
        "Create one user object with specific data",
        "Generate procedural functions for user operations",
        "Create a database schema for user storage"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. When you say 'create a User class,' AI understands you want a blueprint (template) for creating user objects, not a single user instance. The class defines structure (attributes like name, email) and behavior (methods like login, logout). Option 2 confuses class (blueprint) with instance (object). Option 3 would be procedural, not OOP. Option 4 is database design, different domain. This distinction is critical: 'create a class' = define template; 'create an object' = make instance from template. Lesson 2 emphasizes that the class is the cookie cutter, objects are the cookies. Understanding this helps you give precise instructions to AI.",
      source: "Lesson 2: Classes and Objects Basics"
    },
    {
      question: "What is the relationship between a class and an object in Python?",
      options: [
        "Class is the blueprint; object is instance created from it",
        "Class contains objects; objects contain methods",
        "Object is the template; class is the implementation",
        "Class and object are two names for same thing"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. The class is the blueprint or template that defines structure and behavior; the object is a specific instance created from that blueprint. Think: architectural plans (class) vs. actual building (object). Option 2 is backwards. Option 3 reverses the relationship. Option 4 is wrong—they're distinct concepts. You can create unlimited objects from one class, each with independent data: `dog1 = Dog('Max')`, `dog2 = Dog('Buddy')` creates two objects from the Dog class. When collaborating with AI, saying 'create the Dog class' means define the template, while 'create a dog object' means instantiate from that template. Lesson 2 demonstrates this with the cookie cutter analogy.",
      source: "Lesson 2: Classes and Objects Basics"
    },
    {
      question: "What does the `self` parameter represent in a Python instance method?",
      options: [
        "The specific object instance calling the method",
        "The class definition itself, not any instance",
        "A global variable accessible across all methods",
        "The parent class from which this inherits"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. `self` represents 'this specific object'—the instance that called the method. When `dog1.bark()` is called, inside the bark() method, `self` refers to dog1. This is how methods operate on instance-specific data: `self.name` accesses that particular dog's name. Option 2 describes `cls` in class methods. Option 3 is wrong (self is not global). Option 4 describes inheritance, not self. Python requires explicit `self` so you clearly see when you're accessing instance data (`self.balance`) vs. local variables (`amount`). When reviewing AI-generated code, verify that instance methods have `self` as first parameter and use `self.attribute` to access object state. Lesson 2 covers this thoroughly.",
      source: "Lesson 2: Classes and Objects Basics"
    },
    {
      question: "You create two objects: `alice = BankAccount('Alice', 1000)` and `bob = BankAccount('Bob', 500)`. After calling `alice.deposit(200)`, what is bob's balance?",
      options: [
        "500 (unchanged, objects have independent state)",
        "700 (both balances increased by 200)",
        "1200 (bob's balance matches alice's new balance)",
        "Error: cannot modify one object without affecting others"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Each object maintains independent state in its own memory space. When you call `alice.deposit(200)`, only alice's balance changes—bob's remains 500. This is the fundamental power of objects: data isolation. Option 2 suggests objects share state (they don't). Option 3 suggests state synchronization (doesn't happen automatically). Option 4 is completely wrong—object independence is core to OOP. This contrasts with global variables in procedural code, where modifications affect all functions. In AI collaboration, this independence means you can ask AI to 'create 1,000 accounts' and know each operates independently. Lesson 2 demonstrates this through experiments with multiple dog objects.",
      source: "Lesson 2: Classes and Objects Basics"
    },
    {
      question: "What is the purpose of the `__init__` method in a Python class?",
      options: [
        "Automatically initialize object attributes when instance is created",
        "Define which methods the class can access",
        "Specify the class inheritance hierarchy",
        "Delete the object when it goes out of scope"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. `__init__` is the constructor that Python calls automatically when you create an object. It initializes attributes with values: `def __init__(self, name, age): self.name = name; self.age = age`. This eliminates the repetition of manually setting attributes on every object. Option 2 is wrong (methods are defined separately). Option 3 is wrong (inheritance uses different syntax). Option 4 describes `__del__`, the destructor. Without `__init__`, you'd write: `dog = Dog(); dog.name = 'Max'; dog.breed = 'Lab'`—repetitive and error-prone. With `__init__`: `dog = Dog('Max', 'Lab')`—concise and enforced. When working with AI, you'd say 'the constructor should accept name and age as required parameters,' and AI generates the `__init__` method. Lesson 2 shows this evolution from manual attribute setting to constructor-based initialization.",
      source: "Lesson 2: Classes and Objects Basics"
    },
    {
      question: "Why does Python require the explicit `self` parameter in instance methods?",
      options: [
        "To clearly distinguish instance data from local variables",
        "Because Python is slower without explicit parameters",
        "To prevent methods from accessing class attributes",
        "Self is optional; it's just a naming convention"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Explicit `self` makes it obvious when you're accessing instance data (`self.balance`) vs. local variables (`amount` in a method). This clarity prevents bugs and makes code readable. Option 2 is wrong (no performance reason). Option 3 is backwards (self enables access, doesn't prevent it). Option 4 is dangerous misinformation—`self` is required, though the name could technically be different (but 'self' is the universal convention). When reviewing AI-generated code, if you see a method accessing attributes without `self.`, that's a bug. This explicit design philosophy appears throughout Python: 'explicit is better than implicit' (Zen of Python). Lesson 2 demonstrates how `self` enables object independence.",
      source: "Lesson 2: Classes and Objects Basics"
    },
    {
      question: "When should a constructor parameter have a default value instead of being required?",
      options: [
        "When the attribute represents optional or convenience data",
        "Always use defaults to make the API flexible",
        "Never use defaults; all data should be explicit",
        "Only when the class has exactly three parameters"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Use defaults for optional, non-critical data (like `age=0` for unknown age, or `role='user'` for typical role). Required parameters force callers to provide critical data (like `name`—every person must have one). Option 2 is wrong (too flexible can hide required data). Option 3 is too rigid (defaults improve usability). Option 4 is nonsense. Design principle: required data = no default; optional/convenience data = default value. When describing class design to AI: 'name and email are required, role defaults to user, created_at defaults to now.' This balances explicitness with usability. Lesson 3 covers default parameter patterns and when each approach fits.",
      source: "Lesson 3: Constructors, Destructors, and Attributes"
    },
    {
      question: "What's the critical difference between instance attributes and class attributes in memory?",
      options: [
        "Instance attributes: each object has own copy; class attributes: shared by all",
        "Instance attributes are faster because they're stored differently",
        "Class attributes can only store strings; instance attributes any type",
        "Instance attributes must be public; class attributes must be private"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Instance attributes (like `self.name`) are stored per-object—each instance has its own copy. Class attributes (like `species = 'Dog'` defined in class body) are shared across all instances—one copy in memory that all objects access. Option 2 is wrong (no speed difference in this context). Option 3 is false (both can store any type). Option 4 is false (access control is independent of attribute type). Use case: `Dog.species` should be class attribute (all dogs are 'Canis familiaris'), but `dog.name` should be instance attribute (each dog has unique name). When working with AI: 'make base_url a class attribute since all API clients share it, but make api_key instance attribute since each client has unique credentials.' Lesson 3 demonstrates this with experiments.",
      source: "Lesson 3: Constructors, Destructors, and Attributes"
    },
    {
      question: "You have `class Counter: count = 0`. Then `c1 = Counter(); c1.count = 5`. What happened?",
      options: [
        "Created instance attribute on c1 shadowing the class attribute",
        "Modified the class attribute; all Counter objects now see 5",
        "Raised AttributeError; cannot modify class attributes through instances",
        "Created a reference; c1.count and Counter.count are now linked"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Assigning `c1.count = 5` creates a new instance attribute on c1, shadowing the class attribute. Now `c1.count` returns 5 (instance attribute), but `c2.count` and `Counter.count` still return 0 (class attribute). This shadowing is often unintentional and confusing. Option 2 is wrong—to modify class attributes, use `Counter.count = 5`. Option 3 is wrong—Python allows this, which is part of the problem. Option 4 is wrong—no linking occurs. To detect shadowing: check `'count' in c1.__dict__` (True = instance attribute). When reviewing AI-generated code where objects need shared state, verify updates use `ClassName.attribute`, not `instance.attribute`. Lesson 3 demonstrates this trap and how to avoid it.",
      source: "Lesson 3: Constructors, Destructors, and Attributes"
    },
    {
      question: "Why is `def __init__(self, items: list = []):` considered dangerous?",
      options: [
        "Mutable default is shared across all instances, causing hidden bugs",
        "Lists are too large to use as default parameters",
        "The syntax is invalid; Python requires None for lists",
        "Empty lists cause the constructor to fail"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. When you use a mutable default like `[]` or `{}`, that same object is shared across all instances created without providing the parameter. So `obj1 = MyClass(); obj2 = MyClass(); obj1.items.append('x')` results in obj2.items also containing 'x'—unexpected sharing. Option 2 is false. Option 3 is wrong (syntax is valid, just problematic). Option 4 is false. Correct pattern: `def __init__(self, items: list = None): self.items = items if items is not None else []`. This creates a new list for each instance. When working with AI, if you see mutable defaults in generated code, request: 'use None as default and create new list inside __init__.' This is a classic Python gotcha covered in Lesson 3.",
      source: "Lesson 3: Constructors, Destructors, and Attributes"
    },
    {
      question: "You asked AI to create an APIClient class. It should use the same base_url for all clients but unique api_key per client. How should AI structure this?",
      options: [
        "base_url as class attribute; api_key as instance attribute in __init__",
        "Both base_url and api_key as instance attributes",
        "Both base_url and api_key as class attributes",
        "base_url as instance attribute; api_key as class attribute"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. `base_url` should be a class attribute (shared by all clients: `class APIClient: base_url = 'https://api.example.com'`). `api_key` should be instance attribute (unique per client: `def __init__(self, api_key): self.api_key = api_key`). This way, all clients hit the same API endpoint but authenticate with different keys. Option 2 wastes memory duplicating base_url. Option 3 makes all clients share the same api_key—security disaster. Option 4 is backwards. This demonstrates when shared state (class attribute) vs. independent state (instance attribute) is appropriate. Describing to AI: 'base_url is constant across all instances, api_key is unique per instance' signals this design. Lesson 3 covers choosing attribute scope based on data semantics.",
      source: "Lesson 3: Constructors, Destructors, and Attributes"
    },
    {
      question: "What is the primary purpose of encapsulation in object-oriented programming?",
      options: [
        "Bundle data and methods together; control access to protect data integrity",
        "Make code run faster by hiding implementation details",
        "Prevent other developers from reading the code",
        "Automatically generate getters and setters for all attributes"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Encapsulation means bundling data (attributes) and methods that operate on that data into a single unit (class), then controlling access to protect data integrity. For example, making `balance` private and only allowing changes through validated `deposit()`/`withdraw()` methods prevents invalid states like negative balances. Option 2 is wrong—encapsulation is about design, not performance. Option 3 is wrong—it's about API design and validation, not secrecy. Option 4 is wrong—getters/setters are one mechanism, not the goal. When working with AI: 'encapsulate balance with validation' tells AI to use properties or methods to protect that data. Lesson 4 demonstrates this with BankAccount example where direct balance access allows corruption, but encapsulated access enforces rules.",
      source: "Lesson 4: Encapsulation and Method Types"
    },
    {
      question: "In Python, what do naming conventions communicate about attribute access?",
      options: [
        "_single_leading: internal use; __double_leading: name mangling; no_leading: public",
        "Any attribute with underscore is completely inaccessible from outside",
        "Python enforces private attributes at runtime; external access raises errors",
        "Naming conventions are cosmetic; Python has true access modifiers like Java"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Python uses conventions: no leading underscore = public API (`self.name`); single leading underscore = internal implementation, don't depend on it (`self._internal`); double leading underscore = name mangling to avoid conflicts in inheritance (`self.__private`). These are signals, not enforced barriers—you can still access `obj._internal` or `obj._ClassName__private`. Option 2 is wrong—Python doesn't enforce true privacy. Option 3 is wrong—no runtime errors for accessing underscored attributes. Option 4 is backwards—Python relies on conventions, not enforced modifiers like Java's `private`. When reviewing AI-generated code, underscored attributes signal 'implementation detail, subject to change.' Lesson 4 covers Python's 'we're all consenting adults' philosophy.",
      source: "Lesson 4: Encapsulation and Method Types"
    },
    {
      question: "When should you use an instance method vs. a class method vs. a static method?",
      options: [
        "Instance: operates on object state; Class: operates on class state; Static: utility function",
        "Always use instance methods; class and static methods are rarely needed",
        "Instance: public API; Class: protected methods; Static: private methods",
        "Use whichever is fastest for the operation being performed"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Instance methods (def method(self):) operate on instance state using `self`. Class methods (@classmethod, def method(cls):) operate on class state or serve as alternative constructors. Static methods (@staticmethod, def method():) are utility functions logically grouped with the class but don't need instance or class data. Option 2 is wrong—class/static methods have specific use cases. Option 3 confuses method types with access control. Option 4 is wrong—performance isn't the deciding factor. Example: `account.deposit(100)` = instance method (modifies this account); `Date.from_string('2024-01-01')` = class method (creates instance); `Math.calculate_distance(p1, p2)` = static method (utility). When describing to AI: 'needs access to instance data' → instance method. Lesson 4 demonstrates all three types.",
      source: "Lesson 4: Encapsulation and Method Types"
    },
    {
      question: "What's the purpose of the @property decorator in Python?",
      options: [
        "Create methods that look like attributes but execute validation logic",
        "Make attributes load faster by caching their values",
        "Automatically generate __init__ parameters from attributes",
        "Prevent attributes from being modified after initialization"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. @property lets you write methods that are accessed like attributes: `user.age` instead of `user.get_age()`. Behind the scenes, your method runs, allowing validation or computation. Example: `@property def balance(self): return self._balance` makes `account.balance` call the method. You can add `@balance.setter` to validate assignments: `account.balance = -100` can be rejected. Option 2 is wrong (caching is @functools.lru_cache). Option 3 is wrong (dataclasses do this). Option 4 is wrong (that's `frozen` in dataclasses or using __setattr__). When working with AI: 'create a balance property that prevents negative values' signals using @property with validation in the setter. Lesson 4 covers properties as Pythonic encapsulation.",
      source: "Lesson 4: Encapsulation and Method Types"
    },
    {
      question: "You asked AI to create a Temperature class. It should allow `temp.celsius = 25` and automatically compute `temp.fahrenheit`. What should AI use?",
      options: [
        "@property decorator to make fahrenheit a computed attribute",
        "Store both celsius and fahrenheit in __init__",
        "Create get_fahrenheit() and set_fahrenheit() methods",
        "Use class attributes for the temperature values"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. You'd store only celsius as instance data, then use @property to compute fahrenheit on-the-fly: `@property def fahrenheit(self): return self.celsius * 9/5 + 32`. This ensures fahrenheit is always in sync with celsius—no duplicate state. Option 2 creates synchronization bugs (if celsius changes, fahrenheit is stale). Option 3 works but isn't Pythonic (prefer attribute access over method calls for simple access). Option 4 is wrong—temperatures are instance data. When describing to AI: 'celsius is stored attribute, fahrenheit is computed property' signals this pattern. This demonstrates encapsulation: internal representation (celsius) differs from external interface (both celsius and fahrenheit appear as attributes). Lesson 4 covers computed properties.",
      source: "Lesson 4: Encapsulation and Method Types"
    },
    {
      question: "What does the @classmethod decorator enable that instance methods cannot do?",
      options: [
        "Access class-level data and create alternative constructors",
        "Run faster by bypassing instance creation overhead",
        "Make methods accessible without creating any instances",
        "Automatically validate all method parameters"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. @classmethod receives `cls` (the class) as first parameter instead of `self` (instance). This enables: (1) accessing/modifying class attributes (`cls.count += 1`), and (2) alternative constructors that return class instances (`@classmethod def from_string(cls, date_str): return cls(...)`). Option 2 is wrong—no performance benefit. Option 3 is misleading—static methods do this better for utilities that don't need `cls`. Option 4 is wrong—validation is separate concern. When working with AI: 'create a from_json class method that deserializes JSON into an object' tells AI to use @classmethod. Example: `user = User.from_json(json_data)` calls class method that creates instance. Lesson 4 demonstrates class methods as factory patterns.",
      source: "Lesson 4: Encapsulation and Method Types"
    },
    {
      question: "When does a static method (@staticmethod) make sense in class design?",
      options: [
        "Utility function logically belongs with class but doesn't need instance/class data",
        "Method needs to modify class-level state across all instances",
        "Method must access instance attributes without using self",
        "Static methods are deprecated; use class methods instead"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Static methods are utility functions that don't need `self` or `cls` but are logically part of the class. Example: `class Math: @staticmethod def add(a, b): return a + b`. Calling `Math.add(2, 3)` groups the utility with Math class without needing instance/class data. Option 2 describes class methods, not static. Option 3 is impossible—can't access instance data without self. Option 4 is false—static methods are valid. When to use: function conceptually belongs to class but is pure computation. When working with AI: 'create a validate_email static method' signals this pattern. However, static methods are less common in Python than instance/class methods—often a module-level function is clearer. Lesson 4 covers this distinction.",
      source: "Lesson 4: Encapsulation and Method Types"
    },
    {
      question: "You're building a Game Character System with Player, Enemy, and Item classes. What's the first design decision you should make?",
      options: [
        "Identify what data each class stores and what actions each performs",
        "Write all the code for Player class before designing others",
        "Choose between inheritance and composition for class relationships",
        "Decide on private vs public attributes for all classes"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Before writing code, map out: What data does each class need (Player: health, inventory; Enemy: health, attack_power; Item: name, value)? What actions does each perform (Player: take_damage(), use_item(); Enemy: attack(); Item: describe())? This conceptual design informs all other decisions. Option 2 jumps to implementation too early. Option 3 is premature—can't choose relationships without knowing what each class does. Option 4 is detail-level, comes later. When collaborating with AI on multi-class systems: 'Player has health and inventory, can take damage and use items. Enemy has health and attack power, can attack players. Item has name and value, can be described.' This design-first approach (Lesson 5) ensures cohesive architecture before coding.",
      source: "Lesson 5: Game Character System (Capstone)"
    },
    {
      question: "In a multi-class system, what does 'composition' mean?",
      options: [
        "One class contains instances of other classes as attributes",
        "Multiple classes inherit from the same parent class",
        "Classes are defined in the same Python file",
        "Methods from different classes are combined into one"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Composition means 'has-a' relationship: Player has an Inventory (where Inventory is another class). In code: `class Player: def __init__(self): self.inventory = Inventory()`. The Player object contains an Inventory object as an attribute. Option 2 describes inheritance ('is-a'). Option 3 describes file organization, not relationships. Option 4 misunderstands composition. Composition is powerful for modeling real-world relationships: Car has an Engine, Game has Players, Document has Paragraphs. When describing to AI: 'Player has an inventory that stores items' signals composition—AI should create Inventory class and include `self.inventory = Inventory()` in Player. Lesson 5's Game Character System uses composition: Player/Enemy have Inventory instances.",
      source: "Lesson 5: Game Character System (Capstone)"
    },
    {
      question: "What's the advantage of using an abstract base class in a game character system?",
      options: [
        "Enforces that Player and Enemy both implement required methods like take_damage()",
        "Makes the code run faster by optimizing method calls",
        "Prevents creating too many character objects in memory",
        "Automatically generates default implementations for all subclasses"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Abstract base classes (ABC) define contracts: if Player and Enemy inherit from Character ABC, both MUST implement abstract methods like `take_damage()` or Python raises errors at instantiation. This ensures interface consistency. Option 2 is wrong—no performance benefit. Option 3 is wrong—ABCs don't limit instances. Option 4 is backwards—abstract methods have no implementation, subclasses must provide it. Example: `from abc import ABC, abstractmethod; class Character(ABC): @abstractmethod def take_damage(self, amount): pass`. Now any subclass must implement take_damage(). When working with AI: 'create Character abstract base class requiring take_damage and attack methods' ensures AI sets up the contract. Lesson 5 uses ABC to enforce interface consistency across character types.",
      source: "Lesson 5: Game Character System (Capstone)"
    },
    {
      question: "When asking AI to implement a multi-class system, which prompt produces better architecture?",
      options: [
        "Design a turn-based game with Player (health, inventory), Enemy (health, attack), combat loop",
        "Write classes for a game",
        "Create Player class, Enemy class, and Item class",
        "Use OOP to build a game system"
      ],
      correctOption: 0,
      explanation: "Option 1 is best because it describes WHAT (turn-based game), WHO (Player/Enemy), WHAT THEY HAVE (health, inventory, attack), and HOW THEY INTERACT (combat loop). This gives AI enough context to design cohesive architecture. Option 2 is too vague—AI has no constraints. Option 3 lists classes but not their relationships or purpose. Option 4 mentions OOP but no domain details. Good AI prompts for system design answer: What is the system? What are the entities? What data do they have? How do they interact? When AI has this context, it can suggest appropriate patterns (composition vs inheritance, which methods each class needs). Lesson 5 demonstrates planning with AI at architecture level before implementation.",
      source: "Lesson 5: Game Character System (Capstone)"
    },
    {
      question: "What testing strategy validates that objects maintain independent state in a multi-class system?",
      options: [
        "Create two instances, modify one, verify the other is unchanged",
        "Run the program multiple times and check for different results",
        "Use assert statements on class definitions, not instances",
        "Testing is unnecessary; Python guarantees object independence"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. To verify independence: `player1 = Player('Alice'); player2 = Player('Bob'); player1.take_damage(50); assert player2.health == 100` (unchanged). If both players share state, player2's health would also decrease—bug detected. Option 2 doesn't test object independence. Option 3 makes no sense—you test instances, not definitions. Option 4 is dangerous—while Python does ensure independence, bugs in your code (like using class attributes instead of instance attributes) can break this. This testing pattern appears throughout Lesson 2 and is applied to multi-class systems in Lesson 5. When working with AI, request: 'generate tests verifying player1 and player2 have independent health values.'",
      source: "Lesson 5: Game Character System (Capstone)"
    },
    {
      question: "You have `class Dog: pass` and create `my_dog = Dog()`. Then you add attributes: `my_dog.name = 'Max'`. What's the problem with this approach?",
      options: [
        "Attributes must be set manually for every object; no type enforcement",
        "Python raises AttributeError when adding attributes after creation",
        "The attributes are shared across all Dog instances",
        "This is the correct way to add attributes in Python"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Manual attribute assignment works but is repetitive, error-prone, and lacks type enforcement. For 100 dogs, you'd write 300+ lines of attribute assignments. If you forget `my_dog.breed`, no error until you access it. Option 2 is false—Python allows dynamic attribute addition. Option 3 is false—these are instance attributes, not shared. Option 4 is wrong—while it works, it's not professional practice. Solution: use `__init__` constructor to enforce required attributes with type hints: `def __init__(self, name: str, breed: str)`. Then `Dog('Max', 'Lab')` ensures every dog has name and breed. When working with AI, you'd never say 'create empty class then add attributes manually'—you'd say 'create Dog class with name and breed attributes in constructor.' Lesson 2 demonstrates this evolution.",
      source: "Lesson 2: Classes and Objects Basics"
    },
    {
      question: "What does `dir(my_dog)` reveal about objects created from a simple class?",
      options: [
        "All methods Python automatically provides, including __init__, __str__, __dict__",
        "Only the methods you explicitly defined in the class",
        "An error; dir() only works on built-in types",
        "The memory address where the object is stored"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. `dir(object)` lists all attributes and methods, including those Python provides automatically (called 'dunder methods' or 'magic methods'): `__init__`, `__str__`, `__dict__`, `__eq__`, etc. Even an empty class (`class Dog: pass`) gets dozens of default methods from Python. Option 2 is wrong—dir() shows inherited and default methods too. Option 3 is false—dir() works on any object. Option 4 confuses dir() with id(). This reveals that Python objects are richer than they appear—when you define a class, you're extending Python's default object behavior. When debugging with AI: 'show me dir(my_object) to understand what methods are available' helps explore object capabilities. Lesson 2 uses dir() to discover Python's automatic method provision.",
      source: "Lesson 2: Classes and Objects Basics"
    },
    {
      question: "When calling `dog1 = Dog('Max', 'Labrador', 5)`, what happens internally in Python?",
      options: [
        "Python creates object, then calls __init__ with 'Max', 'Labrador', 5 and the object as self",
        "The Dog class is copied and modified to include the specific values",
        "__init__ runs first, creates the object, then returns it to dog1",
        "Each parameter creates a separate object that's combined into dog1"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. The sequence: (1) `Dog(...)` triggers `__new__` which creates a blank object, (2) Python automatically calls `__init__(newly_created_object, 'Max', 'Labrador', 5)`, passing the new object as `self`, (3) `__init__` sets `self.name = 'Max'`, etc., (4) Python returns the initialized object to `dog1`. Option 2 is wrong—the class isn't copied. Option 3 is backwards—object creation precedes __init__. Option 4 misunderstands instantiation. Understanding this flow helps debug: if `__init__` raises an exception, object creation fails. When working with AI: 'add validation in __init__ to prevent invalid objects' leverages this initialization sequence. Lesson 2 traces through object creation step-by-step.",
      source: "Lesson 2: Classes and Objects Basics"
    },
    {
      question: "Why is `class BankAccount: balance = 0` (class attribute) problematic for a banking system?",
      options: [
        "All accounts would share the same balance value; deposit to one affects all",
        "Class attributes are slower to access than instance attributes",
        "Python doesn't allow numeric class attributes",
        "Class attributes can only be accessed by class methods"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. `balance` as class attribute means one shared value for ALL BankAccount instances. If `account1.deposit(100)`, the shared `balance` increases, so `account2.balance` also shows 100—disaster in banking. Every account would share one balance. Option 2 is false (negligible performance difference). Option 3 is false (Python allows any type). Option 4 is false (accessible from anywhere). Correct design: `def __init__(self, initial_balance=0): self.balance = initial_balance`—now each account has independent balance. This is a critical instance vs. class attribute decision. When reviewing AI-generated code for multi-object systems, verify per-object data uses `self.attribute` in `__init__`, not class-level definition. Lesson 3 demonstrates this through experiments.",
      source: "Lesson 3: Constructors, Destructors, and Attributes"
    },
    {
      question: "You want to track the total number of Dog objects created. Should this be an instance or class attribute?",
      options: [
        "Class attribute; it's shared data across all instances",
        "Instance attribute; each dog needs its own count",
        "Global variable; classes shouldn't track instance counts",
        "Neither; Python automatically tracks this"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Total count is shared across all instances, so use class attribute: `class Dog: count = 0; def __init__(self, ...): Dog.count += 1` (note: use `Dog.count`, not `self.count`, to modify class attribute). Every time a Dog is created, the shared counter increments. Option 2 makes no sense—each dog doesn't have its own count. Option 3 works but pollutes global namespace and isn't object-oriented. Option 4 is false—Python doesn't auto-track. When working with AI: 'track total instances created using class attribute' signals this pattern. This demonstrates when shared state (class attribute) is the right choice. Note: incrementing in `__init__` via `Dog.count += 1` avoids the shadowing problem. Lesson 3 covers using class attributes for shared counters.",
      source: "Lesson 3: Constructors, Destructors, and Attributes"
    },
    {
      question: "What does `obj.__dict__` reveal?",
      options: [
        "All instance attributes for that specific object",
        "All methods defined in the class",
        "The class hierarchy and parent classes",
        "The memory address of the object"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. `obj.__dict__` is a dictionary of instance attributes: `{'name': 'Max', 'age': 5, ...}`. It shows what's stored specifically on that instance. Class attributes won't appear in `obj.__dict__` (they're in `ClassName.__dict__`). Option 2 is wrong—methods aren't in instance __dict__, they're in class. Option 3 is wrong (that's `obj.__class__.__mro__`). Option 4 is wrong (that's `id(obj)`). Use case: debugging whether attribute is instance or class: `'name' in obj.__dict__` returns True if instance attribute. When working with AI to debug attribute access issues: 'print obj.__dict__ to see what attributes are stored on the instance.' Lesson 3 uses __dict__ for debugging attribute scope.",
      source: "Lesson 3: Constructors, Destructors, and Attributes"
    },
    {
      question: "You asked AI to create a BankAccount class that prevents negative balances. How should AI implement this?",
      options: [
        "Use @property for balance with validation in the setter",
        "Store balance as public attribute and trust users not to set negative values",
        "Make balance a class attribute so it's shared and validated once",
        "Use __setattr__ to prevent all attribute modifications"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Implementation: `@property def balance(self): return self._balance` (getter) and `@balance.setter def balance(self, value): if value < 0: raise ValueError('Negative balance'); self._balance = value` (validated setter). Now `account.balance = -100` raises error. Option 2 is wrong—no validation, trusts external code. Option 3 is wrong—balance must be per-account, not shared. Option 4 is too broad—blocks all attribute updates. This demonstrates encapsulation: internal representation (`_balance`) protected by public interface (`balance` property) with validation. When describing to AI: 'encapsulate balance with property that prevents negative values' signals this pattern. Lesson 4 covers properties as validation mechanism.",
      source: "Lesson 4: Encapsulation and Method Types"
    },
    {
      question: "What's the purpose of name mangling (double underscore prefix like `__private`) in Python?",
      options: [
        "Prevent naming conflicts in inheritance by renaming to _ClassName__private",
        "Make attributes truly inaccessible from outside the class",
        "Improve performance by optimizing private attribute access",
        "Signal that attributes should be accessed through properties"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Name mangling transforms `self.__private` into `self._ClassName__private` to avoid conflicts when subclasses define same attribute name. If Parent has `__x` and Child has `__x`, they become `_Parent__x` and `_Child__x`—no collision. Option 2 is wrong—you CAN access via mangled name `obj._ClassName__private` (just discouraged). Option 3 is false—no performance benefit. Option 4 is wrong—single underscore signals 'internal use,' double underscore is for inheritance conflict prevention. This is rarely needed in practice—single underscore is more common. When reviewing AI-generated code, if you see `__` prefix, verify it's intentional for inheritance scenarios, not misused for 'privacy' (Python has no true private). Lesson 4 explains Python's convention-based access control.",
      source: "Lesson 4: Encapsulation and Method Types"
    },
    {
      question: "When should you use getter and setter methods instead of direct attribute access?",
      options: [
        "When you need validation, computation, or side effects on attribute access",
        "Always use getters/setters; direct access is bad practice",
        "Never in Python; properties are always better",
        "Only when the attribute is a class attribute"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Use properties (Pythonic getters/setters) when: (1) validation needed (`balance` must be non-negative), (2) computation required (`fahrenheit` computed from `celsius`), (3) side effects needed (logging, caching). For simple attributes with no logic, direct access is fine and Pythonic. Option 2 is wrong—unnecessary getters/setters add boilerplate without value. Option 3 is wrong—properties ARE getters/setters, just Pythonic. Option 4 is wrong—unrelated to class vs instance. When working with AI: 'create age property that rejects negative values' signals validation need. Start simple (direct access), add properties when complexity demands it. This aligns with Python philosophy: simple when possible, powerful when needed. Lesson 4 demonstrates when properties add value.",
      source: "Lesson 4: Encapsulation and Method Types"
    },
    {
      question: "You have a User class and want to create instances from JSON data: `User.from_json({'name': 'Alice', 'age': 30})`. What method type should from_json be?",
      options: [
        "@classmethod; it needs access to cls to create instances",
        "Instance method; it operates on user data",
        "@staticmethod; it doesn't need self or cls",
        "Regular function outside the class; classes can't create themselves"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Alternative constructors use @classmethod: `@classmethod def from_json(cls, data): return cls(data['name'], data['age'])`. The `cls` parameter is the class itself (User), so `cls(...)` creates an instance. This pattern is cleaner than external factory functions. Option 2 is wrong—instance methods need existing instance; this creates one. Option 3 is wrong—@staticmethod could work but you'd have to hardcode `User(...)` instead of `cls(...)`; @classmethod is more flexible for inheritance. Option 4 is wrong—classes can definitely create instances via class methods. When working with AI: 'create from_json class method for deserialization' signals this factory pattern. Lesson 4 demonstrates alternative constructors with @classmethod.",
      source: "Lesson 4: Encapsulation and Method Types"
    },
    {
      question: "What's the difference between these two approaches to computed attributes: (1) storing both celsius and fahrenheit in __init__, (2) storing celsius and computing fahrenheit via @property?",
      options: [
        "Option 2 avoids synchronization bugs; fahrenheit is always correct when celsius changes",
        "Option 1 is faster; no computation needed on access",
        "Option 2 uses less memory by not storing fahrenheit",
        "They are functionally identical; use whichever you prefer"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Storing both (option 1) creates duplicate state that can desynchronize: if `temp.celsius = 30`, `temp.fahrenheit` remains the old value unless you update it—bug. Computing fahrenheit via @property (option 2) eliminates this: `@property def fahrenheit(self): return self.celsius * 9/5 + 32` ensures fahrenheit is always current. Option 2 (speed) is misleading—trivial computation cost. Option 3 is technically true but minor benefit. Option 4 is wrong—they differ in maintainability and correctness. This demonstrates a key principle: avoid duplicate state when one value is computable from another. When working with AI: 'make fahrenheit a computed property from celsius' signals this pattern. Lesson 4 covers eliminating duplicate state via properties.",
      source: "Lesson 4: Encapsulation and Method Types"
    },
    {
      question: "In a Game Character System, you want both Player and Enemy to have health and take_damage(). What OOP pattern best models this shared behavior?",
      options: [
        "Create Character base class with health and take_damage(), Player/Enemy inherit from it",
        "Copy-paste health and take_damage() into both Player and Enemy classes",
        "Use class attributes to share health across all characters",
        "Create global functions for health and damage that accept character objects"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Inheritance lets you define shared behavior once in a parent (Character) and reuse in children (Player, Enemy). Define `class Character: def __init__(self): self.health = 100; def take_damage(self, amount): self.health -= amount`, then `class Player(Character): pass` inherits these. Option 2 violates DRY principle and creates maintenance nightmare (bug fix needs to happen twice). Option 3 makes all characters share ONE health value—disaster. Option 4 works but isn't object-oriented. When working with AI: 'create Character base class with shared health and damage mechanics, then Player and Enemy inherit from it' signals this inheritance pattern. Lesson 5's capstone demonstrates inheritance for code reuse.",
      source: "Lesson 5: Game Character System (Capstone)"
    },
    {
      question: "What does 'has-a' relationship mean in object-oriented design?",
      options: [
        "Composition: one class contains instances of another (Player has Inventory)",
        "Inheritance: one class extends another (Dog is Animal)",
        "Association: two classes reference each other (User has Friends)",
        "Aggregation: classes share ownership of objects"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. 'Has-a' is composition: Player has an Inventory, Car has an Engine. In code: `class Player: def __init__(self): self.inventory = Inventory()`. The Player object contains an Inventory object. Option 2 describes 'is-a' (inheritance). Option 3 is association (broader concept). Option 4 is a specific composition variant. Composition models real-world containment. When describing to AI: 'Player has an Inventory that stores Items' signals composition—AI should create Inventory class and use it as Player attribute. Contrast with inheritance: Player is-a Character (inheritance), Player has-an Inventory (composition). Lesson 5 demonstrates both patterns in Game Character System.",
      source: "Lesson 5: Game Character System (Capstone)"
    },
    {
      question: "You asked AI to implement combat system where Player attacks Enemy. AI generates `player.attack(enemy)`. What did AI understand about the design?",
      options: [
        "Attack is Player behavior that takes Enemy as parameter",
        "Attack should be a global function, not a method",
        "Enemy should call its own attack method",
        "Attack returns a new Player object"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. AI interpreted 'player attacks enemy' as Player behavior (instance method) that operates on Enemy object: `class Player: def attack(self, target): target.take_damage(self.attack_power)`. The attacker is `self` (player), target is parameter (enemy). Option 2 is un-OOP. Option 3 is backwards—player initiates attack, enemy receives damage. Option 4 makes no sense. This demonstrates how natural language maps to OOP design: subject performs action on object → subject.method(object). When describing system interactions to AI, specify: WHO does WHAT to WHOM. 'Player attacks Enemy for damage' → `player.attack(enemy)`. Lesson 5's capstone implements combat interactions between objects.",
      source: "Lesson 5: Game Character System (Capstone)"
    },
    {
      question: "Why use abstract base classes instead of just defining methods in a parent class with `pass`?",
      options: [
        "ABCs enforce implementation; instantiating class without required methods raises TypeError",
        "ABCs make code run faster through optimized method dispatch",
        "ABCs automatically generate default implementations for subclasses",
        "ABCs prevent too many subclasses from being created"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. With ABC: `from abc import ABC, abstractmethod; class Character(ABC): @abstractmethod def attack(self): pass`, if you create `Player(Character)` without implementing `attack()`, `Player()` raises TypeError. Without ABC, `pass` methods just do nothing—no enforcement. Option 2 is false—no performance benefit. Option 3 is backwards—abstract methods have NO implementation. Option 4 is false. ABCs catch design errors early: 'You forgot to implement attack()' at instantiation time, not when you try to call it. When working with AI: 'use ABC to enforce that all characters implement attack and take_damage' signals this contract pattern. Lesson 5 uses ABCs to ensure interface consistency.",
      source: "Lesson 5: Game Character System (Capstone)"
    },
    {
      question: "In organizing a multi-class game system, what's the benefit of creating separate files for Player, Enemy, and Inventory classes?",
      options: [
        "Separation improves maintainability; changes to Player don't risk breaking Enemy",
        "Python requires each class in its own file",
        "Separate files make the program run faster",
        "Classes in different files can't access each other's attributes"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Organizing related classes into modules (`player.py`, `enemy.py`, `inventory.py`) improves maintainability: you can modify Player implementation without opening enemy.py, reducing risk of accidental breakage. Imports make relationships explicit: `from enemy import Enemy`. Option 2 is false—Python allows multiple classes per file. Option 3 is false—no performance impact. Option 4 is false—imports enable access. This is software engineering principle: separation of concerns. When collaborating with AI on multi-class systems: 'organize Player, Enemy, Inventory into separate modules with clear imports' signals professional structure. However, for small projects, single file is fine. Lesson 5's capstone can be organized either way.",
      source: "Lesson 5: Game Character System (Capstone)"
    },
    {
      question: "You asked AI to implement a turn-based combat system. AI should understand this requires:",
      options: [
        "Loop alternating player/enemy actions, tracking state changes, checking win/loss conditions",
        "Just implementing attack methods; turns aren't AI's responsibility",
        "Creating separate Player and Enemy classes; no loop needed",
        "Using multithreading so both act simultaneously"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Turn-based combat requires: (1) game loop (`while player.is_alive() and enemy.is_alive():`), (2) alternating actions (player turn, then enemy turn), (3) state updates (health changes), (4) win/loss detection (break when someone dies). AI should generate this structure when given enough context. Option 2 is insufficient—methods alone don't create turns. Option 3 misses the gameplay logic. Option 4 is wrong—turn-based is sequential, not simultaneous. When describing to AI: 'implement turn-based combat: player attacks, enemy attacks, repeat until one dies' gives structure. Good AI prompt describes WHAT happens and IN WHAT ORDER. Lesson 5's capstone integrates classes into working game loop.",
      source: "Lesson 5: Game Character System (Capstone)"
    },
    {
      question: "What's the testing strategy for validating that Player and Enemy both correctly implement take_damage()?",
      options: [
        "Create instances, call take_damage() with known amounts, assert health decreased correctly",
        "Just verify the method exists; implementation doesn't need testing",
        "Use type hints; they automatically validate behavior",
        "Abstract methods don't need testing; Python checks them"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Test behavior, not just existence: `player = Player(); initial = player.health; player.take_damage(30); assert player.health == initial - 30`. Same for Enemy. This verifies implementation correctness. Option 2 is insufficient—method might exist but be buggy. Option 3 is wrong—type hints check types, not logic. Option 4 is wrong—ABCs enforce existence, not correctness of implementation. Testing principle: verify observable behavior (health decreases by damage amount), not internal implementation details. When working with AI: 'generate tests verifying take_damage() reduces health correctly for both Player and Enemy' produces behavior-focused tests. Lesson 5 emphasizes testing multi-class interactions.",
      source: "Lesson 5: Game Character System (Capstone)"
    },
    {
      question: "You're extending the Game Character System to add a Boss class with special abilities. What's the OOP approach?",
      options: [
        "Inherit from Enemy, add special_attack() method and higher health",
        "Copy all Enemy code and modify it for Boss",
        "Create Boss as separate class with no relationship to Enemy",
        "Modify Enemy class to include boss-specific code with conditionals"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Boss IS-A type of Enemy with enhanced capabilities, so inherit: `class Boss(Enemy): def __init__(self): super().__init__(); self.health *= 3; def special_attack(self): ...`. Boss reuses Enemy's core behavior and adds specialization. Option 2 violates DRY and creates maintenance nightmare. Option 3 loses code reuse benefits. Option 4 pollutes Enemy with boss-specific logic (violates single responsibility). This demonstrates inheritance for specialization. When describing to AI: 'create Boss class inheriting from Enemy with triple health and special_attack method' signals this extension pattern. Good OOP design enables extending systems without modifying existing code. Lesson 5's extension suggestions include adding Boss via inheritance.",
      source: "Lesson 5: Game Character System (Capstone)"
    },
    {
      question: "What does it mean that objects have 'independent state' in OOP?",
      options: [
        "Each object's attributes are stored separately; modifying one doesn't affect others",
        "Objects cannot communicate or interact with each other",
        "Each object must be in a separate file",
        "Objects share state for efficiency but appear independent"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Independent state means each object has its own memory for attributes: `player1 = Player('Alice'); player2 = Player('Bob'); player1.health = 50` only affects player1, not player2. They're stored in different memory locations. Option 2 is wrong—objects can interact (player attacks enemy). Option 3 confuses code organization with memory. Option 4 is backwards—state is actually separate, not just appearing so. This independence is core to OOP's power: you can create 1,000 players, each with different health/inventory, without them interfering. Testing for this: modify one instance, assert others unchanged. When AI generates multi-object systems, verify `self.attribute` is used (instance data), not class attributes (shared data). Lesson 2 demonstrates and tests object independence.",
      source: "Lesson 2: Classes and Objects Basics"
    },
    {
      question: "When reviewing AI-generated class code, what's a red flag that suggests mixing procedural and OOP styles incorrectly?",
      options: [
        "Methods that don't use self; they should probably be module functions",
        "Methods that accept parameters; OOP should use only attributes",
        "Classes with more than 5 methods; too complex for OOP",
        "Using __init__ constructor; real OOP uses class attributes"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. If instance methods don't access `self`, they're probably utilities that should be module functions or @staticmethod. Example: `def calculate_distance(self, p1, p2): return sqrt((p1.x - p2.x)**2 + ...)` doesn't use self—should be static method or module function. Option 2 is wrong—methods should accept parameters for flexibility. Option 3 is arbitrary—classes can have many methods. Option 4 is backwards—__init__ is fundamental to OOP. This signals code smell: method in class but doesn't need object state. When AI does this, request: 'move calculate_distance to module level or make it @staticmethod since it doesn't need instance data.' Lessons 2 and 4 cover proper method design.",
      source: "Lesson 4: Encapsulation and Method Types"
    },
    {
      question: "You asked AI to create a logging system where all components share the same log file. Should log_file be instance or class attribute?",
      options: [
        "Class attribute; it's shared configuration across all logger instances",
        "Instance attribute; each component might need different log files",
        "Global variable; classes shouldn't store file paths",
        "Neither; use function parameters instead of attributes"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. If ALL loggers share one file, use class attribute: `class Logger: log_file = 'app.log'`. All instances access the same path. If requirement changes to 'each component CAN have different log file,' use instance attribute with class attribute as default: `def __init__(self, log_file=None): self.log_file = log_file or Logger.log_file`. Option 2 might be correct depending on requirements. Option 3 works but isn't OOP. Option 4 misses the point—you need to store this somewhere. Design decision: shared across all = class attribute; unique per instance = instance attribute. When describing to AI, specify: 'all loggers use the same log file' → class attribute; 'each logger can have different log file' → instance attribute. Lesson 3 covers choosing attribute scope.",
      source: "Lesson 3: Constructors, Destructors, and Attributes"
    },
    {
      question: "What's the advantage of using type hints in class definitions (`def __init__(self, name: str, age: int)`) when working with AI?",
      options: [
        "Signals expected types to AI; helps AI generate compatible method implementations",
        "Makes Python enforce types at runtime; prevents all type errors",
        "Required for classes; Python rejects classes without type hints",
        "Automatically generates validation code for all parameters"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. Type hints communicate intent to AI and human developers: 'name should be str, age should be int.' When AI generates methods using these attributes, it knows: 'age is int, so I can use numeric operations.' Type hints also enable IDE autocomplete and type checkers like mypy. Option 2 is wrong—Python's type hints are documentation, not runtime enforcement (unless you use Pydantic or similar). Option 3 is false—type hints are optional. Option 4 is false—type hints don't generate validation. When collaborating with AI: 'add type hints to constructor and all methods' improves generated code quality and catches type mismatches. Lessons 2-4 consistently use type hints as professional practice.",
      source: "Lesson 2: Classes and Objects Basics"
    },
    {
      question: "You're debugging a multi-class system and objects are behaving unexpectedly. What's the first diagnostic step using OOP principles?",
      options: [
        "Print instance __dict__ to see actual attribute values for each object",
        "Rewrite the entire system; debugging OOP is too hard",
        "Add global variables to track state across objects",
        "Remove all classes and convert to procedural code"
      ],
      correctOption: 0,
      explanation: "Option 1 is correct. `print(player1.__dict__)`, `print(player2.__dict__)` shows exactly what's stored on each instance: `{'name': 'Alice', 'health': 50, ...}`. This reveals: Are attributes set correctly? Are objects independent or sharing state? Are attributes instance or class level? Option 2 is premature—debug first. Option 3 makes things worse, not better. Option 4 abandons OOP without diagnosis. Additional diagnostics: `print(type(player1))` (verify object type), `dir(player1)` (see available methods), `id(player1)` (memory address). When working with AI to debug: 'print player1.__dict__ and player2.__dict__ to compare their states' helps identify where objects diverge from expected. Lesson 3 teaches using __dict__ for debugging attribute issues.",
      source: "Lesson 3: Constructors, Destructors, and Attributes"
    }
  ]}
/>
