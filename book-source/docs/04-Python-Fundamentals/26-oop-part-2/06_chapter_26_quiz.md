---
sidebar_position: 6
title: "Chapter 26: Object-Oriented Programming Part II Quiz"
---

# Chapter 26: Object-Oriented Programming Part II Quiz

Test your understanding of advanced OOP patterns including inheritance with Method Resolution Order, polymorphism and duck typing, composition over inheritance, special methods, and professional design patterns.

<Quiz
  title="Chapter 26: Object-Oriented Programming Part II Assessment"
  questions={[    {
      question: "You have a class hierarchy: Vehicle → Car → SportsCar. When SportsCar calls super().__init__(), what happens if Car forgets to call super().__init__()?",
      options: [
        "Vehicle's __init__ is never executed and attributes missing",
        "Python automatically calls Vehicle's __init__ anyway using reflection",
        "SportsCar initialization fails with immediate AttributeError at instantiation",
        "Car's __init__ executes but Vehicle attributes use default values"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. When Car forgets to call super().__init__(), the initialization chain breaks. SportsCar calls Car's __init__ via super(), but Car never calls Vehicle's __init__(), so Vehicle's attributes are never set. This creates objects with missing attributes that fail at runtime when accessed. Option 2 is false—Python doesn't automatically call parent constructors. Option 3 is incorrect because instantiation succeeds; the error occurs later when code tries to access Vehicle attributes. Option 4 is wrong—there are no default values; the attributes simply don't exist. This demonstrates why proper super() chains are critical in multi-level inheritance, covered in Lesson 1.",
      source: "Lesson 1: Inheritance and Method Resolution Order"
    },
    {
      question: "Given this diamond inheritance: class D(B, C) where both B and C inherit from A, what is the Method Resolution Order for class D?",
      options: [
        "D → B → A → C → object violates C3 linearization consistency",
        "D → B → C → A → object order maintains left parent priority",
        "D → A → B → C → object ensures base comes first",
        "D → C → B → A → object because right parent wins"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. Python's C3 linearization algorithm produces MRO: D → B → C → A → object. This order satisfies three principles: (1) subclasses before parents, (2) inheritance order preserved (B before C as listed in class D(B, C)), and (3) no class appears twice. Option 2 is wrong—A appears after both B and C, not between them. Option 3 violates the inheritance order preservation rule—B must come before C. Option 4 is backwards—left parent (B) takes priority over right parent (C). Understanding MRO prevents method call confusion in multiple inheritance scenarios, as explained in Lesson 1 with the C3 linearization algorithm.",
      source: "Lesson 1: Inheritance and Method Resolution Order"
    },
    {
      question: "You're building a multi-agent system with BaseAgent, ChatMixin, and ToolMixin. You want ChatMixin's process() method to execute first. How do you structure the inheritance?",
      options: [
        "class SmartAgent(ToolMixin, ChatMixin, BaseAgent) gives ToolMixin the execution priority",
        "class SmartAgent(BaseAgent, ChatMixin, ToolMixin) ensures BaseAgent has priority control",
        "class SmartAgent(ChatMixin, ToolMixin, BaseAgent) puts ChatMixin first in MRO",
        "Inheritance order doesn't matter because Python searches alphabetically by name"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 3. In Python's MRO, classes are searched left-to-right as listed in the inheritance declaration. class SmartAgent(ChatMixin, ToolMixin, BaseAgent) produces MRO: SmartAgent → ChatMixin → ToolMixin → BaseAgent → object. When process() is called, Python finds ChatMixin's version first and executes it. Option 2 would give BaseAgent priority (wrong if you want ChatMixin first). Option 3 would give ToolMixin priority. Option 4 is completely false—Python never searches alphabetically; it uses explicit MRO based on inheritance order. This demonstrates how mixin order controls capability precedence in professional agent architectures, covered in Lesson 1's MRO section.",
      source: "Lesson 1: Inheritance and Method Resolution Order"
    },
    {
      question: "When asking AI to implement a three-level inheritance hierarchy (Animal → Mammal → Dog), which prompt structure produces the most maintainable code?",
      options: [
        "Tell AI to avoid super() and directly initialize all attributes",
        "Ask AI to write all three classes without explaining inheritance relationships",
        "Specify exact line-by-line implementation for each constructor to avoid errors",
        "Describe each level's unique attributes and all levels' super() requirements clearly"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 4. The best AI prompt describes intent and requirements: 'Animal has name, Mammal adds fur_color, Dog adds breed. Each level should call super().__init__() properly.' This lets AI generate correct initialization chains while understanding the design. Option 2 produces fragile code—without context, AI might forget super() or misunderstand attribute ownership. Option 3 is too prescriptive and prevents AI from applying best practices (like using super() correctly). Option 4 actively requests bad code—skipping super() breaks initialization chains in multi-level hierarchies. Effective AI collaboration for inheritance requires explaining the conceptual hierarchy, not micromanaging syntax, as emphasized in Lesson 1's AI collaboration sections.",
      source: "Lesson 1: Inheritance and Method Resolution Order"
    },
    {
      question: "You inspect a class's MRO and see: [SmartAgent, ChatMixin, ToolMixin, BaseAgent, object]. If ChatMixin and BaseAgent both have execute() methods, which one gets called?",
      options: [
        "ChatMixin's execute() because it appears earlier in the MRO search sequence",
        "BaseAgent's execute() because base classes have priority over mixin classes",
        "Python raises AmbiguousMethodError requiring you to specify which to call",
        "Both execute() methods run sequentially in the order they appear"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Python searches the MRO left-to-right and stops at the first class that defines the method. Since ChatMixin appears before BaseAgent in the MRO, ChatMixin's execute() is found and called. BaseAgent's execute() is never reached unless ChatMixin explicitly calls super().execute(). Option 2 is false—Python has no concept of 'base class priority'; only MRO order matters. Option 3 is wrong—Python never raises ambiguity errors; MRO deterministically resolves all method lookups. Option 4 misunderstands method resolution—only one method executes unless explicitly chained with super(). This is why MRO order matters critically in multi-capability agent systems, as explained in Lesson 1.",
      source: "Lesson 1: Inheritance and Method Resolution Order"
    },
    {
      question: "What is the key difference between method overriding and method overloading in Python?",
      options: [
        "Overloading changes method names while overriding keeps the same method name",
        "Python supports overriding but not traditional overloading with multiple signatures",
        "Overriding is compile-time while overloading is runtime behavior in Python",
        "Overloading requires @override decorator while overriding needs @overload marker"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. Python fully supports method overriding (child class replaces parent method with same name), but doesn't support traditional overloading (multiple methods with same name but different signatures like Java/C++). Python uses default arguments and *args/**kwargs for parameter flexibility instead of overloading. Option 2 is backwards—overriding keeps the same name, and overloading (if it existed) would too. Option 3 confuses concepts—both would be runtime in Python, which is dynamically typed. Option 4 invents non-existent decorators—@override doesn't enforce anything in standard Python (it's documentation), and @overload is from typing module for type hints only, not actual overloading. This distinction is foundational for understanding polymorphism in Lesson 2.",
      source: "Lesson 2: Polymorphism and Duck Typing"
    },
    {
      question: "You create an abstract base class Agent with @abstractmethod process(). What happens if you try to instantiate Agent directly?",
      options: [
        "Python issues a warning but allows instantiation for testing purposes",
        "Object is created but process() raises NotImplementedError when called later",
        "Python raises TypeError at instantiation preventing object creation immediately",
        "Abstract methods are automatically implemented with empty pass statements"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 3. Abstract Base Classes with @abstractmethod prevent instantiation at the class level. Calling Agent() immediately raises TypeError: 'Can't instantiate abstract class Agent with abstract method process'. This catches errors early (at object creation) rather than late (at method call). Option 2 describes the old pattern of raising NotImplementedError in method bodies—less safe because the error occurs at runtime, not instantiation. Option 3 is false—Python strictly prevents ABC instantiation, no warnings. Option 4 misunderstands @abstractmethod—abstract methods have no default implementation; subclasses must provide concrete implementations. This enforcement mechanism is why ABCs are superior to informal base classes, as explained in Lesson 2.",
      source: "Lesson 2: Polymorphism and Duck Typing"
    },
    {
      question: "When reviewing AI-generated code with inheritance, you see: class Dog(Animal): def __init__(self, name): self.name = name. What's the likely problem?",
      options: [
        "Name parameter should be *args to allow Animal flexibility in future",
        "The constructor should use __new__ instead of __init__ for proper inheritance",
        "Dog needs to explicitly list all Animal attributes in its constructor",
        "Dog's __init__ doesn't call super().__init__() so Animal initialization is skipped"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 4. The Dog constructor sets self.name but never calls super().__init__(), meaning Animal's initialization logic is skipped. If Animal sets attributes like self.species or performs setup, those never happen for Dog instances. You should request AI to add super().__init__(name) as the first line. Option 2 is wrong—__new__ is rarely needed and isn't the issue here. Option 3 misunderstands inheritance—Dog shouldn't re-implement Animal's attributes; it should call super() to let Animal handle its own. Option 4 is over-engineering—the issue is missing super(), not parameter flexibility. Recognizing missing super() calls is a critical code review skill for AI-generated inheritance hierarchies, emphasized in Lesson 1 and 2.",
      source: "Lesson 2: Polymorphism and Duck Typing"
    },
    {
      question: "In duck typing, when would a function expecting an object with read() method fail, even if the object has that method?",
      options: [
        "Duck typing always works if the method name matches exactly",
        "The read() method exists but has wrong signature like read(self, count, offset)",
        "Python checks method signatures at import time and rejects mismatches",
        "The object's class must inherit from Readable protocol for validation"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. Duck typing checks method existence, not method signatures. If your function calls reader.read() (no arguments) but the object's read() requires read(self, count, offset), the call fails with TypeError: missing required arguments. The method exists, but the signature doesn't match the expected interface. Option 2 is false—method name matching isn't sufficient; the signature must also be compatible. Option 3 is wrong—Python doesn't check signatures at import time (it's dynamically typed). Option 4 misunderstands duck typing—the whole point is NOT requiring inheritance from protocols or interfaces. This signature mismatch is a common duck typing failure mode, as explained in Lesson 2's duck typing section.",
      source: "Lesson 2: Polymorphism and Duck Typing"
    },
    {
      question: "You have PaymentProcessor ABC and three subclasses: CreditCard, PayPal, Crypto. You ask AI to add a logging feature to all processors. What's the best approach?",
      options: [
        "Add log() method to PaymentProcessor base class so all inherit automatically",
        "Ask AI to add identical log() implementation to each of three subclasses",
        "Use duck typing and manually add log() to whichever class needs it",
        "Create a separate Logger class and pass it as parameter to each"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Adding log() to the PaymentProcessor abstract base class makes it immediately available to all subclasses through inheritance. This is the primary benefit of ABC-based polymorphism—shared functionality automatically propagates. No code duplication, one implementation, consistent behavior. Option 2 is maintainability nightmare—three duplicate implementations, three places to update when logging changes. Option 3 sacrifices the benefit of inheritance—duck typing is great for flexibility, but here you have a clear hierarchy where shared behavior makes sense. Option 4 is over-engineering—composition via Logger makes sense for complex logging strategies, but simple logging belongs in the base class. This demonstrates when ABC provides clear value over duck typing, covered in Lesson 2.",
      source: "Lesson 2: Polymorphism and Duck Typing"
    },
    {
      question: "When would duck typing be preferable to using an Abstract Base Class for a file reader interface?",
      options: [
        "When you need to enforce that all readers implement required methods",
        "When you need compile-time type checking with mypy for safety guarantees",
        "When all reader implementations are in your codebase under your control",
        "When readers come from external libraries you don't control and can't modify"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 4. Duck typing shines when you're integrating with third-party libraries (like csv, json, io modules) that weren't designed to inherit from your ABC. If they have a read() method, duck typing lets you use them without wrapper classes or inheritance gymnastics. Option 2 is backwards—ABCs enable better type checking, not duck typing. Option 3 favors ABC (you control the code so you can enforce the contract). Option 4 also favors ABC—enforcement is ABC's strength, not duck typing's. The key insight: duck typing provides flexibility to work with external code you can't modify, while ABC provides enforcement for code you control. This trade-off is central to Lesson 2's design decision framework.",
      source: "Lesson 2: Polymorphism and Duck Typing"
    },
    {
      question: "You ask AI: 'Create a dispatcher for ChatAgent, CodeAgent, DataAgent without isinstance() checks.' What design pattern should AI use?",
      options: [
        "Singleton pattern to ensure only one dispatcher exists in the system",
        "Factory pattern to create agents and dictionary mapping types to handlers",
        "Polymorphism with common base class so dispatcher calls agent.process() for all",
        "Strategy pattern to encapsulate dispatch algorithms for each agent type"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 3. Polymorphism eliminates isinstance() checks by using a common interface (Agent base class with process() method). The dispatcher simply calls agent.process(message) regardless of concrete type—Python's method resolution handles the rest. Option 2 describes Factory (creating agents), not dispatching (routing to agents). Option 3 describes Singleton (single instance), irrelevant to eliminating type checks. Option 4 misapplies Strategy—Strategy is for swappable algorithms, not for avoiding type checks in dispatch. The core insight: polymorphism makes all agents look the same to the dispatcher, eliminating the need for type-specific logic. This is the fundamental value proposition of polymorphism, emphasized throughout Lesson 2.",
      source: "Lesson 2: Polymorphism and Duck Typing"
    },
    {
      question: "A Car has an Engine. An Employee has a Printer capability. Which statement correctly identifies the relationships?",
      options: [
        "Car has-a Engine is composition, Employee has-a PrinterCapability is composition too",
        "Both are inheritance because Car is-a Engine and Employee is-a Printer",
        "Car has-a Engine is aggregation, Employee has-a Printer is inheritance relationship",
        "Car should inherit from Engine and Employee should compose a Printer"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Both are composition ('has-a' relationships). A Car contains an Engine object and delegates engine operations to it. An Employee contains a PrinterCapability (or printing behavior) but isn't a Printer. Neither relationship is inheritance (is-a). Option 2 is completely wrong—Car is NOT an Engine, Employee is NOT a Printer. These are textbook examples of composition, not inheritance. Option 3 misidentifies the Employee/Printer relationship as inheritance—employees aren't printers. Option 4 is backwards—inheritance from Engine makes no sense (Car isn't an Engine type), and composition of Printer is correct but contradicts the recommended approach. This distinction between has-a (composition) and is-a (inheritance) is foundational, covered in Lesson 3.",
      source: "Lesson 3: Composition Over Inheritance and Code Organization"
    },
    {
      question: "Why do professional developers prefer composition over inheritance for adding flying capability to Bird classes?",
      options: [
        "Inheritance allows more code reuse and eliminates duplication across all bird classes",
        "Composition lets birds have or not have flying without forcing rigid hierarchy",
        "Flying requires multiple inheritance which composition cannot provide in Python effectively",
        "Composition makes bird classes immutable while inheritance allows attribute modification freely"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. Composition solves the penguin problem—not all birds fly. With composition, you give flying birds a Flyer object and non-flying birds (penguins) None or a Swimmer. With inheritance (all birds inherit from FlyingBird), penguins break the Liskov Substitution Principle. Composition provides flexibility to mix capabilities without rigid hierarchies. Option 2 is backwards—composition often involves more code initially but provides better long-term flexibility and avoids inheritance fragility. Option 3 is irrelevant—multiple inheritance works in Python but isn't the reason to use composition. Option 4 is nonsense—composition has nothing to do with immutability. The 'composition over inheritance' principle is a hard-learned industry lesson, detailed in Lesson 3.",
      source: "Lesson 3: Composition Over Inheritance and Code Organization"
    },
    {
      question: "What's the key difference between composition and aggregation in terms of object lifetime?",
      options: [
        "Aggregation is faster because it uses pointers instead of composition's deep copying",
        "Aggregation requires manual memory management while composition uses garbage collection automatically",
        "Composition allows multiple containers while aggregation enforces single ownership strictly",
        "Composition means container owns object, aggregation means container just references it"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 4. Composition implies strong ownership—when a Car is destroyed, its Engine is destroyed (the Car created and owns the Engine). Aggregation implies weak relationship—when a University closes, its Departments can exist independently (University references but doesn't own Departments). Option 2 is irrelevant—Python's garbage collection handles both equally; the distinction is conceptual, not memory management. Option 3 is backwards—composition often means single ownership, aggregation allows sharing. Option 4 is completely wrong—Python doesn't expose pointers, and neither pattern does deep copying by default. Understanding this lifetime distinction helps design appropriate object relationships, as explained in Lesson 3.",
      source: "Lesson 3: Composition Over Inheritance and Code Organization"
    },
    {
      question: "You're organizing a multi-agent system into modules. Which package structure follows best practices?",
      options: [
        "Each agent in its own package with nested __init__.py for encapsulation",
        "All agent classes in single agents.py file to keep related code together",
        "agents/ with __init__.py, separate files for chat.py, code.py, data.py modules",
        "Flat structure with agents_chat.py, agents_code.py, agents_data.py in root directory"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 3. A package (agents/ directory) with __init__.py and separate module files (chat.py, code.py, data.py) provides logical organization, clear namespace, and scalability. The __init__.py controls the public API via imports and __all__. Option 2 doesn't scale—as the system grows, a single file becomes unwieldy and hard to navigate. Option 3 is over-engineering—each agent doesn't need its own package; that adds unnecessary nesting. Option 4 pollutes the root directory and uses poor naming (prefix-based organization instead of package hierarchy). This package structure follows Python conventions and enables modular growth, as demonstrated in Lesson 3's module organization section.",
      source: "Lesson 3: Composition Over Inheritance and Code Organization"
    },
    {
      question: "What does the __all__ variable in a package's __init__.py control?",
      options: [
        "Which names are exported when someone does from package import star",
        "Which classes are instantiated automatically when the package is imported first",
        "Which modules are loaded into memory versus lazy-loaded on demand",
        "Which methods in classes are public versus private for encapsulation"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. __all__ = ['Dog', 'Cat'] in animals/__init__.py controls what gets imported when someone writes from animals import *. Only Dog and Cat are imported, even if other classes exist in the package. This lets you define a clean public API while keeping internal helpers private. Option 2 is false—nothing is automatically instantiated; __all__ only controls import behavior. Option 3 misunderstands Python imports—all modules in an __init__.py are imported when the package is imported (unless you use lazy imports explicitly, unrelated to __all__). Option 4 confuses class methods with package exports—__all__ is at the package level, not class level. This controls the public package interface, explained in Lesson 3.",
      source: "Lesson 3: Composition Over Inheritance and Code Organization"
    },
    {
      question: "When asking AI to refactor an inheritance hierarchy to use composition, which prompt gives the clearest guidance?",
      options: [
        "Tell AI to replace all inheritance with composition without explaining why",
        "Explain current inheritance problem, request composition solution with has-a relationships and delegation",
        "Ask AI to convert to composition and keep the same class structure",
        "Request composition using aggregation for all relationships to maintain flexibility"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. The best AI prompt explains the problem ('Penguin inherits fly() but penguins can't fly'), requests the pattern ('use composition with capabilities'), and provides guidance ('birds have Flyer or Swimmer objects, delegate operations to them'). This gives AI context to generate appropriate solutions. Option 2 lacks context—AI doesn't know WHY inheritance is wrong, so it might produce equally flawed composition. Option 3 is contradictory—composition fundamentally changes class structure; you can't 'keep the same structure.' Option 4 misuses aggregation—not all relationships should be weak; some should be strong ownership (composition). Effective AI collaboration requires explaining intent, not just issuing commands, as emphasized in Lessons 2 and 3.",
      source: "Lesson 3: Composition Over Inheritance and Code Organization"
    },
    {
      question: "What's the primary purpose of implementing __str__ versus __repr__ in a custom class?",
      options: [
        "__repr__ runs faster for performance, __str__ is more readable but slower",
        "__str__ is for string concatenation, __repr__ is for printing to console",
        "__str__ provides user-friendly display, __repr__ provides developer-friendly debug representation",
        "__str__ is required by Python, __repr__ is optional documentation only"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 3. __str__ returns human-readable output for end users (print(obj) shows this). __repr__ returns developer-friendly representation for debugging (repr(obj) and interactive shell show this). Convention: __repr__ should ideally return valid Python code to recreate the object like Person(name='Alice', age=30). Option 2 misunderstands their purpose—both return strings, and __str__ is what print() calls, not concatenation specifically. Option 3 is irrelevant—performance difference is negligible; the distinction is semantic (user vs developer audience). Option 4 is backwards—neither is required (Python provides defaults), but __repr__ is more important than __str__ for debugging. Understanding these conventions makes objects feel professional, as explained in Lesson 4.",
      source: "Lesson 4: Special Methods (Magic Methods)"
    },
    {
      question: "You implement __add__ for a Vector class. When should you return NotImplemented instead of raising TypeError?",
      options: [
        "When you want to provide a helpful error message to users",
        "Never, TypeError is always correct for invalid operations in special methods",
        "Only when the left operand is unsupported, not the right operand",
        "When the operation doesn't make sense for the given types Python tries"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 4. Returning NotImplemented (not raising TypeError) tells Python 'I don't know how to handle this operation.' Python then tries the reverse operation (__radd__ on the other operand). If both return NotImplemented, Python raises TypeError. This enables symmetric operations where either operand can handle the addition. Option 2 is wrong—NotImplemented is specifically for special methods to enable fallback behavior. Option 3 misunderstands the protocol—both __add__ and __radd__ should return NotImplemented for unsupported types. Option 4 confuses exception types—NotImplemented is NOT an exception; it's a signal to Python's operator dispatch system. This pattern enables flexible operator overloading, covered in Lesson 4.",
      source: "Lesson 4: Special Methods (Magic Methods)"
    },
    {
      question: "A Playlist class implements __len__ and __getitem__. What additional capability does this provide beyond just storing songs?",
      options: [
        "Songs are automatically sorted alphabetically when retrieved by index",
        "The playlist can be iterated in for loops and supports indexing",
        "The playlist becomes immutable and thread-safe for concurrent access",
        "Memory usage is optimized through lazy loading of song metadata"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. Implementing __len__ and __getitem__ makes the Playlist behave like a sequence. Python automatically enables iteration (for song in playlist), indexing (playlist[0]), and slicing (playlist[1:3]). This is the Container protocol—once __getitem__ works, iteration comes for free. Option 2 is false—__getitem__ doesn't sort; it just provides access to underlying storage in whatever order you defined. Option 3 is unrelated—special methods have nothing to do with immutability or thread safety. Option 4 is also unrelated—lazy loading is a separate design pattern, not something __getitem__ provides. Understanding which protocols enable which behaviors is key to making objects Pythonic, explained in Lesson 4.",
      source: "Lesson 4: Special Methods (Magic Methods)"
    },
    {
      question: "Why must __eq__ and __hash__ be implemented consistently together?",
      options: [
        "Objects comparing equal must have same hash for sets and dictionaries",
        "Python automatically generates __hash__ from __eq__ implementation for you",
        "__eq__ checks memory address while __hash__ checks value equality together",
        "Hash tables require both methods for performance optimization in Python internals"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Python's sets and dictionaries use hash values for fast lookup. If two objects compare equal (__eq__ returns True), they MUST have the same hash value, or they'll be treated as different in sets/dicts, breaking the data structure semantics. If Person(name='Alice', age=30) == Person(name='Alice', age=30), both must have identical hash(). Option 2 is backwards—Python makes objects unhashable if you define __eq__ without __hash__ to prevent this exact inconsistency. Option 3 confuses concepts—neither checks memory address in your implementations. Option 4 misrepresents the requirement—consistency is semantic (correctness), not performance. This hash consistency rule is fundamental for set/dict usage, covered in Lesson 4.",
      source: "Lesson 4: Special Methods (Magic Methods)"
    },
    {
      question: "You implement __iter__ and __next__ for a Countdown class. When must __next__ raise StopIteration?",
      options: [
        "After exactly 1000 iterations to prevent infinite loops in code",
        "When the iterator encounters an error during value generation processing",
        "When iteration is complete and no more values remain to yield",
        "Never, return None instead to signal end of iteration properly"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 3. __next__ must raise StopIteration when the iteration is exhausted. For Countdown(5), after yielding 5, 4, 3, 2, 1, the next call to __next__ should raise StopIteration to signal 'no more values.' Python's for loop catches this exception and terminates the loop cleanly. Option 2 confuses error handling—errors during iteration raise their specific exceptions (ValueError, etc.), not StopIteration. Option 3 is arbitrary nonsense—there's no 1000-iteration limit; StopIteration is based on your iterator's logic. Option 4 is wrong—returning None would make the iteration yield None forever; StopIteration is the protocol for termination. This is the iterator protocol contract, essential for custom iteration in Lesson 4.",
      source: "Lesson 4: Special Methods (Magic Methods)"
    },
    {
      question: "When would implementing __call__ make sense for a class?",
      options: [
        "When objects should be automatically garbage collected after function calls",
        "When you want to replace __init__ with more flexible initialization logic",
        "When the class needs to be compared with == operator for equality",
        "When objects store configuration state and need to be callable functions"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 4. __call__ makes objects callable like functions, enabling patterns like Multiplier(3) creating a callable triple = Multiplier(3), then calling triple(5) → 15. This combines state (the multiplier factor) with behavior (the multiplication operation). Common in decorators, callbacks, and stateful functions. Option 2 misunderstands __call__—it doesn't replace __init__; it makes instances callable after creation. Option 3 confuses __call__ with __eq__—equality has nothing to do with callability. Option 4 is nonsense—__call__ doesn't affect garbage collection. The pattern is powerful for creating function-like objects with state, demonstrated in Lesson 4's callable objects section.",
      source: "Lesson 4: Special Methods (Magic Methods)"
    },
    {
      question: "You ask AI to create a Money class supporting addition and subtraction. What should you specify to ensure correctness?",
      options: [
        "Request __add__, __sub__ with type checking and currency consistency validation",
        "Tell AI to implement __add__ only because Python automatically creates __sub__",
        "Ask for operator overloading without mentioning return types for flexibility",
        "Request all arithmetic operators including multiplication and division for completeness"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. A proper Money class needs explicit __add__ and __sub__ implementations that check: (1) the other operand is also Money (type checking), and (2) currencies match (can't add USD to EUR). Return new Money objects. This prevents nonsensical operations. Option 2 is false—Python doesn't auto-generate __sub__ from __add__; you must implement both. Option 3 is vague—without specifying return types, AI might return floats instead of Money objects, breaking the abstraction. Option 4 is questionable—multiplying Money by scalar (2 * $5) makes sense, but multiplying Money * Money or dividing Money by Money requires careful domain design. Clear specifications prevent AI from making incorrect assumptions, as emphasized in Lesson 4.",
      source: "Lesson 4: Special Methods (Magic Methods)"
    },
    {
      question: "What problem does the Singleton pattern solve in a multi-agent system?",
      options: [
        "Allows multiple managers to coordinate agents across distributed system nodes",
        "Ensures only one AgentManager exists preventing inconsistent agent registry state",
        "Automatically serializes agent creation to prevent race conditions in threads",
        "Reduces memory usage by sharing agent instances across all manager references"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. Singleton ensures that AgentManager() always returns the same instance, preventing the problem where different parts of code create separate managers with inconsistent agent registries. All code shares one manager with one authoritative agent list. Option 2 is backwards—Singleton enforces single instance, it doesn't enable multiple managers. Option 3 confuses thread safety (a separate concern) with the Singleton pattern's purpose (single instance). Option 4 misunderstands the pattern—Singleton controls manager instances, not agent instances. The core value: global coordination point with guaranteed single instance, preventing split-brain scenarios in agent management, as explained in Lesson 5.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "In the Factory pattern, what's the benefit of using a registry dictionary mapping agent types to classes?",
      options: [
        "Registration enables automatic dependency injection for agent constructor parameters",
        "The registry automatically validates that all agents implement required abstract methods",
        "Dictionary lookup is faster than isinstance() checks for performance optimization",
        "New agent types can be added without modifying factory creation logic"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 4. The registry pattern (AgentFactory._registry = {'chat': ChatAgent, 'code': CodeAgent}) enables open/closed principle—add new types by updating the registry, not by modifying create_agent() logic. You can even allow agent classes to self-register. This decouples type creation from type definition. Option 2 is false—registry doesn't validate ABC compliance; that's Python's ABC mechanism. Option 3 is irrelevant—the registry replaces if/elif chains, not isinstance() checks, and performance isn't the primary motivation. Option 4 confuses dependency injection with factory pattern—they're separate patterns. The registry's power is extensibility without modification, core to Factory pattern in Lesson 5.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "Why does the Observer pattern use an EventBus instead of direct agent-to-agent communication?",
      options: [
        "Direct communication is faster but EventBus provides better security through encryption",
        "EventBus automatically serializes events to disk for crash recovery and persistence",
        "Decouples publishers from subscribers enabling agents to communicate without knowing each",
        "Observers can only receive events if they inherit from Observable base class"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 3. Observer pattern decouples senders (subjects) from receivers (observers) through an event bus. Agents publish events to the bus without knowing which agents are listening. Observers register interest without knowing who publishes. This enables flexible, loosely-coupled architectures where you can add/remove agents without changing event publishers. Option 2 describes persistence (unrelated to Observer pattern's purpose). Option 3 is nonsense—security isn't Observer's goal. Option 4 is false—Python's Observer pattern uses duck typing or Protocol; no inheritance requirement. The decoupling enables scalable event-driven systems, as demonstrated in Lesson 5's multi-agent communication example.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "In the Strategy pattern, what makes strategies interchangeable at runtime?",
      options: [
        "Strategies are stored as JSON and loaded dynamically from configuration files",
        "All strategies implement common interface so they're polymorphically swappable",
        "Python's duck typing automatically converts between strategy types at runtime",
        "Strategy pattern requires all strategies to be stateless for thread safety"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. Strategy pattern works through polymorphism—all strategies implement the same interface (e.g., AggressiveStrategy, ConservativeStrategy both implement decide()), so the context can swap them without knowing concrete types. Context holds a strategy reference and calls strategy.decide(), regardless of which specific strategy is currently active. Option 2 confuses implementation details (you could load from JSON) with the pattern's core mechanism (polymorphic interface). Option 3 misapplies duck typing—while duck typing helps, the critical part is the common interface enabling swapping. Option 4 is false—strategies can be stateful or stateless; that's a separate design choice. The polymorphic interface is what enables runtime swapping, explained in Lesson 5.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "You ask AI to implement the Singleton pattern for ConfigManager. What's the critical requirement to specify?",
      options: [
        "Ensure __new__ controls instance creation and __init__ only initializes once",
        "Use global variable at module level instead of class-based singleton implementation",
        "Implement __init__ to check if instance exists and raise Exception if",
        "Make all methods @staticmethod to prevent multiple instantiation of the class"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Proper Singleton in Python uses __new__ to control instance creation (return existing instance or create new one) and guards __init__ with a flag to prevent re-initialization. Specifying this prevents AI from using broken Singleton patterns. Option 2 is an alternative approach but isn't the Singleton pattern—it's a module-level singleton, which works but doesn't demonstrate the pattern. Option 3 is backwards—__init__ shouldn't raise exceptions; it should just skip re-initialization. Option 4 is nonsense—@staticmethod prevents method access to instance altogether, breaking the pattern. Correctly specifying Singleton's __new__/__init__ dance ensures AI generates thread-safe, properly-initialized singletons, as detailed in Lesson 5.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "When would you use Factory pattern instead of directly instantiating agent classes?",
      options: [
        "When agents need to communicate through an event bus for coordination",
        "When you need exactly one instance of each agent type for singletons",
        "When agent type is determined at runtime from configuration or input",
        "When all agents share identical behavior and only differ in names"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 3. Factory pattern decouples object creation from usage, valuable when agent type isn't known until runtime. If config says agent_type: 'chat', the factory creates ChatAgent; if it says 'code', it creates CodeAgent—no conditional logic in caller. Direct instantiation (ChatAgent()) requires knowing the class at compile time. Option 2 describes Singleton pattern, not Factory. Option 3 describes Observer pattern communication. Option 4 suggests no need for Factory—if agents are identical except names, you'd use a single class with name parameters, not a factory. The Factory pattern's strength is dynamic creation based on runtime information, as explained in Lesson 5.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "In a multi-agent system using Observer pattern, how do you prevent infinite event loops where agents keep triggering each other?",
      options: [
        "Strategy pattern handles loop prevention by rotating through different strategies",
        "Observer pattern automatically prevents loops through built-in cycle detection mechanism",
        "Use Singleton pattern for EventBus to ensure all events are deduplicated",
        "Add event filtering logic so observers ignore events they themselves published"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 4. To prevent infinite loops (Agent A publishes event → Agent B reacts by publishing event → Agent A reacts by publishing event → ...), implement event filtering: agents track event IDs or check event sources and ignore their own events. This is application logic you must design. Option 2 is false—Observer pattern has no built-in loop prevention; it's your responsibility. Option 3 misunderstands Singleton—EventBus being singleton doesn't prevent loops; it just ensures one bus exists. Option 4 misapplies Strategy pattern—it's for algorithm selection, not loop prevention. Event loop prevention requires careful event handling logic, an advanced Observer pattern consideration mentioned in Lesson 5.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "What's the key insight of 'composition over inheritance' when applied to agent capabilities?",
      options: [
        "Agents should have capability objects instead of inheriting from capability classes",
        "Agent inheritance hierarchies are always faster than composition-based capability systems",
        "Composition requires more memory but provides better type safety guarantees",
        "Inheritance is simpler for single capabilities while composition handles multiple only"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Instead of class ChatAgent(ConversationCapability, MemoryCapability), use composition: ChatAgent has self.conversation = ConversationCapability() and self.memory = MemoryCapability(). This allows runtime capability swapping, easier testing (mock capabilities), and avoids multiple inheritance complexity. Capabilities are 'has-a' relationships, not 'is-a.' Option 2 is false—performance is not the primary differentiator; flexibility is. Option 3 is backwards—composition doesn't require more memory (similar to inheritance), and type safety is comparable. Option 4 misses the point—composition's advantage is flexibility for both single and multiple capabilities, not just multiple. This insight applies OOP patterns to practical AI agent design, as demonstrated throughout Lesson 5.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "You're integrating Singleton, Factory, Observer, and Strategy patterns in one agent system. Which pattern handles 'creating different agent types dynamically'?",
      options: [
        "Singleton pattern ensures each agent type has exactly one instance",
        "Factory pattern creates agents based on type strings or configuration",
        "Observer pattern creates agents when events trigger creation needs",
        "Strategy pattern selects which agent class to instantiate at runtime"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. Factory pattern's job is object creation—it takes a type identifier (string, enum, config) and returns the appropriate agent instance. AgentFactory.create_agent('chat', 'BotName') returns a ChatAgent. Option 2 misunderstands Singleton—it ensures one instance of a manager, not agents. Option 3 confuses event notification (Observer) with object creation. Option 4 misapplies Strategy—Strategy selects algorithms/behaviors, not which class to instantiate (that's Factory's job). Understanding which pattern solves which problem prevents pattern misuse, critical for the multi-pattern integration in Lesson 5's capstone.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "When asking AI to refactor a system to use design patterns, what's the most important context to provide?",
      options: [
        "Provide the complete existing codebase and let AI infer all problems",
        "Name the patterns you want without explaining why the system needs them",
        "Describe current problems like tight coupling, code duplication, inflexibility needing patterns",
        "List all 23 Gang of Four patterns and ask which apply"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 3. Effective AI collaboration requires explaining the problems design patterns will solve: 'We have 5 places creating agents (duplication) and adding new types requires modifying all 5 (tight coupling). Use Factory pattern to centralize creation.' This gives AI context to apply patterns correctly. Option 2 is vague—AI might apply patterns where they don't fit if it doesn't understand the problem. Option 3 dumps too much work on AI and lacks focus—what specifically needs refactoring? Option 4 is overwhelming and unfocused—you should already know which patterns apply to your problem. Problem-first thinking enables effective pattern application, emphasized throughout Lesson 5's pattern implementations.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "Given: class D(B, C) where B and C both define method foo(), which foo() gets called when D instances call it?",
      options: [
        "Both methods execute sequentially in declaration order of base classes",
        "C's foo() because right-most parent takes priority in Python MRO",
        "Python raises AmbiguousMethodError requiring explicit super() call to choose",
        "B's foo() because B is listed first in inheritance declaration"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 4. Python's MRO for class D(B, C) is D → B → C → object (left-to-right). When D().foo() is called, Python searches D first (not found), then B (found and executed). C's foo() is never reached unless B's foo() explicitly calls super().foo(). Option 2 is backwards—left parent (B) has priority, not right (C). Option 3 is false—Python deterministically resolves method calls via MRO; it never raises ambiguity errors. Option 4 misunderstands method resolution—only ONE method executes unless chained with super(). MRO order determines method precedence in multiple inheritance, foundational to Lesson 1's C3 linearization.",
      source: "Lesson 1: Inheritance and Method Resolution Order"
    },
    {
      question: "What happens when you call len() on an object that implements __getitem__ but not __len__?",
      options: [
        "Python counts items by calling __getitem__ repeatedly until IndexError occurs",
        "Python raises TypeError indicating object has no len() because missing implementation",
        "Python returns None because __len__ is optional for sequence protocol",
        "Python assumes infinite length and returns sys.maxsize for the value"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. len() specifically calls __len__. If __len__ isn't implemented, Python raises TypeError: object of type 'X' has no len(). Having __getitem__ isn't sufficient—__len__ is a separate contract. Option 2 describes an inefficient workaround Python doesn't implement. Option 3 is false—__len__ is not optional if you want len() to work; the sequence protocol doesn't auto-derive length. Option 4 is nonsense—Python doesn't assume anything about missing methods. Understanding which special methods enable which built-in functions (len → __len__, iter → __iter__, etc.) is key to implementing container protocols correctly, covered in Lesson 4.",
      source: "Lesson 4: Special Methods (Magic Methods)"
    },
    {
      question: "You want agents to use different decision-making algorithms at runtime. Which pattern is most appropriate?",
      options: [
        "Strategy pattern lets agents swap decision algorithms without changing agent",
        "Factory pattern creates different agents each with hardcoded decision algorithm",
        "Singleton pattern ensures all agents use the same decision algorithm instance",
        "Observer pattern notifies agents when to change their decision algorithms"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Strategy pattern encapsulates algorithms (AggressiveStrategy, ConservativeStrategy, BalancedStrategy) and makes them swappable. An agent holds a strategy reference and calls strategy.decide(). To change behavior, swap the strategy object at runtime: agent.set_strategy(ConservativeStrategy()). Option 2 misses the point—Factory creates objects but doesn't enable runtime algorithm swapping. Option 3 misapplies Singleton—sharing one algorithm across agents isn't the goal; per-agent swappable algorithms are. Option 4 confuses notification (Observer) with algorithm selection. Strategy pattern is the textbook solution for runtime algorithm selection, demonstrated in Lesson 5.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "Why is returning NotImplemented (not raising NotImplementedError) important in __add__ implementation?",
      options: [
        "They're identical in behavior but NotImplemented is conventional style for",
        "NotImplemented is faster because it doesn't construct exception objects for",
        "NotImplementedError is for abstract methods, NotImplemented is for operators only",
        "NotImplemented signals Python to try reverse operation, NotImplementedError stops"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 4. NotImplemented is a sentinel value that tells Python's operator dispatch: 'I don't handle this, try the other operand's reverse method (__radd__).' This enables commutative operations where 2 + Vector(1,2) works (int's __add__ returns NotImplemented, Python tries Vector's __radd__). NotImplementedError is an exception that halts execution—wrong for operator overloading. Option 2 focuses on irrelevant performance details. Option 3 is false—NotImplementedError is used in abstract method bodies, but NotImplemented isn't 'only for operators'; it's the operator dispatch protocol. Option 4 is wrong—they're completely different things with different behaviors. Understanding this distinction enables proper operator overloading, explained in Lesson 4.",
      source: "Lesson 4: Special Methods (Magic Methods)"
    },
    {
      question: "In ABC-based polymorphism, when should you use @abstractmethod versus leaving a method unimplemented?",
      options: [
        "@abstractmethod is for Python 3.10+ only, unimplemented works in all versions",
        "They're equivalent and stylistic preference determines which to use in code",
        "Use @abstractmethod to enforce implementation, unimplemented methods allow optional overrides",
        "Unimplemented methods are faster because they skip decorator overhead at runtime"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 3. @abstractmethod enforces that subclasses MUST implement the method—instantiation fails if they don't. Unimplemented methods (defined but raise NotImplementedError) are suggestions; instantiation succeeds but crashes at runtime if called. @abstractmethod shifts errors from runtime to instantiation time (much safer). Use @abstractmethod when a method is truly required for the contract. Option 2 is false—they have different semantics. Option 3 is incorrect—@abstractmethod has been in Python since 2.6. Option 4 is irrelevant—decorator overhead is negligible and not the design consideration. Understanding enforcement versus convention is critical for designing robust ABC hierarchies, as explained in Lesson 2.",
      source: "Lesson 2: Polymorphism and Duck Typing"
    },
    {
      question: "What's the risk of using duck typing when integrating with external libraries you don't control?",
      options: [
        "Duck typing prevents external libraries from being imported into your project",
        "Library updates might change method signatures breaking your duck typed code",
        "External library objects are immutable and can't participate in duck typing",
        "Python's type system rejects duck typing across package boundaries for safety"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. Duck typing's flexibility is also its risk—if you assume a library object has read(data), but the library updates to read(data, encoding), your code breaks. With ABC, you'd explicitly define the interface and validate; with duck typing, you trust the interface implicitly. This is why duck typing works best for code you control. Option 2 is nonsense—duck typing is a runtime concept and doesn't affect imports. Option 3 is false—immutability is unrelated to duck typing (which checks methods, not attribute mutability). Option 4 is completely wrong—Python doesn't enforce type boundaries. Understanding duck typing's trade-offs helps decide when it's appropriate, covered in Lesson 2's design decision framework.",
      source: "Lesson 2: Polymorphism and Duck Typing"
    },
    {
      question: "You implement __eq__ for Agent class comparing by agent_id. What must __hash__ return to maintain hash consistency?",
      options: [
        "hash(self.agent_id) so equal agents produce identical hash values always",
        "id(self) because memory address is unique and stable for hashing",
        "hash(self.name) because name is more user-friendly than id for",
        "Random value because Python handles hash collisions automatically with open addressing"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. If __eq__ compares agent_id, then __hash__ must hash agent_id. This ensures that if agent1 == agent2 (same agent_id), then hash(agent1) == hash(agent2). This consistency is required for sets and dicts to work correctly. Option 2 breaks the contract—if two agents have the same agent_id (comparing equal), they might have different memory addresses (different hashes), violating the rule. Option 3 is inconsistent if __eq__ ignores name—you'd have equal agents with different hashes. Option 4 misunderstands hashing—random values break sets/dicts completely; hashes must be deterministic and consistent with equality. This hash/equality consistency is mandatory, explained in Lesson 4.",
      source: "Lesson 4: Special Methods (Magic Methods)"
    },
    {
      question: "In a multi-pattern agent system, which pattern should manage the lifecycle of the EventBus itself?",
      options: [
        "Observer pattern manages EventBus lifecycle through attach and detach methods automatically",
        "Factory pattern creates multiple EventBus instances based on agent types needed",
        "Singleton pattern ensures only one EventBus exists for system-wide event coordination",
        "Strategy pattern selects which EventBus implementation to use at runtime"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 3. EventBus should be a Singleton—all agents need to communicate through the same event bus for consistency. Multiple EventBus instances would create isolated communication channels, defeating the purpose. Singleton guarantees one shared bus. Option 2 misapplies Factory—you don't want multiple buses; you want one centralized communication point. Option 3 confuses Observer (which uses the EventBus) with lifecycle management of the bus itself. Option 4 misapplies Strategy—you're not selecting between bus implementations; you're ensuring one exists. This demonstrates how patterns compose—Singleton for the EventBus, Observer for event distribution through that bus. Covered in Lesson 5's multi-pattern integration.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "When would composition be wrong and inheritance be the right choice?",
      options: [
        "When avoiding fragile base class problem is the primary design goal",
        "When you need runtime flexibility to swap components and behaviors",
        "When the relationship is has-a and delegation is the natural pattern",
        "When there's a true is-a relationship and shared interface matters"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 4. Inheritance is appropriate for genuine is-a relationships where types share a fundamental interface: Circle is-a Shape, ChatAgent is-a Agent. Polymorphism through inheritance enables treating different types uniformly. Inheritance makes sense when shared interface and behavioral substitutability matter. Option 2 favors composition—runtime swapping is easier with composed objects. Option 3 describes composition (has-a with delegation). Option 4 favors composition—fragile base class problem is an anti-pattern of inheritance. The key: use inheritance for true type hierarchies with substitutability, composition for capability aggregation. This design judgment is explored throughout Lesson 3's composition-over-inheritance discussion.",
      source: "Lesson 3: Composition Over Inheritance and Code Organization"
    },
    {
      question: "You ask AI to implement multiple comparison operators for a Task class. What decorator can reduce the implementation burden?",
      options: [
        "@property.comparison automatically generates all six comparison operators from __eq__ alone",
        "@functools.total_ordering fills in missing comparison methods from __eq__ and __lt__",
        "@dataclass automatically implements comparisons based on field order without code",
        "@abstractmethod validates that all comparison methods are properly implemented together"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. The @functools.total_ordering decorator is a built-in Python tool that generates __le__, __gt__, __ge__ from just __eq__ and __lt__. You implement two methods, Python synthesizes the other four. This reduces boilerplate and ensures consistency. Option 2 invents a non-existent decorator—@property is for attribute access, not comparisons. Option 3 is partially true—@dataclass can generate comparisons with order=True, but the question asks about a decorator for custom comparison logic, and total_ordering is the answer. Option 4 misunderstands @abstractmethod—it enforces implementation, doesn't generate it. Understanding total_ordering simplifies comparison implementation, mentioned in Lesson 4's comparison methods section.",
      source: "Lesson 4: Special Methods (Magic Methods)"
    },
    {
      question: "What's the primary advantage of using a Protocol over an ABC for defining agent interfaces?",
      options: [
        "Protocol enables structural typing without requiring explicit inheritance from base",
        "Protocol provides better runtime performance because it skips method validation checks",
        "Protocol automatically implements missing methods unlike ABC which requires manual implementation",
        "Protocol works in Python 2.7 while ABC requires Python 3.4 or later"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. typing.Protocol enables structural subtyping (duck typing with type checking). If a class has the right methods (process(), get_status()), it satisfies the Protocol—no inheritance required. ABCs require explicit inheritance (class ChatAgent(Agent)). Protocol gives duck typing's flexibility with type checker validation. Option 2 is false—Protocols are for static type checking (mypy), not runtime performance. Option 3 is backwards—neither auto-implements methods; both require manual implementation. Option 4 is wrong—Protocol was added in Python 3.8; ABC has been in Python much longer (2.6+). Protocols bridge duck typing and type safety, discussed briefly in Lesson 2's polymorphism alternatives.",
      source: "Lesson 2: Polymorphism and Duck Typing"
    },
    {
      question: "In a Factory with agent type registry, what's the benefit of allowing agent classes to self-register?",
      options: [
        "Self-registration enables agents to be created without constructor parameters required",
        "Self-registration automatically validates agent ABC compliance at import time checks",
        "Factory memory usage is reduced by lazy-loading only registered agents from",
        "New agents can be added by importing files, no factory modification needed"
      ],
      correctOption: 3,
      explanation: "The correct answer is option 4. Self-registration pattern: each agent file has AgentFactory.register('chat', ChatAgent) at the bottom. When the file is imported, the agent registers itself. To add new agent types, just create the file and import it—no factory modification needed. This follows the open/closed principle perfectly. Option 2 is false—registration and ABC validation are separate concerns. Option 3 misunderstands the pattern—memory usage is unaffected; all code loads on import anyway. Option 4 is irrelevant—self-registration is about adding to the factory registry, not about constructor parameters. Self-registration enables truly pluggable architecture, an advanced Factory pattern technique mentioned in Lesson 5.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "When implementing __iter__ and __next__ for a custom collection, what's a common mistake that breaks the iterator protocol?",
      options: [
        "Raising StopIteration too early prevents the last value from being yielded",
        "Implementing __next__ without __iter__ because Python infers __iter__ from __next__ automatically",
        "Forgetting to return self from __iter__ causes iteration to fail",
        "Using return instead of yield in __next__ works but is slower"
      ],
      correctOption: 2,
      explanation: "The correct answer is option 3. The iterator protocol requires __iter__ to return an iterator object (usually self). If __iter__ returns None or nothing, for loops fail with TypeError: iter() returned non-iterator. This is a common beginner mistake. Option 2 is false—Python doesn't infer __iter__; you must implement both. Option 3 misunderstands the protocol—StopIteration should be raised after the last value is yielded, not before. Option 4 confuses generators (which use yield) with manual iterators (which use return in __next__ to return values and raise StopIteration when done). Understanding the complete iterator protocol prevents these mistakes, covered in Lesson 4's iteration section.",
      source: "Lesson 4: Special Methods (Magic Methods)"
    },
    {
      question: "You're refactoring tight coupling in agent creation where every module directly imports and instantiates ChatAgent, CodeAgent, DataAgent. Which pattern addresses this?",
      options: [
        "Factory pattern centralizes agent creation, modules request agents from factory",
        "Singleton pattern ensures each module gets the same agent instances always",
        "Observer pattern eliminates imports by using event-based agent creation instead",
        "Strategy pattern provides different agent creation algorithms based on module needs"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. Factory pattern solves tight coupling to concrete classes. Instead of from agents import ChatAgent; agent = ChatAgent(), modules do from factory import AgentFactory; agent = AgentFactory.create_agent('chat'). Modules depend only on the factory interface, not specific agent classes. This decoupling enables changing agent implementations without modifying all consumer code. Option 2 addresses instance sharing, not creation coupling. Option 3 is convoluted—Observer is for communication, not creation decoupling. Option 4 misapplies Strategy—creation logic isn't the algorithm that varies. Factory is the standard decoupling pattern for object creation, thoroughly explained in Lesson 5.",
      source: "Lesson 5: Design Patterns (Capstone)"
    },
    {
      question: "What's the key difference between aggregation and composition in terms of object ownership semantics?",
      options: [
        "Aggregation uses pass-by-value while composition uses pass-by-reference in Python",
        "Composition implies strong ownership where container owns contained object",
        "Composition allows shared objects across containers while aggregation enforces uniqueness",
        "Aggregation is faster because it uses weak references for garbage collection"
      ],
      correctOption: 1,
      explanation: "The correct answer is option 2. Composition means the container creates and owns the contained object (Car creates its Engine)—strong coupling, shared lifetime. Aggregation means the container references but doesn't own the object (University references Departments)—weak coupling, independent lifetimes. When a Car is destroyed, its Engine is destroyed. When a University closes, Departments can continue independently. Option 2 is nonsense—Python always uses references, not value copying, and that's orthogonal to composition/aggregation semantics. Option 3 is backwards—composition typically means single ownership, aggregation allows sharing. Option 4 confuses implementation details with semantic relationships. Understanding ownership semantics guides architectural decisions, explained in Lesson 3.",
      source: "Lesson 3: Composition Over Inheritance and Code Organization"
    },
    {
      question: "When would you implement __repr__ but not __str__ for a class?",
      options: [
        "When developer debugging representation is sufficient and user display is not",
        "When the class is immutable and therefore __repr__ covers both uses",
        "When performance is critical and __str__ overhead is too expensive for",
        "Never, Python requires both or neither to maintain consistency between them"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. If a class is primarily internal (not user-facing), implementing just __repr__ makes sense. Python falls back to __repr__ when __str__ is missing, so print(obj) will use __repr__. Implementing only __repr__ that returns recreatable representation (like Point(x=3, y=5)) serves both debugging and basic display needs. Option 2 confuses immutability with string representation needs—immutability doesn't determine whether you need __str__. Option 3 is irrelevant—__str__ performance overhead is negligible. Option 4 is false—you can implement only __repr__, and Python uses it for both contexts. Understanding when __repr__ alone suffices simplifies class design, mentioned in Lesson 4.",
      source: "Lesson 4: Special Methods (Magic Methods)"
    },
    {
      question: "In Observer pattern, what problem arises if observers directly modify the subject during notification?",
      options: [
        "Iterating over observer list while modifying it causes runtime errors",
        "Subject state becomes inconsistent because notifications are out of order",
        "Observer pattern requires immutable subjects so modification is prevented automatically",
        "Python's GIL prevents modifications during iteration for thread safety enforcement"
      ],
      correctOption: 0,
      explanation: "The correct answer is option 1. If an observer's update() method calls subject.detach(self) during notification, the subject modifies its _observers list while iterating over it in notify(). This causes RuntimeError: list changed during iteration. Solutions: (1) iterate over a copy of the list, (2) collect detach requests and apply after iteration, or (3) use a more sophisticated observer management mechanism. Option 2 misidentifies the problem—state consistency is a separate concern from iteration errors. Option 3 is false—Observer pattern has no immutability requirement. Option 4 misunderstands GIL—it doesn't prevent this error; the issue is mutating a list during iteration, not thread safety. This is a classic Observer implementation gotcha, mentioned in Lesson 5.",
      source: "Lesson 5: Design Patterns (Capstone)"
    }
  ]}
/>
