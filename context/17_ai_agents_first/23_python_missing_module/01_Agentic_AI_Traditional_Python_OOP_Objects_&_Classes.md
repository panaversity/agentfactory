**@Author: Arif Kasim Rozani - (Team Operation Badar)**

# OOPS WHAT'S THAT...

# **Best Practices in OOP**

Writing clean and maintainable Object-Oriented Programming (OOP) code is essential for building scalable and robust applications. Following best practices like the **SOLID** **principles** can help you achieve this. Let's explore these principles and how to apply them in Python.

## **SOLID Principles in Python**

The **SOLID principles** are a set of five design principles that help developers write maintainable and scalable code. They are:

1.  **Single Responsibility Principle (SRP):**

    A class should have only one reason to change, meaning it should have only one responsibility.

2.  **Open/Closed Principle (OCP):**

    A class should be open for extension but closed for modification. You should be able to add new functionality without changing existing code.

3.  **Liskov Substitution Principle (LSP):**

    Objects of a superclass should be replaceable with objects of a subclass without affecting the correctness of the program.

4.  **Interface Segregation Principle (ISP):**

    Clients should not be forced to depend on interfaces they do not use. Instead of one large interface, create smaller, specific ones.

5.  **Dependency Inversion Principle (DIP):**

    High-level modules should not depend on low-level modules. Both should depend on abstractions. Abstractions should not depend on details. Details should depend on abstractions.

## **Example: Applying SOLID Principles**

Let's create a Python example to demonstrate how to apply these principles.

## **Single Responsibility Principle (SRP)**

```python
# Bad: One class with multiple responsibilities
class Report:
    def generate_report(self, data):
        # Generate report
        pass

    def save_report(self, file_path):
        # Save report to file
        pass

# Good: Separate responsibilities into different classes
class ReportGenerator:
    def generate_report(self, data):
        # Generate report
        pass

class ReportSaver:
    def save_report(self, report, file_path):
        # Save report to file
        pass
```

## **Open/Closed Principle (OCP)**

```python
# Bad: Modify existing code to add new functionality
class AreaCalculator:
    def calculate_area(self, shape):
        if shape.type == "circle":
            return 3.14 * shape.radius ** 2
        elif shape.type == "rectangle":
            return shape.length * shape.width

# Good: Extend functionality without modifying existing code
from abc import ABC, abstractmethod

class Shape(ABC):
    @abstractmethod
    def area(self):
        pass

class Circle(Shape):
    def __init__(self, radius):
        self.radius = radius

    def area(self):
        return 3.14 * self.radius ** 2

class Rectangle(Shape):
    def __init__(self, length, width):
        self.length = length
        self.width = width

    def area(self):
        return self.length * self.width

class AreaCalculator:
    def calculate_area(self, shape):
        return shape.area()
```

## **Liskov Substitution Principle (LSP)**

```python
# Bad: Subclass changes the behavior of the parent class
class Bird:
    def fly(self):
        pass

class Ostrich(Bird):
    def fly(self):
        raise NotImplementedError("Ostriches can't fly")

# Good: Subclass adheres to the behavior of the parent class
class Bird:
    def move(self):
        pass

class Sparrow(Bird):
    def move(self):
        print("Flying")

class Ostrich(Bird):
    def move(self):
        print("Running")
```

## **Interface Segregation Principle (ISP)**

```python
# Bad: One large interface
class Printer:
    def print_document(self):
        pass

    def scan_document(self):
        pass

    def fax_document(self):
        pass

# Good: Smaller, specific interfaces
class Printer:
    def print_document(self):
        pass

class Scanner:
    def scan_document(self):
        pass

class FaxMachine:
    def fax_document(self):
        pass
```

## **Dependency Inversion Principle (DIP)**

```python

# Bad: High-level module depends on low-level module
class LightBulb:
    def turn_on(self):
        pass

    def turn_off(self):
        pass

class Switch:
    def __init__(self):
        self.bulb = LightBulb()

    def operate(self):
        if condition:
            self.bulb.turn_on()
        else:
            self.bulb.turn_off()

# Good: Both depend on abstractions
from abc import ABC, abstractmethod

class Switchable(ABC):
    @abstractmethod
    def turn_on(self):
        pass

    @abstractmethod
    def turn_off(self):
        pass

class LightBulb(Switchable):
    def turn_on(self):
        pass

    def turn_off(self):
        pass

class Switch:
    def __init__(self, device: Switchable):
        self.device = device

    def operate(self):
        if condition:
            self.device.turn_on()
        else:
            self.device.turn_off()
```

## **Key Takeaways**

  * **Single Responsibility Principle (SRP):** Each class should have only one responsibility.

  * **Open/Closed Principle (OCP):** Classes should be open for extension but closed for modification.

  * **Liskov Substitution Principle (LSP):** Subclasses should be substitutable for their superclasses.

  * **Interface Segregation Principle (ISP):** Use smaller, specific interfaces instead of one large interface.
  * **Dependency Inversion Principle (DIP):** High-level and low-level modules should depend on abstractions.


By applying these **SOLID principles**, you can write clean, maintainable, and scalable OOP code in Python. 🚀

**Further resources:**

[SOLID principles: realpython.com](https://realpython.com/solid-principles-python/)

[SOLID principles: github.io](https://yakhyo.github.io/solid-python/)

# **Iterable**

In Python, Iterable is not a parent class but rather an abstract base class (ABC) defined in the collections.abc module. It serves as a protocol or interface that other classes can implement to indicate that they are iterable (i.e., they can be looped over using a for loop or other iteration constructs).

## **What is an Iterable?**

An iterable is any object that can return an iterator when the iter() function is called on it. The iterator is used to traverse through the elements of the iterable.

## **Parent Class Relationship**

The Iterable abstract base class is not a parent class in the traditional sense (like inheritance in object-oriented programming). Instead, it is used to define a protocol that other classes can adhere to by implementing the __iter__() method.

## **Example of Iterable Classes**

Many built-in Python classes are iterable because they implement the __iter__() method. These include:

1.  **Lists**: list
2.  **Tuples**: tuple
3.  **Strings**: str
4.  **Dictionaries**: dict
5.  **Sets**: set
6.  **Ranges**: range
7.  **Generators**: generator

## **How to Check if a Class is Iterable**

You can use the isinstance() function with collections.abc.Iterable to check if an object is iterable:

```python
from collections.abc import Iterable

# Check if built-in types are iterable
print("isinstance([1, 2, 3], Iterable) = ",isinstance([1, 2, 3], Iterable))  # True (list is iterable)
print('isinstance("hello", Iterable)   = ', isinstance("hello", Iterable))    # True (string is iterable)
print("isinstance(123, Iterable)       = ", isinstance(123, Iterable))        # False (integer is not iterable)
```

## **How to Make a Custom Class Iterable**

To make a custom class iterable, you need to implement the __iter__() method, which should return an iterator object. The iterator object must implement the __next__() method.

## **Example:**

```python
from collections.abc import Iterable, Iterator

class MyIterable(Iterable):
    def __init__(self, data):
        self.data = data

    def __iter__(self):
        return MyIterator(self.data)

class MyIterator(Iterator):
    def __init__(self, data):
        self.data = data
        self.index = 0

    def __next__(self):
        if self.index >= len(self.data):
            raise StopIteration
        value = self.data[self.index]
        self.index += 1
        print("Called: MyIterator.__next__")
        return value

# Usage
my_iterable = MyIterable([1, 2, 3])
for item in my_iterable:
    print("item : ",item)  # Output: 1, 2, 3
```

## **Key Points**

* Iterable is an abstract base class (ABC) from the collections.abc module.

* It defines a protocol for iterable objects by requiring the implementation of the __iter__() method.
* Many built-in Python classes (e.g., list, tuple, str, dict) are iterable because they implement this protocol.
* Custom classes can be made iterable by implementing the __iter__() method.

# **Object-Based Language vs. Object-Oriented Language**

## **Object-Based Language**

  * **Definition**: A language that supports objects (data structures with attributes and methods) and encapsulation (data hiding), but lacks key OOP features like inheritance and polymorphism.
  * **Features**:
    * Objects as instances with properties and methods.
    * Encapsulation (e.g., public/private access modifiers).
    * May include basic polymorphism (e.g., operator overloading).
  * Examples: JavaScript (prototype-based, but lacks classical inheritance), classic Visual Basic, Ada.

## **Object-Oriented Language**

  * **Definition**: A language that implements the four pillars of OOP:
    1.  **Encapsulation**: Hiding internal state and requiring interaction via methods.
    2.  **Inheritance**: Creating hierarchical relationships between classes (e.g., subclasses reusing parent class code).
    3.  **Polymorphism**: Allowing objects of different classes to respond to the same method (via inheritance or interfaces).
    4.  **Abstraction**: Simplifying complexity through abstract classes/interfaces.

  * Examples: Python, Java, C++, C#.

# **The Python's Object-Centric Nature**

# **Is Everything in Python an Object? YES!**

**Yes, in Python, absolutely everything is an object. This is a fundamental characteristic of the language and a core design principle.**

  * **Numbers**: Integers, floats, complex numbers are objects.
  * **Strings**: Textual data is represented as string objects.
  * **Lists**, Tuples, Dictionaries, Sets: These are built-in container types and are all objects.
  * **Functions**: Functions are first-class objects in Python. You can assign them to variables, pass them as arguments to other functions, and even return them from functions.
  * **Classes and Modules**: Classes themselves are objects (instances of metaclasses), and modules are also objects.
  * **Even None**: None, which represents the absence of a value, is an object of the NoneType class.
  * **Types/Classes**: In Python, types (like int, str, list) are also objects (they are instances of the metaclass type).

## **How to Verify:**

You can use the type() function in Python to check the type of any entity. It will always return a class (which is itself an object).

```python
>>> type(5)
<class 'int'>
>>> type("hello")
<class 'str'>
>>> type([1, 2, 3])
<class 'list'>
>>> def my_function():
...     pass
>>> type(my_function)
<class 'function'>
>>> class MyClass:
...     pass
>>> obj = MyClass()
>>> type(MyClass)
<class 'type'>  # Classes are instances of 'type' (metaclass)
>>> type(obj)
<class '__main__.MyClass'>
```

## **Why is this important in Python?**

  * **Consistency**: It creates a consistent and unified way to work with data and code. Everything behaves like an object, leading to a more predictable programming model.
  
  * **Flexibility**: Because functions and classes are objects, Python is highly dynamic and allows for powerful meta-programming techniques. You can inspect, modify, and create objects dynamically at runtime.
  * **Object-Oriented Programming**: This "everything is an object" nature is foundational to Python's object-oriented features. It makes it natural to work with classes, inheritance, and polymorphism, as all entities are treated as objects.

## **In summary:**

  * Object-based languages provide objects and some basic object-related features but lack the full suite of OOP principles.

  * Object-oriented languages fully embrace OOP by incorporating classes, encapsulation, abstraction, inheritance, and polymorphism, leading to better software design and organization.
  * **Python is a fully object-oriented language where everything is an object**, contributing to its flexibility, consistency, and power.

# **Pydantic Tutorial for Beginners: A Complete Guide**

Pydantic is a **Python library** for **data validation and settings management** using Python type hints. It ensures your data is always in the correct format, making it perfect for:  
✅ **API request/response validation**  
✅ **Config management**  
✅ **Data parsing & serialization**

## **Table of Contents**

1.  Installation
2.  Basic Model
3.  Field Validation
4.  Nested Models
5.  JSON Serialization
6.  Settings Management
7.  Real-World Example

## **1️⃣ Installation**

```python
%pip -q install pydantic pydantic_settings pydantic[email]
```

## **2️⃣ Basic Model**

### **Create a Simple Model**

```python
from pydantic import BaseModel

class User(BaseModel):
    name: str
    age: int
    is_active: bool = True  # Default value
```

### **Usage**

```python
user = User(name="Alice", age=25)
print(user.name)  # Output: "Alice"
print(user.age)   # Output: 25
print(user.is_active)  # Output: True (default)
```

### **Automatic Validation**

```python
# ❌ Error (age must be an int)
user = User(name="Bob", age="twenty")  # Raises ValidationError
```

## **3️⃣ Field Validation**

### **Using `Field` for Extra Rules**

```python
from pydantic import BaseModel, Field

class Product(BaseModel):
    name: str
    price: float = Field(..., gt=0)  # Must be > 0
    category: str = Field(min_length=3, max_length=50)
```

### **Example**

```python
# ✅ Valid
product = Product(name="Laptop", price=999.99, category="Electronics")
print(product.model_dump_json)
```

```python
# ❌ Invalid (price <= 0)
product = Product(name="Phone", price=0, category="Gadgets")  # Raises error
```

## **4️⃣ Nested Models**

```python
class Address(BaseModel):
    city: str
    country: str

class UserProfile(BaseModel):
    name: str
```
