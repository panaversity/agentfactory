**@Author: Arif Kasim Rozani - (Team Operation Badar)**

# **Best Practices in OOP**

Writing clean and maintainable Object-Oriented Programming (OOP) code is essential for building scalable and robust applications. Following best practices like the **SOLID** **principles** can help you achieve this.

## **Nested Models**

```python
class Address(BaseModel):
    city: str
    country: str

class UserProfile(BaseModel):
    name: str
    address: Address  # Nested model
```

### **Usage**

```python
profile = UserProfile(
    name="Alice",
    address={"city": "New York", "country": "USA"}  # Auto-converted to Address
)
print(profile.address.city)  # Output: "New York"
```

## **JSON Serialization**

### **Convert Model ↔ JSON**

```python
user = User(name="Bob", age=30)

# Model → JSON
user_json = user.model_dump_json()
print(user_json)
# Output: '{"name":"Bob","age":30,"is_active":true}'

# JSON → Model
new_user = User.model_validate_json('{"name":"Charlie","age":40}')
print(new_user.name)  # Output: "Charlie"
print(type(new_user))  # Output: <class 'int'>
```

## **Settings Management**

Pydantic is great for **config files** (e.g., `.env` files). `Use VSCode`

### **Example: Environment Variables**

```python
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    api_key: str = "123123" # run in VSCode without default value
    debug: bool = False

    class Config:
        env_file = ".env"  # Load from .env file
```

### **.env File**

```bash
API_KEY=my-secret-key
DEBUG=True
```

### **Usage**

```python
settings = Settings()
print(settings)  # Output: "my-secret-key"
```

## **Using Validators for Custom Rules**

You can define **custom validation logic** using `@validator`.

### **Example: Validate Email Format**

```python
from pydantic import BaseModel, EmailStr, field_validator

class User(BaseModel):
    name: str
    email: EmailStr

    @field_validator("name")
    def name_must_not_be_empty(cls, value):
        if not value.strip():
            raise ValueError("Name cannot be empty!")
        return value

# Valid user
user = User(name="David", email="david@example.com")
print(user)
```

```python
# Invalid user (empty name)
User(name=" ", email="invalid_email")
```

## **Recap: Key Pydantic Features**

| Feature            | Example                          |
|--------------------|----------------------------------|
| Type Validation    | age: int → Fails if age="twenty" |
| Default Values     | is_active: bool = True           |
| Nested Models      | address: Address                 |
| JSON Serialization | .model_dump_json()               |
| Env Config         | class Settings(BaseSettings)     |

## **Conclusion**

Pydantic ensures **your data is always correct** while keeping code clean and readable. It's a must-have for **APIs, configs, and data pipelines**!

📌 **Key Takeaway:**  
**Pydantic = Type hints + Validation + Serialization** 🚀

# **Generics in Python 3.12: A Complete Beginner's Guide**

Python 3.12 introduced several improvements to generics, making them more powerful and easier to use. This tutorial covers everything you need to know about generics in Python 3.12, with clear examples and practical applications.

Generics in Python 3.12 are part of the type hinting system, which has evolved significantly since Python 3.5 introduced type annotations via PEP 484. Generics allow developers to write reusable, type-safe code by defining classes, functions, or data structures that can operate on different types while maintaining static type checking. With Python 3.12, generics have been refined with improvements in syntax and runtime behavior, building on earlier enhancements like those in PEP 695 (introduced in Python 3.12).

## **Table of Contents**

1.  What Are Generics?
2.  New Features in Python 3.12
3.  Basic Generic Functions
4.  Generic Classes
5.  TypeVar and Bounds
6.  Generic Collections
7.  Overloading with `@overload`
8.  Real-World Examples
9.  Best Practices

## **1. What Are Generics?**

Generics enable you to parameterize types, meaning you can create a blueprint for a class or function that works with any type (or a constrained set of types) specified by the user of that code. For example, a generic List can hold integers, strings, or custom objects, and the type checker ensures consistency without needing separate implementations for each type.

In Python, generics are primarily used with the typing module (e.g., List, Dict, Optional) and, starting with Python 3.12, with new syntax from PEP 695, such as type statements and TypeVar refinements. They don't change runtime behavior directly—Python remains dynamically typed—but they enhance static analysis tools like mypy, pyright, or IDEs.

## **2. Why Are Generics Needed?**

1.  **Code Reusability**: Without generics, you'd need to write duplicate code for different types. For instance, a function to process a list of integers and a list of strings would require two separate implementations. Generics let you write one function that works for any type.
2.  **Type Safety**: Generics allow static type checkers to catch errors before runtime. For example, if a function expects a List[int] but receives a List[str], the type checker flags it.
3.  **Abstraction**: They enable abstract data structures (e.g., stacks, queues, or trees) that can work with any data type, making libraries more flexible and maintainable.
4.  **Interoperability**: Modern Python codebases often integrate with typed libraries or frameworks (e.g., FastAPI, Pydantic). Generics ensure compatibility and clarity in such ecosystems.

## **3. How Do Generics Work in Python 3.12?**

Python 3.12 introduces a cleaner syntax for generics via PEP 695, reducing reliance on the typing module's older constructs.

❌ Example: A Function Without Generics

```python
def double(value):
    return value * 2

print(double(5))     # ✅ Works (int → 10)
print(double("Hi"))  # ✅ Works (str → 'HiHi')
print(double([1, 2])) # ✅ Works ([1,2,1,2])
```

📌 **Issue:**

-   The function accepts **any data type**.
    
-   While **multiplication works for `int` and `str`**, unintended behavior can happen for **custom types**.
    
-   There's **no type safety**, so errors may **only appear at runtime**.

### **🔹 Solution: Use Generics for Type Safety**

Generics allow **type flexibility** while ensuring **type consistency**.

### **jupyter notebook use:**

`This automatically runs mypy on every cell before it is executed.`

This is just for convenience to show the mypy output for this talk.

```python
if "google.colab" in str(get_ipython()):
    !pip install nb-mypy -qqq
%load_ext nb_mypy
```

```python
from typing import TypeVar

T = TypeVar("T", int, float)  # Restrict to int & float

def double(value: T) -> T:
    return value * 2

print(double(5))     # ✅ Works (int → 10)
print(double(3.14))  # ✅ Works (float → 6.28)
print(double("Hi"))  # ❌ TypeError: str not allowed
```

```python
%%writefile test_mypy.py

def multiply(x: float | int, y: float | int) -> float  | int: #Create the data type according to the values given float or int
    return x * y

result: float = multiply(3, 4.5) #Change the value to string data tyoe to see error when validating with mypy
print(type(result)," = ", result)
```

```python
!mypy test_mypy.py
```

## **For VSCode Use:**

**A - By Command:**
  1.  Open terminal window type 'uv add mypy' or 'pip install mypy'
  2.  Validate class typing by using command in terminal window 'mypy class_name.py'

**B - By Installing an Extension:**
  1.  Click on extensions icon on the left side search 'Mypy Type Checker' by Microsoft then install it.
  2.  Now the IDE it self is capable to figure out type error in source code while you are writing Python code.

## **Example With Generics**

```python
from typing import TypeVar, List

T = TypeVar('T', bound=int | str)  # Declare a generic type

def first_item(items: List[T]) -> T:  # Returns the correct type
    return items[0]

# print(first_item([1, 2, 3]))       # Output: int
# print(first_item(["a", "b"]))      # Output: str
print(first_item([1.0, 2.0, 3.0])) # Uncomment to see error
```

## **4. New Features in Python 3.12**

| Feature               | Description                            | Example                              |
|-----------------------|----------------------------------------|--------------------------------------|
| Simpler Syntax        | No need for from typing import TypeVar | def func[T](): ...                   |
| Type Parameter Lists  | Declare generics inline                | class Stack[T]: ...                  |
| TypeVar Improvements  | Easier bounds and constraints          | T = TypeVar('T', bound=int)          |
| Better Error Messages | More readable type errors              | TypeError: Expected 'int', got 'str' |

# ⚠ **Important Note:**

**In order to use new generic type syntax introduced in Python 3.12 we need to upgrade Python enviroment of Google Colab Notebook, below are the steps how to do that.**

```python
%%python --version
```

## **Step-by-Step Guide to Install Python 3.12 in Google Colab**

[How to update Google Colab's Python version](https://saturncloud.io/blog/how-to-update-google-colabs-python-version/)

```python
# Install python 3.12
!apt-get install python3.12
```

```python
# Change default python3 to 3.12
!sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1
!sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.12 2

# Confirm version
!python3 --version
# Python 3.12.1
```

```python
!python3.12 --version
```

## **Problem:**

Google Colab's Python environment might not be fully configured to recognize the new generic type syntax introduced in Python 3.12, despite showing the correct version. This can lead to Syntax errors when you use new Syntax e.g.`swap[T]`.

## **Solution:**

While Colab might indicate it's running Python 3.12, it's actually still referencing the old environment files in the background where generic type Syntax is invalid.

**To solve this issue we will use VSCode, the GitHub link for the new Generic Syntax is here.**

[**Generic: Source Code**](https://github.com/panaversity/learn-modern-ai-python/tree/main/00_python_colab/Lesson_Source_Code/generics)

## **5. Basic Generic Functions**

### **Example: Swap Two Values** **(Colab ERROR: Not recognizing Python Version 3.12)**

```python
def swap[T](a: T, b: T) -> tuple[T, T]:
    return b, a

x, y = swap(10, 20)    # ✅ (int, int)
a, b = swap("A", "B")  # ✅ (str, str)
```

### **Example: Swap Two Values**

```python
def swap[T](a: T, b: T) -> tuple[T, T]:
    return b, a

x, y = swap(10, 20)    # ✅ (int, int)
a, b = swap("A", "B")  # ✅ (str, str)
```

### **Multiple Generic Types**

```python
def get_value[K, V](d: dict[K, V], key: K) -> V:
    return d[key]

person = {"name": "Alice", "age": 30}
print(get_value(person, "name"))  # Returns "Alice" (str)
```

## **6. Generic Classes**

### **Example: Stack Data Structure**

```python
class Stack[T]:
    def __init__(self):
        self.items: list[T] = []
    
    def push(self, item: T) -> None:
        self.items.append(item)
    
    def pop(self) -> T:
        return self.items.pop()

# Usage
int_stack = Stack[int]()
int_stack.push(10)
print(int_stack.pop())  # ✅ Returns int

str_stack = Stack[str]()
str_stack.push("hello")
print(str_stack.pop())  # ✅ Returns str
```

## **7. TypeVar and Bounds**

### **Restricting Types (`bound=`)**

```python
from typing import TypeVar

Numeric = TypeVar('Numeric', bound=int | float)  # Only int or float

def add(a: Numeric, b: Numeric) -> Numeric:
    return a + b

print(add(5, 10))      # ✅ Valid
print(add(3.14, 2.71)) # ✅ Valid
print(add("a", "b"))   # ❌ Error
```

### **Upper Bounds (Inheritance)**

```python
from typing import TypeVar

class Animal: ...

class Dog(Animal): ...

A = TypeVar('A', bound=Animal)  # Must be Animal or subclass

def make_sound(animal: A) -> None:
    print("Sound!")

make_sound(Dog())  # ✅ Valid
```

## **8. Generic Collections**

Python 3.12 supports **inline generic syntax** for collections:

| Collection | Old Syntax | New Syntax (Python 3.12) |
|------------|------------|--------------------------|
| list       | List[T]    | list[T]                  |
| dict       | Dict[K, V] | dict[K, V]               |
| set        | Set[T]     | set[T]                   |

### **Example**

```python
names: list[str] = ["Alice", "Bob"]
ages: dict[str, int] = {"Alice": 25, "Bob": 30}
unique: set[int] = {1, 2, 3}
```

## **9. Overloading with `@overload`**

Define **multiple function signatures** for different input types:

**How it Works:**

1.  **Multiple Signatures:** You define multiple function signatures using `@overload`. These signatures specify the expected input types and return types for different cases.

2.  **Implementation:** You provide a single, general implementation of the function. This implementation typically uses type hints that are compatible with all the overloaded signatures.

3.  **Type Checker's Role:** The type checker uses the overloaded signatures to determine the correct return type based on the input types. The implementation is used at runtime.

```python
from typing import overload

@overload
def double(x: int) -> int: ...
@overload
def double(x: str) -> str: ...

def double(x: int | str) -> int | str:
    return x * 2

print(double(5))     # Returns 10 (int)
print(double("A"))   # Returns "AA" (str)
```

## **10. Real-World Examples**

### **Example 1: API Response Wrapper**

```python
class APIResponse[T, U]:
    def __init__(self, data: T, status: U):
        self.data = data
        self.status = status

# Usage
response = APIResponse[str, int]("Success!", 200)
print(response.data)  # Output: "Success!"

response2 = APIResponse[bool, float](True, 0.200)
print(response2.data)  # Output: True
```

### **Example 2: Caching System**

```python
class Cache[K, V]:
    def __init__(self):
        self.store: dict[K, V] = {}
    
    def set(self, key: K, value: V) -> None:
        self.store[key] = value
    
    def get(self, key: K) -> V:
        return self.store[key]

# Usage
cache = Cache[str, int]()
cache.set("count", 10)
print(cache.get("count"))  # Output: 10
```

## **11. Best Practices**

✅ **Use inline generics (`def func[T]()`) for simplicity**  
✅ **Apply `bound=` when restricting types**  
✅ **Prefer `list[T]` over `List[T]`**  
❌ **Avoid `Any` when generics can be used**

## **Final Thoughts**

Python 3.12 makes generics:  
✔ **Easier to write**  
✔ **More readable**  
✔ **Better for large codebases**
