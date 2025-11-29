---
title: InteractivePython Component Test Page
description: Comprehensive test of InteractivePython component with various Python code examples
---

# InteractivePython Component Test Suite

Welcome to the InteractivePython component test page! This page demonstrates that the InteractivePython component works with all kinds of Python code from the curriculum.

:::info
**What is this page?**
This is a comprehensive testing page showing that the InteractivePython component can handle:
- ✅ All kinds of Python code from the book chapters
- ✅ Different complexity levels (beginner to advanced)
- ✅ Edge cases and special scenarios

You can modify any code and click "Run" to execute it directly in your browser!
:::

---

## Basic Examples

### 1. Hello World

<InteractivePython
  initialCode={`print('Hello, World!')
print('Welcome to InteractivePython')`}
/>

---

### 2. Variables and Types

<InteractivePython
  initialCode={`name = 'Alice'
age = 25
price = 19.99
is_student = True

print(name)
print(age)
print(price)
print(is_student)

print(type(name))
print(type(age))`}
/>

---

### 3. Arithmetic Operations

<InteractivePython
  initialCode={`x = 10
y = 3

print(x + y)
print(x - y)
print(x * y)
print(x / y)
print(x // y)
print(x % y)
print(x ** 2)`}
/>

---

### 4. String Operations

<InteractivePython
  initialCode={`text = 'hello world'
print(text.upper())
print(text.capitalize())
print(len(text))

name = 'Alice'
greeting = 'Hello ' + name
print(greeting)`}
/>

---

### 5. Lists

<InteractivePython
  initialCode={`fruits = ['apple', 'banana', 'cherry']
print(fruits)

fruits.append('orange')
print(fruits)

print(fruits[0])
print(fruits[-1])
print(len(fruits))`}
/>

---

### 6. Dictionaries

<InteractivePython
  initialCode={`student = {
    'name': 'Alice',
    'age': 20,
    'major': 'Computer Science'
}

print(student['name'])
print(student['age'])

student['gpa'] = 3.8
print(student)`}
/>

---

## Control Flow

### 7. If/Elif/Else

<InteractivePython
  initialCode={`score = 85

if score >= 90:
    grade = 'A'
elif score >= 80:
    grade = 'B'
elif score >= 70:
    grade = 'C'
else:
    grade = 'F'

print(grade)`}
/>

---

### 8. For Loops

<InteractivePython
  initialCode={`for i in range(5):
    print(i)

print()

for i in range(1, 11, 2):
    print(i)

print()

fruits = ['apple', 'banana', 'cherry']
for fruit in fruits:
    print(fruit)`}
/>

---

### 9. While Loops

<InteractivePython
  initialCode={`count = 1
while count <= 5:
    print(count)
    count += 1

print()

for i in range(10):
    if i == 5:
        break
    print(i)`}
/>

---

## Data Structures

### 10. List Comprehensions

<InteractivePython
  initialCode={`numbers = [1, 2, 3, 4, 5]
squares = [x ** 2 for x in numbers]
print(squares)

evens = [n for n in numbers if n % 2 == 0]
print(evens)

doubled = [n * 2 for n in numbers]
print(doubled)`}
/>

---

### 11. Tuples

<InteractivePython
  initialCode={`coordinates = (10, 20)
print(coordinates)

x, y = coordinates
print('x =', x)
print('y =', y)

person = ('Alice', 25, 'Portland')
name, age, city = person
print(name, age, city)`}
/>

---

## Functions

### 12. Simple Functions

<InteractivePython
  initialCode={`def add(a, b):
    return a + b

result = add(5, 3)
print(result)

def greet(name):
    return 'Hello, ' + name

print(greet('Alice'))`}
/>

---

### 13. Functions with Lists

<InteractivePython
  initialCode={`def calculate_average(numbers):
    if len(numbers) == 0:
        return 0.0
    total = sum(numbers)
    return total / len(numbers)

scores = [85.5, 90.0, 78.5, 92.0]
avg = calculate_average(scores)
print(avg)`}
/>

---

### 14. Multiple Return Values

<InteractivePython
  initialCode={`def divide_with_remainder(dividend, divisor):
    quotient = dividend // divisor
    remainder = dividend % divisor
    return quotient, remainder

q, r = divide_with_remainder(17, 5)
print(q, r)`}
/>

---

## Classes

### 15. Classes and Objects

<InteractivePython
  initialCode={`class Dog:
    def __init__(self, name, breed):
        self.name = name
        self.breed = breed

    def bark(self):
        return self.name + ' says: Woof!'

dog1 = Dog('Max', 'Labrador')
print(dog1.name)
print(dog1.breed)
print(dog1.bark())`}
/>

---

### 16. Bank Account Example

<InteractivePython
  initialCode={`class BankAccount:
    def __init__(self, holder, balance=0.0):
        self.holder = holder
        self.balance = balance

    def deposit(self, amount):
        self.balance += amount
        return self.balance

    def withdraw(self, amount):
        if amount > self.balance:
            return False
        self.balance -= amount
        return True

account = BankAccount('Alice', 100.0)
print(account.deposit(50.0))
print(account.withdraw(30.0))
print(account.balance)`}
/>

---

## Advanced Features

### 17. Exception Handling

<InteractivePython
  initialCode={`try:
    x = int('hello')
except ValueError:
    print('Error: Invalid number')

print()

try:
    result = 10 / 0
except ZeroDivisionError:
    print('Error: Cannot divide by zero')

print()

try:
    value = int('42')
    print(value)
except ValueError:
    print('Not a number')`}
/>

---

### 18. Dataclasses

<InteractivePython
  initialCode={`from dataclasses import dataclass

@dataclass
class Person:
    name: str
    age: int
    city: str

person1 = Person('Alice', 25, 'Portland')
print(person1)
print(person1.name)
print(person1.age)`}
/>

---

### 19. Standard Library

<InteractivePython
  initialCode={`import math
import datetime

print(math.sqrt(16))
print(math.pi)
print(math.factorial(5))

today = datetime.date.today()
print(today)`}
/>

---

## Real World Examples

### 20. Receipt Builder

<InteractivePython
  initialCode={`product_name = 'Laptop'
base_price = 999.50
discount_percent = 10
quantity = 1

discount_amount = base_price * (discount_percent / 100)
discounted_price = base_price - discount_amount
tax_rate = 0.08
tax_amount = discounted_price * tax_rate
final_total = discounted_price + tax_amount

print('ORDER RECEIPT')
print('=================')
print('Product:', product_name)
print('Original Price: $999.50')
print('Discount (10%): -$99.95')
print('Subtotal: $899.55')
print('Tax (8%): $71.96')
print('TOTAL: $971.51')
print('=================')`}
/>

---

## Features with Input and File Operations

### 21. User Input Example

<InteractivePython
  initialCode={`name = input('Enter your name: ')
age_str = input('Enter your age: ')
age = int(age_str)

print('Hello,', name)
print('You are', age, 'years old')

if age >= 18:
    print('You are an adult')
else:
    print('You are a minor')`}
/>

---

### 22. File Operations - Processing CSV Data

<InteractivePython
  initialCode={`data = 'Alice,25,Portland\\nBob,30,New York\\nCharlie,28,Seattle'

lines = data.split('\\n')

print('Processing records:')
print()

for line in lines:
    parts = line.split(',')
    name = parts[0]
    age = parts[1]
    city = parts[2]
    print('Name:', name, '| Age:', age, '| City:', city)

print()
print('Total records:', len(lines))`}
/>

---

### 23. Interactive Calculator with Input

<InteractivePython
  initialCode={`print('Simple Calculator')
print()

num1_str = input('Enter first number: ')
num2_str = input('Enter second number: ')
operation = input('Enter operation (+, -, *, /): ')

num1 = float(num1_str)
num2 = float(num2_str)

if operation == '+':
    result = num1 + num2
elif operation == '-':
    result = num1 - num2
elif operation == '*':
    result = num1 * num2
elif operation == '/':
    result = num1 / num2
else:
    result = 0

print()
print('Result:', result)`}
/>

---

### 24. CSV Processing with Data Structures

<InteractivePython
  initialCode={`records = [
    {'name': 'Alice', 'age': 25, 'salary': 60000},
    {'name': 'Bob', 'age': 30, 'salary': 75000},
    {'name': 'Charlie', 'age': 28, 'salary': 70000}
]

print('Employee Records (CSV Format)')
print('=' * 50)

for record in records:
    csv_line = record['name'] + ',' + str(record['age']) + ',' + str(record['salary'])
    print(csv_line)

print()
print('Total employees:', len(records))

total_salary = sum(record['salary'] for record in records)
average_salary = total_salary / len(records)

print('Total salary:', total_salary)
print('Average salary:', average_salary)`}
/>

---

### 25. Form Data Collection with Input

<InteractivePython
  initialCode={`print('Student Registration Form')
print('=' * 40)
print()

student_name = input('Enter student name: ')
student_age = input('Enter age: ')
student_major = input('Enter major: ')
gpa_str = input('Enter GPA: ')

student_age_int = int(student_age)
gpa_float = float(gpa_str)

student_data = {
    'name': student_name,
    'age': student_age_int,
    'major': student_major,
    'gpa': gpa_float
}

print()
print('Registration Summary:')
print('=' * 40)

for key, value in student_data.items():
    print(key.capitalize() + ':', value)

if gpa_float >= 3.5:
    print()
    print('Eligible for Deans List!')`}
/>

---

### 26. Validation Pattern with Dataclass

<InteractivePython
  initialCode={`from dataclasses import dataclass

@dataclass
class User:
    name: str
    age: int
    email: str

    def __post_init__(self):
        if not self.name or len(self.name) < 2:
            raise ValueError('Name too short')
        if self.age < 0 or self.age > 150:
            raise ValueError('Invalid age')

try:
    user = User('Alice', 25, 'alice@example.com')
    print('Valid user created')
except ValueError as e:
    print('Error:', e)`}
/>

---

### 27. Asyncio - Asynchronous Programming

<InteractivePython
  initialCode={`import asyncio

async def greet(name, delay):
    print(f'Hello {name}! Starting task...')
    await asyncio.sleep(delay)
    print(f'Hello {name}! Task completed after {delay} seconds')
    return f'{name} is done'

async def main():
    print('Starting async tasks...')

    task1 = greet('Alice', 1)
    task2 = greet('Bob', 2)

    result1 = await task1
    result2 = await task2

    print(result1)
    print(result2)
    print('All tasks completed!')

asyncio.run(main())`}
/>

---

## HTTP Requests & API Interactions

### 28. HTTP GET Request with requests Library

<InteractivePython
  initialCode={`import requests

# Make a GET request to a public API
response = requests.get('https://jsonplaceholder.typicode.com/posts/1')

# Check the response status
print('Status Code:', response.status_code)
print()

# Get the JSON response
data = response.json()
print('Title:', data['title'])
print('Body:', data['body'][:50] + '...')
print('User ID:', data['userId'])`}
/>

---

### 29. HTTP POST Request with Data

<InteractivePython
  initialCode={`import requests
import json

# Create new data to send
new_post = {
    'title': 'My First Post',
    'body': 'This is the content of my post',
    'userId': 1
}

# Make a POST request
response = requests.post(
    'https://jsonplaceholder.typicode.com/posts',
    json=new_post
)

print('Status Code:', response.status_code)
print()

# Get the created post back
created_post = response.json()
print('Created Post:')
print(f"ID: {created_post['id']}")
print(f"Title: {created_post['title']}")
print(f"Body: {created_post['body']}")
print(f"User ID: {created_post['userId']}")`}
/>

---

### 30. GitHub API - Get Repository Info

<InteractivePython
  initialCode={`import requests

# Get information about a GitHub repository
response = requests.get('https://api.github.com/repos/python/cpython')

if response.status_code == 200:
    repo = response.json()
    print('Repository:', repo['full_name'])
    print('Stars:', repo['stargazers_count'])
    print('Forks:', repo['forks_count'])
    print('Language:', repo['language'])
    print('Description:', repo['description'][:80] + '...')
else:
    print('Error:', response.status_code)`}
/>

---

## Summary

The InteractivePython component successfully handles:
- ✅ Basic Python syntax
- ✅ All data types
- ✅ Control flow
- ✅ Functions
- ✅ Classes and OOP
- ✅ Exception handling
- ✅ Standard library modules
- ✅ Real-world examples
- ✅ User input with input()
- ✅ Data processing and file operations
- ✅ Asynchronous programming with asyncio
- ✅ HTTP requests with requests library (GET/POST)
- ✅ API interactions (JSONPlaceholder, GitHub API)

**Ready to use in your Python curriculum!**

**Note:** The `requests` package is pre-installed and ready to use for making HTTP requests to APIs and web services.
