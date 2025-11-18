---
title: "Lists Part 2 — Mutability and Modification Methods"
chapter: 18
lesson: 3
duration_minutes: 210

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "List Mutation with Methods"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content"
    measurable_at_this_level: "Student can use append/extend/insert/remove/pop/clear with correct semantics and predict results"

  - name: "Method vs Function Distinction"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information"
    measurable_at_this_level: "Student understands .append() (method) vs len() (function) difference and knows when to use each"

  - name: "State Change Semantics"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Content"
    measurable_at_this_level: "Student can predict which operations modify in-place vs return new objects"

  - name: "Error Recognition"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Safety"
    measurable_at_this_level: "Student can interpret IndexError, TypeError, ValueError from method misuse and ask AI for explanations"

learning_objectives:
  - objective: "LO-002e: Use append(), extend(), insert() to add elements to lists"
    proficiency_level: "A2-B1"
    bloom_level: "Apply"
    assessment_method: "Code exercises adding elements with correct method choice"

  - objective: "LO-002f: Use remove(), pop(), clear() to delete elements from lists"
    proficiency_level: "A2-B1"
    bloom_level: "Apply"
    assessment_method: "Code exercises deleting elements with appropriate methods"

  - objective: "LO-002g: Choose between append() vs extend() based on semantics"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Scenario-based selection explaining why each method fits"

  - objective: "LO-004a: Call list methods correctly with proper arguments"
    proficiency_level: "A2-B1"
    bloom_level: "Apply"
    assessment_method: "Code examples demonstrating correct method usage and error handling"

cognitive_load:
  new_concepts: 7
  assessment: "Exactly 7 concepts (at A2-B1 limit): append, extend, insert, remove, pop, clear, method-vs-function. Focused on method semantics before advanced operations. ✓"

differentiation:
  extension_for_advanced: "Explore list methods beyond the core 6: count(), index(), reverse(), copy(). Compare behavior and implement advanced exercises."
  remedial_for_struggling: "Focus on append() and remove() first. Add extend() after mastery. Use concrete shopping cart example exclusively. Practice with simple lists."

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/001-part-4-chapter-18/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lists Part 2 — Mutability and Modification Methods

## What Does "Mutable" Mean in Action?

In Lesson 2, we learned that lists are **mutable**—they can be changed. Now let's see what that means practically. When you create a list, you don't just read from it like a locked book. You actively modify it: adding items, removing items, reorganizing. This is the power of lists.

Think of a list like a shopping cart. You start with:
```python
cart: list[str] = ["milk", "eggs"]
```

Then you perform actions:
- Add milk → `cart.append("milk")`
- Add multiple items at once → `cart.extend(["butter", "jam"])`
- Remove a specific item → `cart.remove("milk")`
- Empty the cart → `cart.clear()`

Each operation **changes the list itself**. This lesson teaches you six fundamental methods that modify lists and one critical distinction that will save you hours of debugging later.

---

## Concept 1: `append()` — Add One Item to the End

**What it does**: Adds a single item to the end of a list.

**Syntax**: `list.append(item)`

**Example**:
```python
cart: list[str] = ["milk", "eggs"]
cart.append("bread")
print(cart)  # ["milk", "eggs", "bread"]
```

The key insight: `append()` **modifies the original list**. It doesn't return a new list—it changes the list in-place and returns `None`.

```python
cart: list[str] = ["milk", "eggs"]
result = cart.append("bread")
print(result)   # None (not a list!)
print(cart)     # ["milk", "eggs", "bread"] (original list changed)
```

This trips up beginners who expect `result` to be the updated list. It's not. The list itself changed; that's enough.

#### 💬 AI Colearning Prompt

> "Show me what happens if I do `new_cart = cart.append('bread')`. Will `new_cart` be a list or something else? Why does `append()` work this way?"

---

## Concept 2: `extend()` — Add Multiple Items

**What it does**: Adds all items from an iterable (another list, tuple, string) to the end of the list.

**Syntax**: `list.extend(iterable)`

**Example**:
```python
cart: list[str] = ["milk", "eggs"]
cart.extend(["butter", "jam"])
print(cart)  # ["milk", "eggs", "butter", "jam"]
```

**Critical Distinction: `append()` vs `extend()`**

This is the most important semantic difference in list methods. Watch carefully:

```python
# Using append() with a list
cart1: list[str] = ["milk", "eggs"]
cart1.append(["butter", "jam"])
print(cart1)  # ["milk", "eggs", ["butter", "jam"]]
# You added a SINGLE ITEM: a list containing two items!

# Using extend() with a list
cart2: list[str] = ["milk", "eggs"]
cart2.extend(["butter", "jam"])
print(cart2)  # ["milk", "eggs", "butter", "jam"]
# You added TWO ITEMS: "butter" and "jam" individually
```

**`append()`** adds the argument as a single item (whatever it is).
**`extend()`** unpacks the argument and adds each item individually.

#### 🎓 Expert Insight

> In AI-native development, you don't memorize 47 list methods. You understand: "I'm adding ONE item (append) vs MANY items (extend)." That semantic distinction is gold. The syntax? AI fills it in.

#### 🚀 CoLearning Challenge

Ask your AI companion:

> "I have a shopping cart `['milk', 'eggs']` and I want to add `['butter', 'jam']` to it. Show me what happens if I use `append()` vs `extend()`. Why are the results different?"

**Expected Outcome**: You'll see the nested list from `append()` and understand why `extend()` unpacks items.

---

## Concept 3: `insert()` — Add Item at a Specific Position

**What it does**: Adds an item at a specific index (position) in the list.

**Syntax**: `list.insert(index, item)`

**Example**:
```python
cart: list[str] = ["milk", "eggs", "bread"]
cart.insert(1, "yogurt")  # Insert at position 1
print(cart)  # ["milk", "yogurt", "eggs", "bread"]
```

The item at index 1 ("eggs") shifts to index 2. Everything after the insertion point shifts right.

What if you insert at an index beyond the list length?

```python
cart: list[str] = ["milk", "eggs"]
cart.insert(100, "bread")  # Insert at position 100 (doesn't exist)
print(cart)  # ["milk", "eggs", "bread"] — added to the end!
```

Python is forgiving: if the index is too high, it adds to the end. If the index is negative, it counts from the end:

```python
cart: list[str] = ["milk", "eggs", "bread"]
cart.insert(-1, "yogurt")  # Insert before the last item
print(cart)  # ["milk", "eggs", "yogurt", "bread"]
```

#### ✨ Teaching Tip

> Use Claude Code to explore edge cases: "What happens if I insert at index -100 in a 3-item list? Why does Python handle it that way?"

---

## Concept 4: `remove()` — Delete by Value

**What it does**: Deletes the **first item with a specific value**.

**Syntax**: `list.remove(value)`

**Example**:
```python
cart: list[str] = ["milk", "eggs", "bread", "milk"]
cart.remove("milk")
print(cart)  # ["eggs", "bread", "milk"] — only the FIRST "milk" removed
```

**Critical**: `remove()` looks for the **value**, not the index. It removes only the first match.

What if the value doesn't exist?

```python
cart: list[str] = ["milk", "eggs"]
cart.remove("butter")  # ValueError: list.remove(x): x not in list
```

This raises a `ValueError`. You must be sure the item exists, or handle the error:

```python
cart: list[str] = ["milk", "eggs"]
if "butter" in cart:
    cart.remove("butter")
else:
    print("Item not in cart")
```

---

## Concept 5: `pop()` — Delete by Index (and Return the Item)

**What it does**: Deletes the item at a specific index and **returns it** (unlike `remove()`).

**Syntax**: `list.pop(index)` or `list.pop()` (no index = removes last item)

**Example**:
```python
cart: list[str] = ["milk", "eggs", "bread"]
removed = cart.pop(1)  # Remove item at index 1
print(removed)  # "eggs"
print(cart)     # ["milk", "bread"]
```

Without an index, `pop()` removes and returns the last item:

```python
cart: list[str] = ["milk", "eggs", "bread"]
last = cart.pop()  # Remove the last item
print(last)   # "bread"
print(cart)   # ["milk", "eggs"]
```

**`pop()` vs `remove()`**:

- **`pop()`**: Remove by **index**, return the item
- **`remove()`**: Remove by **value**, return nothing

```python
cart: list[str] = ["milk", "eggs", "bread"]

# pop(0) removes the first item and returns it
item = cart.pop(0)  # item = "milk", cart = ["eggs", "bread"]

# remove("eggs") removes the value "eggs" and returns nothing
cart.remove("eggs")  # cart = ["bread"]
```

What if you `pop()` from an empty list?

```python
cart: list[str] = []
item = cart.pop()  # IndexError: pop from empty list
```

You'll get an `IndexError`. This is a common edge case to watch for.

#### 💬 AI Colearning Prompt

> "Explain the difference between `pop(0)` and `remove(value)`. When would you use each? Show me a scenario where they give different results."

---

## Concept 6: `clear()` — Remove All Items

**What it does**: Empties the list completely.

**Syntax**: `list.clear()`

**Example**:
```python
cart: list[str] = ["milk", "eggs", "bread"]
cart.clear()
print(cart)  # []
```

Simple and direct. After `clear()`, the list exists but is empty.

---

## Concept 7: Method vs Function — The Critical Distinction

This is one of the most important patterns in Python, and understanding it will save you debugging headaches.

**Methods** are functions attached to objects. They use dot notation:
```python
cart: list[str] = ["eggs", "milk", "bread"]
cart.append("milk")   # Method (attached to the cart list)
cart.remove("eggs")   # Method
cart.clear()          # Method
```

**Functions** are standalone. They use parentheses with the object as an argument:
```python
cart: list[str] = ["eggs", "milk", "bread"]
len(cart)             # Function (not attached to cart)
sorted(cart)          # Function
print(cart)           # Function
```

Here's the pattern that matters: **Most list modification methods return `None` because they modify the list in-place.**

```python
cart: list[str] = ["eggs", "milk", "bread"]

# Methods that modify (return None)
cart.append("milk")     # Returns None, modifies cart
cart.extend(["butter"]) # Returns None, modifies cart
cart.pop()              # Returns the item, modifies cart (exception!)
cart.remove("milk")     # Returns None, modifies cart
cart.clear()            # Returns None, modifies cart

# Functions that preserve the original (return new objects)
cart_copy: list[str] = ["eggs", "milk", "bread"]
sorted(cart_copy)       # Returns new sorted list, doesn't modify cart_copy
len(cart_copy)          # Returns count, doesn't modify cart_copy
```

**Never do this**:
```python
cart: list[str] = ["milk", "eggs", "bread"]
# Wrong! sorted() returns a new list, you're not using it
sorted(cart)
print(cart)  # Original cart is unchanged!

# Right! Save the sorted result
sorted_cart = sorted(cart)

# Or if you want to modify the original in-place, use the method
cart.sort()  # Method, modifies cart directly
```

#### 🎓 Expert Insight

> This pattern—methods modify in-place and return None, functions preserve the original and return new objects—appears throughout Python. Understand it once, and you'll read Python code confidently forever. AI handles syntax; you focus on this semantic pattern.

---

## Practice Exercise 1: Building a Shopping Cart

Translate each English operation into Python code:

1. Create a shopping cart with `["milk", "eggs"]`
2. Add "bread" to the end
3. Add ["butter", "jam"] to the cart (using `extend()`)
4. Insert "yogurt" at position 1
5. Remove "milk" from the cart
6. Remove the last item (using `pop()`) and store it in a variable
7. Clear the cart

Write the code using type hints:

```python
cart: list[str] = ["milk", "eggs"]
# Add your code here
```

**Expected output after step 6**:
```
["yogurt", "butter", "jam"]
```

---

## Practice Exercise 2: Common Errors

For each code snippet, predict what happens (runs successfully, or which error occurs?):

**Snippet 1**:
```python
items: list[int] = [1, 2, 3]
items.append([4, 5])
print(items)
```

**Snippet 2**:
```python
items: list[int] = [1, 2, 3]
result = items.remove(2)
print(result)
```

**Snippet 3**:
```python
items: list[int] = [1, 2, 3]
items.pop(10)
```

**Snippet 4**:
```python
items: list[int] = []
items.pop()
```

**Expected results**:
1. `[1, 2, [4, 5]]` (nested list added as single item)
2. `None` (remove returns None, not the item)
3. `IndexError` (index 10 doesn't exist)
4. `IndexError` (can't pop from empty list)

---

## Practice Exercise 3: Method Semantics

Given this scenario:

```python
inventory: list[str] = ["apple", "banana"]
purchases: list[str] = ["banana", "orange"]

# What does each operation do?
inventory.remove("banana")           # Removes "banana", inventory = ["apple"]
inventory.extend(purchases)          # Adds each item from purchases
```

After these operations, what is `inventory`? Run it and verify with AI.

---

## Real-World Application: Inventory Management

Imagine you're building a simple game where players have an inventory (a list of items). Here's how you'd use these methods:

```python
# Game inventory system
inventory: list[str] = ["sword", "shield"]

# Player picks up an item (add one)
inventory.append("potion")

# Player opens a treasure chest with multiple items (add many)
treasure: list[str] = ["gold", "gem", "ancient scroll"]
inventory.extend(treasure)

# Player removes armor they found (by value)
inventory.remove("shield")

# Player uses a potion (remove the first one found)
used = inventory.pop()  # Returns "ancient scroll" (last item)

# Player drops specific item at a position
inventory.insert(0, "map")  # Place map at the start

# Game over: clear inventory
inventory.clear()

print(inventory)  # []
```

This is real-world list manipulation. The methods stay the same whether you're managing game inventory, to-do lists, or student records. The pattern is universal.

#### 🚀 CoLearning Challenge

Ask your AI companion:

> "Build a shopping cart simulation. Implement: add item, add multiple items, remove item, check if item exists, and clear cart. Show me the code with type hints. Then explain which methods modify in-place and which don't."

**Expected Outcome**: You'll see a complete inventory/cart system and reinforce the modify-in-place vs return-new-value pattern.

---

## Try With AI

Use your AI companion (ChatGPT web, Gemini CLI, or Claude Code) for this practice set:

**Prompt 1 (Remember)**: "List the six main list modification methods and what each does in one sentence: append, extend, insert, remove, pop, clear."

Expected outcome: You recall the core purpose of each method.

---

**Prompt 2 (Understand)**: "Explain why `append()` and `extend()` are different. Give an example where using the wrong one would cause a bug in a shopping cart application."

Expected outcome: You articulate the semantic distinction (single vs multiple items) and see a concrete consequence of the error.

---

**Prompt 3 (Apply)**: "I have a list `[1, 2, 3, 4, 5]`. I want to: add 6 to the end, add [7, 8, 9] to it, remove 3, pop the last item, and insert 10 at position 0. Write the code and show the final list."

Expected outcome: You translate natural language requirements into method calls and verify the result matches your prediction.

---

**Prompt 4 (Analyze)**: "Compare these approaches: (1) `list.pop(0)` to remove the first item, (2) `list.remove(value)` to remove by value. When would you use each? What's the difference in what they require and return?"

Expected outcome: You evaluate the semantic difference (index vs value, return value vs None) and understand when to reach for each method professionally.
