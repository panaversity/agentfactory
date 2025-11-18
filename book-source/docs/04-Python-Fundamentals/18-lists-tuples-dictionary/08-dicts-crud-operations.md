---
title: "Dictionaries Part 2 — CRUD Operations"
chapter: 18
lesson: 8
duration_minutes: 210

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Dictionary Key Addition"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can add new keys to a dictionary using assignment syntax and understand that duplicate keys overwrite"

  - name: "Dictionary Value Updates"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can modify existing dictionary values using the same assignment pattern as adding keys"

  - name: "Key Deletion Methods"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can delete dictionary keys using del statement and pop() method, understanding the tradeoffs between each approach"

  - name: "Pop Method Semantics"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands that pop() returns the deleted value while del discards it, and can use default values for safe deletion"

  - name: "Key Existence Checking"
    proficiency_level: "A2-B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can check if a key exists using the 'in' operator before accessing to prevent KeyError"

  - name: "Dictionary Clearing"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can use .clear() to empty a dictionary and understands when this operation is useful"

  - name: "Unique Key Constraint Understanding"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information"
    measurable_at_this_level: "Student understands that dictionary keys must be unique and that assigning to an existing key overwrites the previous value"

learning_objectives:
  - objective: "LO-002l (A2-B1 - Apply): Add and update dictionary keys and values using assignment syntax"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Writing code that adds new keys and updates existing values in a dictionary"

  - objective: "LO-002m (A2-B1 - Apply): Delete keys using del statement and pop() method, understanding tradeoffs"
    proficiency_level: "A2-B1"
    bloom_level: "Apply"
    assessment_method: "Choosing appropriate deletion method and handling results correctly"

  - objective: "LO-004h (A2-B1 - Apply): Use .clear() to empty dictionaries appropriately"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Identifying scenarios where .clear() is the right choice"

  - objective: "LO-004i (B1 - Apply): Check key existence using the 'in' operator idiomatically"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Writing defensive code that checks before accessing"

  - objective: "LO-004j (B1 - Analyze): Understand unique-key constraint and overwrite behavior"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Explaining why duplicate keys create overwrites and implications for data"

cognitive_load:
  new_concepts: 7
  assessment: "Exactly 7 concepts (AT A2-B1 limit): adding keys, updating values, del, pop(), popitem(), clear(), in operator. No overflow. ✓"

differentiation:
  extension_for_advanced: "Explore .popitem() unpredictability in detail; design a function that safely removes keys with validation; compare performance of different deletion patterns"
  remedial_for_struggling: "Focus on assignment-based add/update; defer pop() semantics to later practice; use inventory examples exclusively before moving to abstract scenarios"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/001-part-4-chapter-18/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Dictionaries Part 2 — CRUD Operations

## Building Real Applications with Dictionaries

In Lesson 7, you learned how to create dictionaries and access values safely using `.get()`. Now we're going deeper: **CRUD operations**. CRUD stands for **Create, Read, Update, Delete**—the four fundamental operations you perform on data. You've already done Create and Read. Now you'll master Update and Delete, plus learn how to check what's in a dictionary before you act on it.

This lesson moves you from passive dictionary reading to active dictionary manipulation. By the end, you'll build a working inventory system that adds items, updates quantities, removes sold-out products, and checks stock availability.

---

## Concept 1: Adding Keys — Creating New Entries

In Lesson 7, you created dictionaries with initial key-value pairs. Now let's add new keys **after** creation.

The syntax is beautifully simple: assignment to a key that doesn't exist yet creates it.

```python
# Start with an empty inventory
inventory: dict[str, int] = {}

# Add keys one by one
inventory["apples"] = 50
inventory["bananas"] = 30
inventory["oranges"] = 20

print(inventory)
# Output: {'apples': 50, 'bananas': 30, 'oranges': 20}
```

That's it. No special method. Just assign to a new key, and Python creates the entry.

You can also start with a dictionary and expand it:

```python
inventory: dict[str, int] = {"apples": 50}

# Add more items
inventory["bananas"] = 30
inventory["oranges"] = 20

print(inventory)
# Output: {'apples': 50, 'bananas': 30, 'oranges': 20}
```

#### 💬 AI Colearning Prompt

> "In Python, why does assigning to a non-existent key create it instead of raising an error like accessing a non-existent key does?"

This explores the design philosophy: adding should be permissive (create if missing), while reading should be defensive (error if missing). Your AI can explain this distinction.

---

## Concept 2: Updating Values — Modifying Existing Entries

Updating uses the **exact same syntax** as adding. If the key exists, you overwrite the value:

```python
inventory: dict[str, int] = {
    "apples": 50,
    "bananas": 30,
    "oranges": 20
}

# Update existing value
inventory["apples"] = 55  # Now we have 55 apples (was 50)

print(inventory)
# Output: {'apples': 55, 'bananas': 30, 'oranges': 20}
```

This is intentional: Python doesn't distinguish between "add" and "update" at the syntax level. The same assignment pattern handles both.

**Real-world example**: An inventory system during a recount:

```python
# Morning inventory
store_stock: dict[str, int] = {
    "milk": 20,
    "bread": 15,
    "eggs": 30
}

# Recount at noon - update because items sold
store_stock["milk"] = 18  # 2 sold
store_stock["bread"] = 12  # 3 sold
store_stock["eggs"] = 28   # 2 sold

print(store_stock)
# Output: {'milk': 18, 'bread': 12, 'eggs': 28}
```

#### 🎓 Expert Insight

> In AI-native development, you're not memorizing "add vs update" methods. Python treats them as one operation: assign to a key. Your job is understanding the business intent: "Am I creating a new entry or modifying an existing one?" The code is the same; the context determines which.

---

## Concept 3: Unique Keys Constraint — Understanding Overwrites

Here's a critical rule: **dictionary keys must be unique**. If you assign to a key that already exists, the **old value is replaced**:

```python
student_grades: dict[str, int] = {
    "Alice": 95,
    "Bob": 87
}

# Oops, Alice's grade was entered wrong - update it
student_grades["Alice"] = 98  # Replaces 95 with 98

print(student_grades)
# Output: {'Alice': 98, 'Bob': 87}

# Notice: Alice's entry still exists, with the new value
```

This is different from lists, where adding an item appends it (creates a duplicate). Dictionaries enforce uniqueness.

**Why does this matter?** Suppose you're importing student grades from two different sources:

```python
# Source 1: midterm grades
grades: dict[str, int] = {
    "Alice": 85,
    "Bob": 90,
    "Carol": 88
}

# Source 2: final exam grades (should replace midterm)
grades["Alice"] = 95  # Replace midterm (85) with final exam (95)
grades["Bob"] = 92    # Replace midterm (90) with final exam (92)

print(grades)
# Output: {'Alice': 95, 'Bob': 92, 'Carol': 88}
# Carol's midterm (88) remains because Source 2 had no final exam for Carol
```

Without the unique-key constraint, you'd have duplicate entries and confusion. The constraint forces you to think clearly about data ownership.

#### ✨ Teaching Tip

> When building dictionaries, ask yourself: "Could I have duplicate keys in my data?" If yes, you have a design problem. Solve it by changing the value type (e.g., store `dict[str, list[int]]` if one student has multiple grades) or the key (e.g., use `dict[tuple[str, str], int]` for `(student_name, exam_type)` pairs).

---

## Concept 4: Deleting Keys with `del` — Removing Entries

There are multiple ways to delete keys. The simplest is the `del` statement:

```python
inventory: dict[str, int] = {
    "apples": 50,
    "bananas": 30,
    "oranges": 20
}

# Delete bananas from inventory
del inventory["bananas"]

print(inventory)
# Output: {'apples': 50, 'oranges': 20}
```

The `del` statement removes the key and its value. Simple and direct.

**But be careful**: `del` raises a `KeyError` if the key doesn't exist:

```python
inventory: dict[str, int] = {"apples": 50}

del inventory["bananas"]  # ❌ KeyError! 'bananas' not in inventory
```

This is actually useful behavior—it tells you there's a logic error. But if you want to delete safely (without crashing if the key is missing), that's where `pop()` comes in.

---

## Concept 5: Deleting Keys with `pop()` — Safe Removal

The `.pop()` method deletes a key **and returns its value**. It's safer than `del` because you can provide a default:

```python
inventory: dict[str, int] = {
    "apples": 50,
    "bananas": 30,
    "oranges": 20
}

# Remove bananas and get the quantity
removed_quantity = inventory.pop("bananas")

print(f"Removed {removed_quantity} bananas")
# Output: Removed 30 bananas

print(inventory)
# Output: {'apples': 50, 'oranges': 20}
```

**The key feature: default values**. Unlike `del`, `pop()` doesn't crash if the key is missing—you can provide a fallback:

```python
inventory: dict[str, int] = {"apples": 50}

# Try to remove bananas (doesn't exist)
removed_quantity = inventory.pop("bananas", 0)  # If not found, return 0

print(f"Removed {removed_quantity} bananas")
# Output: Removed 0 bananas (safely, no crash)

print(inventory)
# Output: {'apples': 50}  (unchanged)
```

**Real-world pattern**: Tracking what you sold:

```python
# Store inventory
store_stock: dict[str, int] = {
    "milk": 20,
    "bread": 15,
    "eggs": 30
}

# Customer buys milk (remove from inventory, track what's gone)
quantity_sold = store_stock.pop("milk", 0)
print(f"Sold {quantity_sold} units of milk")

# Customer asks for unavailable item
quantity_sold = store_stock.pop("cheese", 0)
print(f"Sold {quantity_sold} units of cheese (had none)")
# Gracefully handles missing item with default

print(f"Remaining inventory: {store_stock}")
# Output: Remaining inventory: {'bread': 15, 'eggs': 30}
```

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Explain the difference between `del dict[key]` and `dict.pop(key, default)`. Show me scenarios where you'd use each one. What happens if I try to delete a key that doesn't exist with each method?"

**Expected Outcome**: You'll understand that `del` is for certainty (you know the key exists), while `pop()` is for safety (you're not sure if it exists). You'll know when each is appropriate.

---

## Concept 6: Clearing Everything with `.clear()` — Resetting Dictionaries

Sometimes you want to empty an entire dictionary without deleting the variable:

```python
inventory: dict[str, int] = {
    "apples": 50,
    "bananas": 30,
    "oranges": 20
}

# Clear everything
inventory.clear()

print(inventory)
# Output: {}

# The variable still exists, just empty
print(len(inventory))  # 0
```

`.clear()` is useful for **resetting state**. For example, clearing a cache or resetting temporary data:

```python
# Cache of API responses
api_cache: dict[str, str] = {
    "user_1": "data_1",
    "user_2": "data_2"
}

# Invalidate the entire cache (e.g., when database updates)
api_cache.clear()

print(api_cache)  # {} (empty, ready for new data)
```

---

## Concept 7: Checking Key Existence — The `in` Operator

Before accessing or deleting a key, you should check if it exists. Use the `in` operator:

```python
inventory: dict[str, int] = {
    "apples": 50,
    "bananas": 30,
    "oranges": 20
}

# Check if a key exists
if "apples" in inventory:
    print(f"We have {inventory['apples']} apples")
# Output: We have 50 apples

# Check if a key doesn't exist
if "grapes" not in inventory:
    print("We don't have grapes")
# Output: We don't have grapes
```

This is **defensive programming**: check before you act.

**Pattern 1: Safe access with fallback**

You already know `.get()` handles this:

```python
inventory: dict[str, int] = {"apples": 5, "oranges": 3}

quantity = inventory.get("grapes", 0)  # 0 if not found
print(quantity)  # 0
```

**Pattern 2: Safe deletion with check**

```python
inventory: dict[str, int] = {"apples": 5, "oranges": 3}

if "bananas" in inventory:
    del inventory["bananas"]
    print("Bananas removed")
else:
    print("No bananas to remove")
```

Or more idiomatically with `pop()`:

```python
inventory: dict[str, int] = {"apples": 5, "oranges": 3}

# This is cleaner than checking first
quantity_removed = inventory.pop("bananas", None)
if quantity_removed is not None:
    print(f"Removed {quantity_removed} bananas")
else:
    print("No bananas to remove")
```

**Real-world scenario**: Restocking logic

```python
# Current inventory
store_inventory: dict[str, int] = {
    "milk": 10,
    "bread": 5
}

def restock_item(item_name: str, quantity: int) -> None:
    """Add quantity to existing item or create new entry."""
    if item_name in store_inventory:
        store_inventory[item_name] += quantity
    else:
        store_inventory[item_name] = quantity

# Restock existing item
restock_item("milk", 20)  # Now milk: 30

# Stock new item
restock_item("eggs", 24)  # Add new entry

print(store_inventory)
# Output: {'milk': 30, 'bread': 5, 'eggs': 24}
```

---

## Putting It Together: Complete Inventory System

Here's a working example that demonstrates all CRUD operations:

```python
# Track inventory with add, read, update, delete operations
inventory: dict[str, int] = {}

# CREATE: Add initial items
inventory["apples"] = 50
inventory["bananas"] = 30
inventory["oranges"] = 20

print("Initial inventory:", inventory)

# READ: Check what we have
if "apples" in inventory:
    print(f"Apples in stock: {inventory['apples']}")

# UPDATE: Modify quantities
inventory["apples"] = 45  # Sold 5 apples
inventory["bananas"] += 10  # Received shipment

print("After sales and restocking:", inventory)

# DELETE: Remove sold-out items
del inventory["oranges"]  # Oranges are gone

print("After removing oranges:", inventory)

# CLEAR: At end of day, reset if needed
inventory.clear()
print("End of day inventory:", inventory)
```

Output:
```
Initial inventory: {'apples': 50, 'bananas': 30, 'oranges': 20}
Apples in stock: 50
After sales and restocking: {'apples': 45, 'bananas': 40, 'oranges': 20}
After removing oranges: {'apples': 45, 'bananas': 40}
End of day inventory: {}
```

#### 💬 AI Colearning Prompt

> "Walk me through this inventory system. Explain what each CRUD operation does and why it's different. Then, what would happen if I tried to read from an empty dictionary?"

This reinforces understanding of how the four operations work together in a realistic scenario.

---

## Common Patterns and Pitfalls

### Pattern 1: Increment/Decrement Values (Counting)

A common inventory or statistics operation:

```python
# Track word frequency
word_counts: dict[str, int] = {}

words = ["apple", "banana", "apple", "cherry", "apple"]

for word in words:
    if word in word_counts:
        word_counts[word] += 1
    else:
        word_counts[word] = 1

print(word_counts)
# Output: {'apple': 3, 'banana': 1, 'cherry': 1}
```

Or more concisely with `.get()`:

```python
words: list[str] = ["apple", "banana", "apple", "cherry", "apple"]
word_counts: dict[str, int] = {}

for word in words:
    word_counts[word] = word_counts.get(word, 0) + 1

print(word_counts)
# Output: {'apple': 3, 'banana': 1, 'cherry': 1}
```

**Teaching Tip**: The second pattern is more Pythonic. It reads: "Get the current count (or 0 if not found), add 1, and store it back."

### Pattern 2: Checking Before Deletion

Always verify existence before using `del`:

```python
student_grades: dict[str, int] = {"Alice": 95, "Bob": 87}

# Wrong approach (crashes if student not found)
# del student_grades["Carol"]  # ❌ KeyError

# Right approach (check first)
if "Carol" in student_grades:
    del student_grades["Carol"]
else:
    print("Carol not found in gradebook")

# Best approach (use pop with default)
student_grades.pop("Carol", None)  # Silent if not found
```

### Pitfall 1: Modifying While Iterating

Don't add or remove keys while looping—it can skip items:

```python
inventory: dict[str, int] = {"apples": 50, "bananas": 30, "oranges": 20}

# ❌ Wrong: modifying during iteration
# for item in inventory:
#     if inventory[item] == 0:
#         del inventory[item]  # Can skip items!

# ✓ Right: collect keys to delete, then delete after loop
items_to_delete = [item for item in inventory if inventory[item] == 0]
for item in items_to_delete:
    del inventory[item]
```

#### 🎓 Expert Insight

> These patterns show that dictionaries follow consistent, predictable rules once you understand the core concepts. "Increment a count" is always: get-current-or-default, add-one, store-back. "Delete safely" is always: pop-with-default. You're not memorizing methods—you're applying the same logical patterns repeatedly.

---

## Practice Exercises

### Exercise 1: Build a Shopping Cart

Create a shopping cart (dictionary) where items are keys and quantities are values. Implement:
1. Add 3 items to the cart
2. Update the quantity of one item
3. Check if an item is in the cart
4. Remove an item that's sold out
5. Print the final cart

```python
# Start here
cart: dict[str, int] = {}

# Your code here
```

**Expected behavior**:
- Add items: `cart["milk"] = 2`, `cart["bread"] = 1`, `cart["eggs"] = 12`
- Update: `cart["milk"] = 3`
- Check: `if "milk" in cart: print("Milk in cart")`
- Delete: `del cart["bread"]` (after checking it exists)
- Final cart should have milk (3) and eggs (12)

---

### Exercise 2: Word Frequency Counter

Given a list of words, count how many times each word appears using a dictionary:

```python
words = ["apple", "banana", "apple", "orange", "banana", "apple"]

# Create a word_counts dictionary
# Your code here

# Print results: should show apple: 3, banana: 2, orange: 1
```

Use the pattern: `word_counts[word] = word_counts.get(word, 0) + 1`

---

### Exercise 3: Safe Deletion with Tracking

Delete items from inventory and track what was removed:

```python
inventory: dict[str, int] = {
    "apples": 50,
    "bananas": 30,
    "oranges": 20
}

# Remove bananas and track the quantity
removed_qty = inventory.pop("bananas", 0)
print(f"Removed: {removed_qty}")

# Try to remove something not in inventory (should not crash)
removed_qty = inventory.pop("grapes", 0)
print(f"Removed: {removed_qty}")

# Print remaining inventory
print(f"Remaining: {inventory}")
```

**Expected output**:
```
Removed: 30
Removed: 0
Remaining: {'apples': 50, 'oranges': 20}
```

---

## Try With AI

Use **ChatGPT web** (or your AI companion tool if already set up) for the following prompts.

### Prompt 1 (Remember): List All CRUD Operations

> "List the four CRUD operations for dictionaries in Python: Create, Read, Update, Delete. For each one, show me the syntax with a simple example (e.g., adding a key, accessing a value, changing a value, removing a key)."

**Expected Outcome**: You recall all four operations and their syntax. You understand that Create/Update use the same assignment pattern.

---

### Prompt 2 (Understand): When to Use `del` vs `pop()`

> "Explain the difference between using `del dict[key]` and `dict.pop(key, default)`. What happens if the key doesn't exist with each method? Give a real-world scenario where you'd use `pop()` instead of `del`."

**Expected Outcome**: You understand that `del` fails if the key is missing, while `pop()` with a default handles missing keys gracefully. You can identify scenarios like inventory management where safe deletion matters.

---

### Prompt 3 (Apply): Build a User Settings Dictionary

> "Create a dictionary called `user_settings` that stores user preferences: 'theme' (light/dark), 'notifications' (true/false), 'language' (en/es/fr). Then update the theme to 'dark', add a new setting 'auto_save' set to true, and check if 'email' is in settings (it won't be). Show me the final dictionary."

**Expected Outcome**: You write code that adds keys, updates values, and checks for key existence. You demonstrate understanding that missing keys can be checked with `in` operator.

---

### Prompt 4 (Analyze): Design a Inventory Management System

> "Design a dictionary-based inventory system for a small store. You need to add items, update quantities when purchased, remove sold-out items, and check if items are in stock. Show me the function signatures and explain which CRUD operation each function performs. What edge cases do you need to handle (e.g., buying more than available, adding duplicate items)?"

**Expected Outcome**: You apply architectural thinking to dictionary design. You consider edge cases and explain how unique keys enforce data integrity. You design functions that match real business operations.

---

**Safety Note**: When using `del`, always verify the key exists first or use `pop()` with a default to avoid crashes. In production code, defensive programming (checking existence before acting) prevents bugs.

**Next Lesson Preview**: Lesson 9 teaches **dictionary iteration and comprehensions**. You'll learn `.keys()`, `.values()`, and `.items()` methods, plus how to transform dictionaries using comprehensions—essential skills for processing large datasets efficiently.

