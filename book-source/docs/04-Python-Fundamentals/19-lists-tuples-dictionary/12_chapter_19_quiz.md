---
sidebar_position: 12
title: "Chapter 19: Lists, Tuples, and Dictionary Quiz"
---

# Chapter 19: Lists, Tuples, and Dictionary Quiz

Test your understanding of Python's three fundamental collection types: lists (mutable sequences), tuples (immutable sequences), and dictionaries (key-value mappings).

<Quiz
  title="Chapter 19: Lists, Tuples, and Dictionary Assessment"
  questions={[
    {
      question: "You're building an API client that processes paginated user records. Each page returns a list of users that you filter, then store filtered results. Which collection choice is most appropriate for the filtered results?",
      options: [
        "List because you need ordered mutable storage",
        "Tuple because API data shouldn't change once fetched",
        "Dictionary because you'll look up users by ID",
        "Set because you need unique users only"
      ],
      correctOption: 0,
      explanation: "List is correct because you're building up filtered results progressively (mutability needed) and likely processing them in order (sequence access). While tuples are immutable, the filtering process requires building up results dynamically. Dictionaries would be appropriate if you needed fast lookups by ID, but the question describes sequential processing. Sets handle uniqueness but lose ordering, which pagination typically preserves. Understanding when mutability enables your workflow vs when immutability provides guarantees is crucial for data structure selection.",
      source: "Lesson 1: Introduction to Collections"
    },
    {
      question: "What happens when you execute this code: `numbers = [1, 2, 3]; ref = numbers; ref.append(4); print(numbers)`?",
      options: [
        "Prints [1, 2, 3] because ref is a copy",
        "Prints [1, 2, 3, 4] because ref is an alias",
        "Raises error because lists are immutable after creation",
        "Prints [1, 2, 3, 4, 4] because append duplicates last element"
      ],
      correctOption: 1,
      explanation: "The correct answer is that it prints [1, 2, 3, 4] because ref is an alias (reference) to the same list object as numbers, not a copy. When you assign ref = numbers, both variables point to the same list in memory. Modifying the list through ref also affects numbers because they're the same object. Option 1 is wrong because assignment doesn't copy lists in Python—you need list() or [:] for copying. Option 3 is wrong because lists are mutable, not immutable. Option 4 is wrong because append adds one element to the end, not duplicates. This aliasing behavior is a common source of bugs when passing lists to functions.",
      source: "Lesson 3: Lists Mutability and Modification"
    },
    {
      question: "You ask AI to 'add multiple items to a list.' It generates: `items.append([1, 2, 3])`. What's the issue and what should you request?",
      options: [
        "Append is correct; no changes needed",
        "Use extend instead to add items individually",
        "Use insert method with position parameter",
        "Use plus operator to concatenate lists"
      ],
      correctOption: 1,
      explanation: "The correct answer is to use extend instead. The code adds the entire list [1, 2, 3] as a single nested element, making items = [..., [1, 2, 3]]. To add multiple items individually, use extend([1, 2, 3]) which unpacks and adds each element: items = [..., 1, 2, 3]. Option 1 is wrong because append treats the list as one element (creates nesting). Option 3 (insert) requires a position and still treats [1, 2, 3] as one element. Option 4 (+ operator) works but creates a new list rather than modifying in-place, which may not match the requirement. Understanding append vs extend is critical for reviewing AI-generated list manipulation code.",
      source: "Lesson 3: Lists Mutability and Modification"
    },
    {
      question: "When would using a tuple as a dictionary key be more appropriate than using a string?",
      options: [
        "When the key represents multiple related values",
        "When you need faster lookup performance times",
        "When the dictionary will be modified frequently",
        "When the key needs to be changeable"
      ],
      correctOption: 0,
      explanation: "Tuples are ideal for multi-value keys like coordinates (x, y) or date components (year, month, day). Using ('Alice', 2025) as a key is more expressive than concatenating strings like 'Alice_2025'. Option 2 is wrong because tuples and strings have similar O(1) lookup performance as dict keys. Option 3 is irrelevant—dictionary modification frequency doesn't affect key type choice. Option 4 is wrong because dict keys must be immutable (hashable)—changeability would make tuples unsuitable as keys. This pattern appears frequently in data processing where natural keys have multiple components.",
      source: "Lesson 6: Tuples Immutable Sequences"
    },
    {
      question: "What's the difference between `list.sort()` and `sorted(list)` in terms of return value and mutation?",
      options: [
        "sort() returns new list; sorted() returns None",
        "sort() modifies in-place returns None; sorted() returns new",
        "Both return new sorted lists without mutation",
        "Both modify original and return None"
      ],
      correctOption: 1,
      explanation: "The correct answer is that sort() modifies the list in-place and returns None, while sorted() returns a new sorted list without modifying the original. This is a critical distinction: sort() is a method (list.sort()) that mutates; sorted() is a function (sorted(list)) that creates. Option 1 reverses the behavior. Option 3 is wrong because sort() does mutate. Option 4 is wrong because sorted() doesn't mutate. This pattern—methods that mutate return None, functions that create return new objects—appears throughout Python. Understanding it prevents bugs like x = list.sort() (x becomes None, not the sorted list).",
      source: "Lesson 4: Lists Sorting and Advanced Methods"
    },
    {
      question: "You're processing a CSV with 100,000 user records. You need to look up user info by email quickly. Which structure provides O(1) lookup performance?",
      options: [
        "List of tuples with email as first element",
        "Dictionary with email as key and info as value",
        "Sorted list with binary search for performance",
        "Tuple of dictionaries with email field inside"
      ],
      correctOption: 1,
      explanation: "Dictionary with email as key provides O(1) average-case lookup—instant access regardless of size. Option 1 (list of tuples) requires O(n) linear search through all items. Option 3 (sorted list with binary search) gives O(log n), better than O(n) but still slower than O(1) for large datasets. Option 4 (tuple of dictionaries) requires O(n) iteration since you can't index by email—you'd search through the tuple sequentially. For 100,000 records, the difference is dramatic: dict lookup is nearly instant, list search could examine thousands of items. This is why dictionaries are fundamental for ID-based lookups in real applications.",
      source: "Lesson 10: Choosing the Right Structure"
    },
    {
      question: "What does this list comprehension produce: `[x ** 2 for x in range(5) if x % 2 == 0]`?",
      options: [
        "Squares of all numbers zero through four",
        "Squares of even numbers zero through four",
        "Even numbers zero through four without squaring",
        "Squares of numbers divisible by two only"
      ],
      correctOption: 1,
      explanation: "The comprehension produces [0, 4, 16]—squares of even numbers (0, 2, 4) from range(5). The if x % 2 == 0 filter selects even numbers, then x ** 2 squares each. Option 1 is wrong because the filter removes odds (1, 3). Option 3 is wrong because the expression x ** 2 does square values. Option 4 is technically correct but less precise—'even numbers' is clearer than 'divisible by two'. The key insight: comprehensions apply the filter BEFORE the expression, so you filter first (selecting evens), then transform (squaring). This pattern—filter then transform—is fundamental to data processing pipelines.",
      source: "Lesson 5: List Comprehensions"
    },
    {
      question: "You ask AI to 'create a read-only coordinate pair for use as a dict key.' It generates `coord = [10, 20]`. What should you request?",
      options: [
        "Add type hint list[int] for clarity only",
        "Change to tuple because lists aren't hashable",
        "Use dict directly with x and y keys",
        "Convert to string for dictionary key usage"
      ],
      correctOption: 1,
      explanation: "Change to tuple: coord = (10, 20). Lists can't be dict keys because they're mutable (unhashable), but tuples can because they're immutable. Attempting locations[[10, 20]] = 'Point A' raises TypeError: unhashable type: 'list'. Option 1 (type hint) doesn't fix the hashability issue. Option 3 (using a dict) misses the point—you want the coordinate as a key, not a value. Option 4 (string) works technically ('10,20') but loses type information and makes arithmetic operations awkward. When reviewing AI code, check that keys for dictionaries use immutable types (tuples, strings, numbers)—not mutable ones (lists, dicts).",
      source: "Lesson 6: Tuples Immutable Sequences"
    },
    {
      question: "In a function that modifies a list parameter, what happens to the original list passed to it?",
      options: [
        "Original is protected; only local copy changes",
        "Original is modified because lists pass by reference",
        "Python copies the list automatically for safety",
        "Original stays unchanged; function returns modified copy"
      ],
      correctOption: 1,
      explanation: "The original list is modified because Python passes lists by reference (technically, passes the reference by value). When you call process(my_list), the parameter refers to the same list object. Any mutations inside the function (append, remove, sort) affect the original. Option 1 is wrong—Python doesn't automatically copy. Option 3 is wrong for the same reason. Option 4 describes immutable behavior (like strings), not lists. This is a common source of bugs: passing a list to a function can modify it unexpectedly. To prevent this, explicitly pass a copy: process(my_list.copy()) or process(my_list[:]).",
      source: "Lesson 3: Lists Mutability and Modification"
    },
    {
      question: "What's the primary difference between accessing items in a sequence (list/tuple) vs a mapping (dict)?",
      options: [
        "Sequences use numeric indices; mappings use keys",
        "Sequences are faster for all operations",
        "Mappings maintain insertion order; sequences do not",
        "Sequences store unique items; mappings allow duplicates"
      ],
      correctOption: 0,
      explanation: "Sequences use numeric indices (fruits[0], coordinates[2]); mappings use keys (user['name'], grades['Alice']). This reflects their purpose: sequences organize items by position, mappings by meaningful identifiers. Option 2 is wrong—mappings are faster for lookups (O(1) vs O(n)). Option 3 is backwards—modern dicts maintain insertion order, but that's not the primary distinction. Option 4 is wrong—sequences can have duplicate values, and dict keys are unique but values can repeat. Understanding this access pattern difference helps you choose the right structure: if you think 'give me item 5', use a sequence; if you think 'give me Alice's data', use a mapping.",
      source: "Lesson 1: Introduction to Collections"
    },
    {
      question: "Why might you choose a tuple over a list even when you don't need it as a dict key?",
      options: [
        "Tuples use less memory and process faster",
        "Tuples communicate immutability intent and prevent bugs",
        "Tuples support more built-in methods than lists",
        "Tuples allow duplicate values while lists do not"
      ],
      correctOption: 1,
      explanation: "Tuples communicate immutability intent—when you use a tuple, you're saying 'this data shouldn't change.' This prevents accidental modifications and makes code easier to reason about. For example, RGB_RED = (255, 0, 0) communicates this value is constant. Option 1 (memory/speed) has minor truth but isn't the primary reason—the performance difference is negligible for most applications. Option 3 is backwards—lists have more methods (append, extend, remove) precisely because they're mutable. Option 4 is wrong—both allow duplicates. The key insight: immutability is a feature, not a limitation. It provides guarantees that make code safer and more predictable.",
      source: "Lesson 6: Tuples Immutable Sequences"
    },
    {
      question: "What does `user.get('age', 0)` return if the 'age' key doesn't exist in the dictionary?",
      options: [
        "Returns None because key is missing",
        "Returns 0 the default value provided",
        "Raises KeyError because key not found",
        "Returns empty string for missing keys"
      ],
      correctOption: 1,
      explanation: "The .get() method returns the default value (0) when the key is missing. This is safer than user['age'], which raises KeyError for missing keys. Option 1 is wrong—.get() without a second argument returns None, but here we provided 0 as default. Option 3 describes bracket notation behavior (user['age']), not .get(). Option 4 is wrong—the return value is whatever you specify as default, not automatically a string. This pattern is critical for defensive programming: use .get(key, default) when keys might be missing, use [key] when you expect the key to exist and want an error if it doesn't.",
      source: "Lesson 8: Dicts CRUD Operations"
    },
    {
      question: "You need to store a shopping cart where items can be added, removed, and processed in order. Which structure fits best?",
      options: [
        "List because order and mutability matter",
        "Tuple because cart contents are fixed",
        "Dictionary mapping item IDs to quantities",
        "Set because items should be unique"
      ],
      correctOption: 0,
      explanation: "List is ideal: you add/remove items (mutability), process them in order (sequence), and likely don't need instant ID lookups. Option 2 (tuple) fails because carts change—items are added and removed. Option 3 (dict) would work if you needed fast 'does this item exist?' checks or wanted to aggregate quantities, but the question emphasizes order and modification. Option 4 (set) loses order, which matters for displaying the cart consistently. The decision framework: if you need to build up or modify a collection over time and order matters, choose a list. If order doesn't matter and you need fast lookups, choose a dict.",
      source: "Lesson 10: Choosing the Right Structure"
    },
    {
      question: "What does this code output: `nums = [1, 2]; copy = nums[:]; copy.append(3); print(len(nums))`?",
      options: [
        "Prints 3 because copy is an alias",
        "Prints 2 because copy is independent slice",
        "Prints 3 because slice creates shallow copy",
        "Raises error because slice syntax is invalid"
      ],
      correctOption: 1,
      explanation: "Prints 2 because nums[:] creates a new independent list containing the same elements. Modifying copy doesn't affect nums—they're separate objects. Option 1 is wrong—slicing copies, assignment creates aliases. Option 3 is contradictory—'shallow copy' means independent for the list itself (correct), but the conclusion (len 3) is wrong. Option 4 is wrong—[:] is valid Python slice syntax for 'all elements.' This is the key difference: ref = nums creates an alias (both point to same list), copy = nums[:] creates a new list (independent). Understanding copy vs alias prevents subtle bugs when passing lists around.",
      source: "Lesson 3: Lists Mutability and Modification"
    },
    {
      question: "When iterating through a dictionary with `for key in user_dict:`, what does the loop variable contain?",
      options: [
        "Each key from the dictionary",
        "Each value from the dictionary",
        "Tuples of key-value pairs",
        "Indices for accessing dictionary items"
      ],
      correctOption: 0,
      explanation: "The loop variable contains each key. The default iteration for dicts is over keys: for k in my_dict iterates keys. To get values, use for v in my_dict.values(). To get both, use for k, v in my_dict.items(). Option 2 is wrong—values require .values(). Option 3 describes .items() iteration, not default. Option 4 is wrong—dicts don't have numeric indices like lists. This default behavior (iterating keys) makes sense: you can always get the value with my_dict[k], so having the key gives you access to everything. But explicitly using .keys(), .values(), or .items() makes intent clearer.",
      source: "Lesson 9: Dicts Iteration and Comprehensions"
    },
    {
      question: "You ask AI to 'reverse this list in place'. It generates `reversed_list = my_list.reverse()`. What's wrong?",
      options: [
        "Should use my_list.reversed() method instead",
        "reverse() returns None; list is modified in place",
        "Should use sorted function for reversing order",
        "reverse() requires a parameter for direction"
      ],
      correctOption: 1,
      explanation: "The .reverse() method modifies the list in-place and returns None, so reversed_list becomes None, not the reversed list. The correct usage is just my_list.reverse() (no assignment), or use reversed_list = my_list[::-1] to get a reversed copy. Option 1 is wrong—there's no .reversed() method. Option 3 is wrong—sorted() sorts, doesn't reverse (though sorted(my_list, reverse=True) reverses while sorting). Option 4 is wrong—.reverse() takes no parameters. This is a common AI-generated bug: treating in-place mutation methods like functions that return values. Always check if a method returns None—that signals in-place mutation.",
      source: "Lesson 4: Lists Sorting and Advanced Methods"
    },
    {
      question: "What's the output of `{k: v for k, v in [('a', 1), ('b', 2), ('a', 3)]}`?",
      options: [
        "{'a': 1, 'b': 2, 'a': 3} with duplicate keys",
        "{'a': 3, 'b': 2} because last value wins",
        "Raises error because duplicate keys are invalid",
        "{'a': [1, 3], 'b': [2]} with grouped values"
      ],
      correctOption: 1,
      explanation: "The comprehension produces {'a': 3, 'b': 2} because when a key appears multiple times, the last value overwrites previous ones. Dict keys must be unique—assigning to the same key replaces the value. Option 1 is wrong because Python dicts can't have duplicate keys. Option 3 is wrong—no error is raised; the duplicate key simply updates the value. Option 4 describes an accumulator pattern (which requires explicit logic), not default dict behavior. This behavior—last value wins—is important when processing data with possible duplicates. If you want to keep all values, you need explicit accumulation logic.",
      source: "Lesson 9: Dicts Iteration and Comprehensions"
    },
    {
      question: "When would you use `in` operator with a dictionary vs a list, and why does performance differ?",
      options: [
        "Dict checks keys in constant time; list checks linearly",
        "List is always faster because items are indexed",
        "Dict checks values in constant time; list checks keys",
        "Performance is identical for both data structures"
      ],
      correctOption: 0,
      explanation: "The 'in' operator checks dict keys in O(1) constant time (instant) but checks list values in O(n) linear time (examines each item). For a million items, dict lookup is nearly instant, list search could check all million. Option 2 is backwards—dicts are faster for membership checks. Option 3 is wrong—dicts check keys, not values (checking values requires 'in my_dict.values()'). Option 4 is wrong—the performance difference is dramatic at scale. This is why you use dicts for large collections where you frequently ask 'does this ID exist?'—the speed difference matters in production systems.",
      source: "Lesson 10: Choosing the Right Structure"
    },
    {
      question: "What does this tuple unpacking do: `name, age, *others = ('Alice', 25, 'CS', 3.8, True)`?",
      options: [
        "Assigns first two items; others becomes list with rest",
        "Raises error because asterisk is invalid syntax",
        "Assigns all items to name ignoring others",
        "Creates three tuples splitting the original sequence"
      ],
      correctOption: 0,
      explanation: "Unpacking assigns name='Alice', age=25, and others becomes a list ['CS', 3.8, True] containing remaining items. The asterisk (*) collects 'the rest' into a list. Option 2 is wrong—this is valid Python 3 extended unpacking syntax. Option 3 is wrong—each variable gets its value; nothing is ignored. Option 4 is wrong—this creates variables, not tuples. This pattern is useful when processing records with variable-length data: you extract known fields, collect the rest. Note that others becomes a list even though you're unpacking a tuple—the asterisk always collects into a list.",
      source: "Lesson 6: Tuples Immutable Sequences"
    },
    {
      question: "Why does `[1, 2, 3] + [4, 5]` create a new list instead of modifying the first one?",
      options: [
        "Python always copies lists for safety reasons",
        "Plus operator creates new objects; doesn't mutate operands",
        "Plus operator only works with tuples not lists",
        "Python can't mutate lists with operator syntax"
      ],
      correctOption: 1,
      explanation: "The + operator creates a new list without modifying either operand, following the principle that operators return new values. If you want in-place modification, use list1.extend(list2). Option 1 overgeneralizes—Python doesn't always copy. Option 3 is wrong—+ works with lists, tuples, and strings. Option 4 is wrong—operators don't mutate, but methods like .append() do. Understanding operator vs method behavior is critical: operators create new objects (a + b), methods often mutate (a.extend(b)). This distinction appears throughout Python: + creates, += may mutate depending on type.",
      source: "Lesson 2: Lists Creation and Basics"
    },
    {
      question: "You're building a config validator. User provides a dict that might be missing required keys. Which approach safely checks for missing keys?",
      options: [
        "Use dict.get with default values for validation",
        "Use try-except to catch KeyError exceptions",
        "Check with key in dict before accessing",
        "All three approaches work for safe access"
      ],
      correctOption: 3,
      explanation: "All three approaches safely handle missing keys, each with different use cases. Option 1 (.get(key, default)) is best when you want a fallback value. Option 2 (try-except) is best when missing keys are exceptional and you want to handle them specially. Option 3 (key in dict) is best when you need to check existence before deciding what to do. The best choice depends on context: .get() for defaults, 'in' for conditional logic, try-except for exceptional cases. Understanding all three patterns makes you a better code reviewer—you can evaluate whether AI chose the most appropriate approach for the situation.",
      source: "Lesson 8: Dicts CRUD Operations"
    },
    {
      question: "What happens if you try to create a dict with a list as a key: `locations = {[0, 0]: 'Origin'}`?",
      options: [
        "Works fine because lists are hashable types",
        "Raises TypeError because lists aren't hashable",
        "Converts list to tuple automatically for key",
        "Creates string key from list representation"
      ],
      correctOption: 1,
      explanation: "Raises TypeError: unhashable type: 'list'. Dict keys must be hashable (immutable), and lists are mutable, therefore not hashable. Use a tuple instead: {(0, 0): 'Origin'}. Option 1 is wrong—lists are explicitly not hashable. Option 3 is wrong—Python doesn't automatically convert types for you. Option 4 is wrong—no string conversion happens. The hashability requirement ensures keys can't change after being used—if a list key could be modified, the dictionary's internal structure would break. This is why tuples, strings, and numbers work as keys, but lists, dicts, and sets don't.",
      source: "Lesson 6: Tuples Immutable Sequences"
    },
    {
      question: "When processing API responses with variable structure, which pattern safely handles missing fields?",
      options: [
        "Use .get() with defaults for optional fields",
        "Access with brackets and assume all present",
        "Convert to tuple to lock the structure",
        "Use list comprehension to filter missing fields"
      ],
      correctOption: 0,
      explanation: "Using .get(key, default) safely handles missing fields: age = user.get('age', 0) returns 0 if 'age' is missing. This is defensive programming for variable API structures. Option 2 (bracket access) raises KeyError for missing fields—use this only when fields are required and you want errors. Option 3 (tuple) doesn't help—tuples are for immutability, not handling missing data. Option 4 (list comprehension) is irrelevant to dict field access. Real APIs often have optional fields (email_verified, phone_number), and .get() with sensible defaults prevents crashes while providing fallback values.",
      source: "Lesson 8: Dicts CRUD Operations"
    },
    {
      question: "What's the difference between `my_list.append(x)` and `my_list = my_list + [x]` in terms of mutation?",
      options: [
        "Both mutate original list identically",
        "append() mutates in place; plus creates new list",
        "Plus mutates faster than append method",
        "append() creates copy; plus mutates original"
      ],
      correctOption: 1,
      explanation: "append() mutates the original list in place (adds x without creating a new list). The + operator creates an entirely new list and reassigns my_list to point to it. If other variables reference the original list (ref = my_list), append affects ref, but + does not. Option 1 is wrong—their mutation behavior differs. Option 3 is backwards—append is faster because it doesn't allocate a new list. Option 4 reverses the behavior. This distinction matters when lists are shared: if multiple variables reference the same list, append affects all of them, but + only affects the variable being reassigned.",
      source: "Lesson 3: Lists Mutability and Modification"
    },
    {
      question: "You ask AI to 'iterate through a dict and print both keys and values.' It generates `for k in user: print(user[k])`. What's the issue?",
      options: [
        "Should use user.keys() for explicit key iteration",
        "Works but user.items() is more Pythonic",
        "Should use user.values() to get values",
        "Requires enumerate to get both keys and values"
      ],
      correctOption: 1,
      explanation: "The code works (iterates keys, accesses values via user[k]), but using for k, v in user.items() is more Pythonic—it's clearer and more efficient. items() gives you both key and value directly without repeated lookups. Option 1 (user.keys()) is redundant—for k in user already iterates keys. Option 3 (user.values()) would only give values, losing keys. Option 4 (enumerate) is for sequences with indices, not dicts. When reviewing AI code, recognize patterns that work but aren't idiomatic: accessing values via keys inside the loop is a code smell suggesting .items() is better.",
      source: "Lesson 9: Dicts Iteration and Comprehensions"
    },
    {
      question: "Why does `coords = (10,)` require a trailing comma while `coords = (10, 20)` does not?",
      options: [
        "Parentheses alone mean grouping; comma makes tuple",
        "Single-item tuples are invalid without the comma",
        "Python requires commas for all tuple definitions",
        "Trailing comma is optional syntactic sugar"
      ],
      correctOption: 0,
      explanation: "Without the comma, (10) is just grouping (evaluating the expression 10), not a tuple. The comma signals tuple creation: (10,) is a single-item tuple. For multi-item tuples, the comma is already present, so (10, 20) is unambiguous. Option 2 is close but imprecise—single-item tuples are valid, but require the comma. Option 3 is wrong—you can omit parentheses entirely: coords = 10, 20 is valid. Option 4 is wrong—the comma is required, not optional. This syntax quirk catches many beginners: x = (10) creates an int, x = (10,) creates a tuple. Always include the comma for single-item tuples.",
      source: "Lesson 6: Tuples Immutable Sequences"
    },
    {
      question: "When building a frequency counter for word occurrences, which approach correctly accumulates counts?",
      options: [
        "Use list to store each word occurrence",
        "Use dict with words as keys and counts as values",
        "Use tuple to track unique word and count pairs",
        "Use set to collect unique words only"
      ],
      correctOption: 1,
      explanation: "Use a dict mapping words to counts: {'apple': 3, 'banana': 2}. You look up the word (key), increment its count (value). Option 1 (list) requires O(n) search for each word—inefficient. Option 3 (tuple) is immutable and can't be updated as you count. Option 4 (set) tracks unique words but loses count information. The accumulator pattern: check if word in counts, if yes increment counts[word] += 1, if no initialize counts[word] = 1. Or use .get(): counts[word] = counts.get(word, 0) + 1. This pattern appears in data processing constantly.",
      source: "Lesson 9: Dicts Iteration and Comprehensions"
    },
    {
      question: "What's the result of `[x for x in range(10) if x > 5][:2]`?",
      options: [
        "First two numbers greater than five",
        "All numbers greater than five with limit",
        "Numbers six and seven from filtered results",
        "Empty list because slicing filters nothing"
      ],
      correctOption: 0,
      explanation: "The comprehension produces [6, 7, 8, 9] (numbers > 5 from range(10)), then [:2] slices the first two items, giving [6, 7]. Option 1 correctly describes this. Option 2 is imprecise—'all numbers greater than five' would be [6, 7, 8, 9], but the slice limits to two. Option 3 is technically correct but less precise than option 1. Option 4 is wrong—the slice operates on the filtered list, not the filtering process. This pattern (comprehension + slice) is useful when you want 'the first N items matching a condition' without stopping early.",
      source: "Lesson 5: List Comprehensions"
    },
    {
      question: "You need to store configuration values that shouldn't change during program execution. Which choice best communicates this intent?",
      options: [
        "List because configuration is ordered data",
        "Dict with uppercase keys to signal constants",
        "Tuple because immutability prevents modification",
        "Set because configuration values are unique"
      ],
      correctOption: 2,
      explanation: "Tuple communicates immutability intent: CONFIG = (800, 600, 'Title') says 'this data is fixed.' Attempting CONFIG[0] = 1024 raises an error, preventing accidental changes. Option 1 (list) provides no protection—config can be modified. Option 2 (dict with uppercase keys) is a convention for constant names, but dicts are still mutable—you can change values. Option 4 (set) is unordered and loses positional meaning. When you choose a tuple over a list for non-key data, you're using the type system to document and enforce immutability—a form of defensive programming that makes bugs impossible.",
      source: "Lesson 6: Tuples Immutable Sequences"
    },
    {
      question: "What does `list1 = list2` do in terms of object identity and mutation?",
      options: [
        "Creates independent copy of list2 in list1",
        "Makes list1 alias pointing to same object",
        "Converts list2 to immutable tuple in list1",
        "Raises error because lists require explicit copying"
      ],
      correctOption: 1,
      explanation: "Assignment creates an alias: list1 and list2 point to the same list object in memory. Mutating through either variable affects both because there's only one list. To create an independent copy, use list1 = list2[:] or list1 = list2.copy(). Option 1 is wrong—assignment doesn't copy. Option 3 is wrong—no type conversion happens. Option 4 is wrong—assignment is valid, just doesn't copy. This aliasing behavior is a frequent source of bugs: you modify list1 and are surprised when list2 changes. Use is operator to check identity: list1 is list2 returns True for aliases, False for copies.",
      source: "Lesson 3: Lists Mutability and Modification"
    },
    {
      question: "When would a dict comprehension be more appropriate than a loop with dictionary building?",
      options: [
        "When transformation logic is simple and readable",
        "When you need complex nested iteration logic",
        "When error handling is required during building",
        "Loops are always better than dict comprehensions"
      ],
      correctOption: 0,
      explanation: "Dict comprehensions are ideal for simple, readable transformations: {k: v.upper() for k, v in pairs} is clearer than a 4-line loop. But for complex logic (nested conditions, error handling, multi-step transformations), loops are more readable. Option 2 describes when loops are better—complex nesting hurts comprehension readability. Option 3 describes when loops are necessary—comprehensions can't include try-except. Option 4 overgeneralizes—comprehensions are often clearer for simple cases. The guideline: if the comprehension fits on one line and is immediately clear, use it; if it requires mental parsing or multiple lines, use a loop.",
      source: "Lesson 9: Dicts Iteration and Comprehensions"
    },
    {
      question: "What's the output of `print(len({1, 2, 2, 3}))` and why does it differ from a list?",
      options: [
        "Prints 4 because all items count",
        "Prints 3 because sets store unique values",
        "Raises error because duplicate values invalid",
        "Prints 4 but only stores unique items"
      ],
      correctOption: 1,
      explanation: "Prints 3 because sets automatically deduplicate—{1, 2, 2, 3} becomes {1, 2, 3}. This differs from lists, which keep all items including duplicates: [1, 2, 2, 3] has length 4. Option 1 describes list behavior. Option 3 is wrong—no error is raised; duplicates are silently ignored. Option 4 is contradictory. Sets are useful when you need to track unique items (unique user IDs, seen values) or perform set operations (intersection, union). This question tests understanding that different collection types have different deduplication semantics.",
      source: "Lesson 1: Introduction to Collections"
    },
    {
      question: "You ask AI to 'safely remove an item from a list if it exists.' It generates `list.remove(item)`. What's missing?",
      options: [
        "Should use list.pop() instead for removal",
        "Should check if item in list first",
        "Should use del list[item] for safe removal",
        "Nothing is missing; remove is always safe"
      ],
      correctOption: 1,
      explanation: "The code raises ValueError if item isn't in the list. For safe removal, check first: if item in list: list.remove(item). Option 1 (pop) removes by index, not value, and also raises error if index is invalid. Option 3 (del) requires an index, not a value, and would fail with del list[item] if item isn't an integer. Option 4 is wrong—remove() raises ValueError for missing items. Alternatively, use a try-except: try: list.remove(item); except ValueError: pass. When reviewing AI code for list modification, check whether error handling is needed for missing items.",
      source: "Lesson 3: Lists Mutability and Modification"
    },
    {
      question: "Why does dict iteration order match insertion order in Python 3.7+ but this wasn't always guaranteed?",
      options: [
        "Python 3.7 changed dict implementation to preserve order",
        "Dicts always maintained order but docs were wrong",
        "Order preservation is still not guaranteed officially",
        "Earlier versions used lists internally not hashes"
      ],
      correctOption: 0,
      explanation: "Python 3.7 changed the dict implementation to preserve insertion order as a language guarantee (previously a CPython implementation detail in 3.6). Before 3.7, dict order was undefined—iterating the same dict could yield different orders. Option 2 is wrong—earlier versions explicitly didn't guarantee order. Option 3 is wrong—it's officially guaranteed since 3.7. Option 4 is wrong—earlier versions used hash tables, not lists. This history matters: you can rely on dict order in modern Python, but code targeting older versions shouldn't depend on it. When choosing dicts, you get both fast lookup AND predictable order.",
      source: "Lesson 7: Dicts Key-Value Basics"
    },
    {
      question: "What does `[item.strip() for item in data if item]` do with whitespace and empty strings?",
      options: [
        "Removes whitespace from all items including empty strings",
        "Strips whitespace from non-empty items only",
        "Removes empty items but keeps whitespace intact",
        "Raises error because empty strings are invalid"
      ],
      correctOption: 1,
      explanation: "The if item filter excludes empty strings (because empty string is falsy), then strip() removes leading/trailing whitespace from remaining items. If data = ['  a  ', '', 'b'], the result is ['a', 'b']. Option 1 is wrong—empty strings are filtered out before stripping. Option 3 is backwards—empty items are removed AND whitespace is stripped. Option 4 is wrong—no error occurs. This pattern (filter then transform) is common in data cleaning: if item checks removes empty/None values, then strip() cleans remaining strings. Understanding filter order matters: filter first, transform second.",
      source: "Lesson 5: List Comprehensions"
    },
    {
      question: "When aggregating data by category, which pattern correctly accumulates values into dict values?",
      options: [
        "Check if key exists; append or create list",
        "Use dict.append() to add values to keys",
        "Use tuple keys with values as second element",
        "Create separate list for each category manually"
      ],
      correctOption: 0,
      explanation: "The accumulator pattern: if category in results: results[category].append(value); else: results[category] = [value]. This builds {category: [values...]} for each category. Option 2 is wrong—dicts don't have append() method. Option 3 describes tuple keys, not value accumulation. Option 4 misses the point—manually creating lists defeats the purpose of dynamic aggregation. Alternatively, use setdefault: results.setdefault(category, []).append(value), or defaultdict. This pattern appears constantly in data processing: grouping sales by region, aggregating scores by student, collecting errors by type.",
      source: "Lesson 9: Dicts Iteration and Comprehensions"
    },
    {
      question: "What's the difference between `list.clear()` and `list = []` in terms of references?",
      options: [
        "Both create new empty list objects",
        "clear() empties list in place; assignment creates new",
        "clear() creates new list; assignment empties existing",
        "No difference; both have identical effect"
      ],
      correctOption: 1,
      explanation: "clear() empties the existing list in place—other references see the change. Assignment (list = []) creates a new empty list and rebinds the variable—other references still point to the old list. If ref = list, then list.clear() makes ref empty too, but list = [] leaves ref unchanged. Option 1 is wrong—clear() doesn't create a new object. Option 3 reverses the behavior. Option 4 is wrong—they differ when aliases exist. This distinction matters when lists are shared: methods mutate the object in place (affecting all references), assignment rebinds only the variable (leaving other references unchanged).",
      source: "Lesson 3: Lists Mutability and Modification"
    },
    {
      question: "You're storing user preferences with string keys and mixed value types (strings, ints, bools). Which type hint is most accurate?",
      options: [
        "dict[str, str] because keys are strings",
        "dict[str, int | str | bool] union for mixed values",
        "dict[str, Any] to allow all value types",
        "list[tuple[str, str]] for key-value pairs"
      ],
      correctOption: 1,
      explanation: "dict[str, int | str | bool] explicitly documents the allowed value types (union type). This is more precise than Any, which allows literally anything. Option 1 is wrong—values aren't all strings. Option 3 (Any) works but loses type information—you want to communicate which types are valid. Option 4 changes the structure entirely (list of tuples, not dict). Using precise union types helps type checkers catch errors: preferences['count'] = [] would be caught because list isn't in the union. Modern Python (3.10+) uses pipe syntax (int | str | bool), older versions use Union[int, str, bool].",
      source: "Lesson 7: Dicts Key-Value Basics"
    },
    {
      question: "What does this nested comprehension produce: `[[x*y for x in range(3)] for y in range(2)]`?",
      options: [
        "Single flat list with all products",
        "List of lists with multiplication table pattern",
        "Raises error because nesting is invalid",
        "Set of unique products from multiplication"
      ],
      correctOption: 1,
      explanation: "Produces [[0, 0, 0], [0, 1, 2]]—a list of lists. The outer loop (for y in range(2)) runs twice, creating two sublists. The inner comprehension (for x in range(3)) creates [x*y for x in range(3)] for each y value. When y=0: [0, 0, 0]; when y=1: [0, 1, 2]. Option 1 is wrong—nesting preserves structure. Option 3 is wrong—nested comprehensions are valid Python. Option 4 is wrong—this creates a list, not a set. Nested comprehensions are powerful but harder to read—this one could be clearer as a loop. They're appropriate for creating matrices or nested structures.",
      source: "Lesson 5: List Comprehensions"
    },
    {
      question: "Why does `sorted(my_dict)` return a list of keys rather than a sorted dictionary?",
      options: [
        "Python can't sort dictionaries by values",
        "sorted() works on iterables; dicts iterate keys",
        "Dictionaries don't support sorting operations",
        "sorted() requires explicit key parameter for dicts"
      ],
      correctOption: 1,
      explanation: "sorted() accepts any iterable. When you iterate a dict (for x in my_dict), you get keys. Therefore sorted(my_dict) sorts and returns keys as a list. To sort by values, use sorted(my_dict.items(), key=lambda x: x[1]). Option 1 is wrong—you can sort by values with the right approach. Option 3 is wrong—you can sort dict items; you just get a list back, not a dict. Option 4 is partially true (key parameter is needed for sorting by values) but misses the core concept. Understanding what sorted() returns helps you process dict data correctly: keys only, items (tuples), or values, depending on what you pass.",
      source: "Lesson 9: Dicts Iteration and Comprehensions"
    },
    {
      question: "You ask AI to 'insert an item at the beginning of a list.' It generates `list.append(item)`. What should you request?",
      options: [
        "Use list.prepend(item) for beginning insertion",
        "Use list.insert(0, item) for position zero",
        "Append is correct because lists grow forward",
        "Use list[0] = item to replace first item"
      ],
      correctOption: 1,
      explanation: "Use insert(0, item) to insert at position 0 (beginning). append() adds to the end, not beginning. Option 1 is wrong—Python lists don't have a prepend() method. Option 3 is wrong—append() explicitly adds to the end. Option 4 (list[0] = item) replaces the first item, doesn't insert (changes length from n to n, not n to n+1). Understanding list modification methods: append() adds to end, insert(index, item) adds at specific position, [index] = item replaces at position. When reviewing AI code, verify the method matches the operation: insert for adding, assignment for replacing.",
      source: "Lesson 3: Lists Mutability and Modification"
    },
    {
      question: "When would you choose a list of dicts over a dict of lists for storing records?",
      options: [
        "When each record is independent with own fields",
        "When fields are uniform across all records",
        "When you need fast lookup by ID",
        "List of dicts is always better than alternatives"
      ],
      correctOption: 0,
      explanation: "List of dicts ([{name: 'Alice', age: 25}, {name: 'Bob', age: 30}]) is ideal when each record is independent and may have different fields. Dict of lists ({names: ['Alice', 'Bob'], ages: [25, 30]}) works when fields are uniform and you process by column. Option 2 describes when dict of lists works better. Option 3 describes when you need a dict with ID keys: {101: {name: 'Alice'}, 102: {name: 'Bob'}}. Option 4 overgeneralizes. The decision: list of dicts for row-oriented data (adding/removing records easily), dict of lists for column-oriented data (processing all names together).",
      source: "Lesson 10: Choosing the Right Structure"
    },
    {
      question: "What does `my_dict.pop('key', None)` return if 'key' doesn't exist in the dictionary?",
      options: [
        "Raises KeyError because key is missing",
        "Returns None the default value provided",
        "Returns empty dict for missing keys",
        "Deletes dictionary because key missing"
      ],
      correctOption: 1,
      explanation: "pop(key, default) returns the value and removes the key, or returns the default if key is missing. Here, it returns None if 'key' doesn't exist (no error). Without the default argument, pop() raises KeyError for missing keys. Option 1 describes pop(key) without a default. Option 3 is wrong—return value is the default you provide, not a dict. Option 4 is wrong—nothing dramatic happens. This pattern is useful for 'try to remove and use this value, or use a default': user_id = cache.pop('user_id', None). It combines retrieval, removal, and fallback in one operation.",
      source: "Lesson 8: Dicts CRUD Operations"
    },
    {
      question: "Why does `list1 == list2` compare differently than `list1 is list2` for identical contents?",
      options: [
        "Both always return same result for lists",
        "== compares values; is compares object identity",
        "is compares values; == compares object identity",
        "== is invalid for list comparison operations"
      ],
      correctOption: 1,
      explanation: "The == operator compares values (content): [1, 2] == [1, 2] is True even if they're separate objects. The is operator compares identity (whether they're the same object in memory): [1, 2] is [1, 2] is False because Python creates two separate list objects. Option 1 is wrong—they differ. Option 3 reverses the operators. Option 4 is wrong—== works fine for lists. Use == to check 'do these lists contain the same items?', use is to check 'are these the same list object?'. In practice, you almost always want ==; is matters mainly for checking None (x is None) or debugging aliasing issues.",
      source: "Lesson 2: Lists Creation and Basics"
    },
    {
      question: "You're processing JSON API data with nested dicts and lists. Which structure preserves the natural JSON hierarchy?",
      options: [
        "Flatten everything to single-level dict immediately",
        "Keep nested dicts and lists matching JSON structure",
        "Convert all lists to tuples for immutability",
        "Use only lists to simplify access patterns"
      ],
      correctOption: 1,
      explanation: "Keep the nested structure matching JSON: user['address']['city'] directly reflects the JSON hierarchy. Flattening loses structure and makes updates harder. Option 1 (flattening) works but loses semantic grouping—user_address_city is less clear than user['address']['city']. Option 3 (converting to tuples) prevents modification, which you often need for API data. Option 4 (only lists) loses key-based access, requiring numeric indices. Preserving JSON structure makes code more maintainable: the data structure documents itself by matching the API shape.",
      source: "Lesson 10: Choosing the Right Structure"
    },
    {
      question: "What's the output of `fruits = ['apple', 'banana']; print('cherry' in fruits)`?",
      options: [
        "Prints True because in checks list contents",
        "Prints False because cherry not in list",
        "Raises error because in requires index",
        "Prints None because cherry is missing"
      ],
      correctOption: 1,
      explanation: "Prints False because 'cherry' is not in the list ['apple', 'banana']. The 'in' operator checks membership, returning True if the item exists, False otherwise. Option 1 is wrong—'cherry' isn't present. Option 3 is wrong—'in' checks membership, not indices. Option 4 is wrong—'in' returns a boolean (True/False), not None. The 'in' operator is used for membership testing before operations: if item in my_list: process(item). For large lists, this is O(n) (checks each item); for large dicts, 'in' checks keys in O(1) (instant). This performance difference matters when checking thousands of items.",
      source: "Lesson 2: Lists Creation and Basics"
    },
    {
      question: "When implementing a cache with size limits, which operation efficiently removes the oldest entry from the beginning?",
      options: [
        "Use list.pop(0) to remove first element",
        "Use deque.popleft() from collections module",
        "Slice to remove first element with list[1:]",
        "Use list.remove(list[0]) for oldest removal"
      ],
      correctOption: 1,
      explanation: "deque.popleft() is O(1) constant time—instant removal from the beginning. list.pop(0) is O(n) because all remaining items must shift left. For a cache with thousands of entries, deque is dramatically faster. Option 1 works but is slow. Option 3 (slicing) creates a new list and reassigns—even slower. Option 4 (remove) requires finding the item first, adding overhead. This is a real performance issue: using list.pop(0) in a loop for large lists creates O(n²) behavior. When you need frequent operations on both ends of a sequence, use deque, not list.",
      source: "Lesson 10: Choosing the Right Structure"
    },
    {
      question: "You ask AI to 'create a dict from two lists: keys and values.' It generates `dict(keys, values)`. What's wrong?",
      options: [
        "Should use dict.fromkeys(keys, values) constructor",
        "Should use dict(zip(keys, values)) to pair them",
        "dict() constructor only accepts single argument",
        "Should manually loop through and assign pairs"
      ],
      correctOption: 1,
      explanation: "Use dict(zip(keys, values)) to pair corresponding items: zip() creates tuples (key, value), dict() converts those tuples to a dictionary. Option 1 (fromkeys) creates {key: values for key in keys}—all keys get the same value, not corresponding values. Option 3 is wrong—dict() accepts various forms, just not two separate lists directly. Option 4 works but is unnecessarily verbose when zip() solves it elegantly. Understanding zip() is key: zip(['a', 'b'], [1, 2]) produces [('a', 1), ('b', 2)], perfect for dict construction. This pattern appears when processing parallel data arrays.",
      source: "Lesson 7: Dicts Key-Value Basics"
    },
    {
      question: "Why does `my_list.sort(reverse=True)` work for numbers but might fail for mixed-type lists?",
      options: [
        "reverse parameter only works with homogeneous types",
        "Python can't compare different types for ordering",
        "sort() requires all items be same type",
        "Mixed lists need custom comparison function key"
      ],
      correctOption: 1,
      explanation: "Python can't compare different types for ordering: '5' < 3 raises TypeError: '<' not supported between instances of 'str' and 'int'. sort() needs to compare items, which fails with mixed types. Option 1 is close but imprecise—the issue is comparison, not reverse. Option 3 is close but sort() doesn't check types upfront; it fails when comparing. Option 4 suggests a workaround (key parameter for custom comparison) but doesn't explain the root cause. The solution: ensure all items are comparable (all ints, all strings, or use key=str to convert during comparison). Understanding type compatibility for operations prevents errors.",
      source: "Lesson 4: Lists Sorting and Advanced Methods"
    },
    {
      question: "What does `{**dict1, **dict2}` do when both dicts have overlapping keys?",
      options: [
        "Raises error because duplicate keys are invalid",
        "Merges with dict2 values overwriting dict1 values",
        "Creates nested dict with both values preserved",
        "Keeps only unique keys from both dictionaries"
      ],
      correctOption: 1,
      explanation: "The ** unpacking operator merges dicts, with rightmost values winning for duplicate keys. {**{'a': 1, 'b': 2}, **{'b': 3, 'c': 4}} produces {'a': 1, 'b': 3, 'c': 4}—dict2's 'b': 3 overwrites dict1's 'b': 2. Option 1 is wrong—no error, last value wins. Option 3 is wrong—no nesting occurs. Option 4 is wrong—all keys are kept, but duplicate keys have one value (the latest). This is useful for config merging: base_config = {...}, user_config = {...}, final = {**base_config, **user_config} applies user overrides to defaults.",
      source: "Lesson 8: Dicts CRUD Operations"
    }
  ]}
/>
