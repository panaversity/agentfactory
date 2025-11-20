---
sidebar_position: 7
title: "Chapter 20: Set, Frozen Set, and GC Quiz"
---

# Chapter 20: Set, Frozen Set, and GC Quiz

Test your understanding of Python's hash-based data structures and automatic memory management—from O(1) lookups with sets to immutable frozensets and garbage collection internals.

<Quiz
  title="Chapter 20: Set, Frozen Set, and GC Assessment"
  questions={[    {
      question: "You create a list with duplicate values: `[1, 2, 3, 1, 2, 4]`. What happens when you convert it to a set?",
      options: [
        "Duplicates are marked as invalid",
        "Raises ValueError for duplicate elements",
        "Keeps duplicates in unordered positions",
        "Duplicates are removed automatically"
      ],
      correctOption: 3,
      explanation: "Sets automatically eliminate duplicates because of the uniqueness property. When you convert `[1, 2, 3, 1, 2, 4]` to a set, it becomes `{1, 2, 3, 4}`—the duplicate 1 and 2 are removed. Option 2 is wrong because Python doesn't raise errors for duplicates in sets; it silently removes them. Option 3 is wrong because sets don't keep duplicates at all, ordered or unordered. Option 4 is wrong because there's no marking mechanism; duplicates are simply eliminated during set creation. This automatic deduplication is a core set feature, making sets ideal for extracting unique values from collections with potential duplicates.",
      source: "Lesson 1: Set Basics"
    },
    {
      question: "Why does attempting `my_set = {[1, 2], [3, 4]}` raise a TypeError?",
      options: [
        "Lists aren't hashable because they're mutable",
        "Sets require homogeneous types only",
        "Lists can't be nested inside sets",
        "Sets don't support bracket notation"
      ],
      correctOption: 0,
      explanation: "Lists cannot be set elements because they're mutable (changeable), and mutable objects aren't hashable. Sets require hashable elements because they use hash values for O(1) lookup. If a list could be in a set and then you modified it, the hash value would change, breaking the set's internal structure. Option 2 is wrong because sets can contain mixed immutable types like `{1, 'a', (2, 3)}`. Option 3 is misleading—lists can't be nested because they're unhashable, not because of nesting restrictions. Option 4 is wrong because bracket notation has nothing to do with this error; the issue is mutability. To store sequences in sets, use immutable tuples instead: `{(1, 2), (3, 4)}`.",
      source: "Lesson 1: Set Basics"
    },
    {
      question: "What's the primary difference between `my_set.remove(x)` and `my_set.discard(x)` when `x` doesn't exist?",
      options: [
        "discard() raises KeyError; remove() does nothing",
        "remove() raises KeyError; discard() does nothing",
        "Both raise KeyError for missing elements",
        "Both silently ignore missing elements completely"
      ],
      correctOption: 1,
      explanation: "The key difference is error handling: `remove(x)` raises a KeyError if x doesn't exist in the set, while `discard(x)` silently does nothing. Use `remove()` when you're certain the element exists and want an error if it doesn't (defensive programming). Use `discard()` when you want to ensure an element is gone regardless of whether it was there initially. Option 2 reverses the behavior. Option 3 is wrong because `discard()` specifically doesn't raise errors. Option 4 is wrong because `remove()` does raise errors. This design mirrors the difference between dict access with brackets `dict[key]` (raises KeyError) versus `dict.get(key)` (returns None).",
      source: "Lesson 1: Set Basics"
    },
    {
      question: "You need to track unique user IDs from millions of login events. Which structure provides O(1) membership checking?",
      options: [
        "Tuple because immutability improves performance significantly",
        "List because indexed access is constant time",
        "Set because hash-based lookup is constant time",
        "Dict because key-value pairs are faster"
      ],
      correctOption: 2,
      explanation: "Sets provide O(1) average-case membership checking using hash tables. When you check `user_id in user_set`, Python computes the hash, jumps to the slot, and finds the element instantly—regardless of set size. With millions of IDs, this is dramatically faster than lists. Option 2 is wrong because list membership checking (`in` operator) is O(n)—it checks each element sequentially. Indexed access like `list[5]` is O(1), but membership checking is not. Option 3 is wrong because tuples have the same O(n) membership checking as lists. Option 4 is misleading—dicts also provide O(1) lookup, but only for keys. If you only need membership, sets are more appropriate than dicts.",
      source: "Lesson 1: Set Basics"
    },
    {
      question: "Why must you use `set()` to create an empty set instead of `{}`?",
      options: [
        "Empty sets need the constructor syntax",
        "Python requires explicit type hints always",
        "Braces are reserved for frozenset creation",
        "Empty braces create an empty dictionary"
      ],
      correctOption: 3,
      explanation: "The syntax `{}` creates an empty dictionary, not an empty set. This is a historical quirk: dictionaries existed in Python before sets, so `{}` was already claimed. To create an empty set, you must use the `set()` constructor. Option 2 is wrong because type hints are optional in Python, not required. Option 3 is wrong because frozensets use `frozenset()`, not braces. Option 4 is partially true but doesn't explain why—the real reason is that `{}` is already reserved for dicts. Non-empty sets can use braces: `{1, 2, 3}` is unambiguous because the presence of elements (not key-value pairs) signals it's a set.",
      source: "Lesson 1: Set Basics"
    },
    {
      question: "What's the result of `{1, 2, 3} | {3, 4, 5}` and what operation does `|` perform?",
      options: [
        "Union: {1, 2, 3, 4, 5} combining all elements",
        "Intersection: {3} containing only shared elements",
        "Difference: {1, 2} containing first set's unique items",
        "Symmetric difference: {1, 2, 4, 5} excluding shared"
      ],
      correctOption: 0,
      explanation: "The `|` operator performs set union, combining all elements from both sets. `{1, 2, 3} | {3, 4, 5}` produces `{1, 2, 3, 4, 5}`—all unique elements from both sets. The duplicate 3 appears only once because sets enforce uniqueness. Option 2 describes the `&` (intersection) operator, which produces `{3}`. Option 3 describes the `-` (difference) operator, which produces `{1, 2}`. Option 4 describes the `^` (symmetric difference) operator, which produces `{1, 2, 4, 5}`. Union is useful for combining datasets—like merging user sets from different sources while automatically removing duplicates.",
      source: "Lesson 2: Set Operations"
    },
    {
      question: "When would you use `set_a & set_b` instead of `set_a | set_b` in production code?",
      options: [
        "Combining all unique items from collections",
        "Finding common items between two collections",
        "Removing duplicates from merged data sets",
        "Checking if sets have identical contents"
      ],
      correctOption: 1,
      explanation: "The `&` (intersection) operator finds elements that exist in BOTH sets, making it ideal for finding commonalities. Use cases include: finding users who are in both groups, identifying shared tags, or discovering overlapping IDs. Option 2 describes union (`|`), which combines all elements. Option 3 is also union—merging and deduplicating. Option 4 describes equality checking (`==`), not intersection. Intersection is powerful in data analysis: if you have `active_users` and `premium_users`, then `active_users & premium_users` gives you active premium users specifically. This is more readable and efficient than looping through both sets manually.",
      source: "Lesson 2: Set Operations"
    },
    {
      question: "What does `{1, 2, 3} - {2, 3, 4}` produce and when is this operation useful?",
      options: [
        "Union: {1, 2, 3, 4} combining elements",
        "Intersection: {2, 3} containing shared elements only",
        "Difference: {1} containing first set's unique items",
        "Symmetric difference: {1, 4} excluding shared items"
      ],
      correctOption: 2,
      explanation: "The `-` (difference) operator produces `{1}`—elements in the first set but not in the second. This answers 'what does the first set have that the second doesn't?' Use cases include: finding users who haven't completed a task (all_users - completed_users), identifying missing items (required_ids - processed_ids), or filtering out unwanted elements. Option 2 describes `&` (intersection). Option 3 describes `|` (union). Option 4 describes `^` (symmetric difference). Difference is asymmetric: `A - B` differs from `B - A`. In the example, `{2, 3, 4} - {1, 2, 3}` would give `{4}`, not `{1}`.",
      source: "Lesson 2: Set Operations"
    },
    {
      question: "Why does `{1, 2, 3}.issubset({1, 2, 3, 4, 5})` return True while `{1, 2, 6}.issubset({1, 2, 3, 4, 5})` returns False?",
      options: [
        "First set's all elements exist in second",
        "Subsets require exactly half the elements match",
        "Second set must contain duplicate values",
        "Sets must share at least one element"
      ],
      correctOption: 0,
      explanation: "A subset relationship means every element in the first set exists in the second set. `{1, 2, 3}` is a subset of `{1, 2, 3, 4, 5}` because all three elements (1, 2, 3) are present in the larger set. `{1, 2, 6}` is not a subset because 6 doesn't exist in the second set. Option 2 is wrong—subsets don't have size requirements; even `{1}` is a subset of `{1, 2, 3}`. Option 3 is wrong because sets never contain duplicates. Option 4 describes set overlap, not subset. Use cases for `issubset()`: checking if user permissions are within allowed permissions, verifying required features are available, or testing if dependencies are satisfied.",
      source: "Lesson 2: Set Operations"
    },
    {
      question: "What's the output of `{x**2 for x in range(6) if x % 2 == 0}` and why?",
      options: [
        "Set {0, 4, 16} of squared evens"
      ],
      correctOption: 0,
      explanation: "Set comprehensions use `{...}` syntax with an expression and optional filter. This produces `{0, 4, 16}`—the squares of even numbers from range(6). The `if x % 2 == 0` filter selects evens (0, 2, 4), then `x**2` squares each (0, 4, 16). Option 2 is wrong because `{...}` creates a set, not a list. List comprehensions use `[...]`. Option 3 is wrong because the filter removes odds (1, 3, 5). Option 4 is wrong because the expression `x**2` squares values; it doesn't just filter evens. Set comprehensions are useful for building unique transformed values: `{user.email for user in users if user.is_active}` creates unique active user emails.",
      source: "Lesson 2: Set Operations"
    },
    {
      question: "You ask AI to 'find common tags between two blog posts.' It generates `set_a | set_b`. What should you request?",
      options: [
        "Use difference (-) to find unique tags",
        "Union is correct for finding shared tags",
        "Use intersection (&) instead for common items",
        "Use symmetric difference (^) for matching tags"
      ],
      correctOption: 2,
      explanation: "Finding common items requires intersection (`&`), not union (`|`). Union combines all unique elements from both sets, which is the opposite of finding shared elements. The correct code is `set_a & set_b`, which returns elements present in BOTH sets. Option 2 is wrong—union doesn't find shared items; it merges all items. Option 3 is wrong—difference finds items unique to one set, not common to both. Option 4 is wrong—symmetric difference finds items in either set but NOT both, which excludes common items. This is a common AI-generation error: confusing union with intersection. Always verify that the operation matches your intent: & for commonalities, | for merging.",
      source: "Lesson 2: Set Operations"
    },
    {
      question: "Why is checking `user_id in active_users` O(1) for sets but O(n) for lists with millions of users?",
      options: [
        "Lists use binary search for faster access",
        "Sets use hash tables for instant lookup",
        "Sets pre-sort elements for quick searching",
        "Lists require sequential scanning through all items"
      ],
      correctOption: 1,
      explanation: "Sets use hash tables, which enable O(1) average-case lookup. When you check `user_id in active_users`, Python computes the hash of `user_id`, jumps directly to that slot in the hash table, and checks if it's there—instant, regardless of set size. Lists require O(n) linear search: Python checks element 0, then 1, then 2, until it finds the value or reaches the end. With 1 million users, set lookup is nearly instant; list lookup could check thousands of elements. Option 2 is wrong—lists don't use binary search for the `in` operator (binary search requires sorted data and isn't automatic). Option 3 is wrong—sets are unordered and don't sort. Option 4 describes the problem, not the solution. This performance difference is why sets are critical for large-scale membership checking.",
      source: "Lesson 3: Set Internals & Hashing"
    },
    {
      question: "What makes an object hashable in Python, and why does this matter for sets?",
      options: [
        "Hashability requires implementing custom __hash__ methods",
        "All objects are hashable by default",
        "Hashable objects must be numeric types",
        "Immutability ensures stable hash values for lookup"
      ],
      correctOption: 3,
      explanation: "Hashable objects are immutable, meaning their hash value never changes after creation. Sets require hashable elements because they store items in hash table slots based on hash values. If an object could change after being added to a set, its hash would change, and lookups would break. Immutable types (int, str, tuple, frozenset) are hashable; mutable types (list, dict, set) are not. Option 2 is wrong—mutable objects like lists aren't hashable. Option 3 is wrong—strings and tuples are hashable despite being non-numeric. Option 4 is wrong—built-in immutable types are automatically hashable without custom code. The connection: sets use hash(element) to determine storage location, so hash stability is essential for correctness.",
      source: "Lesson 3: Set Internals & Hashing"
    },
    {
      question: "Why does `hash('hello')` return the same value every time within a session but might differ between Python sessions?",
      options: [
        "Hashing is deterministic per session for consistency",
        "Hashing uses random values for every call",
        "String hashes are cached globally forever",
        "Hashes change based on string length"
      ],
      correctOption: 0,
      explanation: "Hash functions are deterministic within a session—the same input always produces the same hash value. This ensures set lookups work correctly: if you add 'hello' to a set, you need `hash('hello')` to return the same value when checking membership later. However, Python uses hash randomization across sessions for security reasons (prevents hash collision attacks). Option 2 is wrong—hashes must be consistent within a session. Option 3 is wrong—hashes aren't cached globally; they're computed each time but produce consistent results. Option 4 is wrong—hashes aren't based on length; they're computed from content. This consistency matters because sets rely on stable hashing: `my_set.add('hello')` followed by `'hello' in my_set` must find the element.",
      source: "Lesson 3: Set Internals & Hashing"
    },
    {
      question: "What happens when you try to add a dictionary to a set: `my_set.add({'key': 'value'})`?",
      options: [
        "Adds dictionary successfully as a set element",
        "Converts dictionary to frozenset automatically for storage",
        "Raises TypeError because dicts aren't hashable",
        "Adds dictionary keys but ignores values"
      ],
      correctOption: 2,
      explanation: "Dictionaries are mutable, so they're unhashable and cannot be set elements. Attempting `my_set.add({'key': 'value'})` raises `TypeError: unhashable type: 'dict'`. If dicts could be in sets, you could modify the dict after adding it, changing its hash value and breaking the set's lookup structure. Option 2 is wrong—Python doesn't automatically convert types. Option 3 is wrong—it fails with an error. Option 4 is wrong—no partial adding occurs. To store dict-like data in sets, use frozensets (immutable set variant) or tuples of key-value pairs: `{(('key', 'value'),)}` works because tuples are immutable and hashable.",
      source: "Lesson 3: Set Internals & Hashing"
    },
    {
      question: "When comparing sets vs. lists for 1 million elements, which operation shows the most dramatic performance difference?",
      options: [
        "Iteration speed is dramatically faster for sets",
        "Membership checking (in operator) shows dramatic difference",
        "Memory usage is dramatically lower for sets",
        "Adding elements is dramatically faster for sets"
      ],
      correctOption: 1,
      explanation: "Membership checking (`x in collection`) shows the most dramatic performance difference: O(1) for sets vs. O(n) for lists. With 1 million elements, set lookup is nearly instant; list lookup averages 500,000 comparisons. The difference can be 1000x or more. Option 2 is wrong—iteration is similar speed for both (O(n) for both). Option 3 is wrong—sets often use MORE memory than lists due to hash table overhead. Option 4 is wrong—adding elements is fast for both in most cases. The key insight: when you need frequent membership checks on large collections, sets are dramatically faster. This matters in production: checking if user IDs exist, validating against allowed values, or deduplicating large datasets.",
      source: "Lesson 3: Set Internals & Hashing"
    },
    {
      question: "How do hash collisions affect set performance, and why is O(1) only average case?",
      options: [
        "Collisions only occur with poorly designed hashes",
        "Collisions improve performance by grouping similar items",
        "Python eliminates all collisions automatically with resizing",
        "Collisions slow lookups; worst case is linear time"
      ],
      correctOption: 3,
      explanation: "Hash collisions occur when multiple objects hash to the same slot. Sets handle collisions through chaining or probing, but this adds overhead. In the worst case (all elements collide to one slot), lookup degrades to O(n). However, Python's hash functions and automatic resizing make collisions rare, so O(1) average case is reliable. Option 2 is wrong—collisions slow performance, not improve it. Option 3 is wrong—resizing reduces collisions but can't eliminate them entirely. Option 4 is wrong—collisions are mathematically inevitable when the hash space is smaller than the number of possible objects. The practical takeaway: O(1) is 'average case,' but even with collisions, sets are dramatically faster than lists for membership checks.",
      source: "Lesson 3: Set Internals & Hashing"
    },
    {
      question: "What's the primary difference between `set()` and `frozenset()` in terms of mutability?",
      options: [
        "Frozensets cannot be modified after creation",
        "Sets cannot be modified after creation",
        "Both are mutable and can add elements",
        "Both are immutable and cannot change"
      ],
      correctOption: 0,
      explanation: "Frozensets are immutable—they cannot be modified after creation. They have no `.add()`, `.remove()`, or `.discard()` methods. Regular sets are mutable and can be changed anytime with `.add()`, `.remove()`, etc. This immutability makes frozensets hashable, enabling them to be dictionary keys or members of other sets. Option 2 reverses the behavior—sets ARE mutable. Option 3 is wrong because frozensets don't have mutation methods. Option 4 is wrong because regular sets are mutable. The trade-off: frozensets sacrifice modification capability to gain hashability. Choose frozensets when you need immutability or want to use sets as dict keys.",
      source: "Lesson 4: Frozensets"
    },
    {
      question: "Why can frozensets be dictionary keys while regular sets cannot?",
      options: [
        "Frozensets use special dictionary-compatible storage",
        "Frozensets are immutable and therefore hashable",
        "Sets are hashable but inefficient as keys",
        "Python automatically converts sets to frozensets"
      ],
      correctOption: 1,
      explanation: "Dictionary keys must be hashable, and hashable objects must be immutable. Frozensets are immutable (can't change after creation), making them hashable. Regular sets are mutable, so they're unhashable and cannot be dict keys. Attempting `{my_set: value}` raises `TypeError: unhashable type: 'set'`. Option 2 is wrong—there's no special storage; immutability is the requirement. Option 3 is wrong—sets aren't hashable at all. Option 4 is wrong—Python doesn't auto-convert; you must explicitly create frozensets. Use case: permission levels as frozenset keys: `{frozenset(['admin']): 'full_access'}` works because frozensets can't be modified, ensuring dict lookup integrity.",
      source: "Lesson 4: Frozensets"
    },
    {
      question: "You need to store groups of user IDs where each group is a set. Which structure works?",
      options: [
      ],
      correctOption: 0,
      explanation: "Sets can only contain hashable elements. Frozensets are hashable (immutable), so `set[frozenset[int]]` works: `{frozenset([1, 2]), frozenset([3, 4])}`. Regular sets and lists are unhashable (mutable), so they cannot be set members. Option 2 fails with `TypeError: unhashable type: 'set'`. Option 3 works but doesn't provide set-level uniqueness or O(1) group lookup. Option 4 fails with `TypeError: unhashable type: 'list'`. This pattern is common in data analysis: tracking unique groups, modeling relationships, or finding group overlaps. The nested structure allows set operations on groups: `groups_a & groups_b` finds shared groups.",
      source: "Lesson 4: Frozensets"
    },
    {
      question: "What happens when you try to modify a frozenset: `my_frozen.add(5)`?",
      options: [
        "Silently ignores the add operation completely",
        "Raises TypeError because frozensets are immutable",
        "Creates a new frozenset with element added",
        "Raises AttributeError because add method doesn't exist"
      ],
      correctOption: 3,
      explanation: "Frozensets don't have mutation methods like `.add()`, `.remove()`, or `.discard()`. Attempting `my_frozen.add(5)` raises `AttributeError: 'frozenset' object has no attribute 'add'`. The error is AttributeError (method doesn't exist), not TypeError (operation not supported). Option 2 uses the wrong error type. Option 3 is wrong—frozensets don't create modified copies automatically; you must manually create a new frozenset. Option 4 is wrong—an error is raised, not silently ignored. If you need to 'modify' a frozenset, create a new one: `new_frozen = frozenset(list(my_frozen) + [5])`. This immutability enforces the hashability contract.",
      source: "Lesson 4: Frozensets"
    },
    {
      question: "When would you choose `frozenset()` over `set()` in production code?",
      options: [
        "When using as dict keys or set members",
        "When you need to add elements frequently",
        "When performance is critical for modifications",
        "When you need to remove elements later"
      ],
      correctOption: 0,
      explanation: "Choose frozensets when you need hashability: as dictionary keys, as members of other sets, or when data shouldn't change. Common use cases include: permission levels as dict keys, immutable group membership, or configuration constants. Options 2, 3, and 4 all require modification capabilities, which frozensets don't provide—use regular sets for those scenarios. Frozensets trade modification ability for hashability and immutability guarantees. The decision framework: if you need to use a set as a key or member, you must use frozenset. If you need to modify contents, use regular set. If both requirements exist, reconsider your data model.",
      source: "Lesson 4: Frozensets"
    },
    {
      question: "How do frozenset operations like union and intersection differ from regular set operations?",
      options: [
        "Frozenset operations return regular sets always",
        "Frozenset operations modify the original frozenset",
        "Both return frozensets; neither modifies original frozensets",
        "Frozensets don't support union or intersection"
      ],
      correctOption: 2,
      explanation: "Frozenset operations (`|`, `&`, `-`, `^`) work identically to set operations but return frozensets and never modify originals (since frozensets are immutable). `frozenset([1, 2]) | frozenset([3, 4])` produces `frozenset({1, 2, 3, 4})`. Option 2 is wrong—frozensets cannot be modified. Option 3 is wrong—operations return frozensets when applied to frozensets. Option 4 is wrong—frozensets support all read-only operations. The key difference from sets: with sets, you can use `.update()` to modify in-place; with frozensets, all operations create new frozensets. This immutability ensures hash values remain stable.",
      source: "Lesson 4: Frozensets"
    },
    {
      question: "You ask AI to create a permission lookup system with role combinations as keys. It uses `dict[set[str], str]`. What's the issue?",
      options: [
        "Sets work fine as dictionary keys",
        "Use frozenset keys because sets aren't hashable",
        "Use list keys instead for mutability",
        "Convert sets to strings for key usage"
      ],
      correctOption: 1,
      explanation: "Dictionary keys must be hashable, and regular sets are unhashable (mutable). The code `dict[set[str], str] = {{'admin'}: 'access'}` raises `TypeError: unhashable type: 'set'`. Use frozensets instead: `dict[frozenset[str], str]` with `{frozenset(['admin']): 'access'}` works perfectly. Option 2 is wrong—sets cannot be dict keys. Option 3 is wrong—lists also aren't hashable. Option 4 works but loses the semantic meaning of 'a set of roles' and makes comparisons awkward. This is a common AI code generation error: using sets where frozensets are required. Always check hashability requirements when using sets as keys or nested in other sets.",
      source: "Lesson 4: Frozensets"
    },
    {
      question: "What's Python's primary mechanism for automatically freeing unused memory?",
      options: [
        "Reference counting tracks usage and frees at zero",
        "Periodic garbage collection scans all objects",
        "Manual memory deallocation by developer code",
        "Objects are freed at program termination only"
      ],
      correctOption: 0,
      explanation: "Python uses reference counting as the primary memory management mechanism. Every object has a reference count: how many variables point to it. When you create an object, count starts at 1. When you add a reference, it increases. When you delete a reference (variable goes out of scope or is deleted), it decreases. When the count reaches zero, Python immediately frees the memory. Option 2 describes the cyclic garbage collector, which is a backup for circular references, not the primary mechanism. Option 3 is wrong—Python is automatic, not manual. Option 4 is wrong—memory is freed throughout execution, not just at termination. Reference counting enables immediate reclamation: `x = [1, 2, 3]; del x` frees the list immediately.",
      source: "Lesson 5: Garbage Collection"
    },
    {
      question: "Why can reference counting alone fail to free memory with circular references?",
      options: [
        "Python disables counting for performance with cycles",
        "Reference counting doesn't work with mutable objects",
        "Circular references bypass the counting mechanism",
        "Objects reference each other so counts never reach zero"
      ],
      correctOption: 3,
      explanation: "Circular references occur when objects reference each other: `a.ref = b; b.ref = a`. Each object's reference count is at least 1 (from the other object), even when no external variables reference them. Reference counting alone can't detect this: the counts never reach zero, so memory is never freed. Python's cyclic garbage collector solves this by periodically finding unreachable cycles and freeing them. Option 2 is wrong—reference counting works fine with mutable objects. Option 3 is wrong—circular references don't bypass counting; they just prevent counts from reaching zero. Option 4 is wrong—counting is always active. Real-world example: tree structures, linked lists, or graph nodes often create circular references.",
      source: "Lesson 5: Garbage Collection"
    },
    {
      question: "What does `gc.collect()` do and when should you call it manually?",
      options: [
        "Counts total objects currently in memory",
        "Prevents garbage collection for performance optimization",
        "Forces garbage collection; rarely needed in normal code",
        "Disables reference counting to save overhead"
      ],
      correctOption: 2,
      explanation: "`gc.collect()` forces an immediate garbage collection cycle, running the cyclic garbage collector to find and free circular references. You rarely need to call it manually because Python automatically runs GC periodically. Use cases for manual collection: after creating many temporary objects (data processing loop), before measuring memory usage (to get accurate counts), or when debugging memory leaks. Option 2 is wrong—it triggers collection, not prevents it. Option 3 is wrong—that's `len(gc.get_objects())`. Option 4 is wrong—it doesn't disable reference counting. Most production code never calls `gc.collect()` explicitly because automatic collection works well. Over-using it can hurt performance.",
      source: "Lesson 5: Garbage Collection"
    },
    {
      question: "How does Python's generational garbage collection optimize performance?",
      options: [
        "All objects are checked every collection cycle",
        "Objects are grouped by age; young checked more frequently",
        "Old objects are never garbage collected",
        "Generation tracking replaces reference counting entirely"
      ],
      correctOption: 1,
      explanation: "Python divides objects into three generations (0, 1, 2) based on age. Newly created objects are generation 0. If they survive a GC cycle, they're promoted to generation 1, then generation 2. Generation 0 is checked most frequently because most objects die young (temporary variables, intermediate results). Older generations are checked less often because long-lived objects rarely become garbage. Option 2 is wrong—generational GC specifically avoids checking everything. Option 3 is wrong—old objects ARE checked, just less frequently. Option 4 is wrong—generations optimize cyclic GC, not replace reference counting. This optimization improves performance: checking thousands of long-lived objects on every GC cycle would be wasteful.",
      source: "Lesson 5: Garbage Collection"
    },
    {
      question: "What does `sys.getrefcount(obj)` return and why is the value often higher than expected?",
      options: [
        "Returns generation number for garbage collector",
        "Returns memory size in bytes for object",
        "Returns number of variables sharing the object",
        "Returns reference count including temporary function parameter reference"
      ],
      correctOption: 3,
      explanation: "`sys.getrefcount(obj)` returns the reference count for `obj`, but the value is typically 1 higher than you expect because passing `obj` to the function creates a temporary reference. If you have `x = [1, 2, 3]` and call `sys.getrefcount(x)`, you'd expect 1 (just `x`), but it returns 2 (x + the parameter inside getrefcount). Option 2 describes `sys.getsizeof()`. Option 3 is partially correct but misses the temporary parameter reference. Option 4 describes GC generation tracking, not refcount. Practical use: debugging memory issues by checking if objects have unexpected references preventing deletion.",
      source: "Lesson 5: Garbage Collection"
    },
    {
      question: "When debugging memory leaks, which `gc` module function helps identify what objects are consuming memory?",
      options: [
        "gc.get_objects() returns all tracked objects",
        "gc.collect() displays memory usage statistics",
        "gc.free() shows unreferenced objects only",
        "gc.count() returns memory in megabytes"
      ],
      correctOption: 0,
      explanation: "`gc.get_objects()` returns a list of all objects tracked by the garbage collector, enabling memory profiling. You can analyze this list to find types consuming memory: `from collections import Counter; Counter(type(obj).__name__ for obj in gc.get_objects())` shows counts by type. Option 2 is wrong—`gc.collect()` runs collection and returns number of objects freed, but doesn't display stats. Option 3 is wrong—no `gc.free()` function exists. Option 4 is wrong—`gc.get_count()` returns generation counts, not memory size. Real-world use: finding memory leaks by taking snapshots before and after operations, then comparing object counts to identify what's not being freed.",
      source: "Lesson 5: Garbage Collection"
    },
    {
      question: "Why does deleting all references to an object with `del` immediately free memory in most cases?",
      options: [
        "Python schedules deletion for next GC cycle",
        "Reference count reaches zero triggering immediate deallocation",
        "Objects are marked for deletion but freed later",
        "Manual deletion bypasses reference counting system"
      ],
      correctOption: 1,
      explanation: "When you delete the last reference to an object, its reference count drops to zero, and Python immediately frees the memory. This is reference counting in action: `x = [1, 2, 3]; del x` instantly frees the list. No waiting for GC cycles. Option 2 is wrong—immediate freeing occurs, not scheduled. Option 3 is wrong—freeing happens immediately for reference-counted objects. Option 4 is wrong—`del` decrements reference count; it doesn't bypass the system. Exception: circular references won't be freed immediately because their counts won't reach zero. The cyclic GC handles those later. This immediate reclamation makes Python memory-efficient for most common patterns.",
      source: "Lesson 5: Garbage Collection"
    },
    {
      question: "What's the relationship between reference counting and garbage collection in Python's memory management?",
      options: [
        "They're independent systems that never interact",
        "Garbage collection is primary; reference counting is backup",
        "Reference counting handles most cases; GC handles circular references",
        "Reference counting was replaced by garbage collection"
      ],
      correctOption: 2,
      explanation: "Reference counting is Python's primary memory management mechanism, handling the vast majority of objects efficiently. Garbage collection (cyclic GC) is a supplementary system specifically designed to handle circular references that reference counting can't detect. Most objects are freed immediately when their refcount hits zero; only objects in cycles require GC intervention. Option 2 reverses the relationship. Option 3 is wrong—they complement each other. Option 4 is wrong—reference counting is still active and primary. The combination provides both efficiency (immediate reclamation via refcounting) and completeness (cycle detection via GC).",
      source: "Lesson 5: Garbage Collection"
    },
    {
      question: "You create a memory profiler that tracks object creation. Which combination of tools captures creation and deletion events?",
      options: [
        "Use gc.get_objects() before and after operations",
        "Monitor sys.getrefcount() for all variables continuously",
        "Call gc.collect() repeatedly to track changes",
        "Use set operations to track only additions"
      ],
      correctOption: 0,
      explanation: "Taking snapshots with `gc.get_objects()` before and after operations, then comparing the lists, reveals what was created and what was freed. You can use set operations on object IDs to find differences: `created = set(id(obj) for obj in snapshot_after) - set(id(obj) for obj in snapshot_before)`. Option 2 is impractical—monitoring refcounts for all variables isn't feasible. Option 3 is wrong—`gc.collect()` doesn't provide tracking; it just runs collection. Option 4 is incomplete—you need both creation and deletion tracking, plus object type information. Real memory profilers use this approach with additional filters to track specific object types and sizes.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "Why are sets ideal for tracking unique object IDs in a memory profiler?",
      options: [
        "Sets preserve insertion order for tracking",
        "Sets store objects sorted by memory address",
        "Sets use less memory than lists",
        "O(1) membership checking and automatic deduplication"
      ],
      correctOption: 3,
      explanation: "Sets provide O(1) membership checking (`id in seen_ids`) and automatically eliminate duplicate IDs. When tracking thousands of objects, fast lookup matters: checking if an ID was already seen is instant with sets but would be O(n) with lists. Option 2 is wrong—sets are unordered, not sorted. Option 3 is wrong—sets often use more memory than lists due to hash table overhead. Option 4 is wrong in spirit—while modern Python sets maintain insertion order, that's not why they're ideal here; performance is the key benefit. For profiling: `seen_ids = set(); new_id = id(obj); if new_id not in seen_ids: seen_ids.add(new_id)` efficiently tracks unique objects.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "How would you use frozensets to track immutable snapshots of object states in a profiler?",
      options: [
        "Use frozensets to modify object states",
        "Store frozensets as dict keys for state lookup",
        "Frozensets prevent garbage collection of tracked objects",
        "Frozensets replace gc.get_objects() for profiling"
      ],
      correctOption: 1,
      explanation: "Frozensets can serve as dictionary keys to map immutable states to metadata: `state_map[frozenset(obj_ids)] = {'timestamp': ..., 'count': ...}`. This enables tracking when specific object combinations existed. Option 2 is wrong—frozensets are immutable and can't modify anything. Option 3 is wrong—storing object IDs doesn't prevent GC (IDs aren't references). Option 4 is wrong—frozensets don't replace `gc.get_objects()`; they complement it for analysis. Use case: tracking unique sets of live objects over time, where the set itself becomes the lookup key for historical data: `snapshots_seen = {frozenset([id1, id2, id3]): 'snapshot_1'}`.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "When profiling memory with gc.get_objects(), why should you call gc.collect() first?",
      options: [
        "Makes gc.get_objects() return fewer results faster",
        "Prevents new objects from being created",
        "Ensures unreachable objects are freed before counting",
        "Disables reference counting for accurate measurements"
      ],
      correctOption: 2,
      explanation: "Calling `gc.collect()` before `gc.get_objects()` ensures that any unreachable circular references are freed, giving you an accurate count of truly live objects. Without it, you might see objects that are technically garbage but haven't been collected yet. Option 2 is wrong—GC doesn't prevent object creation. Option 3 is technically true but misses the point—you want accurate counts, not just fewer results. Option 4 is wrong—GC doesn't disable reference counting. Profiling pattern: `gc.collect(); before = len(gc.get_objects()); # do work; gc.collect(); after = len(gc.get_objects()); print(f'Net objects created: {after - before}')`.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "How do you count occurrences of each object type in gc.get_objects() output?",
      options: [
        "Use set to eliminate duplicates first",
        "Manually iterate and increment dictionary counts",
        "Sort objects by type then count groups",
        "Use Counter with type names from objects"
      ],
      correctOption: 3,
      explanation: "The most Pythonic approach uses `Counter` from collections: `from collections import Counter; type_counts = Counter(type(obj).__name__ for obj in gc.get_objects())`. This creates a dictionary-like object mapping type names to counts. Option 2 works but is verbose compared to Counter. Option 3 is inefficient—sorting then counting is slower than Counter's hash-based counting. Option 4 misunderstands the task—we want counts of each type, not unique types. Counter is ideal for this pattern: it handles incrementing, initialization, and provides a clean summary: `type_counts.most_common(10)` shows the 10 most common types.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "What insight does comparing frozenset[IDs] before and after operations provide in memory profiling?",
      options: [
        "Shows which exact objects were created or destroyed",
        "Improves garbage collection performance significantly",
        "Prevents memory leaks by freezing references",
        "Converts mutable objects to immutable automatically"
      ],
      correctOption: 0,
      explanation: "Storing object IDs as frozensets enables set operations to find differences: `created = after_ids - before_ids` shows new objects, `destroyed = before_ids - after_ids` shows freed objects, `survived = before_ids & after_ids` shows persistent objects. Frozensets work as dict keys, enabling you to map ID sets to metadata. Option 2 is wrong—this is analysis, not optimization. Option 3 is wrong—storing IDs doesn't prevent leaks or affect GC. Option 4 is wrong—no automatic conversion occurs. Practical profiling: `before = frozenset(id(obj) for obj in gc.get_objects()); # operation; after = frozenset(id(obj) for obj in gc.get_objects()); leaked = after - before`.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "Why might tracking only object counts (not individual IDs) miss important memory leak patterns?",
      options: [
        "Counts increase naturally so leaks aren't visible",
        "Object counts are always inaccurate with GC",
        "Counts hide which specific objects are leaking",
        "Individual IDs are required for garbage collection"
      ],
      correctOption: 2,
      explanation: "Aggregate counts tell you 'how many' but not 'which ones.' If you see 1000 dict objects before and 1000 after, you might think nothing changed. But if 200 were freed and 200 new ones created, tracking IDs would reveal this churn. Leaks manifest as specific objects persisting across operations. Option 2 is wrong—counts are accurate if you collect first. Option 3 is wrong—steady growth would show in counts, but subtle leaks might balance out. Option 4 is wrong—IDs aren't required for GC, just for detailed tracking. Best practice: track both counts (overall trends) and IDs (specific leak identification).",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "You create a memory profiler that stores snapshots in a dict with frozenset keys. Why is this structure powerful for analysis?",
      options: [
        "Prevents garbage collection of tracked objects permanently",
        "Enables fast lookup of historical states by object sets",
        "Automatically frees memory when snapshots aren't needed",
        "Converts all objects to immutable forms"
      ],
      correctOption: 1,
      explanation: "Using frozensets as dict keys enables powerful queries: `snapshots[frozenset(current_ids)]` instantly finds if this exact set of objects existed before, enabling cycle detection and state comparison. You can track: 'Have I seen these exact objects together before?' Option 2 is wrong—storing IDs doesn't prevent GC; IDs are just numbers, not references. Option 3 is wrong—you must manually delete snapshots to free memory. Option 4 is wrong—IDs don't convert objects. Real analysis: `if frozenset(live_ids) in historical_snapshots: print('Memory state has repeated—possible leak pattern')`.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "When would you choose a list over a set for tracking objects in a memory profiler?",
      options: [
        "When you need to preserve observation order",
        "When you want O(1) membership checking",
        "When you need automatic duplicate elimination",
        "When you're tracking millions of objects"
      ],
      correctOption: 0,
      explanation: "Lists preserve insertion order and maintain duplicates, useful when tracking creation sequence matters: seeing objects in the order they were allocated. Sets eliminate duplicates and are unordered (though modern Python maintains insertion order, the semantic purpose differs). Option 2 describes why you'd choose sets, not lists. Option 3 also describes sets. Option 4 strongly favors sets due to performance. Use lists when: timeline analysis (when was each object created?), tracking duplicate creations (same object created multiple times), or maintaining event sequences. Use sets when: uniqueness matters, membership checks are frequent, or order doesn't matter.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "How does combining set operations with gc.get_objects() enable leak detection in long-running applications?",
      options: [
        "gc.get_objects() returns sets instead of lists",
        "Set operations automatically free leaked memory",
        "Sets prevent circular references from forming",
        "Compare snapshots to identify persistent objects"
      ],
      correctOption: 3,
      explanation: "Taking periodic snapshots and using set operations reveals leaks: `leaked = snapshot_now - snapshot_earlier` identifies objects that should have been freed but weren't. Over multiple iterations, `always_present = snapshot_1 & snapshot_2 & snapshot_3` finds persistent objects. If unexpected objects persist across iterations where they should have been freed, you've found a leak. Option 2 is wrong—analysis doesn't free memory automatically. Option 3 is wrong—sets don't prevent circular references in your code. Option 4 is wrong—`gc.get_objects()` returns a list. Pattern: `baseline = set(id(obj) for obj in gc.get_objects()); # run operation 100 times; current = set(id(obj) for obj in gc.get_objects()); growth = current - baseline`.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "What's the advantage of storing metadata about object types using frozenset keys in profiling?",
      options: [
        "Frozensets consume less memory than regular sets",
        "Immutable keys prevent accidental modification of historical data",
        "Frozensets automatically update when objects are freed",
        "Type metadata requires immutable storage by Python"
      ],
      correctOption: 1,
      explanation: "Using frozensets as dict keys ensures your historical data can't be accidentally modified. Once you store `metadata[frozenset(ids)] = {'timestamp': ..., 'types': ...}`, that key is immutable, preventing bugs from key mutations. Option 2 is wrong—frozensets don't save memory; they enable hashability. Option 3 is wrong—frozensets are snapshots; they don't auto-update. Option 4 is wrong—there's no Python requirement; it's a design choice for data integrity. Benefits: time-series analysis (how object distribution changed over time), leak pattern recognition (which object type combinations persist), and safe historical lookups without risk of key mutation bugs.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "Why would a profiler use `{type(obj).__name__: id(obj)}` dicts stored in frozensets for snapshots?",
      options: [
        "Frozensets automatically serialize dictionaries efficiently",
        "Enables grouping objects by type with IDs",
        "This approach is flawed; frozensets can't contain dicts",
        "Dict storage prevents garbage collection issues"
      ],
      correctOption: 2,
      explanation: "This is a trick question highlighting a common mistake. Frozensets can only contain hashable elements, and dicts are unhashable (mutable). Attempting `frozenset([{'type': 'list', 'id': 123}])` raises `TypeError: unhashable type: 'dict'`. To store type-ID mappings, use tuples: `frozenset([('list', 123), ('dict', 456)])` or separate data structures. Option 2 describes what you might WANT to do, but the approach doesn't work. Options 3 and 4 are wrong because the entire premise is flawed. Lesson: always verify hashability requirements when designing data structures involving sets or frozensets.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "In a memory profiler, what does `len(gc.get_objects())` after `gc.collect()` represent?",
      options: [
        "Count of objects waiting for collection",
        "Total memory usage in bytes",
        "Number of objects leaked since program start",
        "Count of all currently reachable objects"
      ],
      correctOption: 3,
      explanation: "`gc.collect()` frees all unreachable circular references, so `len(gc.get_objects())` afterward represents truly reachable objects—objects still in use or referenced somewhere. This is your baseline: everything Python is actively tracking. Option 2 is wrong—`len()` counts objects, not bytes. Option 3 is wrong—it's not leaks; it's all reachable objects (most are legitimate). Option 4 is wrong—objects waiting for collection were just freed by `gc.collect()`. Use this for trend analysis: if this count grows steadily in a long-running app despite freeing temporary objects, you likely have a memory leak. Tracking `len(gc.get_objects())` over time reveals memory health.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "How would you implement a 'top memory consumers' feature showing which object types use the most memory?",
      options: [
        "Combine Counter and sys.getsizeof for type-based memory totals",
        "Sort gc.get_objects() by object ID naturally",
        "Use sets to count unique object types",
        "Frozensets automatically track memory consumption patterns"
      ],
      correctOption: 0,
      explanation: "Combine `Counter` for type aggregation with `sys.getsizeof()` for size measurement: `from collections import defaultdict; mem_by_type = defaultdict(int); for obj in gc.get_objects(): mem_by_type[type(obj).__name__] += sys.getsizeof(obj)`. Then sort by value to find top consumers. Option 2 is wrong—IDs don't correlate with memory size. Option 3 is wrong—sets count unique types but don't measure memory. Option 4 is wrong—frozensets don't automatically track anything. This analysis reveals: are dicts consuming gigabytes? Are there thousands of small objects adding up? Should you optimize specific types?",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "Why is it important to take gc snapshots BEFORE starting timed operations in profiling?",
      options: [
        "Makes gc.get_objects() run faster during operation",
        "Prevents garbage collection during the operation",
        "Establishes baseline for comparison of memory growth",
        "Ensures objects are immutable during profiling"
      ],
      correctOption: 2,
      explanation: "Taking a 'before' snapshot establishes your baseline: what objects existed before the operation. After the operation, taking an 'after' snapshot enables comparison: `growth = after - before` shows net object creation. Without a baseline, you can't distinguish objects created by your operation from pre-existing objects. Option 2 is wrong—snapshots don't prevent GC. Option 3 is wrong—snapshot timing doesn't affect performance. Option 4 is wrong—snapshots don't make objects immutable. Profiling pattern: `gc.collect(); before = set(id(obj) for obj in gc.get_objects()); run_operation(); gc.collect(); after = set(id(obj) for obj in gc.get_objects()); net_created = after - before`.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "What does `id(obj)` return and why is it useful for tracking objects across snapshots?",
      options: [
        "Returns object's hash value for set operations",
        "Returns unique object memory address for identity tracking",
        "Returns object type name for categorization",
        "Returns reference count for the object"
      ],
      correctOption: 1,
      explanation: "`id(obj)` returns the object's unique memory address (identity). Two different objects never have the same ID during their simultaneous lifetime. This enables tracking: if ID 12345 exists in snapshot_before and snapshot_after, it's the same object. IDs are hashable integers, perfect for storing in sets for fast lookups. Option 2 confuses `id()` with `hash()`—they're different. Option 3 describes `type(obj).__name__`. Option 4 describes `sys.getrefcount()`. Usage: `before_ids = {id(obj) for obj in gc.get_objects()}; after_ids = {id(obj) for obj in gc.get_objects()}; surviving = before_ids & after_ids` finds objects that existed in both snapshots.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "When building a memory profiler, why use sets for object ID tracking instead of lists?",
      options: [
        "Sets preserve chronological order better",
        "Sets automatically prevent garbage collection",
        "Lists can't store integer IDs efficiently",
        "O(1) membership checks vs O(n) for lists"
      ],
      correctOption: 3,
      explanation: "Sets provide O(1) membership checking: `if obj_id in seen_ids` is instant regardless of set size. Lists require O(n) scanning through all elements. When tracking millions of objects, this performance difference is critical. Option 2 is wrong—sets don't affect GC behavior. Option 3 is wrong—lists store IDs fine; performance is the issue. Option 4 is backwards—lists preserve order, but profiling doesn't need order; it needs fast lookups. Typical profiler pattern: maintaining a `seen_ids` set and checking `if id(obj) not in seen_ids: # new object discovered` thousands of times per second. With sets, this is efficient; with lists, it's prohibitively slow.",
      source: "Lesson 6: Memory Profiler Capstone"
    },
    {
      question: "You notice your memory profiler itself is consuming significant memory. What's the likely cause?",
      options: [
        "Storing references instead of IDs keeps objects alive",
        "Using sets instead of lists wastes memory",
        "gc.get_objects() prevents garbage collection globally",
        "Frozensets automatically duplicate all tracked objects"
      ],
      correctOption: 0,
      explanation: "If your profiler stores actual object references (even in snapshots), those references prevent garbage collection of those objects—your profiler keeps them alive. Store IDs instead: `id(obj)` is just an integer, not a reference. IDs don't prevent GC. Option 2 is wrong—sets may use more memory than lists, but not significantly enough to be the primary issue. Option 3 is wrong—`gc.get_objects()` doesn't prevent GC. Option 4 is wrong—frozensets store references/IDs, not duplicate objects. Fix: change `snapshot = [obj for obj in gc.get_objects()]` to `snapshot = {id(obj) for obj in gc.get_objects()}`. The former keeps all objects alive; the latter just stores numbers.",
      source: "Lesson 6: Memory Profiler Capstone"
    }
  ]}
/>
