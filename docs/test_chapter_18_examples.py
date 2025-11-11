#!/usr/bin/env python3
"""
Test code examples from Chapter 18: Lists, Tuples, and Dictionary
Validates that all examples run successfully on Python 3.14+
"""

def test_lesson_1_collections_intro():
    """Lesson 1: Introduction to Collections - EX-001"""
    print("=== Lesson 1: Collections Introduction ===")

    # Lists (mutable sequences)
    shopping_list: list[str] = ["milk", "eggs", "bread", "butter"]
    print(f"Shopping list: {shopping_list}")

    # Tuples (immutable sequences)
    coordinates: tuple[int, int] = (10, 20)
    print(f"Coordinates: {coordinates}")

    # Dicts (key-value mappings)
    student: dict[str, str | int] = {
        "name": "Alice",
        "major": "Computer Science",
        "gpa": 3.8
    }
    print(f"Student: {student}")
    print("✓ Lesson 1 examples passed\n")


def test_lesson_2_lists_basics():
    """Lesson 2: Lists Part 1 - EX-002, EX-003"""
    print("=== Lesson 2: Lists Basics ===")

    # Indexing and slicing
    fruits: list[str] = ["apple", "banana", "cherry", "date"]
    print(f"First fruit: {fruits[0]}")  # apple
    print(f"Last fruit: {fruits[-1]}")  # date
    print(f"First two: {fruits[0:2]}")  # ['apple', 'banana']

    # Type hints
    numbers: list[int] = [1, 2, 3, 4, 5]
    print(f"Numbers: {numbers}")
    print(f"Length: {len(numbers)}")

    print("✓ Lesson 2 examples passed\n")


def test_lesson_3_lists_mutation():
    """Lesson 3: Lists Part 2 - EX-005"""
    print("=== Lesson 3: Lists Mutation ===")

    cart: list[str] = ["apple", "banana"]
    cart.append("orange")  # Returns None
    print(f"After append: {cart}")

    cart.extend(["grape", "mango"])
    print(f"After extend: {cart}")

    removed = cart.pop()  # Returns removed item
    print(f"Popped: {removed}, Cart now: {cart}")

    cart.remove("banana")  # Removes first occurrence
    print(f"After remove: {cart}")

    print("✓ Lesson 3 examples passed\n")


def test_lesson_4_lists_advanced():
    """Lesson 4: Lists Part 3 - EX-007, EX-008"""
    print("=== Lesson 4: Lists Advanced ===")

    # sort() vs sorted()
    numbers: list[int] = [5, 2, 8, 1, 9]
    numbers.sort()  # In-place, returns None
    print(f"After sort(): {numbers}")

    original: list[int] = [5, 2, 8, 1, 9]
    sorted_copy: list[int] = sorted(original)  # Returns new list
    print(f"sorted() result: {sorted_copy}")
    print(f"Original unchanged: {original}")

    # reverse() vs [::-1]
    letters: list[str] = ["a", "b", "c"]
    letters.reverse()  # In-place
    print(f"After reverse(): {letters}")

    original2: list[str] = ["a", "b", "c"]
    reversed_copy: list[str] = original2[::-1]  # Returns new list
    print(f"[::-1] result: {reversed_copy}")

    print("✓ Lesson 4 examples passed\n")


def test_lesson_5_comprehensions():
    """Lesson 5: List Comprehensions - EX-010"""
    print("=== Lesson 5: List Comprehensions ===")

    # Basic comprehension
    squares: list[int] = [x**2 for x in range(1, 6)]
    print(f"Squares: {squares}")  # [1, 4, 9, 16, 25]

    # With filtering
    even_squares: list[int] = [x**2 for x in range(1, 11) if x % 2 == 0]
    print(f"Even squares: {even_squares}")  # [4, 16, 36, 64, 100]

    # String transformation
    names: list[str] = ["alice", "bob", "charlie"]
    capitalized: list[str] = [name.capitalize() for name in names]
    print(f"Capitalized: {capitalized}")

    print("✓ Lesson 5 examples passed\n")


def test_lesson_6_tuples():
    """Lesson 6: Tuples - EX-011"""
    print("=== Lesson 6: Tuples ===")

    # Basic tuple
    point: tuple[int, int] = (10, 20)
    print(f"Point: {point}")

    # Single-element tuple (requires trailing comma)
    single: tuple[int] = (1,)
    print(f"Single-element tuple: {single}, type: {type(single)}")

    # Unpacking
    x, y = point
    print(f"Unpacked: x={x}, y={y}")

    # Tuples as dict keys
    game_map: dict[tuple[int, int], str] = {
        (0, 0): "Starting Point",
        (10, 20): "Forest Village",
        (30, 40): "Mountain Peak"
    }
    location: str = game_map[(10, 20)]
    print(f"Location at (10, 20): {location}")

    # Tuple methods
    numbers: tuple[int, ...] = (1, 2, 3, 2, 4, 2, 5)
    count_twos: int = numbers.count(2)
    first_three: int = numbers.index(3)
    print(f"Count of 2s: {count_twos}, Index of first 3: {first_three}")

    print("✓ Lesson 6 examples passed\n")


def test_lesson_7_dicts_basics():
    """Lesson 7: Dicts Part 1 - EX-012"""
    print("=== Lesson 7: Dicts Basics ===")

    # Basic dict with union types
    student: dict[str, str | int] = {
        "name": "Alice",
        "major": "Computer Science",
        "age": 20,
        "gpa": 3.8
    }
    print(f"Student name: {student['name']}")

    # .get() with default
    notes: str = student.get("notes", "No notes")
    print(f"Notes: {notes}")

    # Check key existence
    if "email" in student:
        print(f"Email: {student['email']}")
    else:
        print("Email not found")

    print("✓ Lesson 7 examples passed\n")


def test_lesson_8_dicts_crud():
    """Lesson 8: Dicts Part 2 - EX-013"""
    print("=== Lesson 8: Dicts CRUD ===")

    inventory: dict[str, int] = {
        "apples": 50,
        "bananas": 30,
        "oranges": 20
    }

    # Create/Update
    inventory["grapes"] = 40
    inventory["apples"] = 60  # Update
    print(f"After update: {inventory}")

    # Delete with pop (safe)
    removed_qty: int = inventory.pop("bananas", 0)
    print(f"Removed bananas: {removed_qty}")

    # Iterate
    print("Inventory items:")
    for item, qty in inventory.items():
        print(f"  {item}: {qty}")

    print("✓ Lesson 8 examples passed\n")


def test_lesson_9_dict_comprehensions():
    """Lesson 9: Dict Comprehensions - EX-014"""
    print("=== Lesson 9: Dict Comprehensions ===")

    # Temperature conversion
    celsius: dict[str, int] = {"New York": 20, "Los Angeles": 25, "Chicago": 15}
    fahrenheit: dict[str, float] = {
        city: (temp * 9/5) + 32
        for city, temp in celsius.items()
    }
    print(f"Fahrenheit temps: {fahrenheit}")

    # Filtering
    warm_cities: dict[str, float] = {
        city: temp
        for city, temp in fahrenheit.items()
        if temp > 70
    }
    print(f"Warm cities (>70°F): {warm_cities}")

    # Word frequency
    words: list[str] = ["python", "is", "great", "python", "is", "powerful"]
    word_counts: dict[str, int] = {}
    for word in words:
        word_counts[word] = word_counts.get(word, 0) + 1
    print(f"Word counts: {word_counts}")

    print("✓ Lesson 9 examples passed\n")


def test_lesson_10_choosing_structure():
    """Lesson 10: Choosing Structure - Decision Matrix"""
    print("=== Lesson 10: Choosing Structure ===")

    # Shopping cart (mutable, order matters)
    cart: list[str] = ["apple", "banana", "orange"]
    cart.append("grape")
    print(f"Shopping cart: {cart}")

    # Coordinates (immutable, fixed structure)
    position: tuple[float, float] = (40.7128, -74.0060)
    lat, lon = position
    print(f"Position: lat={lat}, lon={lon}")

    # User profile (named access, key-value)
    profile: dict[str, str | int] = {
        "username": "alice",
        "email": "alice@example.com",
        "age": 25
    }
    print(f"User: {profile['username']}, Age: {profile['age']}")

    print("✓ Lesson 10 examples passed\n")


def test_lesson_11_capstone_pipeline():
    """Lesson 11: Capstone - Data Processing Pipeline - EX-015"""
    print("=== Lesson 11: Capstone Pipeline ===")

    # Simplified pipeline (without full parsing function)
    raw_data = """name,major,gpa
Alice,CS,3.8
Bob,Math,3.2
Carol,CS,3.9
David,Physics,3.5"""

    # Parse into list[dict]
    lines: list[str] = raw_data.strip().split('\n')
    headers: list[str] = lines[0].split(',')

    students: list[dict[str, str | float]] = []
    for line in lines[1:]:
        values: list[str] = line.split(',')
        record: dict[str, str | float] = {}
        for i, header in enumerate(headers):
            value = values[i]
            # Convert GPA to float
            if header == 'gpa':
                record[header] = float(value)
            else:
                record[header] = value
        students.append(record)

    print(f"Parsed {len(students)} students")

    # Filter with comprehension (CS majors with GPA > 3.5)
    filtered: list[dict[str, str | float]] = [
        student for student in students
        if student['major'] == 'CS' and student['gpa'] > 3.5
    ]
    print(f"Filtered to {len(filtered)} CS students with GPA > 3.5")

    # Aggregate statistics by major
    stats: dict[str, dict[str, int | float]] = {}
    for student in students:
        major: str = str(student['major'])
        if major not in stats:
            stats[major] = {'count': 0, 'total_gpa': 0.0}
        stats[major]['count'] += 1
        stats[major]['total_gpa'] += float(student['gpa'])

    # Calculate averages
    print("\n=== Statistics by Major ===")
    for major, data in stats.items():
        count: int = int(data['count'])
        total: float = float(data['total_gpa'])
        avg: float = total / count
        print(f"{major}: {count} students, avg GPA {avg:.2f}")

    print("\n✓ Lesson 11 capstone pipeline passed\n")


def main():
    """Run all test examples"""
    print("=" * 60)
    print("Chapter 18 Code Examples Validation")
    print("Testing all examples on Python 3.14+")
    print("=" * 60)
    print()

    # Get Python version
    import sys
    version = sys.version_info
    print(f"Python Version: {version.major}.{version.minor}.{version.micro}")
    print()

    # Run all tests
    test_lesson_1_collections_intro()
    test_lesson_2_lists_basics()
    test_lesson_3_lists_mutation()
    test_lesson_4_lists_advanced()
    test_lesson_5_comprehensions()
    test_lesson_6_tuples()
    test_lesson_7_dicts_basics()
    test_lesson_8_dicts_crud()
    test_lesson_9_dict_comprehensions()
    test_lesson_10_choosing_structure()
    test_lesson_11_capstone_pipeline()

    print("=" * 60)
    print("✅ ALL EXAMPLES PASSED - Chapter 18 is validated for Python 3.14+")
    print("=" * 60)


if __name__ == "__main__":
    main()
