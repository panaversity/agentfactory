---
title: "Lesson 4: Structured Data Formats (CSV and JSON)"
chapter: 22
lesson: 4
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Reading CSV Files with csv.reader and csv.DictReader"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can parse CSV files using both csv.reader (tuples) and csv.DictReader (dicts), handle headers correctly, and iterate through rows"

  - name: "Writing CSV Files with csv.writer and csv.DictWriter"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write Python data structures to CSV files with proper headers and formatting using csv.DictWriter"

  - name: "Loading JSON Files with json.load()"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can load JSON from files with proper encoding, handle the resulting Python objects, and understand deserialization"

  - name: "Saving Data to JSON with json.dump()"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can serialize Python objects to JSON with proper encoding, indentation, and ensure_ascii handling"

  - name: "Understanding Character Encoding (UTF-8)"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain why UTF-8 encoding matters for international text and when to use ensure_ascii=False"

  - name: "Handling Format-Specific Exceptions"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can catch JSONDecodeError, ValueError (malformed CSV), and provide helpful error messages"

  - name: "Choosing Between CSV and JSON"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain when CSV is appropriate (tabular data, Excel) vs JSON (nested data, APIs) and justify format selection"

learning_objectives:
  - objective: "Read and parse CSV files using both csv.reader and csv.DictReader with proper header handling"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Write code that reads CSV and displays structured data"

  - objective: "Write CSV files with proper headers using csv.DictWriter"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Write code that exports Python data structures to CSV"

  - objective: "Load and save JSON data with correct encoding (UTF-8, ensure_ascii=False)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Write code that persists and retrieves JSON data with international characters"

  - objective: "Handle parsing errors gracefully for both CSV and JSON formats"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Write error handling code that catches and responds to format errors"

  - objective: "Analyze data structure requirements and choose appropriate format (CSV vs JSON)"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Justify format selection for given use cases"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (CSV format, csv.reader, csv.DictReader, JSON format, json.load/dump, encoding, format selection) matches B1 limit ✓"

differentiation:
  extension_for_advanced: "Explore CSV dialects (tab-delimited, pipe-separated), custom JSON encoders for complex objects, streaming large JSON files"
  remedial_for_struggling: "Focus on csv.DictReader and json.dump/load first; delay format selection comparison until concepts solidify"

# Generation metadata
generated_by: "lesson-writer v4.5.0"
source_spec: "specs/001-part-4-chapter-22/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 4: Structured Data Formats (CSV and JSON)

Real-world applications rarely work with data in isolation. You need to import employee lists from Excel, export analysis results to share with colleagues, integrate with APIs that return JSON, and store application data for later retrieval. This lesson teaches you how to work with two of the most common structured data formats: CSV (Comma-Separated Values) for tabular data and JSON (JavaScript Object Notation) for nested, hierarchical data.

By the end of this lesson, you'll understand when to use each format, how to read and write both with Python's standard library modules, and how to handle encoding correctly so international characters (emoji, accented letters, non-Latin scripts) are preserved.

---

## Why Structured Data Formats Matter

Before diving into code, let's understand the problem these formats solve. When you have data—a list of contacts, inventory records, or application configuration—you need to:

1. **Store it persistently** — Save data to disk so it survives when your program closes
2. **Share it with others** — Export to Excel, send to API, or hand off to colleague
3. **Integrate with systems** — Read data that other applications created (database exports, web APIs, configuration files)
4. **Structure it meaningfully** — Preserve relationships between data (customer has multiple orders, organization has departments)

Structured formats give you a standard way to solve all these problems. CSV and JSON are ubiquitous because they're human-readable, language-independent, and handle international characters correctly.

**Key decision**: Use **CSV for tabular data** (rows and columns, like spreadsheets) and **JSON for hierarchical data** (nested objects, mixed types, API responses).

---

## Understanding CSV: Simple, Tabular Data

CSV (Comma-Separated Values) represents data as rows and columns, just like a spreadsheet. Here's what a CSV file looks like:

```
name,age,department
Alice,28,Engineering
Bob,35,Sales
Carol,32,Marketing
```

**Structure**: Each row is one record. Each column is a field. Commas separate values. The first row (usually) contains headers—field names.

**Strengths**:
- Simple, human-readable format
- Native support in Excel, Google Sheets, databases
- Compact (no extra formatting syntax)
- Perfect for tabular data (employees, contacts, inventory)

**Limitations**:
- Only handles flat data (rows and columns)
- Can't represent nested structures (customer with multiple orders)
- Requires conventions for special cases (commas in values, newlines in cells)

**When to use CSV**: Sales data, employee directories, scientific measurements, any rectangular dataset

---

## Understanding JSON: Flexible, Hierarchical Data

JSON (JavaScript Object Notation) represents data as nested objects and arrays, with support for strings, numbers, booleans, and null. Here's a JSON example:

```json
{
  "id": "1",
  "title": "Python Basics",
  "tags": ["learning", "python"],
  "metadata": {
    "created": "2025-11-09",
    "updated": "2025-11-09"
  }
}
```

**Structure**: Data is organized as key-value pairs (objects) and ordered lists (arrays). Values can be strings, numbers, booleans, null, objects, or arrays.

**Strengths**:
- Handles nested, hierarchical data naturally
- Supports multiple data types (strings, numbers, booleans, null)
- Standard format for web APIs (REST, GraphQL)
- Self-documenting (keys describe values)
- Readable with proper indentation

**Limitations**:
- More verbose than CSV (extra syntax)
- Overkill for simple tabular data
- Requires proper syntax (trailing commas cause errors)

**When to use JSON**: API responses, configuration files, application state, nested data structures, anything with hierarchy or mixed types

---

## Core Concept: The csv Module

Python's `csv` module provides two main readers:

### csv.reader — Simple tuple-based access

```python
import csv

with open('employees.csv', 'r', encoding='utf-8') as f:
    reader = csv.reader(f)
    header = next(reader)  # Get first row as list of headers
    for row in reader:
        print(row)  # row is a tuple: ('Alice', '28', 'Engineering')
        name = row[0]  # Access by index
```

**Access method**: By index (row[0], row[1], row[2])

**Advantage**: Lightweight, works with any CSV structure

**Disadvantage**: Requires memorizing column positions (error-prone)

### csv.DictReader — Dictionary-based access with headers

```python
import csv

with open('employees.csv', 'r', encoding='utf-8') as f:
    reader = csv.DictReader(f)  # Automatically reads first row as headers
    for row in reader:
        print(row)  # row is a dict: {'name': 'Alice', 'age': '28', 'department': 'Engineering'}
        name = row['name']  # Access by key (more readable!)
```

**Access method**: By column name (row['name'], row['age'])

**Advantage**: Self-documenting, column names clear, robust to column reordering

**Disadvantage**: Slightly more overhead (creates dicts for each row)

**Professional recommendation**: Use `csv.DictReader` for production code—the clarity is worth the minimal overhead.

---

## Core Concept: The json Module

Python's `json` module handles serialization (Python → JSON) and deserialization (JSON → Python):

### json.load() — Read JSON from file

```python
import json

with open('notes.json', 'r', encoding='utf-8') as f:
    data = json.load(f)  # data is now a Python object (dict or list)
```

### json.dump() — Write JSON to file

```python
import json

notes = [
    {"id": "1", "title": "Python Basics", "tags": ["learning", "python"]},
    {"id": "2", "title": "File I/O Guide", "tags": ["io", "files"]}
]

with open('notes.json', 'w', encoding='utf-8') as f:
    json.dump(notes, f, indent=2, ensure_ascii=False)
```

**Key parameters**:
- `indent=2` — Pretty-print with 2-space indentation (makes file readable)
- `ensure_ascii=False` — Preserve Unicode characters (emoji, accents, non-Latin scripts)
- `encoding='utf-8'` — Always use UTF-8 for text files

---

## Core Concept: Character Encoding (UTF-8)

**Problem**: Without proper encoding, international characters become corrupted:

```python
# WRONG — omits encoding
with open('messages.json', 'w') as f:
    json.dump({"message": "Hello 👋"}, f)  # Emoji might be escaped or corrupted

# RIGHT — explicit UTF-8
with open('messages.json', 'w', encoding='utf-8') as f:
    json.dump({"message": "Hello 👋"}, f, ensure_ascii=False)
```

**UTF-8**: Universal, variable-width encoding that handles every human language plus emoji. It's the **standard for web and modern applications**.

**ensure_ascii=False**: By default, JSON escapes non-ASCII characters as `\uXXXX` (ugly and hard to read). Setting `ensure_ascii=False` keeps emoji and international characters visible in the file.

**Best practice**: Always use `encoding='utf-8'` and `ensure_ascii=False` for JSON files that contain international text.

#### 💬 AI Colearning Prompt

> "Show me the difference between JSON with `ensure_ascii=True` (default) and `ensure_ascii=False` when saving emoji. Which is more readable and why?"

---

## Reading CSV Files: Pattern and Practice

Here's the professional pattern for reading CSV files:

**Code Example 4.1: Reading CSV Files**

```python
import csv
from pathlib import Path

# Create sample CSV file
csv_file: Path = Path("employees.csv")
csv_file.write_text("""name,age,department
Alice,28,Engineering
Bob,35,Sales
Carol,32,Marketing
""")

# ========== Pattern 1: csv.reader (tuples) ==========
print("Using csv.reader (by index):")
with open(csv_file, 'r', encoding='utf-8') as f:
    reader = csv.reader(f)
    header = next(reader)  # Manually get first row
    print(f"Columns: {header}")
    for row in reader:
        # Access by index: row[0] = name, row[1] = age, row[2] = department
        print(f"  {row[0]} ({row[2]})")

# ========== Pattern 2: csv.DictReader (dictionaries) ==========
print("\nUsing csv.DictReader (by name):")
with open(csv_file, 'r', encoding='utf-8') as f:
    reader = csv.DictReader(f)  # Automatically reads headers
    for row in reader:
        # Access by column name: row['name'], row['age'], row['department']
        print(f"  {row['name']} - {row['department']}")
```

**Specification Reference**: This example demonstrates FR-019 (reading CSV with csv.reader and csv.DictReader)

**AI Prompts Used**:
1. "Show me how to read CSV files with both csv.reader and csv.DictReader"
2. "Explain the difference between accessing by index vs by column name"

**Validation Steps**:
1. Created sample CSV with headers and 3 data rows
2. Read with csv.reader and printed rows as tuples
3. Read with csv.DictReader and accessed columns by name
4. Verified that DictReader automatically handled headers

**Expected Outcome**: You see csv.DictReader is clearer—you use column names instead of remembering positions.

#### 🎓 Instructor Commentary

> In AI-native development, you don't memorize CSV module parameters—you understand your DATA STRUCTURE. Need column access by name? csv.DictReader. Dealing with unheadered data? csv.reader. Your AI generates the boilerplate; your job is specifying your intent.

---

## Writing CSV Files: Preserving Structure

Writing CSV requires careful handling of headers and row formatting:

**Code Example 4.2: Writing CSV Files**

```python
import csv
from pathlib import Path

# Data to write (list of dictionaries)
employees: list[dict[str, str | int]] = [
    {"name": "Alice", "age": 28, "department": "Engineering"},
    {"name": "Bob", "age": 35, "department": "Sales"},
    {"name": "Carol", "age": 32, "department": "Marketing"},
]

csv_file: Path = Path("output.csv")

# ========== Writing with csv.DictWriter ==========
with open(csv_file, 'w', newline='', encoding='utf-8') as f:
    fieldnames = ["name", "age", "department"]  # Define column order
    writer = csv.DictWriter(f, fieldnames=fieldnames)

    writer.writeheader()  # Write header row
    writer.writerows(employees)  # Write data rows

# ========== Verify by reading back ==========
print("Written CSV content:")
with open(csv_file, 'r', encoding='utf-8') as f:
    print(f.read())

# Output:
# name,age,department
# Alice,28,Engineering
# Bob,35,Sales
# Carol,32,Marketing
```

**Key detail**: `newline=''` in open() — This prevents extra blank lines in CSV output (platform-specific newline handling).

**Specification Reference**: This example demonstrates FR-020 (writing CSV with proper headers)

**Validation Steps**:
1. Created list of dictionaries with employee data
2. Opened file in write mode with newline=''
3. Used csv.DictWriter to write headers and rows
4. Read file back to verify output format
5. Confirmed headers and data rows are correctly formatted

**Expected Outcome**: You understand csv.DictWriter handles header order and formatting automatically.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "I have employee data with fields: id, name, email, hire_date. Write code using csv.DictWriter to export this to a CSV file. Then explain why we use DictWriter instead of manually writing strings with commas."

**Expected Outcome**: You'll understand csv.DictWriter handles edge cases (commas in values, special characters) automatically.

---

## Reading and Writing JSON: Serialization/Deserialization

JSON is Python's preferred format for storing application data:

**Code Example 4.3: Reading and Writing JSON**

```python
import json
from pathlib import Path

# ========== Create Python data structure ==========
notes: list[dict[str, str | list[str]]] = [
    {"id": "1", "title": "Python Basics", "tags": ["learning", "python"]},
    {"id": "2", "title": "File I/O Guide", "tags": ["io", "files", "python"]},
]

json_file: Path = Path("notes.json")

# ========== Write to JSON (serialization) ==========
with open(json_file, 'w', encoding='utf-8') as f:
    json.dump(notes, f, indent=2, ensure_ascii=False)

print(f"Written {json_file}")

# ========== Display raw file content ==========
print("\nFile content (human-readable):")
print(json_file.read_text(encoding='utf-8'))

# Output:
# [
#   {
#     "id": "1",
#     "title": "Python Basics",
#     "tags": [
#       "learning",
#       "python"
#     ]
#   },
#   {
#     "id": "2",
#     "title": "File I/O Guide",
#     "tags": [
#       "io",
#       "files",
#       "python"
#     ]
#   }
# ]

# ========== Read from JSON (deserialization) ==========
with open(json_file, 'r', encoding='utf-8') as f:
    loaded_notes: list[dict] = json.load(f)

print(f"\nLoaded {len(loaded_notes)} notes:")
for note in loaded_notes:
    print(f"  - {note['title']} (tags: {', '.join(note['tags'])})")

# Output:
# Loaded 2 notes:
#   - Python Basics (tags: learning, python)
#   - File I/O Guide (tags: io, files, python)
```

**Key points**:
- `json.dump()` converts Python objects → JSON (serialization)
- `json.load()` converts JSON → Python objects (deserialization)
- `indent=2` makes output human-readable (not required, but professional)
- Nested structures (dicts, lists) are preserved perfectly

**Specification Reference**: This example demonstrates FR-021 and FR-022 (JSON load and dump with encoding)

**Validation Steps**:
1. Created list of dicts with nested structure
2. Used json.dump() with indent=2 for readability
3. Displayed raw file content to show pretty-printing
4. Used json.load() to read back into Python
5. Verified structure is preserved exactly

**Expected Outcome**: You see JSON preserves nested structures perfectly—Python dicts become JSON objects, lists become arrays.

---

## Handling Encoding for International Text

One of the biggest real-world problems: **international characters get corrupted**. Here's how to prevent it:

**Code Example 4.4: Handling Encoding for International Text**

```python
import json
from pathlib import Path

# ========== Data with international characters ==========
messages: list[dict[str, str]] = [
    {"user": "Alice", "text": "Hello! 👋"},
    {"user": "Bob", "text": "Привет (Russian for hello)"},
    {"user": "Carol", "text": "你好 (Chinese for hello)"},
]

json_file: Path = Path("messages.json")

# ========== WRONG: omits ensure_ascii=False ==========
print("WITHOUT ensure_ascii=False (default):")
with open(json_file, 'w', encoding='utf-8') as f:
    json.dump(messages, f, indent=2)  # ensure_ascii=True by default

# Looks ugly: emoji and accents become escaped
content = json_file.read_text(encoding='utf-8')
print(content)

# Output shows: \u1f44b (escaped emoji) and \u043f (escaped Cyrillic)

# ========== CORRECT: use ensure_ascii=False ==========
print("\n\nWITH ensure_ascii=False:")
with open(json_file, 'w', encoding='utf-8') as f:
    json.dump(messages, f, indent=2, ensure_ascii=False)

# Looks great: emoji and international characters visible
content = json_file.read_text(encoding='utf-8')
print(content)

# Output shows readable emoji 👋 and text "Привет", "你好"

# ========== Read back and verify ==========
print("\n\nLoaded messages:")
with open(json_file, 'r', encoding='utf-8') as f:
    loaded = json.load(f)
    for msg in loaded:
        print(f"  {msg['user']}: {msg['text']}")

# Output:
#   Alice: Hello! 👋
#   Bob: Привет (Russian for hello)
#   Carol: 你好 (Chinese for hello)
```

**The lesson**: Always use both:
1. `encoding='utf-8'` in open()
2. `ensure_ascii=False` in json.dump()

Without both, emoji and international characters become either corrupted or escaped.

**Specification Reference**: This example demonstrates FR-024 (understanding encoding and ensure_ascii)

**Validation Steps**:
1. Created data with emoji and international scripts
2. Demonstrated problem: escaped characters without ensure_ascii=False
3. Showed solution: both encoding='utf-8' and ensure_ascii=False
4. Read back and verified characters preserved
5. Confirmed file is both readable (in text editor) and parseable (by json.load())

**Expected Outcome**: You understand why UTF-8 + ensure_ascii=False matter for international applications.

#### ✨ Teaching Tip

> Use Claude Code to explore encoding issues: "What happens if I save JSON with emoji but use `encoding='latin-1'` or omit `ensure_ascii=False`? Show me the broken output and how to fix it."

---

## Error Handling for Format Errors

Real-world files are often corrupted, incomplete, or malformed. Here's professional error handling:

**Code Example 4.5: Error Handling for Format Errors**

```python
import json
import csv
from pathlib import Path

# ========== Safe JSON loading ==========
def safe_load_json(filename: str) -> list[dict] | None:
    """Load JSON with comprehensive error handling."""
    try:
        path = Path(filename)
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
        return None
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON format - {e.msg} at line {e.lineno}")
        return None
    except Exception as e:
        print(f"Unexpected error: {e}")
        return None

# ========== Safe CSV loading ==========
def safe_load_csv(filename: str) -> list[dict[str, str]] | None:
    """Load CSV with error handling."""
    try:
        path = Path(filename)
        with open(path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None:
                print("Error: CSV file is empty or has no headers")
                return None
            return list(reader)
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
        return None
    except ValueError as e:
        print(f"Error: Invalid CSV format - {e}")
        return None
    except Exception as e:
        print(f"Unexpected error: {e}")
        return None

# ========== Test with valid files ==========
valid_json = Path("valid.json")
valid_json.write_text('[{"name": "Alice"}, {"name": "Bob"}]')
data = safe_load_json("valid.json")
print(f"Loaded JSON: {data}")

valid_csv = Path("valid.csv")
valid_csv.write_text("name,age\nAlice,28\nBob,35")
data = safe_load_csv("valid.csv")
print(f"Loaded CSV: {data}")

# ========== Test with invalid files ==========
invalid_json = Path("invalid.json")
invalid_json.write_text('[{"name": "Alice"} BROKEN')  # Missing closing bracket
print("\nAttempting to load invalid JSON:")
data = safe_load_json("invalid.json")
# Output: Error: Invalid JSON format - Expecting ',' delimiter at line 1

# Missing file
print("\nAttempting to load missing file:")
data = safe_load_json("nonexistent.json")
# Output: Error: File 'nonexistent.json' not found
```

**What this teaches**:
- **JSONDecodeError** includes line and position information (helps debug)
- **FileNotFoundError** for missing files
- Return None on error (allows graceful handling)
- Check for empty CSV (no headers = invalid)

**Specification Reference**: This example demonstrates FR-023 (handling JSONDecodeError and ValueError)

**Validation Steps**:
1. Wrote safe_load_json() with specific exception handling
2. Wrote safe_load_csv() with header validation
3. Tested both with valid files
4. Tested with invalid JSON (syntax error)
5. Tested with missing file
6. Verified error messages are helpful, not crashes

**Expected Outcome**: You see professional error handling prevents crashes and provides actionable error messages.

### Python 3.14 Improved Error Messages

**Python 3.14 enhanced JSON error reporting** with exception notes that identify exactly where errors occur:

```python
# Python 3.14+ provides richer error context
try:
    data = json.loads('{"name": "Alice", "age": }')  # Missing value
except json.JSONDecodeError as e:
    print(f"Error: {e}")
    # Python 3.14+ adds a note showing the exact issue:
    # "Expecting value: line 1 column 28 (char 27)"
    # The note highlights the missing value location
```

**What improved**: In Python 3.13 and earlier, you got basic line/column numbers. In Python 3.14+, exception notes provide **contextual hints** about what's wrong (missing value, trailing comma, unclosed string, etc.).

**Why this matters for AI-native development**: When JSON parsing fails, your AI companion gets better error messages to diagnose the issue. This means faster debugging—your AI can pinpoint "missing closing brace on line 12" instead of generic "invalid JSON" errors.

**For this chapter**: The error handling patterns we teach work in all Python versions. When you encounter JSON errors, Python 3.14+ will just give you more helpful messages automatically.

---

## Choosing Between CSV and JSON: Decision Framework

When should you use each format? Here's the decision tree:

| Question | Answer | Use CSV | Use JSON |
|----------|--------|---------|----------|
| **Is data tabular?** (rows and columns) | Yes | ✓ CSV | |
| **Does data have nested structure?** (objects within objects) | Yes | | ✓ JSON |
| **Will it open in Excel?** | Required | ✓ CSV | |
| **Is it an API response?** | Yes | | ✓ JSON |
| **Configuration file?** | Yes | | ✓ JSON |
| **Only strings and numbers?** | No (has booleans, nulls) | ✓ CSV | |

**Real-world examples**:

- **CSV**: Employee directory, sales data, scientific measurements, database exports, anything from Excel
- **JSON**: API responses, application configuration, note storage (Lesson 5), any hierarchical data, web application state

**Professional guideline**: When in doubt, use JSON. It's more flexible, handles edge cases better, and integrates naturally with web APIs.

---

## Practice Exercises

### Exercise 1: Read and Display CSV

Write a program that:
1. Reads a CSV file with at least 3 columns and 5 rows
2. Displays each row in a formatted table with column headers
3. Uses csv.DictReader for clean column access

**Validation**: Test with a file containing special characters (commas in values, quotes)

**Success**: All rows display correctly, headers shown

### Exercise 2: Convert CSV to JSON

Write a program that:
1. Reads employee data from a CSV file
2. Converts to a Python list of dictionaries
3. Saves to JSON with `indent=2` and `ensure_ascii=False`
4. Reads back and verifies structure matches

**Validation**: Test with data containing international characters (names with accents, emoji in descriptions)

**Success**: CSV and JSON contain same data, international characters preserved

### Exercise 3: Error Handling for Both Formats

Write error-handling code that:
1. Attempts to load both CSV and JSON files
2. Catches FileNotFoundError, JSONDecodeError, and ValueError
3. Provides helpful error messages (not just crashes)
4. Returns None on failure so calling code can handle gracefully

**Validation**: Test with:
- Valid CSV and JSON files
- Corrupted JSON (missing bracket)
- Missing file
- Empty file

**Success**: No crashes, appropriate error messages for each case

---

## Try With AI

These prompts help you synthesize everything this lesson taught: reading/writing both formats, handling encoding, managing errors, and choosing appropriate formats.

### Prompt 1: Understand Format Differences

```
Ask your AI: "What's the difference between CSV and JSON?
When would you use each format for real-world applications?
Show me examples."

Expected Outcome: You understand when tabular (CSV) is appropriate
vs hierarchical/nested (JSON) data, and can justify format selection.
```

### Prompt 2: Apply Format Conversion

```
Ask your AI: "Write code that reads a CSV file containing contacts
(name, email, phone), filters for a specific email domain, and saves
the results to JSON with proper UTF-8 encoding."

Expected Outcome: You see CSV→JSON transformation with filtering,
and understand how csv.DictReader and json.dump work together.
```

### Prompt 3: Analyze Performance Tradeoffs

```
Ask your AI: "Compare reading a large CSV file (10,000 rows) with
csv.reader vs csv.DictReader. What's the memory and performance
difference? When would you use each?"

Expected Outcome: You understand tradeoffs between convenience
(DictReader) and efficiency (reader), and when each is appropriate.
```

### Prompt 4: Synthesize Real-World Scenario

```
Ask your AI: "Design a data import system that reads CSV from a
user's export, validates required fields, handles encoding issues
(international names), converts to JSON, saves to file, and
implements comprehensive error handling. What error cases must
you handle? What does the data structure look like?"

Expected Outcome: You've integrated CSV/JSON with validation and
error handling in a realistic workflow, demonstrating mastery of
structured data formats. This prepares you for Lesson 5's Note-Taking
App capstone that persists data as JSON files.
```

**Safety & Ethics Note**: When importing data from external sources, always validate and sanitize inputs. Don't assume CSV/JSON files are trustworthy—implement bounds checking, type validation, and error handling. This protects your application and data integrity.
