# Structured Data Formats (CSV and JSON) Summary

### Core Concept

CSV (tabular, rows/columns) and JSON (hierarchical, nested) are complementary formats. Use CSV for spreadsheet-like data and Excel integration; use JSON for nested structures, APIs, and configuration files. Encoding (UTF-8) matters for international characters.

### Key Mental Models

**Format Matching Intent**: CSV represents flat tables (employee list, sales records). JSON represents relationships and hierarchy (customer with multiple orders, configuration with sections). Choose format before you code.

**Serialization Bridge**: Python objects → JSON/CSV (serialization) and JSON/CSV → Python objects (deserialization). Type conversion happens at boundaries; understand what Python types map to format types.

### Critical Patterns

- `csv.DictReader(f)` → read with column names (clearer than index access)
- `csv.DictWriter(f, fieldnames=[...])` → write with headers safely
- `newline=''` in `open()` → prevent extra blank lines in CSV
- `json.load(f)` → deserialize JSON file to Python dict/list
- `json.dump(data, f, indent=2, ensure_ascii=False)` → serialize with pretty-printing and Unicode
- `encoding='utf-8'` → always explicitly set for international text (emoji, accents)

### AI Collaboration Keys

Specify format rationale: "I have employee data with IDs, names, departments. Should I use CSV or JSON?" AI explains tradeoffs; you decide based on use case and who reads the data.

### Common Mistakes

- Mixing JSON syntax errors (trailing commas, quotes) cause `JSONDecodeError`
- Forgetting `ensure_ascii=False` corrupts emoji/international characters
- Using `csv.reader()` without headers (index numbers instead of names)
- Not catching format-specific exceptions (`JSONDecodeError`, `ValueError`)

### Connections

Builds on Lesson 2 (reading/writing files). Requires Lesson 3 (pathlib for finding data files). Combined with Lesson 1 in Lesson 5 capstone (notes stored as JSON, searched interactively).
