# Python Code Validation Report - 13-introduction-to-python

**Generated:** 2025-11-12 12:28:19
**Python Version:** 3.14.0
**Container:** python-sandbox-validator

## Executive Summary

**Total Code Blocks:** 45
**Syntax Errors:** 6
**Runtime Errors:** 13
**Successful:** 26
**Success Rate:** 57.8%

❌ **VALIDATION FAILURES DETECTED**

- 6 syntax error(s) require immediate fix
- 13 runtime error(s) need review

---

## Results by File

### 01-what-is-python.md
- **Total Blocks:** 2
- **Errors:** 0
- **Status:** ✅ PASS

### 03-variables-and-type-hints.md
- **Total Blocks:** 20
- **Errors:** 5
- **Status:** ❌ FAIL (5 error(s))

**Error Details:**

- `03-variables-and-type-hints.md:177` (block 8)
  - **SYNTAX**: invalid decimal literal

- `03-variables-and-type-hints.md:184` (block 9)
  - **SYNTAX**: invalid syntax

- `03-variables-and-type-hints.md:190` (block 10)
  - **SYNTAX**: invalid syntax

- `03-variables-and-type-hints.md:302` (block 15)
  - **SYNTAX**: invalid syntax

- `03-variables-and-type-hints.md:339` (block 19)
  - **SYNTAX**: invalid decimal literal

### 04-basic-syntax-and-first-programs.md
- **Total Blocks:** 17
- **Errors:** 8
- **Status:** ❌ FAIL (8 error(s))

**Error Details:**

- `04-basic-syntax-and-first-programs.md:109` (block 2)
  - **RUNTIME**: NameError: name 'current_position' is not defined

- `04-basic-syntax-and-first-programs.md:115` (block 3)
  - **RUNTIME**: NameError: name 'current_position' is not defined

- `04-basic-syntax-and-first-programs.md:149` (block 6)
  - **RUNTIME**: NameError: name 'name' is not defined

- `04-basic-syntax-and-first-programs.md:174` (block 8)
  - **RUNTIME**: NameError: name 'name' is not defined

- `04-basic-syntax-and-first-programs.md:179` (block 9)
  - **RUNTIME**: NameError: name 'name' is not defined

- `04-basic-syntax-and-first-programs.md:313` (block 14)
  - **RUNTIME**: NameError: name 'Hello' is not defined

- `04-basic-syntax-and-first-programs.md:322` (block 15)
  - **SYNTAX**: cannot assign to function call here. Maybe you meant '==' instead of '='?

- `04-basic-syntax-and-first-programs.md:332` (block 16)
  - **RUNTIME**: NameError: name 'name' is not defined

### 05-capstone-project.md
- **Total Blocks:** 6
- **Errors:** 6
- **Status:** ❌ FAIL (6 error(s))

**Error Details:**

- `05-capstone-project.md:149` (block 0)
  - **RUNTIME**: EOFError: EOF when reading a line

- `05-capstone-project.md:166` (block 1)
  - **RUNTIME**: NameError: name 'name' is not defined

- `05-capstone-project.md:187` (block 2)
  - **RUNTIME**: EOFError: EOF when reading a line

- `05-capstone-project.md:304` (block 3)
  - **RUNTIME**: EOFError: EOF when reading a line

- `05-capstone-project.md:313` (block 4)
  - **RUNTIME**: NameError: name 'name' is not defined

- `05-capstone-project.md:322` (block 5)
  - **RUNTIME**: EOFError: EOF when reading a line


---

## Recommended Actions

### High Priority (Syntax Errors)
Fix these immediately—code will not run:

- **03-variables-and-type-hints.md:177** — invalid decimal literal
- **03-variables-and-type-hints.md:184** — invalid syntax
- **03-variables-and-type-hints.md:190** — invalid syntax
- **03-variables-and-type-hints.md:302** — invalid syntax
- **03-variables-and-type-hints.md:339** — invalid decimal literal
- **04-basic-syntax-and-first-programs.md:322** — cannot assign to function call here. Maybe you meant '==' instead of '='?

### Medium Priority (Runtime Errors)
Review these—syntax is valid but execution fails:

- **05-capstone-project.md:149** — Review runtime behavior
- **05-capstone-project.md:166** — Review runtime behavior
- **05-capstone-project.md:187** — Review runtime behavior
- **05-capstone-project.md:304** — Review runtime behavior
- **05-capstone-project.md:313** — Review runtime behavior
- **05-capstone-project.md:322** — Review runtime behavior
- **04-basic-syntax-and-first-programs.md:109** — Review runtime behavior
- **04-basic-syntax-and-first-programs.md:115** — Review runtime behavior
- **04-basic-syntax-and-first-programs.md:149** — Review runtime behavior
- **04-basic-syntax-and-first-programs.md:174** — Review runtime behavior
- **04-basic-syntax-and-first-programs.md:179** — Review runtime behavior
- **04-basic-syntax-and-first-programs.md:313** — Review runtime behavior
- **04-basic-syntax-and-first-programs.md:332** — Review runtime behavior

