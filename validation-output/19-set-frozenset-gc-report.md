# Python Code Validation Report - 19-set-frozenset-gc

**Generated:** 2025-11-12 12:12:45
**Python Version:** 3.14.0
**Container:** python-sandbox-validator

## Executive Summary

**Total Code Blocks:** 50
**Syntax Errors:** 0
**Runtime Errors:** 6
**Successful:** 44
**Success Rate:** 88.0%

❌ **VALIDATION FAILURES DETECTED**

- 0 syntax error(s) require immediate fix
- 6 runtime error(s) need review

---

## Results by File

### 01-set-basics.md
- **Total Blocks:** 8
- **Errors:** 1
- **Status:** ❌ FAIL (1 error(s))

**Error Details:**

- `01-set-basics.md:313` (block 7)
  - **RUNTIME**: TypeError: cannot use 'list' as a set element (unhashable type: 'list')

### 02-set-operations.md
- **Total Blocks:** 6
- **Errors:** 0
- **Status:** ✅ PASS

### 03-set-internals-hashing.md
- **Total Blocks:** 11
- **Errors:** 2
- **Status:** ❌ FAIL (2 error(s))

**Error Details:**

- `03-set-internals-hashing.md:187` (block 2)
  - **TIMEOUT**: Execution exceeded 5 second timeout

- `03-set-internals-hashing.md:417` (block 9)
  - **TIMEOUT**: Execution exceeded 5 second timeout

### 04-frozensets.md
- **Total Blocks:** 11
- **Errors:** 0
- **Status:** ✅ PASS

### 05-garbage-collection.md
- **Total Blocks:** 11
- **Errors:** 1
- **Status:** ❌ FAIL (1 error(s))

**Error Details:**

- `05-garbage-collection.md:146` (block 1)
  - **RUNTIME**: NameError: name 'sys' is not defined. Did you forget to import 'sys'?

### 06-memory-profiler-capstone.md
- **Total Blocks:** 3
- **Errors:** 2
- **Status:** ❌ FAIL (2 error(s))

**Error Details:**

- `06-memory-profiler-capstone.md:364` (block 1)
  - **RUNTIME**: NameError: name 'MemoryProfiler' is not defined

- `06-memory-profiler-capstone.md:482` (block 2)
  - **RUNTIME**: NameError: name 'MemoryProfiler' is not defined


---

## Recommended Actions

### Medium Priority (Runtime Errors)
Review these—syntax is valid but execution fails:

- **03-set-internals-hashing.md:187** — Review runtime behavior
- **03-set-internals-hashing.md:417** — Review runtime behavior
- **06-memory-profiler-capstone.md:364** — Review runtime behavior
- **06-memory-profiler-capstone.md:482** — Review runtime behavior
- **05-garbage-collection.md:146** — Review runtime behavior
- **01-set-basics.md:313** — Review runtime behavior

