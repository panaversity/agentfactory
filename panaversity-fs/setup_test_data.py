#!/usr/bin/env python3
"""
Setup script to create test data in any storage backend.

Usage:
    # Local filesystem
    export PANAVERSITY_STORAGE_BACKEND=fs
    python setup_test_data.py

    # Cloudflare R2
    export PANAVERSITY_STORAGE_BACKEND=s3
    export PANAVERSITY_S3_BUCKET=panaversity-books
    # ... other S3 credentials
    python setup_test_data.py

    # Supabase
    export PANAVERSITY_STORAGE_BACKEND=supabase
    export PANAVERSITY_SUPABASE_URL=https://xxx.supabase.co
    # ... other Supabase credentials
    python setup_test_data.py
"""

import asyncio
import sys
import os

sys.path.insert(0, 'src')

from panaversity_fs.storage import get_operator
from panaversity_fs.config import get_config


async def create_test_data():
    """Create comprehensive test data structure."""
    print("🚀 PanaversityFS Test Data Setup")
    print("=" * 60)

    # Get configuration
    config = get_config()
    print(f"\n📊 Configuration:")
    print(f"   Backend: {config.storage_backend}")

    if config.storage_backend == "fs":
        print(f"   Root: {config.storage_root}")
    elif config.storage_backend == "s3":
        print(f"   Bucket: {config.s3_bucket}")
        print(f"   Endpoint: {config.s3_endpoint}")
    elif config.storage_backend == "supabase":
        print(f"   URL: {config.supabase_url}")
        print(f"   Bucket: {config.supabase_bucket}")

    # Get operator
    op = get_operator()

    print("\n📝 Creating test data...\n")

    # 1. Create registry.yaml
    print("1. Creating registry.yaml...")
    registry_content = """books:
  - book_id: ai-native-python
    title: AI-Native Python Development
    storage_backend: {backend}
    created_at: "2025-01-01T00:00:00Z"
    status: active

  - book_id: generative-ai-fundamentals
    title: Generative AI Fundamentals
    storage_backend: {backend}
    created_at: "2025-02-01T00:00:00Z"
    status: active
""".format(backend=config.storage_backend)

    await op.write("registry.yaml", registry_content.encode('utf-8'))
    print("   ✅ registry.yaml created")

    # 2. Create lessons
    print("\n2. Creating sample lessons...")

    lessons = [
        {
            "path": "books/ai-native-python/lessons/part-1/chapter-01/lesson-01.md",
            "content": """---
title: Introduction to Python
chapter: 1
lesson: 1
proficiency: A2
---

# Lesson 1: Introduction to Python

## Learning Objectives

- Understand Python basics
- Write your first Python program
- Learn about variables and data types

## Introduction

Python is a high-level programming language that emphasizes code readability.
It uses OpenDAL for storage operations and is perfect for AI-native development.

## Your First Python Program

```python
def hello_world():
    print("Hello, World!")

# Call the function
hello_world()
```

## Variables and Data Types

Python supports various data types:

```python
# String
name = "Alice"

# Integer
age = 25

# Float
height = 5.8

# Boolean
is_student = True
```

## Practice Exercise

Write a Python function that:
1. Takes a name as input
2. Returns a greeting message

## Summary

In this lesson, you learned the fundamentals of Python programming including:
- Basic syntax
- Functions
- Variables and data types
"""
        },
        {
            "path": "books/ai-native-python/lessons/part-1/chapter-01/lesson-02.md",
            "content": """---
title: Python Control Flow
chapter: 1
lesson: 2
proficiency: A2
---

# Lesson 2: Python Control Flow

## Learning Objectives

- Understand if/else statements
- Learn about loops (for, while)
- Use OpenDAL for file operations

## Conditional Statements

```python
age = 18

if age >= 18:
    print("You are an adult")
else:
    print("You are a minor")
```

## Loops

### For Loop
```python
for i in range(5):
    print(f"Count: {i}")
```

### While Loop
```python
count = 0
while count < 5:
    print(f"Count: {count}")
    count += 1
```

## Working with OpenDAL

```python
import opendal

# Initialize operator
op = opendal.Operator("fs", root="/tmp/data")

# Write file
op.write("hello.txt", b"Hello, OpenDAL!")

# Read file
content = op.read("hello.txt")
print(content.decode())
```

## Summary

You learned about control flow in Python and basic OpenDAL usage.
"""
        }
    ]

    for lesson in lessons:
        await op.write(lesson["path"], lesson["content"].encode('utf-8'))
        print(f"   ✅ {lesson['path']}")

    # 3. Create chapter summaries
    print("\n3. Creating chapter summaries...")

    summaries = [
        {
            "path": "books/ai-native-python/chapters/chapter-01/.summary.md",
            "content": """# Chapter 1 Summary

## Key Concepts

- Introduction to Python programming
- Basic syntax and structure
- Variables and data types
- Control flow (if/else, loops)
- OpenDAL storage operations

## What Students Learned

Students learned how to write their first Python programs, work with variables,
use control flow statements, and integrate OpenDAL for storage operations.

## Prerequisites for Next Chapter

Before moving to Chapter 2, students should be comfortable with:
- Writing basic Python functions
- Using variables and data types
- Understanding if/else statements
- Working with for/while loops
"""
        }
    ]

    for summary in summaries:
        await op.write(summary["path"], summary["content"].encode('utf-8'))
        print(f"   ✅ {summary['path']}")

    # 4. Create sample assets (text-based for compatibility)
    print("\n4. Creating sample assets...")

    assets = [
        {
            "path": "books/ai-native-python/assets/images/README.md",
            "content": "# Image Assets\n\nThis directory contains course images."
        },
        {
            "path": "books/ai-native-python/assets/slides/README.md",
            "content": "# Slide Assets\n\nThis directory contains course slides."
        }
    ]

    for asset in assets:
        await op.write(asset["path"], asset["content"].encode('utf-8'))
        print(f"   ✅ {asset['path']}")

    # 5. Create book metadata
    print("\n5. Creating book metadata...")

    book_yaml = """title: AI-Native Python Development
author: Panaversity Team
version: 1.0.0
storage_backend: {backend}
created_at: "2025-01-01T00:00:00Z"

content_structure:
  parts:
    - part_id: part-1
      title: Python Fundamentals
      chapters:
        - chapter-01
"""

    await op.write(
        "books/ai-native-python/book.yaml",
        book_yaml.format(backend=config.storage_backend).encode('utf-8')
    )
    print("   ✅ books/ai-native-python/book.yaml")

    print("\n" + "=" * 60)
    print("✅ Test data creation complete!")
    print("=" * 60)

    print("\n📊 Created:")
    print("   - 1 registry file")
    print("   - 2 books (ai-native-python, generative-ai-fundamentals)")
    print("   - 2 lessons")
    print("   - 1 chapter summary")
    print("   - 2 asset directories")
    print("   - 1 book metadata file")

    print("\n🧪 Next Steps:")
    print("   1. Start server: uv run python -m panaversity_fs.server")
    print("   2. Run tests: uv run python test_all_tools.py")
    print("   3. Use MCP Inspector: npx @modelcontextprotocol/inspector http://localhost:8000/mcp")

    return True


async def verify_data():
    """Verify created data."""
    print("\n🔍 Verifying data...")

    op = get_operator()

    # Check registry
    try:
        registry = await op.read("registry.yaml")
        print("   ✅ registry.yaml exists")
    except:
        print("   ❌ registry.yaml not found")
        return False

    # Check sample lesson
    try:
        lesson = await op.read("books/ai-native-python/lessons/part-1/chapter-01/lesson-01.md")
        print(f"   ✅ lesson-01.md exists ({len(lesson)} bytes)")
    except:
        print("   ❌ lesson-01.md not found")
        return False

    print("\n✅ Verification complete!")
    return True


async def main():
    """Main entry point."""
    try:
        # Create data
        success = await create_test_data()

        if success:
            # Verify data
            await verify_data()

            print("\n" + "=" * 60)
            print("🎉 Setup complete! Your storage backend is ready for testing.")
            print("=" * 60)

    except Exception as e:
        print(f"\n❌ ERROR: {type(e).__name__}: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
