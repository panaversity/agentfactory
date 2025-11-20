#!/usr/bin/env python3
"""
Fix quiz explanations to use generic references instead of "Option N" after answer redistribution.
This ensures explanations remain accurate regardless of option order.
"""

import re

def fix_explanation(text):
    """
    Replace "Option N" references with generic descriptions.
    Pattern: "Option N is wrong/incorrect" → "The option about X is wrong/incorrect"
    """
    # This is a simple pass - we'll manually review critical explanations
    # For now, just flag them
    option_refs = re.findall(r'Option \d+', text)
    if option_refs:
        print(f"Found option references: {option_refs}")
    return text

def main():
    quiz_file = "book-source/docs/04-Python-Fundamentals/16-operators-keywords-variables/06_chapter_16_quiz.md"

    with open(quiz_file, 'r') as f:
        content = f.read()

    # Find all explanations with "Option" references
    explanations = re.findall(r'explanation: "(.*?)"', content, re.DOTALL)

    print(f"Total explanations: {len(explanations)}")

    option_count = 0
    for i, exp in enumerate(explanations):
        if 'Option' in exp and re.search(r'Option \d+', exp):
            option_count += 1
            print(f"\nQuestion {i+1} has Option references")

    print(f"\nTotal explanations with 'Option N' references: {option_count}")

if __name__ == "__main__":
    main()
