#!/usr/bin/env python3
"""
Comprehensive audit script for Chapter 16 quiz.
Checks technical accuracy, explanation correctness, and quality standards.
"""

import re
import json

def extract_questions(file_path):
    """Extract all questions from the quiz file."""
    with open(file_path, 'r') as f:
        content = f.read()

    # Extract the questions array
    match = re.search(r'questions=\{(\[.*?\])\}', content, re.DOTALL)
    if not match:
        return []

    questions_text = match.group(1)

    # Parse questions manually (JSON-like but JSX)
    questions = []
    question_blocks = re.findall(r'\{(.*?)\}(?=\s*,?\s*(?:\{|$))', questions_text, re.DOTALL)

    for block in question_blocks:
        question_match = re.search(r'question:\s*"(.*?)"', block, re.DOTALL)
        options_match = re.search(r'options:\s*\[(.*?)\]', block, re.DOTALL)
        correct_match = re.search(r'correctOption:\s*(\d+)', block)
        explanation_match = re.search(r'explanation:\s*"(.*?)"(?=,\s*source:)', block, re.DOTALL)
        source_match = re.search(r'source:\s*"(.*?)"', block)

        if all([question_match, options_match, correct_match, explanation_match, source_match]):
            options_text = options_match.group(1)
            options = re.findall(r'"(.*?)"', options_text)

            questions.append({
                'question': question_match.group(1),
                'options': options,
                'correctOption': int(correct_match.group(1)),
                'explanation': explanation_match.group(1),
                'source': source_match.group(1)
            })

    return questions

def count_words(text):
    """Count words in a string."""
    return len(text.split())

def audit_option_lengths(questions):
    """Audit option length compliance (±3 words)."""
    issues = []

    for i, q in enumerate(questions, 1):
        word_counts = [count_words(opt) for opt in q['options']]
        min_words = min(word_counts)
        max_words = max(word_counts)
        diff = max_words - min_words

        if diff > 3:
            issues.append({
                'question': i,
                'issue': f'Option length variance: {diff} words (min: {min_words}, max: {max_words})',
                'word_counts': word_counts
            })

    return issues

def audit_answer_distribution(questions):
    """Check if answers are distributed evenly across indices 0-3."""
    distribution = {0: 0, 1: 0, 2: 0, 3: 0}

    for q in questions:
        distribution[q['correctOption']] += 1

    return distribution

def audit_explanation_correctness(questions):
    """Check if explanations reference the correct answer."""
    issues = []

    for i, q in enumerate(questions, 1):
        correct_idx = q['correctOption']
        correct_text = q['options'][correct_idx]
        explanation = q['explanation']

        # Check if explanation mentions being correct
        if 'correct answer' not in explanation.lower():
            issues.append({
                'question': i,
                'issue': 'Explanation does not explicitly state "correct answer"'
            })

        # Check for "Option N" references that might be problematic
        option_refs = re.findall(r'Option (\d+)', explanation)
        if option_refs:
            # This is informational, not necessarily an error
            issues.append({
                'question': i,
                'issue': f'Uses numeric option references: {option_refs}',
                'type': 'info'
            })

    return issues

def audit_technical_accuracy(questions):
    """Check for known Python behavior correctness."""
    issues = []

    # Define correct answers for key concepts
    correct_behaviors = {
        'division always returns float': True,
        '10 // 3': 3,
        '17 % 5': 2,
        'mixing int and float': 'always float',
        '2 ** 10': 1024,
        'and operator': 'both must be True',
        'or operator': 'at least one True',
        '5 == 5.0': True,
        "'5' == 5": False,
        'keywords count': 35
    }

    for i, q in enumerate(questions, 1):
        question_text = q['question'].lower()

        # Check division operator
        if 'divide two integers using `/`' in question_text:
            correct_option_text = q['options'][q['correctOption']]
            if 'float' not in correct_option_text.lower():
                issues.append({
                    'question': i,
                    'issue': 'Division operator question: correct answer should mention float',
                    'severity': 'critical'
                })

        # Check floor division
        if '10 // 3' in question_text:
            correct_option_text = q['options'][q['correctOption']]
            if '3' not in correct_option_text and 'integer' not in correct_option_text.lower():
                issues.append({
                    'question': i,
                    'issue': 'Floor division question: correct answer should be 3 (integer)',
                    'severity': 'critical'
                })

        # Check modulus
        if '17 % 5' in question_text:
            correct_option_text = q['options'][q['correctOption']]
            if '2' not in correct_option_text or 'remainder' not in correct_option_text.lower():
                issues.append({
                    'question': i,
                    'issue': 'Modulus question: correct answer should be 2 (remainder)',
                    'severity': 'critical'
                })

    return issues

def audit_source_fields(questions):
    """Check if all questions have proper source attribution."""
    issues = []

    for i, q in enumerate(questions, 1):
        source = q.get('source', '')
        if not source:
            issues.append({
                'question': i,
                'issue': 'Missing source field',
                'severity': 'high'
            })
        elif not re.match(r'Lesson \d+:', source):
            issues.append({
                'question': i,
                'issue': f'Source format incorrect: "{source}"',
                'severity': 'medium'
            })

    return issues

def main():
    quiz_file = "book-source/docs/04-Python-Fundamentals/16-operators-keywords-variables/06_chapter_16_quiz.md"

    print("=" * 70)
    print("CHAPTER 16 QUIZ - FINAL AUDIT REPORT")
    print("=" * 70)
    print()

    questions = extract_questions(quiz_file)
    print(f"✓ Extracted {len(questions)} questions\n")

    if len(questions) != 50:
        print(f"❌ CRITICAL: Expected 50 questions, found {len(questions)}")
        return

    # 1. Option Length Audit
    print("1. OPTION LENGTH COMPLIANCE (±3 words)")
    print("-" * 70)
    length_issues = audit_option_lengths(questions)
    if length_issues:
        print(f"⚠️  Found {len(length_issues)} questions with length variance >3 words:")
        for issue in length_issues[:5]:  # Show first 5
            print(f"   Question {issue['question']}: {issue['issue']}")
            print(f"   Word counts: {issue['word_counts']}")
    else:
        print("✓ All questions pass option length requirements")
    print()

    # 2. Answer Distribution
    print("2. ANSWER DISTRIBUTION (Should be ~12-13 per index)")
    print("-" * 70)
    distribution = audit_answer_distribution(questions)
    print(f"   Index 0: {distribution[0]} questions")
    print(f"   Index 1: {distribution[1]} questions")
    print(f"   Index 2: {distribution[2]} questions")
    print(f"   Index 3: {distribution[3]} questions")

    # Check if distribution is acceptable (each index should have 10-15 questions)
    if all(10 <= count <= 15 for count in distribution.values()):
        print("✓ Answer distribution is well-balanced")
    else:
        print("⚠️  Answer distribution may be unbalanced")
    print()

    # 3. Explanation Correctness
    print("3. EXPLANATION QUALITY")
    print("-" * 70)
    explanation_issues = audit_explanation_correctness(questions)

    # Separate critical issues from info
    critical_exp = [i for i in explanation_issues if i.get('type') != 'info']
    info_exp = [i for i in explanation_issues if i.get('type') == 'info']

    if critical_exp:
        print(f"⚠️  Found {len(critical_exp)} explanation issues:")
        for issue in critical_exp[:3]:
            print(f"   Question {issue['question']}: {issue['issue']}")
    else:
        print("✓ All explanations explicitly state 'correct answer'")

    if info_exp:
        print(f"ℹ️  {len(info_exp)} explanations use numeric option references (Option N)")
    print()

    # 4. Technical Accuracy
    print("4. TECHNICAL ACCURACY")
    print("-" * 70)
    tech_issues = audit_technical_accuracy(questions)
    if tech_issues:
        print(f"❌ Found {len(tech_issues)} technical accuracy issues:")
        for issue in tech_issues:
            severity = issue.get('severity', 'unknown')
            print(f"   [{severity.upper()}] Question {issue['question']}: {issue['issue']}")
    else:
        print("✓ All spot-checked questions are technically accurate")
    print()

    # 5. Source Attribution
    print("5. SOURCE ATTRIBUTION")
    print("-" * 70)
    source_issues = audit_source_fields(questions)
    if source_issues:
        print(f"⚠️  Found {len(source_issues)} source issues:")
        for issue in source_issues[:5]:
            print(f"   Question {issue['question']}: {issue['issue']}")
    else:
        print("✓ All questions have proper source attribution")
    print()

    # Final Summary
    print("=" * 70)
    print("AUDIT SUMMARY")
    print("=" * 70)
    total_issues = len(length_issues) + len(critical_exp) + len(tech_issues) + len(source_issues)

    if total_issues == 0:
        print("✅ QUIZ PASSED ALL AUDITS")
        print("   Ready for production use")
    else:
        print(f"⚠️  QUIZ HAS {total_issues} ISSUES REQUIRING ATTENTION")
        print(f"   - Option length issues: {len(length_issues)}")
        print(f"   - Explanation issues: {len(critical_exp)}")
        print(f"   - Technical accuracy issues: {len(tech_issues)}")
        print(f"   - Source attribution issues: {len(source_issues)}")

    print(f"\nℹ️  INFO: {len(info_exp)} questions use 'Option N' references (not critical)")
    print()

if __name__ == "__main__":
    main()
