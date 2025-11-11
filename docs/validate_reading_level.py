#!/usr/bin/env python3
"""
Reading Level Validation for Chapter 18
Target: Grade 7-8 (Flesch-Kincaid Grade Level)
"""

import re
from pathlib import Path


def extract_prose_from_markdown(content: str) -> str:
    """Extract prose text from markdown, removing code blocks and frontmatter."""
    # Remove YAML frontmatter
    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

    # Remove code blocks
    content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)

    # Remove inline code
    content = re.sub(r'`[^`]+`', '', content)

    # Remove headings markers but keep text
    content = re.sub(r'^#+\s*', '', content, flags=re.MULTILINE)

    # Remove markdown links but keep text
    content = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', content)

    # Remove emoji markers
    content = re.sub(r'[💬🎓🚀✨]', '', content)

    # Remove bullet points
    content = re.sub(r'^\s*[-*]\s+', '', content, flags=re.MULTILINE)

    # Remove blockquotes
    content = re.sub(r'^\s*>\s+', '', content, flags=re.MULTILINE)

    return content.strip()


def count_syllables(word: str) -> int:
    """Count syllables in a word (simplified algorithm)."""
    word = word.lower()
    vowels = 'aeiouy'
    syllable_count = 0
    previous_was_vowel = False

    for char in word:
        is_vowel = char in vowels
        if is_vowel and not previous_was_vowel:
            syllable_count += 1
        previous_was_vowel = is_vowel

    # Adjust for silent e
    if word.endswith('e') and syllable_count > 1:
        syllable_count -= 1

    # Ensure at least one syllable
    return max(1, syllable_count)


def flesch_kincaid_grade(text: str) -> float:
    """Calculate Flesch-Kincaid Grade Level."""
    # Split into sentences (simplified)
    sentences = re.split(r'[.!?]+', text)
    sentences = [s.strip() for s in sentences if s.strip()]

    # Split into words
    words = re.findall(r'\b[a-zA-Z]+\b', text)

    if not sentences or not words:
        return 0.0

    # Count syllables
    total_syllables = sum(count_syllables(word) for word in words)

    # Calculate metrics
    avg_sentence_length = len(words) / len(sentences)
    avg_syllables_per_word = total_syllables / len(words)

    # Flesch-Kincaid Grade Level formula
    grade_level = (0.39 * avg_sentence_length) + (11.8 * avg_syllables_per_word) - 15.59

    return grade_level


def analyze_lesson(file_path: Path) -> dict:
    """Analyze reading level of a single lesson."""
    content = file_path.read_text(encoding='utf-8')
    prose = extract_prose_from_markdown(content)

    # Split into sentences and words
    sentences = re.split(r'[.!?]+', prose)
    sentences = [s.strip() for s in sentences if s.strip()]
    words = re.findall(r'\b[a-zA-Z]+\b', prose)

    # Calculate reading level
    grade_level = flesch_kincaid_grade(prose)

    return {
        'file': file_path.name,
        'sentences': len(sentences),
        'words': len(words),
        'grade_level': grade_level,
        'target_range': (7.0, 8.5),
        'passes': 6.5 <= grade_level <= 9.0  # Allow slight margin
    }


def main():
    """Validate reading level for all Chapter 18 lessons."""
    print("=" * 70)
    print("Chapter 18 Reading Level Validation (Flesch-Kincaid Grade Level)")
    print("Target: Grade 7-8 | Acceptable Range: 6.5-9.0")
    print("=" * 70)
    print()

    chapter_dir = Path("book-source/docs/04-Part-4-Python-Fundamentals/18-lists-tuples-dictionary")

    if not chapter_dir.exists():
        print(f"❌ Chapter directory not found: {chapter_dir}")
        return

    # Find all lesson files
    lesson_files = sorted(chapter_dir.glob("*-lesson-*.md"))

    if not lesson_files:
        print(f"❌ No lesson files found in {chapter_dir}")
        return

    results = []
    for lesson_file in lesson_files:
        result = analyze_lesson(lesson_file)
        results.append(result)

        status = "✅ PASS" if result['passes'] else "⚠️  REVIEW"
        print(f"{status} | {result['file']:20} | Grade: {result['grade_level']:4.1f} | "
              f"Words: {result['words']:5} | Sentences: {result['sentences']:4}")

    print()
    print("=" * 70)

    # Summary
    passed = sum(1 for r in results if r['passes'])
    total = len(results)
    avg_grade = sum(r['grade_level'] for r in results) / total

    print(f"Summary: {passed}/{total} lessons in target range")
    print(f"Average Grade Level: {avg_grade:.1f}")

    if passed == total:
        print("✅ ALL LESSONS PASS - Reading level appropriate for Grade 7-8 audience")
    else:
        print("⚠️  REVIEW NEEDED - Some lessons outside target range")
        print("\nRecommendations for out-of-range lessons:")
        print("- Too high (>9.0): Shorten sentences, use simpler vocabulary")
        print("- Too low (<6.5): Verify sufficient educational depth")

    print("=" * 70)

    # Detailed breakdown for outliers
    outliers = [r for r in results if not r['passes']]
    if outliers:
        print("\nDetailed Analysis for Out-of-Range Lessons:")
        print("-" * 70)
        for result in outliers:
            print(f"{result['file']}:")
            print(f"  Grade Level: {result['grade_level']:.1f} (target: 7.0-8.5)")
            if result['grade_level'] > 9.0:
                print(f"  Issue: Too complex for Grade 7-8 audience")
                print(f"  Suggestion: Simplify sentence structure and vocabulary")
            elif result['grade_level'] < 6.5:
                print(f"  Issue: May be too simple for intermediate learners")
                print(f"  Suggestion: Verify educational depth is maintained")
            print()


if __name__ == "__main__":
    main()
