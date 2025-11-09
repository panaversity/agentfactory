"""
TutorGPT Agent Tools - Autonomous Capabilities

These are the tools the TutorGPT agent CAN use. The agent (LLM) autonomously
decides WHEN and HOW to use them based on student needs.

This is TRUE agentic behavior - tools define CAPABILITIES, the LLM decides STRATEGY.
"""

from agents import function_tool
from typing import Literal, Optional, List


@function_tool
def search_book_content(
    query: str,
    scope: Literal["lesson", "chapter", "book"] = "lesson",
    current_chapter: Optional[str] = None,
    current_lesson: Optional[str] = None
) -> str:
    """
    Search the AI-Native Software Development book for relevant content.

    Use this tool when a student asks about a topic covered in the book.
    The agent should call this tool FIRST before explaining, to ensure
    answers are grounded in the book content.

    Args:
        query: The search query (concept, topic, or student's question)
        scope: Where to search - "lesson" (most specific), "chapter" (broader), or "book" (entire book)
        current_chapter: Current chapter student is reading (e.g., "04-python")
        current_lesson: Current lesson student is on (e.g., "01-intro")

    Returns:
        Relevant book content with chapter/lesson citations

    Examples:
        - search_book_content("What is Python?", scope="lesson")
        - search_book_content("async programming", scope="chapter")
        - search_book_content("variables", scope="book")
    """
    # TODO: Implement RAG search (will connect to ChromaDB + Gemini embeddings)
    # For now, return mock response for testing

    return f"""
    [Book Content - Chapter {current_chapter or 'Unknown'}, Lesson {current_lesson or 'Unknown'}]

    Found relevant content for query: "{query}"

    Python is a high-level, interpreted programming language known for its readability
    and versatility. It's widely used in AI development, web development, and automation.

    Key features:
    - Dynamic typing
    - Extensive standard library
    - Strong community support

    [Source: AI-Native Software Development Book, Chapter 4: Python Fundamentals]
    """


@function_tool
def explain_concept(
    concept: str,
    depth: Literal["simple", "detailed", "advanced"] = "detailed",
    use_analogy: bool = False,
    student_level: Literal["beginner", "intermediate", "advanced"] = "beginner"
) -> str:
    """
    Provide an explanation of a programming concept with adaptive depth.

    The agent should use this tool AFTER searching the book to provide
    a clear, student-appropriate explanation.

    Args:
        concept: The concept to explain (e.g., "variables", "async programming")
        depth: Level of detail - "simple" (beginner-friendly), "detailed" (standard), "advanced" (technical)
        use_analogy: Whether to include analogies/metaphors for easier understanding
        student_level: Student's proficiency level to tailor explanation

    Returns:
        Clear explanation appropriate for the depth level and student

    Examples:
        - explain_concept("variables", depth="simple", use_analogy=True)
        - explain_concept("recursion", depth="advanced", student_level="advanced")
    """
    # TODO: Implement adaptive explanation logic
    # For now, return mock response for testing

    analogy = ""
    if use_analogy:
        analogy = "\n\n💡 Think of it like: A variable is like a labeled box where you can store things. You give the box a name (variable name) and put something inside (value)."

    return f"""
    Explaining: {concept} (Depth: {depth}, Level: {student_level})

    A {concept} is a fundamental programming concept that allows you to store and manipulate data.
    {analogy}

    Example:
    ```python
    name = "Alice"  # 'name' is a variable storing "Alice"
    age = 25        # 'age' is a variable storing 25
    ```
    """


@function_tool
def provide_code_example(
    concept: str,
    context: Optional[str] = None,
    difficulty: Literal["beginner", "intermediate", "advanced"] = "beginner"
) -> str:
    """
    Provide a practical code example demonstrating a concept.

    Use this tool when a student asks "how" questions or needs to see
    concrete implementation.

    Args:
        concept: The concept to demonstrate (e.g., "async/await", "list comprehension")
        context: Additional context about what specific aspect to demonstrate
        difficulty: Complexity level of the example

    Returns:
        Well-commented code example with explanation

    Examples:
        - provide_code_example("async/await", context="basic usage")
        - provide_code_example("decorators", difficulty="intermediate")
    """
    # TODO: Implement code example generation
    # For now, return mock response for testing

    return f"""
    Here's a {difficulty}-level code example for: {concept}

    ```python
    # Example: {concept}
    # Context: {context or 'Basic usage'}

    def example_function():
        '''Demonstrates {concept}'''
        result = "This is a {difficulty} example"
        return result

    # Usage
    output = example_function()
    print(output)  # Output: This is a {difficulty} example
    ```

    💡 This example shows how {concept} works in a practical scenario.
    """


@function_tool
def detect_student_confusion(
    current_topic: str,
    question_count: int = 1
) -> str:
    """
    Analyze student behavior to detect confusion patterns.

    The agent should call this tool if a student asks multiple questions
    about the same topic or shows signs of struggle.

    Args:
        current_topic: Current topic being discussed
        question_count: Number of questions asked about this topic

    Returns:
        Analysis of confusion with recommendations

    Examples:
        - detect_student_confusion("variables", question_count=3)
    """
    # TODO: Implement confusion detection algorithm
    # For now, return mock response for testing

    is_confused = question_count >= 3

    if is_confused:
        return f"""
        CONFUSION DETECTED for topic: '{current_topic}'

        Analysis: Student has asked {question_count} questions about this topic.

        Recommendations:
        - Use simpler language
        - Include analogies and metaphors
        - Provide concrete examples
        - Break down into smaller concepts
        - Check understanding frequently

        The student is showing signs of struggling with this concept. Adjust teaching approach accordingly.
        """
    else:
        return f"""
        NO CONFUSION DETECTED for topic: '{current_topic}'

        Analysis: Student appears to be following along well ({question_count} question(s) asked).

        Recommendation: Continue with current teaching approach.
        """


@function_tool
def celebrate_milestone(
    milestone_type: Literal["chapter_complete", "lesson_complete", "concept_mastered", "first_code"],
    achievement: str
) -> str:
    """
    Celebrate a student's achievement or milestone.

    The agent should call this tool when detecting that a student has
    completed a significant learning milestone.

    Args:
        milestone_type: Type of milestone achieved
        achievement: Description of what was accomplished

    Returns:
        Encouraging celebration message

    Examples:
        - celebrate_milestone("chapter_complete", "Completed Chapter 4: Python Fundamentals")
        - celebrate_milestone("concept_mastered", "Understanding of async programming")
    """
    # TODO: Implement personalized celebration logic
    # For now, return mock response for testing

    celebrations = {
        "chapter_complete": "🎉 Fantastic work completing this chapter!",
        "lesson_complete": "✅ Great job finishing this lesson!",
        "concept_mastered": "💡 You've mastered this concept!",
        "first_code": "👏 Your first code example - well done!"
    }

    return f"""
    {celebrations.get(milestone_type, "🎉 Great achievement!")}

    {achievement}

    You're making excellent progress! Keep up the great work! 🚀

    Ready to continue learning?
    """


@function_tool
def suggest_next_lesson(
    completed_lesson: str
) -> str:
    """
    Suggest the next appropriate lesson based on student progress.

    The agent should call this tool after a student completes a lesson
    or chapter to guide their learning journey.

    Args:
        completed_lesson: Lesson just completed (e.g., "04-python/01-intro")

    Returns:
        Suggestion for next lesson with brief description

    Examples:
        - suggest_next_lesson("04-python/01-intro")
    """
    # TODO: Implement intelligent lesson suggestion based on progress
    # For now, return mock response for testing

    return f"""
    Based on completing: {completed_lesson}

    📚 Next Recommended Lesson:
    → Chapter 4, Lesson 2: Variables and Data Types

    This builds directly on what you just learned about Python basics!

    Would you like to continue?
    """


# Export all tools for the agent
TUTORGPT_TOOLS = [
    search_book_content,
    explain_concept,
    provide_code_example,
    detect_student_confusion,
    celebrate_milestone,
    suggest_next_lesson,
]
