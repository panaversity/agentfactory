$json_content = Get-Content -Path "C:\Users\SS COMP\Desktop\Python\ai-native-software-development_featured\book-source\src\quiz-data-chapters-after-42.json" | Out-String
$quiz_data_original = ConvertFrom-Json $json_content

$filtered_chapters = @()
foreach ($chapter in $quiz_data_original.quiz_data.chapters) {
    if ($chapter.chapter_id -ge 77 -and $chapter.chapter_id -le 111) {
        # Ensure questions array is empty for reset
        $chapter.questions = @()
        $filtered_chapters += $chapter
    }
}

$new_quiz_data_reset = @{
    quiz_data = @{
        chapters = $filtered_chapters
    }
}

$reset_json_content = $new_quiz_data_reset | ConvertTo-Json -Depth 100

Set-Content -Path "C:\Users\SS COMP\Desktop\Python\ai-native-software-development_featured\book-source\src\quiz-data-chater-77-111.json" -Value $reset_json_content

# Now, add the MCQs for Chapter 77, 78, 79, 81 and mark 80 as skipped

$quiz_data = ConvertFrom-Json (Get-Content -Path "C:\Users\SS COMP\Desktop\Python\ai-native-software-development_featured\book-source\src\quiz-data-chater-77-111.json" | Out-String)

$mcqs_77_raw = @'
[
    {
        "question": "What is the primary purpose of Context Compression?",
        "options": {
            "a": "To make your code files smaller on your hard drive.",
            "b": "To summarize a long conversation into a concise checkpoint, freeing up space in the context window to continue working.",
            "c": "To delete your conversation history permanently.",
            "d": "To encrypt your conversation for security."
        },
        "correct_answer": "b",
        "explanation": "The chapter defines it as \"the process of summarizing your session's progress, saving it as a checkpoint, and starting a fresh session with just the essential summary.\""
    },
    {
        "question": "What is Context Isolation?",
        "options": {
            "a": "Working with the AI without an internet connection.",
            "b": "Using a single, massive AI session for all of your tasks for the day.",
            "c": "Using separate, focused AI sessions or environments for different, unrelated tasks to prevent context from mixing.",
            "d": "Preventing the AI from accessing any of your local files."
        },
        "correct_answer": "c",
        "explanation": "Context Isolation is defined as \"the practice of using separate 'context environments' for different tasks, so information doesn't mix or interfere.\""
    },
    {
        "question": "Which of the following is a recommended trigger for performing context compression?",
        "options": {
            "a": "As soon as you start a new session.",
            "b": "After every single message you send.",
            "c": "After about 90 minutes of work or 10-15 interactions, or when you notice the AI slowing down.",
            "d": "You should never compress context."
        },
        "correct_answer": "c",
        "explanation": "The 'When to Compress' section lists these triggers: 'Every 10-15 interactions,' 'After completing a major milestone,' and 'When you notice context degradation signs.'"
    },
    {
        "question": "If you need to fix a critical bug in your authentication system while also designing a new UI for the marketing page, which strategy is most appropriate?",
        "options": {
            "a": "Context Compression",
            "b": "Context Isolation",
            "c": "Progressive Loading",
            "d": "All-at-Once Loading"
        },
        "correct_answer": "b",
        "explanation": "These are two unrelated tasks. The chapter advises using Context Isolation (e.g., separate terminal sessions) to 'Keep contexts focused and separate' and 'Prevents Cross-Contamination.'"
    },
    {
        "question": "What is the main purpose of creating and maintaining memory files like DECISIONS.md and PATTERNS.md?",
        "options": {
            "a": "They are required by the AI model to function.",
            "b": "To create extra work for the developer.",
            "c": "To document key architectural decisions and code conventions in a way that persists across sessions and can be loaded as context.",
            "d": "To share your project on social media."
        },
        "correct_answer": "c",
        "explanation": "These files serve as a form of 'Agentic Memory' that prevents the AI from 'forgetting' key decisions and patterns between sessions."
    }
]
'@

$mcqs_78_raw = @'
[
    {
        "question": "What is \"Context Curation\"?",
        "options": {
            "a": "Allowing the AI to read every file in the project.",
            "b": "Explicitly controlling which specific files the AI reads to keep the context focused and relevant.",
            "c": "Deleting files from your project to reduce context.",
            "d": "Writing all your code in a single file."
        },
        "correct_answer": "b",
        "explanation": "The chapter defines Context Curation as when \"You explicitly control which files AI reads,\" contrasting it with the bad practice of asking the AI to \"Read all files.\""
    },
    {
        "question": "What is the principle of \"Example-Driven Context\"?",
        "options": {
            "a": "Describing a coding pattern to the AI using only words.",
            "b": "Providing the AI with concrete code examples of a pattern instead of just describing it, to ensure it generates new code in the same style.",
            "c": "Asking the AI to find examples on the internet.",
            "d": "Giving the AI a link to a tutorial."
        },
        "correct_answer": "b",
        "explanation": "The chapter states the principle is to \"Show, Don't Tell,\" and provides examples of giving the AI code snippets to teach it a pattern."
    },
    {
        "question": "A \"Multi-Agent Architecture\" involves what approach to solving a complex problem?",
        "options": {
            "a": "Using one single, very large AI session for all parts of the problem.",
            "b": "Using separate, specialized AI agents (or sessions) with isolated contexts for different concerns like architecture, implementation, and testing.",
            "c": "Asking multiple different AI models the same question and comparing the answers.",
            "d": "Having multiple human developers work with a single AI agent."
        },
        "correct_answer": "b",
        "explanation": "The chapter describes using different agents for different roles: an \"Architecture Agent,\" an \"Implementation Agent,\" and a \"Testing Agent,\" each with a focused context."
    },
    {
        "question": "What is \"Just-In-Time Context Fetching\"?",
        "options": {
            "a": "Loading every possible file into the context at the very beginning of the session.",
            "b": "Letting the AI tell you what context it needs as it works, and then providing only that specific information.",
            "c": "A strategy where context is only loaded at the end of the project.",
            "d": "A feature that automatically deletes context every five minutes."
        },
        "correct_answer": "b",
        "explanation": "The chapter defines this as letting the \"AI tell you what context it needs, then provide only that,\" contrasting it with loading everything upfront."
    },
    {
        "question": "If you are starting a new feature and want to ensure it follows the project's established coding patterns, which advanced strategy is most directly applicable?",
        "options": {
            "a": "Just-In-Time Fetching",
            "b": "Multi-Agent Architecture",
            "c": "Example-Driven Context",
            "d": "Audio & Speech"
        },
        "correct_answer": "c",
        "explanation": "Example-Driven Context is the strategy of showing the AI concrete examples of existing patterns to ensure the new code it generates is consistent."
    }
]
'@

$mcqs_79_raw = @'
[
    {
        "question": "What is the \"hidden connection\" that this chapter reveals?",
        "options": {
            "a": "That good context is only necessary for debugging, not for writing new features.",
            "b": "That clear specifications can be written without any understanding of the project's context.",
            "c": "That rich context engineering is the essential foundation for being able to write clear, effective specifications.",
            "d": "That prompt engineering and context engineering are the same skill."
        },
        "correct_answer": "c",
        "explanation": "The introduction states the crucial insight: \"Context engineering... is the foundation for writing clear specifications.\""
    },
    {
        "question": "In the comparison of Developer A and Developer B, what was the fundamental flaw in Developer A's approach?",
        "options": {
            "a": "Developer A's prompt was too long and detailed.",
            "b": "Developer A immediately asked the AI to implement a feature without first establishing any context about the existing project.",
            "c": "Developer A used the wrong AI tool for the job.",
            "d": "Developer A's project was too simple for an AI to understand."
        },
        "correct_answer": "b",
        "explanation": "The chapter shows Developer A \"Immediately jumps to asking AI\" without providing any context, leading to generic code and hours of rework."
    },
    {
        "question": "Why can't a developer write a good specification without first understanding the context?",
        "options": {
            "a": "Because specifications are not important in AI-native development.",
            "b": "Because without understanding the existing system, patterns, and constraints, any specification they write will be vague and disconnected from the project's reality.",
            "c": "Because the AI can write the specification for them without any context.",
            "d": "Because good specifications can only be written after the code is complete."
        },
        "correct_answer": "b",
        "explanation": "The chapter argues, \"You can't specify what you don't understand,\" explaining that good specs require knowledge of the system, which comes from context."
    },
    {
        "question": "What is the \"Context-First Specification\" workflow?",
        "options": {
            "a": "A workflow where you write the code first, and then load context to write the specification.",
            "b": "A workflow where you load all relevant context to gain a deep understanding of the project before you write the specification.",
            "c": "A workflow where the AI writes the specification without any human input.",
            "d": "A workflow that skips writing a specification altogether."
        },
        "correct_answer": "b",
        "explanation": "The chapter defines this as \"the practice of loading all relevant context BEFORE writing a specification.\""
    },
    {
        "question": "What is the relationship between the quality of the developer's understanding and the quality of the AI's output?",
        "options": {
            "a": "They are unrelated; a powerful AI can produce good code regardless of the developer's understanding.",
            "b": "The AI's output quality is directly related to the developer's understanding, because if the developer's thinking is vague (due to poor context), their prompts and specs will be vague, leading to vague AI output.",
            "c": "A developer with poor understanding gets better results because the AI has more freedom.",
            "d": "Only the AI's understanding matters, not the developer's."
        },
        "correct_answer": "b",
        "explanation": "The chapter explains, \"If your thinking is vague, the AI's output will be vague too.\" Good context leads to clear thinking, which leads to clear specs and good code."
    }
]
'@

$mcqs_81_raw = @'
[
    {
        "question": "What is the most common context engineering mistake made by beginners?",
        "options": {
            "a": "Loading too little context.",
            "b": "Loading all project files into the context window at the start of a session.",
            "c": "Never restarting an AI session.",
            "d": "Using memory files to document decisions."
        },
        "correct_answer": "b",
        "explanation": "The chapter identifies \"Mistake #1: Loading All Files Upfront\" as the most common issue, explaining that it overloads the AI and degrades performance."
    },
    {
        "question": "'Context rot' refers to the degradation of AI performance in a long session. What is the recommended strategy to combat this?",
        "options": {
            "a": "Continuing to work in the same session until it completely breaks.",
            "b": "Using context compression: summarizing the session's progress into a checkpoint and starting a fresh session with the summary.",
            "c": "Speaking to the AI instead of typing to save tokens.",
            "d": "Closing the tool and taking a break for an hour before resuming."
        },
        "correct_answer": "b",
        "explanation": "Context compression is presented as the solution to long sessions, allowing you to \"transport\" your progress into a fresh, performant session."
    },
    {
        "question": "What is the primary purpose of using memory files like DECISIONS.md and PATTERNS.md?",
        "options": {
            "a": "To make the project directory more cluttered.",
            "b": "To document key decisions and code conventions so that this critical context persists across different AI sessions.",
            "c": "They serve as the main source code for the application.",
            "d": "To log every error that occurs during development."
        },
        "correct_answer": "b",
        "explanation": "The chapter explains that memory files solve the problem of the AI \"forgetting\" what was decided in previous sessions by creating a persistent record."
    },
    {
        "question": "Which of the '5-Point Context Health Check' questions helps you identify if you are loading irrelevant information?",
        "options": {
            "a": "Context Fill Level",
            "b": "Context Relevance",
            "c": "Context Recency",
            "d": "Context Clarity"
        },
        "correct_answer": "b",
        "explanation": "The 'Context Relevance' check specifically asks, 'Is everything in context relevant to current task?' to diagnose if you are loading unnecessary files."
    },
    {
        "question": "What is the key difference between the 'code-first' (old) way of thinking and the 'context-first' (new) way of thinking?",
        "options": {
            "a": "The code-first approach is faster for all projects.",
            "b": "In the context-first approach, a developer immediately starts coding. In the code-first approach, they plan first.",
            "c": "In the context-first approach, a developer's first step is to understand the project context before writing a specification or code.",
            "d": "There is no difference."
        },
        "correct_answer": "c",
        "explanation": "The chapter explicitly contrasts the two: code-first is 'I'll just start coding,' while context-first is 'First, what context do I need?'"
    }
]
'@

$mcqs_77_parsed = $mcqs_77_raw | ConvertFrom-Json
$mcqs_78_parsed = $mcqs_78_raw | ConvertFrom-Json
$mcqs_79_parsed = $mcqs_79_raw | ConvertFrom-Json
$mcqs_81_parsed = $mcqs_81_raw | ConvertFrom-Json

$chapters_to_update_map = @{
    77 = $mcqs_77_parsed
    78 = $mcqs_78_parsed
    79 = $mcqs_79_parsed
    81 = $mcqs_81_parsed
}

foreach ($chapter_id in $chapters_to_update_map.Keys) {
    $new_mcqs = $chapters_to_update_map[$chapter_id]
    
    foreach ($chapter in $quiz_data.quiz_data.chapters) {
        if ($chapter.chapter_id -eq $chapter_id) {
            if ($chapter.questions -eq $null) {
                $chapter.questions = @()
            }
            $chapter.questions += $new_mcqs
            break
        }
    }
}

# Handle Chapter 80 as skipped
foreach ($chapter in $quiz_data.quiz_data.chapters) {
    if ($chapter.chapter_id -eq 80) {
        $chapter.skipped = $true
        $chapter.questions = @() # Clear questions if skipped
        break
    }
}

$updated_json_content = $quiz_data | ConvertTo-Json -Depth 100

Set-Content -Path "C:\Users\SS COMP\Desktop\Python\ai-native-software-development_featured\book-source\src\quiz-data-chater-77-111.json" -Value $updated_json_content
