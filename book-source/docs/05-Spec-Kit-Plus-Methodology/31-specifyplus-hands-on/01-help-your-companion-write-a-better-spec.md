---
title: "Help Your Companion Write a Better Spec"
chapter: 31
lesson: 1
duration: "2 hours"
skills:
  - name: "Specification Review"
    proficiency: "B1"
    category: "Technical"
  - name: "Feedback Delivery"
    proficiency: "B1"
    category: "Soft"
  - name: "Iterative Improvement"
    proficiency: "A2"
    category: "Conceptual"
learning_objectives:
  - "Identify deficiencies and ambiguities in AI-generated specifications (B1)"
  - "Provide constructive feedback to improve specification clarity (B1)"
  - "Guide an AI companion through iterative spec refinement (B1)"
---

# Help Your Companion Write a Better Spec

## What You're About to Do

You're going to become a specification coach. Instead of learning *about* specs in a lecture, you'll **teach your companion how to write them**—and in the process, you'll discover what makes specifications actually work.

Here's how it works: Your companion has a vague idea. You ask clarifying questions. Together, you refine that vague idea into something specific. By the end, you'll have extracted six components that make specifications professional. And you'll understand *why* each component matters because you'll see what happens when they're missing.

This is the essence of specification-driven development: **clarity upstream saves chaos downstream.**

---

## Part 1: Your Companion's Vague Idea

Your AI companion is going to present a project idea. It's intentionally vague—like how most people actually think about what they want built.

**Here's the prompt to give your companion:**

```
I want you to help me build a project idea. I have a concept,
but it's not very well defined yet. Here's my initial thought:

"I want a system that helps teachers grade student work."

That's all I have so far. I know it's vague. Can you help me
think through what I'm actually asking for?
```

Paste this into your AI companion (Claude Code, ChatGPT, or Gemini CLI—whatever you use).

### What Your Companion Will Say

Your companion will probably respond with something like:

> "That's a good starting point, but I have a lot of questions. Let me ask some of them:
> - What kind of student work? Essays? Code? Math problems?
> - How do teachers input grades? Web interface? Command line? Spreadsheet?
> - What should the output be? Just a number? Feedback for students?
> - How many students are we talking about? 10? 100? 1,000?
> - Does it need to store grades over time, or is it one-off?
> - Can multiple teachers use the same system?
>
> These answers will completely change what I'd build."

Write down these questions your companion asks. Don't answer them yet—just observe that your companion naturally asked for clarity.

---

## Part 2: You Coach Your Companion to Clarity

Now you're going to do something counterintuitive: **you're going to answer your own companion's questions**, and in answering them, you'll discover what you actually need to build.

**Give your companion this prompt:**

```
Let me answer these questions, and as I do, I want you to help me
spot any contradictions or new questions that come up.

Here are my answers:

1. **Kind of work**: Students submit essays and code assignments.
2. **Input method**: Teachers upload student files and assign them to rubrics.
3. **Output**: A score (0-100) and automated feedback text.
4. **Scale**: We're starting with one teacher and 30 students.
5. **History**: Yes, we need to track grades over a semester.
6. **Multiple teachers**: Not yet, but design it so we can add this later.

As I answer these, what new questions do you have?
```

Your companion will probably respond with something like:

> "Good answers. But now I have more questions:
> - What's a 'rubric'? Does the teacher define criteria, or do we have templates?
> - Does the automated feedback use AI, or is it template-based?
> - How long can a student essay be? How long can code submissions be?
> - What happens if a teacher wants to adjust a grade after the fact?
> - What formats are you expecting? .txt files? .pdf? .py files?
> - Is security important? Should students see each other's grades?"

**This is the magic moment**: your companion isn't being annoying. It's revealing that your initial idea wasn't as clear as you thought. Each question is something that would have caused problems later if left unanswered.

---

## Part 3: Extract the Six Specification Components

As you and your companion go back and forth, notice that the conversation is naturally falling into six categories. Let's name them:

### Component 1: Overview
**What are we building?** A one-sentence summary that even a non-technical person can understand.

For the grading system: *"A system that helps one teacher assign numerical scores and automated feedback to 30 student assignments (essays and code)."*

### Component 2: Inputs
**What goes in?** What does the system receive as input?

For the grading system:
- Student essays (text files)
- Student code submissions (Python files)
- Teacher rubrics (defined by the teacher)

### Component 3: Outputs
**What comes out?** What does the system produce?

For the grading system:
- A numerical grade (0-100)
- Automated feedback text
- Grade history over the semester

### Component 4: Success Criteria
**How do we know it works?** What would good look like?

For the grading system:
- Teachers can assign grades in under 1 minute per student
- Feedback is specific to the rubric used
- Grades are stored and retrievable
- Teachers can edit grades after assignment

### Component 5: Constraints
**What are the boundaries?** What's *not* included, or what must be true?

For the grading system:
- Only one teacher (for now)
- Maximum 100 students per semester
- Essays up to 10,000 words
- Code files up to 1,000 lines
- No security or authentication required yet

### Component 6: Assumptions
**What are we taking for granted?** What knowledge or conditions are we assuming?

For the grading system:
- Teachers know how to define rubrics
- Students can upload files to a system
- Automated feedback will use AI (which means API access is available)

---

## Part 4: Create Your First Spec Rubric

Now that you've seen these six components emerge naturally from conversation, you're going to create a rubric for evaluating specifications.

**Give your companion this prompt:**

```
Based on what we just did, I want you to create a rubric
for evaluating specifications.

We just built a spec by answering six kinds of questions:
1. Overview (What?)
2. Inputs (What goes in?)
3. Outputs (What comes out?)
4. Success Criteria (How do we know it works?)
5. Constraints (What are the boundaries?)
6. Assumptions (What are we taking for granted?)

Create a rubric that says:
- What does EACH component need to be "good"?
- What does a VAGUE version look like?
- What does a CLEAR version look like?

Format it as a table so it's easy to reference.
```

Your companion will generate something like this:

| Component | Vague | Clear |
|-----------|-------|-------|
| **Overview** | "A system for grading" | "A system that assigns 0-100 scores and automated feedback to student essays and code submissions" |
| **Inputs** | "Student work and a rubric" | "Essay files (.txt, max 10k words), Python files (.py, max 1k lines), teacher-defined rubric with 3 criteria" |
| **Outputs** | "Grades and feedback" | "Numerical score (0-100), rubric-specific feedback (50-200 words), date/time of grading" |
| **Success Criteria** | "Teachers can use it easily" | "Each assignment graded in < 1 min, 95% accuracy vs. human graders, feedback updated if teacher changes grade" |
| **Constraints** | "Some limitations" | "Single teacher, ≤30 students, manual rubric definition, no authentication" |
| **Assumptions** | "Users know what they're doing" | "Teachers can define criteria clearly, students can upload files, API access available for AI feedback" |

**Save this rubric.** You'll use it to evaluate your own specs moving forward.

---

## Part 5: Reflection—What Changed?

Let's zoom out for a moment. When you started this lesson, you had a vague idea: "Help teachers grade student work."

Now you have something completely different: a clear specification that an AI companion could actually build from.

**Ask yourself these questions** (write them down):

1. **What made the difference?** What changed between the vague version and the clear version?
   - Answer: Asking questions. Answering them. Repeating. Each iteration added specificity.

2. **What would have happened if you'd skipped this step?** What if you'd just said "build a grading system" to an AI?
   - Answer: You'd get something that didn't quite work. Missing features. Wrong format. And then you'd blame the AI instead of realizing that the specification wasn't clear enough.

3. **How will you use this the next time you have an idea?** Will this change how you describe what you want?
   - Answer: Yes. You now know that clarity upstream saves chaos downstream. You'll ask yourself the six questions before you ask someone else to build something.

4. **What's the skill being taught here?** What's the actual thing you're learning?
   - Answer: Not "how to write specs" in the abstract. But "how to think clearly about problems" and "how to coach AI (or anyone) to build what you actually want."

---

## Try With AI

You're going to build your own spec by coaching your companion through the same process you just observed.

**Pick one of these projects:**

1. **A todo app** (store tasks, mark them done, filter by status)
2. **A book recommendation system** (users rate books, get recommendations)
3. **A habit tracker** (users log daily habits, see streaks and history)

**Your conversation steps:**

### Prompt 1: The Vague Idea
```
I want to build a [project name]. Here's my initial thought:
"[write a vague 1-sentence description]"

Help me ask the right clarification questions so I can
think through what I'm really building.
```

**What to do**: Your companion will ask 5-7 clarifying questions. Write them down. Read them carefully. Notice how many things you hadn't thought about.

### Prompt 2: Answer and Refine
```
Here are my answers to your questions:

[Answer each question in 1-2 sentences]

What new questions come up based on these answers?
```

**What to do**: Your companion will ask more questions. Keep answering. Do this 2-3 times until your companion says something like: "I think we have a solid foundation now."

### Prompt 3: Extract Your Six Components
```
Based on our conversation, here are my six specification
components for [project name]:

1. Overview: [your one-sentence summary]
2. Inputs: [what goes in, in bullet points]
3. Outputs: [what comes out, in bullet points]
4. Success Criteria: [how we know it works, in bullet points]
5. Constraints: [boundaries and limits, in bullet points]
6. Assumptions: [what we're taking for granted, in bullet points]

Do these look complete? What's missing?
```

**What to do**: Your companion will validate or challenge your components. If challenged, refine them. This is you learning to communicate clearly.

### Expected Outcome

You'll have written a 1-page specification (the six components) for a real project. It won't be perfect, but it will be clear. And that clarity will make the difference between a companion building something useful and building something that misses the mark.

---

## Safety & Next Steps

You've just practiced the core skill of specification-driven development: **asking clarifying questions until you have clarity.** This is the skill that makes AI collaboration work.

When you move to the next lesson, you'll see how to structure this in a tool called SpecifyPlus, which automates and standardizes this process. But the thinking—the questioning, the refining, the clarity—that's what you're doing right now.

**Next**: You'll set up SpecifyPlus and see how to formalize these six components into a specification file that AI tools can work with.
