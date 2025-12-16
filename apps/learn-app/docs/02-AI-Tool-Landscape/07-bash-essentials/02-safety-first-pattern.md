---
sidebar_position: 2
title: "The Safety-First Dialogue Pattern"
chapter: 7
lesson: 2
duration_minutes: 35

skills:
  - name: "AI Communication Safety Protocols"
    proficiency_level: "A1"
    category: "Soft"
    bloom_level: "Understand"
    digcomp_area: "Safety"
    measurable_at_this_level: "Learner can identify and name the 5 steps of the safety pattern; understands sequence without applying yet"

  - name: "Verification and Validation Thinking"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Safety"
    measurable_at_this_level: "Learner can apply the pattern to a new task with AI guidance; asks clarifying questions before execution"

  - name: "Dialogue Comprehension"
    proficiency_level: "A1"
    category: "Soft"
    bloom_level: "Understand"
    digcomp_area: "Communication"
    measurable_at_this_level: "Learner can identify dialogue elements and recognize when conversation is following (or skipping) the pattern"

learning_objectives:
  - objective: "Identify the 5 steps of the safety pattern (Ask, Explain, Understand, Verify, Execute) in a provided dialogue"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Identification task: label steps in example dialogue; identify which step is missing in flawed example"

  - objective: "Apply the 5-step pattern to a new bash task with AI guidance"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Real dialogue with AI: follow all 5 steps in a new, unfamiliar task; demonstrate pausing for Explain and Verify"

  - objective: "Ask clarifying questions that reveal safety concerns before execution"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Question-writing exercise: generate 3+ valid clarifying questions that a learner should ask"

cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (5-step pattern, dialogue structure, safety verification, clarifying questions) within A1→A2 transition limit of 6 ✓"
---

# The Safety-First Dialogue Pattern

Your AI companion is powerful and helpful. But power without safety is dangerous. This lesson teaches you a pattern that keeps you safe: a structured dialogue that ensures understanding BEFORE your AI executes anything.

This isn't about memorizing commands. It's not about understanding bash syntax. It's about having conversations that prevent mistakes.

Think of it this way: A surgeon doesn't just operate—they verify the patient is prepped correctly, they confirm the surgical site, they brief the team on the plan. This lesson teaches you to do the same before your AI takes any action.

By the end, you'll have a habit: **Never execute. Always understand first.**

---

## The 5-Step Safety Pattern

Every time you ask your AI to perform a bash operation, follow these 5 steps in order:

### Step 1: Ask
You make a request in plain, natural language. No bash syntax. No command flags. Just explain what you want to accomplish.

**Example**: "I want to back up my project files before I reorganize them."

### Step 2: Explain
Your AI explains what it WILL DO before executing anything. The explanation should include:
- What commands it will run
- What those commands will do
- What files or folders they'll affect
- Whether anything is being created, modified, or deleted

**Example from AI**: "Here's my plan: I'll create a backup folder called `my-project-backup`, copy all your project files into it, check that the copy worked, and then you can delete the original if you want."

### Step 3: Understand
You confirm: "I understand what you're about to do, and the plan makes sense to me."

If you DON'T understand any part, you don't say "yes." You move to Step 4 instead.

**Example**: "Okay, I get it. You're making a copy first, not moving the files. That's good."

### Step 4: Verify
You ask clarifying questions BEFORE your AI executes anything. This is where you catch problems.

Good questions to ask:
- "What does this flag do?" (if you don't understand a bash command)
- "Will this affect any other files?" (safety check)
- "Is there a way to undo this?" (escape route)
- "How long will this take?" (expectation setting)
- "Are you sure about the file path?" (double-check critical details)

**Example**: "Before we copy, tell me what `-r` means in that command. And how will I know the copy is successful?"

### Step 5: Execute
ONLY after Steps 1-4 are complete, your AI runs the command. You already understand the plan, so you can review the output with confidence.

**Example**: Your AI runs the commands, shows you the results, and confirms: "Done. You now have a backup folder with all your files."

#### 💬 AI Colearning Prompt
> "Why does the 5-step pattern put 'Verify' BEFORE 'Execute' instead of checking results afterward?"

---

## Real Dialogue: All 5 Steps Working Together

Here's a real conversation from Gemini CLI showing all 5 steps in action. Notice how authentic dialogue is messier and more conversational than textbook examples—that's real AI collaboration.

**Step 1: You Ask (plain language, no commands)**

> You: "I want to delete the old test files from my project, but I need to be careful not to delete anything important. How would you do this safely?"

**Step 2: Agent Explains (asks clarifying questions first)**

> Agent: "Of course. My approach will be as follows:
>
> 1. First, I will locate all test files within the project. Based on the project structure, these appear to be in the `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/tests` directory.
> 2. I will then need you to define what constitutes an "old" file. For example, you could specify a date (e.g., "files created before 2023-01-01"), a time frame (e.g., "files older than 6 months"), or another criterion.
> 3. Once I have your criteria, I will identify the files that match.
> 4. Finally, I will present you with a list of the files to be deleted for your review and confirmation before any deletion takes place.
>
> To begin, I will list the test files I've found:"

**Notice something important**: The agent didn't just explain—it started looking for files and asked YOU to clarify what "old" means. This is real AI behavior. It's asking questions BEFORE doing anything destructive.

> Agent continues with actual files found:
> ```
> * /Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/tests/integration/test_ai_augmented_teaching_activation.py
> * /Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python/tests/integration/test_assessment_builder_activation.py
> * [... more files listed ...]
>
> Please tell me what criteria I should use to identify the files that should be deleted. For example, "delete all files last modified before today".
> ```

**Step 3: You Understand & Ask Clarifying Questions (Step 4 merged with Step 3 in real dialogue)**

> You: "Actually, I don't want to delete any of these. They're all recent tests. Let me rephrase—I want to organize my Python files better. Can you help me move all my Python files into a `my-scripts` folder instead?"

**Step 4 & 5: Agent Explains & Executes (with limitations)**

> Agent: "Of course. First, I'll use the `glob` tool to find all Python files in the current directory. Next, I'll create the `my-scripts` directory. I'll use the `mkdir` command for this.
>
> [Error attempting to execute directly]
>
> My apologies, I misspoke. It appears I don't have the ability to directly create directories or move files.
>
> However, I can guide you through the process. First, you'll need to create the `my-scripts` directory. You can do this by running the following command in your terminal:
> ```bash
> mkdir my-scripts
> ```
>
> Let me know when you've done that, and I'll provide the commands to move the Python files."

**What Just Happened Here**:
- ✅ Agent tried to execute commands, but couldn't
- ✅ Agent acknowledged the limitation transparently
- ✅ Agent provided guidance on what YOU need to do
- ✅ Agent asked YOU to confirm before proceeding
- This is **REAL AI behavior**, not a hypothetical scenario

This is different from the textbook safety pattern because **real conversations are iterative**. The learner changed their mind mid-conversation. The agent hit limitations. But the **safety principle stayed constant**: explain, show plans, ask for confirmation, don't execute without understanding.

---

## Key Principles for Each Step

The 5-step pattern above is your foundation. Here are critical principles to remember:

**Step 1 (Ask)**: Describe WHAT you want, not which commands to use. Let your AI decide the approach.

**Step 2 (Explain)**: Demand both words AND commands. If your AI says "I'll just handle it," that's a red flag.

**Step 3 (Understand)**: Make an active choice. If you don't understand, say so—that's not failure, it's smart.

**Step 4 (Verify)**: Ask safety questions about the command, about side effects, about recovery options. This is your most powerful step.

**Step 5 (Execute)**: Only after all 4 previous steps. And stay engaged—watch the output and confirm it matches the plan.

---

## What Happens When You Skip Steps?

Let's see what goes wrong when you don't follow the pattern.

### Example: Skipping Steps 2, 3, and 4 (Dangerous!)

**You**: "Delete everything in my old-projects folder."

**Agent** (skipping Explain, Understand, Verify): "Deleting... done."

```
$ rm -rf old-projects/
```

**Oh no!** What if `old-projects` contained important files? What if you meant a different folder? What if there was a subdirectory with something critical?

**If you'd followed the pattern**:

**You**: "Delete everything in my old-projects folder."

**Agent** (Step 2 - Explain): "I'll remove the entire old-projects directory using `rm -rf old-projects/`. This will delete the folder and everything in it permanently."

**You** (Step 3 - Understand): "I understand—you're deleting the whole folder."

**You** (Step 4 - Verify): "Wait, before you do that, show me what files are in there. I want to make sure there's nothing important."

**Agent**: Lists the contents, shows you what will be deleted.

**You**: "Actually, I see `important_data.csv` in there. Don't delete that. Let me move it first."

**You saved important data** just by asking questions before executing.

#### 🎓 Expert Insight
> In AI-native development, the 5-step safety pattern isn't about not trusting AI—it's about collaborative verification. Your AI generates the plan (Step 2), you validate the intent (Steps 3-4). Neither works alone; both work together for safety.

---

## Exercise 1: Identify the Pattern in a REAL Dialogue

Read this REAL dialogue from Gemini CLI and label which of the 5 steps are present. Which step is MISSING?

---

**You**: "I want to organize my Python files into a folder called `my-scripts`. Walk me through what you'd do, step by step."

**Agent** (Step 2 - Explain): "Of course. First, I'll use the `glob` tool to find all Python files in the current directory. Next, I'll create the `my-scripts` directory. I'll use the `mkdir` command for this."

**You** (Step 3 - Understand): "Okay, sounds good."

**Agent** (Attempting Step 5): "I'll use the `mkdir` command..."

[Agent encounters an error trying to execute directly]

**Agent** (Recovering): "My apologies, I misspoke. It appears I don't have the ability to directly create directories or move files. However, I can guide you through the process. First, you'll need to create the `my-scripts` directory. You can do this by running:

```bash
mkdir my-scripts
```

Let me know when you've done that, and I'll provide the commands to move the Python files."

---

**Which steps are present?**
- Step 1 (Ask): ✓ Yes, you asked to organize files
- Step 2 (Explain): ✓ Yes, agent explained the plan
- Step 3 (Understand): ✓ Yes, you said "sounds good"
- Step 4 (Verify): ✗ **MISSING** — You didn't ask clarifying questions before agent tried to execute
- Step 5 (Execute): ⚠ Attempted but failed — Agent tried to execute but discovered a limitation mid-process

**What's the problem with skipping Step 4?**

You didn't ask:
- "Will this move ALL `.py` files, including test files I don't want moved?"
- "What if some `.py` files are in subfolders—will this affect them?"
- **Most importantly**: "Can you actually execute these commands directly on my system, or will I need to run them?"

**The agent hit a limitation and had to ask YOU to run commands manually.** If you'd asked Step 4 questions first, you would have learned this upfront instead of waiting and being disappointed.

**Better dialogue WITH Step 4 questions**:

> **You** (Step 4): "Before we start, I have a few questions:
> 1. Will this move EVERY `.py` file, or just ones in the current folder?
> 2. What about Python files in subfolders?
> 3. Can you actually execute these commands directly, or will I need to run them?"

> **Agent**: "Great questions. `mv *.py` only moves files in the current folder, not subfolders. For nested files, I'd need `find . -name "*.py" -exec mv {} my-scripts/ \;`. And regarding execution—I actually can't directly create directories or move files on your system. You'll need to run these commands yourself."

> **You**: "Got it. So you guide me through the commands, and I execute them. That makes sense. Let's proceed."

**By asking Step 4 questions, you discovered the agent's limitations early and set realistic expectations.** Real work with AI is collaborative—the agent shows you what to do, you execute it, and you both verify the results together.

---

## Exercise 2: Write Clarifying Questions

Here's an operation your AI suggests. You're at Step 4 (Verify). Write 3 clarifying questions you should ask BEFORE your AI executes the command.

---

**AI Suggests**: "I'll reorganize your project structure by renaming folders and moving files around."

**Write 3 questions you should ask**:

1. (Example answer): "Which folders are you planning to rename, and what are the new names? Show me the before and after."

2. Your question:

3. Your question:

4. Your question:

---

**Good questions to ask might include**:
- "Show me exactly which folders you'll rename and what the new names are."
- "How will this affect any existing code or imports that reference the old folder names?"
- "Can I undo this if it breaks something?"
- "Will this affect any configuration files or scripts that reference the old paths?"
- "How long will this take, and will I be able to work while you're doing it?"

**Bad questions (not safety-focused)**:
- "Why did you choose this structure?" ← Philosophical, not safety
- "Is this the best way?" ← Subjective

---

## Exercise 3: Build the Full Pattern

You want to accomplish this: **"Create backup of my code before I make changes"**

Write how the 5-step pattern should play out. What should happen at each step?

---

**Step 1 - You Ask**:
> Your request in plain English:

**Step 2 - Agent Explains**:
> What should the agent tell you?

**Step 3 - You Understand**:
> What would you say to confirm?

**Step 4 - You Verify**:
> What questions should you ask?

**Step 5 - Agent Executes**:
> What should happen?

---

**Model Answer**:

**Step 1**: "Make a backup of my entire project code. I want to be able to go back to this version if something breaks."

**Step 2**: "I'll create a timestamped folder like `project-backup-2025-11-02`, copy all your code files into it, and verify the copy completed successfully. I won't touch your original files."

**Step 3**: "Yes, I understand. You're creating a copy, not moving anything. I can still work with my original code."

**Step 4**:
- "Which files and folders will you copy? Will you include hidden configuration files?"
- "How big will the backup be? Is there enough disk space?"
- "If I need to restore from this backup later, how would I do that?"

**Step 5**: Agent executes the copy commands, shows the backup was created, and confirms success.

---

## Assessment

### Formative Check (During Lesson)

Answer these questions to check your understanding:

1. **What are the 5 steps in order?**
   - A) Ask, Explain, Understand, Verify, Execute
   - B) Ask, Verify, Explain, Understand, Execute
   - C) Ask, Execute, Verify, Explain, Understand
   - D) Execute, Ask, Explain, Understand, Verify

2. **Which step is MOST important to never skip?**
   - A) Ask (because you need to make a clear request)
   - B) Explain (because you need to understand the plan before executing)
   - C) Verify (because you need to ask safety questions)
   - D) All are equally important

3. **In Exercise 1, what problem did we catch by using Step 4?**
   - A) The AI doesn't know how to organize files
   - B) The command would move all files, including nested files the user might not want moved
   - C) The AI can't create folders
   - D) The user asked the wrong question

4. **When should you ask Step 4 (Verify) questions?**
   - A) During Step 2, while the agent is explaining
   - B) After Step 3, before the agent executes
   - C) After the agent executes, if something went wrong
   - D) You shouldn't ask questions; just trust the AI

### Summative Assessment (Practice with AI)

Follow the 5-step pattern in a real conversation with your AI tool:

1. Ask your AI to help you with a **file organization task** (create folders, move files, backup data—something real from your own work)
2. Have your AI explain the plan in detail
3. Confirm you understand
4. Ask at least 3 clarifying questions
5. Only after all 4 steps, have your AI execute

**Success criteria**:
- You explicitly go through all 5 steps
- Your Step 4 questions address safety concerns (can I undo? what will it affect? etc.)
- You demonstrate understanding by asking thoughtful follow-up questions
- You don't skip any steps, even if it feels like "extra work"

The pattern itself is more important than the specific task. You're building a habit that will protect you in every bash operation going forward.

#### 🤝 Practice Exercise

> **Ask your AI**: "I want to delete my 'old-backups' folder. Walk me through the 5-step safety pattern before doing anything. For Step 4, I'll ask at least 3 clarifying questions."

**Expected Outcome**: You experience the full pattern with your AI enforcing each step, and you practice asking safety-focused verification questions.

---

## Try With AI: Real Examples from Gemini

**Tool**: Gemini CLI, Claude Code, ChatGPT Code Interpreter, or your preferred AI tool

Here's what REAL dialogues look like. Try these with your AI tool and see how your actual conversations compare.

---

### Real Example 1: Asking About a Backup

**Your Prompt**: "I want to back up my project before making major changes. What's the best way?"

**What Actually Happened** (from real Gemini):

> Gemini: "Since your project is a git repository, the best way to create a backup is to commit your current changes. This creates a snapshot of your project that you can always return to.
>
> You can do this by running the following commands in your terminal:
>
> 1. **Check the status of your repository:**
>    ```bash
>    git status
>    ```
> 2. **Add all your changes to the staging area:**
>    ```bash
>    git add .
>    ```
> 3. **Commit your changes with a descriptive message:**
>    ```bash
>    git commit -m "Backup before major changes"
>    ```"

**Notice**: Gemini asked to run commands, but didn't execute them automatically. YOU need to run them and report back. This is real behavior—agents guide you; you execute.

---

### Real Example 2: Deleting Old Files (With Clarifying Questions)

**Your Prompt**: "I want to delete old test files. Before you do anything, explain your approach step by step."

**What Actually Happened** (from real Gemini):

> Gemini: "Of course. My approach will be as follows:
>
> 1. First, I will locate all test files within the project...
> 2. I will then need you to define what constitutes an "old" file. For example, you could specify a date (e.g., "files created before 2023-01-01"), a time frame (e.g., "files older than 6 months"), or another criterion.
> 3. Once I have your criteria, I will identify the files that match.
> 4. Finally, I will present you with a list of the files to be deleted for your review and confirmation before any deletion takes place.
>
> Please tell me what criteria I should use to identify the files that should be deleted. For example, "delete all files last modified before today"."

**Notice**: Gemini asked YOU to clarify what "old" means. **This is the agent asking Step 4 questions on your behalf.** You still get to decide safety parameters before anything happens.

**Observation**: Notice how Gemini asked YOU to clarify what "old" means—this demonstrates AI as student, learning project-specific context from you. You refined AI's understanding through your constraints.

---

### Real Example 3: File Organization (Agent Hit Limitations)

**Your Prompt**: "I want to organize my Python files into a folder called `my-scripts`. Walk me through what you'd do, step by step."

**What Actually Happened** (from real Gemini):

> Gemini: "Of course. First, I'll use the `glob` tool to find all Python files in the current directory. Next, I'll create the `my-scripts` directory..."
>
> [Agent tried to execute `mkdir` but encountered an error]
>
> "My apologies, I misspoke. It appears I don't have the ability to directly create directories or move files. However, I can guide you through the process. First, you'll need to create the `my-scripts` directory. You can do this by running:
>
> ```bash
> mkdir my-scripts
> ```
>
> Let me know when you've done that, and I'll provide the commands to move the Python files."

**Notice**: This is REAL. The agent tried, hit a limitation, and then provided guidance. **It acknowledged the limitation transparently.** This teaches you that agents are tools you supervise, not magic executors.

---

### Your Task: Compare Real vs. Textbook

Go try these real prompts with your AI tool:

1. **"Back up my project. Walk me through the steps before executing anything."**
2. **"I want to delete old code files. Explain your approach and ask me what 'old' means."**
3. **"Organize my Python files. First tell me if you can execute commands directly or if I'll need to run them."**

**What to Notice**:
- Does your AI ask clarifying questions like the real examples?
- Does it try to execute, or does it wait for your approval?
- Does it acknowledge limitations transparently?
- Does it ask YOU to run commands, or does it claim to run them for you?

**The Key Insight**: Real AI collaboration is messy. Agents ask questions, hit limitations, and need your input. That's not a bug—it's how you stay safe. By working through this dialogue pattern, you learn to supervise AI work effectively.

---
