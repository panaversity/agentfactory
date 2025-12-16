### Core Concept
Before executing any bash operation through AI, follow a structured 5-step dialogue pattern: Ask, Explain, Understand, Verify, Execute. This pattern ensures you understand what will happen BEFORE anything runs—like a surgeon confirming the surgical site before making an incision.

### Key Mental Models
- **5-Step Safety Pattern**: Ask (plain language request) → Explain (AI describes plan) → Understand (you confirm comprehension) → Verify (ask clarifying questions) → Execute (only after all previous steps)
- **Verify Before Execute**: The power step is Step 4—asking safety questions BEFORE execution catches problems when they're still preventable
- **Collaborative Verification**: AI generates the plan, you validate the intent—neither works alone, both work together for safety

### Critical Patterns
- Describe WHAT you want, not which commands to use—let AI decide the approach
- Demand both words AND commands in explanations—"I'll just handle it" is a red flag
- Ask recovery questions: "Is there a way to undo this?" before any destructive operation
- Stay engaged during execution—watch output and confirm it matches the plan
- Real AI conversations are iterative: agents ask questions, hit limitations, and need your input

### Common Mistakes
- Skipping Step 4 (Verify) because it feels like "extra work"—this is where you catch critical problems
- Saying "sounds good" without truly understanding the plan
- Not asking about side effects: "Will this affect any other files?"
- Assuming AI can always execute commands directly—many agents guide you to run commands yourself
- Trusting AI outputs without asking about edge cases (nested files, hidden configs)

### Connections
- **Builds on**: Terminal and bash fundamentals (Lesson 1), AI tool landscape (Chapters 5-6)
- **Leads to**: Bash commands for file and directory management (Lesson 3), git operations (Lesson 4)
