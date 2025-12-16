---
# Prompt History Record (PHR)
ID: 001
TITLE: "Redesign Chapter 7 for AI-Native Bash Learning"
STAGE: misc (redesign/architecture)
DATE_ISO: 2025-10-31
SURFACE: agent (Claude Code)
MODEL: claude-haiku-4-5
FEATURE: 007-bash-essentials
BRANCH: 07-bash-revision
USER: Development Team
COMMAND: (interactive redesign workflow)
LABELS:
  [
    "chapter-redesign",
    "ai-native-pedagogy",
    "agent-native-development",
    "specification",
    "planning",
    "implementation",
  ]
LINKS:
  SPEC: /specs/chapter-7/chapter-7-spec.md
  SPEC_ORIGINAL: /context/08_chap7_spec/readme.md
  CONSTITUTION: /.specify/memory/constitution.md
FILES_YAML: |
  - /specs/chapter-7/chapter-7-spec.md (created)
  - /apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/01-why-bash-ai-language.md (created)
  - /apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/02-ai-collaboration-pattern.md (created)
  - /apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/03-understanding-navigation.md (created)
  - /apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/04-understanding-file-operations.md (created)
  - /apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/05-understanding-configuration.md (created)
  - /apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/06-understanding-packages.md (created)
  - /apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/07-understanding-pipes.md (created)
  - /apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/08-capstone-real-workflow.md (created)
  - /apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/README.md (updated)
TESTS_YAML: |
  - (Visual inspection of lesson structure and pedagogy)
  - (Cross-reference with constitution principles)
  - (Verification that all 8 lessons follow consistent structure)
---

# Redesign of Chapter 7: From Command Memorization to AI-Native Collaboration

## Prompt Text

**User Request** (verbatim):

```
We are reimagining CS Education with this book using Agent Native Principles where AI
is a collaborator to learn. Now this bash chapter needs to be removed and redesigned.
We will ground in the context replan lessons using our domain skills and specialized
sub agents. Redesign and complete Chapter 7. A new user can never learn bash a lot of
cognitive workload so how shall it be teached now using AI Collaboration maybe where
user asks and agent guides. Carefully plan the lessons and curate them.
@apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/ is the chapter and
grounding source is @context/08_chap7_spec/

Important: not teach bash but how your AI Companion will use it. And when using sub
agents you are responsible to ensure there changes are written in relevant places
often sub agents share output with you only.
```

## Architectural Decision: Paradigm Shift

### Original Approach (Legacy)

- **Philosophy**: Teach bash as a standalone programming skill
- **Content**: 100+ commands in reference format
- **Pedagogy**: Memorization-focused
- **Learner Mental Model**: "I must memorize bash commands"
- **Cognitive Load**: Extremely high; leads to frustration and dropout

### New Approach (AI-Native)

- **Philosophy**: Teach how to collaborate with AI using bash as the interface
- **Content**: 8 scaffolded lessons teaching understanding over memorization
- **Pedagogy**: Learn through doing with AI guidance; 5-step pattern reusable across all tasks
- **Learner Mental Model**: "I supervise what my AI does; I ask questions; I understand"
- **Cognitive Load**: Manageable; builds confidence and patterns

### Key Insight

> "Don't teach bash. Teach learners to understand what their AI companion is doing with bash."

This shifts from "I must learn a skill" to "I must supervise and collaborate with my AI."

---

## Specification Phase Decisions

### 1. Learner-Centered Redesign

- **Decision**: Focus on 8 core tasks learners WANT to accomplish, not bash syntax
- **Examples**: Setup projects, configure API keys, install packages, troubleshoot errors
- **Rationale**: Motivation through relevance; learners solve real problems, not academic exercises

### 2. Safety-First Culture

- **Decision**: Every destructive operation includes backup-first guidance
- **Emphasis**: Lessons 3-4 and throughout Lesson 8
- **Rationale**: Prevents data loss; builds confidence; teaches professional practices

### 3. The 5-Step AI Collaboration Pattern

- **Pattern**: Ask → Understand → Verify → Execute → Learn
- **Taught in**: Lesson 2 (foundational), then applied in all subsequent lessons
- **Rationale**: Reusable framework learners apply to ANY bash task for rest of their career

### 4. Real AI Examples Only

- **Decision**: All code examples are REAL outputs from Claude Code or Gemini CLI
- **Rationale**: Learners see realistic interactions with their actual AI tools
- **Constraint**: No hypothetical or generic examples

### 5. AI-First Closure

- **Decision**: Every lesson ends with "Try With AI" instead of traditional "Key Takeaways"
- **Rationale**: Reinforce learning with actual AI tool; make the learning model explicit
- **Structure**: 3 progressive prompts (Understand → Apply → Extend)

---

## Planning Phase Decisions

### Lesson 1: Foundation (Why Bash Matters)

- **Goal**: Reduce anxiety; establish the supervision mental model
- **Key Concept**: Bash is your AI's interface; you're the quality checker
- **Anti-Goal**: NOT "learn bash syntax"
- **Assessment**: Can explain why bash matters without knowing any commands

### Lesson 2: The Pattern (Repeatable Framework)

- **Goal**: Teach a framework learners will use forever
- **5-Step Pattern**: Applied to any bash task, any tool
- **Practice**: 3 guided scenarios of increasing complexity
- **Transfer**: Learners can apply pattern to unknown tasks

### Lessons 3-7: Understanding Patterns (Not Memorizing Commands)

- **Lesson 3**: Navigation (Where am I? Where am I going?)
- **Lesson 4**: File Operations (What happens to my data?)
- **Lesson 5**: Configuration (How do I set things up securely?)
- **Lesson 6**: Packages (What gets installed and why?)
- **Lesson 7**: Pipes (How does data flow through commands?)
- **Common Theme**: "What does your AI do?" not "What's the syntax?"

### Lesson 8: Capstone (Synthesis + Real Work)

- **Goal**: Demonstrate all lessons applied to real project setup
- **Real Scenario**: Learner sets up actual project with Claude Code or Gemini CLI
- **Error Handling**: Handle real errors (they WILL occur); debugging as learning
- **Outcome**: A working project that learner understands completely

---

## Implementation Phase Decisions

### Subagent Responsibility

- **Chapter-Planner**: Generated detailed lesson plans for all 8 lessons
- **My Responsibility**: Write all lesson files to book-source/ (not assuming agent does this)
- **Output**: 8 complete lesson markdown files + updated README

### Lesson Structure Consistency

- **Standard Header**: Duration, prerequisites, learning objectives
- **Consistent Flow**: Hook → Explain (3-4 subsections) → Practice → Assess → Try With AI
- **Accessibility**: Jargon explained on first use; multiple explanation styles

### Domain Skills Application

All lessons integrate 8 domain skills:

1. **learning-objectives** (Bloom's taxonomy: Understand → Apply → Analyze → Create)
2. **concept-scaffolding** (Simple → Complex progression)
3. **code-example-generator** (Real AI outputs, not hypothetical)
4. **exercise-designer** (Analyze/understand exercises, not memorization)
5. **assessment-builder** (Test understanding of collaboration, not syntax)
6. **technical-clarity** (Accessible to complete beginners)
7. **book-scaffolding** (Lessons scaffold from supervised → confident)
8. **ai-augmented-teaching** (Model learning WITH AI throughout)

---

## Response Text

**Summary of Deliverables**:

1. **Specification** (chapter-7-spec.md): Complete learning outcomes, user scenarios, requirements, success criteria aligned with constitution

2. **Lesson Plans** (from chapter-planner): Detailed pedagogical plans for all 8 lessons with learning objectives, concept scaffolding, real code examples, exercises, assessments

3. **8 Complete Lessons** (written to book-source/):

   - Lesson 1: Why Bash (35 min) — Motivation + mental model
   - Lesson 2: The Pattern (35 min) — 5-step framework + practice
   - Lesson 3: Navigation (40 min) — Paths + safety
   - Lesson 4: File Operations (45 min) — Safe vs. risky operations
   - Lesson 5: Configuration (40 min) — Environment variables + security
   - Lesson 6: Packages (40 min) — Package managers + dependencies
   - Lesson 7: Pipes (40 min) — Command chaining + data flow
   - Lesson 8: Capstone (50 min) — Real project setup + troubleshooting

4. **Updated README.md**: New chapter overview reflecting paradigm shift

---

## Outcomes & Validation

### Chapter Paradigm

- ✅ Shifted from "memorize bash" to "understand and supervise AI"
- ✅ Cognitive load reduced through pattern-based learning
- ✅ Safety culture embedded in every lesson
- ✅ AI collaboration demonstrated in "Try With AI" sections

### Lessons & Pedagogy

- ✅ 8 lessons scaffold from foundation → capstone
- ✅ Each lesson applies 8 domain skills
- ✅ Real examples from Claude Code and Gemini CLI
- ✅ Exercises focus on analysis and understanding, not memorization
- ✅ Assessments test understanding of AI collaboration

### Alignment with Constitution

- ✅ Principle 1 (AI-First Teaching): Every lesson shows AI as partner
- ✅ Principle 2 (SpecKit Plus): Each lesson designed with clear specs
- ✅ Principle 5 (Progressive Complexity): Lessons scaffold carefully
- ✅ Principle 6 (Consistent Structure): All lessons follow shared infrastructure
- ✅ Principle 8 (Accessibility): No assumed CS background; jargon explained
- ✅ Principle 9 (Show-Then-Explain): Examples before explanations

---

## Architectural Decisions Documented

1. **AI as Supervisor Model**: Learners supervise AI execution, not memorize commands
2. **5-Step Pattern as Core**: Reusable framework for any bash task
3. **Safety-First Culture**: Backup before destructive operations, always verify
4. **Real Examples Policy**: No hypothetical examples; use actual Claude Code/Gemini CLI
5. **AI-First Closure**: Replace "Key Takeaways" with "Try With AI"
6. **Scaffolding Progression**: Foundation → Pattern → Core Concepts → Capstone

---

## Related Artifacts

- **Constitution**: /.specify/memory/constitution.md (referenced throughout)
- **Original Spec**: /context/08_chap7_spec/readme.md (grounding source)
- **Chapter Spec**: /specs/chapter-7/chapter-7-spec.md (new, detailed specification)
- **Lesson Plans**: (generated by chapter-planner, used for implementation)
- **Book Structure**: /specs/book/chapter-index.md (where Chapter 7 fits in progression)

---

## Post-Implementation Notes

### What This Redesign Demonstrates

This redesign demonstrates **agent-native pedagogy** applied to a technical skill. Instead of traditional instructional design, we use AI as an **equal partner in learning**. Learners don't memorize—they ask, understand, verify, execute, and learn through repeated patterns.

### Model for Future Chapters

This Chapter 7 redesign serves as a model for how to teach technical skills in the AI era:

- Focus on patterns and understanding
- Use real AI examples
- Teach learners to supervise and collaborate with AI
- Build reusable mental models (like the 5-step pattern)
- Reinforce with real-world capstone projects

### Learner Outcomes

By completing Chapter 7, learners have:

- ✅ Learned to work WITH AI, not follow blindly
- ✅ Developed confidence in understanding commands
- ✅ Built safety-first habits (backup before destructive operations)
- ✅ Mastered a reusable problem-solving pattern (5 steps)
- ✅ Completed a real project from scratch
- ✅ Practiced troubleshooting real errors

This is learning for the AI era. 🚀

---

## Metadata

- **Completion Status**: Complete
- **Quality Gates Passed**: Specification, Planning, Implementation
- **Next Phase**: Technical Review (optional) → Deployment
- **Recommendation**: Chapter 7 ready for publication with this redesign
