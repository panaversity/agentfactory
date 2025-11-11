---
title: "Links, Images & Your First Complete Specification"
description: "Integrating all markdown skills to create a complete AI-parseable specification"
sidebar_label: "Links, Images & Your First Complete Specification"
sidebar_position: 5
chapter: 9
lesson: 5
duration_minutes: 60
proficiency: "A2"
concepts: 4

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Applying Emphasis"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can use bold for important terms and italic for emphasis in specifications"

  - name: "Creating Links"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create working links to documentation and other resources in markdown"

  - name: "Adding Images"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can add images to markdown documents for README screenshots, diagrams, and logos"

  - name: "Writing Specification Documents"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can write complete specification document integrating all Tier 1 elements (headings, lists, code blocks, emphasis, links)"

  - name: "Understanding Specification Structure"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem Solving"
    measurable_at_this_level: "Student can explain purpose of each specification section and why structure matters for AI parsing"

learning_objectives:
  - objective: "Apply bold and italic formatting to emphasize key terms and concepts in specifications"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Exercise includes correctly emphasized key terms in specification"

  - objective: "Create working hyperlinks to documentation and external resources"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Exercise includes at least one valid link that renders correctly"

  - objective: "Add images to markdown documents for visual communication (screenshots, diagrams, logos)"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Exercise includes at least one properly formatted image"

  - objective: "Integrate all markdown elements (headings, lists, code blocks, emphasis, links, images) into a complete specification document"
    proficiency_level: "A2"
    bloom_level: "Create"
    assessment_method: "Complete specification template submission meets all criteria in rubric"

  - objective: "Recognize how specification structure enables AI agents to parse and understand requirements"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Reflection on AI feedback about specification clarity"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts: (1) emphasis syntax (bold/italic bundled as one pattern), (2) when to use emphasis strategically, (3) link syntax, (4) image syntax, (5) alt text and URL formatting, (6) integrating all prior lessons. Within A2 limit of 7 ✓. Note: This is an integration lesson building on familiar Task Tracker context from Lessons 2-4, which reduces extraneous cognitive load."
  mitigation_strategy: "Cumulative Task Tracker App exercise (familiar context across 4 lessons) reduces cognitive load by eliminating need to conceptualize new project. Students add new skills to existing, well-understood specification rather than learning new syntax AND new context simultaneously."

differentiation:
  extension_for_advanced: "Explore advanced emphasis patterns (nested emphasis, emphasis in lists); research URL encoding for special characters in links; practice writing more complex specifications with nested feature lists"
  remedial_for_struggling: "Start with emphasis-only exercise (only bold, no italic); practice link syntax with provided URLs before finding own; use spec template with more scaffolding prompts"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/001-chapter-9-markdown/spec.md"
created: "2025-11-06"
last_modified: "2025-11-07"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.1"
---

# Links, Images & Your First Complete Specification

## Completing Your Markdown Foundation

You've learned the core building blocks: headings for structure, lists for organization, code blocks for examples, and emphasis for highlighting. Now you'll learn the final two pieces that make specifications truly complete: **links** to connect to external resources, and **images** to show what things look like.

This lesson brings together everything you've learned in Lessons 1-4 plus these final elements. By the end, you'll write your **first complete specification** – one that an AI agent could actually implement from.

---

## Concept 1: Emphasis - Making Text Stand Out

Markdown gives you two ways to emphasize text: **bold** and *italic*.

### When to Use Bold

Use **bold** for terms that are critical to understanding your specification:

- **Feature names**: "**Add tasks** to the list" (shows the exact feature name)
- **Important constraints**: "The app MUST **run offline**" (shows non-negotiable requirements)
- **Key terms**: "Use **UUID** for unique identifiers" (defines technology choices)
- **Action words**: "**Validate** user input before saving" (shows what must happen)

### When to Use Italic

Use *italic* for emphasis or definitions:

- *Emphasis on importance*: "This feature is *especially* critical for users with slow connections"
- *Introduced terms*: "A *specification* is a document describing what to build"
- *Conditional emphasis*: "Only users with *admin* roles can delete tasks"

### The Syntax (Direct Teaching)

Markdown uses **very simple patterns** for emphasis:

**Bold** - surround text with `**double asterisks**`:
```markdown
This is **important**.
```

*Italic* - surround text with `*single asterisks*`:
```markdown
This is *emphasized*.
```

That's it! No complex syntax to memorize. The pattern is:
- Two asterisks on each side = **bold**
- One asterisk on each side = *italic*

---

## Example 1: README with Proper Emphasis

Here's a real specification that emphasizes key information:

```markdown
# Todo List Application

## Problem
Users need a **simple, fast** way to manage their daily tasks. Existing todo apps are too complex.

## Solution
Create a **command-line todo app** that is easy to use and **runs locally** (no internet required).

## Features
- **Add** tasks with a short description
- **List** all tasks with their status
- **Mark** tasks as complete
- **Delete** completed tasks
- **Save** tasks to a file so they persist between sessions

## Expected Output
When a user runs `list`, the app should show:

```
Your tasks:
1. Buy groceries [pending]
2. Finish homework [pending]
3. Call mom [complete]
```

## Why This Matters
By using **bold** on feature names and key constraints, anyone reading this specification – human or AI – immediately understands what's *most important*. This clarity **reduces misunderstandings** and helps AI agents generate more accurate code.
```

Notice how the emphasis makes it easy to scan and understand what matters. Without bold, every word has equal weight and it's harder to find the key ideas.

---

## Concept 2: Links - Connecting to Resources

A specification doesn't exist in isolation. You often need to reference **external documentation**, **examples**, or **standards**. Links solve this problem.

### Why Links Matter in Specifications

When you write "Use Python's requests library," the reader might not know:
- What is the requests library?
- Where do I find it?
- How do I use it?

But if you write "[Use Python's **requests** library](https://requests.readthedocs.io/)," the reader can click through to complete documentation instantly.

### The Syntax (Direct Teaching)

Markdown links are also very simple:

```markdown
[link text](url)
```

- **link text** = what the reader sees and clicks
- **url** = where the link goes

Example:

```markdown
Read the [Python documentation](https://docs.python.org/) for help.
```

That renders as: Read the [Python documentation](https://docs.python.org/) for help.

### Common Mistake: Spaces in URLs Break Links

Beginner mistake:

```markdown
[Wrong link](https://docs.python.org/3/ reference guide)
```

This **won't work** because the space breaks the URL. Either:
1. Use a URL without spaces (recommended):
```markdown
[Python reference](https://docs.python.org/3/reference/)
```

2. Or use URL encoding (replace space with `%20`):
```markdown
[reference guide](https://docs.python.org/3/reference%20guide)
```

For specifications, **always stick with option 1** – find clean URLs without spaces.

---

## Example 2: Links to Documentation

Here's how to add helpful links to your specification:

```markdown
# Weather App Specification

## Required Libraries
- [Python requests library](https://requests.readthedocs.io/) - for making API calls
- [OpenWeatherMap API](https://openweathermap.org/api) - free weather data source

## Data Format
Data should be formatted as JSON. See the [JSON specification](https://www.json.org/) for details.

## Testing
When you test your app, verify it works like the examples in the [OpenWeatherMap docs](https://openweathermap.org/current).

```

Now readers can click through and see:
- How to use the requests library
- Where to get weather data
- What JSON looks like
- What sample outputs should look like

This **dramatically improves clarity** because you're not just describing, you're *showing* where to find more information.

---

## Example 3: Invalid Link Syntax (Common Error)

Here's what NOT to do:

```markdown
[Check this out](https://docs.python.org/3/ with spaces)
```

The space in the middle of the URL breaks the link. The reader will see the text highlighted, but clicking won't work properly.

**Fix it by removing spaces:**

```markdown
[Check the Python docs](https://docs.python.org/3/reference/)
```

---

## Concept 3: Images - Showing What Things Look Like

Sometimes words aren't enough. You need to **show** what something looks like. That's where images come in.

### Why Images Matter in Documentation

Images help readers understand:
- **What the UI looks like** - Screenshots show expected interface
- **How data flows** - Diagrams explain system architecture
- **Project branding** - Logos make READMEs professional

### The Syntax (Very Similar to Links!)

Markdown images use almost the same syntax as links, with one difference - an exclamation mark `!` at the start:

```markdown
![alt text](image-url)
```

- **alt text** = description of the image (shown if image doesn't load, read by screen readers)
- **image-url** = where the image is located (web URL or local file path)

Example:

```markdown
![Python logo](https://www.python.org/static/community_logos/python-logo.png)
```

### Where Images Come From

**Option 1: Online images** (easiest for beginners)
Use a direct image URL from the web:

```markdown
![Example screenshot](https://example.com/screenshot.png)
```

**Option 2: Local images in your project**
Put images in a folder (like `images/` or `assets/`) and reference them:

```markdown
![App screenshot](./images/screenshot.png)
```

**For beginners**: Start with online image URLs (from GitHub, documentation sites). Later you can add local images to your projects.

### AI Native Approach: Let AI Help with Images

If you need diagrams or don't have good screenshots yet:

**Ask your AI companion:**
```
I need a simple architecture diagram showing: User → API → Database.
Can you suggest a tool to create this, or describe how I should visualize it?
```

AI can suggest tools (like Excalidraw, draw.io) or help you find placeholder images while building your spec.

---

## Example: README with Image

Here's how images make READMEs more professional:

```markdown
# Weather Dashboard

![Weather Dashboard Screenshot](https://via.placeholder.com/800x400.png?text=Weather+Dashboard)

## Problem
Users want quick access to local weather information.

## Features
- **Display** current temperature and conditions
- **Show** 7-day forecast
- **Save** favorite locations

## Screenshot
Here's what the app looks like in action:

![App interface](https://via.placeholder.com/600x300.png?text=App+Interface)
```

See how images make it immediately clear what the app looks like? That's powerful.

### Common Image Mistakes

**Mistake 1: Forgetting the `!` at the start**

```markdown
[Missing exclamation](image.png)
```

This creates a *link* to the image, not an embedded image. Always use `![...]` for images.

**Mistake 2: Broken image paths**

```markdown
![Screenshot](screenshot.png)
```

If `screenshot.png` doesn't exist in the same folder, the image won't show. Check your paths!

**Mistake 3: Too many large images**

Don't embed 20 screenshots in one README. Use images strategically:
- 1 logo/banner at the top
- 1-2 key screenshots showing the app
- Diagrams only when words aren't enough

---

## Concept 4: Bringing It All Together - Your First Complete Specification

You now know:
- **Headings** (Lesson 2) - to create document structure
- **Lists** (Lesson 3) - to organize features and steps
- **Code blocks** (Lesson 4) - to show expected output
- **Emphasis** (Lesson 5) - to highlight what matters
- **Links** (Lesson 5) - to connect to resources
- **Images** (Lesson 5) - to show what things look like

When you combine all five elements, you create a **specification document** that both humans and AI can understand clearly.

Here's the complete picture:

```markdown
# Smart Reminder App

## Problem
Users forget important deadlines because they have no easy way to track them.

## Solution
Build a **command-line reminder app** that:
- **Stores** reminders in a file
- **Checks** for upcoming reminders when the app starts
- **Notifies** users of due reminders

## Features
- **Create** a new reminder with date and time
- **List** all reminders, showing which are due soon
- **Mark** reminders as done
- **Delete** completed reminders

## Required Libraries
- [Python datetime module](https://docs.python.org/3/library/datetime.html) - for handling dates
- [Python JSON module](https://docs.python.org/3/library/json.html) - for storing reminders

## Expected Output
When a user lists reminders, they should see:

```python
Upcoming reminders:
1. Buy birthday gift - due TODAY
2. Call dentist - due in 3 days
3. Submit report - due in 5 days
```

## Success Criteria
- ✓ App runs from the **command line**
- ✓ Reminders **persist** between sessions (saved to file)
- ✓ All dates are **validated** (no invalid dates like "Feb 30")
- ✓ Code is **tested** (ask an AI to review for bugs)
```

See what's happening here?

- **Headings** (`#`) create a clear structure
- **Bold** highlights feature names and important concepts
- **Lists** show features and criteria clearly
- **Code blocks** with language tags show expected output
- **Links** point to documentation

This is what an AI agent reads when you hand them your specification. Because it's **well-structured** and **clear**, the AI can generate more accurate code.

---

## Major Exercise: Task Tracker App (Part 4 - Complete It!)

**Final Step**: Open your Task Tracker App specification from Lessons 2-4. You'll now **complete it** by adding links, images, and emphasis — creating a publication-ready specification.

### Your Task for Lesson 5

Add the final polish to make your specification complete and professional:

**Part 1: Add Emphasis to Key Terms**

Go through your specification and add **bold** to important terms:

```markdown
## Problem
Users forget important tasks because they lack a **simple, offline** task management system.

## Features

### Add Tasks
- Create tasks with **title** and **description**
- Set **optional due dates**
- Assign **priority levels** (high, medium, low)

### View Tasks
- Display all tasks with **status indicator**
- Filter by priority or due date
- Show completed and pending **separately**
```

**Part 2: Add Reference Links**

Add a new section with helpful links:

```markdown
## Reference Links
- [Python datetime documentation](https://docs.python.org/3/library/datetime.html) - for handling due dates
- [Python file I/O guide](https://docs.python.org/3/tutorial/inputoutput.html#reading-and-writing-files) - for saving tasks to file
- [Task management best practices](https://en.wikipedia.org/wiki/Task_management) - understanding task tracking concepts
```

**Part 3: Add an Image (Optional but Recommended)**

At the top of your spec, add a screenshot or diagram:

```markdown
# Task Tracker App

![Task Tracker Screenshot](https://via.placeholder.com/800x200.png?text=Task+Tracker+CLI)

## Problem
...
```

Use a placeholder for now (`https://via.placeholder.com/800x200.png?text=Task+Tracker+CLI`). In real projects, you'd replace this with an actual screenshot.

### Acceptance Criteria

Your **complete Task Tracker App specification** is ready when:

- [ ] **Heading hierarchy is correct** (from Lesson 2): One `#` for title, `##` for sections, `###` for feature details
- [ ] **Lists are appropriate** (from Lesson 3): Unordered for features, ordered for installation steps
- [ ] **Code blocks show expected output** (from Lesson 4): At least one fenced block with `text` tag showing what the program prints
- [ ] **Emphasis highlights key terms** (Lesson 5): Bold on important feature names and constraints
- [ ] **At least 3 working reference links** (Lesson 5): Real URLs to Python docs or relevant resources
- [ ] **Image is included** (Lesson 5 - optional): Placeholder or actual screenshot at the top
- [ ] **Overall clarity**: Someone reading your spec could implement this app

### Congratulations!

You've just completed a **full specification built across 4 lessons**:
- **Lesson 2**: Created heading structure
- **Lesson 3**: Added lists to organize features and steps
- **Lesson 4**: Added code blocks to show expected behavior
- **Lesson 5**: Added emphasis, links, and images for clarity

This is **exactly how real specifications are built** — iteratively, adding detail at each stage. You now have a publication-ready spec that an AI agent can implement.

---

## Understanding Why This Matters

At this point, you've learned **all the foundational markdown skills** an AI-native developer needs. But more importantly, you understand **why** these skills matter:

- Specifications are how you communicate intent to AI agents
- Clarity in your specification directly affects code quality
- Structure (headings, lists, code blocks) lets AI agents parse meaning
- Emphasis and links guide attention to what matters most

You're not just learning markdown syntax – you're learning the **Intent Layer** of AIDD (AI-Driven Development). This is the core skill that makes everything else possible.

When you can write a clear specification, you can:
- Generate code that matches your intent
- Validate whether AI output is correct
- Refine your spec when AI misunderstands
- Collaborate with others (and AI) effectively

That's **professional-level development** – even if you haven't written a single line of code yet.

---

## Try With AI

Now test your **complete Task Tracker App specification** with AI implementation and validation.

### Setup
Use ChatGPT web (if you haven't set up another AI tool yet). If you already have Claude Code, Gemini CLI, or another AI companion installed from earlier chapters, feel free to use that instead.

### Prompt Set

**Prompt 1 (Initial Validation):**
Copy your **complete Task Tracker App specification** and paste it into ChatGPT. Then ask:

```
Read this Task Tracker App specification carefully. Is this specification
clear enough for you to implement? What parts are clear? What parts are
missing or confusing?
```

**Expected Output:** ChatGPT will tell you which parts are clear and which need more detail.

**Prompt 2 (Clarity Refinement - If Needed):**
If ChatGPT found confusing parts, revise them and ask:

```
Based on your feedback, I've revised the unclear parts.
Here's my improved specification: [paste updated spec]

Is this clearer now? Can you identify any remaining issues?
```

**Prompt 3 (Full Implementation Test):**
Once your spec is validated, ask:

```
Implement this Task Tracker App specification in Python.
Make the code match the expected output I showed in the spec.
Include all features I listed.
```

<Quiz title="Chapter 5 Quiz" questions={[{"question":"What are the six phases of the traditional software development lifecycle mentioned in the chapter?","options":{"a":"Discovery, Design, Development, Debugging, Deployment, Decommissioning","b":"Planning, Design, Implementation, Testing, Deployment, Operations","c":"Requirements, Architecture, Coding, Review, Release, Retirement","d":"Idea, Prototype, Build, Test, Launch, Iterate"},"correct_answer":"b","explanation":"The chapter explicitly lists the phases: \u0027The traditional software development lifecycle looks something like this: Planning → Design → Implementation → Testing → Deployment → Operations.\u0027"},{"question":"How does the AI-augmented approach to the \u0027Planning \u0026 Requirements\u0027 phase differ from the traditional approach?","options":{"a":"AI completely replaces product managers.","b":"AI helps extract requirements, suggest edge cases, and identify inconsistencies before development starts.","c":"The planning phase is skipped entirely.","d":"AI only helps with formatting the requirements document."},"correct_answer":"b","explanation":"The text states the AI-augmented approach includes: \u0027Natural language processing helps extract requirements... AI agents suggest edge cases... Automated analysis identifies inconsistencies and ambiguities...\u0027"},{"question":"In the \u0027Testing \u0026 Quality Assurance\u0027 phase, what is a key benefit of using an AI-augmented approach?","options":{"a":"It eliminates the need for human QA engineers.","b":"It only works for unit tests, not integration tests.","c":"AI can generate comprehensive test suites and identify edge cases that humans might miss.","d":"It makes testing slower but more thorough."},"correct_answer":"c","explanation":"The chapter highlights that with AI, it \u0027generates comprehensive test suites from requirements and code\u0027 and \u0027Automatically identifies edge cases developers didn\u0027t think to test.\u0027"},{"question":"What is the \u0027compounding effect\u0027 of AI transformation across the development lifecycle?","options":{"a":"Each phase becomes more complex and expensive.","b":"Improvements in one phase are isolated and do not affect other phases.","c":"An improvement in an early phase (like planning) leads to benefits and efficiencies in all subsequent phases.","d":"The total time for development increases due to the need for more reviews."},"correct_answer":"c","explanation":"The text explains, \u0027Each phase improvement compounds with others. When AI helps you identify edge cases during planning, you write better requirements. Better requirements lead to better architecture,\u0027 and so on."},{"question":"What is happening to specialized roles like \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 as a result of AI tools?","options":{"a":"These roles are becoming more distinct and siloed.","b":"The boundaries between these roles are blurring as AI enables individuals to handle more responsibilities.","c":"These roles are being completely eliminated.","d":"Only the developer role is changing."},"correct_answer":"b","explanation":"The chapter notes, \u0027The boundaries between \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 are blurring. AI tools enable individual contributors to handle responsibilities that previously required dedicated specialists.\u0027"}]} />

