---
title: "Lesson 5: Links, Images & Integration"
description: "Integrating all markdown skills to create a complete AI-parseable specification"
sidebar_label: "Links, Images & Integration"
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
  new_concepts: 4
  assessment: "4 new concepts (emphasis, links, images, specification integration) within A2 limit of 7 ✓"

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

## Major Exercise: Your First Complete Specification

Now it's your turn. You're going to write a **complete specification** using all five markdown elements you've learned.

### Instructions

**Use this template** to write your first complete specification. Create a new file and fill in each section:

```markdown
# [Your Project Title]

## Problem
[What problem does your project solve? 1-2 sentences]

## Solution
[How does your project solve it? 1-2 sentences]

## Features
- [Feature 1]
- [Feature 2]
- [Feature 3]

## Expected Output
Show an example of what your program should produce:

```
[Your expected output here]
```

## Reference Links
- [Documentation Name](https://replace-with-actual-url.com)
```

**Fill in each section** with your project idea, then check your work against the acceptance criteria below.

### Acceptance Criteria

Your specification is complete when:

- [ ] **Heading hierarchy is correct**: One `#` for title, `##` for sections, no skipped levels
- [ ] **Feature list is complete**: At least 3 features, each on its own bullet point
- [ ] **Code block has language tag**: Expected output uses triple backticks with a language identifier (like ` ```python ` or ` ```text `)
- [ ] **At least 1 working link**: Includes a real URL in `[text](url)` format (not a placeholder)
- [ ] **Overall clarity**: Someone reading your spec could understand what to build

### Tips for Success

- **Use emphasis strategically**: Bold feature names and important constraints so they stand out
- **Find real URLs**: Search Google for actual documentation links (Python docs, APIs, libraries)
- **Show concrete output**: Your code block should be specific, not vague ("The program prints the reminder" is vague; the example output above is specific)
- **Keep it concise**: You're writing a specification, not a full design document – 1-2 sentences per section is plenty

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

Now that you've written your first complete specification, it's time to test if it's clear enough for an AI agent to understand.

### Setup
Use ChatGPT web (if you haven't set up another AI tool yet). If you already have Claude Code, Gemini CLI, or another AI companion installed from earlier chapters, feel free to use that instead.

### Prompt Set

**Prompt 1 (Initial Validation):**
Copy your complete specification and paste it into ChatGPT. Then ask:

```
Read this specification carefully. Is this specification clear enough
for you to implement? What parts are clear? What parts are missing
or confusing?
```

**Expected Output:** ChatGPT will tell you which parts are clear and which need more detail. Common feedback includes:
- "Feature X is vague – what exactly should happen?"
- "I don't understand what 'Expected Output' means – can you clarify?"
- "This is clear! I can implement this."

**Prompt 2 (Clarity Refinement - Optional):**
If ChatGPT found confusing parts, ask:

```
Based on your feedback, I've revised the unclear parts.
Here's my improved specification: [paste updated spec]

Is this clearer now? Can you identify any remaining issues?
```

**Expected Output:** ChatGPT confirms that unclear parts are now clearer, or identifies remaining issues. Your specification becomes progressively more clear.

**Prompt 3 (Stretch - Implementation Test):**
Once your spec is clear, you can ask:

```
Implement this specification in Python.
Make the code match the expected output I showed.
```

**Expected Output:** ChatGPT generates Python code matching your specification. If the code doesn't match what you wanted, you now know which part of your specification needs more detail.

### Safety & Ethics Note

When ChatGPT generates code from your specification:
- **Always review the code** before running it – understand what it does
- **Check for errors** – AI-generated code may have bugs
- **Verify it matches your spec** – does it do what you asked?

This validation step teaches you to work *with* AI, not just take its output as fact.