### Core Concept
Integration lesson combining emphasis (bold/italic), links (referencing external resources), and images (visual communication) with Lessons 2-4 elements to create **publication-ready specifications** that AI agents can implement directly from.

### Key Mental Models
- **Semantic markup hierarchy**: Headings (structure) → Lists (organization) → Code blocks (examples) → Emphasis (attention) → Links (resources) → Images (visualization)
- **Emphasis as semantic signal**: Bold marks critical terms/constraints; italic marks introduced terms/conditions; both guide AI to what matters most
- **Links as knowledge graph**: References to documentation, standards, and related resources create context; AI can follow links to understand deeper intent
- **Visual documentation**: Images show what success looks like (screenshots, diagrams, UI mockups) preventing ambiguity about appearance/interaction
- **Cumulative learning**: Each element added to familiar Task Tracker context reduces cognitive load—students add skills to known project, not learn new syntax + new project simultaneously

### Critical Patterns
- **Bold syntax**: `**text**` (two asterisks on each side) for critical terms, feature names, constraints
- **Italic syntax**: `*text*` (one asterisk on each side) for emphasis, introduced terms, conditions
- **Link syntax**: `[link text](url)` where link text is visible/clickable and url is destination
- **Image syntax**: `![alt text](image-url)` (note: leading `!` distinguishes from links)
- **Integration pattern**: Heading → Description + Bold terms → Bulleted lists → Code blocks showing output → Links to docs → Image showing interface

### AI Collaboration Keys
- **Emphasis guides parsing**: AI recognizes bold terms as critical—these become requirements, not optional enhancements
- **Links provide context**: AI can reference linked documentation to understand standards, patterns, and deep context
- **Images communicate design**: Screenshots/diagrams let AI understand UI expectations without prose descriptions
- **Complete specification semantics**: Full spec structure tells AI: "here's what matters (bold), here's where to learn more (links), here's what it looks like (images)"

### Common Mistakes
- Forgetting space in markdown (`**text**` vs `** text**`; latter breaks formatting)
- URL spaces breaking links (`[text](url with spaces)` fails; use clean URLs or URL encoding)
- Missing `!` in image syntax (creates link instead of embedded image)
- Too much emphasis (bolding every other word dilutes importance; reserve for truly critical terms)
- Broken image paths (images that don't exist render as broken in markdown viewers)
- Using links without context (link text should be descriptive: "[Python docs](url)" not "[click here](url)")

### Connections
- **Builds on**: All previous lessons (2-4) applied to single Task Tracker App specification iteratively built across 4 lessons
- **Integration principle**: Lessons 1-4 taught building blocks separately; Lesson 5 shows how professional specifications integrate all elements
- **Real-world practice**: Professional READMEs combine: clear heading structure + feature lists + code examples + bold important terms + links to docs + screenshots
- **Specification maturity**: Students move from "markdown syntax" understanding to "specification writing" competency

### Common Mistakes (Integration Level)
- Overwhelming single specification with too many images/links (use strategically: 1 logo, 1-2 key screenshots, links only to essential resources)
- Inconsistent emphasis style (bold all features in one section, italic them in another)
- Missing integration: properly formatted code blocks but no emphasis on what's important within them
- Specification completeness gap: all syntax correct but missing practical context (links to libraries, screenshots of expected interface)

### Connections
- **Prerequisite knowledge**: Lessons 1-4 markdown elements
- **Cumulative assessment**: Task Tracker App built iteratively through 4 lessons demonstrates full specification-writing workflow
- **Next steps**: Chapter 11 (Prompt Engineering) applies these markdown specifications as inputs to AI prompts; Chapter 12 (Context Engineering) manages multi-specification complexity
