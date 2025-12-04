### Core Concept
Markdown is structured text that creates a bridge between human intent (what you want to build) and machine interpretation (what AI agents understand), forming the **Intent Layer of AIDD** where specifications flow down to AI reasoning and code generation.

### Key Mental Models
- **Structured vs Unstructured**: Structured text (markdown) removes ambiguity by using explicit labels (headings, lists) that AI agents can parse; unstructured paragraphs force AI to guess
- **Three-Layer AIDD**: Markdown is Layer 1 (your intent specification) → AI Reasoning (Layer 2) → Code Generation (Layer 3)
- **Intent Layer Philosophy**: Clear specifications = better AI outputs; vague specifications = AI guessing (poor results)
- **Dual-Nature Format**: Markdown is simultaneously human-readable (no special software) and machine-parseable (structured for AI agents)

### Critical Patterns
- **Specification Quality Effect**: Same information presented as unstructured paragraph vs markdown structure produces dramatically different AI outputs
- **GitHub README Convention**: Professional developers use markdown for README.md files (not Word/TXT) because it's version-control friendly and renders beautifully
- **Semantic Meaning in Structure**: Headings and lists communicate semantic meaning—AI extracts this structure to understand scope, dependencies, and relationships

### AI Collaboration Keys
- AI agents **parse structure**: Markdown headings tell AI where to find features, constraints, and success criteria
- Structured specs reduce **AI guessing**: When you show markdown structure, the AI doesn't have to infer what "reminders" means or how they should sort
- **Specification by example**: Showing expected output in code blocks gives AI concrete targets instead of interpretations

### Common Mistakes
- Treating markdown as "just formatting" rather than specification language that AI parses
- Writing unstructured paragraphs when structure (lists, headings) would clarify scope
- Mixing explicit requirements with vague descriptions in same specification
- Not validating that AI understands your spec (ask AI to restate what it will build before implementing)

### Connections
- **Builds on**: GitHub README familiarity, intuitive understanding that clear writing helps communication
- **Leads to**: Lesson 2 (headings for structure), Lesson 3 (lists for organization), Lesson 4 (code blocks for examples), Lesson 5 (complete specifications)
- **Practical application**: Every AI prompt, GitHub documentation, and technical specification uses markdown; this is foundational to AI-native development
