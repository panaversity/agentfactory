### Core Concept
Heading hierarchy (# through ####) creates document structure that lets AI agents quickly locate sections (features, installation, expected output) and understand relationships between ideas—the foundation of machine-parseable specifications.

### Key Mental Models
- **Heading Levels = Folder Structure**: Level 1 (main), Level 2 (sections), Level 3 (subsections) mirror computer folder hierarchies; AI navigates documents using this structure just as file systems use folder nesting
- **Semantic Meaning in Levels**: Each heading level communicates relationship—`## Features` followed by `### Add Tasks` tells AI "Add Tasks is a detail within Features," not an independent concept
- **No Skipping Levels**: Breaking hierarchy (jumping from # to ###) breaks document semantics for both humans and AI; intermediate levels anchor subsections in proper context

### Critical Patterns
- **Correct Hierarchy**: One Level 1 title, multiple Level 2 sections, Level 3 subsections nested under their parent Level 2
- **Common Sections**: Professional specs follow pattern: `# Title`, `## Problem`, `## Features`, `## Expected Output`, `## Installation`
- **Hash Syntax + Space**: `# Title` (not `#Title`); space is required for markdown parser to recognize heading
- **Single Level 1**: Use `#` only once for document title; subsequent sections use `##`

### AI Collaboration Keys
- **Navigation landmarks**: AI uses headings as search anchors—can locate "Features section" instantly without reading entire document
- **Structure parsing**: AI counts heading levels to understand document scope ("this spec has 4 main sections")
- **Validation completeness**: AI checks if spec includes required sections (Problem, Features, Installation) by looking for heading names
- **Code generation mapping**: "## Features" with 5 items tells AI "generate 5 functions"; structure → implementation

### Common Mistakes
- Forgetting space after `#` symbols (`#Title` instead of `# Title`)
- Using multiple Level 1 headings (multiple `# Title` sections confuses document structure)
- Skipping heading levels (going from `# Title` directly to `### Subsection`)
- Creating too many levels (Level 1, 2, 3, 4, 5, 6 in small document overwhelms readers)
- Vague heading names ("Section A" instead of "Features" or "Installation")

### Connections
- **Builds on**: Lesson 1 understanding that structure enables AI parsing
- **Leads to**: Lesson 3 (lists under headings), Lesson 4 (code blocks after headings), Lesson 5 (complete spec integrating all elements)
- **Professional context**: Documentation sites (Docusaurus, ReadTheDocs) use heading hierarchy to auto-generate table of contents and navigation menus
