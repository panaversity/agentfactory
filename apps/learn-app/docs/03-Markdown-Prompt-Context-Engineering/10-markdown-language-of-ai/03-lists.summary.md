### Core Concept
Unordered and ordered lists communicate whether items are **independent** (features, requirements—use bullets) or **dependent** (steps, sequences—use numbering), allowing AI agents to understand dependencies and generate appropriately independent vs sequential code.

### Key Mental Models
- **Order Matters Test**: The question "Do these items need to happen in sequence?" determines list type: YES = ordered (numbered), NO = unordered (bullets)
- **Semantic Information via List Type**: Unordered list under "Features" tells AI "these can be developed independently"; ordered list under "Installation" tells AI "these must execute in sequence"
- **Dependency Chains**: Ordered lists communicate causal relationships (Step 2 depends on Step 1 completing) that AI uses for workflow generation
- **Parallel Development**: Unordered feature lists enable AI to suggest modular architecture where features can be coded independently

### Critical Patterns
- **Unordered syntax**: `- item` or `* item` (space after dash/asterisk required)
- **Ordered syntax**: `1. step` (numbering can be automatic; markdown renders `1. 2. 3. ...` even if you type `1. 1. 1. ...`)
- **Unordered context**: Features, requirements, options, capabilities (independent items)
- **Ordered context**: Installation steps, troubleshooting sequences, workflows (step-by-step procedures)
- **Space requirement**: `- item` (not `-item`); space signals list to markdown parser

### AI Collaboration Keys
- **Feature count extraction**: AI counts items in unordered feature list to know how many functions/capabilities to generate
- **Sequence understanding**: Numbered steps tell AI this is a workflow or procedure; it can generate sequential scripts
- **Error prevention**: Correct list type prevents AI from: (1) running installation steps out of order, (2) treating features as a ranked priority list, (3) missing dependencies
- **Modular code**: Unordered feature lists encourage AI to generate modular functions instead of one monolithic function

### Common Mistakes
- Using ordered lists for features (creates false impression of priority/sequence when items are independent)
- Using unordered lists for installation steps (AI may execute commands out of order, breaking setup)
- Forgetting space after dash or number (`-item` or `1.item`)
- Mixed list types in same section (inconsistent formatting confuses readers and parsers)
- Creating nested lists without proper indentation (breaks structure for some markdown parsers)

### Connections
- **Builds on**: Lesson 2 (headings create context for lists)
- **Leads to**: Lesson 4 (code blocks showing implementation of features listed here), Lesson 5 (integrating all elements)
- **Real-world**: Professional specifications distinguish features (unordered), installation (ordered), and requirements (unordered) precisely to communicate these dependencies
