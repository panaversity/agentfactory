### Core Concept
Zed embeds AI directly in your editor through three main features—inline assistant (Ctrl+I for complex tasks), tab autocomplete (continuous suggestions), and multi-model configuration—enabling real-time iteration rather than copy-pasting from external tools.

### Key Mental Models
- **Synchronous Collaboration**: AI lives in your editor responding in milliseconds, unlike terminal tools where you request→wait→receive in a separate context
- **Model-Task Matching**: Use fast models (GPT-4o Mini) for autocomplete, capable models (Claude Sonnet) for complex reasoning—match the tool to the job
- **Incremental Refinement**: Don't specify everything upfront; generate basic code, then iterate: "add error handling", "add type hints", "extract into function"

### Critical Patterns
- **Ctrl+I** opens inline assistant for code generation at cursor position
- **Tab** accepts ghost-text autocomplete suggestions as you type
- **Select + Ctrl+I** enables refactoring by describing the transformation you want
- Configure models per feature in `settings.json`: fast model for `tab_completion`, capable model for `assistant`
- Red flags checklist: missing type hints, no docstrings, no error handling, hardcoded values, unclear names

### Common Mistakes
- Using slow models for autocomplete (distracts with latency) or fast models for complex logic (misses edge cases)
- Accepting AI code without evaluation—always check for red flags before committing
- Writing vague prompts like "write a function" instead of specifying signature, validation behavior, and constraints

### Connections
- **Builds on**: Zed installation and AI provider configuration (Lesson 2)
- **Leads to**: Cursor IDE features to compare how same concepts manifest in different tools
