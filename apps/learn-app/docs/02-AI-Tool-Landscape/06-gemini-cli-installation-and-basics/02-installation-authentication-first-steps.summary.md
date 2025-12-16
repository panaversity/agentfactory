### Core Concept
Installing Gemini CLI brings an AI assistant directly into your terminal—authenticate with your Google account to get 60 requests/minute and 1,000 requests/day free.

### Key Mental Models
- **Browser OAuth = Security**: Authentication happens in your browser (not terminal) because browsers have secure connections to Google; the terminal just receives a token after you approve
- **Shell Mode = Context Preservation**: The `!command` prefix runs terminal commands without leaving Gemini, keeping your conversation context intact
- **AI Troubleshooting = Describe → Iterate**: When errors occur, provide full context (OS, versions, complete error) and iterate with AI until solved

### Critical Patterns
- Three installation methods: `npm install -g` (permanent), `npx` (temporary), version tags (`@latest`, `@preview`)
- Verify with `gemini -v` before proceeding
- Slash commands navigate the interface: `/help`, `/tools`, `/stats`, `/quit`
- Shell mode (`!ls`, `!git status`) runs terminal commands inside Gemini
- Copy complete error messages with system context when troubleshooting

### Common Mistakes
- Starting without Node.js 20+ installed—check with `node --version` first
- Copying only the last line of an error instead of the full output with context
- Trying to memorize commands instead of asking "What commands are available?"

### Connections
- **Builds on**: Chapter 6 Lesson 1 (Gemini CLI overview and tool selection framework)
- **Leads to**: Lesson 3 (built-in tools) and Lesson 4 (GEMINI.md context files)
