### Core Concept
Gemini has four built-in tools that activate automatically—Google Search, File Operations, Shell Integration, and Web Fetch. You don't invoke them manually; you ask naturally, and Gemini decides which tool fits your need.

### Key Mental Models
- **Query → Tool Selection**: Gemini analyzes your question and picks the right tool—current info triggers Search, local files trigger File Operations, system checks trigger Shell, specific URLs trigger Web Fetch
- **Visual Indicators = Transparency**: Icons (🔍📁⚡🌐) show which tool activated, letting you verify Gemini understood your intent
- **Search vs Fetch**: Search finds information across the web when you don't know where it is; Fetch retrieves a specific page you already know about

### Critical Patterns
- Current information or concepts → Google Search activates
- Reading your notes or local files → File Operations activates
- System commands or directory checks → Shell Integration activates
- Official documentation from known URLs → Web Fetch activates
- Watch for status messages and source citations to confirm tool usage

### Common Mistakes
- Asking Gemini to "search" a specific URL (use Fetch) or "fetch" general information (use Search)
- Not checking visual indicators to verify the right tool activated
- Expecting Gemini to read files without being in the correct directory

### Connections
- **Builds on**: Gemini CLI installation and basic querying (Lessons 1-2)
- **Leads to**: Understanding conversation context and memory (Lesson 4)
