# Lesson Summary: The Architecture of Agent Expertise

## Key Concepts

### Intelligence + Code = Execution (But Not Expertise)
- **Models provide intelligence**: reasoning, analysis, synthesis, generation
- **Code provides execution**: APIs, file system, Python analysis, output generation
- Together: an intelligent agent that can execute (Claude Code)
- **Missing piece**: Domain expertise—the knowledge that makes generic capability specifically useful

### The Expertise Gap
- AI agents are brilliant but lack specialized, practical knowledge
- The bottleneck isn't intelligence—it's access to domain expertise
- Dr. Claude analogy: knows medicine in theory, never done surgery at THIS hospital
- Solution: give Claude access to expertise it doesn't have (via skills)

### Why Skills Are "Just Folders"
- Intentionally simple: organized files anyone (human or agent) can create
- Works with existing tools: Git, Google Drive, zip files
- Three-level architecture protects context window:
  1. **Metadata** (always loaded): description of what skill does
  2. **Instructions** (on-demand): full SKILL.md when needed
  3. **Supporting files** (if needed): scripts, reference docs

### Three Sources of Skills
1. **Foundational**: Basic capabilities (document creation, PDF handling)
2. **Partner/Third-Party**: Software-specific expertise (browser automation, Notion)
3. **Enterprise/Custom**: Organizational knowledge (coding standards, internal workflows)

### Skills + MCP Complementarity
- **MCP servers**: Connection to external data and tools
- **Skills**: Expertise for USING those connections
- Together: Claude queries data (MCP) + analyzes using your procedures (skill)
- Complementary, not competing

### The Stack Analogy
- Models ≈ Processors (massive investment, limited alone)
- Agent runtimes ≈ Operating Systems (orchestrate resources)
- Skills ≈ Applications (where domain expertise lives)
- Paradigm shift: "Stop building agents, build skills instead"

## Mental Models

### Dr. Claude
Brilliant doctor who graduated top of class but has never done surgery at THIS hospital. Knows theory, lacks practical expertise for specific context.

### Phone Apps
100 apps installed, but phone doesn't run all 100 at once. Apps stay closed until needed. Skills work the same—available when relevant, dormant otherwise.

### Computing Stack
You don't build processors (models) or operating systems (runtimes). You build applications (skills) that encode domain expertise.

## Key Insight
The agent infrastructure is mature. Claude Code exists. What's missing is YOUR expertise in a format Claude can access. Skills open the "applications layer" for everyone—including non-technical domain experts.

## Connection to Other Lessons
- **Lesson 04**: WHY you'd encode personal procedures → skills as individual benefit
- **Lesson 05 (this)**: WHERE skills fit architecturally → skills as platform paradigm
- **Lesson 06**: HOW to create skills → SKILL.md syntax and hands-on creation

## Common Misconceptions

| Misconception | Reality |
|---------------|---------|
| "I need to build a custom agent" | The agent exists; you teach it via skills |
| "Skills are just for developers" | Non-technical users (finance, legal, recruiting) create high-value skills |
| "Skills compete with MCP servers" | They're complementary: connectivity + expertise |
| "Skills are fancy saved prompts" | Skills encode reasoning patterns, not commands |

## Questions to Test Understanding

1. What do models provide vs. what does code provide? What's still missing?
2. Why is "expertise, not intelligence" the real bottleneck with AI agents?
3. How does the three-level loading architecture enable scale?
4. What's the difference between what MCP provides and what skills provide?
5. Why does the "skills are applications" analogy matter for adoption?
6. How do skills compound value across an organization?

## Source Material

This lesson is based on the Anthropic talk "Don't Build Agents, Build Skills Instead" by Barry Zhang and Mahesh Murag.

**Watch**: https://www.youtube.com/watch?v=CEvIs9y1uog
