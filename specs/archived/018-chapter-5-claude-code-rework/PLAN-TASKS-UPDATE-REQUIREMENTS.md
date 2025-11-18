# Plan.md and Tasks.md Update Requirements

**Date**: 2025-01-13
**Status**: Specifications Updated, Plan/Tasks Pending

---

## 📋 Required Changes to plan.md

### **Lesson 4: Update Title and Metadata**

**Current** (line 473):
```markdown
## Lesson 4: Skills & MCP Integration
**Duration**: 6-8 minutes
```

**Change to**:
```markdown
## Lesson 4: Skills, Plugins, and MCP Integration
**Duration**: 10-12 minutes
```

### **Learning Objectives: ADD 2 NEW**

**Add Objective 4**:
```markdown
- **Objective 4** (A2 - Apply): Demonstrate installing a plugin via `/plugin` command and browsing marketplace
  - Assessment: Can execute `/plugin marketplace add anthropics/claude-code` → `/plugin install feature-dev` → verify activation
  - Bloom's Level: Apply
  - Skills Taught: "Plugin Installation and Marketplace Navigation" (A2, Hands-on)
```

**Add Objective 5**:
```markdown
- **Objective 5** (A2 - Analyze): Distinguish between Skills (autonomous), Plugins (bundled), and MCP (external) in extensibility hierarchy
  - Assessment: Can explain: "Plugins are containers that bundle Skills + Commands + Agents + Hooks + MCP configs"
  - Bloom's Level: Analyze
  - Skills Taught: "Extensibility Architecture Comprehension" (A2, Conceptual)
```

### **Content Outline: REPLACE with 4 Sections**

**Current**: 5 sections (Skills concept, MCP protocol, Real-world examples, Why it matters, Building custom)

**Replace with**:
```markdown
1. **Agent Skills Deep Dive** (3-4 min):
   - Progressive disclosure pattern (3 levels: metadata → core → referenced files)
   - SKILL.md structure (YAML frontmatter + markdown + bundled files)
   - PDF skill example (form filling + field extraction with Python script)
   - When Claude invokes skills autonomously

2. **Plugins System** (3-4 min):
   - What plugins bundle (commands + agents + skills + hooks + MCP)
   - Plugin architecture (.claude-plugin/plugin.json manifest + directories)
   - Installation workflow (`/plugin` → browse → marketplace add → install → restart)
   - Community ecosystem (anthropics/claude-code, Dan Ávila, Seth Hobson 80+ sub-agents)
   - Team configuration (.claude/settings.json)

3. **MCP Integration** (2-3 min):
   - Protocol for external tool connections
   - JSON configuration structure (mcpServers → command/args/oauth)
   - Examples: GitHub MCP (@github), Filesystem MCP (@filesystem)
   - How MCP fits inside plugins

4. **Relationship Hierarchy** (1-2 min):
   - Visual diagram: Plugins (containers) → contain → [Skills, Commands, Agents, Hooks, MCP]
   - Skills = autonomous (Claude decides when)
   - Commands = user-invoked (slash commands)
   - Agents = specialized subagents
   - Hooks = event-triggered automation
   - MCP = external integrations
```

### **Try With AI: REPLACE with 4 Prompts**

**Current**: 3 prompts (Explore Skills, Integration value, Future learning)

**Replace with**:
```markdown
#### Prompt 1: Explore Plugins
claude "/plugin"
# Browse marketplace, see available plugins

#### Prompt 2: Understand Skills Architecture
claude "Explain agent skills in Claude Code. What is progressive disclosure? Give me the PDF skill example showing the 3 levels."

#### Prompt 3: Install Example Plugin (Hands-On)
claude "/plugin marketplace add anthropics/claude-code"
claude "/plugin install feature-dev"
# Restart Claude Code
# Verify: `/help` should show new commands

#### Prompt 4: Plan Custom Plugin (Aspiration)
claude "If I wanted to create a [framework name] plugin with [workflow] shortcuts, what would I include? Commands? Agents? Skills? Hooks?"
```

### **Success Criteria Mapping: UPDATE**

**Current**:
- SC-013: Strategic understanding

**Change to**:
- SC-005: Agent skills progressive disclosure
- SC-006: Plugin installation hands-on
- SC-007: Relationship hierarchy
- SC-008: MCP configuration understanding
- Eval 8: Skills and Plugins Personalization

### **Edge Cases: UPDATE**

**Current**: "Confusion between Skills, Plugins, and MCP"

**Update with clear hierarchy**:
```markdown
**Confusion between Skills, Plugins, and MCP**:
- **Skills**: Autonomous capabilities (Claude invokes based on task context)
- **Plugins**: Containers that bundle multiple components (commands + agents + skills + hooks + MCP)
- **MCP**: External tool integration protocol (contained within plugins)
- **Relationship**: Plugins → contain → Skills (among other components)
- **Key difference**: Skills are invoked automatically; Plugins are installed by user
```

---

## 📋 Required Changes to tasks.md

### **Phase 6 (User Story 4): REPLACE Tasks**

**Current Tasks** (T019-T022):
- T019: Write Lesson 4 (Skills & MCP)
- T020: Add MCP configuration example
- T021: Add "Try With AI" section
- T022: Add frontmatter

**Replace with** (T019-T027):
```markdown
- [ ] T019 [P] [US4] Write Lesson 4 Agent Skills Deep Dive section (04-skills-plugins-mcp.md):
  - Progressive disclosure pattern (3 levels with clear explanations)
  - SKILL.md structure (YAML frontmatter example + markdown + bundled files)
  - Filesystem discovery and dynamic loading explanation
  - PDF skill concrete example (form filling + Python script for field extraction)
  - When Claude invokes skills autonomously vs user-invoked commands

- [ ] T020 [P] [US4] Write Lesson 4 Plugins System section (04-skills-plugins-mcp.md):
  - Plugin architecture (.claude-plugin/plugin.json manifest structure)
  - What plugins bundle (commands/, agents/, skills/, hooks/, .mcp.json)
  - Installation workflow demonstration (`/plugin` command with screenshots/examples)
  - Marketplace discovery (anthropics/claude-code, Dan Ávila DevOps/docs/testing, Seth Hobson 80+ sub-agents)
  - Team configuration (.claude/settings.json extraKnownMarketplaces)
  - Use cases: team standards enforcement, framework shortcuts, internal tools, debugging, deployment

- [ ] T021 [P] [US4] Write Lesson 4 MCP Integration section (04-skills-plugins-mcp.md):
  - MCP protocol definition (external tool integration standard)
  - JSON configuration structure (mcpServers object → server name → command/args/oauth)
  - GitHub MCP example (npx @anthropic-ai/mcp-server-github with OAuth scopes)
  - Filesystem MCP example (npx @modelcontextprotocol/server-filesystem with path restrictions)
  - @-mentions activation (@github, @filesystem for invoking MCP servers)
  - How MCP configs fit inside plugins

- [ ] T022 [P] [US4] Write Lesson 4 Relationship Hierarchy section (04-skills-plugins-mcp.md):
  - Visual diagram showing: Plugins (containers) → contain → [Skills + Commands + Agents + Hooks + MCP]
  - Skills distinction: autonomous capabilities (Claude decides when to invoke)
  - Commands distinction: user-invoked (slash commands like `/plugin`, `/code-review`)
  - Agents distinction: specialized subagents (Explore, Plan mode)
  - Hooks distinction: event-triggered automation (PreToolUse, PostToolUse, SessionStart)
  - MCP distinction: external integrations (GitHub, Filesystem, Google Drive, Jira)
  - Clear hierarchy: Plugins are broadest container; Skills are one component among many

- [ ] T023 [US4] Add "Try With AI" section to 04-skills-plugins-mcp.md (4 prompts expanded from 3):
  - Prompt 1: Explore Plugins (`/plugin` → browse marketplace)
  - Prompt 2: Understand Skills (`Explain agent skills, progressive disclosure, PDF example`)
  - Prompt 3: Install Example Plugin (Hands-on: `/plugin marketplace add anthropics/claude-code` → `/plugin install feature-dev` → restart → verify)
  - Prompt 4: Plan Custom Plugin (`If I wanted [framework] plugin with [workflow], what include?`)

- [ ] T024 [US4] Add SKILL.md Example to 04-skills-plugins-mcp.md:
  - Complete SKILL.md structure showing YAML frontmatter
  - Example: PDF skill with name/description in frontmatter
  - Markdown instructional content
  - Reference to bundled Python script for form extraction
  - 3-level progressive disclosure annotation

- [ ] T025 [US4] Add Plugin Architecture Diagram to 04-skills-plugins-mcp.md:
  - Directory structure visualization (.claude-plugin/, commands/, agents/, skills/, hooks/, .mcp.json)
  - plugin.json manifest example with metadata fields
  - Component relationships clearly labeled

- [ ] T026 [US4] Update frontmatter in 04-skills-plugins-mcp.md:
  - `sidebar_position: 4`
  - `title: "Skills, Plugins, and MCP Integration"` (updated title)
  - `duration: "10-12 min"` (updated duration from 6-8 min)

- [ ] T027 [US4] Add Edge Cases section to 04-skills-plugins-mcp.md:
  - Confusion between Skills/Plugins/MCP (with clear hierarchy explanation)
  - Plugin installation failures (network issues, permissions, restart requirement)
  - Marketplace not accessible (check firewall, internet connection)
  - "No plugins available" message (marketplace needs to be added first)
```

### **Phase 13 (Technical Validation): ADD Plugin Testing**

**Add after T068**:
```markdown
- [ ] T068A Test plugin installation workflow in sandbox:
  - Run `/plugin` → verify marketplace browser appears
  - Run `/plugin marketplace add anthropics/claude-code` → verify success message
  - Run `/plugin install feature-dev` → verify download/install
  - Restart Claude Code → verify plugin activated
  - Run `/help` → verify new commands appear (e.g., `/feature-dev`)
  - Document any errors or issues encountered
```

### **Success Criteria Validation Table: UPDATE**

**Add rows**:
```markdown
| T019-T027 | SC-005, SC-006, SC-007, SC-008 | Eval 8 | Lesson 4 comprehensive with Skills/Plugins/MCP |
| T068A | SC-016 | Eval 8 | Plugin installation workflow validated |
```

---

## 🎯 Implementation Priority

1. **Update plan.md Lesson 4** - Expands content outline, adds 2 learning objectives, updates "Try With AI"
2. **Update tasks.md Phase 6** - Breaks Lesson 4 into 9 detailed tasks (T019-T027)
3. **Update tasks.md Phase 13** - Adds plugin installation sandbox testing (T068A)
4. **Update tasks.md Success Criteria table** - Maps new tasks to SC-005 to SC-008 and Eval 8

---

## ✅ Validation Checklist

After updates complete:
- [ ] plan.md Lesson 4 duration changed to 10-12 minutes
- [ ] plan.md Lesson 4 has 5 learning objectives (added Obj 4-5)
- [ ] plan.md Lesson 4 content outline has 4 sections (Agent Skills, Plugins, MCP, Hierarchy)
- [ ] plan.md Lesson 4 "Try With AI" has 4 prompts (with hands-on plugin installation)
- [ ] tasks.md Phase 6 has 9 tasks (T019-T027)
- [ ] tasks.md Phase 13 has plugin testing (T068A)
- [ ] tasks.md Success Criteria table updated with SC-005 to SC-008 mappings
- [ ] Total chapter duration updated from 55-75 min to 60-80 min

---

## 📊 Impact Summary

**spec.md**: ✅ COMPLETE (50 FRs, 25 SCs, 9 Evals, 10 acceptance scenarios for US4)
**plan.md**: ⏳ PENDING (Lesson 4 expansion from 6-8 min to 10-12 min, 4 content sections, 4 prompts)
**tasks.md**: ⏳ PENDING (Phase 6 expansion from 4 tasks to 9 tasks, Phase 13 add plugin testing)

**Next Action**: Execute updates to plan.md and tasks.md following this specification.
