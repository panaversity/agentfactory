# Chapter 6 Revision Specification: Google Gemini CLI - Open Source and Everywhere

**Version**: 1.0.0
**Created**: 2025-01-14
**Status**: Draft - Awaiting Approval
**Target Audience**: Aspiring developers (Part 2, Chapter 6, CEFR A2-B1)
**Complexity Level**: Beginner-Intermediate
**Verified Intelligence**: `intelligence/chapter-6-gemini-cli-verified-docs.md`

---

## 0. EVALS: Success Criteria (Defined FIRST)

### Learning Outcome Evals

**Eval 1: Installation & Verification (Pass/Fail)**
- **Measure**: Student successfully installs Gemini CLI and verifies with `gemini -v`
- **Business Goal**: Enable hands-on learning without technical blockers
- **Success Criteria**:
  - ✅ Student can run `npm install -g @google/gemini-cli` without errors
  - ✅ Student can verify installation with `gemini -v` showing version number
  - ✅ Student understands what "global installation" means in practical terms
- **Failure Mode**: Student stuck on Node.js prerequisites or permission errors
- **Validation**: Screenshot or terminal output showing version number

**Eval 2: Feature Discovery (Comprehension Test)**
- **Measure**: Student can explain WHEN to use Gemini CLI vs Claude Code for specific scenarios
- **Business Goal**: Strategic tool selection (not memorization)
- **Success Criteria**:
  - ✅ Student identifies 2+ scenarios where Gemini CLI is better (large context, free tier, custom integrations)
  - ✅ Student identifies 2+ scenarios where Claude Code is better (web interface, enterprise support)
  - ✅ Student asks AI to recommend tool for their specific project (meta-skill)
- **Failure Mode**: Student memorizes chart without understanding tradeoffs
- **Validation**: Open-ended question: "Which tool for analyzing 12 monthly reports? Why?"

**Eval 3: Configuration Management (Applied Skill)**
- **Measure**: Student can configure Gemini CLI using hierarchy and environment variables
- **Business Goal**: Professional environment setup (global vs project settings)
- **Success Criteria**:
  - ✅ Student understands global (`~/.gemini/settings.json`) vs project (`.gemini/settings.json`) distinction
  - ✅ Student can use `.env` files for secrets management
  - ✅ Student knows configuration precedence order
- **Failure Mode**: Student hardcodes secrets in settings.json (security risk)
- **Validation**: Successfully configured project-specific settings with .env file

**Eval 4: Memory & Context Management (Workflow Skill)**
- **Measure**: Student can manage context and use conversational branching
- **Business Goal**: Efficient multi-task workflows
- **Success Criteria**:
  - ✅ Student uses `/clear` and `/compress` appropriately
  - ✅ Student can save/resume conversations with `/chat` commands
  - ✅ Student understands GEMINI.md for long-term memory
- **Failure Mode**: Student runs out of context tokens, doesn't know how to recover
- **Validation**: Successfully demonstrates `/chat save` and `/chat resume` workflow

**Eval 5: Custom Commands Creation (Reusable Workflows)**
- **Measure**: Student can create custom slash commands with injection patterns
- **Business Goal**: Automate repetitive prompts, team standardization
- **Success Criteria**:
  - ✅ Student creates TOML file with `description` and `prompt` fields
  - ✅ Student uses `{{args}}`, `!{shell}`, or `@{file}` injection
  - ✅ Student understands global vs project command locations
- **Failure Mode**: Student copies prompts manually every time (inefficient)
- **Validation**: Successfully created and executed custom command

**Eval 6: MCP Integration (External Tools)**
- **Measure**: Student can add MCP server using CLI commands (NEW - not manual JSON editing)
- **Business Goal**: Modern workflow adoption (CLI-first, not config-file-first)
- **Success Criteria**:
  - ✅ Student uses `gemini mcp add` command successfully
  - ✅ Student verifies with `gemini mcp list` showing connected server
  - ✅ Student understands what MCP servers enable (external capabilities)
- **Failure Mode**: Student falls back to manual JSON editing (old workflow)
- **Validation**: Terminal output showing `gemini mcp list` with active server

**Eval 7: Extension Lifecycle (Development Awareness)**
- **Measure**: Student understands extension development workflow (create/link/update)
- **Business Goal**: Enable contribution to Gemini CLI ecosystem
- **Success Criteria**:
  - ✅ Student can explain difference between `install` (production) and `link` (development)
  - ✅ Student knows how to create extension from template (`gemini extensions new`)
  - ✅ Student can update extensions (`gemini extensions update --all`)
- **Failure Mode**: Student only knows installation, can't develop/contribute
- **Validation**: Quiz question on install vs link workflow

**Eval 8: IDE Integration (Workflow Improvement)**
- **Measure**: Student can enable IDE integration and explain benefits
- **Business Goal**: Professional workflow optimization
- **Success Criteria**:
  - ✅ Student enables IDE integration with `/ide enable`
  - ✅ Student can list 3 benefits (recent files, cursor position, native diffs)
  - ✅ Student understands when IDE integration adds value
- **Failure Mode**: Student skips IDE integration, misses productivity gains
- **Validation**: Successfully enabled IDE connection (status check)

### Engagement Evals

**Reading Completion Rate**: 75%+ complete chapter
- **Measure**: Analytics (if available) or self-reported completion
- **Business Goal**: Content engaging enough to finish

**AI Exercise Completion**: 80%+ attempt "Try With AI" prompts
- **Measure**: Self-reported or exercise submission
- **Business Goal**: Hands-on practice with AI collaboration

### Accessibility Evals

**Reading Level**: Grade 7-8 (A2-B1 CEFR)
- **Measure**: Flesch-Kincaid readability score
- **Business Goal**: Accessible to non-native English speakers
- **Validation**: Automated readability check

**Cognitive Load**: Max 7 concepts per section
- **Measure**: Manual concept counting per section
- **Business Goal**: Prevent overwhelm in beginner audience
- **Validation**: Review checklist

---

## 1. Problem Statement

**Current State**:
Chapter 6 exists with 5 lessons covering Gemini CLI basics. Content is pedagogically sound and constitutionally aligned.

**Gap Identified**:
Missing **8 critical feature areas** from Medium Tutorial Series and Context7 research:

**From Context7 Research** (5 gaps):
1. CLI MCP management commands (`gemini mcp add/list/remove`)
2. IDE integration (`/ide` commands + VS Code companion)
3. Extension lifecycle management (create/link/update/disable)
4. OAuth for MCP servers (`/mcp auth`)
5. Tool filtering for security (`includeTools`/`excludeTools`)

**From Medium Tutorial Series** (3 additional gaps):
6. Configuration hierarchy (Global vs Project settings, precedence order, .env files)
7. Custom slash commands (TOML syntax, injection patterns: `{{args}}`, `!{shell}`, `@{file}`)
8. Memory & context management (`/clear`, `/compress`, `/chat` branching, GEMINI.md)

**Business Impact**:
- Students learn outdated workflows (manual JSON editing vs CLI commands)
- Students miss major productivity features (IDE integration, custom commands, context management)
- Students can't configure environments properly (no understanding of hierarchy or .env files)
- Students run out of context tokens without knowing recovery strategies
- Students can't create reusable workflows (no custom command knowledge)
- Students can't contribute to ecosystem (no extension development knowledge)
- Students lack security awareness (tool filtering, secrets management)

**User Pain**:
"I followed the chapter but Gemini CLI has features I didn't learn about. My context keeps filling up, I'm manually editing JSON files, and I keep retyping the same prompts. Am I missing something?"

---

## 2. Objectives

### Primary Objectives

1. **Update Chapter 6 with Verified Intelligence**
   - Replace all tool-specific claims with verified facts from `intelligence/chapter-6-gemini-cli-verified-docs.md`
   - Ensure 100% technical accuracy (75 verified claims: 47 from Context7 + 28 from Medium Tutorial)
   - Source citations for all configuration examples

2. **Integrate 8 Missing Critical Feature Areas**
   - **From Context7**: CLI MCP management, IDE integration, extension lifecycle, OAuth, tool filtering
   - **From Medium Tutorial**: Configuration hierarchy, custom slash commands, memory/context management

3. **Restructure Chapter for Logical Learning Flow**
   - **Foundation Tier** (Lessons 1-3): Tool understanding
   - **Configuration Tier** (Lessons 4-5): Environment customization
   - **Extension Tier** (Lessons 6-8): Capability expansion
   - **Progressive Complexity**: Beginner → Intermediate → Advanced within chapter

4. **Maintain Constitutional Alignment**
   - Principle 7: Technical Accuracy and Currency (verified claims only)
   - Principle 13: Graduated Teaching (Foundation → Configuration → Extension)
   - Core Philosophy #1: AI Development Spectrum (teach WHEN to use AI vs direct commands)
   - Cognitive Load Management: ≤7 concepts per section (all lessons validated)

5. **Preserve Existing Pedagogical Quality**
   - Keep strong "Try With AI" prompts (Chapter 1 format - 3-4 prompts per lesson)
   - Maintain reading accessibility (Grade 7-8 level)
   - Clear section headings for scannability

### Secondary Objectives

6. **Strategic AI Usage Throughout**
   - **Tier 1**: Direct commands (installation, CLI operations, 1-5 seconds)
   - **Tier 2**: AI companion (understanding concepts, troubleshooting, strategic decisions)
   - **Tier 3**: AI orchestration (complex multi-step workflows, extension design)
   - Avoid over-engineering simple tasks with AI

7. **Realistic Durations**
   - Installation: 1-2 minutes (not inflated)
   - Authentication: 2-3 minutes (browser OAuth flow)
   - MCP setup: 5-10 minutes (CLI commands, not 30+ minutes)
   - Custom command creation: 3-5 minutes (TOML file + test)
   - Total chapter: 137-150 minutes (2h 17min - 2h 30min)

---

## 3. Scope

### In Scope

**Content Updates** (8 Lessons Total - Restructured for Logical Flow):

**Foundation Tier** (Lessons 1-3):
- ✅ Lesson 1: Why Gemini CLI Matters (15 min)
  - Positioning, comparison with ChatGPT/Claude Code/Qwen Code
  - Free tier quotas, context window advantages
  - Use case scenarios

- ✅ Lesson 2: Installation, Authentication & First Steps (15 min)
  - Installation verification
  - OAuth authentication flow
  - Basic session commands
  - Troubleshooting

- ✅ Lesson 3: Built-In Tools Deep Dive (20-25 min)
  - File operations, shell integration, web fetching, search grounding
  - Tool combination strategies
  - Limitations and error literacy

**Configuration Tier** (Lessons 4-5):
- ✅ Lesson 4: **NEW** - Configuration & Settings (15-17 min)
  - Part 1: Global vs Project settings hierarchy
  - Part 2: Common configuration options (theme, model, checkpointing)
  - Part 3: Environment variables (.env files, ${VAR} syntax)
  - Part 4: Context window configuration
  - Part 5: Security best practices

- ✅ Lesson 5: **NEW** - Memory & Context Management (18-20 min)
  - Part 1: Understanding context (1M tokens, construction)
  - Part 2: Context management (`/clear`, `/compress`)
  - Part 3: Conversational branching (`/chat` commands)
  - Part 4: Long-term memory (GEMINI.md hierarchy)
  - Part 5: Memory commands (`/memory` commands)

**Extension Tier** (Lessons 6-8):
- ✅ Lesson 6: **NEW** - Custom Slash Commands (16-18 min)
  - Part 1: What are custom commands?
  - Part 2: TOML file structure
  - Part 3: Injection patterns (`{{args}}`, `!{shell}`, `@{file}`)
  - Part 4: Namespacing and organization
  - Part 5: Common use cases

- ✅ Lesson 7: MCP Servers & Integration (20 min)
  - Part 1: Understanding MCP
  - Part 2: CLI MCP management (`gemini mcp add/list/remove`)
  - Part 3: OAuth for MCP servers (`/mcp auth`)
  - Part 4: Business workflows

- ✅ Lesson 8: **NEW** - Extensions, Security & IDE Integration (18-20 min)
  - Part 1: Extension development workflow
  - Part 2: Tool filtering for security
  - Part 3: IDE integration
  - Part 4: Choosing the right workflow

**Total Duration**: 137-150 minutes (2h 17min - 2h 30min)

**Key Structural Changes from Original Spec**:
- **Was**: 6 lessons (110-135 min)
- **Now**: 8 lessons (137-150 min)
- **Added**: Lesson 4 (Configuration), Lesson 5 (Memory), Lesson 6 (Custom Commands)
- **Logical Flow**: Foundation → Configuration → Extension (beginner to expert)

**Verified Examples**:
- All JSON configurations from verified intelligence cache
- All CLI commands tested against official docs
- All feature claims sourced from Context7

### Out of Scope

**NOT Changing**:
- ❌ Learning objectives format (keep existing structure)
- ❌ Pedagogical approach (keep "Try With AI" format from Chapter 1)
- ❌ Audience tier (still A2-B1 beginner-intermediate)
- ❌ Part 2 designation (still "AI Tool Landscape")

**NOT Adding**:
- ❌ Advanced enterprise features (service account impersonation - too complex)
- ❌ Qwen Code CLI deep dive (brief mention only, focus on Gemini)
- ❌ MCP server development (writing custom servers - separate chapter)
- ❌ Playwright/Context7 deep tutorials (examples only)

**Explicitly Excluded**:
- ❌ Over-engineering simple tasks with AI (e.g., "Ask AI to run `gemini mcp add`")
- ❌ Verbose explanations of trivial operations
- ❌ 8+ "Try With AI" prompts (keep 3-4 focused ones per lesson)

---

## 4. AI Usage Strategy (Constitutional Requirement)

### When Students Use AI (Strategic, Not Everything)

**Tier 1: Direct Commands** (Students run directly, 1-5 seconds):
```bash
npm install -g @google/gemini-cli    # 30-60 seconds
gemini -v                             # 1 second
gemini                                # 2 seconds (launches session)
gemini mcp add my-server python server.py   # 3-5 seconds
gemini mcp list                       # 1 second
gemini extensions list                # 1 second
/ide enable                          # 2 seconds
```
**Why Direct**: Simple, deterministic commands. Typing is faster than prompting AI.

**Tier 2: AI Companion** (Students use AI for understanding/troubleshooting):
- Understanding MCP architecture ("What is Model Context Protocol?")
- Troubleshooting errors ("I got 'npm: command not found' - what does this mean?")
- Explaining configuration ("What does `timeout: 15000` do in this JSON?")
- Strategic decisions ("Should I use Gemini CLI or Claude Code for analyzing 12 reports?")
- Interpreting OAuth flow ("Why did a browser window open during authentication?")

**Tier 3: AI Orchestration** (Complex multi-step workflows):
- Designing custom extension from scratch (AI helps with structure)
- Complex MCP server integration (AI suggests configuration for specific tools)
- Multi-tool workflow design (AI plans sequence: install → configure → verify → use)

### "Try With AI" Prompts Focus

**Chapter 1 Clean Format** (3-4 focused prompts per lesson):
```markdown
### Prompt 1: Title
<backticks>
Specific, actionable prompt
</backticks>

**Expected outcome**: What student should receive
```

**NOT** 8+ verbose prompts with pre-explanations. Keep it clean and scannable.

### Anti-Pattern Prevention

**DON'T**:
- ❌ "Ask AI to install Gemini CLI" (students run `npm install` directly)
- ❌ "Tell AI to add MCP server" (students run `gemini mcp add` directly)
- ❌ "Request AI to list extensions" (students run `gemini extensions list` directly)
- ❌ 50-minute lessons for 5-minute operations (realistic durations)

**DO**:
- ✅ "If installation fails, ask AI: 'I got error X, what does it mean?'"
- ✅ "Ask AI: 'Should I use Gemini CLI or Claude Code for my project [describe]?'"
- ✅ "Ask AI: 'Explain what MCP servers enable and when I'd need them'"

---

## 5. Design Decisions

### Decision 1: Expand Lesson 5 vs Add Lesson 6

**Options**:
- A) Expand existing Lesson 5 with new features
- B) Add new Lesson 6 for advanced features

**Chosen**: **Option A** (Expand Lesson 5)

**Rationale**:
- Keeps chapter scope manageable (5 lessons, not 6)
- New features are natural extensions of MCP/Extensions topic
- Prevents cognitive overload from too many lessons
- Constitutional alignment: Progressive complexity within lessons

**Tradeoffs**:
- Lesson 5 becomes longer (~30-35 minutes vs 20-25 minutes currently)
- Mitigated by: Clear subsection headings, scannable structure

### Decision 2: CLI Commands vs Manual JSON Editing

**Old Workflow** (manual JSON editing):
```json
// Edit ~/.gemini/settings.json
{
  "mcpServers": {
    "my-server": {
      "command": "python",
      "args": ["server.py"]
    }
  }
}
```

**New Workflow** (CLI commands - PREFERRED):
```bash
gemini mcp add my-server python server.py
gemini mcp list
```

**Chosen**: **Teach CLI commands FIRST, mention JSON as alternative**

**Rationale**:
- CLI commands are beginner-friendly (less error-prone)
- Modern workflow (matches industry direction)
- Easier to verify success (`gemini mcp list` shows status)
- Constitutional alignment: Teach current best practices

**Implementation**:
- Primary examples use CLI commands
- Brief note: "Advanced: You can also edit `~/.gemini/settings.json` manually"

### Decision 3: IDE Integration Placement

**Options**:
- A) Add IDE integration to Lesson 5 (MCP & Extensions)
- B) Create separate Lesson 6 for IDE workflows
- C) Add IDE integration to Lesson 2 (Installation)

**Chosen**: **Option A** (Add to Lesson 5 as final section)

**Rationale**:
- IDE integration is an "extension" of capabilities (thematic fit)
- Lesson 2 is already complete (installation + auth + first steps)
- Keeps chapter cohesive (all "extending Gemini CLI" in one lesson)

**Structure**:
```
Lesson 5: MCP and Extensions Ecosystem
├── Part 1: Understanding MCP
├── Part 2: Adding MCP Servers (CLI commands)
├── Part 3: Extensions Management
├── Part 4: IDE Integration (NEW)
└── Try With AI (4 prompts)
```

### Decision 4: OAuth for MCP - Beginner vs Advanced

**Challenge**: OAuth authentication is complex for beginners

**Chosen**: **Simplified explanation with trust-based approach**

**Implementation**:
- Explain WHAT OAuth does (secure authentication for protected APIs)
- Show `/mcp auth` command (simple usage)
- Explain automatic token management (students don't manage tokens manually)
- Skip OAuth internals (authorization flows, token refresh - too advanced)

**Rationale**:
- A2-B1 audience needs practical understanding, not protocol details
- Constitutional alignment: Cognitive load limits (max 7 concepts)
- Students learn "how to use OAuth-protected servers" not "how OAuth works"

### Decision 5: Tool Filtering - Security Education

**Challenge**: Security concepts can be abstract for beginners

**Chosen**: **Real-world scenario-based teaching**

**Example**:
```markdown
### Why Tool Filtering Matters

**Scenario**: You add an MCP server from a public GitHub repository.
It includes tools: `read_file`, `write_file`, `delete_file`.

**Without filtering**: All 3 tools available. AI could accidentally delete files.

**With filtering**:
```json
{
  "includeTools": ["read_file"],  // Only allow reading
  "excludeTools": ["delete_file"]  // Block deletion
}
```

**Result**: AI can read files for analysis but cannot delete them.
```

**Rationale**:
- Concrete example > abstract security lecture
- Students understand WHY filtering matters (not just HOW)
- Constitutional alignment: Learning by building real scenarios

---

## 6. Technical Requirements

### Verified Intelligence Requirements

**MUST USE**:
- All examples from `intelligence/chapter-6-gemini-cli-verified-docs.md`
- Verified CLI commands (tested against official docs)
- Verified JSON configurations (from Context7 sources)

**FORBIDDEN**:
- Assumed commands (not verified)
- Placeholder configurations (use real examples from verified cache)
- Outdated syntax (verify against 2025 docs)

### Example Verification Checklist

For every code example:
- [ ] Command verified in `intelligence/chapter-6-gemini-cli-verified-docs.md`
- [ ] JSON configuration matches verified structure
- [ ] Expected output documented
- [ ] Source citation included (Context7 library ID)

### Platform Requirements

**Cross-Platform Testing**:
- Commands work on Windows (PowerShell/Command Prompt)
- Commands work on macOS (Terminal)
- Commands work on Linux (Bash)

**File Path Examples**:
- Windows: `C:\Users\username\.gemini\settings.json`
- macOS/Linux: `~/.gemini/settings.json`

---

## 7. Content Structure (Lesson-by-Lesson)

### Lesson 1: Why Gemini CLI Matters (MINOR UPDATES)

**Changes**:
- Update "Game-Changing Differences" with IDE integration mention
- Verify free tier quotas (60 req/min, 1000 req/day - CONFIRMED)
- Update comparison table with IDE integration row

**Duration**: 15 minutes (unchanged)
**Estimated Reading Time**: 12-15 minutes
**Try With AI**: 4 prompts (unchanged)

### Lesson 2: Installation, Authentication & First Steps (MINOR UPDATES)

**Changes**:
- Add troubleshooting subsection for common errors
  - "npm: command not found" → Install Node.js
  - "Permission denied" → Use sudo (macOS/Linux) or admin PowerShell (Windows)
  - "Module not found" → Clear npm cache, reinstall
- Verify installation command: `npm install -g @google/gemini-cli` (CONFIRMED)
- Update session commands list with `/ide` and `/mcp auth`

**Duration**: 15 minutes (unchanged)
**Estimated Reading Time**: 12-15 minutes
**Try With AI**: 4 prompts (unchanged)

### Lesson 3: Built-In Tools Deep Dive (VERIFICATION ONLY)

**Changes**:
- Cross-reference all tool examples with verified intelligence
- Confirm file types supported (CSV, JSON, PDF, text, XML, Markdown)
- Verify web fetching capabilities
- Confirm Google Search grounding features

**Duration**: 20-25 minutes (unchanged)
**Estimated Reading Time**: 18-22 minutes
**Try With AI**: 4 prompts (unchanged)

### Lesson 4: Context Window and Tool Comparison (MINOR UPDATES)

**Changes**:
- Verify token counts (1M tokens = ~750,000 words = ~100,000 lines of code)
- Update comparison table with verified numbers
- Add IDE integration row to comparison

**Duration**: 25 minutes (unchanged)
**Estimated Reading Time**: 20-25 minutes
**Try With AI**: 4 prompts (unchanged)

### Lesson 5: MCP Servers and Integration (REVISED & EXPANDED)

**Current Structure** (4 parts):
- Part 1: Understanding MCP
- Part 2: Adding MCP Servers (manual JSON editing)
- Part 3: Business Workflows Using MCP
- Part 4: Extensions (brief intro)

**NEW Structure** (4 parts - MCP FOCUS):
- Part 1: Understanding MCP (unchanged - verify examples)
- **Part 2: CLI MCP Management** (NEW - 7 minutes)
  - `gemini mcp add` for stdio/HTTP/SSE servers
  - `gemini mcp list` status checking
  - `gemini mcp remove` cleanup
  - CLI vs manual JSON editing comparison
  - When to use each approach
- **Part 3: OAuth for MCP Servers** (NEW - 5 minutes)
  - `/mcp auth` command (high-level)
  - Automatic token management
  - Use case: Accessing secured APIs
  - Browser-based authentication flow
- Part 4: Business Workflows Using MCP (unchanged - verify examples)
  - Multi-competitor research (Playwright)
  - API documentation (Context7)
  - Tool evaluation workflows
- Try With AI: 4 prompts (focused on MCP workflows)

**New Duration**: 20 minutes (was 30 minutes)
**Estimated Reading Time**: 18-20 minutes
**Try With AI**: 4 prompts (MCP-focused)

**Cognitive Load Check**:
- Part 1: 5 concepts (MCP basics) ✓
- Part 2: 6 concepts (CLI commands + comparison) ✓
- Part 3: 4 concepts (OAuth high-level) ✓
- Part 4: 6 concepts (business workflows) ✓

All parts within 7 concept limit ✅

---

### Lesson 6: Extensions, Security & IDE Integration (NEW LESSON)

**Structure** (4 parts - EXTENSIONS FOCUS):
- **Part 1: Extension Development Workflow** (NEW - 6 minutes)
  - `gemini extensions new` (create from templates)
  - `gemini extensions link` (development mode)
  - `install` vs `link` distinction
  - `gemini extensions update` (maintenance)
  - `enable`/`disable` lifecycle management
  - Extension manifest structure (`gemini-extension.json`)
- **Part 2: Tool Filtering for Security** (NEW - 5 minutes)
  - `includeTools` (allowlist approach)
  - `excludeTools` (blocklist approach)
  - Real-world security scenario (MCP server with dangerous tools)
  - Best practices for external tool integration
- **Part 3: IDE Integration** (NEW - 6 minutes)
  - `/ide install` command (VS Code companion)
  - `/ide enable` connection
  - Benefits: 10 recent files, cursor position, native diffs
  - VS Code workflow demonstration
  - When IDE integration adds value vs terminal-only
- Part 4: Choosing the Right Workflow (NEW - 3 minutes)
  - Decision framework: When to use MCP vs Extensions vs IDE
  - Combining multiple capabilities
  - Professional development setup examples
- Try With AI: 4 prompts (focused on extensions + IDE workflows)

**Duration**: 15-20 minutes
**Estimated Reading Time**: 15-18 minutes
**Try With AI**: 4 prompts (extensions + IDE focused)

**Cognitive Load Check**:
- Part 1: 6 concepts (extension lifecycle) ✓
- Part 2: 4 concepts (security filtering) ✓
- Part 3: 5 concepts (IDE integration) ✓
- Part 4: 3 concepts (decision framework) ✓

All parts within 7 concept limit ✅

---

## 8. Quality Standards

### Constitutional Compliance Checklist

**Principle 7: Technical Accuracy and Currency**:
- [ ] All commands verified in `intelligence/chapter-6-gemini-cli-verified-docs.md`
- [ ] All JSON configurations sourced from verified intelligence
- [ ] All free tier quotas confirmed (60/min, 1000/day)
- [ ] All tool capabilities verified against official docs
- [ ] Source citations for all technical claims

**Principle 13: Graduated Teaching**:
- [ ] Tier 1 (Direct Commands): Installation, CLI operations documented
- [ ] Tier 2 (AI Companion): Understanding, troubleshooting, strategic decisions
- [ ] Tier 3 (AI Orchestration): Complex workflows, multi-step integration
- [ ] Clear distinction when to use each tier

**Core Philosophy #1: AI Development Spectrum**:
- [ ] Assisted techniques (simple CLI commands) taught directly
- [ ] Driven methodology (AI for understanding) demonstrated
- [ ] Native patterns (when appropriate) introduced

**Principle 12: Cognitive Load Consciousness**:
- [ ] Max 7 concepts per section (all sections checked)
- [ ] Reading level: Grade 7-8 (Flesch-Kincaid score)
- [ ] Progressive complexity within lessons

**Principle 8: Accessibility and Inclusivity**:
- [ ] No gatekeeping language ("obviously", "simply", "just")
- [ ] Platform-specific instructions (Windows/Mac/Linux)
- [ ] Diverse example contexts
- [ ] Error literacy section ("Red Flags to Watch")

### Content Quality Checklist

**Try With AI Format** (Chapter 1 Clean Format):
- [ ] 3-4 focused prompts per lesson (not 8+)
- [ ] Clean structure: `### Prompt N: Title` → code block → `**Expected outcome:**`
- [ ] No verbose pre-explanations
- [ ] Actionable, specific prompts

**Realistic Durations**:
- [ ] Installation: 1-2 minutes (not 30+)
- [ ] MCP setup: 5-10 minutes (CLI commands, not 45+)
- [ ] Lesson reading: Matches actual content length

**Anti-Pattern Prevention**:
- [ ] No "Ask AI to run simple commands"
- [ ] No over-engineering trivial operations
- [ ] No inflated time estimates
- [ ] Strategic AI use only

---

## 9. Validation Strategy

### Pre-Implementation Validation

**Spec Review**:
- [ ] Human approves this specification
- [ ] Verified intelligence cache reviewed
- [ ] Gap coverage plan approved
- [ ] AI usage strategy validated

### Post-Implementation Validation

**Technical Accuracy**:
- [ ] All CLI commands tested in sandbox
  - `gemini mcp add` with stdio/HTTP/SSE transports
  - `gemini extensions new/link/update`
  - `/ide install` and `/ide enable`
  - OAuth flow with `/mcp auth`
- [ ] All JSON configurations validated against schema
- [ ] All "Try With AI" prompts tested for achievability

**Constitutional Alignment**:
- [ ] Invoke `validation-auditor` subagent
- [ ] Check against constitution v3.1.3
- [ ] Verify graduated teaching pattern
- [ ] Confirm cognitive load limits

**Pedagogical Quality**:
- [ ] Reading level check (Flesch-Kincaid)
- [ ] Concept counting per section
- [ ] "Try With AI" format validation
- [ ] Duration realism check

**Sandbox Testing** (CRITICAL):
- [ ] Install Gemini CLI in fresh environment
- [ ] Test every command from lessons
- [ ] Verify OAuth flow (if test server available)
- [ ] Test IDE integration (VS Code)
- [ ] Document actual output vs lesson claims

**Create**: `SANDBOX-AUDIT-REPORT.md` with:
- Commands tested (with actual output)
- Errors found (with line numbers in lessons)
- Fixes applied (with evidence)
- Re-test results (verification)

### Evals Validation

**Eval 1 Test**: Can student install and verify?
- [ ] Fresh install tested
- [ ] Common errors documented with fixes
- [ ] Verification command output shown

**Eval 2 Test**: Can student explain tool selection?
- [ ] Quiz question written for tool comparison
- [ ] Scenario-based questions prepared
- [ ] Answer key with reasoning

**Eval 3 Test**: Can student use CLI MCP commands?
- [ ] Step-by-step tested: add → list → remove
- [ ] Terminal output screenshots captured
- [ ] Success criteria clearly defined

**Eval 4 Test**: Does student understand extension workflow?
- [ ] Quiz question on install vs link
- [ ] Development workflow explained clearly
- [ ] Common pitfalls documented

**Eval 5 Test**: Can student enable IDE integration?
- [ ] `/ide enable` tested in VS Code
- [ ] Benefits clearly demonstrated
- [ ] Status check documented

---

## 10. Success Metrics

### Completion Metrics

**Content Complete**:
- [ ] 5 lessons revised/expanded
- [ ] 5 critical features integrated
- [ ] All examples verified
- [ ] All "Try With AI" prompts tested

**Quality Gates Passed**:
- [ ] Technical accuracy: 100% (all claims verified)
- [ ] Constitutional alignment: Pass (validation-auditor validation)
- [ ] Sandbox testing: Pass (all commands work)
- [ ] Reading level: Grade 7-8 (Flesch-Kincaid)
- [ ] Cognitive load: ≤7 concepts per section

**Line Count**:
- [ ] Lesson 5 expansion: ~500-700 lines added (for 5 new sections)
- [ ] Total chapter: ~3,000-3,500 lines (reasonable for 5 lessons)
- [ ] No unnecessary verbosity

### Learning Outcome Metrics

**Student Can**:
- [ ] Install Gemini CLI successfully (Eval 1)
- [ ] Explain tool selection strategically (Eval 2)
- [ ] Use CLI MCP commands (Eval 3)
- [ ] Understand extension development (Eval 4)
- [ ] Enable IDE integration (Eval 5)

**Student Understands**:
- [ ] WHEN to use AI vs direct commands
- [ ] WHY Gemini CLI matters in tool landscape
- [ ] HOW to extend capabilities with MCP/Extensions
- [ ] SECURITY implications of external tool connections

---

## 11. Risks and Mitigations

### Risk 1: Lesson 5 Too Long (Cognitive Overload)

**Risk**: Expanding Lesson 5 from 20-25 min to 30-35 min may overwhelm beginners

**Mitigation**:
- Clear subsection headings for scannability
- Each new part is 3-5 minutes (digestible chunks)
- "Try With AI" prompts provide practice breaks
- Students can skip advanced topics (OAuth, tool filtering) on first read

**Fallback**: If beta readers report overwhelm, split Lesson 5 into Lessons 5A and 5B

### Risk 2: CLI Commands Don't Work Cross-Platform

**Risk**: `gemini mcp add` syntax differs on Windows vs macOS/Linux

**Mitigation**:
- Test all commands on Windows (PowerShell + Command Prompt), macOS, Linux
- Document platform-specific variations
- Provide troubleshooting for common platform issues

**Sandbox Testing Required**: MUST test on all 3 platforms before publication

### Risk 3: OAuth Flow Requires External Services

**Risk**: Testing OAuth (`/mcp auth`) requires actual OAuth-protected MCP server

**Mitigation**:
- Use publicly available OAuth test servers (if available)
- Document OAuth conceptually with verified examples
- Note: "OAuth testing requires secured MCP server (optional for beginners)"
- Focus on `/mcp auth` command usage, not full OAuth implementation

**Acceptable**: Conceptual understanding > hands-on OAuth testing for A2-B1 audience

### Risk 4: Features Change Before Publication

**Risk**: Gemini CLI updates may change commands/syntax between revision and publication

**Mitigation**:
- Document current version in spec (e.g., "as of Jan 2025, version 0.4.0+")
- Add update trigger note: "Review quarterly or when major version released"
- Verified intelligence cache timestamped (easy to re-verify if needed)

**Monitoring**: Check Gemini CLI changelog before publication

### Risk 5: Students Skip "Try With AI" Prompts

**Risk**: Students read passively, don't practice with AI

**Mitigation**:
- Make prompts actionable and immediately useful
- Show "Expected outcome" to motivate trying
- Keep prompts focused (not verbose - lower barrier to trying)

**Measurement**: Track self-reported completion of AI exercises (if possible)

---

## 12. Dependencies

### External Dependencies

**Required Before Implementation**:
- [x] Verified intelligence cache (`intelligence/chapter-6-gemini-cli-verified-docs.md`)
- [x] Constitution v3.1.3 reference
- [x] Chapter index (Part 2, Chapter 6 location confirmed)
- [ ] Human approval of this specification

**Optional (Nice to Have)**:
- [ ] Beta reader feedback on Lesson 5 length
- [ ] OAuth test server for hands-on validation
- [ ] Analytics on current chapter completion rates (baseline)

### Internal Dependencies

**Subagents Required**:
- `content-implementer` - Execute content revision
- `validation-auditor` - Validate constitutional alignment
- `factual-verifier` - Final quality gate

**Skills Required**:
- `learning-objectives` - Measurable outcomes aligned with evals
- `concept-scaffolding` - Break new features into learnable chunks
- `code-example-generator` - Verified examples from intelligence cache
- `technical-clarity` - Ensure accessibility for A2-B1 audience
- `ai-collaborate-teaching` - Design "Try With AI" prompts
- `content-evaluation-framework` - Quality rubrics

**Workflow Dependencies**:
1. This spec approved → 2. Plan created → 3. Tasks generated → 4. Implementation → 5. Validation

---

## 13. Timeline Estimate

**Phase 1: Specification** (This Document)
- Time: 2 hours
- Status: Complete (awaiting approval)

**Phase 2: Planning**
- Time: 1 hour
- Agent: `chapter-planner`
- Output: `plan.md`, `tasks.md`

**Phase 3: Implementation**
- Time: 4-6 hours
- Agent: `content-implementer`
- Output: Revised lesson files

**Phase 4: Validation**
- Time: 2-3 hours
- Agents: `validation-auditor`, `factual-verifier`
- Sandbox testing: 1 hour
- Output: Validation reports, fixes

**Phase 5: Finalization**
- Time: 30 minutes
- Update chapter index
- Create PHR
- Final human review

**Total Estimated Time**: 10-13 hours (AI-driven workflow)

---

## 14. Acceptance Criteria

### This Spec is Complete When:

- [x] Evals defined (5 learning outcome evals + 2 engagement evals + 2 accessibility evals)
- [x] AI usage strategy documented (Tier 1/2/3 with examples)
- [x] 5 critical features scoped for integration
- [x] Lesson-by-lesson structure defined
- [x] Verified intelligence referenced for all technical claims
- [x] Quality standards checklist created
- [x] Validation strategy defined
- [x] Risks identified with mitigations
- [ ] **Human approval obtained**

### Implementation is Complete When:

- [ ] All 5 lessons revised/verified
- [ ] 5 critical features integrated (CLI MCP, OAuth, extension lifecycle, tool filtering, IDE integration)
- [ ] All examples from verified intelligence cache
- [ ] All "Try With AI" prompts tested
- [ ] Technical-reviewer validation: PASS
- [ ] Sandbox testing: PASS (all commands work)
- [ ] Reading level: Grade 7-8
- [ ] Cognitive load: ≤7 concepts per section
- [ ] Line count reasonable (~3,000-3,500 total)

### Chapter is Publication-Ready When:

- [ ] All evals validated (students can achieve success criteria)
- [ ] Beta reader feedback incorporated (if available)
- [ ] Docusaurus build succeeds
- [ ] No broken links or missing images
- [ ] Cross-platform testing complete (Windows/Mac/Linux)
- [ ] Human final review approved

---

## 15. Open Questions (For Human Decision)

### Question 1: Lesson 5 Length Acceptable?

**Current**: 20-25 minutes
**Proposed**: 30-35 minutes (5 new sections added)

**Options**:
- A) Accept 30-35 minute lesson (scannable with clear subsections)
- B) Split into Lesson 5A and Lesson 5B (creates 6-lesson chapter)

**Recommendation**: **Option A** (single longer lesson), but defer to human judgment

**Your Decision**: ✅ **APPROVED - Split into Lessons 5 & 6 logically**
- Lesson 5: MCP Servers and Integration (20 min)
- Lesson 6: Extensions, Security & IDE Integration (15 min)

### Question 2: OAuth Deep Dive or High-Level?

**Current Plan**: High-level (show `/mcp auth` command, explain automatic token management)

**Alternative**: Deep OAuth explanation (authorization flows, token refresh, scope management)

**Recommendation**: **High-level** (A2-B1 audience, avoid complexity), but confirm

**Your Decision**: ✅ **APPROVED - High-level only**
- Show `/mcp auth` command usage
- Explain automatic token management (students don't manage manually)
- Skip OAuth protocol internals (too complex for A2-B1)

### Question 3: Qwen Code CLI Coverage?

**Current Plan**: Brief mention only (1 paragraph as alternative in Lesson 1)

**Alternative**: Dedicated section comparing Gemini CLI vs Qwen Code CLI

**Recommendation**: **Brief mention** (keep focus on Gemini CLI), but confirm

**Your Decision**: ✅ **APPROVED - Brief mention (1 paragraph)**
- Include in Lesson 1 "Open Source Ecosystem Effect" section
- Explain Qwen Code as fork with 2,000 req/day free tier
- Keep focus on Gemini CLI for hands-on learning

---

## 16. Next Steps

**Upon Approval**:

1. **Phase 2: Planning** (invoke `chapter-planner` subagent)
   - Create detailed lesson-by-lesson plan
   - Generate task checklist
   - Map verified examples to sections

2. **Phase 3: Implementation** (invoke `content-implementer` subagent)
   - Revise Lessons 1-4 (minor updates + verification)
   - Expand Lesson 5 (5 new sections)
   - Apply all verified intelligence
   - Create "Try With AI" prompts

3. **Phase 4: Validation** (invoke reviewers)
   - Technical-reviewer: Constitutional alignment
   - Proof-validator: Final quality gate
   - Sandbox testing: All commands
   - Create audit report

4. **Phase 5: Finalization**
   - Update chapter index status
   - Create PHR documenting revision
   - Human final review

**Awaiting**: Human approval to proceed to Phase 2 (Planning)

---

**Specification Status**: ✅ COMPLETE - Ready for Review and Approval

**Key Decision Points Requiring Human Input**:
1. Lesson 5 length (30-35 min acceptable?)
2. OAuth depth (high-level vs detailed?)
3. Qwen Code coverage (brief vs dedicated?)

**Next Action**: Human reviews and approves this spec (or provides feedback for iteration)
