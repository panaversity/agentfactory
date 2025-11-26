### Core Concept
Brownfield adoption (adding Spec-Kit Plus to existing projects) requires evidence-based backup strategy and intelligent merge planning because `init --here` COMPLETELY overwrites CLAUDE.md—not automatically merged, requiring manual recovery and categorization.

### Key Mental Models
- **Experimental branch isolation**: Keeps main branch untouched so rollback is git checkout (zero risk exposure to team)
- **Backup redundancy**: Three recovery methods (git history, backup file, branch rollback) means single failure doesn't cause data loss
- **Content categorization**: Standards/architecture → constitution.md (project rules); AI patterns → CLAUDE.md (behavioral instructions)
- **Risk assessment matrix**: Content volume + refinement time → determine backup strategy (LOW risk = standard workflow; VERY HIGH = manual integration safer than init)

### Critical Patterns
- **Evidence before action**: File inspection reveals exactly what overwrites (CLAUDE.md yes, custom commands no, source code untouched)
- **Safety checklist**: Pre-flight validation (git exists, branches available, time allocated, merge complexity estimated) before running init
- **File preservation zones**: `.claude/commands/` safe; `.specify/` new; CLAUDE.md overwritten; source code untouched (understanding boundaries prevents surprises)
- **Merge strategy by risk**: LOW (backup sufficient) → MODERATE (git + file backups) → HIGH (team review first) → VERY HIGH (don't use init --here)

### AI Collaboration Keys
- AI helps design safety workflow (redundant recovery methods)
- AI adapts merge guidance to your specific content categories
- Convergence toward solution: You provide constraints → AI suggests approach → You refine → Merged result matches neither starting proposal

### Common Mistakes
- **No-backup YOLO**: Running init without branch/backup/commit (one typo = permanent 800-line knowledge loss)
- **Misunderstanding "merge"**: Warning says "merge with existing content" (true at directory level) but CLAUDE.md is OVERWRITTEN not merged (false at file level)
- **Assuming git exists**: No git = no recovery method besides manual backup (critical blocker)
- **Time pressure shortcuts**: "30-min demo needed, skip safety workflow" → demo fails, turns into data recovery crisis

### Connections
- **Builds on**: Understanding Spec-Kit Plus structure (chapters 13-14) before adopting it in real projects
- **Leads to**: Capability to safely integrate new workflows without destroying institutional knowledge
- **Critical for**: Consulting work where client CLAUDE.md contains 6-12 months irreplaceable team knowledge
