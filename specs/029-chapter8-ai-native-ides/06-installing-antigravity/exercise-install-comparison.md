# Exercise: Comparative Analysis - Antigravity vs Zed vs Cursor Installation

## Objective

Deepen your understanding of the three IDEs by comparing their installation experiences, configuration approaches, and architectural philosophies.

**Time estimate**: 10-15 minutes (reflection and comparison)

**Success criteria**: Complete structured comparison identifying 3+ architectural differences between IDEs, document which IDE suits different scenarios

---

## Context: You've Now Installed All Three IDEs

By completing Lessons 2-6, you've installed and explored:
1. **Lesson 2**: Zed IDE (fast, native, Anthropic-backed)
2. **Lesson 4**: Cursor IDE (VS Code evolution, AI-first)
3. **Lesson 6**: Antigravity IDE (agent-first architecture)

This exercise asks you to reflect on what you experienced and compare the three approaches.

---

## Part 1: Installation Experience Comparison

### Guided Comparison

Fill in the following table based on your installation experience:

| Aspect | Zed | Cursor | Antigravity |
|--------|-----|--------|-------------|
| **Download time** | [Your observation] | [Your observation] | [Your observation] |
| **Installation steps** | [Count the steps] | [Count the steps] | [Count the steps] |
| **Configuration complexity** | Simple / Moderate / Complex | Simple / Moderate / Complex | Simple / Moderate / Complex |
| **API key entry** | Easy / Confusing / Clear | Easy / Confusing / Clear | Easy / Confusing / Clear |
| **Time from download to working** | [Minutes] | [Minutes] | [Minutes] |
| **First error encountered** | [List] | [List] | [List] |
| **Documentation quality** | Good / OK / Needs work | Good / OK / Needs work | Good / OK / Needs work |

### Reflection Questions

After filling the table, answer:

1. **Which IDE was easiest to install?** Why do you think that was?

2. **Which IDE had the most clear documentation?** What made it clearer?

3. **Which IDE took longest to get working?** What was the main bottleneck?

4. **Did your expectations match reality?** (Based on marketing/description, did the IDE behave as promised?)

---

## Part 2: Configuration Approach Comparison

### How Each IDE Approaches Setup

**Zed's approach**:
- Simple settings.json file
- You manually add API key
- One model per feature (inline, autocomplete, commits)
- Fast to configure, minimal options

**Cursor's approach**:
- VS Code settings compatibility
- Import existing VS Code extensions
- .cursorrules configuration file
- Moderate complexity, many options

**Antigravity's approach**:
- Workspace-based organization
- Agent Manager for orchestration
- Artifact system for transparency
- Configuration before agent creation

### Comparison Question

**How would you characterize each IDE's configuration philosophy?**

- **Zed**: ?
- **Cursor**: ?
- **Antigravity**: ?

**Example answers**:
- Zed: "Minimal, opinionated, fast"
- Cursor: "Familiar to VS Code users, extensible"
- Antigravity: "Agent-centric, artifact-driven"

---

## Part 3: Architectural Differences

### Identify Three Architectural Differences

Based on your installation and setup experience, identify three ways the IDEs differ architecturally:

**Difference 1**: [Your observation about how they're fundamentally different]

**Evidence**:
- From installation: [Evidence from installation process]
- From setup: [Evidence from configuration]

**Why it matters**: [How this difference affects workflow]

---

**Difference 2**: [Your observation]

**Evidence**:
- From installation: [Evidence]
- From setup: [Evidence]

**Why it matters**: [How this difference affects workflow]

---

**Difference 3**: [Your observation]

**Evidence**:
- From installation: [Evidence]
- From setup: [Evidence]

**Why it matters**: [How this difference affects workflow]

---

### Possible Architecturally Differences (Hints)

If you're stuck, consider:

- **Paradigm**: Is the IDE editor-first (Zed/Cursor) or agent-first (Antigravity)?
- **Extensibility**: Does IDE support plugins/extensions (Cursor) or built-in only (Zed/Antigravity)?
- **Coordination**: How do AI features coordinate? (Inline for Zed, Chat/Agent for Cursor, Agent Manager for Antigravity)
- **Data flow**: Where does your code go? (Local only for Zed/Cursor, Workspace + Artifacts for Antigravity)
- **AI agency**: How much can AI do autonomously? (Limited in Zed/Cursor, extensive in Antigravity)

---

## Part 4: Scenario Matching

### Choose the Right IDE for Different Scenarios

For each scenario, pick the best IDE (Zed, Cursor, or Antigravity) and justify with 2+ architectural reasons.

**Scenario 1: Solo developer, small Python script, wants fast AI suggestions**

Best IDE: **________**

Reasoning:
1. [Architectural reason 1]
2. [Architectural reason 2]

**Scenario 2: Team lead with 5 developers, VS Code users, needs .cursorrules for team consistency**

Best IDE: **________**

Reasoning:
1. [Architectural reason 1]
2. [Architectural reason 2]

**Scenario 3: Complex full-stack project, needs agents to research APIs, test automatically, generate documentation**

Best IDE: **________**

Reasoning:
1. [Architectural reason 1]
2. [Architectural reason 2]

**Scenario 4: Learning environment, want to see AI's step-by-step thinking and decision-making**

Best IDE: **________**

Reasoning:
1. [Architectural reason 1]
2. [Architectural reason 2]

**Scenario 5: Need to integrate with GitHub, automated testing, CI/CD pipelines**

Best IDE: **________**

Reasoning:
1. [Architectural reason 1]
2. [Architectural reason 2]

---

## Part 5: Written Synthesis

### Write a 300-500 Word Comparison

Synthesize your observations into a coherent comparison. Include:

1. **Opening statement**: What surprised you most about the three IDEs?

2. **Installation journey**: What was the progression from easiest to hardest to install? Why?

3. **Architectural comparison**: Summarize the three key architectural differences you identified in Part 3.

4. **Design philosophy**: What does each IDE's design tell you about their creators' vision?
   - Zed: What does simplicity/speed suggest about Zed's vision?
   - Cursor: What does VS Code compatibility suggest about Cursor's vision?
   - Antigravity: What does agent-first architecture suggest about Antigravity's vision?

5. **Personal preference**: Which IDE appeals to you most and why? Base on architecture, not hype.

6. **Conclusion**: How does tool choice reflect developer values/needs?

---

## Expected Insights

By completing this exercise, you should realize:

1. **No single "best" IDE**
   - Zed excels at speed and simplicity
   - Cursor excels at familiarity and extensibility
   - Antigravity excels at agent autonomy and transparency

2. **Architecture drives experience**
   - Installation complexity reflects architectural choices
   - Configuration approach reveals design philosophy
   - Workflow differences stem from fundamental architectural decisions

3. **Context determines choice**
   - Different scenarios reward different IDEs
   - Your workflow determines the best tool
   - Tool choice should match project and team needs

4. **Learn from each approach**
   - Even if you prefer one, each IDE teaches something
   - Zed teaches you minimalism
   - Cursor teaches you extensibility
   - Antigravity teaches you agent-driven workflows

---

## Verification Checklist

Check that your comparison:

- [ ] Completed installation experience table with honest observations
- [ ] Answered reflection questions with specific examples
- [ ] Identified three architectural differences (not cosmetic differences)
- [ ] Matched 5 scenarios to appropriate IDE with 2+ reasons each
- [ ] Wrote 300-500 word synthesis covering all six points
- [ ] Included personal perspective (not just summary of facts)
- [ ] Referenced concrete evidence from installation (not speculation)

**All checked?** Exercise complete!

---

## Extension: Deep Dive

If you want to explore further:

### Option 1: Create Same Project in All Three IDEs

Create the same simple project (e.g., "hello world" in three files) using each IDE. Observe:
- How long does each take?
- Which suggests better code?
- Which is most comfortable to use?
- Which documentation is clearest?

### Option 2: Advanced Feature Comparison

Explore one advanced feature in each IDE:
- **Zed**: Multi-model configuration with tab autocomplete
- **Cursor**: Agent mode with .cursorrules customization
- **Antigravity**: Complex multi-task agent with parallel execution

Document which IDE handles advanced features best.

### Option 3: Team Collaboration Scenario

Imagine you're starting a startup. Which IDE would you choose if:
- You have 5 developers
- You want consistent AI behavior across team
- You plan to grow to 50+ developers
- You need auditable AI decision-making

Justify your choice architecturally.

---

## Comparison Template for Writing

If you want structure for Part 5 (written synthesis):

```
# Architectural Comparison: Zed, Cursor, and Antigravity

## My Installation Journey

I was surprised that [observation about your experience].
The order of ease was [1] easiest to [3] hardest because [reasons].

## Three Architectural Differences

**Difference 1: [Name]**
- Zed: [How Zed does it]
- Cursor: [How Cursor does it]
- Antigravity: [How Antigravity does it]
- Why it matters: [Impact on workflow]

[Repeat for Differences 2 and 3]

## Design Philosophy

Each IDE's architecture reveals its creators' values:
- Zed's simplicity suggests: [your interpretation]
- Cursor's extensibility suggests: [your interpretation]
- Antigravity's agency suggests: [your interpretation]

## My Preference

I prefer [IDE] because [architectural reasons—not hype].
For [use case], I'd choose [IDE] because [reasons].

## Conclusion

Tool choice reveals [insight about architecture and developer needs].
```

---

## Why This Matters

This isn't just an academic exercise. By comparing IDEs architecturally:

1. **You develop taste**: You learn to evaluate tools based on principles, not marketing
2. **You understand tradeoffs**: You see that speed, simplicity, and agency can't all be maximized equally
3. **You think systemically**: You recognize that small architectural choices compound into very different experiences
4. **You make better choices**: Future tool selection becomes intentional, not random

---

## Next: Apply This Learning

In Lesson 7, you'll dive deep into **Antigravity agent workflows**. You'll use what you learned here to understand:
- Why Antigravity's agent-first architecture enables capabilities Zed/Cursor can't match
- How to leverage agent autonomy for complex projects
- When Antigravity is the right choice vs other IDEs

---

## Summary

| IDE | Installation | Philosophy | Best For |
|-----|--------------|-----------|----------|
| **Zed** | [Your assessment] | [Your assessment] | [Your assessment] |
| **Cursor** | [Your assessment] | [Your assessment] | [Your assessment] |
| **Antigravity** | [Your assessment] | [Your assessment] | [Your assessment] |

**By completing this exercise, you've moved from "I installed three IDEs" to "I understand the architectural principles underlying each IDE."** That's significant learning.

---

## Reflection: What You've Accomplished

Across Lessons 2-6, you:
- ✅ Installed three distinct IDE architectures
- ✅ Configured AI providers (same keys, different UIs)
- ✅ Experienced installation UX and troubleshooting
- ✅ Created agents across different platforms
- ✅ Compared and contrasted approaches

**That's a comprehensive survey of the AI-native IDE landscape in 2025.**

Next lesson: Master Antigravity's advanced agent features.

---

**Time check**: If you spent 10-15 minutes on this exercise, you're on pace. If you went deeper and extended it, that's excellent learning—you've developed richer understanding.

**Ready for Lesson 7?** You've earned it!
