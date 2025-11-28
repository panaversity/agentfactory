---
name: "mermaid-diagram"
description: "Generate Mermaid diagrams for educational content. Use when visualizing concepts, architectures, or workflows."
version: "0.1.0"
status: "placeholder"
---

# Mermaid Diagram Skill

**Purpose**: Generate clear, educational Mermaid diagrams that visualize concepts for learners.

## When to Use

- Visualizing system architectures (ROS 2 node graphs)
- Showing data flow (sensor → processing → action)
- Illustrating state machines (robot behaviors)
- Explaining hierarchies (hardware tiers, module structure)

## Diagram Types

| Type | Use Case |
|------|----------|
| `flowchart` | Process flows, decision trees |
| `sequenceDiagram` | Message passing, ROS topics |
| `stateDiagram` | Robot states, FSMs |
| `classDiagram` | Code structure, URDF hierarchy |
| `graph` | System architecture |

## Output

- Mermaid code block ready for MDX
- Alt text for accessibility
- Caption explaining the diagram

## Integration

Used by lesson-generator when content requires visualization.

---

**Status**: Placeholder - To be implemented with diagram generation patterns.
