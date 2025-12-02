# Module README Template

**Purpose**: Consistent structure for all module landing pages (README.md files)

## MDX Frontmatter

```yaml
---
id: module-[N]-[slug]
title: "Module [N]: [Title]"
sidebar_position: [N]
sidebar_label: "Module [N]: [Short Title]"
description: "[1-2 sentence description for SEO and preview]"
keywords: ["physical-ai", "robotics", "[module-specific-keywords]"]
---
```

## Section Order (MUST be followed exactly)

### 1. Module Overview
- 2-3 paragraphs explaining the module's role
- Connect to overall Physical AI curriculum
- Real-world applications and relevance

### 2. Learning Objectives
- 4-6 objectives using action verbs
- Format: "By the end of this module, you will be able to:"
- Verbs: understand, recognize, explain, write, build, debug, configure, deploy, design, implement, orchestrate

### 3. What You'll Learn (Chapter Index)
- List each chapter with:
  - Chapter title
  - Brief description (1-2 sentences)
  - Estimated time
- Format as `### Chapter [N]: [Title] ([X hours])`

### 4. 4-Layer Teaching Method
- Indicate which layers apply to which chapters
- Format:
  ```
  | Chapter | Layer 1 (Manual) | Layer 2 (AI Collab) | Layer 3 (Intelligence) | Layer 4 (Spec-Driven) |
  ```

### 5. Hardware Requirements
- Table format with Tiers 1-4
- MUST include Tier 1 (cloud/browser) fallback for all content
- Format:
  ```
  | Tier | Equipment | What You Can Do | Cloud Fallback |
  |------|-----------|-----------------|----------------|
  | 1    | Laptop/Browser | [Access level] | N/A - this IS the fallback |
  | 2    | RTX GPU | [Access level] | [Cloud option] |
  | 3    | Jetson/Edge | [Access level] | [Cloud option] |
  | 4    | Physical Robot | [Access level] | [Simulation option] |
  ```

### 6. Prerequisites
- Prior modules required
- Prior knowledge assumed
- Links to prerequisite content

### 7. Module Progression (Mermaid Diagram)
- Visual flow of chapters and layer progression
- Use Mermaid graph LR or TD syntax

### 8. Capstone Project
- Clear description of Layer 4 project
- What students will build
- How it integrates module concepts

### 9. Assessment (for instructors)
- Primary evaluation: Capstone project
- Secondary: Chapter exercises
- Week range for curriculum planning

### 10. Navigation
- Previous module link (if not Module 1)
- Next module link (if not Module 4)
- "Start Learning" CTA button linking to first chapter

### 11. Research Sources
- Official documentation links
- All technical claims must cite sources

## Content Guidelines

1. **No lesson content** - READMEs are navigation/orientation only
2. **Tier 1 accessibility** - Every section must acknowledge cloud fallback
3. **Action verbs** - Learning objectives use measurable verbs
4. **Consistent terminology** - Use terms from constitution vocabulary
5. **Mermaid diagrams** - Use for visual flow (will be enabled via theme)

## Hardware Tier Fallback Paths

| Module | Primary Tier | Tier 1 Fallback |
|--------|--------------|-----------------|
| Module 1 (ROS 2) | Tier 1 | MockROS in browser, Pyodide |
| Module 2 (Gazebo) | Tier 1-2 | Cloud Gazebo, theconstructsim.com |
| Module 3 (Isaac) | Tier 2-3 | NVIDIA Omniverse Cloud |
| Module 4 (VLA) | Tier 1-4 | Simulation + Cloud Voice APIs |

## Week Mapping (13-week curriculum)

- Module 1: Weeks 1-5 (5 weeks)
- Module 2: Weeks 6-7 (2 weeks)
- Module 3: Weeks 8-10 (3 weeks)
- Module 4: Weeks 11-13 (3 weeks)
