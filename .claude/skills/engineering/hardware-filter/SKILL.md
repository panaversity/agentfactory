---
name: "hardware-filter"
description: "Filter and adapt content based on student's hardware tier (1-4). Use for personalized content delivery."
version: "0.1.0"
status: "placeholder"
---

# Hardware Filter Skill

**Purpose**: Filter and adapt content based on student's hardware profile (Tier 1-4).

## Hardware Tiers

| Tier | Equipment | Capabilities |
|------|-----------|--------------|
| **1** | Laptop/Cloud | Browser, MockROS, Pyodide |
| **2** | RTX GPU | Local Isaac Sim, Gazebo |
| **3** | Jetson Edge | Real sensors, RealSense |
| **4** | Physical Robot | Unitree Go2/G1 |

## When to Use

- Filtering lesson content by student profile
- Showing/hiding hardware-specific sections
- Providing cloud fallbacks for GPU content

## Inputs Required

- Student's hardware tier (from profile)
- Content with hardware tier markers
- Fallback preferences

## Output

- Filtered content appropriate for student's tier
- Cloud alternatives for higher-tier content
- "Upgrade path" suggestions

## Integration

- Used by HardwareContext provider
- Works with `<HardwareGate>` and `<CloudFallback>` components
- Integrated with personalization endpoint

---

**Status**: Placeholder - To be implemented for hackathon personalization feature (50 bonus points).
