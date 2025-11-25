## Core Concept
Context windows are the AI's working memory with fixed capacity; understanding token counting and utilization thresholds is the foundation for managing context effectively across sessions.

## Key Mental Models
- **Context as working memory**: Fixed capacity (200K for Claude Sonnet 4.5) shared between all loaded content and conversation, not permanent storage
- **Token estimation rule**: 1 word ≈ 1-1.2 tokens for quick estimates without tools
- **Utilization warning zones**: Green (0-70%, safe), Yellow (70-85%, plan checkpoint), Red (85-100%, create checkpoint immediately)
- **Session-specific scope**: Each context window is isolated; patterns don't persist across sessions without explicit documentation

## Critical Patterns
- **Session note template**: Tracks context loaded, progress, conversation estimate, and token utilization percentage to build awareness
- **Observable degradation signals**: Responses slower/shorter (yellow), AI forgets patterns or repeats suggestions (red) — observe behaviors before token counts

## AI Collaboration Keys
- AI can validate your token estimates and identify estimation factors you missed (formatting overhead, special characters)
- AI helps identify current warning zone based on content and remaining work
- AI recommends checkpoint timing based on estimated remaining tasks

## Common Mistakes
- Assuming context is unlimited (it's fixed and fills quickly)
- Loading everything upfront to avoid "missing context" (wastes capacity, prevents on-demand loading)
- Trusting AI's estimates over your observations (if responses feel generic/repetitive, degradation is occurring regardless)
- Not tracking context manually until AI-based solutions are needed (awareness builds intuition)

## Connections
- **Builds on**: Understanding tokens and context windows as a concept (Part 3 prerequisite)
- **Leads to**: Lesson 2 (recognizing seven degradation symptoms), Lesson 3 (preventing degradation through progressive loading)
