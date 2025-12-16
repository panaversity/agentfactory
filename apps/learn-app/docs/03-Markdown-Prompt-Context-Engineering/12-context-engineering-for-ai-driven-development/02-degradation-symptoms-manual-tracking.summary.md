### Core Concept
Seven distinct degradation symptoms (repetitive suggestions, forgotten patterns, performance degradation, generic responses, lost context references, contradictory advice, task confusion) signal context saturation; pattern recognition builds diagnostic intuition.

### Key Mental Models
- **Degradation as attention mechanism failure**: As context window fills, earlier information becomes harder to retrieve effectively
- **Symptom specificity**: Each symptom reveals different aspects of degradation (retrieval failure vs generation hallucination vs computational slowdown)
- **Symptom clustering**: Multiple symptoms appearing together indicates severe degradation requiring immediate action
- **Healthy vs degraded progression**: Same question at 30% utilization produces specific project-focused advice; at 85% produces generic boilerplate

### Critical Patterns
- **Symptom 1 - Repetition**: Same explanation word-for-word without acknowledgment (indicates model regenerating from generic knowledge)
- **Symptom 4 - Generic responses**: Shift from project-specific (mentioning table names, technologies) to "best practices" language
- **Symptom 5 - Lost context**: Information provided but not retrieved even when explicitly referenced
- **Symptom 7 - Task confusion**: Requesting clarification on information already established (scope reset)

### AI Collaboration Keys
- AI can identify symptoms from transcripts more reliably than manual observation of long sessions
- AI explains why each symptom occurs mechanically (attention computation, context eviction)
- AI validates your severity assessment and recommends action (continue/compress/isolate/restart)

### Common Mistakes
- Confusing high utilization % with actual degradation (80% healthy session may outperform 50% degraded session if degradation symptoms present)
- Trusting "my context seems fine" when multiple symptoms are present (confirmation bias)
- Missing subtle symptoms early (performance degradation is easier to miss than repetition)
- Not distinguishing between temporary slowness and systemic degradation

### Connections
- **Builds on**: Lesson 1 (token estimation, warning thresholds)
- **Leads to**: Lesson 3 (preventing symptoms through progressive loading before they emerge), Lesson 4 (compressing and restarting when symptoms detected)
