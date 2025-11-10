# Research Findings: Highlight Selection AI Dialog

**Feature Branch**: `001-highlight-ai-dialog`
**Created**: 2025-11-10

## Research Task 1: Highlight Detection for Diagrams and Code Blocks in Docusaurus

### Goal

Identify the most effective and Docusaurus-native way to detect and extract content from various highlight types (text, diagrams, code blocks) to accurately capture selection context.

### Findings

*   **Decision**: For text highlighting, leverage the browser's native `Selection` API. For code snippets, integrate with Docusaurus's existing code block rendering (e.g., `prism-react-renderer`) to identify code elements. For diagrams, initially focus on text-based descriptions or alt-text if diagrams are images, or explore Docusaurus plugins for SVG/interactive diagram handling if available.
*   **Rationale**: Native `Selection` API is robust for text. Integrating with existing code rendering is efficient. Diagram highlighting is complex and requires further investigation, so starting with text-based approaches is pragmatic.
*   **Alternatives Considered**: Using a third-party highlighting library (e.g., `react-highlight-words`) for all content types (rejected due to potential conflicts with Docusaurus rendering and over-engineering for initial text/code focus).

## Research Task 2: AI Prompt Engineering for Gemini

### Goal

Explore best practices for prompt engineering with Gemini for generating structured educational content (context, theory, instructions, examples) from diverse inputs (text, code, diagram descriptions).

### Findings

*   **Decision**: Use a structured prompt format that clearly defines the expected output sections (Context, Theory, Instructions, Example) and provides clear instructions for each. Include examples of desired output. Leverage few-shot prompting with diverse examples covering text, code, and diagram descriptions.
*   **Rationale**: Structured prompts improve AI response consistency and quality. Few-shot examples guide the model to the desired output format and content style. Explicitly defining sections helps the model adhere to the required output structure.
*   **Alternatives Considered**: Simple, unstructured prompts (rejected due to inconsistent output format and lower relevance); fine-tuning a model (deferred for later optimization, as prompt engineering is a faster initial approach).
