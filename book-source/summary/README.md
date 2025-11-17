# AI-Generated Summaries

This directory stores AI-generated page summaries created by the CollapsibleSummary component.

## Structure

Summaries are stored as JSON files with the following naming convention:
- Chapter path: `docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/readme.md`
- Summary file: `01-Introducing-AI-Driven-Development__01-ai-development-revolution__readme.json`

## Format

Each summary file contains:
```json
{
  "pagePath": "docs/path/to/page.md",
  "summary": "AI-generated summary text in markdown format",
  "generatedAt": "ISO 8601 timestamp"
}
```

## Notes

- Summaries are generated on-demand when users first open the collapsible section
- Once generated, summaries are cached and reused
- Summaries are generated using Google Gemini AI (up to 500 words)
