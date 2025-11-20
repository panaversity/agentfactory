# Data Model — SEO/AEO/GEO

Date: 2025-11-04  
Branch: 001-seo-aeo-geo

## Entities

### KnowledgeUnit

- id: string (stable)
- title: string
- summary: string
- questions: string[]
- answers: string[] (aligned by index with questions)
- claims: string[]
- evidence: string[] (citations/links)
- canonical_url: string
- last_updated: datetime (ISO8601)
- license: string (SPDX or URL)
- authors: string[]

### RetrievalFeedPage

- page_number: integer
- page_size: integer
- items: KnowledgeUnit[]
- next_page: string | null

### CitationEvent

- id: string
- source_assistant: enum (ChatGPT | Perplexity | GoogleAIO | Other)
- query: string
- destination_url: string
- timestamp: datetime
- channel: enum (SEO | AEO | GEO)
- attributed: boolean

## Validation Rules

- KnowledgeUnit must have canonical_url, last_updated, license, authors (non-empty).
- questions.length === answers.length.
- IDs are globally unique and stable across builds.
- Retrieval pages must be deterministic for a given snapshot.
