# Research: Highlight Selection AI Dialog - Storage Decision

**Branch**: `002-highlight-ai-dialog` | **Date**: 2025-11-12 | **Spec**: /specs/002-highlight-ai-dialog/spec.md

## Decision: Storage for Gemini API Key and Model Selection

**What was chosen**: The Gemini API key and model selection will be stored in a backend configuration file (e.g., a JSON file) managed by the Python backend. The frontend will interact with the backend API to save and retrieve these settings.

**Rationale**:
- **Security**: Storing sensitive information like API keys on the backend is more secure than in browser local storage, as it prevents exposure to client-side attacks.
- **Centralized Management**: The backend can manage the configuration, providing a single source of truth.
- **User Configuration**: The frontend can provide a user interface to update these settings, which are then persisted on the backend.
- **Simplicity for E-book**: For a Docusaurus e-book, a simple JSON file on the backend is a straightforward and effective solution without requiring a full-fledged database.

**Alternatives considered**:
- **Browser Local Storage**: Rejected due to security concerns for storing API keys.
- **Environment Variables (Backend)**: Rejected because the API key needs to be user-configurable, not just set at deployment time.