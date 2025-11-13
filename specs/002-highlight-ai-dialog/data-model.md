# Data Model: Highlight Selection AI Dialog

**Branch**: `002-highlight-ai-dialog` | **Date**: 2025-11-12 | **Spec**: /specs/002-highlight-ai-dialog/spec.md

## Entity: AIConfiguration

Represents the configuration settings for the AI dialog, including the Gemini API key and the selected Gemini model.

### Fields

-   **`api_key` (string)**:
    -   **Description**: The API key used to authenticate requests to the Google Gemini API.
    -   **Validation**: Must be a non-empty string. Backend validation will ensure its validity with the Gemini API.
    -   **Security**: Should be stored securely on the backend and never exposed to the frontend.
-   **`model` (string)**:
    -   **Description**: The identifier for the selected Gemini AI model.
    -   **Validation**: Must be `gemini-2.5-flash`.
    -   **Default**: `gemini-2.5-flash`

### Relationships

-   None (standalone configuration entity).

### State Transitions

-   **Initial State**: `api_key` and `model` may be unset or default.
-   **Configured State**: `api_key` is set and validated; `model` is set to `gemini-2.5-flash`.
-   **Update**: `api_key` and `model` can be updated by the user via the configuration interface.