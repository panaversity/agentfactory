"""Path validation utilities for PanaversityFS.

Provides centralized path validation to ensure consistent schema enforcement
across content writes, overlay resolution, and validation tools.

Pattern Format (FR-007, FR-008):
- Content: content/{NN-Name}/{NN-Name}/{NN-name}(.summary)?.md
- Assets: static/(images|slides|videos|audio)/{path}
- Overlays: users/{user_id}/content/{...}
"""

import re
from dataclasses import dataclass, field
from typing import Optional


# =============================================================================
# Validation Result
# =============================================================================

@dataclass
class ValidationResult:
    """Result of path validation.

    Attributes:
        is_valid: Whether the path passes validation
        errors: List of validation error messages (empty if valid)
        normalized_path: Cleaned/normalized path (if valid)
    """
    is_valid: bool
    errors: list[str] = field(default_factory=list)
    normalized_path: Optional[str] = None


# =============================================================================
# Compiled Regex Patterns
# =============================================================================

# Content path: content/{NN-Name}/{NN-Name}/{NN-name}(.summary)?.md
# - NN = two digits (00-99)
# - Name = alphanumeric with hyphens
# - Optional .summary suffix before .md
CONTENT_PATH_PATTERN = re.compile(
    r"^content/"
    r"(?P<part>\d{2}-[a-zA-Z0-9][a-zA-Z0-9-]*)/"  # Part: NN-Name
    r"(?P<chapter>\d{2}-[a-zA-Z0-9][a-zA-Z0-9-]*)/"  # Chapter: NN-Name
    r"(?P<lesson>\d{2}-[a-zA-Z0-9][a-zA-Z0-9-]*)"  # Lesson: NN-name
    r"(?P<summary>\.summary)?\.md$"  # Optional .summary + .md extension
)

# Asset path: static/(images|slides|videos|audio)/{path}
# - Allowed asset types are specific (matches AssetType enum values)
# - Path after type can be any valid filename/path
ASSET_PATH_PATTERN = re.compile(
    r"^static/"
    r"(?P<asset_type>images|slides|videos|audio)/"
    r"(?P<filename>.+)$"
)

# Overlay path: users/{user_id}/content/{...}
# - user_id must be alphanumeric with hyphens/underscores
# - After users/{user_id}/ follows standard content path structure
OVERLAY_PATH_PATTERN = re.compile(
    r"^users/"
    r"(?P<user_id>[a-zA-Z0-9_-]+)/"
    r"(?P<content_path>content/.+)$"
)

# Security: Dangerous path components to reject
DANGEROUS_PATTERNS = [
    r"\.\.",          # Path traversal
    r"^\s*/",         # Leading slash (absolute path)
    r"\x00",          # Null byte injection
    r"[\r\n]",        # Newline injection
    r"^~",            # Home directory expansion
]

DANGEROUS_REGEX = re.compile("|".join(DANGEROUS_PATTERNS))


# =============================================================================
# Security Validation
# =============================================================================

def _check_security(path: str) -> list[str]:
    """Check path for security issues.

    Args:
        path: Path to validate

    Returns:
        List of security-related error messages (empty if safe)
    """
    errors = []

    if DANGEROUS_REGEX.search(path):
        if ".." in path:
            errors.append("Path traversal detected (..) - rejected for security")
        if path.startswith("/"):
            errors.append("Absolute paths not allowed - use relative paths")
        if "\x00" in path:
            errors.append("Null byte injection detected - rejected for security")
        if "\r" in path or "\n" in path:
            errors.append("Newline characters not allowed in paths")
        if path.startswith("~"):
            errors.append("Home directory expansion not allowed")

    return errors


# =============================================================================
# Path Validators
# =============================================================================

def validate_content_path(path: str) -> ValidationResult:
    """Validate a content file path against the book schema (FR-007).

    Valid format: content/{NN-Name}/{NN-Name}/{NN-name}(.summary)?.md

    Examples:
        - content/01-introduction/01-basics/01-hello-world.md (valid)
        - content/01-intro/02-setup/03-installation.summary.md (valid)
        - lessons/random/file.md (invalid - wrong prefix)
        - content/1-intro/01-basics/01-hello.md (invalid - single digit)

    Args:
        path: Path to validate

    Returns:
        ValidationResult with is_valid, errors, and normalized_path
    """
    errors = _check_security(path)
    if errors:
        return ValidationResult(is_valid=False, errors=errors)

    match = CONTENT_PATH_PATTERN.match(path)
    if not match:
        return ValidationResult(
            is_valid=False,
            errors=[
                f"Path must match content/{{NN-Name}}/{{NN-Name}}/{{NN-name}}(.summary)?.md",
                f"Got: {path}"
            ]
        )

    return ValidationResult(is_valid=True, normalized_path=path)


def validate_asset_path(path: str) -> ValidationResult:
    """Validate an asset file path (FR-008).

    Valid format: static/(images|slides|videos|audio)/{path}

    Examples:
        - static/images/diagram.png (valid)
        - static/videos/lesson1.mp4 (valid)
        - static/docs/readme.txt (invalid - docs not allowed type)
        - assets/images/photo.jpg (invalid - wrong prefix)

    Args:
        path: Path to validate

    Returns:
        ValidationResult with is_valid, errors, and normalized_path
    """
    errors = _check_security(path)
    if errors:
        return ValidationResult(is_valid=False, errors=errors)

    match = ASSET_PATH_PATTERN.match(path)
    if not match:
        return ValidationResult(
            is_valid=False,
            errors=[
                f"Path must match static/(images|slides|videos|audio)/{{filename}}",
                f"Got: {path}"
            ]
        )

    return ValidationResult(is_valid=True, normalized_path=path)


def validate_overlay_path(path: str, expected_user_id: str) -> ValidationResult:
    """Validate an overlay path and verify user_id matches (FR-019).

    Valid format: users/{user_id}/content/{...}

    The user_id in the path must match the expected_user_id parameter
    to prevent unauthorized access to other users' overlays.

    Args:
        path: Overlay path to validate
        expected_user_id: User ID that should own this overlay

    Returns:
        ValidationResult with is_valid, errors, and normalized_path
    """
    errors = _check_security(path)
    if errors:
        return ValidationResult(is_valid=False, errors=errors)

    match = OVERLAY_PATH_PATTERN.match(path)
    if not match:
        return ValidationResult(
            is_valid=False,
            errors=[
                f"Overlay path must match users/{{user_id}}/content/...",
                f"Got: {path}"
            ]
        )

    path_user_id = match.group("user_id")
    if path_user_id != expected_user_id:
        return ValidationResult(
            is_valid=False,
            errors=[
                f"User ID mismatch in overlay path",
                f"Path user: {path_user_id}, expected: {expected_user_id}"
            ]
        )

    # Also validate the nested content path
    content_path = match.group("content_path")
    content_result = validate_content_path(content_path)
    if not content_result.is_valid:
        return ValidationResult(
            is_valid=False,
            errors=[f"Invalid content path in overlay: {e}" for e in content_result.errors]
        )

    return ValidationResult(is_valid=True, normalized_path=path)


# =============================================================================
# Path Conversion Utilities
# =============================================================================

def extract_user_id_from_overlay(path: str) -> Optional[str]:
    """Extract user_id from an overlay path.

    Args:
        path: Potential overlay path

    Returns:
        user_id if path is a valid overlay path, None otherwise
    """
    match = OVERLAY_PATH_PATTERN.match(path)
    if match:
        return match.group("user_id")
    return None


def convert_base_to_overlay(base_path: str, user_id: str) -> str:
    """Convert a base content path to an overlay path for a specific user.

    Args:
        base_path: Base content path (e.g., "content/01-intro/01-basics/01-hello.md")
        user_id: User ID for the overlay

    Returns:
        Overlay path (e.g., "users/user123/content/01-intro/01-basics/01-hello.md")

    Raises:
        ValueError: If base_path doesn't start with "content/"
    """
    if not base_path.startswith("content/"):
        raise ValueError(f"Base path must start with 'content/': {base_path}")

    return f"users/{user_id}/{base_path}"


def convert_overlay_to_base(overlay_path: str) -> str:
    """Convert an overlay path back to the corresponding base path.

    Args:
        overlay_path: Overlay path (e.g., "users/user123/content/01-intro/...")

    Returns:
        Base path (e.g., "content/01-intro/...")

    Raises:
        ValueError: If overlay_path is not a valid overlay path format
    """
    match = OVERLAY_PATH_PATTERN.match(overlay_path)
    if not match:
        raise ValueError(f"Not a valid overlay path: {overlay_path}")

    return match.group("content_path")


def is_overlay_path(path: str) -> bool:
    """Check if a path is an overlay path.

    Args:
        path: Path to check

    Returns:
        True if path matches overlay pattern, False otherwise
    """
    return OVERLAY_PATH_PATTERN.match(path) is not None


def is_content_path(path: str) -> bool:
    """Check if a path is a content path.

    Args:
        path: Path to check

    Returns:
        True if path matches content pattern, False otherwise
    """
    return CONTENT_PATH_PATTERN.match(path) is not None


def is_asset_path(path: str) -> bool:
    """Check if a path is an asset path.

    Args:
        path: Path to check

    Returns:
        True if path matches asset pattern, False otherwise
    """
    return ASSET_PATH_PATTERN.match(path) is not None
