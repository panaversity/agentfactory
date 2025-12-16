"""Schema validation tools for PanaversityFS.

Implements validate_book tool for book structure validation (FR-007, FR-008):
- Content paths: content/{NN-Name}/{NN-Name}/{NN-name}(.summary)?.md
- Asset paths: static/(images|slides|videos|audio)/{path}
"""

from mcp.server.fastmcp.server import Context

from panaversity_fs.app import mcp
from panaversity_fs.models import ValidateBookInput, OperationType, OperationStatus
from panaversity_fs.storage import get_operator
from panaversity_fs.path_utils import (
    validate_content_path,
    validate_asset_path,
    is_content_path,
    is_asset_path
)
from panaversity_fs.audit import log_operation
from datetime import datetime, timezone
from dataclasses import dataclass, field
import json


@dataclass
class ValidationIssue:
    """A single validation issue found during book validation."""
    path: str
    severity: str  # "error" or "warning"
    message: str
    suggestion: str | None = None


@dataclass
class ValidationResult:
    """Complete validation result for a book."""
    valid: bool
    book_id: str
    total_files: int
    content_files: int
    asset_files: int
    errors: list[ValidationIssue] = field(default_factory=list)
    warnings: list[ValidationIssue] = field(default_factory=list)

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            "valid": self.valid,
            "book_id": self.book_id,
            "summary": {
                "total_files": self.total_files,
                "content_files": self.content_files,
                "asset_files": self.asset_files,
                "error_count": len(self.errors),
                "warning_count": len(self.warnings)
            },
            "errors": [
                {
                    "path": e.path,
                    "severity": e.severity,
                    "message": e.message,
                    "suggestion": e.suggestion
                } for e in self.errors
            ],
            "warnings": [
                {
                    "path": w.path,
                    "severity": w.severity,
                    "message": w.message,
                    "suggestion": w.suggestion
                } for w in self.warnings
            ] if self.warnings else []
        }


@mcp.tool(
    name="validate_book",
    annotations={
        "title": "Validate Book Schema",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def validate_book(params: ValidateBookInput, ctx: Context) -> str:
    """Validate book structure against schema (FR-007, FR-008).

    Scans entire book and validates:
    - Content paths match: content/{NN-Name}/{NN-Name}/{NN-name}(.summary)?.md
    - Asset paths match: static/(img|slides|videos|audio)/{path}
    - No invalid files in content/ or static/ directories
    - Proper directory structure

    Args:
        params (ValidateBookInput): Validated input containing:
            - book_id (str): Book identifier to validate
            - strict (bool): Fail on first error vs collect all (default: False)
            - include_warnings (bool): Include non-critical warnings (default: True)

    Returns:
        str: JSON validation report with errors and warnings

    Example:
        ```
        # Validate book
        Input: {"book_id": "ai-native-python"}
        Output: {
          "valid": false,
          "book_id": "ai-native-python",
          "summary": {
            "total_files": 45,
            "content_files": 30,
            "asset_files": 15,
            "error_count": 2,
            "warning_count": 3
          },
          "errors": [
            {
              "path": "content/1-intro/lesson.md",
              "severity": "error",
              "message": "Invalid part number format (must be NN-Name)",
              "suggestion": "Rename to content/01-intro/..."
            }
          ],
          "warnings": [...]
        }

        # Strict mode (fail fast)
        Input: {"book_id": "ai-native-python", "strict": true}
        Output: {"valid": false, "error": "...", "failed_at": "content/..."}
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        op = get_operator()
        book_path = f"books/{params.book_id}/"

        result = ValidationResult(
            valid=True,
            book_id=params.book_id,
            total_files=0,
            content_files=0,
            asset_files=0
        )

        # Check if book exists
        try:
            await op.stat(book_path.rstrip('/'))
        except Exception:
            # Log error
            await log_operation(
                operation=OperationType.VALIDATE_BOOK,
                path=book_path,
                status=OperationStatus.ERROR,
                error_message=f"Book not found: {params.book_id}",
                book_id=params.book_id
            )
            return json.dumps({
                "valid": False,
                "book_id": params.book_id,
                "error": f"Book not found: {params.book_id}",
                "summary": {"total_files": 0, "content_files": 0, "asset_files": 0, "error_count": 1, "warning_count": 0},
                "errors": [{"path": book_path, "severity": "error", "message": "Book directory does not exist", "suggestion": "Create the book directory first"}],
                "warnings": []
            }, indent=2)

        # Scan all files in book
        entries = await op.scan(book_path)

        async for entry in entries:
            # Skip directories
            if entry.path.endswith('/'):
                continue

            result.total_files += 1

            # Extract relative path from book root
            rel_path = entry.path.replace(book_path, "")

            # Validate based on path type
            if rel_path.startswith("content/"):
                result.content_files += 1

                # Skip non-.md files in content (warning)
                if not rel_path.endswith(".md"):
                    if params.include_warnings:
                        result.warnings.append(ValidationIssue(
                            path=rel_path,
                            severity="warning",
                            message="Non-markdown file in content directory",
                            suggestion="Move to static/ directory or convert to markdown"
                        ))
                    continue

                # Validate content path structure
                validation = validate_content_path(rel_path)
                if not validation.is_valid:
                    error = ValidationIssue(
                        path=rel_path,
                        severity="error",
                        message="; ".join(validation.errors),
                        suggestion=_suggest_content_fix(rel_path)
                    )
                    result.errors.append(error)
                    result.valid = False

                    if params.strict:
                        # Fail fast in strict mode
                        await log_operation(
                            operation=OperationType.VALIDATE_BOOK,
                            path=book_path,
                            status=OperationStatus.ERROR,
                            error_message=f"Strict validation failed at: {rel_path}",
                            book_id=params.book_id
                        )
                        return json.dumps({
                            "valid": False,
                            "book_id": params.book_id,
                            "error": error.message,
                            "failed_at": rel_path,
                            "suggestion": error.suggestion
                        }, indent=2)

            elif rel_path.startswith("static/"):
                result.asset_files += 1

                # Validate asset path structure
                validation = validate_asset_path(rel_path)
                if not validation.is_valid:
                    error = ValidationIssue(
                        path=rel_path,
                        severity="error",
                        message="; ".join(validation.errors),
                        suggestion=_suggest_asset_fix(rel_path)
                    )
                    result.errors.append(error)
                    result.valid = False

                    if params.strict:
                        await log_operation(
                            operation=OperationType.VALIDATE_BOOK,
                            path=book_path,
                            status=OperationStatus.ERROR,
                            error_message=f"Strict validation failed at: {rel_path}",
                            book_id=params.book_id
                        )
                        return json.dumps({
                            "valid": False,
                            "book_id": params.book_id,
                            "error": error.message,
                            "failed_at": rel_path,
                            "suggestion": error.suggestion
                        }, indent=2)

            elif rel_path.startswith("users/"):
                # Overlay content - count but don't validate (overlays are user-specific)
                if params.include_warnings:
                    result.warnings.append(ValidationIssue(
                        path=rel_path,
                        severity="warning",
                        message="User overlay file (skipped from validation)",
                        suggestion=None
                    ))

            else:
                # Unknown file type
                if params.include_warnings:
                    result.warnings.append(ValidationIssue(
                        path=rel_path,
                        severity="warning",
                        message="File outside of content/ or static/ directories",
                        suggestion="Move to appropriate directory or remove"
                    ))

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.VALIDATE_BOOK,
            path=book_path,
            status=OperationStatus.SUCCESS if result.valid else OperationStatus.ERROR,
            execution_time_ms=execution_time,
            book_id=params.book_id
        )

        # Build response
        response = result.to_dict()
        if not params.include_warnings:
            response["warnings"] = []

        return json.dumps(response, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.VALIDATE_BOOK,
            path=f"books/{params.book_id}/",
            status=OperationStatus.ERROR,
            error_message=str(e),
            book_id=params.book_id
        )

        return f"Error validating book: {type(e).__name__}: {str(e)}"


def _suggest_content_fix(path: str) -> str:
    """Generate suggestion for fixing a content path."""
    parts = path.split("/")

    if len(parts) < 4:
        return "Content path needs: content/{NN-Part}/{NN-Chapter}/{NN-lesson}.md"

    suggestions = []

    # Check part format
    if len(parts) > 1:
        part = parts[1]
        if not part[:2].isdigit():
            suggestions.append(f"Part '{part}' should start with NN- (e.g., '01-{part}')")

    # Check chapter format
    if len(parts) > 2:
        chapter = parts[2]
        if not chapter[:2].isdigit():
            suggestions.append(f"Chapter '{chapter}' should start with NN- (e.g., '01-{chapter}')")

    # Check lesson format
    if len(parts) > 3:
        lesson = parts[3]
        if not lesson.endswith(".md"):
            suggestions.append("Lesson file must end with .md")
        elif not lesson[:2].isdigit():
            lesson_name = lesson.replace(".md", "").replace(".summary", "")
            suggestions.append(f"Lesson '{lesson_name}' should start with NN- (e.g., '01-{lesson_name}')")

    return "; ".join(suggestions) if suggestions else "Rename to match content/{NN-Part}/{NN-Chapter}/{NN-lesson}.md pattern"


def _suggest_asset_fix(path: str) -> str:
    """Generate suggestion for fixing an asset path."""
    parts = path.split("/")

    if len(parts) < 3:
        return "Asset path needs: static/{type}/{filename} where type is images|slides|videos|audio"

    if len(parts) > 1:
        asset_type = parts[1]
        if asset_type not in ("images", "slides", "videos", "audio"):
            return f"Unknown asset type '{asset_type}'. Use one of: images, slides, videos, audio"

    return "Asset path format: static/(images|slides|videos|audio)/{filename}"
