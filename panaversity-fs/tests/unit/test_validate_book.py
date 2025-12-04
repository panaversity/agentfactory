"""Unit tests for validate_book tool (FR-007, FR-008).

Tests book structure validation against schema.
"""

import pytest
import json
from panaversity_fs.tools.validate import validate_book
from panaversity_fs.models import ValidateBookInput
from panaversity_fs.storage import get_operator


class TestValidateBookBasic:
    """Basic tests for validate_book tool."""

    @pytest.mark.asyncio
    async def test_validate_nonexistent_book(self, setup_fs_backend, mock_context):
        """Validates that nonexistent book returns error."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]

        params = ValidateBookInput(book_id=f"nonexistent-{unique_id}")
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        assert data["valid"] is False
        assert "not found" in data["error"].lower()

    @pytest.mark.asyncio
    async def test_validate_empty_book(self, setup_fs_backend, mock_context):
        """Empty book should be valid (vacuously)."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"empty-book-{unique_id}"

        op = get_operator()

        # Create just the book directory with a dummy file
        await op.write(f"books/{book_id}/.keep", b"")

        params = ValidateBookInput(book_id=book_id)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        assert data["valid"] is True
        assert data["summary"]["total_files"] == 1
        assert data["summary"]["error_count"] == 0


class TestValidateContentPaths:
    """Tests for content path validation (FR-007)."""

    @pytest.mark.asyncio
    async def test_valid_content_path(self, setup_fs_backend, mock_context):
        """Valid content paths pass validation."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"valid-content-{unique_id}"

        op = get_operator()

        # Create valid content structure
        await op.write(
            f"books/{book_id}/content/01-Part/01-Chapter/01-lesson.md",
            b"# Lesson 1"
        )

        params = ValidateBookInput(book_id=book_id)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        assert data["valid"] is True
        assert data["summary"]["content_files"] == 1
        assert data["summary"]["error_count"] == 0

    @pytest.mark.asyncio
    async def test_valid_summary_path(self, setup_fs_backend, mock_context):
        """Valid summary paths pass validation."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"valid-summary-{unique_id}"

        op = get_operator()

        # Create valid summary file
        await op.write(
            f"books/{book_id}/content/01-Part/01-Chapter/01-lesson.summary.md",
            b"# Summary"
        )

        params = ValidateBookInput(book_id=book_id)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        assert data["valid"] is True
        assert data["summary"]["content_files"] == 1
        assert data["summary"]["error_count"] == 0

    @pytest.mark.asyncio
    async def test_invalid_single_digit_part(self, setup_fs_backend, mock_context):
        """Single-digit part number fails validation."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"invalid-part-{unique_id}"

        op = get_operator()

        # Create invalid content path (single digit part)
        await op.write(
            f"books/{book_id}/content/1-Part/01-Chapter/01-lesson.md",
            b"# Lesson"
        )

        params = ValidateBookInput(book_id=book_id)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        assert data["valid"] is False
        assert data["summary"]["error_count"] == 1
        assert "1-Part" in data["errors"][0]["path"] or "content/1" in data["errors"][0]["path"]

    @pytest.mark.asyncio
    async def test_invalid_wrong_prefix(self, setup_fs_backend, mock_context):
        """Wrong prefix (not content/) fails validation."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"invalid-prefix-{unique_id}"

        op = get_operator()

        # Create valid structure under wrong prefix
        await op.write(
            f"books/{book_id}/lessons/01-Part/01-Chapter/01-lesson.md",
            b"# Lesson"
        )

        params = ValidateBookInput(book_id=book_id, include_warnings=True)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        # File outside content/ or static/ should be a warning
        assert len(data["warnings"]) >= 1

    @pytest.mark.asyncio
    async def test_non_md_file_in_content_warning(self, setup_fs_backend, mock_context):
        """Non-.md file in content directory generates warning."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"non-md-{unique_id}"

        op = get_operator()

        # Create .txt file in content
        await op.write(
            f"books/{book_id}/content/01-Part/01-Chapter/notes.txt",
            b"Some notes"
        )

        params = ValidateBookInput(book_id=book_id, include_warnings=True)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        # Should have warning about non-markdown file
        assert len(data["warnings"]) >= 1
        assert any("non-markdown" in w["message"].lower() for w in data["warnings"])


class TestValidateAssetPaths:
    """Tests for asset path validation (FR-008)."""

    @pytest.mark.asyncio
    async def test_valid_img_path(self, setup_fs_backend, mock_context):
        """Valid image path passes validation."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"valid-img-{unique_id}"

        op = get_operator()

        await op.write(
            f"books/{book_id}/static/images/diagram.png",
            b"PNG data"
        )

        params = ValidateBookInput(book_id=book_id)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        assert data["valid"] is True
        assert data["summary"]["asset_files"] == 1

    @pytest.mark.asyncio
    async def test_valid_slides_path(self, setup_fs_backend, mock_context):
        """Valid slides path passes validation."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"valid-slides-{unique_id}"

        op = get_operator()

        await op.write(
            f"books/{book_id}/static/slides/lesson1.pdf",
            b"PDF data"
        )

        params = ValidateBookInput(book_id=book_id)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        assert data["valid"] is True
        assert data["summary"]["asset_files"] == 1

    @pytest.mark.asyncio
    async def test_invalid_asset_type(self, setup_fs_backend, mock_context):
        """Invalid asset type fails validation."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"invalid-type-{unique_id}"

        op = get_operator()

        # Create asset with invalid type
        await op.write(
            f"books/{book_id}/static/docs/readme.txt",
            b"Invalid type"
        )

        params = ValidateBookInput(book_id=book_id)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        assert data["valid"] is False
        assert data["summary"]["error_count"] == 1
        assert "docs" in data["errors"][0]["message"].lower() or "type" in data["errors"][0]["message"].lower()


class TestValidateStrictMode:
    """Tests for strict mode validation."""

    @pytest.mark.asyncio
    async def test_strict_mode_fails_fast(self, setup_fs_backend, mock_context):
        """Strict mode stops on first error."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"strict-{unique_id}"

        op = get_operator()

        # Create multiple invalid files
        await op.write(f"books/{book_id}/content/1-Part/01-Chapter/01-lesson.md", b"First error")
        await op.write(f"books/{book_id}/content/2-Part/01-Chapter/01-lesson.md", b"Second error")

        params = ValidateBookInput(book_id=book_id, strict=True)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        assert data["valid"] is False
        assert "failed_at" in data
        assert "error" in data
        # In strict mode, only the first error is reported
        assert "errors" not in data or len(data.get("errors", [])) <= 1

    @pytest.mark.asyncio
    async def test_non_strict_collects_all_errors(self, setup_fs_backend, mock_context):
        """Non-strict mode collects all errors."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"non-strict-{unique_id}"

        op = get_operator()

        # Create multiple invalid files
        await op.write(f"books/{book_id}/content/1-Part/01-Chapter/01-lesson.md", b"First")
        await op.write(f"books/{book_id}/content/2-Part/01-Chapter/01-lesson.md", b"Second")

        params = ValidateBookInput(book_id=book_id, strict=False)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        assert data["valid"] is False
        # Non-strict collects all errors
        assert len(data["errors"]) >= 2


class TestValidateIncludeWarnings:
    """Tests for include_warnings parameter."""

    @pytest.mark.asyncio
    async def test_include_warnings_true(self, setup_fs_backend, mock_context):
        """Warnings included when include_warnings=True."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"warnings-true-{unique_id}"

        op = get_operator()

        # Create file that generates warning
        await op.write(f"books/{book_id}/content/01-Part/01-Chapter/notes.txt", b"Warning file")

        params = ValidateBookInput(book_id=book_id, include_warnings=True)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        assert len(data["warnings"]) >= 1

    @pytest.mark.asyncio
    async def test_include_warnings_false(self, setup_fs_backend, mock_context):
        """Warnings excluded when include_warnings=False."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"warnings-false-{unique_id}"

        op = get_operator()

        # Create file that generates warning
        await op.write(f"books/{book_id}/content/01-Part/01-Chapter/notes.txt", b"Warning file")

        params = ValidateBookInput(book_id=book_id, include_warnings=False)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        assert data["warnings"] == []


class TestValidateMixedContent:
    """Tests for books with mixed valid/invalid content."""

    @pytest.mark.asyncio
    async def test_mixed_content_reports_all_issues(self, setup_fs_backend, mock_context):
        """Mixed valid/invalid content reports accurately."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"mixed-{unique_id}"

        op = get_operator()

        # Create valid content
        await op.write(f"books/{book_id}/content/01-Part/01-Chapter/01-lesson.md", b"Valid")
        await op.write(f"books/{book_id}/static/images/diagram.png", b"Valid")

        # Create invalid content
        await op.write(f"books/{book_id}/content/1-Invalid/01-Chapter/01-lesson.md", b"Invalid part")

        params = ValidateBookInput(book_id=book_id)
        result = await validate_book(params, mock_context)
        data = json.loads(result)

        assert data["valid"] is False
        assert data["summary"]["total_files"] == 3
        assert data["summary"]["content_files"] >= 2
        assert data["summary"]["asset_files"] == 1
        assert data["summary"]["error_count"] == 1  # Only one invalid file
