"""Unit tests for path validation utilities (T014)."""

import pytest
from panaversity_fs.path_utils import (
    validate_content_path,
    validate_asset_path,
    validate_overlay_path,
    extract_user_id_from_overlay,
    convert_base_to_overlay,
    convert_overlay_to_base,
    is_overlay_path,
    is_content_path,
    is_asset_path,
    CONTENT_PATH_PATTERN,
    ASSET_PATH_PATTERN,
    OVERLAY_PATH_PATTERN,
)


class TestContentPathValidation:
    """Tests for validate_content_path() per FR-007."""

    def test_valid_simple_path(self):
        """Valid path with standard structure."""
        result = validate_content_path("content/01-intro/01-basics/01-hello.md")
        assert result.is_valid
        assert result.errors == []
        assert result.normalized_path == "content/01-intro/01-basics/01-hello.md"

    def test_valid_with_summary_suffix(self):
        """Valid path with .summary suffix."""
        result = validate_content_path("content/01-intro/02-setup/03-install.summary.md")
        assert result.is_valid
        assert result.errors == []

    def test_valid_with_hyphens_in_names(self):
        """Valid path with multiple hyphens in names."""
        result = validate_content_path("content/01-part-one/02-chapter-two/03-lesson-three.md")
        assert result.is_valid

    def test_valid_with_numbers_in_names(self):
        """Valid path with numbers in name portions."""
        result = validate_content_path("content/01-intro2python/01-ch1basics/01-lesson1start.md")
        assert result.is_valid

    def test_invalid_wrong_prefix(self):
        """Invalid: wrong prefix (lessons instead of content)."""
        result = validate_content_path("lessons/01-intro/01-basics/01-hello.md")
        assert not result.is_valid
        assert len(result.errors) > 0

    def test_invalid_single_digit_part(self):
        """Invalid: single digit instead of two digits."""
        result = validate_content_path("content/1-intro/01-basics/01-hello.md")
        assert not result.is_valid

    def test_invalid_missing_prefix_numbers(self):
        """Invalid: missing NN- prefix."""
        result = validate_content_path("content/intro/basics/hello.md")
        assert not result.is_valid

    def test_invalid_wrong_extension(self):
        """Invalid: wrong file extension."""
        result = validate_content_path("content/01-intro/01-basics/01-hello.txt")
        assert not result.is_valid

    def test_invalid_no_extension(self):
        """Invalid: no file extension."""
        result = validate_content_path("content/01-intro/01-basics/01-hello")
        assert not result.is_valid

    def test_invalid_double_summary(self):
        """Invalid: double .summary suffix."""
        result = validate_content_path("content/01-intro/01-basics/01-hello.summary.summary.md")
        assert not result.is_valid


class TestContentPathSecurityValidation:
    """Security-focused tests for path traversal and injection attacks."""

    def test_reject_path_traversal_dotdot(self):
        """Reject path traversal with .. (T014 security case)."""
        result = validate_content_path("content/01-intro/../../../etc/passwd")
        assert not result.is_valid
        assert any("traversal" in e.lower() for e in result.errors)

    def test_reject_path_traversal_in_middle(self):
        """Reject path traversal in middle of path."""
        result = validate_content_path("content/01-intro/01-basics/../../01-other/01-file.md")
        assert not result.is_valid

    def test_reject_null_byte(self):
        """Reject null byte injection (T014 security case)."""
        result = validate_content_path("content/01-intro/01-basics/01-hello.md\x00.txt")
        assert not result.is_valid
        assert any("null" in e.lower() for e in result.errors)

    def test_reject_absolute_path(self):
        """Reject absolute paths starting with /."""
        result = validate_content_path("/content/01-intro/01-basics/01-hello.md")
        assert not result.is_valid

    def test_reject_home_expansion(self):
        """Reject home directory expansion."""
        result = validate_content_path("~/content/01-intro/01-basics/01-hello.md")
        assert not result.is_valid

    def test_reject_newline_injection(self):
        """Reject newline characters in path."""
        result = validate_content_path("content/01-intro/01-basics/01-hello\n.md")
        assert not result.is_valid


class TestAssetPathValidation:
    """Tests for validate_asset_path() per FR-008."""

    def test_valid_img_path(self):
        """Valid image asset path."""
        result = validate_asset_path("static/images/diagram.png")
        assert result.is_valid

    def test_valid_slides_path(self):
        """Valid slides asset path."""
        result = validate_asset_path("static/slides/presentation.pdf")
        assert result.is_valid

    def test_valid_videos_path(self):
        """Valid videos asset path."""
        result = validate_asset_path("static/videos/lesson1.mp4")
        assert result.is_valid

    def test_valid_audio_path(self):
        """Valid audio asset path."""
        result = validate_asset_path("static/audio/podcast.mp3")
        assert result.is_valid

    def test_valid_nested_path(self):
        """Valid nested path within asset type."""
        result = validate_asset_path("static/images/chapter1/diagram.png")
        assert result.is_valid

    def test_invalid_wrong_prefix(self):
        """Invalid: wrong prefix (assets instead of static)."""
        result = validate_asset_path("assets/img/photo.jpg")
        assert not result.is_valid

    def test_invalid_unknown_asset_type(self):
        """Invalid: unknown asset type (docs not allowed)."""
        result = validate_asset_path("static/docs/readme.txt")
        assert not result.is_valid

    def test_invalid_path_traversal(self):
        """Reject path traversal in asset path."""
        result = validate_asset_path("static/img/../../../etc/passwd")
        assert not result.is_valid


class TestOverlayPathValidation:
    """Tests for validate_overlay_path() per FR-019."""

    def test_valid_overlay_path(self):
        """Valid overlay path with matching user_id."""
        result = validate_overlay_path(
            "users/user123/content/01-intro/01-basics/01-hello.md",
            expected_user_id="user123"
        )
        assert result.is_valid

    def test_valid_overlay_with_hyphen_userid(self):
        """Valid overlay with hyphenated user_id."""
        result = validate_overlay_path(
            "users/user-abc-123/content/01-intro/01-basics/01-hello.md",
            expected_user_id="user-abc-123"
        )
        assert result.is_valid

    def test_valid_overlay_with_underscore_userid(self):
        """Valid overlay with underscored user_id."""
        result = validate_overlay_path(
            "users/user_abc_123/content/01-intro/01-basics/01-hello.md",
            expected_user_id="user_abc_123"
        )
        assert result.is_valid

    def test_invalid_userid_mismatch(self):
        """Invalid: user_id in path doesn't match expected."""
        result = validate_overlay_path(
            "users/user123/content/01-intro/01-basics/01-hello.md",
            expected_user_id="otheruser"
        )
        assert not result.is_valid
        assert any("mismatch" in e.lower() for e in result.errors)

    def test_invalid_missing_users_prefix(self):
        """Invalid: missing users/ prefix."""
        result = validate_overlay_path(
            "overlay/user123/content/01-intro/01-basics/01-hello.md",
            expected_user_id="user123"
        )
        assert not result.is_valid

    def test_invalid_nested_content_path(self):
        """Invalid: nested content path is invalid."""
        result = validate_overlay_path(
            "users/user123/content/intro/basics/hello.md",  # Invalid nested content
            expected_user_id="user123"
        )
        assert not result.is_valid


class TestPathConversionUtilities:
    """Tests for path conversion functions."""

    def test_extract_userid_from_valid_overlay(self):
        """Extract user_id from valid overlay path."""
        user_id = extract_user_id_from_overlay("users/user123/content/01-intro/01-basics/01-hello.md")
        assert user_id == "user123"

    def test_extract_userid_from_non_overlay(self):
        """Return None for non-overlay path."""
        user_id = extract_user_id_from_overlay("content/01-intro/01-basics/01-hello.md")
        assert user_id is None

    def test_convert_base_to_overlay(self):
        """Convert base path to overlay path."""
        overlay = convert_base_to_overlay("content/01-intro/01-basics/01-hello.md", "user123")
        assert overlay == "users/user123/content/01-intro/01-basics/01-hello.md"

    def test_convert_base_to_overlay_invalid_base(self):
        """Raise error for invalid base path."""
        with pytest.raises(ValueError):
            convert_base_to_overlay("invalid/01-intro/01-basics/01-hello.md", "user123")

    def test_convert_overlay_to_base(self):
        """Convert overlay path back to base path."""
        base = convert_overlay_to_base("users/user123/content/01-intro/01-basics/01-hello.md")
        assert base == "content/01-intro/01-basics/01-hello.md"

    def test_convert_overlay_to_base_invalid(self):
        """Raise error for invalid overlay path."""
        with pytest.raises(ValueError):
            convert_overlay_to_base("content/01-intro/01-basics/01-hello.md")


class TestPathTypeCheckers:
    """Tests for is_*_path() functions."""

    def test_is_content_path_true(self):
        """Correctly identify content path."""
        assert is_content_path("content/01-intro/01-basics/01-hello.md")

    def test_is_content_path_false(self):
        """Correctly reject non-content path."""
        assert not is_content_path("static/img/photo.png")

    def test_is_asset_path_true(self):
        """Correctly identify asset path."""
        assert is_asset_path("static/images/photo.png")

    def test_is_asset_path_false(self):
        """Correctly reject non-asset path."""
        assert not is_asset_path("content/01-intro/01-basics/01-hello.md")

    def test_is_overlay_path_true(self):
        """Correctly identify overlay path."""
        assert is_overlay_path("users/user123/content/01-intro/01-basics/01-hello.md")

    def test_is_overlay_path_false(self):
        """Correctly reject non-overlay path."""
        assert not is_overlay_path("content/01-intro/01-basics/01-hello.md")
