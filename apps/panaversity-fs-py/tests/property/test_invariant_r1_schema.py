"""Property-based tests for schema enforcement (T054).

Tests R1 invariant: All content paths must match FR-007 schema pattern.
Uses hypothesis from_regex strategy for property-based testing.
"""

import pytest
from hypothesis import given, strategies as st, settings, HealthCheck, assume
from panaversity_fs.path_utils import (
    validate_content_path,
    validate_asset_path,
    CONTENT_PATH_PATTERN,
    ASSET_PATH_PATTERN,
)


# Hypothesis settings
HYPOTHESIS_SETTINGS = {
    "max_examples": 50,
    "deadline": None,
    "suppress_health_check": [HealthCheck.function_scoped_fixture, HealthCheck.too_slow]
}


# =============================================================================
# Strategies for Valid Paths (should PASS validation)
# =============================================================================

# Valid part/chapter/lesson number (00-99)
valid_number = st.integers(min_value=0, max_value=99).map(lambda n: f"{n:02d}")

# Valid name component (alphanumeric with hyphens, starts with alphanumeric)
valid_name = st.text(
    alphabet="abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-",
    min_size=1,
    max_size=30
).filter(lambda s: s[0].isalnum() and not s.endswith('-'))


@st.composite
def valid_content_path(draw):
    """Generate valid content paths matching FR-007 schema."""
    part_num = draw(valid_number)
    part_name = draw(valid_name)
    chapter_num = draw(valid_number)
    chapter_name = draw(valid_name)
    lesson_num = draw(valid_number)
    lesson_name = draw(valid_name)
    is_summary = draw(st.booleans())

    suffix = ".summary.md" if is_summary else ".md"
    return f"content/{part_num}-{part_name}/{chapter_num}-{chapter_name}/{lesson_num}-{lesson_name}{suffix}"


# Valid asset types (must match ASSET_PATH_PATTERN regex: images|slides|videos|audio)
valid_asset_type = st.sampled_from(["images", "slides", "videos", "audio"])

# Valid filename (alphanumeric with dots, hyphens, underscores)
# Filter out filenames that could trigger security checks (e.g., containing "..")
valid_filename = st.text(
    alphabet="abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.-_",
    min_size=1,
    max_size=50
).filter(lambda s: s[0].isalnum() and not s.startswith('.') and '..' not in s)


@st.composite
def valid_asset_path(draw):
    """Generate valid asset paths matching FR-008 schema."""
    asset_type = draw(valid_asset_type)
    filename = draw(valid_filename)
    return f"static/{asset_type}/{filename}"


# =============================================================================
# Strategies for Invalid Paths (should FAIL validation)
# =============================================================================

# Path traversal attacks
traversal_attacks = st.sampled_from([
    "../etc/passwd",
    "content/../../../etc/passwd",
    "content/01-Part/../../../secret",
    "..%2F..%2Fetc/passwd",
    "content/01-Part/01-Chapter/..%00/secret.md",
])

# Null byte injection
null_byte_attacks = st.sampled_from([
    "content/01-Part/01-Chapter/01-lesson\x00.md",
    "content/01-Part\x00/01-Chapter/01-lesson.md",
    "static/images/image\x00.png",
])

# Absolute paths
absolute_paths = st.sampled_from([
    "/etc/passwd",
    "/content/01-Part/01-Chapter/01-lesson.md",
    " /content/01-Part/01-Chapter/01-lesson.md",
])

# Invalid content path structure
invalid_content_paths = st.sampled_from([
    # Missing parts
    "content/01-lesson.md",
    "content/01-Part/01-lesson.md",
    # Wrong prefix
    "lessons/01-Part/01-Chapter/01-lesson.md",
    "docs/01-Part/01-Chapter/01-lesson.md",
    # Invalid numbering (missing leading zero)
    "content/1-Part/01-Chapter/01-lesson.md",
    "content/01-Part/1-Chapter/01-lesson.md",
    # Missing name after number
    "content/01/01-Chapter/01-lesson.md",
    "content/01-Part/01/01-lesson.md",
    # Wrong extension
    "content/01-Part/01-Chapter/01-lesson.txt",
    "content/01-Part/01-Chapter/01-lesson",
    # Invalid characters
    "content/01-Part/01-Chapter/01-lesson name.md",
    "content/01-Part/01-Chapter/01-lesson@special.md",
])

# Invalid asset path structure
invalid_asset_paths = st.sampled_from([
    # Wrong asset type
    "static/img/file.png",     # Should be 'images' not 'img'
    "static/video/file.mp4",   # Should be 'videos' not 'video'
    "static/sound/file.mp3",   # Should be 'audio' not 'sound'
    # Wrong prefix
    "assets/images/file.png",
    "public/images/file.png",
    # Missing filename
    "static/images/",
    "static/videos",
])


class TestSchemaEnforcementR1:
    """Property tests for R1: Schema enforcement.

    R1 invariant: All content paths must conform to FR-007 schema pattern.
    All asset paths must conform to FR-008 schema pattern.
    """

    @pytest.mark.asyncio
    @given(path=valid_content_path())
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_valid_content_paths_accepted(self, path):
        """R1: Valid content paths matching FR-007 should be accepted."""
        result = validate_content_path(path)
        assert result.is_valid, f"Valid path rejected: {path}, errors: {result.errors}"

    @pytest.mark.asyncio
    @given(path=valid_asset_path())
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_valid_asset_paths_accepted(self, path):
        """R1: Valid asset paths matching FR-008 should be accepted."""
        result = validate_asset_path(path)
        assert result.is_valid, f"Valid path rejected: {path}, errors: {result.errors}"

    @pytest.mark.asyncio
    @given(path=invalid_content_paths)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_invalid_content_paths_rejected(self, path):
        """R1: Invalid content paths should be rejected with clear errors."""
        result = validate_content_path(path)
        assert not result.is_valid, f"Invalid path accepted: {path}"
        assert len(result.errors) > 0, "No error message provided"

    @pytest.mark.asyncio
    @given(path=invalid_asset_paths)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_invalid_asset_paths_rejected(self, path):
        """R1: Invalid asset paths should be rejected with clear errors."""
        result = validate_asset_path(path)
        assert not result.is_valid, f"Invalid path accepted: {path}"
        assert len(result.errors) > 0, "No error message provided"

    @pytest.mark.asyncio
    @given(path=traversal_attacks)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_path_traversal_rejected(self, path):
        """R1/FR-009: Path traversal attacks should be rejected."""
        result = validate_content_path(path)
        assert not result.is_valid, f"Path traversal accepted: {path}"
        # Should have security-related error message
        assert any("traversal" in err.lower() or "security" in err.lower() for err in result.errors)

    @pytest.mark.asyncio
    @given(path=null_byte_attacks)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_null_byte_injection_rejected(self, path):
        """R1/FR-009: Null byte injection should be rejected."""
        result = validate_content_path(path)
        assert not result.is_valid, f"Null byte injection accepted: {path}"

    @pytest.mark.asyncio
    @given(path=absolute_paths)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_absolute_paths_rejected(self, path):
        """R1/FR-009: Absolute paths should be rejected."""
        result = validate_content_path(path)
        assert not result.is_valid, f"Absolute path accepted: {path}"


class TestContentPathComponents:
    """Test individual components of content paths."""

    @pytest.mark.asyncio
    @given(
        part_num=valid_number,
        part_name=valid_name,
        chapter_num=valid_number,
        chapter_name=valid_name,
        lesson_num=valid_number,
        lesson_name=valid_name
    )
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_all_valid_components_produce_valid_path(
        self, part_num, part_name, chapter_num, chapter_name, lesson_num, lesson_name
    ):
        """When all components are valid, the combined path should be valid."""
        path = f"content/{part_num}-{part_name}/{chapter_num}-{chapter_name}/{lesson_num}-{lesson_name}.md"
        result = validate_content_path(path)
        assert result.is_valid, f"Valid components produced invalid path: {path}, errors: {result.errors}"

    @pytest.mark.asyncio
    @given(is_summary=st.booleans())
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_summary_suffix_handled(self, is_summary):
        """Summary files (.summary.md) should be valid."""
        suffix = ".summary.md" if is_summary else ".md"
        path = f"content/01-Introduction/01-GettingStarted/01-welcome{suffix}"
        result = validate_content_path(path)
        assert result.is_valid, f"Path rejected: {path}, errors: {result.errors}"


class TestAssetPathComponents:
    """Test individual components of asset paths."""

    @pytest.mark.asyncio
    @given(asset_type=valid_asset_type, filename=valid_filename)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_all_valid_components_produce_valid_asset_path(self, asset_type, filename):
        """When all components are valid, the combined asset path should be valid."""
        path = f"static/{asset_type}/{filename}"
        result = validate_asset_path(path)
        assert result.is_valid, f"Valid components produced invalid path: {path}, errors: {result.errors}"

    @pytest.mark.asyncio
    @given(asset_type=valid_asset_type)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_all_asset_types_supported(self, asset_type):
        """All valid asset types (img, slides, videos, audio) should work."""
        path = f"static/{asset_type}/example.file"
        result = validate_asset_path(path)
        assert result.is_valid, f"Asset type {asset_type} rejected"


class TestPathNormalization:
    """Test path normalization behavior."""

    @pytest.mark.asyncio
    @given(path=valid_content_path())
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_valid_paths_return_normalized_form(self, path):
        """Valid paths should return a normalized form."""
        result = validate_content_path(path)
        assert result.is_valid
        assert result.normalized_path is not None
        # Normalized path should not have double slashes
        assert "//" not in result.normalized_path

    @pytest.mark.asyncio
    async def test_trailing_slashes_handled(self):
        """Paths with trailing slashes should be normalized."""
        path = "content/01-Part/01-Chapter/01-lesson.md"
        result = validate_content_path(path)
        assert result.is_valid
        assert not result.normalized_path.endswith("/")
