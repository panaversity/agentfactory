#!/usr/bin/env bash
# Common functions and variables for all scripts

# Get repository root, with fallback for non-git repositories
get_repo_root() {
    if git rev-parse --show-toplevel >/dev/null 2>&1; then
        git rev-parse --show-toplevel
    else
        # Fall back to script location for non-git repos
        local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        (cd "$script_dir/../../.." && pwd)
    fi
}

# Get current branch, with fallback for non-git repositories
get_current_branch() {
    # First check if SPECIFY_FEATURE environment variable is set
    if [[ -n "${SPECIFY_FEATURE:-}" ]]; then
        echo "$SPECIFY_FEATURE"
        return
    fi

    # Then check git if available
    if git rev-parse --abbrev-ref HEAD >/dev/null 2>&1; then
        git rev-parse --abbrev-ref HEAD
        return
    fi

    # For non-git repos, try to find the latest feature directory
    local repo_root=$(get_repo_root)
    local specs_dir="$repo_root/specs"

    if [[ -d "$specs_dir" ]]; then
        local latest_feature=""
        local highest=0

        for dir in "$specs_dir"/*; do
            if [[ -d "$dir" ]]; then
                local dirname=$(basename "$dir")
                if [[ "$dirname" =~ ^([0-9]{3})- ]]; then
                    local number=${BASH_REMATCH[1]}
                    number=$((10#$number))
                    if [[ "$number" -gt "$highest" ]]; then
                        highest=$number
                        latest_feature=$dirname
                    fi
                fi
            fi
        done

        if [[ -n "$latest_feature" ]]; then
            echo "$latest_feature"
            return
        fi
    fi

    echo "main"  # Final fallback
}

# Check if we have git available
has_git() {
    git rev-parse --show-toplevel >/dev/null 2>&1
}

check_feature_branch() {
    local branch="$1"
    local has_git_repo="$2"

    # For non-git repos, we can't enforce branch naming but still provide output
    if [[ "$has_git_repo" != "true" ]]; then
        echo "[specify] Warning: Git repository not detected; skipped branch validation" >&2
        return 0
    fi

    # Accept BOTH old and new naming conventions
    # NEW: part-N-chapter-M (e.g., part-4-chapter-15)
    # OLD: NNN-feature-name (e.g., 015-operators-keywords-variables)
    if [[ "$branch" =~ ^part-[0-9]+-chapter-[0-9]+$ ]]; then
        # New convention - valid
        return 0
    elif [[ "$branch" =~ ^[0-9]{3}- ]]; then
        # Old convention - valid
        return 0
    elif [[ "$branch" == "main" ]]; then
        # On main branch - allow (commands will create feature branch in Phase 1)
        return 0
    else
        echo "ERROR: Not on a feature branch. Current branch: $branch" >&2
        echo "Feature branches should be named like:" >&2
        echo "  - NEW convention: part-4-chapter-15" >&2
        echo "  - OLD convention: 015-feature-name" >&2
        return 1
    fi
}

get_feature_dir() { echo "$1/specs/$2"; }

# Find feature directory by branch name - supports BOTH old and new naming conventions
# OLD: 015-operators-keywords-variables → specs/015-operators-keywords-variables/
# NEW: part-4-chapter-15 → specs/part-4-chapter-15/
find_feature_dir_by_prefix() {
    local repo_root="$1"
    local branch_name="$2"
    local specs_dir="$repo_root/specs"

    # NEW CONVENTION: part-N-chapter-M pattern
    if [[ "$branch_name" =~ ^part-([0-9]+)-chapter-([0-9]+)$ ]]; then
        local part="${BASH_REMATCH[1]}"
        local chapter="${BASH_REMATCH[2]}"
        local spec_dir="$specs_dir/part-$part-chapter-$chapter"

        # Check if directory exists
        if [[ -d "$spec_dir" ]]; then
            echo "$spec_dir"
            return
        else
            # Directory doesn't exist yet - return expected path (will be created by /sp.specify)
            echo "$spec_dir"
            return
        fi
    fi

    # OLD CONVENTION: Extract numeric prefix from branch (e.g., "015" from "015-whatever")
    if [[ "$branch_name" =~ ^([0-9]{3})- ]]; then
        local prefix="${BASH_REMATCH[1]}"

        # Search for directories in specs/ that start with this prefix
        local matches=()
        if [[ -d "$specs_dir" ]]; then
            for dir in "$specs_dir"/"$prefix"-*; do
                if [[ -d "$dir" ]]; then
                    matches+=("$(basename "$dir")")
                fi
            done
        fi

        # Handle results
        if [[ ${#matches[@]} -eq 0 ]]; then
            # No match found - return the branch name path (will fail later with clear error)
            echo "$specs_dir/$branch_name"
        elif [[ ${#matches[@]} -eq 1 ]]; then
            # Exactly one match - perfect!
            echo "$specs_dir/${matches[0]}"
        else
            # Multiple matches - this shouldn't happen with proper naming convention
            echo "ERROR: Multiple spec directories found with prefix '$prefix': ${matches[*]}" >&2
            echo "Please ensure only one spec directory exists per numeric prefix." >&2
            echo "$specs_dir/$branch_name"  # Return something to avoid breaking the script
        fi
        return
    fi

    # FALLBACK: Exact branch name match (for non-standard branches or direct spec dir names)
    echo "$specs_dir/$branch_name"
}

get_feature_paths() {
    local repo_root=$(get_repo_root)
    local current_branch=$(get_current_branch)
    local has_git_repo="false"

    if has_git; then
        has_git_repo="true"
    fi

    # Use prefix-based lookup to support multiple branches per spec
    local feature_dir=$(find_feature_dir_by_prefix "$repo_root" "$current_branch")

    cat <<EOF
REPO_ROOT='$repo_root'
CURRENT_BRANCH='$current_branch'
HAS_GIT='$has_git_repo'
FEATURE_DIR='$feature_dir'
FEATURE_SPEC='$feature_dir/spec.md'
IMPL_PLAN='$feature_dir/plan.md'
TASKS='$feature_dir/tasks.md'
RESEARCH='$feature_dir/research.md'
DATA_MODEL='$feature_dir/data-model.md'
QUICKSTART='$feature_dir/quickstart.md'
CONTRACTS_DIR='$feature_dir/contracts'
EOF
}

check_file() { [[ -f "$1" ]] && echo "  ✓ $2" || echo "  ✗ $2"; }
check_dir() { [[ -d "$1" && -n $(ls -A "$1" 2>/dev/null) ]] && echo "  ✓ $2" || echo "  ✗ $2"; }

