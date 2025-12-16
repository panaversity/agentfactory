### Core Concept
Dependency management follows a lifecycle—add (express intent), update (adopt improvements), remove (clean unused)—with UV handling version resolution, lockfile updates, and conflict detection automatically, allowing developers to focus on expressing needs rather than memorizing constraints.

### Key Mental Models
- **Dependency Resolution Automation**: UV maps transitive dependencies, detects version conflicts, pins exact versions in lockfile automatically—developers rarely touch version constraint syntax
- **Production vs. Development Distinction**: Runtime dependencies ship with code; development tools (pytest, black) excluded from production, keeping deployments lean and secure
- **Semantic Versioning Contract**: MAJOR.MINOR.PATCH encoding breaking changes, features, fixes; version constraints (`^`, `>=`, `==`) express compatibility assumptions, delegated to AI for generation
- **Lockfile Reproducibility**: `uv.lock` pins exact versions (transitive included), enabling byte-for-byte environment matching across developers, branches, and deployments

### Critical Patterns
- **Transitive Dependency Transparency**: `uv add requests` installs 1 direct + 5 transitive; UV handles explosion automatically, students understand tree structure without memorizing individual packages
- **Conflict Resolution Options**: UV identifies incompatibilities (e.g., scikit-learn==1.0.0 needs numpy>=1.14.6, conflicting with numpy==1.20.0), presents solutions—students choose with AI guidance

### AI Collaboration Keys
- AI generates version constraints matching project requirements, handles complex conflict scenarios with tradeoff analysis
- AI explains dependency trees and why each transitive dependency is needed, building mental models

### Common Mistakes
- Memorizing version constraint syntax instead of asking AI (`^1.0.0` vs `>=1.0.0`)
- Updating dependencies without testing (breaking changes in major versions)
- Adding production dependencies that should be development-only (bloats deployments)
- Manually editing `uv.lock` (should be auto-regenerated only)

### Connections
- **Builds on**: Project creation, understanding dependencies, Python fundamentals
- **Leads to**: Running code in isolated environments, team collaboration with synchronized lockfiles, code quality tools
