### Core Concept
Package managers are like supplier networks—when you request one package, they automatically find and install all its dependencies (other packages it needs to work). You focus on WHAT you need; the package manager handles the complexity of finding everything required.

### Key Mental Models
- **Supplier Network Analogy**: Instead of making every piece yourself, you order from a supplier who sources from manufacturers—package managers do the same for software tools
- **Transitive Dependencies**: Packages depend on other packages, which depend on others. When you install `requests`, you get 5 packages; when you install `express`, you get 50+—this is normal, not an error
- **Language-Specific Managers**: Python uses `pip`, JavaScript/Node uses `npm`, macOS uses `brew`, Linux uses `apt`—you don't choose, each language has its standard manager

### Critical Patterns
- `pip install <package>` installs Python packages from PyPI
- `pip show <package>` verifies installation and displays version, location, and dependencies
- `pip list | grep <package>` confirms a package is available
- `npm install <package>` installs Node.js packages and creates `package-lock.json` for version consistency
- Verification through importing: `python3` → `import requests` → if no error, it's installed

### Common Mistakes
- Thinking 50+ packages installing is an error—it's normal dependency resolution
- Trying to memorize which package manager to use—AI knows from project context
- Skipping verification after installation—always confirm packages are available
- Not understanding why installation "failed"—often a missing dependency or version conflict

### Connections
- **Builds on**: Terminal navigation (Lesson 1-3), file permissions and operations (Lesson 5)
- **Leads to**: Virtual environments and project isolation (future lessons), managing requirements.txt and package.json files
