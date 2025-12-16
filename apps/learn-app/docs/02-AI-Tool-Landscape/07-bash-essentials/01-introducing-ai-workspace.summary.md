### Core Concept
Your AI companion has a current location in your file system, and knowing that location before it acts is the foundation of safe supervision—always ask "Where are you?" before letting it do anything.

### Key Mental Models
- **AI Has a Location**: Your AI isn't operating in abstract space—it's "standing" in a specific folder, and all actions happen relative to that position
- **Supervision Pattern**: Ask → Show Location → Show Contents → Verify → Execute (never let AI act without this sequence)
- **Same Commands, Different Computers**: AI runs identical commands to what you'd run; outputs differ only because you're in different places

### Critical Patterns
- `pwd` answers "Where am I?" by showing the current directory path
- `ls -la` answers "What can I see?" by listing files with details
- Reading `ls` output: lines starting with `d` are directories, lines starting with `-` are files
- Verification prompt before any operation: "Show me where we are and what I'll be affecting"

### Common Mistakes
- Letting AI act without first asking "Where are you?"—risking operations in the wrong folder
- Not verifying which directory will be affected before destructive operations like delete
- Trusting AI's description instead of reading the actual `pwd`/`ls` output yourself

### Connections
- **Builds on**: Basic terminal familiarity (opening a terminal, typing commands)
- **Leads to**: File system navigation with `cd` and understanding absolute vs relative paths
