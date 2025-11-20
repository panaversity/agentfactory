---
sidebar_position: 8
title: "Chapter 7: Bash Essentials for AI-Driven Development Quiz"
---

# Chapter 7: Bash Essentials for AI-Driven Development Quiz

Test your understanding of bash fundamentals through the lens of AI collaboration. This assessment focuses on conceptual understanding of file system navigation, safety patterns, and effective dialogue with AI tools rather than memorizing commands.

<Quiz
  title="Chapter 7: Bash Essentials Assessment"
  questions={[
    {
      question: "Your AI suggests running `rm -rf backup/` to delete old files. Following the safety-first pattern, what should happen BEFORE this command executes?",
      options: [
        "AI should explain the command, you verify the folder location and contents, then execute",
        "AI should immediately execute and show you the results afterward",
        "You should memorize what rm -rf does before asking AI",
        "AI should create a backup of the backup folder first"
      ],
      correctOption: 0,
      explanation: "The safety-first pattern requires verification BEFORE execution. AI should explain what rm -rf does (remove recursively and forcefully), show you the current directory with pwd, list the backup folder contents with ls, and wait for your confirmation. Only then should it execute. Option B (execute first) violates the safety pattern by acting before verification. Option C misunderstands AI-native development—you don't need to memorize commands; you need to understand what AI is doing. Option D addresses the wrong problem—backing up the backup doesn't help verify you're deleting the right folder. The correct approach is: Ask → Explain → Understand → Verify (check location and contents) → Execute. This prevents accidentally deleting the wrong folder or losing important files. Source: Lesson 2",
      source: "Lesson 2: The Safety-First Dialogue Pattern"
    },
    {
      question: "When your AI shows `pwd` output as `/Users/mjs/Documents/projects/my-app`, what does this path tell you about the AI's current context?",
      options: [
        "AI is in my-app folder within projects, which is inside Documents",
        "AI can only access files named my-app",
        "AI is confused and needs to navigate to a different location",
        "AI has created a new folder called my-app for you"
      ],
      correctOption: 0,
      explanation: "The pwd output shows AI's working directory as an absolute path. Reading from left to right, AI is in the my-app folder, which is inside projects, which is inside Documents, which is inside mjs's home directory under Users. This tells you exactly where AI is 'standing' in the file system—all subsequent relative path operations will be relative to this location. Option B is incorrect—pwd shows location, not file access restrictions. Option C misunderstands what pwd does—it's not showing an error, just reporting current location. Option D is wrong—pwd displays existing location; it doesn't create folders. Understanding AI's working directory is crucial because it affects which files AI can access with relative paths and prevents mistakes like operating on the wrong folder. Source: Lesson 1",
      source: "Lesson 1: Introducing Your AI Companion's Workspace"
    },
    {
      question: "You ask your AI to 'organize Python files into a scripts folder.' The AI responds: 'I'll run mkdir scripts && mv *.py scripts/' but then reports it cannot execute commands directly. What does this reveal about AI collaboration?",
      options: [
        "AI can guide and explain commands, but you may need to execute them yourself",
        "The AI is broken and should be able to run all commands",
        "You should switch to a different AI tool immediately",
        "AI should never suggest commands it cannot execute on its own"
      ],
      correctOption: 0,
      explanation: "This scenario (adapted from real Gemini CLI dialogue in Lesson 2) demonstrates that AI collaboration is not about AI doing everything automatically. Some AI tools guide you through commands rather than executing directly on your system. This is actually a safety feature—you maintain control over what runs on your machine. The AI explains WHAT to do and WHY, you verify it makes sense, then YOU execute the commands. Option B is incorrect—different AI tools have different execution capabilities; this doesn't mean they're broken. Option C is unnecessary—guidance is valuable even without direct execution. Option D misunderstands the collaboration model—AI suggesting commands you execute is a valid and often safer workflow. Real AI-native development involves AI as teacher/coworker, not just as an automated script runner. Source: Lesson 2",
      source: "Lesson 2: The Safety-First Dialogue Pattern"
    },
    {
      question: "Your AI runs `ls -la` and shows several items starting with 'd' and others starting with '-'. What critical information does this output provide before file operations?",
      options: [
        "Items with d are directories, items with dash are files, helping distinguish folder versus file operations",
        "The d indicates files that are ready to delete safely",
        "The dash means files are hidden and should not be accessed",
        "This output shows file permissions and should be ignored for navigation"
      ],
      correctOption: 0,
      explanation: "The first character in ls -la output distinguishes file types. 'd' indicates directories (folders), while '-' indicates regular files. This distinction is crucial BEFORE performing operations—copying a folder requires cp -r (recursive flag), while copying a file uses cp alone. Deleting a folder versus a file also requires different awareness. Option B is incorrect—'d' has nothing to do with deletion safety; it just marks directories. Option C is wrong—hidden files start with a dot in their name, not with a dash in the permissions field. Option D misunderstands the output—while the string does show permissions (rwxr-xr-x), the first character specifically identifies file type, which is essential for navigation and operations. Recognizing d versus - helps you supervise AI operations by knowing whether AI is operating on folders or files. Source: Lesson 1",
      source: "Lesson 1: Introducing Your AI Companion's Workspace"
    },
    {
      question: "According to the 5-step safety pattern, you're at Step 4 (Verify) before your AI moves files. Which question demonstrates the BEST safety verification?",
      options: [
        "Show me which specific files will be moved and what the destination folder currently contains",
        "Is this the fastest way to move these files?",
        "Why did you choose to use mv instead of cp for this task?",
        "What operating system are you running this command on?"
      ],
      correctOption: 0,
      explanation: "Step 4 (Verify) focuses on safety-critical questions that prevent data loss or mistakes. Asking AI to show WHICH files will be affected and WHAT already exists in the destination prevents overwriting important files or moving the wrong items. This is a concrete verification step with actionable output. Option B (fastest way) is about efficiency, not safety—it's a philosophical question that doesn't prevent mistakes. Option C (why mv instead of cp) is about understanding alternatives, which is useful but doesn't verify the CURRENT operation's safety. Option D (operating system) is tangential information that doesn't help you verify whether the right files are being moved to the right place. Good Step 4 questions focus on: What exactly will be affected? Can I undo this? Are there conflicts? Show me before you act. Source: Lesson 2",
      source: "Lesson 2: The Safety-First Dialogue Pattern"
    },
    {
      question: "Your AI is in `/Users/mjs/projects` and runs `cd app/src`. After this, what will `pwd` show, and why does this matter?",
      options: [
        "It will show /Users/mjs/projects/app/src because cd used a relative path from the current location",
        "It will show app/src because cd always shows just the folder you navigated to",
        "It will show /app/src because AI navigated to a new root folder",
        "It will show an error because you cannot use cd with two folder levels"
      ],
      correctOption: 0,
      explanation: "When cd uses a relative path (no leading /), it navigates FROM the current directory. Starting from /Users/mjs/projects and running cd app/src moves into the app subfolder, then into src within app, resulting in /Users/mjs/projects/app/src. This full path will be shown by pwd. Understanding this matters because it shows how relative paths build on your current location. Option B is incorrect—pwd always shows the complete absolute path, not just the last folder name. Option C misunderstands relative vs absolute paths—app/src without a leading / is relative, not a new root path. Option D is wrong—cd accepts multi-level relative paths like app/src or even app/src/components. Knowing how relative paths resolve helps you track where AI is navigating and verify it's in the correct location before operations. Source: Lesson 3",
      source: "Lesson 3: Understanding File Navigation Through Dialogue"
    },
    {
      question: "You ask your AI to copy a folder called `data/` to `data-backup/`. The AI suggests `cp data data-backup`. What's wrong with this command, and what should happen instead?",
      options: [
        "The command needs cp -r for recursive folder copying; AI should explain the -r flag is required",
        "Nothing is wrong; cp works the same for files and folders without any difference",
        "AI should use mv instead of cp because folders cannot be copied at all",
        "The command needs cp data/ data-backup to include the trailing slashes for folders"
      ],
      correctOption: 0,
      explanation: "Copying folders requires the -r (recursive) flag because folders contain other items (files and subfolders) that must be copied too. cp data data-backup without -r will fail with an error like 'data is a directory (not copied).' The correct command is cp -r data data-backup. AI should explain that folders need special handling and use -r to copy everything inside recursively. Option B is incorrect—cp without flags only works for individual files, not directories. Option C is wrong—folders CAN be copied; you just need the right flag. mv would MOVE (not copy) the folder, which is different from creating a backup. Option D misunderstands bash syntax—trailing slashes are optional and don't affect whether cp works; the -r flag is what matters. Understanding when operations need flags (like -r for folders) helps you verify AI is using the right command structure. Source: Lesson 4",
      source: "Lesson 4: Understanding File Operations Safely"
    },
    {
      question: "Your project needs an API key. You're deciding between `export API_KEY='sk-123'` in terminal or storing it in a `.env` file. What's the critical difference that affects your choice?",
      options: [
        "Export is temporary and disappears when terminal closes; .env file persists across terminal sessions",
        "Export is more secure than .env files because it does not write to disk at all",
        "There is no difference; both approaches work identically in all situations",
        ".env files are only for production environments; export is for development work"
      ],
      correctOption: 0,
      explanation: "The export command sets an environment variable for the current terminal session only. When you close the terminal, the variable is gone. A .env file writes the configuration to disk, so it persists across terminal restarts and can be loaded by your application consistently. This makes .env files better for project-specific configuration that needs to be available every time you work on the project. Option B is incorrect—export and .env have similar security properties; both can be secure or insecure depending on how you protect them (.env needs .gitignore). Option C is wrong—persistence is a major difference that affects workflow. Option D misunderstands the use case—.env files are used in both development and production; the key is ensuring .env is in .gitignore so secrets don't reach version control. Understanding persistence helps you choose the right configuration approach for your workflow. Source: Lesson 5",
      source: "Lesson 5: Understanding Configuration and Secrets Safely"
    },
    {
      question: "You hardcoded `API_KEY = 'sk-abc123'` directly in your Python code and pushed it to GitHub. Your AI warns this is dangerous. What specific risk does this create?",
      options: [
        "Anyone with access to your GitHub repository can see and use your API key to access your account",
        "The API key will stop working immediately when it appears in code files",
        "Python cannot read API keys from code files and will throw syntax errors always",
        "GitHub will automatically delete your repository when it detects hardcoded secrets in any code"
      ],
      correctOption: 0,
      explanation: "When you hardcode secrets in code and push to version control, anyone who can access your repository—including collaborators, future employers looking at your portfolio, or malicious actors if it's public—can see your API key. They can use it to access your account, rack up charges, or steal data. This is a critical security vulnerability. Option B is incorrect—API keys don't stop working just because they're in code; they continue to work (which is the problem—others can use them too). Option C is wrong—Python reads hardcoded strings just fine; the issue is security, not syntax. Option D overstates the consequence—GitHub may warn you about exposed secrets, but doesn't automatically delete repositories. The real risk is unauthorized access to your services. The solution is storing secrets in environment variables or .env files (with .env in .gitignore) and reading them with os.getenv(). Source: Lesson 5",
      source: "Lesson 5: Understanding Configuration and Secrets Safely"
    },
    {
      question: "Your AI runs `pip install requests` and shows that 5 packages were installed even though you only asked for 1. What does this reveal about package management?",
      options: [
        "Package managers automatically install dependencies, so requests needs 4 other packages to function properly and pip installs them all",
        "The AI made a mistake and installed extra packages you do not actually need for your project",
        "pip always installs 5 packages minimum regardless of what you request to ensure system stability works",
        "The extra 4 packages are backups in case requests fails to install correctly the first time"
      ],
      correctOption: 0,
      explanation: "When you install a package, the package manager resolves its dependency tree. requests depends on certifi, charset-normalizer, idna, and urllib3. pip automatically installs all of these because requests won't work without them. This is called transitive dependency resolution—dependencies of dependencies are handled automatically. This saves you from manually tracking down and installing every required package. Option B is incorrect—pip is working correctly, not making a mistake. Dependencies are necessary, not extra. Option C is wrong—pip doesn't have a minimum package count; it installs exactly what's needed. If a package has no dependencies, pip installs just that one package. Option D misunderstands dependencies—they're not backups; they're required functional components. Understanding dependency resolution helps you recognize that installing packages is rarely a one-to-one operation and verify all dependencies installed successfully. Source: Lesson 6",
      source: "Lesson 6: Understanding Dependencies and Packages"
    },
    {
      question: "You're working on a Python project and need to install a JavaScript library. Your AI refuses and says it needs npm instead of pip. What does this reveal about package managers?",
      options: [
        "Different languages have different package managers; Python uses pip, JavaScript uses npm, and they manage different package ecosystems entirely",
        "The AI is malfunctioning because pip should be able to install any type of package regardless of language",
        "You can install JavaScript packages with pip by adding the --js flag to the install command",
        "npm and pip are the same tool with different names depending on operating system you are using"
      ],
      correctOption: 0,
      explanation: "Package managers are language-specific. pip accesses PyPI (Python Package Index) and installs Python packages. npm accesses the npm registry and installs JavaScript/Node.js packages. They manage completely different ecosystems and cannot install each other's packages. Python projects use pip; JavaScript projects use npm; macOS system tools use brew; Linux uses apt. Your AI recognizes your project context and uses the appropriate package manager. Option B is incorrect—pip only works with Python packages; it can't install JavaScript libraries. Option C is wrong—there is no --js flag for pip. Option D misunderstands package managers—npm and pip are different tools for different languages, not the same tool with different names. Understanding that package managers are language-specific helps you verify AI is using the right tool for your project type. Source: Lesson 6",
      source: "Lesson 6: Understanding Dependencies and Packages"
    },
    {
      question: "Your AI suggests the command `ls -la | grep '.py' | wc -l` to count Python files. Trace the data flow through this pipeline—what happens at each step?",
      options: [
        "ls lists all files, grep filters to lines containing .py, wc -l counts the filtered lines giving Python file count",
        "ls counts files, grep searches for Python, wc displays the results in a formatted table",
        "The pipe combines all three commands into one operation that runs simultaneously on all files at once",
        "ls finds Python files first, grep verifies they are valid, wc checks their size and reports total bytes"
      ],
      correctOption: 0,
      explanation: "Pipes connect commands sequentially, passing output from one as input to the next. ls -la produces a complete file listing (all files, all details). The pipe | sends this output to grep '.py', which filters to only lines containing '.py' (Python files). The second pipe sends those filtered lines to wc -l, which counts them. Result: the count of Python files. This is assembly-line data processing. Option B is incorrect—ls doesn't count, grep doesn't search file contents (it filters lines), and wc -l counts lines, not formatting output. Option C misunderstands pipes—commands run in sequence (output → filter → count), not simultaneously. Option D is wrong about what each command does—ls lists all files (not just Python), grep filters text (doesn't validate files), and wc -l counts lines (not sizes). Understanding data flow through pipes helps you predict output and verify AI's pipeline logic makes sense. Source: Lesson 7",
      source: "Lesson 7: Understanding Pipes and Complex Commands"
    },
    {
      question: "You ask your AI to find errors in a 10,000-line log file. It suggests `cat application.log | grep 'ERROR' | wc -l`. What would you modify to see the actual error messages instead of just counting them?",
      options: [
        "Remove the wc -l part to stop at grep, which will display all error lines instead of counting",
        "Add a --show flag to wc -l to display lines while counting them simultaneously",
        "Replace grep with cat to show all log lines including the errors mixed with other messages",
        "Change the pipe to && so all commands run independently showing different output each time"
      ],
      correctOption: 0,
      explanation: "The pipeline cat application.log | grep 'ERROR' | wc -l counts error lines. To see the actual error messages, stop the pipeline before wc -l. Running cat application.log | grep 'ERROR' (without the count step) will output all lines containing 'ERROR' to your terminal. The final command in a pipeline determines what you see—wc -l shows a count, grep shows matching lines. By removing wc -l, you change the output from a number to the actual error text. Option B is incorrect—wc -l doesn't have a --show flag; it only counts. Option C is wrong—replacing grep with cat would show ALL 10,000 lines, not just errors. Option D misunderstands operators— && runs commands sequentially based on success, not in a data flow pipeline. Understanding how to modify pipelines by adding/removing stages helps you adapt AI suggestions to get exactly the output you need. Source: Lesson 7",
      source: "Lesson 7: Understanding Pipes and Complex Commands"
    },
    {
      question: "The 5-step safety pattern includes Ask, Explain, Understand, Verify, Execute. Why is 'Verify' positioned BEFORE 'Execute' instead of checking results afterward?",
      options: [
        "Verification before execution prevents destructive mistakes that cannot be undone; checking after is too late for dangerous operations",
        "Verification after execution is more accurate because you can see what actually happened in real results",
        "The order does not matter since you can undo any operation in bash with the undo command",
        "Execute must come last in the sequence for grammatical reasons to make the acronym work properly"
      ],
      correctOption: 0,
      explanation: "Verify comes before Execute because some operations are destructive and irreversible. If AI runs rm -rf backup/ and deletes the wrong folder, checking afterward doesn't help—your files are gone. Verification means asking questions like 'Show me what's in that folder' and 'Are you sure this is the right location?' BEFORE deletion. This prevents mistakes. Option B misunderstands the purpose—verification isn't about accuracy of results; it's about preventing dangerous operations from running in the first place. Option C is incorrect—bash has no general undo command. Deleted files are very difficult to recover. Option D is irrelevant—the order is about safety, not acronyms. Real-world example: Verifying before a git push --force prevents overwriting team members' work. Verifying before rm -rf prevents data loss. The pattern prioritizes prevention over recovery because recovery is often impossible. Source: Lesson 2",
      source: "Lesson 2: The Safety-First Dialogue Pattern"
    },
    {
      question: "Your AI is in `/Users/mjs/projects` and you want it to navigate to your home directory `/Users/mjs`. What's the MOST RELIABLE command and why?",
      options: [
        "cd ~ because tilde is a shortcut to home directory that works regardless of current location",
        "cd .. because it always goes up one level to the parent directory which is home",
        "cd /Users/mjs because absolute paths are always longer and therefore more reliable than shortcuts",
        "cd home because home is a universal bash keyword that navigates to home from anywhere"
      ],
      correctOption: 0,
      explanation: "cd ~ is most reliable because ~ is a bash shortcut that always refers to your home directory, regardless of where you currently are. From /Users/mjs/projects or /var/log or anywhere else, cd ~ always takes you home. This is an absolute navigation that doesn't depend on your current location. Option B is incorrect—cd .. goes up ONE level. From /Users/mjs/projects, that goes to /Users/mjs (which happens to be home in this case), but from /Users/mjs/projects/app/src, cd .. would only go to /Users/mjs/projects/app, not home. Option C is wrong—absolute paths aren't more reliable because they're longer; they're reliable because they specify a complete location. But you'd need to know your exact home path (/Users/mjs vs /home/mjs on Linux). ~ works everywhere. Option D is incorrect—'home' is not a bash keyword. Understanding shortcuts like ~ helps you navigate confidently without knowing absolute paths. Source: Lesson 3",
      source: "Lesson 3: Understanding File Navigation Through Dialogue"
    },
    {
      question: "You run `cd ..` three times in a row from `/Users/mjs/projects/app/src`. Where do you end up, and what does this teach about relative navigation?",
      options: [
        "You end up at /Users/mjs because each cd .. moves up one level toward the root directory",
        "You end up back at /Users/mjs/projects/app/src because cd .. reverses the previous cd command",
        "You get an error because cd .. can only be used once per terminal session for safety",
        "You end up at / because cd .. always goes to the root directory when repeated"
      ],
      correctOption: 0,
      explanation: "Each cd .. moves up one level in the directory tree. Starting from /Users/mjs/projects/app/src: first cd .. → /Users/mjs/projects/app, second cd .. → /Users/mjs/projects, third cd .. → /Users/mjs. You move up toward the root directory with each execution. This demonstrates that relative navigation is cumulative—each command is relative to your new location after the previous command. Option B is incorrect—cd .. doesn't reverse or undo; it navigates up one level each time. Option C is wrong—there's no limit on using cd ..; you can run it as many times as needed. Option D misunderstands the behavior—cd .. moves up ONE level per execution, not all the way to root. To reach root (/) you'd need to cd .. many more times (or use cd /). Understanding cumulative relative navigation helps you track your location through multiple commands. Source: Lesson 3",
      source: "Lesson 3: Understanding File Navigation Through Dialogue"
    },
    {
      question: "You want to delete a folder called `old-backup` but first need to verify it's the right one. Following the safety pattern, what dialogue sequence should occur?",
      options: [
        "AI shows pwd and ls to confirm location and folder contents, you verify it is the correct folder, then AI executes rm -rf after your confirmation",
        "AI immediately runs rm -rf old-backup and shows you the success message afterward for confirmation it worked correctly",
        "AI creates a backup of old-backup first, then deletes the original, then asks if you want to keep the backup created",
        "You manually memorize the folder contents before asking AI to delete so you can verify it deleted correctly"
      ],
      correctOption: 0,
      explanation: "The safety pattern for destructive operations requires: (1) AI shows current location with pwd, (2) AI lists old-backup contents with ls -la old-backup/, (3) You review what will be deleted, (4) You explicitly confirm, (5) AI executes rm -rf old-backup. This prevents deleting the wrong folder. Option B violates the safety pattern by executing before verification—if old-backup contains important files or AI is in the wrong directory, it's too late. Option C addresses the wrong problem—backing up something you intend to delete is contradictory. The issue isn't creating backups; it's verifying you're deleting the right thing. Option D misunderstands AI-native workflow—you don't need to memorize; AI shows you, and you verify visually before proceeding. The key is verification BEFORE execution for irreversible operations. Source: Lesson 4",
      source: "Lesson 4: Understanding File Operations Safely"
    },
    {
      question: "Your AI suggests `mv project/ project-backup/` to rename a folder. Later you realize you wanted a COPY, not a move. What's the problem with mv, and what should you do differently?",
      options: [
        "mv relocates the original folder leaving nothing at the old location; you should use cp -r to create a copy instead",
        "mv is correct because renaming and copying are the same operation in bash with identical results for folders",
        "You can undo mv by running mv project-backup/ project/ which restores the original location with no data loss",
        "mv is safer than cp because it prevents having duplicate folders with the same name in different locations"
      ],
      correctOption: 0,
      explanation: "mv (move) relocates the folder, changing its location or name. After mv project/ project-backup/, the folder exists ONLY as project-backup; there's no copy left at project/. If you wanted both folders, you needed cp -r project/ project-backup/, which copies the folder and all its contents while leaving the original in place. This is a critical distinction—mv is destructive to the original location. Option B is incorrect—renaming (mv) and copying (cp) are fundamentally different. mv leaves one copy; cp creates two. Option C is partially true—you can mv back, but this misses the point: if you wanted BOTH a backup AND the original, mv doesn't accomplish that. You'd have to mv back and start over with cp. Option D is irrelevant—safety isn't about preventing duplicates; it's about understanding what operation you actually need. Understanding mv vs cp prevents data organization mistakes. Source: Lesson 4",
      source: "Lesson 4: Understanding File Operations Safely"
    },
    {
      question: "You created a `.env` file with secrets and pushed it to GitHub by mistake. Your AI suggests adding `.env` to `.gitignore` now. Does this fix the problem, and why or why not?",
      options: [
        "No, this does not fix it because the .env file is already in Git history and publicly visible; you must rotate your secrets and remove the file from history",
        "Yes, adding to .gitignore immediately hides the file and removes it from GitHub automatically fixing the security issue for you",
        "No, because .gitignore only works for public repositories and has no effect on private repositories with any secrets",
        "Yes, .gitignore prevents the file from being accessed by anyone who clones the repository going forward solving the problem"
      ],
      correctOption: 0,
      explanation: "Adding a file to .gitignore only prevents FUTURE commits from including it. If .env is already committed and pushed, it exists in Git history and is publicly visible on GitHub. Anyone can check out previous commits or view file history to see your secrets. The fix requires: (1) Remove .env from Git history using git filter-branch or BFG Repo-Cleaner, (2) Rotate (change) all exposed secrets immediately, (3) Add .env to .gitignore, (4) Commit and push. Option B is incorrect—.gitignore doesn't hide or remove existing tracked files; it only ignores untracked files. Option C is wrong—.gitignore works identically for public and private repos; the difference is visibility, not functionality. Option D misunderstands the problem—even if new clones don't get .env, the secrets are still in history. Understanding Git's immutable history prevents a false sense of security from .gitignore alone. Source: Lesson 5",
      source: "Lesson 5: Understanding Configuration and Secrets Safely"
    },
    {
      question: "Your AI runs `pip show requests` and the output includes 'Requires: charset-normalizer, idna, urllib3, certifi'. What does this tell you, and why does it matter?",
      options: [
        "This shows requests depends on 4 other packages; if any are missing or wrong version, requests will not function properly",
        "This shows requests is redundant and you only need to keep one of these 5 packages to save disk space",
        "This shows pip made an error and installed extra packages you should manually uninstall to clean up your environment",
        "This shows the packages that requests can optionally use but are not required for basic functionality to work"
      ],
      correctOption: 0,
      explanation: "The 'Requires' field lists requests' direct dependencies—packages it needs to function. If charset-normalizer is missing or incompatible, requests will fail when you try to use it. pip show reveals the dependency relationship, helping you understand that requests isn't a standalone package. This matters for troubleshooting—if requests throws an error, the problem might be in one of its dependencies. Option B is incorrect—you cannot delete these packages without breaking requests; they're all necessary. Option C misunderstands dependencies—pip installed these correctly; they're not mistakes. Option D is wrong about 'optional'—'Requires' means mandatory dependencies. If they were optional, the field would say 'Suggests' or 'Recommends'. Understanding the Requires field helps you diagnose dependency conflicts and verify complete installation. Source: Lesson 6",
      source: "Lesson 6: Understanding Dependencies and Packages"
    },
    {
      question: "Your Python project has 50 dependencies. You run `pip install -r requirements.txt` and 200 packages are installed. Your AI says this is normal. Why?",
      options: [
        "The 50 packages you requested each have their own dependencies, creating transitive dependencies that pip automatically resolves and installs",
        "pip always installs 4 times as many packages as requested to ensure backup copies exist for redundancy and stability",
        "The requirements.txt file is corrupt and pip installed extra packages by mistake that should be manually removed from environment",
        "Python requires 150 system packages by default and these are added to your 50 creating the total of 200 packages installed"
      ],
      correctOption: 0,
      explanation: "Transitive dependencies are dependencies of dependencies. Your 50 packages each have their own dependencies, which have their own dependencies, creating a tree. Package A might need B and C. B needs D and E. C needs E (already installed) and F. pip traverses this entire tree and installs all 200 packages needed. This is automatic dependency resolution—you don't track each package manually. Option B is incorrect—pip doesn't install backups or redundant copies; each package is installed once. Option C misunderstands package management—this is normal behavior, not corruption. Option D is wrong—Python's standard library is built-in; pip doesn't install it, and there aren't 150 system packages added automatically. Understanding transitive dependencies helps you recognize that large install counts are normal for complex projects. Source: Lesson 6",
      source: "Lesson 6: Understanding Dependencies and Packages"
    },
    {
      question: "You ask your AI to 'find Python files larger than 1MB.' It suggests a long pipeline with find, exec, awk, and grep. You don't understand awk syntax. What's the AI-native development approach?",
      options: [
        "Ask AI to explain what each part does and how data flows through the pipeline; verify the output looks right rather than memorizing syntax",
        "Refuse to use the command until you have memorized awk documentation completely from official sources first",
        "Switch to a different AI tool that does not use complex commands like awk at all",
        "Manually check every Python file size using a GUI file manager to avoid using terminal commands entirely"
      ],
      correctOption: 0,
      explanation: "AI-native development means understanding WHAT commands do (conceptually) and WHY they work, not memorizing syntax. Ask AI: 'Break down this pipeline step by step. What does each command do? How does data flow?' AI will explain: find locates files, exec runs ls on each, awk extracts sizes, grep filters to large files. You verify the output looks correct. You don't need to become an awk expert—you need to supervise the pipeline logic. Option B misunderstands AI collaboration—memorizing awk is unnecessary when AI generates it. Your job is understanding the data flow, not syntax mastery. Option C is unnecessary—complex commands are powerful; avoiding them limits your capabilities. Option D defeats the purpose of AI collaboration—using GUI when AI can automate means slower workflow. The key insight: In the AI era, understanding concepts and verifying output matters more than syntax memorization. Source: Lesson 7",
      source: "Lesson 7: Understanding Pipes and Complex Commands"
    },
    {
      question: "Your AI suggests `cat log.txt | grep ERROR | sort | uniq -c | sort -nr` to find the most common errors. Which command in this pipeline actually counts error occurrences?",
      options: [
        "uniq -c counts occurrences of each unique error line appearing in sorted output",
        "grep ERROR counts errors as it filters the log file for matching lines",
        "sort -nr counts and orders the errors from most to least frequent",
        "cat log.txt counts total lines and passes the count to the rest of the pipeline"
      ],
      correctOption: 0,
      explanation: "The pipeline flow: cat outputs all lines → grep filters to ERROR lines → sort alphabetically groups identical errors → uniq -c counts consecutive duplicates → sort -nr sorts counts numerically in reverse (highest first). uniq -c is the counting step—it looks at sorted adjacent lines and counts how many times each unique line appears. The result is lines prefixed with counts like '15 ERROR: Database timeout'. Option B is incorrect—grep filters lines; it doesn't count them. Option C misunderstands sort -nr—it sorts by number in reverse order; it doesn't count anything, just reorders what uniq -c already counted. Option D is wrong—cat outputs file contents; it doesn't count lines. Understanding which command does what in a pipeline helps you modify it (e.g., removing sort -nr if you don't care about ordering). Source: Lesson 7",
      source: "Lesson 7: Understanding Pipes and Complex Commands"
    },
    {
      question: "In the 5-step pattern, you complete 'Ask' and 'Explain' but realize you don't understand AI's explanation. What should you do at Step 3 (Understand)?",
      options: [
        "Say you do not understand and ask clarifying questions, returning to Step 2 until the explanation makes sense",
        "Proceed to Step 4 Verify anyway because asking questions shows you do not trust the AI tool",
        "Skip directly to Step 5 Execute since the AI clearly knows what it is doing",
        "Restart completely from Step 1 Ask with a different request using simpler language for the AI"
      ],
      correctOption: 0,
      explanation: "Step 3 (Understand) is a verification gate—you should only move forward when you actually understand what AI will do. If the explanation is unclear, you say so: 'I don't understand what -r does in cp -r' or 'Why do we need the recursive flag?' AI will clarify. This is iterative learning, not a linear checklist. The pattern is: Ask → Explain → Understand (if no, return to Explain) → Verify → Execute. Option B is incorrect—asking questions shows you're engaged and responsible, not distrustful. Good supervision requires understanding. Option C violates the entire safety pattern—executing without understanding is dangerous. Option D is unnecessary—you don't need to restart; you just need clarification on the current explanation. Real example from Lesson 2: When Gemini AI hit a limitation, it explained and asked for clarification rather than proceeding blindly. You should do the same. Source: Lesson 2",
      source: "Lesson 2: The Safety-First Dialogue Pattern"
    },
    {
      question: "Your AI shows `export DATABASE_URL='postgres://localhost/db'` but when you open a new terminal tab, the variable is gone. What does this reveal about export?",
      options: [
        "Export sets variables only for the current terminal session; they do not persist across new terminal windows or tabs",
        "The export command failed and you need to use sudo export for persistence across all terminals and sessions",
        "This is a bug in your terminal application that should not happen with properly configured export commands",
        "Export variables automatically transfer to new tabs but require waiting 30 seconds for synchronization to complete fully"
      ],
      correctOption: 0,
      explanation: "export creates environment variables for the current shell session only. Each terminal tab/window is a separate shell session with its own environment. When you export in Tab A, it exists only in Tab A. Opening Tab B creates a new session without that variable. This is by design—sessions are isolated. For persistence across sessions and reboots, you need either (1) .env files loaded by your application or (2) shell configuration files like ~/.bashrc or ~/.zshrc that export variables on every shell startup. Option B is incorrect—sudo doesn't make export persistent; it would just run the export command with elevated privileges, which doesn't help. Option C misunderstands how shells work—this is correct behavior, not a bug. Option D is wrong—there's no synchronization; new tabs start fresh without previous exports. Understanding session scope prevents confusion when variables 'disappear.' Source: Lesson 5",
      source: "Lesson 5: Understanding Configuration and Secrets Safely"
    },
    {
      question: "You ask AI to verify a file copy worked correctly. It suggests `ls -la source.txt target.txt`. What information does this provide for verification?",
      options: [
        "It shows both files exist and compares their sizes and timestamps to confirm the copy operation completed successfully",
        "It shows only source.txt details because target.txt is not created yet until you manually verify the copy",
        "It automatically detects file corruption by comparing MD5 hashes of both files for you behind the scenes",
        "It shows permissions only without any size or date information that could verify the copy succeeded"
      ],
      correctOption: 0,
      explanation: "Running ls -la on both files shows their details side-by-side: size, modification time, permissions. For a successful copy, you should see: (1) both files exist (no 'not found' error), (2) identical sizes (same number of bytes), (3) target.txt has a recent timestamp (just copied). Different sizes or missing target.txt indicates copy failure. This is basic verification—the copy created a file of the correct size. Option B is incorrect—if target.txt doesn't exist, ls will error, telling you the copy failed. Option C is wrong—ls doesn't calculate hashes; it shows metadata. You'd need additional tools like md5sum for hash comparison. Option D misunderstands ls -la output—it shows size, date, time, permissions, owner, all useful for verification. Understanding how to verify operations using ls helps you confirm AI's actions succeeded. Source: Lesson 4",
      source: "Lesson 4: Understanding File Operations Safely"
    },
    {
      question: "Your AI is in `/Users/mjs/Documents` and you want it in `/Users/mjs/projects/app`. What's the difference between `cd ../projects/app` and `cd ~/projects/app` for navigation?",
      options: [
        "First uses relative path (up one level then down); second uses absolute path from home; both reach the same destination but different navigation logic",
        "The first command will fail because you cannot combine .. and folder names in a single cd command ever",
        "Both commands are identical and bash interprets them exactly the same way during execution with no difference",
        "Second command is invalid because tilde cannot be combined with folder paths in bash navigation commands at all"
      ],
      correctOption: 0,
      explanation: "Both commands work but use different logic. cd ../projects/app is relative: from /Users/mjs/Documents, go up to /Users/mjs (..), then down to projects, then app. Result: /Users/mjs/projects/app. cd ~/projects/app is absolute: ~ means home (/Users/mjs), then down to projects, then app. Same destination, different path reasoning. Relative depends on current location; absolute doesn't. Option B is incorrect—you can absolutely combine .. with folders like ../../../projects/app/src. Option C is wrong—bash interprets them differently: .. is relative navigation up, ~ is home directory expansion. The result happens to be the same in this case. Option D is false—~ combines with paths just fine: ~/Documents, ~/projects/app, ~/.config all work. Understanding relative vs absolute paths helps you choose the right navigation approach. Source: Lesson 3",
      source: "Lesson 3: Understanding File Navigation Through Dialogue"
    },
    {
      question: "You install a package with `pip install flask` and want to verify it is accessible. What's the MOST reliable verification method?",
      options: [
        "Try importing it in Python interactive shell with import flask and check for errors; no error means successful installation",
        "Check if flask appears in pip list output; presence in the list confirms it works correctly",
        "Run pip show flask and verify the version number matches the latest release on PyPI exactly",
        "Restart your computer to ensure all packages are properly registered with the operating system for use"
      ],
      correctOption: 0,
      explanation: "The most reliable verification is attempting to use the package. Start Python (python3 or python), then import flask. If this succeeds without ImportError, Flask is installed and accessible to your Python environment. This tests the actual use case—can your code import it? Option B is partially useful—pip list shows Flask is installed, but doesn't confirm it's importable (it might be installed for a different Python version). Option C is irrelevant to verification—version matching is useful for compatibility, but doesn't tell you if it works. Even outdated versions can work fine. Option D is unnecessary—packages don't require reboot; they're immediately available to new Python processes. Understanding import-based verification tests the actual functionality, not just installation metadata. Source: Lesson 6",
      source: "Lesson 6: Understanding Dependencies and Packages"
    },
    {
      question: "Your AI suggests `rm -rf temp/` to delete temporary files. Before executing, what specific information should you request in Step 4 (Verify) to prevent mistakes?",
      options: [
        "Request pwd to see current location and ls -la temp/ to see what will be deleted before confirming the operation",
        "Request the --dry-run flag to see what would be deleted without actually deleting anything at all",
        "Request confirmation that rm has an automatic undo feature in case the wrong folder gets deleted by mistake",
        "Request a count of how many milliseconds the deletion will take to complete before starting the operation"
      ],
      correctOption: 0,
      explanation: "Before rm -rf (recursive, forceful deletion), you MUST verify: (1) AI's current location with pwd (are we in the right directory?), (2) What's inside temp/ with ls -la temp/ (is this the right folder? does it contain only temp files?). If AI is in /Users/mjs and temp/ is there, you're deleting /Users/mjs/temp/. If AI mistakenly has a temp/ folder somewhere else, you'd catch this. Option B suggests --dry-run, but rm doesn't have this flag. Some commands do (like rsync), but rm doesn't offer dry runs. Option C is incorrect—rm has NO undo; deleted files are very difficult to recover. This makes verification critical. Option D is irrelevant—execution time doesn't prevent mistakes. The key verification points are location and contents before destructive operations. Source: Lesson 2 and Lesson 4",
      source: "Lesson 2: The Safety-First Dialogue Pattern"
    },
    {
      question: "You see a pipeline `find . -name '*.log' | xargs grep 'ERROR' | wc -l`. You don't know what xargs does. Following AI-native development principles, what should you do?",
      options: [
        "Ask AI to explain what xargs does in this context and how it connects find results to grep for processing",
        "Refuse to use this pipeline until you have read the complete xargs man page documentation and understand every flag available",
        "Replace xargs with a command you already know like cat or ls to avoid learning new tools at this time",
        "Copy the pipeline exactly without understanding because if AI suggests it the command must be completely safe to run"
      ],
      correctOption: 0,
      explanation: "AI-native development means asking AI to explain unfamiliar parts rather than independently researching everything. Ask: 'What does xargs do here? How does it connect to find and grep?' AI will explain: find locates .log files, xargs takes that list and passes each filename as an argument to grep, grep searches each file for ERROR, wc counts matches. You understand the data flow without needing to master xargs. Option B contradicts AI collaboration—reading entire man pages for every command is the old way. AI can explain the specific use case. Option C sacrifices power for familiarity—replacing xargs might break the pipeline or require complex alternatives. Option D is dangerous—never execute without understanding, even if AI suggests it. Verification is your responsibility. The insight: Ask AI to teach you unfamiliar components in context rather than avoiding them or over-researching. Source: Lesson 7",
      source: "Lesson 7: Understanding Pipes and Complex Commands"
    },
    {
      question: "Your `.env` file contains `API_KEY=sk-123`. Your code uses `os.getenv('API_KEY')` but gets None. Your AI suggests checking if `.env` is loaded. What's the likely problem?",
      options: [
        "Your code needs to explicitly load the .env file using a library like python-dotenv because .env files do not auto-load in Python",
        "The .env file is corrupt and needs to be recreated from scratch using export commands instead of files",
        "Python cannot read .env files at all and you must use export in terminal for every environment variable always",
        "The API_KEY value sk-123 is too short and Python requires minimum 20 character environment variable values to work"
      ],
      correctOption: 0,
      explanation: ".env files are just text files—Python doesn't automatically load them. You need a library like python-dotenv to read .env and set environment variables. Install with pip install python-dotenv, then in your code: from dotenv import load_dotenv; load_dotenv(); api_key = os.getenv('API_KEY'). Now it works. Without load_dotenv(), Python's os.getenv() only sees environment variables set via export or system environment, not .env file contents. Option B is incorrect—.env files aren't corrupt; they're just not being loaded. Option C is wrong—Python CAN read .env with the right library; you don't need export. Option D is nonsense—environment variable values have no minimum length; 'a' is valid. Understanding that .env requires explicit loading prevents confusion when configuration seems to 'not work.' Source: Lesson 5",
      source: "Lesson 5: Understanding Configuration and Secrets Safely"
    },
    {
      question: "You run `cp -r project/ backup/` and both folders appear with `ls`. However, files inside might be corrupted. What additional verification step would catch corruption?",
      options: [
        "Compare file counts in both directories and optionally checksum files to verify content integrity beyond just existence and size",
        "The ls command already verifies corruption by checking file headers automatically when displaying the listing",
        "Run cp -r a second time because running copy twice automatically detects and fixes any corruption found",
        "Corruption is impossible with cp command so no additional verification is needed beyond seeing both folders exist"
      ],
      correctOption: 0,
      explanation: "Basic verification with ls shows files exist and compares sizes, but doesn't guarantee identical contents. For deeper verification: (1) Compare file counts: ls -R project/ | wc -l vs ls -R backup/ | wc -l (same number of files?), (2) Use checksums: find project/ -type f -exec md5sum {} \\; vs find backup/ -type f -exec md5sum {} \\; (identical hashes mean identical contents). This catches corruption. Option B is incorrect—ls shows metadata, not content. It doesn't verify file contents or detect corruption. Option C is wrong—running cp again overwrites; it doesn't verify. Option D is false—while cp is reliable, corruption can occur due to disk errors, interruptions, or filesystem issues. Adding checksum verification provides confidence in data integrity. Source: Lesson 4",
      source: "Lesson 4: Understanding File Operations Safely"
    },
    {
      question: "Your AI executes `npm install express` and reports 50 packages installed. Later you run `npm list --depth=0` and see only express. What does --depth=0 reveal?",
      options: [
        "It shows only direct dependencies (packages you explicitly installed) hiding transitive dependencies for a cleaner view of your project",
        "It means npm deleted 49 packages after installation to save disk space showing only the most important one remains",
        "It indicates an npm error where 49 packages failed to install properly and only express succeeded in installation",
        "It proves npm install was unnecessary because you already had express and did not need to install anything new"
      ],
      correctOption: 0,
      explanation: "npm list shows your dependency tree. --depth=0 limits the view to top-level packages (ones you explicitly installed), hiding their dependencies. You installed express (direct dependency). Express installed 49 other packages (transitive dependencies). npm list --depth=0 shows only express. npm list --depth=1 would show express and its direct dependencies. npm list without --depth shows the entire tree. This helps you distinguish between 'what I asked for' and 'what got installed automatically.' The 49 packages still exist on disk; --depth just filters the display. Option B is incorrect—packages aren't deleted; they're hidden from view. Option C is wrong—this is normal behavior, not an error. Option D misunderstands what npm list shows—it displays installed packages, not installation decisions. Understanding --depth helps you navigate complex dependency trees. Source: Lesson 6",
      source: "Lesson 6: Understanding Dependencies and Packages"
    },
    {
      question: "You want to copy file.txt from your current directory to a subdirectory called backup/. Your AI suggests `cp file.txt backup/`. What assumption does this make, and what could go wrong?",
      options: [
        "It assumes backup/ directory already exists; if it does not, cp will fail with 'No such file or directory' error message",
        "The command will automatically create the backup/ directory if it does not exist so nothing can go wrong with this approach",
        "It assumes file.txt is larger than 1MB; smaller files need a different copy command to work properly in bash",
        "The command will copy to the wrong location because backup/ is not a valid relative path syntax for subdirectories"
      ],
      correctOption: 0,
      explanation: "cp file.txt backup/ tries to copy file.txt into the backup directory. If backup/ doesn't exist, cp errors: 'backup/: No such file or directory'. The command assumes the destination directory exists. To handle this safely, check first with ls -la or mkdir backup if needed. Following the safety pattern: AI should run pwd and ls to show current directory contents, point out backup/ doesn't exist, suggest mkdir backup && cp file.txt backup/, then execute after your confirmation. Option B is incorrect—cp does NOT create directories; only mkdir creates directories. Option C is nonsense—file size doesn't affect which copy command works. Option D is wrong—backup/ is perfectly valid relative path syntax for a subdirectory. Understanding command assumptions helps you catch missing prerequisites before errors occur. Source: Lesson 4",
      source: "Lesson 4: Understanding File Operations Safely"
    },
    {
      question: "Your AI shows you a pipeline: `cat users.csv | grep 'active' | cut -d',' -f1 | sort | head -10`. You want the top 20 instead. Which part should you modify?",
      options: [
        "Change head -10 to head -20 because head controls how many lines are displayed at the end of the pipeline",
        "Change grep 'active' to grep 'active' -n 20 because grep controls the output limit for all subsequent commands in pipeline",
        "Change sort to sort -n 20 because sort with a number limits how many sorted results are shown throughout the pipeline",
        "Change cat users.csv to cat users.csv --limit=20 because the input command determines how many results the entire pipeline can process"
      ],
      correctOption: 0,
      explanation: "head -10 displays the first 10 lines of whatever comes before it. The pipeline: cat outputs all users, grep filters to active users, cut extracts first column (usernames), sort alphabetizes them, head shows first 10. To get 20, change head -10 to head -20. Each command in a pipeline does one thing; head controls final output count. Option B is incorrect—grep doesn't have a -n flag for limiting output count; -n shows line numbers. grep passes ALL matches to the next command. Option C is wrong—sort orders lines; it doesn't limit them. sort -n sorts numerically, not to 20 results. Option D is false—cat outputs entire file; it has no --limit flag. Limiting happens at the end with head. Understanding which command controls what helps you modify pipelines precisely. Source: Lesson 7",
      source: "Lesson 7: Understanding Pipes and Complex Commands"
    },
    {
      question: "You ask AI to install Python packages from requirements.txt. It suggests `pip install -r requirements.txt`. What does the -r flag tell pip to do?",
      options: [
        "Read the requirements.txt file and install all packages listed inside it line by line automatically",
        "Run in recursive mode installing packages and all their configuration files throughout the project directory recursively",
        "Reinstall all packages even if they are already installed to ensure the latest versions are used in environment",
        "Remove old packages before installing new ones to prevent conflicts between different versions installed on system"
      ],
      correctOption: 0,
      explanation: "The -r flag means 'read from file.' pip install -r requirements.txt reads the file and installs each package listed. A requirements.txt might contain: requests==2.31.0, flask==2.3.0, pandas>=1.5.0. pip reads this file and installs each package (with version constraints if specified). This makes dependency management reproducible—everyone installs the same packages. Option B is incorrect—there's no recursive mode for pip install; -r is simply 'read from file.' Option C describes pip install --upgrade, not -r. Option D describes pip install --force-reinstall, not -r. Understanding -r helps you work with projects that manage dependencies in files rather than manual commands. Source: Lesson 6",
      source: "Lesson 6: Understanding Dependencies and Packages"
    },
    {
      question: "Your AI is in `/Users/mjs/projects/app` and suggests `rm *.log` to delete log files. Before executing, you run `ls *.log` and see 50 files. What safety concern does this raise?",
      options: [
        "The wildcard * matches all .log files in current directory; verify these are all temporary logs and not important before deleting",
        "The * is dangerous because it will delete ALL files in the entire system not just .log files by matching everything",
        "There is no safety concern because *.log is a safe pattern that only matches files exactly named asterisk.log",
        "The command will fail because bash does not allow deleting more than 10 files at once for safety protection"
      ],
      correctOption: 0,
      explanation: "The * wildcard expands to match all files ending with .log in the current directory. rm *.log will delete all 50 files. Your safety verification: Are these all temporary? Are any important logs you need to keep? Checking with ls *.log first (before rm) shows you exactly what will be deleted—this is the verification step. If you see error.log, debug.log, app.log—all temporary—proceed. If you see important-data.log or analysis-results.log—stop and exclude them. Option B is wrong—*.log matches only .log files in current directory, not everything everywhere. Option C misunderstands wildcards—* is a pattern that expands to matching filenames, not a literal asterisk. Option D is false—bash has no such limit. You can delete thousands of files with wildcards. Understanding wildcard expansion prevents accidental mass deletion. Source: Lesson 4",
      source: "Lesson 4: Understanding File Operations Safely"
    },
    {
      question: "After running `export PATH=$PATH:/new/path`, your AI explains this modifies your PATH variable. Later you open a new terminal and the change is gone. What does this reveal about export?",
      options: [
        "Export modifications are session-specific and temporary; for persistence you must add the export to shell config files like .bashrc or .zshrc",
        "The export command was typed incorrectly and needs to use sudo export PATH for changes to persist across all terminal sessions",
        "This is abnormal behavior indicating your terminal is broken and export should always persist to all new terminal windows automatically",
        "You must run export PATH in every new terminal exactly once per day to maintain the configuration for 24 hours"
      ],
      correctOption: 0,
      explanation: "export sets environment variables for the current shell session. When you close the terminal or open a new tab, that session ends and the variable is lost. To make PATH changes persistent, add the export command to your shell's startup file: ~/.bashrc (Bash) or ~/.zshrc (Zsh). Every new shell session loads this file and runs the export automatically. Option B is incorrect—sudo doesn't make export persistent; it runs with elevated privileges, which doesn't help here. Option C is wrong—session isolation is correct behavior, not a bug. Option D is nonsense—there's no 24-hour timer on export. Understanding session scope vs persistent configuration prevents confusion when terminal settings 'disappear.' Source: Lesson 5",
      source: "Lesson 5: Understanding Configuration and Secrets Safely"
    },
    {
      question: "Your AI suggests `ls -la | grep ^d` to show only directories. What does ^d mean in grep's pattern matching, and why does it work?",
      options: [
        "The caret ^ means start of line, so ^d matches lines starting with d, which are directories in ls -la output format",
        "The ^ is a pipe operator that separates grep into two stages for more accurate directory filtering throughout the search",
        "The ^d means 'exclude files with d in the name' showing everything except directories for inverse filtering throughout the system",
        "The caret is a typo and should be removed because grep d works identically to filter directory names correctly"
      ],
      correctOption: 0,
      explanation: "In grep regex patterns, ^ anchors to the start of a line. ^d means 'lines starting with d.' In ls -la output, directory lines start with 'd' (drwxr-xr-x), file lines start with '-' (-rw-r--r--). So grep ^d filters to only directory lines. This is more precise than grep d, which would also match files with 'd' anywhere in the name. Option B is incorrect—^ is not a pipe operator; | is the pipe operator. ^ is a regex anchor. Option C is backwards—^ doesn't exclude; it anchors. To exclude, you'd use grep -v (invert match). Option D is wrong—grep d matches any line containing 'd,' including files like 'old-data.txt.' grep ^d is specific to lines starting with d. Understanding regex basics helps you read and modify grep patterns AI suggests. Source: Lesson 7",
      source: "Lesson 7: Understanding Pipes and Complex Commands"
    },
    {
      question: "You install a package globally with `npm install -g typescript`. Later, `npm list` doesn't show it. Your AI says it is installed. What explains this?",
      options: [
        "Global packages install to a system location outside your project; npm list shows project packages only, use npm list -g for globals",
        "The npm install -g command failed silently and typescript was not actually installed despite the AI claiming it succeeded",
        "TypeScript is a special package that hides itself from npm list for security reasons and cannot be listed by standard commands",
        "The -g flag is not valid for npm and the command installed typescript locally with the flag being ignored by npm"
      ],
      correctOption: 0,
      explanation: "npm install -g installs packages globally (system-wide, available to all projects). npm install (without -g) installs locally (in current project's node_modules). npm list shows local packages only. To see global packages, use npm list -g or npm list -g --depth=0 (for just top-level globals). TypeScript is installed at a system location like /usr/local/lib/node_modules/typescript, not in your project. Both installation types are valid; they serve different purposes. Option B is incorrect—if installation succeeded, the package is there; npm list just doesn't show globals. Option C is nonsense—packages don't hide from listing. Option D is wrong—-g is perfectly valid; it changes install location to global. Understanding local vs global packages prevents confusion when packages 'disappear' from npm list. Source: Lesson 6",
      source: "Lesson 6: Understanding Dependencies and Packages"
    },
    {
      question: "Your AI suggests `cd /nonexistent/path` and then says this will fail. Following the safety pattern, what should AI do BEFORE suggesting this command?",
      options: [
        "AI should first run pwd and ls to show current location and verify the target path exists before suggesting cd command",
        "AI should execute cd /nonexistent/path immediately to demonstrate the error message so you can learn from real failures safely",
        "AI should suggest cd /nonexistent/path knowing it will fail because errors are the best way to learn bash navigation",
        "AI should refuse to suggest any cd commands at all because navigation commands are too risky for users to execute"
      ],
      correctOption: 0,
      explanation: "The safety pattern requires AI to verify context before suggesting commands. Before cd, AI should: (1) show current location with pwd, (2) verify the target path exists (ls /path or test if directory exists), (3) explain what cd will do, (4) wait for confirmation. If the path doesn't exist, AI should say so and ask: 'This path doesn't exist. Do you want to create it with mkdir first?' This prevents executing commands destined to fail. Option B violates safety—intentionally running failing commands wastes time and doesn't teach verification. Option C misunderstands learning—errors have their place, but preventable errors should be prevented through verification. Option D overcompensates—cd is safe when used with verification. AI shouldn't refuse to help; it should guide safely. The pattern: verify context, explain, confirm, execute. Source: Lesson 2",
      source: "Lesson 2: The Safety-First Dialogue Pattern"
    },
    {
      question: "You see `pip install requests==2.31.0` (with version) vs `pip install requests` (without). What's the critical difference in what gets installed?",
      options: [
        "The first installs exactly version 2.31.0; the second installs the latest available version, which might differ over time",
        "Both commands install identical versions because pip always installs the latest stable version regardless of syntax used in command",
        "The == syntax is invalid and will cause pip to fail; you must use >= for any version specifications in pip install commands",
        "The first installs a development version from source code; the second installs a pre-built binary package for production use"
      ],
      correctOption: 0,
      explanation: "requests==2.31.0 pins to a specific version—you always get 2.31.0 no matter when you run it. requests (without version) installs whatever is latest on PyPI at execution time. In January it might install 2.31.0; in June it might install 2.32.0. Version pinning ensures reproducibility—everyone on your team gets the same version. Unpinned installs are flexible but can cause 'works on my machine' problems if different people get different versions. Most projects use requirements.txt with pinned versions for consistency. Option B is incorrect—pip respects version specifiers when provided. Option C is wrong—== is valid (exact version); >= is also valid (minimum version), as are <=, ~=, etc. Option D misunderstands version syntax—== doesn't affect source vs binary; it specifies version number. Understanding version pinning prevents mysterious dependency mismatches. Source: Lesson 6",
      source: "Lesson 6: Understanding Dependencies and Packages"
    },
    {
      question: "You want to see both pwd and ls output before your AI moves files. Following the safety pattern, what's the correct dialogue sequence?",
      options: [
        "You ask to see location and contents, AI shows pwd and ls, you verify it is correct, then AI explains move command and waits for your confirmation",
        "AI immediately moves files and shows pwd and ls afterward to confirm the move completed successfully with results visible",
        "You memorize the directory structure before asking AI to proceed so you can verify mentally without seeing any command output",
        "AI shows pwd only because ls output is redundant when you already know the current directory path from pwd showing location"
      ],
      correctOption: 0,
      explanation: "The safety pattern for file operations: (1) Ask—you request the operation ('move these files'), (2) AI shows context (pwd for location, ls for contents), (3) You verify context is correct, (4) AI explains what mv command will do, (5) You ask clarifying questions, (6) AI executes after confirmation. Context before action prevents operating in the wrong directory or moving the wrong files. Option B violates safety—executing before verification means mistakes can't be prevented. Option C misunderstands AI collaboration—you don't memorize; AI shows you and you verify visually. Option D is wrong—pwd shows location, ls shows what files are there. Both are necessary—pwd tells you where, ls tells you what. Together they provide complete context for safe operations. Source: Lesson 2 and Lesson 4",
      source: "Lesson 2: The Safety-First Dialogue Pattern"
    },
    {
      question: "Your AI suggests a pipeline: `find . -name '*.py' | head -5 | xargs cat`. Trace the data flow—what will this output?",
      options: [
        "It finds all Python files, limits to first 5 found, then displays the contents of those 5 files concatenated together",
        "It finds 5 Python files then counts total lines in each file showing a summary of file sizes",
        "It displays only the first 5 lines of every Python file found in the directory tree recursively searching all subdirectories",
        "It finds all Python files, limits output to 5 characters per filename, then deletes the files with cat removing them"
      ],
      correctOption: 0,
      explanation: "Trace the pipeline: find . -name '*.py' searches recursively for Python files and outputs their paths (one per line). The pipe sends this list to head -5, which takes only the first 5 paths. The pipe sends these 5 paths to xargs cat, which runs cat on each path, displaying their contents. Result: contents of the first 5 Python files found, concatenated. Option B is incorrect—this pipeline uses cat (display contents), not wc (count lines). Option C misunderstands head placement—head -5 limits to 5 files, not 5 lines per file. For 5 lines per file, you'd need cat file | head -5. Option D is wrong—cat displays file contents; it doesn't delete files. Understanding pipeline data flow helps you predict output and modify commands. Source: Lesson 7",
      source: "Lesson 7: Understanding Pipes and Complex Commands"
    },
    {
      question: "You ask AI to delete log files older than 7 days. It suggests a complex find command with -mtime. You don't understand -mtime. What's the AI-native development approach?",
      options: [
        "Ask AI to explain what -mtime +7 means and how it filters files by age before executing the deletion command safely",
        "Execute the command immediately because if AI suggests it the command must be safe regardless of your understanding level",
        "Refuse to proceed until you have completed a full find command tutorial covering all flags comprehensively from documentation",
        "Manually check every log file's timestamp using a file manager GUI to avoid using confusing terminal flags at all"
      ],
      correctOption: 0,
      explanation: "AI-native development means asking AI to explain unfamiliar parts before execution. Ask: 'What does -mtime +7 do? How does it determine file age?' AI will explain: -mtime +7 matches files modified more than 7 days ago. +7 means 'greater than 7 days old,' -7 would mean 'less than 7 days old,' 7 means exactly 7 days. Now you understand the filter logic and can verify it matches your intent. Option B is dangerous—never execute without understanding, especially for destructive operations like deletion. Option C is unnecessary—you don't need to master all of find; just understand this specific use case. Option D defeats automation—manually checking files wastes time and is error-prone. The key: Use AI as a teacher for unfamiliar syntax, understand the logic, verify it matches your need, then execute. Source: Lesson 7",
      source: "Lesson 7: Understanding Pipes and Complex Commands"
    },
    {
      question: "Your AI suggests `cp important-data.csv important-data-backup.csv` before you modify the file. Later you delete important-data.csv by mistake. Can you recover, and what does this teach?",
      options: [
        "Yes, important-data-backup.csv still exists with original contents; this demonstrates why copy operations create safety before risky changes",
        "No, deleting important-data.csv also deletes all backups automatically because bash prevents orphaned backup files from existing alone",
        "Yes, by running cp --undo which restores the most recent deleted file from bash's automatic backup system built into the command",
        "No, because both files reference the same data and deleting one immediately deletes all copies for consistency across the filesystem"
      ],
      correctOption: 0,
      explanation: "When you cp file.csv backup.csv, you create two independent copies. Deleting file.csv doesn't affect backup.csv—it's a separate file with separate content. You can recover by copying back: cp important-data-backup.csv important-data.csv. This is why AI suggests backups before risky operations—copy creates a safety net. If something goes wrong, you have the original. Option B is incorrect—deleting one file doesn't affect other files; copies are independent. Option C is wrong—there's no cp --undo command. bash doesn't have automatic backup. Option D misunderstands how files work—copies are separate entities. Understanding that cp creates independent copies helps you use backups effectively for safety. Source: Lesson 4",
      source: "Lesson 4: Understanding File Operations Safely"
    },
    {
      question: "Your code reads API_KEY from environment with os.getenv('API_KEY'). You set it with export API_KEY='sk-123' but your Python script prints None. What's the likely issue?",
      options: [
        "You ran export in one terminal tab but ran the Python script in a different tab where the variable is not set",
        "Python cannot access environment variables set with export and requires .env files exclusively for all configuration access",
        "The export command only works with uppercase variable names longer than 10 characters for security reasons by design",
        "os.getenv() is deprecated and you must use os.environ['API_KEY'] instead for export variables to work with Python at all"
      ],
      correctOption: 0,
      explanation: "export sets variables for the current shell session only. If you run export API_KEY='sk-123' in Terminal Tab A, then run python script.py in Terminal Tab B (a different session), Tab B doesn't have API_KEY. Solutions: (1) Run export and python script in the same tab, or (2) Use a .env file that the script loads (not session-dependent), or (3) Add export to ~/.bashrc so it's set in all new sessions. Option B is incorrect—Python absolutely can access export variables via os.getenv() when in the same session. Option C is nonsense—export works with any valid variable name regardless of length. Option D is wrong—os.getenv() is not deprecated and works fine; os.environ[] raises errors if missing while os.getenv() returns None, but both access environment variables. Understanding session scope prevents 'variable not found' mysteries. Source: Lesson 5",
      source: "Lesson 5: Understanding Configuration and Secrets Safely"
    },
    {
      question: "You want to modify a pipeline from `cat file.txt | grep ERROR` to also count errors. Where should wc -l be added, and why?",
      options: [
        "Add to the end as cat file.txt | grep ERROR | wc -l because pipes pass output sequentially and wc -l counts the final filtered lines",
        "Replace grep ERROR with wc -l because you cannot have both filtering and counting in the same pipeline simultaneously at all",
        "Add at the beginning as wc -l | cat file.txt | grep ERROR because counting must happen first before filtering can work correctly",
        "Add before grep as cat file.txt | wc -l | grep ERROR because wc -l needs to process the full file before grep filters"
      ],
      correctOption: 0,
      explanation: "Pipes work left to right, passing output to the next command. Current pipeline: cat outputs all lines → grep filters to ERROR lines → output shows error messages. To count, add wc -l at the end: cat file.txt | grep ERROR | wc -l. Now: cat outputs all lines → grep filters to ERROR lines → wc -l counts those filtered lines → output shows count. Each command transforms the data. Option B is incorrect—you CAN combine filtering and counting; that's the power of pipes. Option C is backwards—counting first (wc -l counts all lines) then filtering would filter the count number, which makes no sense. Option D is wrong—wc -l before grep would count all lines, then grep would filter the count output (meaningless). Understanding sequential pipeline flow helps you add commands in the right position. Source: Lesson 7",
      source: "Lesson 7: Understanding Pipes and Complex Commands"
    }
  ]}
  questionsPerBatch={18}
/>
