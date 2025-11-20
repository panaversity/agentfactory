---
sidebar_position: 7
title: "Chapter 9: Git & GitHub for AI-Driven Development Quiz"
---

# Chapter 9: Git & GitHub for AI-Driven Development Quiz

Test your understanding of Git and GitHub as safety mechanisms for AI-driven development. This assessment focuses on practical scenarios you'll encounter when experimenting with AI-generated code.

<Quiz
  title="Chapter 9: Git & GitHub for AI-Driven Development Assessment"
  questions={[
    {
      question: "You've initialized a new Git repository with `git init`. You create three files: `index.html`, `style.css`, and `script.js`. When you run `git status`, all three files appear as 'untracked'. What does this indicate about Git's tracking behavior?",
      options: [
        "Git automatically tracks all files in the repository",
        "Git requires explicit instruction to start tracking files",
        "Git only tracks files with specific file extensions",
        "Git waits until first commit to decide tracking"
      ],
      correctOption: 1,
      explanation: "Git requires explicit instruction to start tracking files. When you initialize a repository, Git creates the tracking infrastructure but doesn't automatically track any files—you must use `git add` to explicitly tell Git which files to track. Option A is incorrect because Git never automatically tracks files; tracking is always an intentional action. Option C is wrong because Git can track any file type—extensions don't determine tracking eligibility. Option D is incorrect because Git doesn't wait until commit to decide; you explicitly add files to the staging area before committing. This explicit tracking design gives you control over what Git monitors, which is crucial when working with AI-generated code where you may want to exclude certain experimental files.",
      source: "Lesson 1: Your First Git Repository"
    },
    {
      question: "After asking Claude Code to generate a contact form, you've made changes to three files: `contact.html`, `validation.js`, and `styles.css`. You want to commit only the HTML and JavaScript files because the CSS changes need more testing. What is the most appropriate workflow?",
      options: [
        "Use git add contact.html validation.js then commit selected files",
        "Commit all three files and revert CSS changes afterward",
        "Create separate branches for HTML and CSS modifications",
        "Delete styles.css temporarily before committing the other files"
      ],
      correctOption: 0,
      explanation: "The most appropriate workflow is to use `git add contact.html validation.js` to stage only the files you want to commit, then create a commit with just those selected files. This is precisely what the staging area is designed for—selective commits. Option B is inefficient and clutters history with unnecessary revert commits. Option C is overkill for this scenario; branches are for isolating entire features or experiments, not for managing which files to include in a single commit. Option D is dangerous and incorrect—you should never delete files to control what gets committed; the staging area exists specifically to handle this situation. The staging area (also called the index) acts as a preparation zone where you explicitly choose what goes into each commit, giving you fine-grained control over your project history when working with AI-generated code.",
      source: "Lesson 1: Your First Git Repository"
    },
    {
      question: "You've just committed changes with the message 'updated files'. Three weeks later, you need to understand what that commit changed and why. What problem does this scenario illustrate about commit messages?",
      options: [
        "Commit messages should describe implementation details thoroughly",
        "Commit messages should explain why changes were made clearly",
        "Commit messages should list every file that was changed",
        "Commit messages should include timestamps for tracking purposes"
      ],
      correctOption: 1,
      explanation: "This scenario illustrates that commit messages should explain why changes were made clearly, not just what was changed. 'Updated files' provides no context about the purpose, motivation, or impact of the changes—information your future self desperately needs. Option A is incorrect because commit messages shouldn't describe implementation details; that's what the code diff shows. Option C is wrong because Git automatically tracks which files changed—listing them redundantly in the message wastes space. Option D is incorrect because Git automatically records timestamps with every commit; manually including them is redundant. Good commit messages answer 'why this change?' and 'what problem does it solve?'—crucial context when you're reviewing AI-generated code weeks later and trying to understand whether to keep, modify, or discard changes. Think of commit messages as documentation for your future self.",
      source: "Lesson 1: Your First Git Repository"
    },
    {
      question: "You're working on a project and run `git status`. You see 'modified: app.js' in red under 'Changes not staged for commit'. What does this tell you about the current state of `app.js`?",
      options: [
        "Git has tracked this file previously and detected changes",
        "This file is new and hasn't been committed before",
        "This file will be included in next commit automatically",
        "This file has merge conflicts that need resolving"
      ],
      correctOption: 0,
      explanation: "The red 'modified: app.js' under 'Changes not staged for commit' tells you that Git has tracked this file previously and detected changes since the last commit. The 'modified' label specifically indicates the file existed in the previous commit and now has differences. Option B is incorrect because new untracked files appear under 'Untracked files', not 'Changes not staged for commit'. Option C is wrong because files with unstaged changes are NOT automatically included in the next commit—you must explicitly `git add` them first. Option D is incorrect because merge conflicts appear with a different status message ('both modified' or 'unmerged paths'). Understanding status output is crucial when working with AI-generated code—you need to know whether changes are staged (green, ready to commit), unstaged (red, need `git add`), or untracked (new files Git doesn't know about).",
      source: "Lesson 1: Your First Git Repository"
    },
    {
      question: "After initializing a repository, you create five files but only want Git to track three of them. The other two are temporary test files you'll delete soon. What strategy should you use?",
      options: [
        "Add all five files then remove unwanted ones later",
        "Add only the three files you want to track",
        "Initialize separate repositories for temporary and permanent files",
        "Wait until deleting temporary files before adding anything"
      ],
      correctOption: 1,
      explanation: "The best strategy is to add only the three files you want to track using selective `git add` commands. This keeps your repository clean and focused on meaningful content from the start. Option A is inefficient and creates unnecessary work—you'd have to use `git rm --cached` to untrack files, and you might accidentally commit them first. Option C is severe overkill; you don't need separate repositories for temporary vs. permanent files—that's what selective tracking and `.gitignore` are for. Option D is problematic because you shouldn't delay tracking important files just because you have temporary ones; they're independent decisions. Git's tracking is always intentional and selective—you explicitly choose what to track. This is especially valuable when working with AI assistants that might generate multiple test files, debug outputs, or experimental code that you don't want in your permanent history.",
      source: "Lesson 1: Your First Git Repository"
    },
    {
      question: "You've staged three files with `git add` but haven't committed yet. You realize one of the staged files (`debug.js`) shouldn't be included in this commit. What's the most appropriate action?",
      options: [
        "Commit all files then immediately revert the debug file",
        "Use git restore staged to unstage the debug file",
        "Delete the git staging area and start over completely",
        "Create a new branch and move debug file there"
      ],
      correctOption: 1,
      explanation: "The most appropriate action is to use `git restore --staged debug.js` to unstage the debug file while keeping your other staged files ready to commit. This surgical approach removes just the unwanted file from the staging area. Option A is inefficient—why commit something you immediately want to undo? This clutters your history unnecessarily. Option C doesn't make sense; you can't 'delete' the staging area, and even if you could, you'd lose all your carefully staged changes. Option D is overkill; branches are for isolating features or experiments, not for managing what goes into a single commit. The staging area is designed precisely for this scenario—it's a flexible preparation zone where you can add and remove files before committing. This flexibility is invaluable when working with AI-generated code, where you often need to review and selectively commit changes.",
      source: "Lesson 1: Your First Git Repository"
    },
    {
      question: "You ask Gemini Code Assist to generate a login form. It creates four files: `login.html`, `auth.js`, `styles.css`, and `temp-notes.txt`. You want to commit the first three but never track the notes file. What's the best approach?",
      options: [
        "Add first three files individually and ignore notes file",
        "Add all files then remove notes file from staging",
        "Commit everything now and delete notes file in next commit",
        "Create gitignore file before adding any files to repository"
      ],
      correctOption: 0,
      explanation: "The best approach is to add the first three files individually and simply ignore the notes file—don't add it to Git at all. Selective adding is the cleanest approach for one-off exclusions. Option B works but requires an extra step (unstaging); it's more work for the same result. Option C is wrong because you'd commit a file you don't want in history at all—even if you delete it later, it remains in Git history. Option D is good practice for patterns of files (like all `.txt` files or `temp-*` files) but overkill for a single file you simply won't add. The key insight: Git only tracks files you explicitly add. You don't need special mechanisms to exclude individual files—just don't add them. This is particularly useful when AI assistants generate temporary files, debug outputs, or scratch notes that aren't part of your actual project.",
      source: "Lesson 1: Your First Git Repository"
    },
    {
      question: "A colleague looks at your repository and asks why you have so many commits with messages like 'fix', 'update', and 'changes'. They suggest commits should be 'intentional save points' rather than automatic captures. What principle are they emphasizing?",
      options: [
        "Commits should capture complete logical units of work",
        "Commits should happen frequently throughout development process",
        "Commits should include every single file change immediately",
        "Commits should only occur at end of day"
      ],
      correctOption: 0,
      explanation: "Your colleague is emphasizing that commits should capture complete logical units of work—meaningful checkpoints that represent completed tasks or fixes, not arbitrary stopping points. Each commit should tell a story: 'Added user authentication', 'Fixed validation bug', or 'Refactored database queries'. Option B is incorrect because frequency isn't the issue—it's about intentionality. Many small meaningful commits are fine; many meaningless commits aren't. Option C is wrong because commits shouldn't include every change immediately—you should group related changes into logical commits using the staging area. Option D is far too rigid; the right time to commit is when you've completed a logical unit of work, which might happen multiple times per day or once every few days depending on the task. When working with AI-generated code, this means committing after you've tested and verified a feature works, not just after receiving the code.",
      source: "Lesson 1: Your First Git Repository"
    },
    {
      question: "Before modifying a working feature, you run `git diff` and see 50 lines of changes in green and red. After reading the diff, you decide the changes are too risky and want to discard them. What does this scenario illustrate about Git's value in AI workflows?",
      options: [
        "Git diffs enable informed decisions before committing changes",
        "Git diffs automatically prevent bad code from being saved",
        "Git diffs replace the need for testing code changes",
        "Git diffs show only syntax errors in modified code"
      ],
      correctOption: 0,
      explanation: "This scenario illustrates that Git diffs enable informed decisions before committing changes. By reviewing exactly what changed (line by line, with context), you can make a conscious choice about whether to keep or discard the modifications. Option B is incorrect because Git doesn't prevent anything automatically—it provides visibility so you can make decisions. Git is a tool for humans, not an automated gatekeeper. Option C is dangerously wrong; diffs show what changed, not whether it works correctly. You still need to test! Diffs complement testing, not replace it. Option D is incorrect because Git diffs show all changes (any modifications to text), not just syntax errors. They're purely textual comparisons. When working with AI-generated code, diffs are your reality check—they show exactly what the AI changed so you can evaluate whether those changes align with your goals before committing.",
      source: "Lesson 2: Viewing Changes & Safe Undo"
    },
    {
      question: "You're reviewing AI-generated changes to `database.js` using `git diff`. You see that the AI replaced a simple query with a complex one involving multiple joins. You're not sure if this improves performance or breaks functionality. What should you do next?",
      options: [
        "Test the changes thoroughly before committing to repository",
        "Commit immediately since AI generated this code optimally",
        "Discard changes automatically because they look too complex",
        "Ask AI to regenerate code until diff shows minimal changes"
      ],
      correctOption: 0,
      explanation: "You should test the changes thoroughly before committing to the repository. A diff reveals what changed, but only testing reveals whether the changes work correctly and improve functionality. Option B is dangerously naive—AI-generated code is not automatically optimal or correct; it requires human verification. The AI might have misunderstood requirements or introduced subtle bugs. Option C is the opposite extreme; complexity isn't inherently bad. Some problems require complex solutions. Discarding changes without testing prevents you from learning whether the AI's approach is actually better. Option D focuses on the wrong goal; minimizing diff size doesn't equal better code. Sometimes large changes are necessary and beneficial. The key insight: diffs inform you what changed, but testing validates whether those changes are good. This is especially critical with AI-generated code, where you're evaluating suggestions from a tool that doesn't truly understand your application's context.",
      source: "Lesson 2: Viewing Changes & Safe Undo"
    },
    {
      question: "After experimenting with three different AI-generated navigation menus, you have unstaged changes in `nav.html`. You decide none of the experiments worked and want to return to the last committed version. What command accomplishes this?",
      options: [
        "Use git restore to discard unstaged changes completely",
        "Use git reset to delete the entire repository history",
        "Use git commit to save current changes as backup",
        "Use git branch to isolate changes before removing them"
      ],
      correctOption: 0,
      explanation: "Use `git restore nav.html` to discard unstaged changes completely and return the file to its last committed state. This is exactly what restore is designed for—safely undoing working directory changes. Option B is dramatically wrong and dangerous; `git reset` can modify history in specific modes, but it doesn't 'delete repository history', and it's certainly not the tool for discarding changes to a single file. Option C contradicts your goal—you want to discard changes, not save them. Committing would permanently record unsuccessful experiments in your history. Option D is backwards; branches isolate changes before you make them, not after. You create branches to safely experiment; you don't create branches to remove changes you've already made in your working directory. The key principle: Git provides safe undo at every stage. Unstaged changes can be discarded with `restore`. Staged changes can be unstaged. Commits can be reverted. This safety net encourages experimentation with AI suggestions because you can always return to a known-good state.",
      source: "Lesson 2: Viewing Changes & Safe Undo"
    },
    {
      question: "You committed changes, then immediately realized you forgot to include an important file. The commit is not pushed to GitHub yet. What's the most appropriate way to fix this?",
      options: [
        "Stage forgotten file and use git commit amend flag",
        "Create new commit immediately after the incorrect one",
        "Use git reset hard to delete commit and start over",
        "Push to GitHub first then fix the problem remotely"
      ],
      correctOption: 0,
      explanation: "The most appropriate fix is to stage the forgotten file with `git add`, then use `git commit --amend` to add it to the previous commit. Amend is designed exactly for this scenario—fixing the most recent commit before sharing it with others. Option B works but creates unnecessary commit clutter—two commits when one would suffice. Your history becomes harder to read: 'Added feature' followed immediately by 'Added file I forgot'. Option C is far too aggressive; `git reset --hard` would discard all commit changes and force you to redo all your work from scratch. Overkill for forgetting one file. Option D makes the problem worse—once you push, the commit becomes part of shared history, making amendments more complicated. Fix locally first. The key insight: Git lets you refine commits before sharing them. Amend, reorder, combine—all fine locally. Once pushed, commits become part of collaborative history and should be modified much more carefully.",
      source: "Lesson 2: Viewing Changes & Safe Undo"
    },
    {
      question: "Your project has five commits. After testing, you realize the last two commits broke a feature that worked in commit three. You want to return to commit three's state while keeping commits four and five in history for reference. What strategy should you use?",
      options: [
        "Use git revert to create new commits undoing problematic changes",
        "Use git reset hard to delete commits four and five permanently",
        "Create a new branch starting from commit three only",
        "Use git restore to undo all five commits at once"
      ],
      correctOption: 0,
      explanation: "Use `git revert` to create new commits that undo the changes from commits four and five. Revert preserves all history (including the problematic commits) while safely returning your code to commit three's working state. Option B is destructive—`git reset --hard` would delete commits four and five permanently from history. You lose the ability to review what went wrong or potentially salvage parts of those commits later. Option C doesn't solve your problem; creating a branch from commit three gives you a separate line of development but doesn't fix the main branch where the broken commits exist. Option D shows a misunderstanding of `restore`—it works on individual files and unstaged changes, not on commits. The key distinction: `revert` undoes changes by creating new commits (additive, safe, preserves history), while `reset` rewrites history (subtractive, risky, destroys records). When working with AI-generated code, preserving history of what didn't work is valuable—it prevents you from asking the AI to try the same failed approach again.",
      source: "Lesson 2: Viewing Changes & Safe Undo"
    },
    {
      question: "After running `git diff`, you see 200 lines of changes. You realize you've been editing for an hour without committing. A colleague suggests you should have committed every 10 minutes. What's the most balanced perspective on commit frequency?",
      options: [
        "Commit when you complete logical units of testable work",
        "Commit every ten minutes regardless of work completion status",
        "Commit only at end of day to minimize history clutter",
        "Commit after every single line of code you write"
      ],
      correctOption: 0,
      explanation: "The most balanced perspective is to commit when you complete logical units of testable work—milestones where you can say 'this feature/fix/change is done and works'. Time is not the right trigger; completion is. Option B is arbitrary and counterproductive. Committing every 10 minutes might capture half-finished thoughts, broken code, or incomplete changes—commits that don't represent meaningful progress. Option C goes too far in the other direction; committing only at day's end creates overly large commits that bundle unrelated changes together, making history hard to understand. Option D is extreme micromanagement that creates overwhelming noise—hundreds of commits per day, many capturing incomplete syntax. The right frequency depends on your work. Small bug fix? One commit. Large feature? Multiple commits as you complete stages. When experimenting with AI-generated code, commit after testing each AI suggestion—that way each commit represents a verified, working checkpoint you can return to if the next experiment fails.",
      source: "Lesson 2: Viewing Changes & Safe Undo"
    },
    {
      question: "You've made extensive changes to `api.js` based on AI suggestions, but the code won't run. Running `git diff` shows 80 lines changed. You don't remember what you changed manually versus what the AI suggested. What does this scenario reveal about workflow practices?",
      options: [
        "Review diffs immediately after AI generates code always",
        "Always commit AI code before reviewing it for accuracy",
        "Disable git diff when working with AI generated code",
        "Delete the repository and start over from scratch entirely"
      ],
      correctOption: 0,
      explanation: "This scenario reveals you should review diffs immediately after AI generates code, before making additional manual changes. This practice helps you understand exactly what the AI did, verify it makes sense, and maintain clear boundaries between AI contributions and your modifications. Option B is backwards—you should review before committing, not after. Committing without understanding creates a repository full of mysterious, potentially broken code. Option C is counterproductive; diff is even more valuable when working with AI code because it shows you exactly what the AI changed, helping you understand and verify its suggestions. Option D is a dramatic overreaction. You can use `git restore` to discard changes and return to a clean state—no need to delete the entire repository. The key workflow: Request AI code → Review diff → Test changes → Commit if working. This creates clear checkpoints where you understand exactly what changed and verified it works. Without this discipline, your repository becomes a confusing mix of working code, broken AI suggestions, and mystery edits.",
      source: "Lesson 2: Viewing Changes & Safe Undo"
    },
    {
      question: "You're about to experiment with three different AI-generated approaches to user authentication. You want to test each approach without the experiments interfering with each other or your working code. What Git feature is designed for this scenario?",
      options: [
        "Create separate branches for each authentication approach experiment",
        "Create separate commits for each approach in main branch",
        "Create separate repositories for each approach to test independently",
        "Create separate folders for approaches within the same branch"
      ],
      correctOption: 0,
      explanation: "Create separate branches for each authentication approach—this is precisely what branches are designed for. Each branch provides an isolated workspace where you can experiment without affecting the main codebase or other experiments. Option B doesn't provide isolation; commits are sequential on the main branch. You'd be constantly undoing and redoing work to switch between approaches—inefficient and confusing. Option C is overkill and creates management overhead. You'd need to keep three separate repositories in sync, handle three different GitHub connections, and manually compare approaches—all unnecessary when branches solve this elegantly. Option D doesn't provide Git-level isolation; all folders are tracked together in one working state. You can't easily switch between approaches or keep them separate in history. Branches let you create parallel timelines: `main` (working code), `auth-approach-1`, `auth-approach-2`, `auth-approach-3`. Test each independently, then merge the winner into `main` and delete the others. Perfect for AI experimentation.",
      source: "Lesson 3: Testing AI Safely with Branches"
    },
    {
      question: "You create a new branch called `ai-experiment` and make extensive changes. After testing, you realize the approach doesn't work. You want to abandon this branch and return to `main` without merging any of the experimental changes. What should you do?",
      options: [
        "Switch to main branch and delete ai-experiment branch entirely",
        "Merge ai-experiment into main then revert the merge immediately",
        "Keep ai-experiment branch active but never commit to it",
        "Reset main branch to before ai-experiment was created originally"
      ],
      correctOption: 0,
      explanation: "Switch to `main` branch with `git checkout main` (or `git switch main`), then delete the `ai-experiment` branch with `git branch -d ai-experiment` (or `-D` to force deletion). This cleanly abandons the failed experiment. Option B is unnecessarily complicated—why merge something you want to discard? You'd create merge clutter in history, then immediately create revert clutter. Just don't merge. Option C leaves abandoned branches cluttering your repository indefinitely. Unused branches are confusing for collaborators and for your future self—'Is this branch important? Should I keep it?' Clean up failed experiments. Option D doesn't make sense; the main branch hasn't changed just because you created another branch. Branches don't affect each other until you merge—that's the point of isolation. You don't need to 'reset' main because main is untouched. This scenario demonstrates a key Git principle: branches are disposable experiments. Create fearlessly, delete ruthlessly. Failed experiments should disappear completely.",
      source: "Lesson 3: Testing AI Safely with Branches"
    },
    {
      question: "You're on the `main` branch with uncommitted changes. You want to switch to a different branch called `feature-test` to review work there. Git refuses to switch branches, saying 'Your local changes to the following files would be overwritten'. What's causing this error?",
      options: [
        "Uncommitted changes might conflict with feature-test branch files",
        "Git always requires pushing to GitHub before switching branches",
        "Feature-test branch is corrupted and must be deleted immediately",
        "Main branch has too many commits to allow switching"
      ],
      correctOption: 0,
      explanation: "The error occurs because your uncommitted changes might conflict with files in the `feature-test` branch. If `feature-test` has different versions of the files you've modified, switching would overwrite your unsaved work. Git protects you from accidental data loss. Option B is incorrect; GitHub and branch switching are unrelated. You can switch branches freely locally—pushing to GitHub is about sharing commits, not about branch operations. Option C is unfounded; this error message doesn't indicate corruption. Git is simply protecting your uncommitted work from being overwritten. Option D makes no sense; commit count has no effect on branch switching. You can switch between branches regardless of how many commits they have. To fix this, you have three options: 1) Commit your changes on main first, 2) Discard your changes if they're not important, or 3) Stash your changes temporarily. This protection is crucial when working with AI experiments—Git ensures you don't accidentally lose work when switching between experiment branches.",
      source: "Lesson 3: Testing AI Safely with Branches"
    },
    {
      question: "You've successfully tested an AI-generated feature in a branch called `new-feature`. You want to integrate these changes into your main branch. What command achieves this, assuming you're currently on the `main` branch?",
      options: [
        "Use git merge new-feature to integrate branch changes",
        "Use git branch new-feature to integrate changes automatically",
        "Use git checkout new-feature to move changes to main",
        "Use git delete main to replace it with new-feature"
      ],
      correctOption: 0,
      explanation: "Use `git merge new-feature` to integrate the new-feature branch changes into main. Merge combines the commit history from new-feature into your current branch (main), preserving all the development work. Option B misunderstands `git branch`—this command creates, lists, or deletes branches; it doesn't move changes between branches. `git branch new-feature` would try to create a branch with that name (and fail if it already exists). Option C is backwards; `git checkout new-feature` switches your working directory to the new-feature branch—you'd be on the wrong branch. To merge into main, you must be on main. Option D is nonsensical and destructive. You don't delete the target branch; you merge into it. The merge workflow is: 1) Switch to the branch you want to merge into (main), 2) Run `git merge <branch-name>` to bring in changes, 3) Delete the feature branch if you no longer need it. This workflow is fundamental for AI experimentation—test in branches, merge winners into main.",
      source: "Lesson 3: Testing AI Safely with Branches"
    },
    {
      question: "After merging a feature branch into `main`, you realize the merge introduced a critical bug that doesn't exist in either branch independently. What does this scenario illustrate about branch merging?",
      options: [
        "Merges can create new issues not present in either branch",
        "Merges always preserve the working state of both branches",
        "Merges automatically test code before integrating the changes",
        "Merges only fail when branches have identical file contents"
      ],
      correctOption: 0,
      explanation: "This scenario illustrates that merges can create new issues not present in either branch independently. When two branches modify related code in different ways, combining those changes can introduce unexpected interactions—called integration bugs. Option B is wishful thinking; merges combine changes, but they don't guarantee the result works. You might merge two individually working branches and get broken code due to incompatibilities. Option C is incorrect; Git performs no testing during merges. It mechanically combines changes based on file content. Verifying the merged code works is your responsibility. Option D is backwards; merges typically fail (require manual resolution) when branches have conflicting changes to the same lines, not identical content. Identical content merges automatically without issues. This is why the workflow is: merge → test → commit. Even clean merges (no conflicts) can introduce bugs. When integrating AI-generated code from branches, always test the merged result before considering the integration complete. Don't assume 'no merge conflicts' means 'working code'.",
      source: "Lesson 3: Testing AI Safely with Branches"
    },
    {
      question: "You're testing three different AI-generated CSS layouts in three branches: `layout-a`, `layout-b`, and `layout-c`. After testing, you want to merge `layout-b` into `main` and delete the other two experimental branches. What does this workflow demonstrate?",
      options: [
        "Branches enable safe parallel testing with selective integration",
        "Branches should always be merged regardless of test results",
        "Branches require merging in alphabetical order before deleting",
        "Branches automatically merge when one proves most successful"
      ],
      correctOption: 0,
      explanation: "This workflow demonstrates that branches enable safe parallel testing with selective integration. You can experiment with multiple approaches simultaneously, then choose the best one to merge and discard the rest—exactly what branches are designed for. Option B is counterproductive; merging failed experiments clutters history with broken code. Selective merging (only merge what works) keeps your main branch clean and functional. Option C is nonsensical; merge order has nothing to do with names. You can merge branches in any order based on what makes sense for your project. Git doesn't impose alphabetical requirements. Option D misunderstands Git; branches never merge automatically. Merging is always an intentional action you trigger with a command. Git can't determine which approach is 'most successful'—that's a human judgment. This is the ideal AI experimentation workflow: create multiple experimental branches → test all approaches → merge the winner → delete the losers. You get to try multiple AI suggestions without risk, knowing you can cleanly abandon approaches that don't work.",
      source: "Lesson 3: Testing AI Safely with Branches"
    },
    {
      question: "You create a branch, make five commits, then merge it into `main`. Later, you realize you want to review the individual changes from those five commits. Will you still be able to see them in Git history?",
      options: [
        "Yes, all five commits remain visible in main history",
        "No, merging consolidates all commits into one merged commit",
        "Only if you used git merge squash command originally",
        "Only if commits happened within the same calendar day"
      ],
      correctOption: 0,
      explanation: "Yes, all five commits remain visible in main's history after a standard merge. Regular `git merge` preserves the complete commit history from the feature branch, integrating all individual commits into the target branch. You can review each commit separately using `git log`. Option B describes squash merging, not regular merging. Standard merges preserve all commits; you'd need to explicitly use `--squash` to consolidate commits. Git defaults to preserving history. Option C is backwards; `--squash` is the option that consolidates commits into one. Without squash, all commits are preserved—that's the default behavior. Option D is absurd; Git doesn't care about calendar days. Commits from any time period are preserved in history. Age is irrelevant. This history preservation is valuable when working with AI experiments. You might merge a feature branch, then later want to review exactly what the AI suggested in each commit—did it solve the problem incrementally, or did one specific commit contain the key insight? Preserved history lets you analyze AI behavior patterns.",
      source: "Lesson 3: Testing AI Safely with Branches"
    },
    {
      question: "You're experimenting with AI-generated code in a branch called `experimental`. A colleague suggests you should never create branches until you're certain the code works. What's wrong with this advice for AI-driven development?",
      options: [
        "Branches are specifically for testing uncertain code safely",
        "Branches should only contain working code that's been verified",
        "Branches require approval from team leads before creation",
        "Branches automatically deploy code to production environments"
      ],
      correctOption: 0,
      explanation: "Branches are specifically for testing uncertain code safely—the entire point of branches is to provide a safe space for experimentation without risking your working codebase. Your colleague's advice is backwards. Option B contradicts the purpose of feature branches. If you wait until code works before creating a branch, you've already made risky changes directly on main—the exact problem branches prevent. Branches let you experiment safely. Option C is organizational overhead that varies by team but isn't a Git requirement. You can create branches freely in Git; whether your team has processes around branch creation is a separate concern. Option D is a dangerous misunderstanding. Branches don't deploy anything automatically; they're local or remote development spaces. Deployment requires explicit actions (like pushing to specific branches that trigger CD pipelines). The correct workflow for AI experimentation: 1) Create branch, 2) Request AI code, 3) Test in branch, 4) Merge if working, delete if not. Branches give you freedom to try AI suggestions without fear—if the AI's approach fails, you simply delete the branch and start over.",
      source: "Lesson 3: Testing AI Safely with Branches"
    },
    {
      question: "You've been working on an AI experiment in a local branch for two weeks. Your laptop crashes and you lose all data. When you get a new laptop, will your experimental branch still exist on GitHub?",
      options: [
        "Only if you pushed the branch to GitHub before crash",
        "Yes, Git automatically syncs all branches to GitHub hourly",
        "Yes, branches are stored in cloud by default always",
        "No, local branches never exist on GitHub under any circumstance"
      ],
      correctOption: 0,
      explanation: "The branch will only exist on GitHub if you pushed it before the crash using `git push -u origin <branch-name>`. Local branches exist only on your computer until you explicitly push them to a remote. Option B is incorrect; Git has no automatic sync mechanism. Everything is manual—you must explicitly push commits and branches to GitHub. Git doesn't operate on timers or automatic schedules. Option C reflects a common misunderstanding of Git's architecture. Git is distributed—your local repository is completely independent from GitHub until you push or pull. Nothing is 'cloud by default'. Option D goes too far; local branches can definitely exist on GitHub, but only after you push them. Before pushing, they're local only. This highlights an important backup strategy: if you're working on an experimental branch for more than a day or two, push it to GitHub even if you're not ready to create a pull request. This gives you cloud backup without requiring code review. Use `git push -u origin experiment-branch` to create a remote copy of your work.",
      source: "Lesson 4: Cloud Backup & Portfolio"
    },
    {
      question: "You're preparing your GitHub profile for job applications. You have three repositories: a polished portfolio website, an incomplete learning project with messy commits, and a test repository with experimental code. What's the best approach for presenting your work professionally?",
      options: [
        "Keep polished work public and set experiments to private",
        "Delete all repositories except the most perfect one only",
        "Make everything private since employers shouldn't review code",
        "Keep all repositories public to demonstrate maximum activity"
      ],
      correctOption: 0,
      explanation: "The best approach is to keep polished work public (showcasing your abilities) and set experiments or incomplete projects to private (avoiding misrepresentation). This curates your profile while maintaining honesty. Option B is far too extreme and counterproductive. Employers want to see a range of work showing growth and exploration—one perfect repository looks suspicious and doesn't demonstrate breadth of skills. Option C defeats the entire purpose of GitHub as a portfolio. Public repositories show employers your code quality, problem-solving approach, and technical skills. Private-only profiles offer no visibility into your actual abilities. Option D ignores quality for quantity. A profile with 50 messy, unfinished, or experimental repositories sends the wrong message—it suggests you start many things without finishing them. Strategic visibility matters. Use public repositories to showcase work you're proud of, private repositories for learning experiments or sensitive projects, and deleting truly embarrassing early work is fine—GitHub isn't a permanent record of every mistake.",
      source: "Lesson 4: Cloud Backup & Portfolio"
    },
    {
      question: "After pushing your project to GitHub, you continue working locally and create three new commits. A team member looks at the GitHub repository and says they don't see your recent work. What's the most likely explanation?",
      options: [
        "You didn't push your new commits to GitHub repository",
        "GitHub automatically hides commits newer than one hour old",
        "Your team member needs admin access to view new commits",
        "Local and remote repositories sync automatically every few minutes"
      ],
      correctOption: 0,
      explanation: "The most likely explanation is you didn't push your new commits to GitHub. Local commits exist only on your computer until you explicitly push them with `git push`. Your team member sees only what's been pushed to the remote repository. Option B is nonsense; GitHub displays all pushed commits immediately, regardless of age. There's no delay or hiding of recent commits—pushed commits appear instantly. Option C is incorrect for viewing commits; repository visibility is determined by public/private status, not admin access. If your team member can view the repository at all, they can see all pushed commits. Option D reflects a common misconception about Git's architecture. Local and remote repositories do not sync automatically—ever. All synchronization is manual via push and pull. This is fundamental to Git's distributed model. The Git workflow is: work locally (commits exist only on your machine) → push when ready (upload commits to GitHub) → collaborators pull (download your commits to their machines). Each step is intentional and manual.",
      source: "Lesson 4: Cloud Backup & Portfolio"
    },
    {
      question: "You want to work on your project from both your home laptop and office desktop. You've pushed your project to GitHub from your laptop. What's the correct workflow for accessing this project on your desktop?",
      options: [
        "Use git clone to create desktop copy from GitHub",
        "Manually copy all files from laptop to desktop computer",
        "Create new repository on desktop and manually sync files",
        "GitHub automatically installs projects on all your devices"
      ],
      correctOption: 0,
      explanation: "Use `git clone <repository-url>` to create a complete copy of the GitHub repository on your desktop, including all commit history and branches. This establishes a connection between your desktop copy and GitHub for future syncing. Option B loses all Git history and tracking—you'd have files but no repository, commits, branches, or connection to GitHub. You'd essentially start over, losing all version control benefits. Option C creates unnecessary work and complexity. Why create a new repository and manually sync when Git is designed exactly for this scenario? Clone gives you a complete, connected copy instantly. Option D is fantastical; GitHub doesn't automatically install anything anywhere. All operations are manual and intentional. Git respects your control over where repositories exist. The correct workflow: push from laptop → clone to desktop → work on desktop → push changes → pull on laptop. This synchronized workflow is essential for developers who work from multiple locations or collaborate with others. GitHub serves as the centralized meeting point.",
      source: "Lesson 4: Cloud Backup & Portfolio"
    },
    {
      question: "Your GitHub repository has 15 commits on the remote. You've been working locally and have 18 commits. When you try to push, Git says you must pull first. What does this indicate?",
      options: [
        "Remote has commits your local repository doesn't have",
        "You exceeded GitHub's maximum allowed commits per push",
        "Your local commits are corrupted and need repair",
        "GitHub requires refreshing local copies every 24 hours"
      ],
      correctOption: 0,
      explanation: "This indicates the remote has commits your local repository doesn't have—someone else pushed changes (or you pushed from another computer) while you were working locally. Git requires you to integrate remote changes before pushing yours. Option B is nonsense; GitHub has no limit on commits per push. You can push one commit or one thousand—the number doesn't trigger restrictions. Option C is unfounded; this error message doesn't indicate corruption. It's a normal situation in collaborative development where multiple people (or you from multiple computers) contribute to the same repository. Option D invents arbitrary rules; Git has no time-based requirements. You can work locally for weeks without pulling—Git doesn't care about time intervals. The workflow to resolve this: 1) `git pull` to download and merge remote changes into your local branch, 2) Resolve any conflicts if they exist, 3) `git push` to upload your integrated commits. This is why the command is 'pull before push'—integrate others' work before contributing your own.",
      source: "Lesson 4: Cloud Backup & Portfolio"
    },
    {
      question: "A technical recruiter looks at your GitHub profile and sees six pinned repositories showcasing different skills: web development, data analysis, API integration, testing automation, documentation, and mobile development. What advantage does this deliberate curation provide?",
      options: [
        "Demonstrates diverse technical skills across multiple domains",
        "Proves you never make mistakes or write imperfect code",
        "Shows maximum commit count to impress with activity levels",
        "Guarantees automatic interviews from all viewing companies"
      ],
      correctOption: 0,
      explanation: "Deliberate curation demonstrates diverse technical skills across multiple domains. By pinning repositories that showcase different capabilities, you create a curated portfolio that immediately communicates your breadth as a developer. Option B misunderstands the purpose of portfolios. Polished repositories don't prove perfection—they demonstrate your ability to complete projects and present work professionally. Everyone makes mistakes; portfolios show how you finish projects despite challenges. Option C focuses on the wrong metric. Commit count is a vanity metric that doesn't predict code quality or project value. A repository with 1,000 commits might be messy iterations; one with 20 might be elegantly architected. Quality over quantity. Option D overpromises; GitHub profiles don't guarantee interviews. They increase visibility and demonstrate skills, making you a more compelling candidate, but hiring involves many factors beyond repositories. Strategic pinning is like arranging items on a resume—you highlight your strongest, most relevant work to create the best first impression. Unpinned repositories still exist but don't dominate your profile.",
      source: "Lesson 4: Cloud Backup & Portfolio"
    },
    {
      question: "You're backing up a project to GitHub that contains API keys in a `.env` file. Before pushing, what should you do?",
      options: [
        "Add the .env file to .gitignore to prevent tracking",
        "Push the .env file since GitHub repositories are always private",
        "Encrypt the .env file before committing to repository safely",
        "Rename .env to .env.txt to hide it from GitHub scanning"
      ],
      correctOption: 0,
      explanation: "Add the `.env` file to `.gitignore` to prevent Git from tracking it entirely. This ensures API keys and other secrets never enter version control and never get pushed to GitHub. Option B is dangerous on multiple levels: 1) GitHub repositories can be public or private—you might accidentally make it public later, 2) Even private repositories can be accessed by collaborators, 3) Once secrets are in Git history, they're very difficult to fully remove. Never commit secrets. Option C adds complexity without solving the root problem. Even encrypted, the file is in version control. If your encryption key is compromised or you make a mistake, secrets are exposed. Better to never track the file at all. Option D is security through obscurity—ineffective and naive. Renaming doesn't hide anything; the file contents are still visible in the repository. Malicious actors scan for common secret patterns, not specific filenames. The correct workflow: 1) Add `.env` to `.gitignore`, 2) Commit `.gitignore`, 3) Create a `.env.example` file with dummy values to document required variables without exposing secrets.",
      source: "Lesson 4: Cloud Backup & Portfolio"
    },
    {
      question: "Your project works perfectly locally but breaks when a collaborator clones it from GitHub. You realize you forgot to push your most recent commits that fixed critical bugs. What does this situation illustrate about Git workflows?",
      options: [
        "Pushing regularly ensures remote repository matches local state",
        "Local commits automatically sync to GitHub within several hours",
        "Collaborators should always test code before reporting problems",
        "GitHub creates backups even without pushing from local repository"
      ],
      correctOption: 0,
      explanation: "This situation illustrates that pushing regularly ensures the remote repository matches your local state. If you fix bugs locally but don't push, collaborators clone old, broken code—they're working with an outdated version. Option B reflects a fundamental misunderstanding of Git's architecture. Git NEVER syncs automatically. The delay can be hours, days, or forever—until you manually push. Automatic sync is not how Git works. Option C deflects responsibility. Yes, collaborators should test, but the root problem is you gave them broken code by not pushing your fixes. Testing doesn't solve the underlying issue of outdated remote code. Option D is incorrect; GitHub only knows about commits you've pushed. It doesn't backup your local commits—those exist only on your computer until you push them. Local commits are not visible to GitHub. Develop a push habit: after completing and testing features, push to GitHub. This keeps the remote repository up-to-date and protects against data loss (laptop crashes, etc.). For solo projects, push daily. For collaborative projects, push after each meaningful unit of work.",
      source: "Lesson 4: Cloud Backup & Portfolio"
    },
    {
      question: "You notice your GitHub profile shows 'contributed to X repositories' and a grid showing your daily commit activity. An interviewer asks about a two-month gap where you made no commits. What does this scenario illustrate about GitHub as a portfolio?",
      options: [
        "GitHub activity reflects public work but not full development picture",
        "Gaps in commit history disqualify candidates from all technical roles",
        "GitHub contribution graphs replace need for traditional resume documents",
        "Daily commits are required for GitHub to count you as active"
      ],
      correctOption: 0,
      explanation: "This scenario illustrates that GitHub activity reflects public work but not your full development picture. The gap might represent private work, non-code activities (learning, design, planning), or contributions to systems not tracked by GitHub. Option B is far too absolute and incorrect. Employment gaps happen for legitimate reasons: vacations, learning new skills, working on private/proprietary projects, personal circumstances. Gaps don't disqualify candidates—context matters. Option C overvalues contribution graphs. GitHub portfolios complement resumes by demonstrating practical skills, but they don't replace comprehensive career documents that include education, professional experience, soft skills, etc. Option D is false; GitHub has no activity requirements. You can make commits daily, weekly, monthly, or irregularly—GitHub doesn't penalize any pattern. The contribution graph simply visualizes whatever activity exists. When discussing gaps, be honest: 'I was working on private client projects', 'I took time to learn a new framework', 'I was contributing to an internal company repository'. GitHub shows one dimension of your work—valuable, but not comprehensive.",
      source: "Lesson 4: Cloud Backup & Portfolio"
    },
    {
      question: "After implementing an AI-generated feature in a branch, you create a pull request to merge it into `main`. Your teammate reviews the PR and asks, 'Which parts did the AI generate versus what you wrote?' You didn't document this. What does this scenario illustrate?",
      options: [
        "PR descriptions should document AI contributions for transparency",
        "Pull requests should never mention AI tools were used",
        "Reviewers shouldn't ask about AI usage during code reviews",
        "AI-generated code is indistinguishable from human written code"
      ],
      correctOption: 0,
      explanation: "This scenario illustrates that PR descriptions should document AI contributions for transparency. Reviewers need to know which parts are AI-generated to properly assess the code—AI-generated sections may need more scrutiny for correctness and understanding. Option B is counterproductive and obscures important context. Transparency about tool usage helps teams calibrate review efforts. If reviewers know a section is AI-generated, they might test more thoroughly or ask clarifying questions about your understanding. Option C dismisses legitimate reviewer needs. Understanding code provenance (who/what wrote it) helps reviewers evaluate whether the author understands the changes—a critical aspect of code review. Option D is dangerously naive. AI-generated code often has patterns: sometimes overly verbose, sometimes missing edge cases, sometimes using outdated approaches. Experienced reviewers can often spot AI code, and hiding its use erodes trust. Better practice: in PR descriptions, note 'AI-generated initial implementation of authentication logic' or 'Used Claude to refactor database queries—I reviewed and tested all changes'. This transparency builds trust and helps reviewers focus their efforts effectively.",
      source: "Lesson 5: Code Review with Pull Requests"
    },
    {
      question: "You create a pull request with 50 file changes and a description that says 'Updated code'. Reviewers complain they can't understand what the PR does or why. What principle does this violate?",
      options: [
        "PR descriptions should explain what changed and why clearly",
        "PR descriptions are optional when file changes are self-explanatory",
        "PR descriptions should only list file names that were modified",
        "PR descriptions should be copied from commit messages verbatim"
      ],
      correctOption: 0,
      explanation: "This violates the principle that PR descriptions should explain what changed and why clearly. With 50 files changed, reviewers need context: What problem does this solve? What approach did you take? Are there areas needing extra attention? Option B is wrong because large changes are never self-explanatory. Even if individual changes make sense in isolation, reviewers need to understand the overall goal and how pieces fit together. Documentation is essential, not optional. Option C provides no useful information. Reviewers can see which files changed in the diff—listing them again in the description is redundant. They need higher-level context: purpose, approach, risks, testing. Option D might work for single-commit PRs, but usually PR descriptions need more synthesis. Commit messages track granular changes; PR descriptions explain the overall feature or fix. A good PR description includes: 1) What problem this solves, 2) Approach taken, 3) Key changes or files to review, 4) Testing performed, 5) Any AI assistance used. This context transforms review from 'What is this?' to 'Is this the right solution?'.",
      source: "Lesson 5: Code Review with Pull Requests"
    },
    {
      question: "During a pull request review, a teammate comments on 15 different lines suggesting improvements. You fix all 15 issues in your local branch. What should you do next?",
      options: [
        "Commit fixes locally and push to update pull request automatically",
        "Close pull request and create a new one with fixes",
        "Ask teammate to make changes directly in your pull request",
        "Wait for teammate to approve before making any requested changes"
      ],
      correctOption: 0,
      explanation: "Commit your fixes locally and push them to update the pull request automatically. Pull requests are dynamic—when you push new commits to the branch being reviewed, those commits appear in the PR instantly. This is the standard workflow. Option B is wasteful and creates unnecessary administrative overhead. Closing and reopening PRs loses conversation context, review history, and forces reviewers to start over. PRs are designed to handle iteration. Option C violates development boundaries. Reviewers provide feedback; authors make changes. Having reviewers directly modify your code bypasses the learning opportunity and prevents you from understanding and confirming the changes. Option D is backwards. 'Approval' comes after you address feedback, not before. The workflow is: receive feedback → make changes → push updates → reviewer verifies changes → approval. Waiting for approval before making changes creates a deadlock. The PR review cycle is iterative: submit → feedback → revisions → more feedback (if needed) → approval → merge. Each push to the branch updates the PR, creating a conversation thread where reviewers can see how you addressed their concerns.",
      source: "Lesson 5: Code Review with Pull Requests"
    },
    {
      question: "You're reviewing a teammate's pull request that adds AI-generated form validation. The code works perfectly when tested, but you notice the logic is much more complex than necessary. What's the most valuable feedback you can provide?",
      options: [
        "Suggest simpler approach and explain why complexity is problematic",
        "Approve immediately since code works correctly when tested thoroughly",
        "Request complete rewrite using your preferred coding style instead",
        "Reject pull request until teammate learns advanced patterns first"
      ],
      correctOption: 0,
      explanation: "The most valuable feedback is to suggest a simpler approach and explain why complexity is problematic—this helps your teammate learn while improving code quality. Explain maintenance burden, readability concerns, or simpler alternatives. Option B prioritizes 'works' over 'works well'. Code that functions but is unnecessarily complex creates long-term maintenance problems—harder to understand, modify, debug, and extend. Working code isn't automatically good code. Option C is overly prescriptive and dismissive of valid working code. Style preferences (which are subjective) shouldn't trigger complete rewrites. Focus on meaningful improvements (simplification, correctness, performance) rather than stylistic opinions. Option D is unreasonably harsh and blocks forward progress. If the code works and is just more complex than ideal, that's a learning opportunity, not grounds for rejection. Suggest improvements, explain reasoning, and approve with recommendations. Good code review balances improvement with encouragement. Point out issues ('This complexity makes the code harder to maintain') while suggesting solutions ('Here's a simpler approach that achieves the same goal'). The goal is better code AND developer growth.",
      source: "Lesson 5: Code Review with Pull Requests"
    },
    {
      question: "Your team uses a policy where every pull request requires at least one approval before merging. You submit a PR, and a teammate approves it, but GitHub still won't let you merge. What's the most likely explanation?",
      options: [
        "Automated checks or tests are failing on your branch",
        "GitHub requires 48 hours between approval and merging",
        "Your teammate lacks sufficient permissions to approve anything",
        "Pull requests can only merge on weekdays not weekends"
      ],
      correctOption: 0,
      explanation: "The most likely explanation is that automated checks or tests are failing on your branch. Many repositories configure GitHub to require passing CI/CD checks before merging—approval from humans isn't sufficient if automated tests fail. Option B is nonsense; GitHub has no time delays between approval and merging. Once approval requirements are met and checks pass, you can merge immediately—there's no waiting period. Option C doesn't explain this scenario. If your teammate lacks permission to approve, they couldn't have approved in the first place—GitHub wouldn't show their approval. The approval exists, so permission isn't the issue. Option D is absurd; GitHub operates 24/7/365. Merge availability has nothing to do with calendar days or time of day—it's entirely based on configured requirements (approvals, checks, etc.). Check the PR status section for red X marks indicating failing checks. Common failures: failing unit tests, linting errors, build failures, security scans. Fix the underlying issues, push updates, wait for checks to re-run and pass, then merge. Branch protection rules can require: passing checks, minimum approvals, up-to-date branches, etc.",
      source: "Lesson 5: Code Review with Pull Requests"
    },
    {
      question: "You create a pull request from an AI experiment branch. The PR description includes: 'Claude Code generated the initial implementation. I tested it thoroughly, made several modifications to handle edge cases, and added comments for clarity.' What does this description demonstrate?",
      options: [
        "Appropriate transparency about AI usage and human contributions",
        "Unnecessary disclosure that undermines confidence in your skills",
        "Incorrect approach since AI usage should never be documented",
        "Excessive detail that reviewers don't need to know about"
      ],
      correctOption: 0,
      explanation: "This description demonstrates appropriate transparency about AI usage and human contributions. It clearly delineates what the AI did (initial implementation) versus what you did (testing, modifications, documentation), helping reviewers understand the code's provenance and calibrate their review. Option B reflects misplaced concern about AI disclosure. Transparency about using tools doesn't undermine your skills—it demonstrates professional maturity. You used a tool effectively, verified its output, and made improvements. That's skilled engineering. Option C promotes opacity over transparency. Hiding AI usage creates problems: reviewers waste time figuring out why code has certain patterns, teams miss opportunities to learn from effective AI use, and if revealed later, lack of disclosure damages trust. Option D dismisses relevant context. Reviewers benefit from knowing which parts need more scrutiny (AI-generated logic) versus which parts represent your thoughtful additions (edge case handling). This context makes reviews more efficient and effective. Modern software development involves many tools—frameworks, libraries, AI assistants. Professional practice means acknowledging tools used while demonstrating you understand and validated the output. Transparency builds trust and improves collaboration.",
      source: "Lesson 5: Code Review with Pull Requests"
    },
    {
      question: "After merging several AI-generated features using pull requests, you notice your merged PRs serve as documentation of different AI approaches you tried, what worked, and what didn't. What value does this PR history provide?",
      options: [
        "Creates searchable record of decisions and approaches over time",
        "Proves you always write perfect code on first attempt",
        "Guarantees protection from all future bugs in codebase",
        "Replaces need for separate project documentation entirely"
      ],
      correctOption: 0,
      explanation: "PR history creates a searchable record of decisions and approaches over time. You can search old PRs to remember why you chose approach A over B, what problems emerged with certain patterns, or how you solved similar issues previously—valuable institutional knowledge. Option B misrepresents what PR history shows. Merged PRs often include multiple rounds of feedback and revision—they document the iterative process toward good solutions, not perfection on first try. Iteration is normal and healthy. Option C is absurdly overoptimistic. PR history documents past decisions; it doesn't prevent future bugs. Bugs emerge from new code, changing requirements, integration issues, etc. Documentation helps you avoid repeating past mistakes, but it's not a bug prevention guarantee. Option D overstates the value. PR history complements separate documentation but doesn't replace it. PRs document specific changes and decisions; separate docs provide overarching architecture, setup instructions, API references, etc. Both serve different purposes. Well-documented PRs become a knowledge base: 'We tried using AI to generate all validation logic in PR #47, but it missed several edge cases. PR #53 shows our revised approach with custom validators.' This history guides future decisions and helps onboard new team members.",
      source: "Lesson 5: Code Review with Pull Requests"
    },
    {
      question: "Your team's pull request template includes sections for: Description, Testing Performed, AI Assistance Used, and Breaking Changes. A new teammate asks why you need a template when people can just write freeform descriptions. What value does the template provide?",
      options: [
        "Ensures consistent information across all pull requests",
        "Guarantees pull requests are approved faster by reviewers",
        "Prevents developers from submitting any pull requests",
        "Eliminates need for reviewers to read code changes"
      ],
      correctOption: 0,
      explanation: "Templates ensure consistent information across all pull requests. When every PR includes the same sections, reviewers know where to find specific information (testing details, AI usage, breaking changes), making reviews more efficient and thorough. Option B confuses consistency with approval speed. Templates make reviews more efficient by providing structured information, but approval speed depends on code quality, complexity, and reviewer availability—templates help but don't guarantee faster approval. Option C is nonsensical; templates don't prevent PR submission. They provide structure and guidance, making it easier to create comprehensive PR descriptions, not harder to submit PRs. Option D dangerously misrepresents templates' purpose. Templates provide context that helps reviewers understand code changes, but reviewers still must read and evaluate the actual code—templates supplement code review, not replace it. Template benefits: 1) Reminds submitters to include important information, 2) Creates consistent structure for reviewers, 3) Captures institutional knowledge about what matters in reviews, 4) Reduces back-and-forth asking for missing information. Good templates balance structure with flexibility—required sections for critical info, optional sections for additional context.",
      source: "Lesson 5: Code Review with Pull Requests"
    },
    {
      question: "You've used Claude Code to generate authentication logic three times across different projects. Each time, you ask Claude to generate slightly different variations, spending hours re-explaining the requirements. What practice would make this more efficient?",
      options: [
        "Document the working pattern as reusable instructions for future use",
        "Ask Claude to remember all previous authentication implementations automatically",
        "Copy and paste authentication code without understanding it",
        "Avoid using AI for authentication since it requires repeated explanations"
      ],
      correctOption: 0,
      explanation: "The most efficient practice is to document the working pattern as reusable instructions. Create a prompt template capturing your authentication requirements, preferences, and context so future requests require minimal explanation—turn successful interactions into repeatable patterns. Option B misunderstands AI limitations. Claude doesn't remember previous conversations or implementations unless you provide that context explicitly. AI sessions are independent; you must build your own knowledge management system. Option C is dangerous. Blindly copying code without understanding creates security vulnerabilities, technical debt, and prevents you from adapting solutions to specific project needs. Understanding is always required. Option D throws away a useful tool because of inefficient usage. The problem isn't AI assistance; it's lack of documentation. Solve the documentation problem rather than abandoning a helpful tool. The solution: after successful AI interactions, document: 1) The prompt that worked well, 2) Specific requirements or constraints, 3) Modifications you typically make to AI output, 4) Edge cases to verify. This turns one-time problem-solving into reusable intellectual property. You build a personal library of effective AI prompts and patterns.",
      source: "Lesson 6: Reusable Git Patterns"
    },
    {
      question: "Your team follows this Git workflow for AI experiments: 1) Create branch, 2) Request AI code, 3) Test thoroughly, 4) Create PR with 'AI-generated' label, 5) Merge after review. A new team member asks why you have a documented workflow instead of just 'using Git naturally'. What value does the documented workflow provide?",
      options: [
        "Provides shared understanding of team practices and expectations",
        "Prevents team members from using any Git commands",
        "Guarantees all code produced is perfect and bug-free",
        "Eliminates need for communication between team members entirely"
      ],
      correctOption: 0,
      explanation: "The documented workflow provides shared understanding of team practices and expectations. Everyone knows the process for AI experiments—no confusion about when to branch, how to label PRs, or review expectations. This consistency improves collaboration and reduces miscommunication. Option B is absurd; workflows don't prevent Git usage—they guide it. Team members still use all necessary Git commands, just with shared conventions about when and how. Option C is impossibly optimistic. Workflows improve process consistency; they don't guarantee code quality. Testing, review, and good engineering practices produce quality—workflows structure how teams apply those practices. Option D is nonsensical. Documentation reduces certain communications (like repeatedly explaining 'How do we handle AI code?'), but teams still need rich communication about requirements, design decisions, blockers, etc. Documentation supplements communication, not replaces it. Benefits of documented workflows: 1) Onboarding new members is faster, 2) Decisions are consistent across the team, 3) Processes improve through documented iteration, 4) Implicit knowledge becomes explicit and shareable. Without documentation, every team member might handle AI experiments differently, creating confusion and inconsistency.",
      source: "Lesson 6: Reusable Git Patterns"
    },
    {
      question: "After six months working with AI tools, you notice you repeatedly use three Git patterns: 'AI exploration branch' (test AI suggestions), 'Incremental AI integration' (merge AI code in small pieces), and 'AI-transparent PR' (document AI usage clearly). What do these patterns represent?",
      options: [
        "Reusable strategies you've refined through experience with AI",
        "The only correct ways to use Git with AI tools",
        "Temporary workarounds until better AI tools are developed",
        "Unnecessary complexity that slows down development workflow"
      ],
      correctOption: 0,
      explanation: "These patterns represent reusable strategies you've refined through experience—workflows that proved effective through trial and error, now codified for consistent application. They're your learned best practices for AI-assisted development. Option B is far too absolute. These are your effective patterns, not universal mandates. Different teams, projects, or individuals might develop different effective patterns. Multiple valid approaches exist. Option C misunderstands pattern value. These aren't workarounds for AI limitations—they're thoughtful approaches to AI integration that will remain relevant as tools improve. How you test, integrate, and document AI code matters regardless of AI sophistication. Option D dismisses the value of refined process. While patterns add structure, they increase efficiency by eliminating repeated decision-making ('What should I do here?'). Known patterns make work faster and more consistent, not slower. Good patterns emerge from: 1) Experience (trying approaches and seeing results), 2) Reflection (analyzing what worked and why), 3) Codification (documenting patterns for reuse), 4) Iteration (refining patterns based on new experience). Patterns are professional expertise made concrete and shareable.",
      source: "Lesson 6: Reusable Git Patterns"
    },
    {
      question: "You create a document titled 'Git Workflow for AI Experimentation' with sections on branching strategy, commit message conventions, and PR documentation standards. A colleague says this is over-engineering and developers should just figure things out themselves. What value does explicit workflow documentation provide for AI-driven development?",
      options: [
        "Reduces cognitive load by answering common workflow questions upfront",
        "Prevents developers from making any personal judgment calls",
        "Guarantees identical code output from every team member",
        "Replaces the need for any Git training or learning"
      ],
      correctOption: 0,
      explanation: "Explicit workflow documentation reduces cognitive load by answering common questions upfront—'How should I branch for AI experiments?', 'How do I document AI usage in PRs?'—freeing mental energy for actual problem-solving rather than process decisions. Option B misrepresents documentation's purpose. Good documentation provides guidance and defaults, not rigid constraints. Developers still make judgments about when to deviate from patterns based on specific situations—documentation informs judgment, not replaces it. Option C is impossibly rigid and ignores developer individuality. Different developers will approach problems differently even with shared workflows. Documentation standardizes process, not thinking or code output. Option D is incorrect; documentation complements training, not replaces it. You still need to understand Git fundamentals, but documentation provides team-specific conventions and patterns beyond basic Git knowledge. Without workflow documentation, every developer invents their own approaches (inconsistent), repeatedly makes the same decisions (inefficient), and hits the same obstacles others already solved (wasteful). Documentation captures and shares collective learning, making the entire team more effective. For AI development, this is especially valuable because the field is new and best practices are still emerging.",
      source: "Lesson 6: Reusable Git Patterns"
    },
    {
      question: "You maintain a personal document with AI prompts that worked well for common tasks: 'Generate REST API endpoint', 'Create form validation', 'Write unit tests'. You include context about what makes each prompt effective. What practice does this represent?",
      options: [
        "Building personal intellectual property from AI experimentation experience",
        "Storing prompts to avoid thinking about problems yourself",
        "Creating prompts that work for everyone in all situations",
        "Replacing programming knowledge with memorized AI prompt templates"
      ],
      correctOption: 0,
      explanation: "This practice represents building personal intellectual property from AI experimentation. You're capturing what works—effective prompts, useful context, successful approaches—and turning one-time discoveries into reusable assets. This is knowledge management applied to AI tools. Option B mischaracterizes knowledge capture as intellectual laziness. Documenting effective approaches doesn't mean avoiding thought—it means not re-solving solved problems. You still think deeply when applying prompts to new situations, customizing them, and evaluating AI output. Option C overgeneralizes personal documentation. These prompts work for you in your contexts with your preferences. They're valuable starting points for others but not universal templates. Sharing them is helpful; claiming universal applicability is overreaching. Option D creates a false dichotomy. Effective AI prompt libraries complement programming knowledge; they don't replace it. You need programming knowledge to write good prompts, evaluate AI output, and debug when things fail. Knowledge and tools work together. Building prompt libraries is like any professional knowledge management: chefs keep recipe collections with notes on techniques, writers maintain style guides with effective phrases, developers document design patterns with implementation notes. AI prompts are another tool in your professional toolkit worthy of systematic organization and refinement.",
      source: "Lesson 6: Reusable Git Patterns"
    },
    {
      question: "After documenting your Git workflows for AI development, you share the document with your team. Three months later, you revisit it and realize several patterns are outdated—you've learned better approaches. What does this illustrate about workflow documentation?",
      options: [
        "Documentation should evolve as you learn more effective practices",
        "Documentation is pointless since practices change over time",
        "Documentation should never change once initially created",
        "Documentation becomes incorrect faster than it provides value"
      ],
      correctOption: 0,
      explanation: "This illustrates that documentation should evolve as you learn more effective practices. Documentation is a living artifact that captures current best understanding—it should be updated as practices improve, not abandoned because they change. Option B uses change as an excuse to avoid documentation—counterproductive. Yes, practices evolve, but that means documentation needs updating, not that it's pointless. Outdated documentation is still better than no documentation; you can update it as you learn. Option C treats documentation as immutable canon—unrealistic and harmful. Practices improve through experience; documentation should reflect those improvements. Static documentation becomes increasingly disconnected from reality and loses value over time. Option D is too pessimistic about documentation's value timeline. Even if documentation requires quarterly updates, it provides value continuously between updates. The effort to update is far less than the collective effort of everyone figuring things out independently. Healthy documentation practices include: 1) Date documentation so readers know currency, 2) Assign owners responsible for updates, 3) Solicit feedback from users, 4) Schedule periodic reviews, 5) Celebrate updates as learning milestones. Documentation that evolves reflects a learning organization.",
      source: "Lesson 6: Reusable Git Patterns"
    },
    {
      question: "Your workflow documentation includes a decision tree: 'Is this an experiment? → Create branch. Is the experiment successful? → Create PR. Did tests pass? → Merge.' A teammate says this is too simplistic and real development is more complex. What value does simplified pattern documentation provide?",
      options: [
        "Provides clear default paths while allowing for justified exceptions",
        "Replaces developer judgment with automated decision making processes",
        "Guarantees perfect decisions in every single development scenario",
        "Prevents developers from handling any complex or unusual situations"
      ],
      correctOption: 0,
      explanation: "Simplified patterns provide clear default paths while allowing for justified exceptions. Most situations fit standard patterns—having a clear default eliminates decision paralysis. For unusual situations, developers can thoughtfully deviate from the pattern. Option B misunderstands documentation's role. Patterns guide human judgment; they don't automate it. Developers still evaluate each situation—patterns just provide a strong starting point so judgment focuses on whether deviation is warranted, not on inventing process from scratch. Option C is impossibly optimistic. No pattern covers every scenario perfectly. Patterns optimize for common cases while allowing flexibility for edge cases. Perfect decisions require judgment that patterns inform but don't replace. Option D suggests patterns create rigidity, but good patterns explicitly acknowledge exceptions. Documentation might say: 'This decision tree covers 90% of scenarios. For complex situations involving X, Y, or Z, consult with the team.' The value of simple patterns: 1) Reduce cognitive load for common cases, 2) Establish shared team defaults, 3) Make deviations visible and discussable, 4) Speed up onboarding. Complex reality doesn't mean documentation should be complex—it means documentation should cover common cases clearly while acknowledging complexity exists.",
      source: "Lesson 6: Reusable Git Patterns"
    },
    {
      question: "You've documented this pattern: 'Before requesting AI code, commit your working code. This creates a clean restore point if the AI's suggestion doesn't work.' A new developer asks why this matters since they can just use git restore anytime. What additional value does the pre-AI commit provide?",
      options: [
        "Creates clear boundary showing exactly what AI changed",
        "Prevents git restore command from working on modified files",
        "Makes AI-generated code automatically better quality and more reliable",
        "Eliminates all risk of bugs in AI generated code"
      ],
      correctOption: 0,
      explanation: "The pre-AI commit creates a clear boundary showing exactly what the AI changed. You can diff between the commit before the AI request and after, seeing precisely the AI's contributions—valuable for review, understanding, and debugging. Option B is incorrect; the pre-AI commit doesn't affect how `git restore` works. You can restore files to any commit, including the pre-AI one, regardless of when you made commits. The commit doesn't change Git's restore capabilities. Option C misunderstands the pattern's purpose—it's about clarity and safety, not about changing AI output quality. The AI generates the same code whether or not you committed beforehand. The commit helps you manage and evaluate that code. Option D is impossibly optimistic. No Git practice eliminates bugs. The commit helps you identify and recover from bugs by clearly showing what changed, but it doesn't prevent bugs from existing in AI-generated code. The pattern workflow: working code (commit) → request AI changes → review diff showing exactly what AI did → test → keep (commit) or discard (restore). This creates clear checkpoints and attribution—you always know what the AI contributed versus what you wrote.",
      source: "Lesson 6: Reusable Git Patterns"
    },
    {
      question: "After accumulating 20 documented Git patterns for AI development, you realize you only regularly use five of them. The other 15 are rarely relevant. Should you remove the infrequent patterns from your documentation?",
      options: [
        "Keep all patterns but organize by frequency of use",
        "Delete all patterns used less than weekly from documentation",
        "Remove all documentation since most patterns are not used regularly",
        "Memorize all twenty patterns to avoid documentation entirely"
      ],
      correctOption: 0,
      explanation: "Keep all patterns but organize by frequency—put common patterns first for easy access, and keep rare patterns in a separate section for when unusual situations arise. All patterns have value; organization determines usability. Option B uses the wrong deletion criteria. Infrequent patterns can be extremely valuable when specific situations arise. A pattern you use twice a year might be critical those two times. Frequency doesn't equal importance—edge cases still need documentation. Option C drastically overreacts to uneven pattern usage. The fact that five patterns see heavy use proves documentation's value for those patterns. Having 15 additional patterns for less common scenarios is good preparation, not waste. Option D misunderstands documentation's purpose. Documentation exists precisely so you don't need to memorize everything. You can look up edge case patterns when needed rather than keeping all 20 in working memory. Effective organization strategies: 1) 'Common Patterns' section with your top five, 2) 'Specialized Patterns' section with situation-specific approaches, 3) Table of contents for quick navigation, 4) Tags or categories (e.g., 'branching', 'merging', 'documentation'). Good documentation serves both daily use and rare situations—optimize for both.",
      source: "Lesson 6: Reusable Git Patterns"
    },
    {
      question: "You're teaching a junior developer your Git workflows for AI development. They ask, 'Should I follow these patterns exactly, or can I modify them?' What's the most helpful response?",
      options: [
        "Understand the principles behind patterns then adapt to situations",
        "Follow patterns exactly without any modifications or deviations whatsoever",
        "Ignore patterns completely and invent your own approaches always",
        "Use patterns only when senior developers are watching you work"
      ],
      correctOption: 0,
      explanation: "The most helpful response is: understand the principles behind patterns, then adapt to your situations. Patterns capture what generally works, but understanding why lets you modify them intelligently when circumstances differ from standard cases. Option B creates inflexible adherence that breaks down when facing situations the patterns don't perfectly fit. Rigid rule-following without understanding prevents learning and adaptation—you become a pattern-executing robot rather than a thinking developer. Option C throws away valuable accumulated knowledge. Yes, independent thinking matters, but ignoring documented patterns means repeating mistakes others already solved. Learning from others' experience is efficient and smart. Option D makes patterns performative rather than useful—you'd follow them for appearances rather than because they improve your work. This defeats the entire purpose of having patterns, which is to make development more effective, not to perform compliance. Good use of patterns: 1) Learn the pattern and why it exists, 2) Apply it to standard situations, 3) Recognize when your situation differs from standard, 4) Adapt the pattern based on your understanding of principles, 5) Document your adaptation if it proves effective. Patterns are starting points for judgment, not replacements for it.",
      source: "Lesson 6: Reusable Git Patterns"
    },
    {
      question: "You've documented these commit message patterns for AI work: 'ai-gen: Initial implementation from Claude', 'ai-fix: Corrected AI output', 'ai-test: Verified AI suggestion works'. A teammate suggests just using standard messages like 'Add feature' and 'Fix bug'. What value do the AI-specific commit message patterns provide?",
      options: [
        "Enable searching commit history for AI contributions specifically",
        "Make commits longer and more impressive looking to reviewers",
        "Guarantee that AI-generated code contains no bugs at all",
        "Prevent anyone from discovering you used AI assistance tools"
      ],
      correctOption: 0,
      explanation: "AI-specific commit message patterns enable searching commit history for AI contributions specifically. You can quickly find all AI-generated code, analyze patterns in AI suggestions, or review AI-assisted commits when debugging—valuable metadata for understanding your codebase. Option B focuses on superficial appearance rather than practical value. Commit message length or impressiveness doesn't matter; clarity and searchability do. The AI prefix adds information value, not cosmetic value. Option C attributes impossible powers to commit messages. How you describe commits has no effect on code quality. Messages document what exists; they don't change whether bugs exist in the code itself. Option D contradicts the pattern's actual effect—these prefixes make AI usage more visible, not less. If you wanted to hide AI usage, you wouldn't use AI-specific prefixes. The pattern explicitly increases transparency. Benefits of AI-specific prefixes: 1) Historical analysis: 'How often do AI suggestions need fixes?', 2) Debugging: 'Was this problem introduced by AI or my code?', 3) Learning: 'Which AI prompts led to commits that never needed fixes?', 4) Team visibility: Others quickly see AI's role in project evolution. Consistent commit message conventions turn Git history into searchable, analyzable data.",
      source: "Lesson 6: Reusable Git Patterns"
    }
  ]}
  questionsPerBatch={18}
/>
