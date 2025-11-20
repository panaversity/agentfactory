# Troubleshooting Guide

## Important Note: Antigravity is a New Product

Antigravity launched publicly on November 18, 2025—just two days before this lesson was created. While the product is stable, you may encounter issues that more mature tools (Zed, Cursor) don't have. This guide covers known issues and solutions.

**Getting help**:
- Official docs: https://antigravity.dev/docs
- GitHub issues: https://github.com/google/antigravity (if public)
- Community Discord: [Check Antigravity website]
- This guide focuses on setup and installation issues

---

## Installation Issues

### macOS: "Antigravity is damaged and can't be opened"

**Cause**: macOS security policy quarantine

**Solution**:
```bash
sudo xattr -rd com.apple.quarantine /Applications/Antigravity.app
/Applications/Antigravity.app/Contents/MacOS/Antigravity
```

Then try launching again from Applications.

**Alternative**: Open System Preferences → Security & Privacy → Allow Antigravity

---

### macOS: App doesn't appear in Applications after drag-drop

**Cause**: .dmg didn't complete copying or disk corruption

**Solution**:
1. Eject the .dmg: Right-click → Eject
2. Empty Trash
3. Re-download from https://antigravity.dev
4. Try again: Mount .dmg → Drag to Applications → Verify copy completed
5. If stuck: Use terminal:
   ```bash
   cp -r /Volumes/Antigravity/Antigravity.app /Applications/
   ```

---

### Linux: Command not found after installation

**Cause**: Symlink not created or PATH not updated

**Solution**:
```bash
# Verify installation
ls -la /opt/antigravity/antigravity

# Verify symlink
ls -la /usr/local/bin/antigravity

# If missing, create it:
sudo ln -sf /opt/antigravity/antigravity /usr/local/bin/antigravity

# Verify PATH includes /usr/local/bin
echo $PATH | grep -q /usr/local/bin && echo "PATH OK" || echo "PATH missing"

# If PATH missing, add to ~/.bashrc:
echo 'export PATH="/usr/local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# Then test
antigravity
```

---

### Linux: "libssl.so.3: cannot open shared object file"

**Cause**: Missing SSL library dependency

**Solution**:

For Ubuntu/Debian:
```bash
sudo apt-get update
sudo apt-get install libssl3
```

For Fedora/RHEL:
```bash
sudo dnf install openssl
```

For Arch:
```bash
sudo pacman -S openssl
```

---

### Windows: "AntigravitySetup.exe cannot be found"

**Cause**: Browser blocked the download or file deleted before install

**Solution**:
1. Download again from https://antigravity.dev
2. Check Downloads folder for .exe (might have different name)
3. If browser blocked it:
   - Use Edge/Chrome instead of Firefox
   - Disable download protection temporarily
   - Or download via PowerShell:
     ```powershell
     Invoke-WebRequest -Uri "https://antigravity.dev/download/windows" -OutFile "$HOME\Downloads\AntigravitySetup.exe"
     & "$HOME\Downloads\AntigravitySetup.exe"
     ```

---

### Windows: "Windows Defender blocked the installer"

**Cause**: New executable without reputation yet

**Solution**:
1. Click "More info" in warning dialog
2. Click "Run anyway" button
3. Installer proceeds normally

If warning doesn't appear but installation fails:
1. Disable Windows Defender temporarily
2. Run installer
3. Re-enable Windows Defender
4. Check firewall allowed Antigravity: Windows Defender → Firewall → Allow an app → Antigravity

---

## Launch Issues

### Application launches but window doesn't appear

**Cause**: Window rendering issue or multiple instances running

**Solution**:
```bash
# Check if running
ps aux | grep -i antigravity

# Kill any stuck processes
killall Antigravity
# or (Windows)
Get-Process Antigravity | Stop-Process

# Wait 5 seconds, then relaunch
antigravity  # or double-click app
```

---

### Application opens but shows blank screen

**Cause**: Graphics driver issue or corrupted cache

**Solution**:
1. Close Antigravity
2. Clear cache:
   ```bash
   rm -rf ~/.antigravity/cache
   # or (Windows)
   Remove-Item "$env:USERPROFILE\.antigravity\cache" -Recurse
   ```
3. Restart Antigravity
4. If still blank, check GPU drivers are up-to-date

---

### Port 8080 already in use

**Cause**: Another application using the editor server port

**Solution**:

Find what's using port 8080:
```bash
# macOS/Linux
lsof -i :8080

# Windows
netstat -ano | findstr :8080
```

Either:
1. Stop the other application
2. Or uninstall/reinstall Antigravity (will try different port)

---

## Workspace Issues

### Workspace creation fails with permission error

**Cause**: ~/.antigravity directory not writable

**Solution**:
```bash
# Check permissions
ls -la ~/.antigravity

# If owner is wrong, fix it
sudo chown -R $USER ~/.antigravity

# Ensure write permissions
chmod u+w ~/.antigravity

# Try workspace creation again
```

---

### "Workspace location is invalid" error

**Cause**: Path contains special characters or doesn't exist

**Solution**:
1. Use default workspace location (leave blank)
2. Or create the directory first:
   ```bash
   mkdir -p ~/my-workspaces
   ```
3. Then specify: `~/my-workspaces/my-project`
4. Avoid special characters in path: Use alphanumeric, hyphens, underscores only

---

### Can't open existing workspace

**Cause**: Workspace corrupted or path changed

**Solution**:
1. Verify workspace folder exists:
   ```bash
   ls -la ~/.antigravity/workspaces/
   ```
2. Check folder name matches workspace name shown in UI
3. If damaged, try opening directly:
   - Agent Manager → "Open Existing Workspace"
   - Navigate to folder manually
4. If still fails, may need to recreate workspace

---

## AI Provider Issues

### "Connection test failed" (any provider)

**Cause**: Invalid API key, network issue, or provider outage

**Solution**:

1. **Verify API key**:
   - Copy key directly from provider's website again
   - Ensure NO leading/trailing spaces
   - Check key hasn't been revoked/deleted

2. **Verify internet connection**:
   ```bash
   ping google.com  # Or ping openai.com, etc.
   ```

3. **Check provider status**:
   - Google: https://status.cloud.google.com
   - Anthropic: https://status.anthropic.com
   - OpenAI: https://status.openai.com

4. **If key valid and internet good**, wait 5 minutes (provider might be temporarily slow) and try again

---

### "Invalid API key" error (Anthropic/OpenAI/Google)

**Cause**: Wrong key format or revoked key

**Solution**:

**For Anthropic**:
- Key should start with `sk-ant-`
- Generate new key: https://console.anthropic.com/api/keys
- Key is shown once; copy immediately
- If you miss it, delete and create new one

**For OpenAI**:
- Key should start with `sk-proj-`
- Verify at: https://platform.openai.com/api/keys
- If multiple keys, try each one
- Check key hasn't been deleted

**For Google AI**:
- Key should start with `AIza`
- Verify at: https://ai.google.dev/settings/api/keys
- Some keys have quota restrictions; verify in Google Cloud console

---

### "Quota exceeded" (free tier limit)

**Cause**: Hit rate limit for free tier

**Solution**:
- **Google**: Free tier = 50 requests/minute. Wait until next minute.
- **Anthropic**: Free tier limited by credits. Add payment method.
- **OpenAI**: Free tier limited by credits. Add payment method.

Or switch to different provider with remaining quota.

---

### "Model not available"

**Cause**: Model requires payment or is deprecated

**Solution**:
1. Check provider's documentation for available models
2. Some models (GPT-4) require payment history
3. Add payment method and wait 5-10 minutes
4. Or try alternative model:
   - Anthropic: Try Haiku instead of Sonnet
   - OpenAI: Try GPT-4-mini instead of GPT-4
   - Google: Try Gemini instead of Gemini 2.0

---

## Agent Issues

### Agent creation dialog won't open

**Cause**: UI rendering issue or workspace not fully loaded

**Solution**:
1. Wait 5 seconds for workspace to fully load
2. Click away from any other dialogs
3. Try again: Agent Manager → Create Agent button
4. If still fails, close and reopen workspace

---

### Agent appears stuck (progress bar frozen)

**Cause**: Agent processing or API timeout

**Solution**:
1. **Wait 60 seconds**. Agents sometimes take time thinking.
2. Check logs:
   ```bash
   tail -f ~/.antigravity/logs/agent.log
   ```
3. If still stuck after 60 seconds, **click "Stop"** to cancel
4. Try again with simpler task: "Write a simple function"
5. Check API quota (might be rate-limited)

---

### Agent generates code but it doesn't work

**Cause**: Task description too vague or contradictory

**Solution**:
1. **Review Implementation Plan** before it runs
   - If plan looks wrong, reject it
   - Agent re-generates new plan

2. **Make task description clearer**:
   - WRONG: "Create a function"
   - RIGHT: "Create a Python function named celsius_to_fahrenheit that takes a float as input and returns the Fahrenheit equivalent, with error handling for non-numeric inputs"

3. **Iterate on agent work**:
   - After agent completes, create new agent: "Fix the function: add better error messages"
   - Agent can improve previous work

---

### Artifacts not generating

**Cause**: Agent still processing or rendering issue

**Solution**:
1. **Wait longer**. Task Lists take 10-20 seconds, Plans another 10-20 seconds.
2. **Refresh Agent Manager**:
   - Close workspace, reopen it
   - Click "Refresh" if available
3. **Check logs** for errors:
   ```bash
   tail -f ~/.antigravity/logs/agent.log
   ```
4. **Check agent status**—if showing "Error", see error message

---

### Code appears in Editor but doesn't match what was planned

**Cause**: Agent deviated from plan during implementation (sometimes intentionally)

**Solution**:
1. Review the code—agent might have made improvement
2. If wrong, you can:
   - Edit code manually in Editor (take over)
   - Or create new agent to fix it: "The celsius_to_fahrenheit function is incorrect. [describe issue]. Fix it."

---

## Editor Issues

### Files don't appear in Editor when agent generates code

**Cause**: Rendering lag or workspace not updated

**Solution**:
1. Wait 5 seconds
2. Click on file in side panel if visible
3. Switch to File Explorer tab (if available)
4. Close and reopen Editor tab
5. Reload workspace: Close workspace, reopen

---

### Tab autocomplete suggestions are wrong or slow

**Cause**: Normal for early product release

**Solution**:
1. Disable autocomplete in settings if distracting:
   - Settings → Editor → Tab Autocomplete → Off
2. Or accept only good suggestions (ignore others)
3. This improves as you use more (agent learns context)

---

### Can't hand off code to agent from Editor

**Cause**: Agent might not be running or UI not responsive

**Solution**:
1. Verify agent is running (check Agent Manager)
2. Click "Hand off" button
3. If button grayed out, agent completed or stopped
4. Create new agent to continue work

---

## Integrated Browser Issues

### Browser won't open when agent needs to test

**Cause**: Browser component not initialized or port issue

**Solution**:
1. **Check port availability**: Integrated Browser uses ports 8081-8085
   ```bash
   lsof -i :8081-8085  # macOS/Linux
   netstat -ano | findstr ":808"  # Windows
   ```
2. **Close other Chrome instances** (Integrated Browser uses Chrome-based engine)
3. **Restart Antigravity**
4. **Check logs**:
   ```bash
   tail -f ~/.antigravity/logs/browser.log
   ```

---

### Browser launches but agent says "Cannot access URL"

**Cause**: Local testing URL not accessible within browser sandbox

**Solution**:
1. Ensure your app is running locally:
   ```bash
   # In separate terminal
   python main.py  # Or npm start, etc.
   ```
2. Use `localhost:PORT` not `127.0.0.1:PORT`
3. Verify port isn't blocked by firewall
4. Agent will retry automatically

---

## General Troubleshooting Steps

### When something isn't working:

1. **Check logs**:
   ```bash
   # Antigravity main log
   tail -f ~/.antigravity/logs/antigravity.log

   # Agent-specific log
   tail -f ~/.antigravity/logs/agent.log

   # Browser/testing log
   tail -f ~/.antigravity/logs/browser.log
   ```

2. **Verify permissions**:
   ```bash
   # Ensure ~/.antigravity is writable
   ls -la ~/.antigravity
   chmod 755 ~/.antigravity
   ```

3. **Check for updates**:
   - Antigravity → Settings → "Check for updates"
   - New product might have frequent updates

4. **Reset workspace** (last resort):
   ```bash
   # Backup current workspace
   cp -r ~/.antigravity/workspaces/myworkspace ~/.antigravity/workspaces/myworkspace.backup

   # Delete config cache
   rm ~/.antigravity/workspaces/myworkspace/.cache

   # Restart Antigravity
   ```

5. **Report issue**:
   - If none of above work, visit https://antigravity.dev and report bug
   - Include: OS, Antigravity version, error message, steps to reproduce

---

## New Product Instability Notes

Antigravity is very new. You might encounter:
- Occasional UI glitches (buttons not responding)
- Performance hiccups (slow response to commands)
- Breaking changes in minor versions
- Undocumented behaviors

This is normal for early-release software. The good news:
- Team is actively fixing issues
- Updates roll out frequently
- Community reports issues and engineers respond quickly

**If you hit something not in this guide**:
1. Check if update available (Settings → Check Updates)
2. Search Antigravity forums/Discord for solution
3. Report to team with reproduction steps

---

## Getting Help

### Resources

| Resource | Purpose |
|----------|---------|
| **https://antigravity.dev/docs** | Official documentation |
| **https://antigravity.dev/api** | API reference |
| **[GitHub repo]** | Bug reports, feature requests |
| **[Discord/Community]** | Community help, discussion |

### What to Include When Reporting Issues

1. **Steps to reproduce**: Exactly what you did
2. **Expected vs actual**: What should happen vs what happened
3. **Environment**: OS, Antigravity version, Python version (if applicable)
4. **Logs**: Error messages from `~/.antigravity/logs/`
5. **Screenshots**: If visual issue

---

## Known Issues (As of Nov 20, 2025)

Based on early product release, known issues include:

- [ ] UI occasionally freezes on slow systems (fix: restart)
- [ ] Artifacts sometimes take 30+ seconds to generate (normal for new product)
- [ ] Agent sometimes creates unnecessary files (expected behavior)
- [ ] Task Lists format occasionally inconsistent (cosmetic, doesn't affect functionality)
- [ ] Integrated Browser memory usage high with long-running tests (optimize: restart browser)

**Check https://antigravity.dev/known-issues** for updated list.

---

## Escalation Path

If you're stuck:

1. **Check this guide** (first)
2. **Check Antigravity docs** (second)
3. **Search community forums** (third)
4. **Create GitHub issue with full reproduction steps** (if confirmed bug)
5. **Ask in Discord/community chat** (if general question)

---

## Summary

Antigravity is a powerful but young product. Most issues resolve with:
- **Clear thinking** (what exactly went wrong?)
- **Checking logs** (they tell you what happened)
- **Restarting** (fixes many transient issues)
- **Updating** (new version might fix it)
- **Asking community** (someone's probably hit this before)

You're doing great! Common issues are solved in minutes. Rare issues escalate quickly.

---

## Next Steps

If you successfully navigated installation and troubleshooting:

✅ **Lesson 6 complete!** You've installed, configured, verified, and troubleshot Antigravity.

🎯 **Ready for Lesson 7?** Explore advanced agent features and complex workflows.

---

**Questions?** Review this guide, check official docs, or ask in community forums. You've got this!
