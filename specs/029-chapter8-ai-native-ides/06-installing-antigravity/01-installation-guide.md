# Installation Guide: Antigravity IDE

## Quick Facts

- **Download site**: https://antigravity.dev (free download as of Nov 2025)
- **Requires**: macOS 11+, Ubuntu 20.04+, Windows 10+
- **Estimated time**: 10-15 minutes per platform
- **Storage**: ~500MB free space
- **Network**: Internet connection for initial setup

---

## Installation by Platform

Choose your operating system below:

---

## macOS Installation

### Step 1: Download
1. Visit https://antigravity.dev
2. Click **"Download for macOS"**
3. Wait for the .dmg file to download (~400MB)

**Expected**: Browser downloads `Antigravity-<version>.dmg`

### Step 2: Install
1. Open **Finder** and locate the downloaded `.dmg` file (likely in **Downloads** folder)
2. Double-click the `.dmg` file
3. A window opens showing **Antigravity** icon and **Applications** folder
4. Drag the **Antigravity** app icon to the **Applications** folder
5. Wait for copy to complete (~30 seconds)

**Expected**: Antigravity appears in **Applications** folder

### Step 3: Launch
1. Open **Applications** folder
2. Find **Antigravity**
3. Double-click to launch
4. If prompted "Are you sure you want to open this?", click **Open** (first-launch security check)
5. Wait for splash screen and main interface (~5-10 seconds)

**Expected**: Antigravity opens with welcome screen showing "Create Workspace" button

### Step 4: Verify
Check that you see:
- ✅ Antigravity window with dark/light theme selector
- ✅ "Create Workspace" button
- ✅ "AI Providers" configuration section

**If you see these**, installation succeeded. Proceed to [02-initial-setup.md](./02-initial-setup.md)

---

## Linux Installation (Ubuntu/Debian)

### Step 1: Download
Option A (Recommended): Direct download
1. Visit https://antigravity.dev
2. Click **"Download for Linux"**
3. Wait for `.tar.gz` file (~450MB)

Option B: Command line
```bash
wget https://antigravity.dev/download/linux -O antigravity-latest.tar.gz
```

**Expected**: File appears in `~/Downloads/` or current directory

### Step 2: Extract
```bash
cd ~/Downloads
tar -xzf antigravity-*.tar.gz
# Creates 'antigravity' directory
```

**Expected**: `antigravity/` folder with executable inside

### Step 3: Install to System Path
```bash
sudo mv antigravity /opt/
cd /opt/antigravity
sudo chmod +x antigravity  # Make executable
sudo ln -s /opt/antigravity/antigravity /usr/local/bin/antigravity
```

**Expected**: Command `antigravity` works from terminal

### Step 4: Create Desktop Entry (Optional but recommended)
This makes Antigravity appear in your applications menu:

```bash
sudo tee /usr/share/applications/antigravity.desktop > /dev/null <<EOF
[Desktop Entry]
Name=Antigravity
Exec=/usr/local/bin/antigravity
Type=Application
Categories=Development;IDE;
Icon=antigravity
EOF
```

### Step 5: Launch
Option A: Command line
```bash
antigravity
```

Option B: Search applications menu for "Antigravity" and click

**Expected**: Antigravity window appears with welcome screen

**If it doesn't launch**, check:
- ✅ `/opt/antigravity/antigravity` file exists and is executable: `ls -la /opt/antigravity/antigravity`
- ✅ Symlink created: `which antigravity` should return `/usr/local/bin/antigravity`

---

## Windows Installation

### Step 1: Download
1. Visit https://antigravity.dev
2. Click **"Download for Windows"**
3. Wait for `.exe` installer (~420MB)

**Expected**: `AntigravitySetup-<version>.exe` appears in Downloads folder

### Step 2: Run Installer
1. Open **Downloads** folder
2. Double-click `AntigravitySetup-*.exe`
3. If prompted "Do you want to allow this app to make changes?", click **Yes**
4. Installer window appears
5. Click **"Next >"** to proceed through setup wizard
6. Accept license agreement: Check **"I agree"**, click **"Next >"**
7. Choose installation location (default `C:\Program Files\Antigravity` is fine)
8. Click **"Install"** and wait (~2 minutes)
9. When prompted "Installation Complete", click **"Finish"**

**Expected**: Antigravity launches automatically after installation

### Step 3: Verify PATH (Optional)
To ensure `antigravity` command works in PowerShell:

```powershell
$env:Path | Select-String "Antigravity"
# Should show: C:\Program Files\Antigravity
```

If not present:
1. Press **Win+X**, select **"System"**
2. Click **"Advanced system settings"**
3. Click **"Environment Variables..."**
4. Under **"System variables"**, find **Path**, click **Edit**
5. Click **"New"** and add: `C:\Program Files\Antigravity`
6. Click **OK**, restart PowerShell

### Step 4: Launch
Double-click **Antigravity** from Start Menu or run:
```powershell
antigravity
```

**Expected**: Antigravity window opens with welcome screen

---

## Verification Checklist

After installation on your platform, verify:

- [ ] **Launch successful**: Antigravity window appears
- [ ] **Welcome screen**: "Create Workspace" button visible
- [ ] **No error messages**: No red error text in console
- [ ] **UI responsive**: Buttons and menus respond to clicks

**All checked?** Great! Proceed to [02-initial-setup.md](./02-initial-setup.md) to configure your first workspace.

---

## Troubleshooting Installation

### Problem: "Antigravity is damaged" (macOS)
**Solution**:
```bash
sudo xattr -rd com.apple.quarantine /Applications/Antigravity.app
```
Then try launching again.

### Problem: Permission denied (Linux)
**Solution**:
```bash
sudo chmod +x /opt/antigravity/antigravity
```

### Problem: Windows Defender blocks installation
**Solution**:
1. Click **"More info"** in warning dialog
2. Click **"Run anyway"**
3. Installer proceeds normally

### Problem: Port already in use (any platform)
Antigravity uses port 8080 for the editor server. If you get "Port 8080 in use":
1. Check what's using the port:
   - macOS/Linux: `lsof -i :8080`
   - Windows: `netstat -ano | findstr :8080`
2. Either:
   - Stop the other application
   - Or uninstall and reinstall Antigravity (will choose different port)

### Problem: "Cannot connect to AI provider" during setup
**Don't worry—this happens next!** This is expected if you haven't configured API keys yet. Proceed to [03-ai-authentication.md](./03-ai-authentication.md).

---

## What's Installed?

After successful installation, your system has:

| Component | Location | Size |
|-----------|----------|------|
| **Antigravity application** | macOS: `/Applications/Antigravity.app`<br>Linux: `/opt/antigravity`<br>Windows: `C:\Program Files\Antigravity` | ~400MB |
| **Configuration files** | `~/.antigravity/config.json` | <1MB |
| **Workspaces** | `~/.antigravity/workspaces/` | Grows with use |
| **Agent artifacts** | `~/.antigravity/workspaces/<name>/artifacts/` | Grows with use |

---

## Next Step

Installation complete! Move on to [02-initial-setup.md](./02-initial-setup.md) to create your first workspace and connect to AI providers.

**Time check**: Installation took ~15 minutes. You have 45-60 minutes remaining for this lesson.

---

## Platform-Specific Notes

### macOS Notes
- Requires macOS 11 (Big Sur) or newer
- Works on both Intel and Apple Silicon (M1/M2/M3)
- First launch may take longer due to code signing verification

### Linux Notes
- Tested on Ubuntu 20.04 LTS and newer
- Also works on Fedora 35+, Arch Linux
- If using different distribution, you may need to install dependencies:
  ```bash
  # Ubuntu/Debian
  sudo apt-get install libssl3 libfuse2

  # Fedora/RHEL
  sudo dnf install openssl fuse
  ```

### Windows Notes
- Requires Windows 10 Build 19041 or later
- .NET Framework 6.0 installed automatically by installer
- Windows 11 recommended for best performance
- WSL2 (Windows Subsystem for Linux) optional but useful for testing
