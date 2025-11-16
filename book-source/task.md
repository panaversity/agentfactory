I am attaching an image.

![Navbar Image](/book-source/navbar.png)

Please recreate the **exact same navbar UI** shown in the screenshot and integrate it into my **Docusaurus Classic** website.

Replace “Model Context Protocol” with: **AI Native Book**

### 🔥 UI Requirements (Pixel-Perfect)
Reproduce the navbar **exactly** as in the screenshot including:

- Dark theme, full-width layout
- Correct font, weight, tracking, color, spacing, height, and alignment
- Logo/icon on left (use placeholder)
- Text next to logo: **AI Native Book**
- Center search bar exactly like screenshot:
  - Same dimensions, corner radius, shadow, placeholder text “Search…”
  - Small “Ctrl K” tag on right inside input like screenshot
- Right section must include: **Blog**, **GitHub**, and **dark/light theme toggle icon**

### 📌 Additional Requirement (Important Update)
In the screenshot, the items **Documentation**, **Specification**, **Community**, **About MCP** appear like tabs.  
**For my website, these must NOT act as tabs.**  
They should be **simple clickable UI buttons** but **must visually look 100% identical** to what is shown:

- Same hover behavior
- Same underline effect on active state
- Same spacing and typography
- Same layout positioning relative to search bar and title
- Same clickable behavior (no tab logic) — they can be normal links or buttons

### 🎯 Deliverables
Provide:

1. Updated `docusaurus.config.js` navbar configuration
2. A full custom navbar React component (e.g. `/src/components/CustomNavbar/index.js`)
3. A dedicated CSS file (e.g. `/src/css/customNavbar.css`)
4. Instructions to override/replace Docusaurus navbar using theme swizzling (`@theme/Navbar`)
5. Search input must remain functional using Docusaurus local search or Algolia
6. Code must be copy-paste runnable **without modification**

### ⚙️ Constraints
- Do **NOT** use Tailwind or Shadcn
- Do **NOT** use libraries for components
- Only **React + CSS**
- Must be pixel-accurate and spacing-accurate
- No placeholders like “adjust later” or “fine-tune yourself”
- Produce final complete working version

Recreate everything so that it looks **exactly** like the provided screenshot.
