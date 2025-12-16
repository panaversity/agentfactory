# Docusaurus OG Image Generator Plugin

Automatically generates beautiful Open Graph (social media preview) images for every page in your Docusaurus site.

## Features

- 🎨 **Automatic Generation**: Creates OG images for all docs during build
- 🎯 **Title-based**: Uses page title and description
- 🖼️ **Beautiful Template**: Professional gradient design with branding
- 📦 **Zero Config**: Works out of the box
- ⚡ **Fast**: Generates during content loading phase

## How It Works

1. **During Build**: The plugin hooks into Docusaurus's `contentLoaded` lifecycle
2. **Reads All Docs**: Extracts title and description from each page
3. **Generates Images**: Creates a 1200x630px PNG image using node-canvas
4. **Saves to Static**: Stores images in `/static/img/og/`
5. **Auto-Injects**: Automatically adds image path to doc metadata (if not already set)

## Generated Image Template

```
┌────────────────────────────────────────────────────────────┐
│  Panaversity • AI Native Development         [gradient bg] │
│                                                             │
│                                                             │
│     Chapter Title Here                                     │
│     Wrapped Across Lines                                   │
│                                                             │
│     Description text here if provided...                   │
│                                                             │
│                                                             │
│  ███████████████████████████████████████████████████████  │
│  ai-native.panaversity.org                                 │
└────────────────────────────────────────────────────────────┘
```

### Design Elements

- **Background**: Dark gradient (navy to deep blue)
- **Brand Label**: "Panaversity • AI Native Development" in cyan
- **Title**: Large white text (56px), auto-wrapped
- **Description**: Smaller gray text (28px), truncated to ~15 words
- **Footer Bar**: Cyan bar with site URL
- **Decorative**: Subtle circular gradients in background
- **Border**: Thin cyan border

## File Naming

Images are named based on the doc ID with slashes replaced by hyphens:

```
docs/intro.md                    → intro.png
docs/chapter-1/lesson-1.md       → chapter-1-lesson-1.png
docs/AI-Tool-Landscape/bash.md   → AI-Tool-Landscape-bash.png
```

## Override for Specific Pages

If a page already has an `image` in its front matter, the plugin won't override it:

```markdown
---
title: "My Custom Page"
image: /img/my-custom-og-image.png
---

This page will use the custom image, not the auto-generated one.
```

## Image Specifications

- **Dimensions**: 1200 x 630 pixels
- **Format**: PNG
- **Color Space**: RGB
- **Quality**: High (lossless PNG)
- **File Size**: Typically 50-150 KB per image

## Dependencies

- `canvas`: Node.js canvas implementation for image generation

## Installation

Already installed! The plugin is located at:

```
plugins/docusaurus-plugin-og-image-generator/
```

And enabled in `docusaurus.config.ts`:

```typescript
plugins: [
  './plugins/docusaurus-plugin-og-image-generator',
  // ... other plugins
],
```

## Customization

To customize the template, edit `plugins/docusaurus-plugin-og-image-generator/index.js`:

### Change Colors

```javascript
// Background gradient
gradient.addColorStop(0, "#YOUR_COLOR_1");
gradient.addColorStop(0.5, "#YOUR_COLOR_2");
gradient.addColorStop(1, "#YOUR_COLOR_3");

// Brand color
ctx.fillStyle = "#YOUR_BRAND_COLOR";
```

### Change Fonts

```javascript
ctx.font = "bold 56px YourFont"; // Title
ctx.font = "28px YourFont"; // Description
```

### Change Layout

Modify the y-coordinates to adjust spacing:

- Brand label: `y = 80`
- Title start: `y = 200`
- Description: `y = y + 80` (relative to title)
- Footer: `height - 80`

## Testing

After building, check:

1. `static/img/og/` directory for generated images
2. View images in browser
3. Test social media previews using:
   - Facebook Debugger: https://developers.facebook.com/tools/debug/
   - Twitter Card Validator: https://cards-dev.twitter.com/validator

## Troubleshooting

### Images Not Generating

1. Check console output during build
2. Verify `canvas` package is installed
3. Ensure `static/img/og/` directory exists
4. Check for error messages in build log

### Canvas Build Issues

If canvas fails to build:

```bash
# Install system dependencies (macOS)
brew install pkg-config cairo pango libpng jpeg giflib librsvg

# Install system dependencies (Ubuntu/Debian)
sudo apt-get install build-essential libcairo2-dev libpango1.0-dev libjpeg-dev libgif-dev librsvg2-dev

# Rebuild
pnpm rebuild canvas
```

### Text Overflow

If titles are too long, they auto-wrap. To adjust:

- Reduce font size in `generateOGImage()` function
- Modify `lineHeight` variable
- Adjust `maxWidth` calculation

## Performance

- **Generation Time**: ~50-100ms per image
- **Total Build Impact**: +2-5 seconds for typical site
- **Cached**: Images are regenerated only when docs change

## Future Enhancements

Potential improvements:

- [ ] Custom templates per section
- [ ] Chapter number indicators
- [ ] Author avatars
- [ ] Reading time estimates
- [ ] Tags/categories display
- [ ] Image optimization/compression
- [ ] Logo/icon support
- [ ] Multiple theme options

## License

MIT
