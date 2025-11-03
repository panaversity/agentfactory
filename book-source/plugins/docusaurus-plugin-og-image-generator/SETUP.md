# OG Images Fix - Issue Resolution

## Problem Summary

Open Graph (OG) images were not appearing when sharing the website on social media platforms (Facebook, Twitter, LinkedIn, Discord, etc.). The images were returning **404 errors** when accessed at URLs like:
- `https://ai-native.panaversity.org/img/og/home.png`
- `https://ai-native.panaversity.org/img/og/preface-agent-native.png`

## Root Cause Analysis

### What Was Working ✅
1. **OG Image Generation**: The plugin was successfully generating 175+ OG images locally
2. **Image Location**: Images were correctly saved to `book-source/static/img/og/`
3. **Build Process**: Images were being copied to `build/img/og/` during local builds
4. **HTML Meta Tags**: The HTML files had correct OG meta tags injected:
   ```html
   <meta property="og:image" content="https://ai-native.panaversity.org/img/og/home.png">
   <meta property="og:image:width" content="1200">
   <meta property="og:image:height" content="630">
   <meta name="twitter:image" content="https://ai-native.panaversity.org/img/og/home.png">
   ```
5. **Plugin Configuration**: The `docusaurus-plugin-og-image-generator` was properly configured and running

### What Was NOT Working ❌
1. **Git Ignored**: The `/static/img/og` directory was explicitly ignored in `.gitignore`:
   ```gitignore
   # Auto-generated OG images (generated during build)
   /static/img/og
   ```
2. **Not Committed**: OG images were never committed to the Git repository
3. **Not Deployed**: Since images weren't in Git, they were never deployed to GitHub Pages
4. **404 Errors**: All OG image URLs returned 404 errors on the live site

## The Fix

### Changes Made

1. **Updated GitHub Actions Workflow** (`.github/workflows/deploy.yml`)
   - **Added**: Font installation step before building
   - **Installs**: DejaVu and Liberation fonts needed by the OG image generator
   - **Ensures**: Plugin can generate images during CI/CD build

2. **Kept `.gitignore`** (`book-source/.gitignore`)
   - **Kept**: `/static/img/og` in the ignore list
   - **Reason**: Images are auto-generated during build, no need to commit them

### Files Changed
- `.github/workflows/deploy.yml` - Added font installation step for OG image generation
- `book-source/.gitignore` - Kept OG image directory in ignore list (images generated during CI)

## Verification Steps

After committing and pushing these changes, verify the fix by:

1. **Check Git Status**:
   ```bash
   git status
   # Should show ~175 new files in book-source/static/img/og/
   ```

2. **Commit Changes**:
   ```bash
   git commit -m "fix: Generate OG images during CI/CD build

   - Added font installation step to GitHub Actions workflow
   - OG images now generated automatically during build
   - Kept /static/img/og in .gitignore (no binary commits)
   - Fixes 404 errors when sharing on social media"
   ```

3. **Push to Main**:
   ```bash
   git push origin main
   ```

4. **Wait for Deployment** (GitHub Actions will deploy automatically)

5. **Test OG Images**:
   ```bash
   # Test if images are accessible
   curl -I https://ai-native.panaversity.org/img/og/home.png
   # Should return: HTTP/2 200
   
   curl -I https://ai-native.panaversity.org/img/og/preface-agent-native.png
   # Should return: HTTP/2 200
   ```

6. **Test Social Media Sharing**:
   - Use [OpenGraph.xyz](https://www.opengraph.xyz/) to test your URLs
   - Share on Twitter/X, Facebook, LinkedIn to see the preview
   - Use [Facebook Sharing Debugger](https://developers.facebook.com/tools/debug/)
   - Use [Twitter Card Validator](https://cards-dev.twitter.com/validator)

## Implementation Approach

We're using the **CI/CD generation approach** (not committing images):

**Benefits**:
- ✅ Images always up-to-date with content
- ✅ No binary files committed to repository
- ✅ Smaller repository size
- ✅ Automatic regeneration on every build

**How it works**:
1. OG images remain in `.gitignore` (not committed)
2. GitHub Actions installs required fonts (DejaVu, Liberation)
3. Docusaurus plugin generates images during build
4. Images are included in the deployed site

**Changes made**:
- Added font installation step to `.github/workflows/deploy.yml`
- Kept `/static/img/og` in `.gitignore`
- Plugin runs during CI build and generates images automatically

## Statistics

- **Total OG Images**: 175 PNG files
- **Image Dimensions**: 1200x630 pixels (optimal for OG)
- **Average File Size**: ~60-80 KB per image
- **Total Size**: ~10-12 MB
- **Coverage**: Home page + all documentation pages

## Plugin Details

The OG image generator plugin (`book-source/plugins/docusaurus-plugin-og-image-generator/`):
- Uses **Satori** for SVG to PNG conversion
- Uses **Sharp** for image processing
- Runs during Docusaurus `postBuild` lifecycle
- Generates images with:
  - Dark gradient background (navy to deep blue)
  - Page title (56px, white)
  - Page description (28px, gray)
  - Panaversity branding
  - Site URL in footer

## Next Steps

1. ✅ Changes staged and ready to commit
2. ⏳ Commit and push changes
3. ⏳ Wait for GitHub Actions deployment
4. ⏳ Verify OG images are accessible
5. ⏳ Test social media sharing

## Notes

- This fix uses the **CI/CD generation approach** (not committing images to Git)
- Images are automatically generated during every build
- Future content changes will automatically have new OG images generated
- No need to manually regenerate or commit images
- Repository stays clean without binary image files
