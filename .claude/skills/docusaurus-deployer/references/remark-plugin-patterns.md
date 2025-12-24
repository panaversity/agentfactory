# Remark Plugin Development Patterns

Reference guide for developing custom remark plugins for Docusaurus MDX processing.

## Plugin Architecture

Remark plugins transform Markdown AST (Abstract Syntax Tree) nodes. The `remark-directive` plugin must be loaded first to parse custom directive syntax.

### Configuration in docusaurus.config.ts

```typescript
remarkPlugins: [
  require('remark-directive'),  // Required first - parses :::directive syntax
  require('../../libs/docusaurus/remark-os-tabs'),  // Custom plugins after
  require('../../libs/docusaurus/remark-content-enhancements'),
],
```

**Order matters**: `remark-directive` must be first to create directive AST nodes that custom plugins can transform.

## Directive Syntax and AST Nodes

The `remark-directive` plugin creates different node types based on colon count:

| Syntax | Node Type | Use Case |
|--------|-----------|----------|
| `:name` | `textDirective` | Inline content |
| `::name` | `leafDirective` | Single-line markers |
| `:::name` | `containerDirective` | Content blocks |
| `::::name` | `containerDirective` | Nested content blocks |

### Colon-Count Matching Rule

**Critical**: The parser matches closing markers by colon count.

```markdown
:::outer          <!-- Opens with 3 colons -->
:::inner          <!-- Opens with 3 colons -->
:::               <!-- Closes BOTH - first match wins! -->
:::               <!-- This closes outer (now only 1 open) -->
```

This causes the **nested container problem**.

## The Nested Container Problem

When nesting containers (like `:::tip` inside `:::os-tabs`), the first `:::` closer terminates the outer container prematurely.

### Problem Example

```markdown
:::os-tabs           <!-- Opens outer -->

::windows
Some content

:::warning           <!-- Opens inner -->
This is a warning
:::                  <!-- CLOSES os-tabs! Parser matches by colon count -->

::macos              <!-- Now orphaned - not inside os-tabs anymore -->
More content

:::                  <!-- Closes nothing meaningful -->
```

**Debug symptom**: Plugin only finds first leaf directive (`::windows`), missing subsequent ones (`::macos`, `::linux`).

### Solution: Use 4 Colons for Outer Container

```markdown
::::os-tabs          <!-- Opens outer with 4 colons -->

::windows
Some content

:::warning           <!-- Opens inner with 3 colons -->
This is a warning
:::                  <!-- Closes warning (3 colons match 3) -->

::macos
More content

::::                 <!-- Closes os-tabs (4 colons match 4) -->
```

**Rule**: When nesting containers, outer must use MORE colons than inner.

## Plugin Development Pattern

### Basic Structure

```javascript
const { visit } = require('unist-util-visit');

function remarkCustomPlugin(options = {}) {
  return (tree) => {
    const nodesToTransform = [];

    // Phase 1: Collect nodes (don't mutate during visit)
    visit(tree, (node, index, parent) => {
      if (node.type === 'containerDirective' && node.name === 'my-directive') {
        nodesToTransform.push({ node, index, parent });
      }
    });

    // Phase 2: Transform collected nodes
    nodesToTransform.forEach(({ node, index, parent }) => {
      const newNode = {
        type: 'mdxJsxFlowElement',
        name: 'MyComponent',
        attributes: [/* ... */],
        children: node.children,
      };
      parent.children[index] = newNode;
    });
  };
}

module.exports = remarkCustomPlugin;
```

### Key Patterns

1. **Two-phase transformation**: Collect nodes first, then transform (avoids mutation during traversal)
2. **Check node.type AND node.name**: Directives have specific types and names
3. **Use mdxJsxFlowElement**: For block-level JSX components
4. **Use mdxJsxTextElement**: For inline JSX components
5. **Preserve children**: Pass through child nodes for nested content

### Debug Logging

Add temporary logging to diagnose issues:

```javascript
visit(tree, (node) => {
  if (node.type.includes('Directive')) {
    console.error(`[plugin] Found: ${node.type}:${node.name}`);
  }
});
```

Use `console.error` for stderr (appears immediately). Clear cache after changes:

```bash
rm -rf .docusaurus node_modules/.cache
npm run start
```

## Existing Platform Plugins

Location: `libs/docusaurus/`

| Plugin | Purpose | Directive Syntax |
|--------|---------|------------------|
| `remark-os-tabs` | OS-specific tabbed content | `::::os-tabs` with `::windows`, `::macos`, `::linux` |
| `remark-content-enhancements` | Slides, enhanced blocks | Various |
| `remark-interactive-python` | Interactive Python runners | Custom |

### remark-os-tabs Usage

**Simple case** (no nested containers):
```markdown
:::os-tabs

::windows
Windows instructions here

::macos
macOS instructions here

::linux
Linux instructions here

:::
```

**With nested containers** (tips, warnings, etc.):
```markdown
::::os-tabs

::windows
Windows instructions

:::warning
Important Windows note
:::

::macos
macOS instructions

::::
```

**Generated output**:
```jsx
<Tabs groupId="operating-systems">
  <TabItem value="windows" label="Windows" default>
    Windows instructions...
  </TabItem>
  <TabItem value="macos" label="macOS">
    macOS instructions...
  </TabItem>
  <TabItem value="linux" label="Linux">
    Linux instructions...
  </TabItem>
</Tabs>
```

## Troubleshooting

### "Unused Markdown directives" Warning

**Cause**: Directive nodes exist but no plugin transformed them.

**Debug steps**:
1. Add logging to plugin to verify it's loaded
2. Check if directive names match what plugin expects
3. Verify node types (`containerDirective` vs `leafDirective`)
4. Check for nested container problem (see above)

### Plugin Not Running

1. Verify `remark-directive` is listed first in remarkPlugins
2. Check plugin path is correct in docusaurus.config.ts
3. Clear cache: `rm -rf .docusaurus node_modules/.cache`

### Children Missing from Container

**Symptom**: Container has fewer children than expected.

**Cause**: Likely nested container problem - inner container's closer terminated outer.

**Fix**: Use 4+ colons for outer container.

## Testing Remark Plugins

```bash
# Start dev server with fresh cache
rm -rf .docusaurus && npm run start

# Check browser console for MDX errors
# Check terminal for plugin logs

# Verify AST structure with debug logging
```

## References

- [remark-directive docs](https://github.com/remarkjs/remark-directive)
- [unist-util-visit](https://github.com/syntax-tree/unist-util-visit)
- [mdast (Markdown AST) spec](https://github.com/syntax-tree/mdast)
- [Docusaurus MDX plugins](https://docusaurus.io/docs/markdown-features/plugins)
