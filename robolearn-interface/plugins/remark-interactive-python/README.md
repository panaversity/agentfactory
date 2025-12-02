# Remark Interactive Python Plugin

Automatically transforms Python code blocks in Markdown files into interactive `<InteractivePython />` components during the Docusaurus build process.

## What It Does

Converts static Python code blocks:

````markdown
```python
print("Hello, World!")
x = 5 + 3
print(x)
```
````

Into interactive components:

```jsx
<InteractivePython initialCode={`print("Hello, World!")
x = 5 + 3
print(x)`} />
```

## Features

- ✅ **Automatic Transformation**: No manual component wrapping needed
- ✅ **Path-Based Filtering**: Only transform files in specific directories (e.g., Python Fundamentals)
- ✅ **Opt-Out Support**: Use `nointeractive` or `static` meta to skip transformation
- ✅ **Build-Time**: Zero runtime overhead, all processing done during build
- ✅ **Preserves Code**: Original code is preserved exactly as written

## Configuration

In `docusaurus.config.ts`:

```typescript
{
  docs: {
    remarkPlugins: [
      [require('./plugins/remark-interactive-python'), {
        // Only transform Python code blocks in these paths
        includePaths: ['/04-Python-Fundamentals/'],

        // Skip code blocks with these meta strings
        excludeMeta: ['nointeractive', 'static'],
      }],
    ],
  }
}
```

## Options

### `includePaths` (Array<string>)

Only transform files whose path includes one of these strings.

**Default**: `['/04-Python-Fundamentals/']`

**Example**:
```javascript
includePaths: [
  '/04-Python-Fundamentals/',
  '/05-Advanced-Python/',
  '/tutorials/',
]
```

### `excludeMeta` (Array<string>)

Skip code blocks with these strings in the meta field.

**Default**: `['nointeractive', 'static']`

**Example**:
```javascript
excludeMeta: ['nointeractive', 'static', 'example-only']
```

## Usage Examples

### Transform All Python Blocks (Default)

````markdown
```python
# This will become interactive
print("Hello!")
```
````

### Exclude Specific Blocks

Use meta strings to prevent transformation:

````markdown
```python nointeractive
# This stays as static code block
print("This is just an example")
```
````

````markdown
```python static
# This also stays static
print("Display only, no execution")
```
````

### Only Transform in Specific Paths

By default, only files in `/04-Python-Fundamentals/` are transformed.

Files outside this path keep their static code blocks.

## How It Works

1. **Build-Time Processing**: Runs during Docusaurus build, before MDX compilation
2. **AST Transformation**: Uses `unist-util-visit` to traverse the markdown AST
3. **Node Replacement**: Replaces `code` nodes (language: `python`) with `jsx` nodes
4. **Path Filtering**: Checks file path against `includePaths` before processing
5. **Meta Checking**: Skips nodes with excluded meta strings

## Dependencies

- `unist-util-visit` - For traversing the markdown AST

## Performance

- **Build Time**: Adds ~1-2 seconds to build (depending on number of Python files)
- **Runtime**: Zero impact (all transformation happens at build time)
- **Bundle Size**: No increase (uses existing InteractivePython component)

## Maintenance

### Adding New Paths

To transform Python blocks in additional directories:

```javascript
includePaths: [
  '/04-Python-Fundamentals/',
  '/new-python-course/',  // Add new paths here
]
```

### Adding New Exclusion Keywords

To add more ways to exclude blocks:

```javascript
excludeMeta: [
  'nointeractive',
  'static',
  'readonly',      // New keyword
  'display-only',  // New keyword
]
```

## Troubleshooting

### Code Blocks Not Transforming

1. Check file path is in `includePaths`
2. Verify language is exactly `python` (not `py` or `Python`)
3. Check for excluded meta strings

### Build Errors

If you see errors about `unist-util-visit`:

```bash
npm install --save-dev unist-util-visit
```

### Backticks or Dollar Signs in Code

The plugin automatically escapes `` ` `` and `$` characters for template literals.

No special handling needed in your markdown.

## Examples from Book

From `docs/04-Python-Fundamentals/16-operators-keywords-variables/02-comparison-operators.md`:

**Before** (Static):
````markdown
```python
x: int = 10
y: int = 5
equal: bool = x == y
print(f"{x} == {y}: {equal}")
```
````

**After** (Interactive):

Students can edit and run this code directly in the browser!

## Version History

- **v1.0.0** (2025-11-24): Initial release with path filtering and meta exclusion
