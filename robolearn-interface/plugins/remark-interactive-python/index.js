/**
 * Remark Plugin: Interactive Python Code Blocks
 *
 * Automatically transforms Python code blocks in markdown into interactive
 * <InteractivePython /> components during the build process.
 *
 * Usage in docusaurus.config.ts:
 *
 * remarkPlugins: [
 *   require('./plugins/remark-interactive-python')
 * ]
 *
 * Transforms:
 * ```python
 * print("Hello")
 * ```
 *
 * Into:
 * <InteractivePython initialCode={`print("Hello")`} />
 */

const { visit } = require('unist-util-visit');

function remarkInteractivePython(options = {}) {
  const {
    // Only transform code blocks in specific paths (e.g., Python Fundamentals)
    includePaths = ['/05-Python-Fundamentals/'],
    // Exclude certain code blocks by checking meta string
    excludeMeta = ['nointeractive', 'static'],
  } = options;

  return (tree, file) => {
    // Check if this file should be processed
    // Normalize Windows backslashes to forward slashes for cross-platform compatibility
    const filePath = (file.history[0] || '').replace(/\\/g, '/');
    const shouldProcess = includePaths.some(path => filePath.includes(path));

    if (!shouldProcess) {
      return; // Skip files not in includePaths
    }

    const nodesToTransform = [];

    // Find all Python code blocks
    visit(tree, 'code', (node, index, parent) => {
      // Only transform Python code blocks
      if (node.lang !== 'python') {
        return;
      }

      // Skip if meta indicates it shouldn't be interactive
      if (node.meta && excludeMeta.some(meta => node.meta.includes(meta))) {
        return;
      }

      // Store node info for transformation
      nodesToTransform.push({ node, index, parent });
    });

    // Transform nodes (do this after visiting to avoid modifying while iterating)
    nodesToTransform.forEach(({ node, index, parent }) => {
      const code = node.value;

      // Escape backticks and dollar signs for template literal
      const escapedCode = code.replace(/`/g, '\\`').replace(/\$/g, '\\$');

      // Create MDX JSX element node for InteractivePython component
      const jsxNode = {
        type: 'mdxJsxFlowElement',
        name: 'InteractivePython',
        attributes: [
          {
            type: 'mdxJsxAttribute',
            name: 'initialCode',
            value: {
              type: 'mdxJsxAttributeValueExpression',
              value: `\`${escapedCode}\``,
              data: {
                estree: {
                  type: 'Program',
                  body: [
                    {
                      type: 'ExpressionStatement',
                      expression: {
                        type: 'TemplateLiteral',
                        quasis: [
                          {
                            type: 'TemplateElement',
                            value: { raw: escapedCode, cooked: code },
                            tail: true,
                          },
                        ],
                        expressions: [],
                      },
                    },
                  ],
                  sourceType: 'module',
                },
              },
            },
          },
        ],
        children: [],
      };

      // Replace code node with JSX node
      parent.children[index] = jsxNode;
    });
  };
}

module.exports = remarkInteractivePython;
