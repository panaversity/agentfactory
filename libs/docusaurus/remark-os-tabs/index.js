/**
 * Remark Plugin: OS-Specific Tabs
 *
 * Transforms clean markdown syntax into Docusaurus Tabs components.
 * Keeps markdown content portable and readable.
 *
 * Usage in docusaurus.config.ts:
 *
 * remarkPlugins: [
 *   require('remark-directive'),  // Required first
 *   require('../../libs/docusaurus/remark-os-tabs')
 * ]
 *
 * Markdown syntax (use 4 colons if nesting other containers inside):
 *
 * ::::os-tabs
 *
 * ::windows
 * Windows-specific content here
 *
 * ::macos
 * macOS-specific content here
 *
 * ::linux
 * Linux-specific content here
 *
 * ::::
 *
 * Transforms into:
 * <Tabs groupId="operating-systems">
 *   <TabItem value="windows" label="Windows" default>
 *     Windows-specific content here
 *   </TabItem>
 *   ...
 * </Tabs>
 *
 * Note: Use 4 colons (::::os-tabs) when nesting :::tip, :::warning, etc.
 * inside the tabs. The closing marker must match (::::).
 */

const { visit } = require('unist-util-visit');

// OS configuration with labels and order
const OS_CONFIG = {
  windows: { label: 'Windows', default: true },
  macos: { label: 'macOS', default: false },
  linux: { label: 'Linux', default: false },
};

function remarkOsTabs(options = {}) {
  const {
    groupId = 'operating-systems',
  } = options;

  return (tree) => {
    const nodesToTransform = [];

    // Find all containerDirective nodes with name 'os-tabs'
    visit(tree, (node, index, parent) => {
      if (node.type === 'containerDirective' && node.name === 'os-tabs') {
        nodesToTransform.push({ node, index, parent });
      }
    });

    // Transform each os-tabs container
    nodesToTransform.forEach(({ node, index, parent }) => {
      const tabItems = [];

      // Process children to find ::windows, ::macos, ::linux leaf directives
      let currentOs = null;
      let currentContent = [];

      node.children.forEach((child) => {
        if (child.type === 'leafDirective' && OS_CONFIG[child.name]) {
          // Save previous OS content if exists
          if (currentOs) {
            tabItems.push({
              os: currentOs,
              children: currentContent,
            });
          }
          // Start new OS section
          currentOs = child.name;
          currentContent = [];
        } else if (currentOs) {
          // Add content to current OS section
          currentContent.push(child);
        }
      });

      // Don't forget the last OS section
      if (currentOs && currentContent.length > 0) {
        tabItems.push({
          os: currentOs,
          children: currentContent,
        });
      }

      // Build the Tabs JSX structure
      const tabsNode = {
        type: 'mdxJsxFlowElement',
        name: 'Tabs',
        attributes: [
          {
            type: 'mdxJsxAttribute',
            name: 'groupId',
            value: groupId,
          },
        ],
        children: tabItems.map(({ os, children }) => {
          const config = OS_CONFIG[os];
          const attributes = [
            {
              type: 'mdxJsxAttribute',
              name: 'value',
              value: os,
            },
            {
              type: 'mdxJsxAttribute',
              name: 'label',
              value: config.label,
            },
          ];

          // Add default attribute for Windows
          if (config.default) {
            attributes.push({
              type: 'mdxJsxAttribute',
              name: 'default',
              value: null,
            });
          }

          return {
            type: 'mdxJsxFlowElement',
            name: 'TabItem',
            attributes,
            children,
          };
        }),
      };

      // Replace the container with the Tabs component
      parent.children[index] = tabsNode;
    });
  };
}

module.exports = remarkOsTabs;
