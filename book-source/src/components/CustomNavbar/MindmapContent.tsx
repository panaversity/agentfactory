import React, { useState, useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import './mindmapContent.css';

// Safe hook wrapper that only uses useDoc when available
let useDocSafe: any = null;
let useFilteredAndTreeifiedTOCSafe: any = null;
let useThemeConfigSafe: any = null;

if (ExecutionEnvironment.canUseDOM) {
  try {
    const docModule = require('@docusaurus/plugin-content-docs/client');
    const themeModule = require('@docusaurus/theme-common/internal');
    const themeCommonModule = require('@docusaurus/theme-common');
    useDocSafe = docModule.useDoc;
    useFilteredAndTreeifiedTOCSafe = themeModule.useFilteredAndTreeifiedTOC;
    useThemeConfigSafe = themeCommonModule.useThemeConfig;
  } catch (e) {
    // Hooks not available
  }
}

type TOCTreeNode = {
  readonly value: string;
  readonly id: string;
  readonly level: number;
  readonly children: readonly TOCTreeNode[];
};

interface MindmapNode {
  id: string;
  text: string;
  level: number;
  children: MindmapNode[];
  x: number;
  y: number;
  collapsed: boolean;
}

const MindmapContent: React.FC = () => {
  const location = useLocation();
  const svgRef = useRef<SVGSVGElement>(null);
  const [selectedNode, setSelectedNode] = useState<string | null>(null);
  const [mindmapNodes, setMindmapNodes] = useState<MindmapNode[]>([]);
  const [zoom, setZoom] = useState(1);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [isPanning, setIsPanning] = useState(false);
  const [panStart, setPanStart] = useState({ x: 0, y: 0 });

  // Try to get doc context - will be null if not on a doc page
  let docContext: any = null;
  let themeConfig: any = null;
  let isDocPage = false;

  try {
    if (useDocSafe) {
      docContext = useDocSafe();
      themeConfig = useThemeConfigSafe();
      isDocPage = true;
    }
  } catch (e) {
    // Not on a doc page
    isDocPage = false;
  }

  const toc = docContext?.toc || [];
  const metadata = docContext?.metadata || { title: 'Current Page', permalink: location.pathname };
  const frontMatter = docContext?.frontMatter || {};

  // Get filtered/treeified TOC (only if we have the hook and we're on a doc page)
  let tocTree: any[] = [];

  if (isDocPage && useFilteredAndTreeifiedTOCSafe && themeConfig) {
    const minLevel = frontMatter.toc_min_heading_level ?? themeConfig.tableOfContents?.minHeadingLevel ?? 2;
    const maxLevel = frontMatter.toc_max_heading_level ?? themeConfig.tableOfContents?.maxHeadingLevel ?? 3;

    tocTree = useFilteredAndTreeifiedTOCSafe({
      toc,
      minHeadingLevel: minLevel,
      maxHeadingLevel: maxLevel,
    });
  }

  // Convert TOC tree to mindmap nodes with positions
  const convertToMindmapNodes = (
    nodes: readonly TOCTreeNode[],
    level: number = 0,
    startY: number = 100,
    parentCollapsed: boolean = false
  ): { nodes: MindmapNode[]; height: number } => {
    let currentY = startY;
    const mindmapNodes: MindmapNode[] = [];
    const nodeHeight = 60;
    const levelSpacing = 280;

    nodes.forEach((node, index) => {
      const mindmapNode: MindmapNode = {
        id: node.id,
        text: node.value,
        level: node.level,
        x: 150 + level * levelSpacing,
        y: currentY,
        children: [],
        collapsed: false,
      };

      const childResult = convertToMindmapNodes(
        node.children,
        level + 1,
        currentY,
        mindmapNode.collapsed
      );

      mindmapNode.children = childResult.nodes;
      mindmapNodes.push(mindmapNode);

      currentY += Math.max(nodeHeight, childResult.height);
    });

    return {
      nodes: mindmapNodes,
      height: currentY - startY || nodeHeight,
    };
  };

  useEffect(() => {
    if (tocTree.length > 0) {
      const result = convertToMindmapNodes(tocTree);
      setMindmapNodes(result.nodes);
    } else {
      setMindmapNodes([]);
    }
  }, [location.pathname, isDocPage]);

  // Flatten mindmap nodes for rendering
  const flattenNodes = (nodes: MindmapNode[], collapsed: boolean = false): MindmapNode[] => {
    let result: MindmapNode[] = [];
    nodes.forEach((node) => {
      if (!collapsed) {
        result.push(node);
        if (!node.collapsed && node.children.length > 0) {
          result = result.concat(flattenNodes(node.children, node.collapsed));
        }
      }
    });
    return result;
  };

  const allNodes = flattenNodes(mindmapNodes);

  // Handle node click
  const handleNodeClick = (nodeId: string) => {
    setSelectedNode(nodeId === selectedNode ? null : nodeId);
    // Scroll to heading on page
    const element = document.getElementById(nodeId);
    if (element) {
      element.scrollIntoView({ behavior: 'smooth', block: 'start' });
    }
  };

  // Handle node collapse/expand
  const toggleNodeCollapse = (nodeId: string) => {
    const toggleInTree = (nodes: MindmapNode[]): MindmapNode[] => {
      return nodes.map((node) => {
        if (node.id === nodeId) {
          return { ...node, collapsed: !node.collapsed };
        }
        if (node.children.length > 0) {
          return { ...node, children: toggleInTree(node.children) };
        }
        return node;
      });
    };

    setMindmapNodes(toggleInTree(mindmapNodes));
  };

  // Zoom controls
  const handleZoomIn = () => setZoom((prev) => Math.min(prev + 0.1, 2));
  const handleZoomOut = () => setZoom((prev) => Math.max(prev - 0.1, 0.5));
  const handleZoomReset = () => {
    setZoom(1);
    setPan({ x: 0, y: 0 });
  };

  // Pan controls
  const handleMouseDown = (e: React.MouseEvent) => {
    if (e.button === 0) {
      setIsPanning(true);
      setPanStart({ x: e.clientX - pan.x, y: e.clientY - pan.y });
    }
  };

  const handleMouseMove = (e: React.MouseEvent) => {
    if (isPanning) {
      setPan({
        x: e.clientX - panStart.x,
        y: e.clientY - panStart.y,
      });
    }
  };

  const handleMouseUp = () => {
    setIsPanning(false);
  };

  // Draw connections between nodes
  const renderConnections = () => {
    const connections: JSX.Element[] = [];

    const drawConnection = (parent: MindmapNode, child: MindmapNode, index: number) => {
      const startX = parent.x + 160;
      const startY = parent.y + 25;
      const endX = child.x;
      const endY = child.y + 25;

      const midX = (startX + endX) / 2;

      const path = `M ${startX} ${startY} C ${midX} ${startY}, ${midX} ${endY}, ${endX} ${endY}`;

      connections.push(
        <path
          key={`${parent.id}-${child.id}-${index}`}
          d={path}
          className="mindmap-connection"
          stroke="var(--mindmap-connection-color)"
          strokeWidth="2"
          fill="none"
          opacity="0.6"
        />
      );
    };

    const processNode = (node: MindmapNode) => {
      if (!node.collapsed && node.children.length > 0) {
        node.children.forEach((child, index) => {
          drawConnection(node, child, index);
          processNode(child);
        });
      }
    };

    mindmapNodes.forEach((node) => processNode(node));

    return connections;
  };

  // Render node
  const renderNode = (node: MindmapNode) => {
    const isSelected = selectedNode === node.id;
    const hasChildren = node.children.length > 0;

    return (
      <g key={node.id} className="mindmap-node-group">
        {/* Node rectangle */}
        <rect
          x={node.x}
          y={node.y}
          width={160}
          height={50}
          rx={8}
          className={`mindmap-node ${isSelected ? 'mindmap-node--selected' : ''}`}
          onClick={() => handleNodeClick(node.id)}
          style={{ cursor: 'pointer' }}
        />

        {/* Node text */}
        <text
          x={node.x + 80}
          y={node.y + 25}
          className="mindmap-node-text"
          textAnchor="middle"
          dominantBaseline="middle"
          onClick={() => handleNodeClick(node.id)}
          style={{ cursor: 'pointer', pointerEvents: 'none' }}
        >
          {node.text.length > 20 ? node.text.substring(0, 20) + '...' : node.text}
        </text>

        {/* Collapse/expand button */}
        {hasChildren && (
          <g
            className="mindmap-node-toggle"
            onClick={(e) => {
              e.stopPropagation();
              toggleNodeCollapse(node.id);
            }}
            style={{ cursor: 'pointer' }}
          >
            <circle
              cx={node.x + 160}
              cy={node.y + 25}
              r={10}
              className="mindmap-toggle-circle"
            />
            <text
              x={node.x + 160}
              y={node.y + 25}
              className="mindmap-toggle-text"
              textAnchor="middle"
              dominantBaseline="middle"
              style={{ pointerEvents: 'none' }}
            >
              {node.collapsed ? '+' : '−'}
            </text>
          </g>
        )}
      </g>
    );
  };

  return (
    <div className="mindmap-content">
      {!isDocPage ? (
        <div className="mindmap-content__empty">
          <p>🗺️ Navigate to a documentation page to view the mindmap.</p>
          <p style={{ fontSize: '14px', marginTop: '12px', color: '#666' }}>
            The mindmap visualizes the table of contents structure of documentation pages.
          </p>
        </div>
      ) : tocTree.length === 0 ? (
        <div className="mindmap-content__empty">
          <p>📄 This page has no table of contents.</p>
          <p style={{ fontSize: '14px', marginTop: '12px', color: '#666' }}>
            The mindmap requires pages with headings to visualize.
          </p>
        </div>
      ) : (
        <>
          {/* Controls */}
          <div className="mindmap-controls">
            <div className="mindmap-controls__group">
              <button
                className="mindmap-controls__btn"
                onClick={handleZoomIn}
                title="Zoom In"
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                  <path
                    d="M8 3V13M3 8H13"
                    stroke="currentColor"
                    strokeWidth="2"
                    strokeLinecap="round"
                  />
                </svg>
              </button>
              <button
                className="mindmap-controls__btn"
                onClick={handleZoomOut}
                title="Zoom Out"
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                  <path
                    d="M3 8H13"
                    stroke="currentColor"
                    strokeWidth="2"
                    strokeLinecap="round"
                  />
                </svg>
              </button>
              <button
                className="mindmap-controls__btn"
                onClick={handleZoomReset}
                title="Reset View"
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                  <path
                    d="M2 8C2 4.68629 4.68629 2 8 2C11.3137 2 14 4.68629 14 8C14 11.3137 11.3137 14 8 14C4.68629 14 2 11.3137 2 8Z"
                    stroke="currentColor"
                    strokeWidth="1.5"
                  />
                </svg>
              </button>
            </div>
            <div className="mindmap-controls__info">
              <span className="mindmap-controls__label">
                {metadata.title}
              </span>
              <span className="mindmap-controls__zoom">
                {Math.round(zoom * 100)}%
              </span>
            </div>
          </div>

          {/* SVG Canvas */}
          <div
            className="mindmap-canvas"
            onMouseDown={handleMouseDown}
            onMouseMove={handleMouseMove}
            onMouseUp={handleMouseUp}
            onMouseLeave={handleMouseUp}
            style={{ cursor: isPanning ? 'grabbing' : 'grab' }}
          >
            <svg
              ref={svgRef}
              className="mindmap-svg"
              width="100%"
              height="100%"
            >
              <g transform={`translate(${pan.x}, ${pan.y}) scale(${zoom})`}>
                {/* Root node (page title) */}
                <g className="mindmap-node-group">
                  <rect
                    x={20}
                    y={100}
                    width={200}
                    height={60}
                    rx={12}
                    className="mindmap-node mindmap-node--root"
                  />
                  <text
                    x={120}
                    y={130}
                    className="mindmap-node-text mindmap-node-text--root"
                    textAnchor="middle"
                    dominantBaseline="middle"
                  >
                    {metadata.title.length > 25
                      ? metadata.title.substring(0, 25) + '...'
                      : metadata.title}
                  </text>
                </g>

                {/* Connection from root to first level */}
                {mindmapNodes.length > 0 &&
                  mindmapNodes.map((node, index) => (
                    <path
                      key={`root-${node.id}`}
                      d={`M 220 130 C 280 130, 280 ${node.y + 25}, ${node.x} ${node.y + 25}`}
                      className="mindmap-connection"
                      stroke="var(--mindmap-connection-color)"
                      strokeWidth="2"
                      fill="none"
                      opacity="0.6"
                    />
                  ))}

                {/* Connections */}
                {renderConnections()}

                {/* Nodes */}
                {allNodes.map((node) => renderNode(node))}
              </g>
            </svg>
          </div>

          {/* Legend */}
          <div className="mindmap-legend">
            <div className="mindmap-legend__item">
              <div className="mindmap-legend__icon mindmap-legend__icon--click"></div>
              <span>Click node to navigate</span>
            </div>
            <div className="mindmap-legend__item">
              <div className="mindmap-legend__icon mindmap-legend__icon--drag"></div>
              <span>Drag to pan</span>
            </div>
            <div className="mindmap-legend__item">
              <div className="mindmap-legend__icon mindmap-legend__icon--expand"></div>
              <span>Click circle to expand/collapse</span>
            </div>
          </div>
        </>
      )}
    </div>
  );
};

export default MindmapContent;
