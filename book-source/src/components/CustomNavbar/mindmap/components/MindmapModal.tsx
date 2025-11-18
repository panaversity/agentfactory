import React, { useCallback, useEffect, useState, useRef } from 'react';
import { createPortal } from 'react-dom';
import {
  ReactFlow,
  Background,
  Controls,
  MiniMap,
  useNodesState,
  useEdgesState,
  Node,
  Edge,
  OnNodesChange,
  OnEdgesChange,
  ReactFlowInstance,
} from '@xyflow/react';
import '@xyflow/react/dist/style.css';
import { MindmapData } from '../types';
import './MindmapModal.css';

interface MindmapModalProps {
  isOpen: boolean;
  mindmap: MindmapData | null;
  onClose: () => void;
}

const MindmapModal: React.FC<MindmapModalProps> = ({ isOpen, mindmap, onClose }) => {
  const [nodes, setNodes, onNodesChange] = useNodesState<Node>([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState<Edge>([]);
  const [isMaximized, setIsMaximized] = useState(false);
  const modalRef = useRef<HTMLDivElement>(null);
  const reactFlowInstance = useRef<ReactFlowInstance | null>(null);

  // Update nodes and edges when mindmap changes
  useEffect(() => {
    if (mindmap) {
      setNodes(mindmap.nodes);
      setEdges(mindmap.edges);
    }
  }, [mindmap, setNodes, setEdges]);

  // Handle node click - navigate to URL if available
  const onNodeClick = useCallback((_event: React.MouseEvent, node: Node) => {
    if (node.data?.url) {
      window.location.href = node.data.url;
    }
  }, []);

  // Handle ESC key to close modal
  useEffect(() => {
    if (!isOpen) return;

    const handleEscKey = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        event.preventDefault();
        onClose();
      }
    };

    document.addEventListener('keydown', handleEscKey);

    return () => {
      document.removeEventListener('keydown', handleEscKey);
    };
  }, [isOpen, onClose]);

  // Prevent body scroll when modal is open
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = '';
    }

    return () => {
      document.body.style.overflow = '';
    };
  }, [isOpen]);

  // Toggle maximized panel state
  const toggleMaximize = useCallback(() => {
    setIsMaximized((prev) => {
      const newState = !prev;

      // Auto-fit view when maximizing
      if (newState && reactFlowInstance.current) {
        // Small delay to ensure panel expansion is complete
        setTimeout(() => {
          reactFlowInstance.current?.fitView({
            padding: 0.2,
            duration: 400,
          });
        }, 100);
      }

      return newState;
    });
  }, []);


  if (!isOpen || !mindmap) {
    return null;
  }

  return createPortal(
    <div className="mindmap-modal-overlay" onClick={onClose}>
      <div
        ref={modalRef}
        className={`mindmap-modal ${isMaximized ? 'mindmap-modal--maximized' : ''}`}
        onClick={(e) => e.stopPropagation()}
      >
        {/* Modal Header */}
        <div className="mindmap-modal__header">
          <h2 className="mindmap-modal__title">{mindmap.title}</h2>
          <div className="mindmap-modal__header-actions">
            <button
              className="mindmap-modal__maximize"
              onClick={toggleMaximize}
              aria-label={isMaximized ? 'Minimize panel' : 'Maximize panel'}
              title={isMaximized ? 'Minimize panel' : 'Maximize panel'}
            >
              {isMaximized ? (
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
                  <path
                    d="M8 3V8H3M21 8H16V3M16 21V16H21M3 16H8V21"
                    stroke="currentColor"
                    strokeWidth="2"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  />
                </svg>
              ) : (
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
                  <path
                    d="M8 3H5C3.89543 3 3 3.89543 3 5V8M21 8V5C21 3.89543 20.1046 3 19 3H16M16 21H19C20.1046 21 21 20.1046 21 19V16M3 16V19C3 20.1046 3.89543 21 5 21H8"
                    stroke="currentColor"
                    strokeWidth="2"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  />
                </svg>
              )}
            </button>
            <button className="mindmap-modal__close" onClick={onClose} aria-label="Close modal">
              <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
                <path
                  d="M18 6L6 18M6 6L18 18"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
              </svg>
            </button>
          </div>
        </div>

        {/* ReactFlow Canvas */}
        <div className="mindmap-modal__content">
          <ReactFlow
            nodes={nodes}
            edges={edges}
            onNodesChange={onNodesChange as OnNodesChange}
            onEdgesChange={onEdgesChange as OnEdgesChange}
            onNodeClick={onNodeClick}
            onInit={(instance) => {
              reactFlowInstance.current = instance;
            }}
            fitView
            attributionPosition="bottom-left"
            minZoom={0.1}
            maxZoom={2}
            defaultEdgeOptions={{
              animated: false,
              style: { strokeWidth: 2 },
            }}
          >
            <Background color="var(--ifm-color-emphasis-300)" gap={16} />
            <Controls
              showZoom={true}
              showFitView={true}
              showInteractive={false}
              position="top-right"
            />
            <MiniMap
              nodeColor={(node) => {
                if (node.type === 'input') return '#667eea';
                if (node.type === 'output') return '#a0aec0';
                return 'var(--ifm-color-primary-light)';
              }}
              maskColor="rgba(0, 0, 0, 0.1)"
              position="bottom-right"
              style={{
                background: 'var(--ifm-background-surface-color)',
                border: '1px solid var(--ifm-color-emphasis-300)',
              }}
            />
          </ReactFlow>
        </div>

        {/* Modal Footer */}
        <div className="mindmap-modal__footer">
          <div className="mindmap-modal__info">
            <span className="mindmap-modal__node-count">
              {nodes.length} node{nodes.length !== 1 ? 's' : ''}
            </span>
            <span className="mindmap-modal__hint">
              💡 Click nodes to navigate • Drag to pan • Scroll to zoom
            </span>
          </div>
          <button className="mindmap-modal__close-btn" onClick={onClose}>
            Close
          </button>
        </div>
      </div>
    </div>,
    document.body
  );
};

export default MindmapModal;
