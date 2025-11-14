import React, { useState, useEffect } from 'react';

interface AIDialogProps {
  isOpen: boolean;
  onClose: () => void;
  content: { 
    contextOfSelection: string;
    basicTheory: string;
    instructions: string;
    example: string;
  } | null;
  isLoading: boolean;
  error: string | null;
}

const AIDialog: React.FC<AIDialogProps> = ({ isOpen, onClose, content, isLoading, error }) => {
  // Auto-close error messages after 5 seconds
  useEffect(() => {
    if (error) {
      const timer = setTimeout(() => {
        onClose(); // Close the dialog when there's an error after 5 seconds
      }, 5000); // 5 seconds

      // Clean up the timer if the component unmounts or error changes
      return () => clearTimeout(timer);
    }
  }, [error, onClose]);

  if (!isOpen) return null;

  return (
    <div className="ai-dialog-container" style={{
      position: 'fixed',
      top: '50%',
      left: '50%',
      transform: 'translate(-50%, -50%)',
      backgroundColor: 'var(--ifm-background-surface-color, white)',
      padding: '20px',
      borderRadius: 'var(--ifm-global-radius, 8px)',
      boxShadow: '0 2px 10px rgba(0, 0, 0, 0.1)',
      zIndex: 1000,
      width: 'min(95vw, 800px)', // Responsive width - 95% of viewport width or max 800px
      maxHeight: '90vh', // Increased from 80vh
      overflowY: 'auto',
      border: '1px solid var(--ifm-color-emphasis-200, #ddd)',
      fontFamily: 'var(--ifm-font-family-base)',
      color: 'var(--ifm-font-color-base)',
    }}>
      <div style={{
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        marginBottom: '15px',
        paddingBottom: '10px',
        borderBottom: '1px solid var(--ifm-color-emphasis-200, #eee)'
      }} className="ai-dialog-header">
        <h3 className="ai-panel-header" style={{ 
          margin: 0,
          fontWeight: 'var(--ifm-heading-font-weight, 600)',
          color: 'var(--ifm-heading-color, inherit)',
          fontSize: '1.5rem'
        }}>
          AI Insights
        </h3>
        <button
          onClick={onClose}
          style={{
            background: 'none',
            border: 'none',
            fontSize: '1.5rem',
            cursor: 'pointer',
            color: 'var(--ifm-color-emphasis-600)',
            padding: '0.25rem',
            borderRadius: '4px',
            width: '32px',
            height: '32px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
          }}
          onMouseOver={(e) => {
            const target = e.target as HTMLElement;
            target.style.backgroundColor = 'var(--ifm-color-emphasis-200)';
          }}
          onMouseOut={(e) => {
            const target = e.target as HTMLElement;
            target.style.backgroundColor = 'transparent';
          }}
        >
          ×
        </button>
      </div>

      {isLoading && (
        <div style={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          padding: '20px'
        }}>
          <div style={{
            width: '24px',
            height: '24px',
            border: '3px solid var(--ifm-color-emphasis-200)',
            borderTop: '3px solid var(--ifm-color-primary)',
            borderRadius: '50%',
            animation: 'spin 1s linear infinite'
          }}></div>
          <span style={{ marginLeft: '10px' }}>Loading AI content...</span>
        </div>
      )}
      {error && (
        <div style={{ 
          backgroundColor: 'rgba(244, 67, 54, 0.1)', 
          color: 'var(--ifm-color-danger)',
          padding: '12px', 
          borderRadius: 'var(--ifm-global-radius, 4px)',
          marginBottom: '15px',
          border: '1px solid var(--ifm-color-danger)',
          display: 'flex',
          alignItems: 'flex-start',
          gap: '10px'
        }}>
          <div style={{
            fontWeight: 'bold',
            minWidth: '60px'
          }}>
            Error:
          </div>
          <div style={{ flex: 1 }}>
            {error}
            <div style={{ 
              fontSize: '0.85em', 
              marginTop: '8px',
              opacity: 0.8 
            }}>
              This dialog will close automatically in a few seconds, or click the close button (×) to close it now.
            </div>
          </div>
        </div>
      )}
      {!isLoading && !error && !content && (
        <p style={{ 
          textAlign: 'center', 
          color: 'var(--ifm-color-emphasis-600)',
          fontStyle: 'italic',
          padding: '20px 0'
        }}>
          No information available for this selection.
        </p>
      )}

      {content && (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '16px' }}>
          {/* Split view: highlighted text on left, AI response on right */}
          <div className="ai-content-split-view" style={{ 
            display: 'flex', 
            gap: '16px', 
            minHeight: '200px', // Reduced min height for mobile
            flexDirection: 'row' // Will be overridden by CSS below
          }}>
            {/* Highlighted Text Panel (Left) */}
            <div className="ai-panel" style={{ 
              flex: 1, 
              padding: '16px', 
              border: '1px solid var(--ifm-color-emphasis-200)', 
              borderRadius: 'var(--ifm-global-radius, 6px)',
              backgroundColor: 'var(--ifm-background-color, #f9f9f9)',
              maxHeight: '200px', // Reduced for better mobile fit
              overflowY: 'auto',
              display: 'flex',
              flexDirection: 'column'
            }}>
              <h4 style={{ 
                marginTop: 0, 
                marginBottom: '12px', 
                color: 'var(--ifm-heading-color, inherit)',
                fontSize: '1rem',
                fontWeight: '600',
                borderBottom: '1px solid var(--ifm-color-emphasis-200)',
                paddingBottom: '6px'
              }}>
                Highlighted Text
              </h4>
              <p style={{ 
                margin: 0, 
                fontStyle: 'italic', 
                color: 'var(--ifm-font-color-base)',
                lineHeight: '1.5',
                flex: 1
              }}>
                {content.contextOfSelection || 'No highlighted text available.'}
              </p>
            </div>
            
            {/* AI Response Panel (Right) */}
            <div className="ai-panel" style={{ 
              flex: 1, 
              padding: '16px', 
              border: '1px solid var(--ifm-color-primary)',
              borderRadius: 'var(--ifm-global-radius, 6px)',
              backgroundColor: 'rgba(0, 31, 63, 0.03)', // Light blue tint matching primary color
              maxHeight: '200px', // Reduced for better mobile fit
              overflowY: 'auto',
              display: 'flex',
              flexDirection: 'column'
            }}>
              <h4 style={{ 
                marginTop: 0, 
                marginBottom: '12px', 
                color: 'var(--ifm-color-primary)',
                fontSize: '1rem',
                fontWeight: '600',
                borderBottom: '1px solid var(--ifm-color-primary)',
                paddingBottom: '6px'
              }}>
                AI Explanation
              </h4>
              <div style={{ 
                margin: 0, 
                color: 'var(--ifm-font-color-base)',
                lineHeight: '1.5',
                flex: 1,
                whiteSpace: 'pre-wrap' // Preserve formatting
              }}>
                {content.basicTheory || 'No AI explanation available.'}
              </div>
            </div>
          </div>
          
          {/* Implementation section */}
          {content.instructions && content.instructions !== "No implementation required." && (
            <div className="ai-panel" style={{ 
              padding: '16px', 
              border: '1px solid var(--ifm-color-emphasis-200)', 
              borderRadius: 'var(--ifm-global-radius, 6px)',
              backgroundColor: 'var(--ifm-background-surface-color, #fafafa)'
            }}>
              <h4 className="ai-panel-header" style={{ 
                marginTop: 0, 
                marginBottom: '10px', 
                color: '#4a6fa5', // Blue color for implementation
                fontSize: '1rem',
                fontWeight: '600'
              }}>
                Implementation
              </h4>
              <p style={{ margin: 0, lineHeight: '1.5' }}>
                {content.instructions}
              </p>
            </div>
          )}
          
          {/* Examples section */}
          {content.example && content.example !== "No examples provided." && (
            <div className="ai-panel" style={{ 
              padding: '16px', 
              border: '1px solid var(--ifm-color-emphasis-200)', 
              borderRadius: 'var(--ifm-global-radius, 6px)',
              backgroundColor: 'var(--ifm-background-surface-color, #fafafa)'
            }}>
              <h4 className="ai-panel-header" style={{ 
                marginTop: 0, 
                marginBottom: '10px', 
                color: '#2e7d32', // Green color for examples
                fontSize: '1rem',
                fontWeight: '600'
              }}>
                Examples
              </h4>
              <p style={{ margin: 0, lineHeight: '1.5' }}>
                {content.example}
              </p>
            </div>
          )}
        </div>
      )}
      
      <style>{`
        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }
        
        /* Mobile responsiveness */
        @media (max-width: 768px) {
          .ai-dialog-container {
            width: 95vw !important;
            max-height: 85vh !important;
            padding: 12px !important;
            top: 2vh !important;
            transform: translate(-50%, 0) !important;
            margin: auto !important;
          }
          
          .ai-content-split-view {
            flex-direction: column !important;
            gap: 12px !important;
            minHeight: 100px !important;
          }
          
          .ai-panel {
            max-height: 120px !important;
            padding: 10px !important;
          }
          
          .ai-dialog-header {
            flex-wrap: wrap !important;
            gap: 8px !important;
          }
          
          .ai-dialog-header h3.ai-panel-header {
            font-size: 1.2rem !important;
          }
          
          .ai-panel h4 {
            font-size: 0.9rem !important;
            margin-bottom: 8px !important;
          }
          
          .ai-panel p {
            font-size: 0.85rem !important;
            line-height: 1.4 !important;
          }
        }
        
        /* Tablet responsiveness */
        @media (max-width: 992px) and (min-width: 769px) {
          .ai-content-split-view {
            flex-direction: column !important;
          }
          
          .ai-panel {
            max-height: 180px !important;
          }
        }
      `}</style>
    </div>
  );
};

export default AIDialog;
