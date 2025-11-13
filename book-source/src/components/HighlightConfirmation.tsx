import React, { useEffect } from 'react';

interface HighlightConfirmationProps {
  highlightedText: string;
  onConfirm: () => void;
  onCancel: () => void;
  position: { x: number; y: number };
  isVisible: boolean;
}

const HighlightConfirmation: React.FC<HighlightConfirmationProps> = ({ 
  highlightedText, 
  onConfirm, 
  onCancel, 
  position, 
  isVisible 
}) => {
  useEffect(() => {
    if (!isVisible) return;

    const timer = setTimeout(() => {
      onCancel(); // Auto-cancel after 10 seconds
    }, 10000);

    // Listen for Enter key to confirm
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Enter') {
        onConfirm();
      } else if (e.key === 'Escape') {
        onCancel();
      }
    };

    window.addEventListener('keydown', handleKeyDown);

    return () => {
      clearTimeout(timer);
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [isVisible, onConfirm, onCancel]);

  if (!isVisible) return null;

  return (
    <div 
      style={{
        position: 'fixed',
        left: position.x,
        top: position.y,
        backgroundColor: 'var(--ifm-background-surface-color, white)',
        border: '1px solid var(--ifm-color-emphasis-300)',
        borderRadius: '8px',
        padding: '12px',
        zIndex: 9999,
        boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
        maxWidth: '300px',
        fontFamily: 'var(--ifm-font-family-base)',
        fontSize: '0.9rem',
        color: 'var(--ifm-font-color-base)',
        display: 'flex',
        flexDirection: 'column',
        gap: '8px'
      }}
    >
      <div style={{
        whiteSpace: 'nowrap',
        overflow: 'hidden',
        textOverflow: 'ellipsis',
        maxWidth: '275px',
        fontStyle: 'italic',
        color: 'var(--ifm-color-emphasis-700)',
        marginBottom: '4px'
      }}>
        "{highlightedText.substring(0, 60)}{highlightedText.length > 60 ? '...' : ''}"
      </div>
      
      <div style={{
        display: 'flex',
        gap: '8px'
      }}>
        <button
          onClick={onConfirm}
          style={{
            flex: 1,
            padding: '6px 12px',
            backgroundColor: 'var(--ifm-color-primary)',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: 'pointer',
            fontSize: '0.85rem'
          }}
          onMouseOver={(e) => {
            (e.target as HTMLElement).style.opacity = '0.9';
          }}
          onMouseOut={(e) => {
            (e.target as HTMLElement).style.opacity = '1';
          }}
        >
          Generate
        </button>
        <button
          onClick={onCancel}
          style={{
            flex: 1,
            padding: '6px 12px',
            backgroundColor: 'var(--ifm-color-emphasis-200)',
            color: 'var(--ifm-font-color-base)',
            border: 'none',
            borderRadius: '4px',
            cursor: 'pointer',
            fontSize: '0.85rem'
          }}
          onMouseOver={(e) => {
            (e.target as HTMLElement).style.opacity = '0.9';
          }}
          onMouseOut={(e) => {
            (e.target as HTMLElement).style.opacity = '1';
          }}
        >
          Cancel
        </button>
      </div>
      
      <div style={{
        fontSize: '0.75rem',
        color: 'var(--ifm-color-emphasis-600)',
        textAlign: 'center',
        marginTop: '4px'
      }}>
        Auto-cancel in 10s • Press Enter to confirm • Esc to cancel
      </div>
    </div>
  );
};

export default HighlightConfirmation;