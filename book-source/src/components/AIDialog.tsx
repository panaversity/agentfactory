import React, { useState, useEffect } from 'react';

interface AIDialogProps {
  isOpen: boolean;
  onClose: () => void;
  content: { // This matches the AIContentResponse from the backend contract
    contextOfSelection: string;
    basicTheory: string;
    instructions: string;
    example: string;
  } | null;
  isLoading: boolean;
  error: string | null;
}

interface SectionProps {
  title: string;
  content: string;
  isEditable?: boolean;
  onContentChange?: (newContent: string) => void;
  onRemove?: () => void;
}

const Section: React.FC<SectionProps> = ({ title, content, isEditable = false, onContentChange, onRemove }) => {
  return (
    <div style={{ marginBottom: '10px', border: '1px solid #eee', padding: '8px', borderRadius: '4px' }}>
      <h4 style={{ marginTop: '0', marginBottom: '5px' }}>{title}</h4>
      {isEditable ? (
        <textarea
          value={content}
          onChange={(e) => onContentChange && onContentChange(e.target.value)}
          style={{ width: '100%', minHeight: '80px', border: '1px solid #ddd', padding: '5px' }}
        />
      ) : (
        <p style={{ margin: '0' }}>{content}</p>
      )}
      {onRemove && (
        <button onClick={onRemove} style={{ background: '#f44336', color: 'white', border: 'none', padding: '5px 10px', cursor: 'pointer', marginTop: '5px' }}>
          Remove
        </button>
      )}
    </div>
  );
};

const AIDialog: React.FC<AIDialogProps> = ({ isOpen, onClose, content, isLoading, error }) => {
  const [editableContent, setEditableContent] = useState(content);
  const [customSections, setCustomSections] = useState<{ id: string; title: string; content: string }[]>([]);

  useEffect(() => {
    setEditableContent(content);
    setCustomSections([]); // Reset custom sections when new content arrives
  }, [content]);

  if (!isOpen) return null;

  const handleContentChange = (sectionKey: keyof typeof editableContent, newContent: string) => {
    if (editableContent) {
      setEditableContent({
        ...editableContent,
        [sectionKey]: newContent,
      });
    }
  };

  const handleCustomSectionChange = (id: string, newContent: string) => {
    setCustomSections(prev =>
      prev.map(section => (section.id === id ? { ...section, content: newContent } : section))
    );
  };

  const handleAddSection = () => {
    setCustomSections(prev => [
      ...prev,
      { id: Date.now().toString(), title: 'New Section', content: '' },
    ]);
  };

  const handleRemoveCustomSection = (id: string) => {
    setCustomSections(prev => prev.filter(section => section.id !== id));
  };

  return (
    <div style={{
      position: 'fixed',
      top: '50%',
      left: '50%',
      transform: 'translate(-50%, -50%)',
      backgroundColor: 'white',
      padding: '20px',
      borderRadius: '8px',
      boxShadow: '0 4px 8px rgba(0, 0, 0, 0.1)',
      zIndex: 1000,
      width: '400px',
      maxHeight: '80vh',
      overflowY: 'auto',
      border: '1px solid #ddd'
    }}>
      <button
        onClick={onClose}
        style={{
          position: 'absolute',
          top: '10px',
          right: '10px',
          background: 'none',
          border: 'none',
          fontSize: '1.2em',
          cursor: 'pointer',
        }}
      >
        &times;
      </button>
      <h3 style={{ marginTop: '0' }}>AI Insights</h3>

      {isLoading && <p>Loading AI content...</p>}
      {error && <p style={{ color: 'red' }}>Error: {error}</p>}
      {!isLoading && !error && !content && <p>No information available for this selection.</p>}

      {editableContent && (
        <div>
          <Section
            title="Context of Selection"
            content={editableContent.contextOfSelection}
            isEditable
            onContentChange={(c) => handleContentChange('contextOfSelection', c)}
          />
          <Section
            title="Basic Theory"
            content={editableContent.basicTheory}
            isEditable
            onContentChange={(c) => handleContentChange('basicTheory', c)}
          />
          <Section
            title="Instructions"
            content={editableContent.instructions}
            isEditable
            onContentChange={(c) => handleContentChange('instructions', c)}
          />
          <Section
            title="Example"
            content={editableContent.example}
            isEditable
            onContentChange={(c) => handleContentChange('example', c)}
          />

          {customSections.map(section => (
            <Section
              key={section.id}
              title={section.title}
              content={section.content}
              isEditable
              onContentChange={(c) => handleCustomSectionChange(section.id, c)}
              onRemove={() => handleRemoveCustomSection(section.id)}
            />
          ))}

          <button onClick={handleAddSection} style={{ background: '#4CAF50', color: 'white', border: 'none', padding: '8px 15px', cursor: 'pointer', marginTop: '10px' }}>
            Add Custom Section
          </button>
        </div>
      )}
    </div>
  );
};

export default AIDialog;
