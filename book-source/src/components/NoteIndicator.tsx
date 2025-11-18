import React, { useState } from 'react';
import './noteIndicator.css';

interface NoteIndicatorProps {
  noteId: string;
  noteName: string;
  onClick: (noteId: string) => void;
}

const NoteIndicator: React.FC<NoteIndicatorProps> = ({ noteId, noteName, onClick }) => {
  const [showTooltip, setShowTooltip] = useState(false);

  const handleClick = (e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    onClick(noteId);
  };

  return (
    <span
      className="note-indicator"
      onClick={handleClick}
      onMouseEnter={() => setShowTooltip(true)}
      onMouseLeave={() => setShowTooltip(false)}
      data-note-id={noteId}
    >
      <span className="note-indicator__icon">📝</span>
      {showTooltip && (
        <span className="note-indicator__tooltip">
          {noteName}
        </span>
      )}
    </span>
  );
};

export default NoteIndicator;
