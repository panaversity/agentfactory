import React, { useEffect } from 'react';
import BookmarkContent from './BookmarkContent';
import MindmapContent from './MindmapContent';
import AssessmentContent from './AssessmentContent';
import { useBookmarks } from '../../contexts/BookmarkContext';
import './rightDrawer.css';

interface RightDrawerProps {
  isOpen: boolean;
  onClose: () => void;
  title: string;
  children?: React.ReactNode;
}

const RightDrawer: React.FC<RightDrawerProps> = ({ isOpen, onClose, title, children }) => {
  const { setHideTOC } = useBookmarks();

  // Add/remove body class to shift content when drawer opens/closes
  useEffect(() => {
    if (isOpen) {
      document.body.classList.add('drawer-open');
      setHideTOC(true);
    } else {
      document.body.classList.remove('drawer-open');
      setHideTOC(false);
    }

    // Cleanup on unmount
    return () => {
      document.body.classList.remove('drawer-open');
      setHideTOC(false);
    };
  }, [isOpen, setHideTOC]);

  // Handle ESC key to close drawer
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

  return (
    <>
      {/* Drawer Panel */}
      <div className={`right-drawer__panel ${isOpen ? 'right-drawer__panel--open' : ''}`}>
        {/* Drawer Header */}
        <div className="right-drawer__header">
          <h2 className="right-drawer__title">{title}</h2>
          <button
            className="right-drawer__close-button"
            onClick={onClose}
            aria-label="Close drawer"
          >
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

        {/* Drawer Content */}
        <div className="right-drawer__content">
          {title === 'Bookmark' ? (
            <BookmarkContent />
          ) : title === 'Mindmap' ? (
            <MindmapContent />
          ) : title === 'Assessment' ? (
            <AssessmentContent onClose={onClose} />
          ) : children ? (
            children
          ) : (
            <div className="right-drawer__placeholder">
              <p className="right-drawer__placeholder-text">
                Make changes to your profile here. Click save when you're done.
              </p>

              {/* Example Form Fields */}
              <div className="right-drawer__form">
                <div className="right-drawer__form-group">
                  <label className="right-drawer__label">Name</label>
                  <input
                    type="text"
                    className="right-drawer__input"
                    placeholder="Pedro Duarte"
                  />
                </div>

                <div className="right-drawer__form-group">
                  <label className="right-drawer__label">Username</label>
                  <input
                    type="text"
                    className="right-drawer__input"
                    placeholder="@peduarte"
                  />
                </div>
              </div>

              {/* Action Buttons */}
              <div className="right-drawer__actions">
                <button className="right-drawer__button right-drawer__button--primary">
                  Save changes
                </button>
                <button className="right-drawer__button right-drawer__button--secondary" onClick={onClose}>
                  Close
                </button>
              </div>
            </div>
          )}
        </div>
      </div>
    </>
  );
};

export default RightDrawer;
