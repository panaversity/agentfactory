import { useLocation } from '@docusaurus/router';
import React, { useEffect, useMemo, useState } from 'react';
import type { Note } from '../../contexts/NotesContext';
import { useNotes } from '../../contexts/NotesContext';
import { useBookmarks } from '../../contexts/BookmarkContext';
import './notesContent.css';

const NotesContent: React.FC = () => {
  const location = useLocation();
  const {
    notes,
    addNote,
    deleteNote,
    updateNote,
    selectedText,
    setSelectedText,
    initialView,
    setInitialView,
    currentNoteId,
    setCurrentNoteId,
  } = useNotes();

  const { currentDoc } = useBookmarks();

  // Detect if current route is a docs page
  const isDocsRoute = location.pathname.startsWith('/docs');
  const isDocPage = isDocsRoute && !!currentDoc;

  const metadata =
    currentDoc?.metadata ||
    ({ title: 'Current Page', permalink: location.pathname } as const);

  const [view, setView] = useState<'add' | 'view'>(initialView);
  const [noteName, setNoteName] = useState('');
  const [noteContent, setNoteContent] = useState('');
  const [searchQuery, setSearchQuery] = useState('');
  const [editingNoteId, setEditingNoteId] = useState<string | null>(null);
  const [editNoteName, setEditNoteName] = useState('');
  const [editNoteContent, setEditNoteContent] = useState('');

  // Reset to initialView when it changes (triggered from outside)
  useEffect(() => {
    setView(initialView);
    // If opening in 'add' mode with selected text, prefill the name field
    if (initialView === 'add' && selectedText) {
      // Extract first few words or complete heading
      const words = selectedText.trim().split(/\s+/);
      const prefillText =
        words.length > 10 ? words.slice(0, 10).join(' ') + '...' : selectedText;
      setNoteName(prefillText);
    }
  }, [initialView, selectedText]);

  // Check if we should open a specific note in view mode
  useEffect(() => {
    if (currentNoteId && notes[currentNoteId]) {
      setView('view');
      setInitialView('view');
      // Scroll to the note in the list after a short delay
      setTimeout(() => {
        const noteElement = document.querySelector(`[data-note-id="${currentNoteId}"]`);
        if (noteElement) {
          noteElement.scrollIntoView({ behavior: 'smooth', block: 'center' });
          // Clear the current note ID after scrolling
          setTimeout(() => setCurrentNoteId(null), 500);
        }
      }, 100);
    }
  }, [currentNoteId, notes, setInitialView, setCurrentNoteId]);

  // Clean up selected text when component unmounts or view changes
  useEffect(() => {
    return () => {
      setSelectedText('');
    };
  }, [setSelectedText]);

  // Handle save note (from Add mode)
  const handleSaveNote = () => {
    if (!noteName.trim()) {
      alert('Please enter a name for the note');
      return;
    }

    if (!noteContent.trim()) {
      alert('Please enter content for the note');
      return;
    }

    // Retrieve text anchor from sessionStorage
    const textAnchorStr = sessionStorage.getItem('pendingNoteTextAnchor');
    let textAnchor = undefined;
    if (textAnchorStr) {
      try {
        textAnchor = JSON.parse(textAnchorStr);
        sessionStorage.removeItem('pendingNoteTextAnchor');
      } catch (e) {
        console.error('Failed to parse text anchor:', e);
      }
    }

    addNote({
      name: noteName.trim(),
      content: noteContent.trim(),
      pageTitle: metadata.title,
      pageUrl: metadata.permalink,
      textAnchor: textAnchor,
    });

    setNoteName('');
    setNoteContent('');
    setSelectedText('');
    setView('view');
    setInitialView('view');
  };

  // Handle clear note form
  const handleClearNote = () => {
    setNoteName('');
    setNoteContent('');
    setSelectedText('');
  };

  // Filter notes by search query (search in name and content)
  const filteredNotes = useMemo(() => {
    const filtered = Object.values(notes).filter((note) => {
      if (!searchQuery) return true;
      const query = searchQuery.toLowerCase();
      return (
        note.name.toLowerCase().includes(query) ||
        note.content.toLowerCase().includes(query) ||
        note.pageTitle.toLowerCase().includes(query)
      );
    });

    // Sort by last modified (newest first)
    return filtered.sort((a, b) => b.lastModified - a.lastModified);
  }, [notes, searchQuery]);

  // Group notes by page
  const groupedNotes = useMemo(() => {
    const grouped: Record<string, Note[]> = {};
    filteredNotes.forEach((note) => {
      if (!grouped[note.pageUrl]) {
        grouped[note.pageUrl] = [];
      }
      grouped[note.pageUrl].push(note);
    });

    return grouped;
  }, [filteredNotes]);

  // Handle start editing note
  const handleStartEdit = (note: Note) => {
    setEditingNoteId(note.id);
    setEditNoteName(note.name);
    setEditNoteContent(note.content);
  };

  // Handle save edited note
  const handleSaveEdit = (id: string) => {
    if (!editNoteName.trim() || !editNoteContent.trim()) {
      alert('Note name and content cannot be empty');
      return;
    }

    updateNote(id, {
      name: editNoteName.trim(),
      content: editNoteContent.trim(),
    });
    setEditingNoteId(null);
    setEditNoteName('');
    setEditNoteContent('');
  };

  // Handle cancel edit
  const handleCancelEdit = () => {
    setEditingNoteId(null);
    setEditNoteName('');
    setEditNoteContent('');
  };

  // Handle note navigation with smooth scrolling
  const handleNoteClick = (e: React.MouseEvent<HTMLAnchorElement>, note: Note) => {
    e.preventDefault();

    const currentPath = location.pathname;
    const targetPath = note.pageUrl;

    // If we're on the same page, scroll immediately
    if (currentPath === targetPath) {
      if (note.textAnchor) {
        scrollToText(note.textAnchor);
      }
    } else {
      // Navigate to the page first, then scroll
      sessionStorage.setItem('pendingScrollNote', JSON.stringify({
        textAnchor: note.textAnchor,
      }));
      // Store flag to keep drawer open and note ID to highlight
      sessionStorage.setItem('keepNotesDrawerOpen', 'true');
      sessionStorage.setItem('highlightNoteId', note.id);
      window.location.href = note.pageUrl;
    }
  };

  // Find text in the page using text anchor (prefix, selected text, suffix)
  const findTextInPage = (textAnchor: any): Range | null => {
    const mainContent = document.querySelector('article, .theme-doc-markdown, main, [role="main"]');
    if (!mainContent) return null;

    const fullText = mainContent.textContent || '';
    const { selectedText, prefix, suffix } = textAnchor;

    // Strategy 1: Try to find with prefix + selectedText + suffix
    const fullMatch = `${prefix}${selectedText}${suffix}`;
    let index = fullText.indexOf(fullMatch);

    if (index !== -1) {
      const startIndex = index + prefix.length;
      const endIndex = startIndex + selectedText.length;
      return createRangeFromIndices(mainContent, startIndex, endIndex);
    }

    // Strategy 2: Try selectedText with prefix only
    const prefixMatch = `${prefix}${selectedText}`;
    index = fullText.indexOf(prefixMatch);

    if (index !== -1) {
      const startIndex = index + prefix.length;
      const endIndex = startIndex + selectedText.length;
      return createRangeFromIndices(mainContent, startIndex, endIndex);
    }

    // Strategy 3: Try selectedText with suffix only
    const suffixMatch = `${selectedText}${suffix}`;
    index = fullText.indexOf(suffixMatch);

    if (index !== -1) {
      const endIndex = index + selectedText.length;
      return createRangeFromIndices(mainContent, index, endIndex);
    }

    // Strategy 4: Try just the selected text
    index = fullText.indexOf(selectedText);

    if (index !== -1) {
      const endIndex = index + selectedText.length;
      return createRangeFromIndices(mainContent, index, endIndex);
    }

    return null;
  };

  // Create a Range object from character indices
  const createRangeFromIndices = (container: Element, startIndex: number, endIndex: number): Range | null => {
    const walker = document.createTreeWalker(
      container,
      NodeFilter.SHOW_TEXT,
      null
    );

    let currentIndex = 0;
    let startNode: Node | null = null;
    let startOffset = 0;
    let endNode: Node | null = null;
    let endOffset = 0;

    let node: Node | null;
    while ((node = walker.nextNode())) {
      const textLength = node.textContent?.length || 0;
      const nextIndex = currentIndex + textLength;

      if (startNode === null && startIndex >= currentIndex && startIndex < nextIndex) {
        startNode = node;
        startOffset = startIndex - currentIndex;
      }

      if (endNode === null && endIndex >= currentIndex && endIndex <= nextIndex) {
        endNode = node;
        endOffset = endIndex - currentIndex;
      }

      if (startNode && endNode) break;

      currentIndex = nextIndex;
    }

    if (startNode && endNode) {
      const range = document.createRange();
      range.setStart(startNode, startOffset);
      range.setEnd(endNode, endOffset);
      return range;
    }

    return null;
  };

  // Scroll to text using text anchor
  const scrollToText = (textAnchor: any) => {
    setTimeout(() => {
      const range = findTextInPage(textAnchor);

      if (range) {
        const rect = range.getBoundingClientRect();

        window.scrollTo({
          top: window.scrollY + rect.top - window.innerHeight / 2,
          behavior: 'smooth',
        });

        // Highlight the text temporarily
        const selection = window.getSelection();
        selection?.removeAllRanges();
        selection?.addRange(range);

        // Add visual highlight
        const span = document.createElement('span');
        span.className = 'note-highlight';
        try {
          range.surroundContents(span);

          setTimeout(() => {
            const parent = span.parentNode;
            if (parent) {
              while (span.firstChild) {
                parent.insertBefore(span.firstChild, span);
              }
              parent.removeChild(span);
              parent.normalize();
            }
            selection?.removeAllRanges();
          }, 2000);
        } catch (e) {
          setTimeout(() => {
            selection?.removeAllRanges();
          }, 2000);
        }
      } else {
        console.warn('Could not find text in page:', textAnchor);
        alert('Could not locate the noted text. The page content may have changed.');
      }
    }, 300);
  };

  // Check for pending scroll on mount or route change
  useEffect(() => {
    const pendingNoteStr = sessionStorage.getItem('pendingScrollNote');
    if (pendingNoteStr) {
      try {
        const pendingNote = JSON.parse(pendingNoteStr);
        sessionStorage.removeItem('pendingScrollNote');

        if (pendingNote.textAnchor) {
          scrollToText(pendingNote.textAnchor);
        }
      } catch (e) {
        console.error('Failed to parse pending scroll note:', e);
      }
    }
  }, [location.pathname]);

  // Format timestamp
  const formatTimestamp = (timestamp: number, label: string) => {
    const date = new Date(timestamp).toLocaleDateString('en-US', {
      month: 'short',
      day: 'numeric',
      year: 'numeric',
      hour: '2-digit',
      minute: '2-digit',
    });
    return `${label}: ${date}`;
  };

  return (
    <div className='notes-content'>
      {/* Header with view toggle */}
      <div className='notes-content__header'>
        <div className='notes-content__tabs'>
          <button
            className={`notes-content__tab ${
              view === 'view' ? 'notes-content__tab--active' : ''
            }`}
            onClick={() => {
              setView('view');
              setInitialView('view');
            }}
          >
            View Notes ({Object.keys(notes).length})
          </button>
          <button
            className={`notes-content__tab ${
              view === 'add' ? 'notes-content__tab--active' : ''
            }`}
            onClick={() => {
              setView('add');
              setInitialView('add');
            }}
          >
            Add Note
          </button>
        </div>
      </div>

      {/* Add Note View */}
      {view === 'add' && (
        <div className='notes-content__add'>
          {!isDocPage ? (
            <div className='notes-content__empty'>
              <p>📝 Navigate to a documentation page to add notes.</p>
              <p style={{ fontSize: '14px', marginTop: '12px', color: '#666' }}>
                Notes can only be created from documentation pages by selecting text.
              </p>
            </div>
          ) : (
            <>
              <div className='notes-content__page-info'>
                <h3 className='notes-content__page-title'>
                  {metadata.title}
                </h3>
                <p className='notes-content__page-url'>
                  {metadata.permalink}
                </p>
              </div>

              {/* Name field */}
              <div className='notes-content__section'>
                <label className='notes-content__label'>Note Name:</label>
                <input
                  type='text'
                  className='notes-content__input'
                  placeholder='Enter note name...'
                  value={noteName}
                  onChange={(e) => setNoteName(e.target.value)}
                />
              </div>

              {/* Content field */}
              <div className='notes-content__section'>
                <label className='notes-content__label'>
                  Note Content:
                </label>
                <textarea
                  className='notes-content__textarea'
                  placeholder='Write your note here...'
                  value={noteContent}
                  onChange={(e) => setNoteContent(e.target.value)}
                  rows={6}
                />
              </div>

              {/* Save and Clear buttons */}
              <div className='notes-content__actions'>
                <button
                  className='notes-content__btn notes-content__btn--primary'
                  onClick={handleSaveNote}
                  disabled={!noteName.trim() || !noteContent.trim()}
                >
                  Save Note
                </button>
                <button
                  className='notes-content__btn notes-content__btn--secondary'
                  onClick={handleClearNote}
                  disabled={!noteName.trim() && !noteContent.trim()}
                >
                  Clear
                </button>
              </div>
            </>
          )}
        </div>
      )}

      {/* View Notes */}
      {view === 'view' && (
        <div className='notes-content__view'>
          {/* Search */}
          <div className='notes-content__search'>
            <input
              type='text'
              className='notes-content__search-input'
              placeholder='Search notes by name or content...'
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
            />
          </div>

          {/* Notes list */}
          {Object.keys(groupedNotes).length === 0 ? (
            <div className='notes-content__empty'>
              <p>📝 No notes found.</p>
              <button
                className='notes-content__btn notes-content__btn--primary'
                onClick={() => {
                  setView('add');
                  setInitialView('add');
                }}
              >
                Add Your First Note
              </button>
            </div>
          ) : (
            <div className='notes-content__groups'>
              {Object.entries(groupedNotes).map(
                ([pageUrl, pageNotes]) => (
                  <div
                    key={pageUrl}
                    className='note-group'
                  >
                    <h4 className='note-group__title'>
                      {pageNotes[0].pageTitle}
                    </h4>
                    <div className='note-group__items'>
                      {pageNotes.map((note) => (
                        <div
                          key={note.id}
                          className='note-item'
                          data-note-id={note.id}
                        >
                          {editingNoteId === note.id ? (
                            // Edit mode
                            <div className='note-item__edit'>
                              <div className='notes-content__section'>
                                <label className='notes-content__label'>Name:</label>
                                <input
                                  type='text'
                                  className='notes-content__input'
                                  value={editNoteName}
                                  onChange={(e) => setEditNoteName(e.target.value)}
                                  autoFocus
                                />
                              </div>
                              <div className='notes-content__section'>
                                <label className='notes-content__label'>Content:</label>
                                <textarea
                                  className='notes-content__textarea'
                                  value={editNoteContent}
                                  onChange={(e) => setEditNoteContent(e.target.value)}
                                  rows={4}
                                />
                              </div>
                              <div className='note-item__actions'>
                                <button
                                  className='notes-content__btn notes-content__btn--small notes-content__btn--primary'
                                  onClick={() => handleSaveEdit(note.id)}
                                >
                                  Save
                                </button>
                                <button
                                  className='notes-content__btn notes-content__btn--small notes-content__btn--secondary'
                                  onClick={handleCancelEdit}
                                >
                                  Cancel
                                </button>
                              </div>
                            </div>
                          ) : (
                            // View mode
                            <>
                              <div className='note-item__header'>
                                <a
                                  href={note.pageUrl}
                                  className='note-item__link'
                                  onClick={(e) => handleNoteClick(e, note)}
                                >
                                  <span className='note-item__icon'>📝</span>
                                  <span className='note-item__name'>
                                    {note.name}
                                  </span>
                                </a>
                                <div className='note-item__header-actions'>
                                  <button
                                    className='note-item__edit-btn'
                                    onClick={() => handleStartEdit(note)}
                                    title='Edit note'
                                  >
                                    ✏️
                                  </button>
                                  <button
                                    className='note-item__delete'
                                    onClick={() => deleteNote(note.id)}
                                    title='Delete note'
                                  >
                                    ×
                                  </button>
                                </div>
                              </div>
                              <p className='note-item__content'>
                                {note.content}
                              </p>
                              <div className='note-item__timestamps'>
                                <span className='note-item__timestamp'>
                                  {formatTimestamp(note.timestamp, 'Created')}
                                </span>
                                {note.lastModified !== note.timestamp && (
                                  <span className='note-item__timestamp note-item__timestamp--modified'>
                                    {formatTimestamp(note.lastModified, 'Modified')}
                                  </span>
                                )}
                              </div>
                            </>
                          )}
                        </div>
                      ))}
                    </div>
                  </div>
                )
              )}
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default NotesContent;
