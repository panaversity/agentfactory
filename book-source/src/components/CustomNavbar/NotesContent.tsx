import React, { useEffect, useMemo, useState, useRef } from 'react';
import { useNotes } from '../../contexts/NotesContext';
import './notesContent.css';

const NotesContent: React.FC = () => {
  const {
    notes,
    addNote,
    updateNote,
    deleteNote,
    searchNotes,
    activeNoteId,
    setActiveNoteId,
    view,
    setView,
  } = useNotes();

  const [searchQuery, setSearchQuery] = useState('');
  const [noteTitle, setNoteTitle] = useState('New Note');
  const [noteContent, setNoteContent] = useState('');
  const contentEditableRef = useRef<HTMLDivElement>(null);

  // Filter notes based on search query
  const filteredNotes = useMemo(() => {
    return searchNotes(searchQuery);
  }, [searchNotes, searchQuery]);

  // Handle view changes
  useEffect(() => {
    if (view === 'add') {
      setNoteTitle('New Note');
      setNoteContent('');
      setActiveNoteId(null);
      if (contentEditableRef.current) {
        contentEditableRef.current.innerHTML = '';
      }
    } else if (view === 'edit' && activeNoteId && notes[activeNoteId]) {
      const note = notes[activeNoteId];
      setNoteTitle(note.title);
      setNoteContent(note.content);
      if (contentEditableRef.current) {
        contentEditableRef.current.innerHTML = note.content;
      }
    } else if (view === 'list') {
      setNoteTitle('New Note');
      setNoteContent('');
      setActiveNoteId(null);
      if (contentEditableRef.current) {
        contentEditableRef.current.innerHTML = '';
      }
    }
  }, [view, activeNoteId, notes, setActiveNoteId]);

  // Listen for text selection events to append to active note
  useEffect(() => {
    const handleAppendText = (event: CustomEvent) => {
      const { selectedText } = event.detail;

      if (view === 'add' || view === 'edit') {
        // Append to content
        if (contentEditableRef.current) {
          const currentContent = contentEditableRef.current.innerHTML;
          const newContent = currentContent
            ? `${currentContent}<br><br>${selectedText}`
            : selectedText;
          contentEditableRef.current.innerHTML = newContent;
          setNoteContent(newContent);
        }
      }
    };

    window.addEventListener('appendToNote' as any, handleAppendText);

    return () => {
      window.removeEventListener('appendToNote' as any, handleAppendText);
    };
  }, [view]);

  const handleSave = () => {
    const content = contentEditableRef.current?.innerHTML || '';

    if (!noteTitle.trim()) {
      alert('Please enter a note title');
      return;
    }

    if (view === 'edit' && activeNoteId) {
      updateNote(activeNoteId, {
        title: noteTitle.trim(),
        content: content,
      });
    } else {
      addNote({
        title: noteTitle.trim(),
        content: content,
      });
    }

    // Return to list view
    setView('list');
    setNoteTitle('New Note');
    setNoteContent('');
    if (contentEditableRef.current) {
      contentEditableRef.current.innerHTML = '';
    }
  };

  const handleClear = () => {
    if (contentEditableRef.current) {
      contentEditableRef.current.innerHTML = '';
      setNoteContent('');
    }
  };

  const handleEdit = (noteId: string) => {
    setActiveNoteId(noteId);
    setView('edit');
  };

  const handleDelete = (noteId: string) => {
    if (confirm('Are you sure you want to delete this note?')) {
      deleteNote(noteId);
    }
  };

  const handleContentChange = () => {
    if (contentEditableRef.current) {
      setNoteContent(contentEditableRef.current.innerHTML);
    }
  };

  const isClearDisabled = !noteContent.trim() && !contentEditableRef.current?.textContent?.trim();

  return (
    <div className='notes-content'>
      {/* List View */}
      {view === 'list' && (
        <div className='notes-content__list-view'>
          {/* Search */}
          <div className='notes-content__search'>
            <input
              type='text'
              className='notes-content__search-input'
              placeholder='Search notes by title or content...'
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
            />
          </div>

          {/* Notes list */}
          {filteredNotes.length === 0 ? (
            <div className='notes-content__empty'>
              <p>No notes found.</p>
              <button
                className='notes-content__btn notes-content__btn--primary'
                onClick={() => setView('add')}
              >
                Add Note
              </button>
            </div>
          ) : (
            <>
              <div className='notes-content__header-actions'>
                <button
                  className='notes-content__btn notes-content__btn--primary'
                  onClick={() => setView('add')}
                >
                  + Add Note
                </button>
              </div>
              <div className='notes-content__list'>
                {filteredNotes.map((note) => (
                  <div key={note.id} className='note-card'>
                    <div className='note-card__header'>
                      <h3 className='note-card__title'>{note.title}</h3>
                      <div className='note-card__actions'>
                        <button
                          className='note-card__btn note-card__btn--edit'
                          onClick={() => handleEdit(note.id)}
                          title='Edit note'
                        >
                          Edit
                        </button>
                        <button
                          className='note-card__btn note-card__btn--delete'
                          onClick={() => handleDelete(note.id)}
                          title='Delete note'
                        >
                          Delete
                        </button>
                      </div>
                    </div>
                    {note.content && (
                      <div
                        className='note-card__preview'
                        dangerouslySetInnerHTML={{
                          __html: note.content.length > 200
                            ? note.content.substring(0, 200) + '...'
                            : note.content
                        }}
                      />
                    )}
                    <div className='note-card__timestamp'>
                      {new Date(note.timestamp).toLocaleDateString('en-US', {
                        month: 'short',
                        day: 'numeric',
                        year: 'numeric',
                        hour: '2-digit',
                        minute: '2-digit',
                      })}
                    </div>
                  </div>
                ))}
              </div>
            </>
          )}
        </div>
      )}

      {/* Add/Edit View */}
      {(view === 'add' || view === 'edit') && (
        <div className='notes-content__edit-view'>
          <div className='notes-content__form'>
            {/* Title */}
            <div className='notes-content__section'>
              <label className='notes-content__label'>Title:</label>
              <input
                type='text'
                className='notes-content__input'
                placeholder='Enter note title...'
                value={noteTitle}
                onChange={(e) => setNoteTitle(e.target.value)}
              />
            </div>

            {/* Rich Textarea */}
            <div className='notes-content__section'>
              <label className='notes-content__label'>Description:</label>
              <div
                ref={contentEditableRef}
                className='notes-content__rich-textarea'
                contentEditable
                onInput={handleContentChange}
                data-placeholder='Enter your notes here... You can also select text from the page and add it to your notes.'
              />
            </div>

            {/* Actions */}
            <div className='notes-content__actions'>
              <button
                className='notes-content__btn notes-content__btn--primary'
                onClick={handleSave}
              >
                Save
              </button>
              <button
                className='notes-content__btn notes-content__btn--secondary'
                onClick={handleClear}
                disabled={isClearDisabled}
              >
                Clear
              </button>
              <button
                className='notes-content__btn notes-content__btn--tertiary'
                onClick={() => setView('list')}
              >
                Cancel
              </button>
            </div>

            <div className='notes-content__hint'>
              <p>💡 Tip: Select text from the page and click "Notes" to append it to your note.</p>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default NotesContent;
