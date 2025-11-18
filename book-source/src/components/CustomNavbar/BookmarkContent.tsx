import { useLocation } from '@docusaurus/router';
import React, { useEffect, useMemo, useState } from 'react';
import type { Bookmark } from '../../contexts/BookmarkContext';
import { useBookmarks } from '../../contexts/BookmarkContext';
import './bookmarkContent.css';

const BookmarkContent: React.FC = () => {
  const location = useLocation();
  const {
    bookmarks,
    addBookmark,
    deleteBookmark,
    updateBookmarkNote,
    isBookmarked,
    currentDoc,
    selectedText,
    setSelectedText,
    selectedElementId,
    setSelectedElementId,
    initialView,
    setInitialView,
  } = useBookmarks();

  // Detect if current route is a docs page (routeBasePath is "/docs" in docusaurus.config.ts)
  const isDocsRoute = location.pathname.startsWith('/docs');

  // We consider it a doc page when both the route matches and
  // the currentDoc info has been populated by the DocItem wrapper.
  const isDocPage = isDocsRoute && !!currentDoc;

  const metadata =
    currentDoc?.metadata ||
    ({ title: 'Current Page', permalink: location.pathname } as const);

  const [view, setView] = useState<'add' | 'view'>(initialView);
  const [bookmarkName, setBookmarkName] = useState('');
  const [bookmarkNote, setBookmarkNote] = useState('');
  const [searchQuery, setSearchQuery] = useState('');
  const [editingNoteId, setEditingNoteId] = useState<string | null>(null);
  const [editNoteText, setEditNoteText] = useState('');

  // Check if entire page is bookmarked
  const isPageBookmarked = isBookmarked(metadata.permalink);

  // Reset to initialView when it changes (triggered from outside)
  useEffect(() => {
    setView(initialView);
    // If opening in 'add' mode with selected text, prefill the name field
    if (initialView === 'add' && selectedText) {
      // Extract first few words or complete heading
      const words = selectedText.trim().split(/\s+/);
      const prefillText =
        words.length > 10 ? words.slice(0, 10).join(' ') + '...' : selectedText;
      setBookmarkName(prefillText);
    }
  }, [initialView, selectedText]);

  // Clean up selected text when component unmounts or view changes
  useEffect(() => {
    return () => {
      setSelectedText('');
    };
  }, [setSelectedText]);

  // Handle bookmark entire page
  const handleBookmarkPage = () => {
    addBookmark({
      name: metadata.title,
      pageTitle: metadata.title,
      pageUrl: metadata.permalink,
      note: bookmarkNote,
      isEntirePage: true,
    });
    setBookmarkName('');
    setBookmarkNote('');
    setView('view');
    setInitialView('view');
  };

  // Handle bookmark with custom name (from selected text or manual entry)
  const handleSaveBookmark = () => {
    if (!bookmarkName.trim()) {
      alert('Please enter a name for the bookmark');
      return;
    }

    // Retrieve text anchor from sessionStorage
    const textAnchorStr = sessionStorage.getItem('pendingBookmarkTextAnchor');
    let textAnchor = undefined;
    if (textAnchorStr) {
      try {
        textAnchor = JSON.parse(textAnchorStr);
        sessionStorage.removeItem('pendingBookmarkTextAnchor');
      } catch (e) {
        console.error('Failed to parse text anchor:', e);
      }
    }

    addBookmark({
      name: bookmarkName.trim(),
      pageTitle: metadata.title,
      pageUrl: metadata.permalink,
      note: bookmarkNote.trim(),
      isEntirePage: false,
      elementId: selectedElementId,
      textAnchor: textAnchor,
    });

    setBookmarkName('');
    setBookmarkNote('');
    setSelectedText('');
    setSelectedElementId(undefined);
    setView('view');
    setInitialView('view');
  };

  // Handle remove bookmark (clear form)
  const handleRemoveBookmark = () => {
    setBookmarkName('');
    setBookmarkNote('');
    setSelectedText('');
    setSelectedElementId(undefined);
  };

  // Filter bookmarks by search query (search in name and note)
  const filteredBookmarks = useMemo(() => {
    const filtered = Object.values(bookmarks).filter((bookmark) => {
      if (!searchQuery) return true;
      const query = searchQuery.toLowerCase();
      return (
        bookmark.name.toLowerCase().includes(query) ||
        bookmark.note.toLowerCase().includes(query) ||
        bookmark.pageTitle.toLowerCase().includes(query)
      );
    });

    // Sort by timestamp (newest first)
    return filtered.sort((a, b) => b.timestamp - a.timestamp);
  }, [bookmarks, searchQuery]);

  // Group bookmarks by page
  const groupedBookmarks = useMemo(() => {
    const grouped: Record<string, Bookmark[]> = {};
    filteredBookmarks.forEach((bookmark) => {
      if (!grouped[bookmark.pageUrl]) {
        grouped[bookmark.pageUrl] = [];
      }
      grouped[bookmark.pageUrl].push(bookmark);
    });

    return grouped;
  }, [filteredBookmarks]);

  // Handle edit note
  const handleStartEditNote = (id: string, currentNote: string) => {
    setEditingNoteId(id);
    setEditNoteText(currentNote);
  };

  const handleSaveNote = (id: string) => {
    updateBookmarkNote(id, editNoteText);
    setEditingNoteId(null);
    setEditNoteText('');
  };

  const handleCancelEdit = () => {
    setEditingNoteId(null);
    setEditNoteText('');
  };

  // Handle bookmark navigation with smooth scrolling
  const handleBookmarkClick = (e: React.MouseEvent<HTMLAnchorElement>, bookmark: Bookmark) => {
    // For entire page bookmarks, let default navigation happen
    if (bookmark.isEntirePage) {
      return;
    }

    e.preventDefault();

    const currentPath = location.pathname;
    const targetPath = bookmark.pageUrl;

    // If we're on the same page, scroll immediately
    if (currentPath === targetPath) {
      if (bookmark.textAnchor) {
        scrollToText(bookmark.textAnchor);
      } else if (bookmark.elementId) {
        scrollToElement(bookmark.elementId);
      }
    } else {
      // Navigate to the page first, then scroll
      // Store the bookmark data in sessionStorage to scroll after navigation
      sessionStorage.setItem('pendingScrollBookmark', JSON.stringify({
        elementId: bookmark.elementId,
        textAnchor: bookmark.textAnchor,
      }));
      // Store flag to keep drawer open after navigation
      sessionStorage.setItem('keepBookmarkDrawerOpen', 'true');
      window.location.href = bookmark.pageUrl;
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
      // Found exact match with context
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

    // Strategy 4: Try just the selected text (may not be unique)
    index = fullText.indexOf(selectedText);

    if (index !== -1) {
      const endIndex = index + selectedText.length;
      return createRangeFromIndices(mainContent, index, endIndex);
    }

    return null;
  };

  // Create a Range object from character indices in the full text
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

      // Find start position
      if (startNode === null && startIndex >= currentIndex && startIndex < nextIndex) {
        startNode = node;
        startOffset = startIndex - currentIndex;
      }

      // Find end position
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
        // Get the bounding rectangle of the range
        const rect = range.getBoundingClientRect();

        // Scroll to bring the text into view
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
        span.className = 'bookmark-highlight';
        try {
          range.surroundContents(span);

          // Remove highlight after 2 seconds
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
          // If surroundContents fails (e.g., crosses element boundaries), just select it
          setTimeout(() => {
            selection?.removeAllRanges();
          }, 2000);
        }
      } else {
        console.warn('Could not find text in page:', textAnchor);
        alert('Could not locate the bookmarked text. The page content may have changed.');
      }
    }, 300); // Longer delay for cross-page navigation
  };

  // Scroll to element with smooth behavior (fallback for elementId-based bookmarks)
  const scrollToElement = (elementId: string) => {
    setTimeout(() => {
      const element = document.getElementById(elementId);
      if (element) {
        element.scrollIntoView({
          behavior: 'smooth',
          block: 'center',
          inline: 'nearest',
        });

        // Add a temporary highlight effect
        element.classList.add('bookmark-highlight');
        setTimeout(() => {
          element.classList.remove('bookmark-highlight');
        }, 2000);
      }
    }, 100);
  };

  // Check for pending scroll on mount or route change
  useEffect(() => {
    // Check for new bookmark-based scroll system
    const pendingBookmarkStr = sessionStorage.getItem('pendingScrollBookmark');
    if (pendingBookmarkStr) {
      try {
        const pendingBookmark = JSON.parse(pendingBookmarkStr);
        sessionStorage.removeItem('pendingScrollBookmark');

        // Prefer text anchor over element ID
        if (pendingBookmark.textAnchor) {
          scrollToText(pendingBookmark.textAnchor);
        } else if (pendingBookmark.elementId) {
          scrollToElement(pendingBookmark.elementId);
        }
      } catch (e) {
        console.error('Failed to parse pending scroll bookmark:', e);
      }
    }

    // Legacy: Support old elementId-based scrolling
    const scrollToElementId = sessionStorage.getItem('scrollToElementId');
    if (scrollToElementId) {
      sessionStorage.removeItem('scrollToElementId');
      scrollToElement(scrollToElementId);
    }
  }, [location.pathname]);

  return (
    <div className='bookmark-content'>
      {/* Header with view toggle */}
      <div className='bookmark-content__header'>
        <div className='bookmark-content__tabs'>
          <button
            className={`bookmark-content__tab ${
              view === 'view' ? 'bookmark-content__tab--active' : ''
            }`}
            onClick={() => {
              setView('view');
              setInitialView('view');
            }}
          >
            View Bookmarks ({Object.keys(bookmarks).length})
          </button>
          <button
            className={`bookmark-content__tab ${
              view === 'add' ? 'bookmark-content__tab--active' : ''
            }`}
            onClick={() => {
              setView('add');
              setInitialView('add');
            }}
          >
            Add Bookmark
          </button>
        </div>
      </div>

      {/* Add Bookmark View */}
      {view === 'add' && (
        <div className='bookmark-content__add'>
          {!isDocPage ? (
            <div className='bookmark-content__empty'>
              <p>📄 Navigate to a documentation page to add bookmarks.</p>
              <p style={{ fontSize: '14px', marginTop: '12px', color: '#666' }}>
                Bookmarks can only be created from documentation pages with
                content.
              </p>
            </div>
          ) : (
            <>
              <div className='bookmark-content__page-info'>
                <h3 className='bookmark-content__page-title'>
                  {metadata.title}
                </h3>
                <p className='bookmark-content__page-url'>
                  {metadata.permalink}
                </p>
              </div>

              {/* Bookmark entire page option */}
              <div className='bookmark-content__section'>
                <button
                  className='bookmark-content__bookmark-page-btn'
                  onClick={handleBookmarkPage}
                  disabled={isPageBookmarked}
                >
                  {isPageBookmarked
                    ? '✓ Page Already Bookmarked'
                    : '📄 Bookmark Entire Page'}
                </button>
              </div>

              {/* Name and Note fields */}
              <div className='bookmark-content__section'>
                <label className='bookmark-content__label'>Name:</label>
                <input
                  type='text'
                  className='bookmark-content__input'
                  placeholder='Enter bookmark name...'
                  value={bookmarkName}
                  onChange={(e) => setBookmarkName(e.target.value)}
                />
              </div>

              <div className='bookmark-content__section'>
                <label className='bookmark-content__label'>
                  Note (Optional):
                </label>
                <textarea
                  className='bookmark-content__textarea'
                  placeholder='Add a personal note to this bookmark...'
                  value={bookmarkNote}
                  onChange={(e) => setBookmarkNote(e.target.value)}
                  rows={3}
                />
              </div>

              {/* Save and Remove buttons */}
              <div className='bookmark-content__actions'>
                <button
                  className='bookmark-content__btn bookmark-content__btn--primary'
                  onClick={handleSaveBookmark}
                  disabled={!bookmarkName.trim()}
                >
                  Save
                </button>
                <button
                  disabled={!bookmarkName.trim() || !bookmarkNote.trim()}
                  className='bookmark-content__btn bookmark-content__btn--secondary'
                  onClick={handleRemoveBookmark}
                >
                  Clear
                </button>
              </div>
            </>
          )}
        </div>
      )}

      {/* View Bookmarks */}
      {view === 'view' && (
        <div className='bookmark-content__view'>
          {/* Search */}
          <div className='bookmark-content__search'>
            <input
              type='text'
              className='bookmark-content__search-input'
              placeholder='Search bookmarks by name or note...'
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
            />
          </div>

          {/* Bookmarks list */}
          {Object.keys(groupedBookmarks).length === 0 ? (
            <div className='bookmark-content__empty'>
              <p>No bookmarks found.</p>
              <button
                className='bookmark-content__btn bookmark-content__btn--primary'
                onClick={() => {
                  setView('add');
                  setInitialView('add');
                }}
              >
                Add Your First Bookmark
              </button>
            </div>
          ) : (
            <div className='bookmark-content__groups'>
              {Object.entries(groupedBookmarks).map(
                ([pageUrl, pageBookmarks]) => (
                  <div
                    key={pageUrl}
                    className='bookmark-group'
                  >
                    <h4 className='bookmark-group__title'>
                      {pageBookmarks[0].pageTitle}
                    </h4>
                    <div className='bookmark-group__items'>
                      {pageBookmarks.map((bookmark) => (
                        <div
                          key={bookmark.id}
                          className='bookmark-item'
                        >
                          <div className='bookmark-item__header'>
                            <a
                              href={bookmark.pageUrl}
                              className='bookmark-item__link'
                              onClick={(e) => handleBookmarkClick(e, bookmark)}
                            >
                              <span className='bookmark-item__icon'>
                                {bookmark.isEntirePage ? '📄' : '🔖'}
                              </span>
                              <span className='bookmark-item__heading'>
                                {bookmark.name}
                              </span>
                            </a>
                            <button
                              className='bookmark-item__delete'
                              onClick={() => deleteBookmark(bookmark.id)}
                              title='Delete bookmark'
                            >
                              ×
                            </button>
                          </div>
                          {editingNoteId === bookmark.id ? (
                            <div className='bookmark-item__note-edit'>
                              <textarea
                                className='bookmark-content__textarea'
                                value={editNoteText}
                                onChange={(e) =>
                                  setEditNoteText(e.target.value)
                                }
                                rows={2}
                                autoFocus
                              />
                              <div className='bookmark-item__note-actions'>
                                <button
                                  className='bookmark-content__btn bookmark-content__btn--small bookmark-content__btn--primary'
                                  onClick={() => handleSaveNote(bookmark.id)}
                                >
                                  Save
                                </button>
                                <button
                                  className='bookmark-content__btn bookmark-content__btn--small bookmark-content__btn--secondary'
                                  onClick={handleCancelEdit}
                                >
                                  Cancel
                                </button>
                              </div>
                            </div>
                          ) : (
                            <>
                              {bookmark.note && (
                                <p
                                  className='bookmark-item__note'
                                  onClick={() =>
                                    handleStartEditNote(
                                      bookmark.id,
                                      bookmark.note
                                    )
                                  }
                                >
                                  {bookmark.note}
                                </p>
                              )}
                              {!bookmark.note && (
                                <button
                                  className='bookmark-item__add-note'
                                  onClick={() =>
                                    handleStartEditNote(bookmark.id, '')
                                  }
                                >
                                  + Add note
                                </button>
                              )}
                            </>
                          )}
                          <div className='bookmark-item__timestamp'>
                            {new Date(bookmark.timestamp).toLocaleDateString(
                              'en-US',
                              {
                                month: 'short',
                                day: 'numeric',
                                year: 'numeric',
                                hour: '2-digit',
                                minute: '2-digit',
                              }
                            )}
                          </div>
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

export default BookmarkContent;
