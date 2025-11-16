import React, { useState, useMemo } from 'react';
import { useLocation } from '@docusaurus/router';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { useBookmarks } from '../../contexts/BookmarkContext';
import type { Bookmark } from '../../contexts/BookmarkContext';
import './bookmarkContent.css';

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

interface TOCItemProps {
  node: TOCTreeNode;
  selectedHeadings: Set<string>;
  onToggle: (id: string, text: string, level: number) => void;
}

const TOCItemNode: React.FC<TOCItemProps> = ({ node, selectedHeadings, onToggle }) => {
  const isSelected = selectedHeadings.has(node.id);

  return (
    <li className={`bookmark-toc-item bookmark-toc-item--level-${node.level}`}>
      <label className="bookmark-toc-label">
        <input
          type="checkbox"
          checked={isSelected}
          onChange={() => onToggle(node.id, node.value, node.level)}
          className="bookmark-toc-checkbox"
        />
        <span className="bookmark-toc-text">{node.value}</span>
      </label>
      {node.children.length > 0 && (
        <ul className="bookmark-toc-list">
          {node.children.map((child) => (
            <TOCItemNode
              key={child.id}
              node={child}
              selectedHeadings={selectedHeadings}
              onToggle={onToggle}
            />
          ))}
        </ul>
      )}
    </li>
  );
};

const BookmarkContent: React.FC = () => {
  const location = useLocation();
  const { bookmarks, addBookmark, deleteBookmark, updateBookmarkNote, isBookmarked } = useBookmarks();

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

  const [view, setView] = useState<'add' | 'view'>('view');
  const [selectedHeadings, setSelectedHeadings] = useState<Set<string>>(new Set());
  const [bookmarkNote, setBookmarkNote] = useState('');
  const [searchQuery, setSearchQuery] = useState('');
  const [editingNoteId, setEditingNoteId] = useState<string | null>(null);
  const [editNoteText, setEditNoteText] = useState('');

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

  // Check if entire page is bookmarked
  const isPageBookmarked = isBookmarked(metadata.permalink);

  // Handle heading selection toggle
  const handleToggleHeading = (id: string, text: string, level: number) => {
    setSelectedHeadings((prev) => {
      const updated = new Set(prev);
      if (updated.has(id)) {
        updated.delete(id);
      } else {
        updated.add(id);
      }
      return updated;
    });
  };

  // Handle bookmark entire page
  const handleBookmarkPage = () => {
    addBookmark({
      pageTitle: metadata.title,
      pageUrl: metadata.permalink,
      note: bookmarkNote,
    });
    setBookmarkNote('');
    setView('view');
  };

  // Handle bookmark selected headings
  const handleBookmarkHeadings = () => {
    if (selectedHeadings.size === 0) return;

    // Flatten the TOC tree to find selected headings
    const flattenTOC = (nodes: readonly TOCTreeNode[]): TOCTreeNode[] => {
      return nodes.reduce((acc, node) => {
        return [...acc, node, ...flattenTOC(node.children)];
      }, [] as TOCTreeNode[]);
    };

    const flatTOC = flattenTOC(tocTree);

    selectedHeadings.forEach((headingId) => {
      const heading = flatTOC.find((item) => item.id === headingId);
      if (heading) {
        addBookmark({
          pageTitle: metadata.title,
          pageUrl: metadata.permalink,
          headingId: heading.id,
          headingText: heading.value,
          headingLevel: heading.level,
          note: bookmarkNote,
        });
      }
    });

    setSelectedHeadings(new Set());
    setBookmarkNote('');
    setView('view');
  };

  // Group bookmarks by page
  const groupedBookmarks = useMemo(() => {
    const filtered = Object.values(bookmarks).filter((bookmark) => {
      if (!searchQuery) return true;
      const query = searchQuery.toLowerCase();
      return (
        bookmark.pageTitle.toLowerCase().includes(query) ||
        bookmark.headingText?.toLowerCase().includes(query) ||
        bookmark.note.toLowerCase().includes(query)
      );
    });

    const grouped: Record<string, Bookmark[]> = {};
    filtered.forEach((bookmark) => {
      if (!grouped[bookmark.pageUrl]) {
        grouped[bookmark.pageUrl] = [];
      }
      grouped[bookmark.pageUrl].push(bookmark);
    });

    // Sort by timestamp within each group
    Object.keys(grouped).forEach((pageUrl) => {
      grouped[pageUrl].sort((a, b) => b.timestamp - a.timestamp);
    });

    return grouped;
  }, [bookmarks, searchQuery]);

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

  return (
    <div className="bookmark-content">
      {/* Header with view toggle */}
      <div className="bookmark-content__header">
        <div className="bookmark-content__tabs">
          <button
            className={`bookmark-content__tab ${view === 'view' ? 'bookmark-content__tab--active' : ''}`}
            onClick={() => setView('view')}
          >
            View Bookmarks ({Object.keys(bookmarks).length})
          </button>
          <button
            className={`bookmark-content__tab ${view === 'add' ? 'bookmark-content__tab--active' : ''}`}
            onClick={() => setView('add')}
          >
            Add Bookmark
          </button>
        </div>
      </div>

      {/* Add Bookmark View */}
      {view === 'add' && (
        <div className="bookmark-content__add">
          {!isDocPage ? (
            <div className="bookmark-content__empty">
              <p>📄 Navigate to a documentation page to add bookmarks.</p>
              <p style={{ fontSize: '14px', marginTop: '12px', color: '#666' }}>
                Bookmarks can only be created from documentation pages with content and headings.
              </p>
            </div>
          ) : (
            <>
              <div className="bookmark-content__page-info">
                <h3 className="bookmark-content__page-title">{metadata.title}</h3>
                <p className="bookmark-content__page-url">{metadata.permalink}</p>
              </div>

              {/* Bookmark entire page option */}
              <div className="bookmark-content__section">
                <button
                  className="bookmark-content__bookmark-page-btn"
                  onClick={handleBookmarkPage}
                  disabled={isPageBookmarked}
                >
                  {isPageBookmarked ? '✓ Page Already Bookmarked' : '📄 Bookmark Entire Page'}
                </button>
              </div>

              {/* TOC headings selection */}
              {tocTree.length > 0 && (
                <div className="bookmark-content__section">
              <h4 className="bookmark-content__section-title">Or Select Specific Headings:</h4>
              <div className="bookmark-content__toc">
                <ul className="bookmark-toc-list">
                  {tocTree.map((node) => (
                    <TOCItemNode
                      key={node.id}
                      node={node}
                      selectedHeadings={selectedHeadings}
                      onToggle={handleToggleHeading}
                    />
                  ))}
                </ul>
              </div>
            </div>
          )}

              {/* Note input */}
              <div className="bookmark-content__section">
                <label className="bookmark-content__label">Add Note (Optional):</label>
                <textarea
                  className="bookmark-content__textarea"
                  placeholder="Add a personal note to this bookmark..."
                  value={bookmarkNote}
                  onChange={(e) => setBookmarkNote(e.target.value)}
                  rows={3}
                />
              </div>

              {/* Action buttons */}
              {selectedHeadings.size > 0 && (
                <div className="bookmark-content__actions">
                  <button className="bookmark-content__btn bookmark-content__btn--primary" onClick={handleBookmarkHeadings}>
                    Bookmark {selectedHeadings.size} Heading{selectedHeadings.size > 1 ? 's' : ''}
                  </button>
                  <button
                    className="bookmark-content__btn bookmark-content__btn--secondary"
                    onClick={() => setSelectedHeadings(new Set())}
                  >
                    Clear Selection
                  </button>
                </div>
              )}
            </>
          )}
        </div>
      )}

      {/* View Bookmarks */}
      {view === 'view' && (
        <div className="bookmark-content__view">
          {/* Search */}
          <div className="bookmark-content__search">
            <input
              type="text"
              className="bookmark-content__search-input"
              placeholder="Search bookmarks..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
            />
          </div>

          {/* Bookmarks list */}
          {Object.keys(groupedBookmarks).length === 0 ? (
            <div className="bookmark-content__empty">
              <p>No bookmarks found.</p>
              <button className="bookmark-content__btn bookmark-content__btn--primary" onClick={() => setView('add')}>
                Add Your First Bookmark
              </button>
            </div>
          ) : (
            <div className="bookmark-content__groups">
              {Object.entries(groupedBookmarks).map(([pageUrl, pageBookmarks]) => (
                <div key={pageUrl} className="bookmark-group">
                  <h4 className="bookmark-group__title">{pageBookmarks[0].pageTitle}</h4>
                  <div className="bookmark-group__items">
                    {pageBookmarks.map((bookmark) => (
                      <div key={bookmark.id} className="bookmark-item">
                        <div className="bookmark-item__header">
                          <a href={bookmark.headingId ? `${bookmark.pageUrl}#${bookmark.headingId}` : bookmark.pageUrl} className="bookmark-item__link">
                            {bookmark.headingText ? (
                              <>
                                <span className="bookmark-item__icon">#</span>
                                <span className="bookmark-item__heading">{bookmark.headingText}</span>
                              </>
                            ) : (
                              <>
                                <span className="bookmark-item__icon">📄</span>
                                <span className="bookmark-item__heading">Full Page</span>
                              </>
                            )}
                          </a>
                          <button
                            className="bookmark-item__delete"
                            onClick={() => deleteBookmark(bookmark.id)}
                            title="Delete bookmark"
                          >
                            ×
                          </button>
                        </div>
                        {editingNoteId === bookmark.id ? (
                          <div className="bookmark-item__note-edit">
                            <textarea
                              className="bookmark-content__textarea"
                              value={editNoteText}
                              onChange={(e) => setEditNoteText(e.target.value)}
                              rows={2}
                              autoFocus
                            />
                            <div className="bookmark-item__note-actions">
                              <button
                                className="bookmark-content__btn bookmark-content__btn--small bookmark-content__btn--primary"
                                onClick={() => handleSaveNote(bookmark.id)}
                              >
                                Save
                              </button>
                              <button
                                className="bookmark-content__btn bookmark-content__btn--small bookmark-content__btn--secondary"
                                onClick={handleCancelEdit}
                              >
                                Cancel
                              </button>
                            </div>
                          </div>
                        ) : (
                          <>
                            {bookmark.note && (
                              <p className="bookmark-item__note" onClick={() => handleStartEditNote(bookmark.id, bookmark.note)}>
                                {bookmark.note}
                              </p>
                            )}
                            {!bookmark.note && (
                              <button
                                className="bookmark-item__add-note"
                                onClick={() => handleStartEditNote(bookmark.id, '')}
                              >
                                + Add note
                              </button>
                            )}
                          </>
                        )}
                        <div className="bookmark-item__timestamp">
                          {new Date(bookmark.timestamp).toLocaleDateString('en-US', {
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
                </div>
              ))}
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default BookmarkContent;
