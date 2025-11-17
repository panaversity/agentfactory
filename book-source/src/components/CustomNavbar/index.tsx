import React, { useState, useCallback, useEffect, useRef } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useColorMode } from '@docusaurus/theme-common';
import { useSearch } from '@/contexts/SearchContext';
import type { SearchResult } from '@/contexts/SearchContext';
import { useSidebarControl } from '@/contexts/SidebarContext';
import RightDrawer from './RightDrawer';
import SelectionToolbar from './SelectionToolbar';
import Assistant from '../PanaChat';
import './customNavbar.css';

interface NavButtonProps {
  label: string;
  onClick?: () => void;
  isActive?: boolean;
}

const NavButton: React.FC<NavButtonProps> = ({ label, onClick, isActive }) => {
  return (
    <button
      onClick={onClick}
      className={`custom-navbar__nav-button ${isActive ? 'custom-navbar__nav-button--active' : ''}`}
    >
      {label}
    </button>
  );
};

const CustomNavbar: React.FC = () => {
  const { siteConfig } = useDocusaurusContext();
  const { colorMode, setColorMode } = useColorMode();
  const { search, isLoading: searchLoading } = useSearch();
  const { isSidebarCollapsed, collapseSidebar, expandSidebar } = useSidebarControl();
  const [searchFocused, setSearchFocused] = useState(false);
  const [searchQuery, setSearchQuery] = useState('');
  const [selectedIndex, setSelectedIndex] = useState(0);
  const [drawerOpen, setDrawerOpen] = useState(false);
  const [drawerTitle, setDrawerTitle] = useState('');
  const [chatOpen, setChatOpen] = useState(false);
  const searchInputRef = useRef<HTMLInputElement>(null);

  const toggleColorMode = useCallback(() => {
    setColorMode(colorMode === 'dark' ? 'light' : 'dark');
  }, [colorMode, setColorMode]);

  const openDrawer = useCallback((title: string) => {
    setDrawerTitle(title);
    setDrawerOpen(true);
    // Collapse left sidebar when opening right drawer for maximum space
    console.log('🔴 Dispatching collapseSidebar event');
    window.dispatchEvent(new CustomEvent('collapseSidebar'));
  }, []);

  const closeDrawer = useCallback(() => {
    setDrawerOpen(false);
  }, []);

  const handleSelectionAction = useCallback((action: string, selectedText: string) => {
    // Open the drawer with the selected action
    openDrawer(action);

    // You can also store the selected text for use in the drawer
    console.log(`Action: ${action}, Selected Text: ${selectedText}`);
  }, [openDrawer]);

  const toggleChat = useCallback(() => {
    setChatOpen((prev) => !prev);
  }, []);

  const closeChat = useCallback(() => {
    setChatOpen(false);
  }, []);

  const toggleSidebarCollapse = useCallback(() => {
    if (isSidebarCollapsed) {
      expandSidebar();
    } else {
      collapseSidebar();
    }
  }, [isSidebarCollapsed, collapseSidebar, expandSidebar]);


  // Use the real search function from context
  const filteredResults = searchQuery ? search(searchQuery) : [];

  const handleSearchFocus = useCallback(() => {
    setSearchFocused(true);
    setSelectedIndex(0);
    // Don't auto-focus the modal input here, let it handle its own focus
  }, []);

  const handleSearchBlur = useCallback(() => {
    // Blur is now handled by modal close
  }, []);

  const handleModalClose = useCallback(() => {
    setSearchFocused(false);
    setSearchQuery('');
    setSelectedIndex(0);
  }, []);

  const handleSearchChange = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    setSearchQuery(e.target.value);
    setSelectedIndex(0);
  }, []);

  const handleNavbarSearchClick = useCallback(() => {
    setSearchFocused(true);
  }, []);

  // Handle keyboard shortcut for search (Ctrl+K or Cmd+K) and ESC
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Ctrl+K or Cmd+K to open search modal
      if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
        e.preventDefault();
        setSearchFocused(true);
      }

      // ESC to close search modal
      if (e.key === 'Escape' && searchFocused) {
        e.preventDefault();
        handleModalClose();
      }

      // Arrow navigation
      if (searchFocused && filteredResults.length > 0) {
        if (e.key === 'ArrowDown') {
          e.preventDefault();
          setSelectedIndex((prev) => (prev < filteredResults.length - 1 ? prev + 1 : prev));
        } else if (e.key === 'ArrowUp') {
          e.preventDefault();
          setSelectedIndex((prev) => (prev > 0 ? prev - 1 : 0));
        } else if (e.key === 'Enter') {
          e.preventDefault();
          if (filteredResults[selectedIndex]) {
            window.location.href = filteredResults[selectedIndex].url;
          }
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [searchFocused, filteredResults, selectedIndex, handleModalClose]);

  return (
    <>
      {/* Search Modal Overlay - appears when search is focused */}
      {searchFocused && (
        <>
          {/* Blur overlay for content below navbar */}
          <div className="custom-navbar__search-overlay" onClick={handleModalClose} />

          {/* Centered Search Modal */}
          <div className="custom-navbar__search-modal">
            <div className="custom-navbar__search-modal-container">
              {/* New centered search bar */}
              <div className="custom-navbar__search-modal-input-wrapper">
                <svg
                  className="custom-navbar__search-modal-icon"
                  width="20"
                  height="20"
                  viewBox="0 0 16 16"
                  fill="none"
                >
                  <path
                    d="M7 12C9.76142 12 12 9.76142 12 7C12 4.23858 9.76142 2 7 2C4.23858 2 2 4.23858 2 7C2 9.76142 4.23858 12 7 12Z"
                    stroke="currentColor"
                    strokeWidth="1.5"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  />
                  <path
                    d="M10.5 10.5L14 14"
                    stroke="currentColor"
                    strokeWidth="1.5"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  />
                </svg>
                <input
                  ref={searchInputRef}
                  type="text"
                  className="custom-navbar__search-modal-input"
                  placeholder="Search..."
                  value={searchQuery}
                  onChange={handleSearchChange}
                  autoFocus
                />
                <kbd className="custom-navbar__search-modal-kbd">ESC</kbd>
              </div>

              {/* Search Results Dropdown in Modal */}
              {searchQuery && filteredResults.length > 0 && (
                <div className="custom-navbar__search-modal-dropdown">
                  {filteredResults.map((result, index) => {
                    console.log("Working")
                    return (
                      <Link
                        key={index}
                        to={result.url}
                        onClick={handleModalClose}
                        className={`custom-navbar__search-result ${index === selectedIndex ? 'custom-navbar__search-result--selected' : ''
                          }`}
                      >
                        <div className="custom-navbar__search-result-header">
                          <span className="custom-navbar__search-result-icon">
                            {result.type === 'page' ? (
                              // Page icon - document/file icon
                              <svg width="14" height="14" viewBox="0 0 16 16" fill="none">
                                <path
                                  d="M9 1H3C2.44772 1 2 1.44772 2 2V14C2 14.5523 2.44772 15 3 15H13C13.5523 15 14 14.5523 14 14V6L9 1Z"
                                  stroke="currentColor"
                                  strokeWidth="1.5"
                                  strokeLinecap="round"
                                  strokeLinejoin="round"
                                />
                                <path
                                  d="M9 1V6H14"
                                  stroke="currentColor"
                                  strokeWidth="1.5"
                                  strokeLinecap="round"
                                  strokeLinejoin="round"
                                />
                              </svg>
                            ) : (
                              // Heading icon - hash symbol
                              <span style={{ fontSize: '12px', fontWeight: 600 }}>#</span>
                            )}
                          </span>
                          <span className="custom-navbar__search-result-title">{result.title}</span>
                          <svg
                            className="custom-navbar__search-result-arrow"
                            width="16"
                            height="16"
                            viewBox="0 0 16 16"
                            fill="none"
                          >
                            <path
                              d="M6 12L10 8L6 4"
                              stroke="currentColor"
                              strokeWidth="1.5"
                              strokeLinecap="round"
                              strokeLinejoin="round"
                            />
                          </svg>
                        </div>
                        <div className="custom-navbar__search-result-breadcrumb">{result.breadcrumb}</div>
                        <div className="custom-navbar__search-result-description">{result.description}</div>
                      </Link>
                    )
                  })}
                </div>
              )}
            </div>
          </div>
        </>
      )}

      <nav className="navbar custom-navbar navbar--fixed-top">
        <div className="custom-navbar__container">
          {/* Left Section - Logo and Title */}
          <div className="custom-navbar__left">
            <Link to="/" className="custom-navbar__brand">
              <svg
                className="custom-navbar__logo"
                viewBox="0 0 24 24"
                fill="none"
                xmlns="http://www.w3.org/2000/svg"
              >
                <path
                  d="M3 3L8 3L8 21L3 21L3 3Z"
                  fill="currentColor"
                />
                <path
                  d="M11 3L16 3L16 21L11 21L11 3Z"
                  fill="currentColor"
                  fillOpacity="0.7"
                />
                <path
                  d="M19 3L21 3L21 21L19 21L19 3Z"
                  fill="currentColor"
                  fillOpacity="0.5"
                />
              </svg>
              <span className="custom-navbar__title">AI Native Book</span>
            </Link>
          </div>

          {/* Center Section - Search Bar (click to open modal) */}
          <div className="custom-navbar__center">
            <div
              className="custom-navbar__search"
              onClick={handleNavbarSearchClick}
            >
              <svg
                className="custom-navbar__search-icon"
                width="16"
                height="16"
                viewBox="0 0 16 16"
                fill="none"
              >
                <path
                  d="M7 12C9.76142 12 12 9.76142 12 7C12 4.23858 9.76142 2 7 2C4.23858 2 2 4.23858 2 7C2 9.76142 4.23858 12 7 12Z"
                  stroke="currentColor"
                  strokeWidth="1.5"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
                <path
                  d="M10.5 10.5L14 14"
                  stroke="currentColor"
                  strokeWidth="1.5"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
              </svg>
              <span className="custom-navbar__search-placeholder">Search...</span>
              <kbd className="custom-navbar__search-kbd">Ctrl K</kbd>
            </div>
          </div>

          {/* Right Section - Chat, GitHub, Theme Toggle */}
          <div className="custom-navbar__right">
            <button onClick={toggleChat} className="custom-navbar__link custom-navbar__link--button">
              Chat
            </button>
            <a
              href="https://github.com/panaversity/ai-native-software-development"
              className="custom-navbar__link"
              target="_blank"
              rel="noopener noreferrer"
            >
              GitHub
            </a>
            <button
              className="custom-navbar__theme-toggle"
              onClick={toggleColorMode}
              aria-label="Toggle theme"
              title={`Switch to ${colorMode === 'dark' ? 'light' : 'dark'} mode`}
            >
              {colorMode === 'dark' ? (
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                  <path
                    d="M8 1V2M8 14V15M15 8H14M2 8H1M12.95 12.95L12.24 12.24M3.76 3.76L3.05 3.05M12.95 3.05L12.24 3.76M3.76 12.24L3.05 12.95M11 8C11 9.65685 9.65685 11 8 11C6.34315 11 5 9.65685 5 8C5 6.34315 6.34315 5 8 5C9.65685 5 11 6.34315 11 8Z"
                    stroke="currentColor"
                    strokeWidth="1.5"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  />
                </svg>
              ) : (
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                  <path
                    d="M14 8.5C13.8 10.7 12 12.5 9.8 12.9C7.6 13.3 5.4 12.4 4.3 10.6C3.2 8.8 3.4 6.5 4.8 4.9C6.2 3.3 8.4 2.8 10.4 3.5C9.2 5.3 9.5 7.7 11.1 9.1C12.7 10.5 15.1 10.5 16.7 9.1C15.9 11.3 14 13.1 11.8 13.5"
                    stroke="currentColor"
                    strokeWidth="1.5"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  />
                </svg>
              )}
            </button>
          </div>
        </div>

        {/* Secondary Navigation Row - Tab-like buttons */}
        <div className="custom-navbar__secondary">
          <button
            className="custom-navbar__sidebar-toggle"
            onClick={toggleSidebarCollapse}
            aria-label={isSidebarCollapsed ? 'Expand sidebar' : 'Collapse sidebar'}
            title={isSidebarCollapsed ? 'Expand sidebar' : 'Collapse sidebar'}
          >
            {isSidebarCollapsed ? (
              // Expand icon - panel-right-open
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <rect x="3" y="3" width="18" height="18" rx="2"/>
                <path d="M15 3v18"/>
                <path d="M8 9l3 3-3 3"/>
              </svg>
            ) : (
              // Collapse icon - panel-left-close
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <rect x="3" y="3" width="18" height="18" rx="2"/>
                <path d="M9 3v18"/>
                <path d="M14 9l-3 3 3 3"/>
              </svg>
            )}
          </button>
          <div className="custom-navbar__nav-buttons">
            <NavButton label="Bookmark" onClick={() => openDrawer('Bookmark')} />
            <NavButton label="Mindmap" onClick={() => openDrawer('Mindmap')} />
            <NavButton label="Notes" onClick={() => openDrawer('Notes')} />
            <NavButton label="Assessment" onClick={() => openDrawer('Assessment')} />
          </div>
        </div>
      </nav>

      {/* Right Drawer */}
      <RightDrawer isOpen={drawerOpen} onClose={closeDrawer} title={drawerTitle} />

      {/* Selection Toolbar - appears above selected text */}
      <SelectionToolbar onAction={handleSelectionAction} />

      {/* Assistant - bottom right chat widget */}
      <Assistant isOpen={chatOpen} onClose={closeChat} />
    </>
  );
};

export default CustomNavbar;
