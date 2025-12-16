import React from 'react';
import { useNavbarMobileSidebar, useNavbarSecondaryMenu } from '@docusaurus/theme-common/internal';
import { useColorMode } from '@docusaurus/theme-common';
import NavbarLogo from '@theme/Navbar/Logo';
import styles from './styles.module.css';

// Back arrow icon
function BackIcon() {
  return (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
      <polyline points="15 18 9 12 15 6" />
    </svg>
  );
}

// Sun icon for light mode
function SunIcon() {
  return (
    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
      <circle cx="12" cy="12" r="5" />
      <line x1="12" y1="1" x2="12" y2="3" />
      <line x1="12" y1="21" x2="12" y2="23" />
      <line x1="4.22" y1="4.22" x2="5.64" y2="5.64" />
      <line x1="18.36" y1="18.36" x2="19.78" y2="19.78" />
      <line x1="1" y1="12" x2="3" y2="12" />
      <line x1="21" y1="12" x2="23" y2="12" />
      <line x1="4.22" y1="19.78" x2="5.64" y2="18.36" />
      <line x1="18.36" y1="5.64" x2="19.78" y2="4.22" />
    </svg>
  );
}

// Moon icon for dark mode
function MoonIcon() {
  return (
    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
      <path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z" />
    </svg>
  );
}

// Close icon
function CloseIcon() {
  return (
    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
      <line x1="18" y1="6" x2="6" y2="18" />
      <line x1="6" y1="6" x2="18" y2="18" />
    </svg>
  );
}

// Back button - only shown when in secondary menu (docs sidebar)
function BackButton() {
  const secondaryMenu = useNavbarSecondaryMenu();

  if (!secondaryMenu.shown) {
    return null;
  }

  return (
    <button
      className={styles.backButton}
      onClick={() => secondaryMenu.hide()}
      title="Back"
      aria-label="Back to main menu"
    >
      <BackIcon />
      <span>Back</span>
    </button>
  );
}

function ThemeToggleButton() {
  const { colorMode, setColorMode } = useColorMode();

  const toggleTheme = () => {
    setColorMode(colorMode === 'dark' ? 'light' : 'dark');
  };

  return (
    <button
      className={styles.themeToggle}
      onClick={toggleTheme}
      title={colorMode === 'dark' ? 'Switch to light mode' : 'Switch to dark mode'}
      aria-label={colorMode === 'dark' ? 'Switch to light mode' : 'Switch to dark mode'}
    >
      {colorMode === 'dark' ? <SunIcon /> : <MoonIcon />}
    </button>
  );
}

function CloseButton() {
  const mobileSidebar = useNavbarMobileSidebar();
  return (
    <button
      className={styles.closeButton}
      onClick={() => mobileSidebar.toggle()}
      title="Close menu"
      aria-label="Close menu"
    >
      <CloseIcon />
    </button>
  );
}

export default function MobileSidebarHeader(): React.ReactElement {
  return (
    <div className={styles.header}>
      <NavbarLogo />
      <div className={styles.actions}>
        <BackButton />
        <ThemeToggleButton />
        <CloseButton />
      </div>
    </div>
  );
}
