/**
 * LanguageToggle Component
 * 
 * Globe icon dropdown that allows users to switch between English and Urdu locales.
 * Integrates with Docusaurus i18n routing.
 */

import React, { useState, useRef, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { Globe } from 'lucide-react';
import clsx from 'clsx';
import styles from './styles.module.css';

export function LanguageToggle(): React.ReactElement {
  const {
    i18n: { currentLocale, locales, defaultLocale },
    siteConfig,
  } = useDocusaurusContext();
  const location = useLocation();
  const [isOpen, setIsOpen] = useState(false);
  const [isTransitioning, setIsTransitioning] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);
  
  // Get baseUrl from config (e.g., "/robolearn/" or "/")
  const baseUrl = siteConfig.baseUrl || '/';

  // Close dropdown when clicking outside or pressing Escape
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    };

    const handleEscape = (event: KeyboardEvent) => {
      if (event.key === 'Escape' && isOpen) {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    document.addEventListener('keydown', handleEscape);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
      document.removeEventListener('keydown', handleEscape);
    };
  }, [isOpen]);

  const switchLocale = async (targetLocale: string) => {
    if (targetLocale === currentLocale) {
      setIsOpen(false);
      return;
    }

    // Visual feedback: show transition state
    setIsTransitioning(true);
    setIsOpen(false);

    // Get current pathname
    let pathname = location.pathname;
    
    // Normalize baseUrl (remove trailing slash, handle root)
    const baseUrlPath = baseUrl === '/' ? '' : baseUrl.replace(/\/$/, '');
    
    // Remove baseUrl from pathname if present
    if (baseUrlPath && pathname.startsWith(baseUrlPath)) {
      pathname = pathname.slice(baseUrlPath.length);
    }
    
    // Ensure pathname starts with /
    if (!pathname.startsWith('/')) {
      pathname = '/' + pathname;
    }
    
    // Remove current locale prefix (e.g., "/en" or "/ur")
    // Handle patterns like "/en/docs/...", "/ur/docs/...", "/en/", "/ur/"
    const pathWithoutLocale = pathname.replace(/^\/(en|ur)(\/|$)/, '/') || '/';
    
    // Build new path: baseUrl + locale (if not default) + path
    let newPath = baseUrlPath || '';
    
    // Add locale prefix if not default locale
    if (targetLocale !== defaultLocale) {
      newPath += `/${targetLocale}`;
    }
    
    // Add the path (ensure it starts with /)
    const finalPath = pathWithoutLocale === '/' ? '' : pathWithoutLocale;
    newPath += finalPath;
    
    // Ensure path starts with / (for root baseUrl case)
    if (!newPath.startsWith('/')) {
      newPath = '/' + newPath;
    }

    // Small delay for smooth transition animation
    await new Promise(resolve => setTimeout(resolve, 150));

    // Navigate to new locale
    window.location.href = newPath;
  };

  const getLocaleLabel = (locale: string): string => {
    const labels: Record<string, string> = {
      en: 'EN',
      ur: 'UR',
    };
    return labels[locale] || locale;
  };

  const getLocaleFullName = (locale: string): string => {
    const names: Record<string, string> = {
      en: 'English',
      ur: 'Urdu',
    };
    return names[locale] || locale;
  };

  return (
    <div className={styles.languageToggle} ref={dropdownRef}>
      <button
        className={clsx('clean-btn', styles.toggleButton, {
          [styles.isOpen]: isOpen,
          [styles.isTransitioning]: isTransitioning,
        })}
        type="button"
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Select language"
        aria-expanded={isOpen}
        aria-haspopup="true"
        title={`Current language: ${getLocaleFullName(currentLocale)}`}
      >
        <Globe className={styles.globeIcon} aria-hidden />
        {isTransitioning && (
          <span className={styles.loadingIndicator} aria-hidden />
        )}
      </button>
      
      {isOpen && (
        <div className={styles.dropdown} role="menu">
          {locales.map((locale, index) => (
            <button
              key={locale}
              className={clsx(styles.dropdownItem, {
                [styles.active]: locale === currentLocale,
              })}
              onClick={() => switchLocale(locale)}
              aria-label={`Switch to ${getLocaleFullName(locale)}`}
              role="menuitem"
              style={{ animationDelay: `${index * 30}ms` }}
            >
              <span className={styles.localeCode}>{getLocaleLabel(locale)}</span>
              <span className={styles.localeName}>{getLocaleFullName(locale)}</span>
              {locale === currentLocale && (
                <span className={styles.checkmark} aria-hidden>✓</span>
              )}
            </button>
          ))}
        </div>
      )}
    </div>
  );
}

