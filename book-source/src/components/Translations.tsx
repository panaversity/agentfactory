import React from 'react';
import Link from '@docusaurus/Link';
import Translate from '@docusaurus/Translate';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useLocation } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';

// Translation button component for individual language switching
const TranslationButton = ({ locale, label, currentLocale }) => {
  const location = useLocation();
  const isCurrentLocale = locale === currentLocale;

  // Construct the URL for the translated page
  const getTranslatedUrl = () => {
    // If we're trying to switch to the default locale
    if (locale === 'en') {
      // Remove any existing locale prefix from the current path
      const pathWithoutLocale = location.pathname.replace(/^\/(ur-PK|zh-CN)/, '');
      return pathWithoutLocale === '' ? '/' : pathWithoutLocale;
    }
    
    // If we're on the default locale path and want to switch to another locale
    if (!location.pathname.startsWith('/ur-PK') && !location.pathname.startsWith('/zh-CN')) {
      // Add the locale prefix to the current path
      const pathWithLocale = `/${locale}${location.pathname}`;
      return pathWithLocale === `/${locale}/` ? `/${locale}` : pathWithLocale;
    }
    
    // If we're switching between non-default locales
    if (currentLocale !== 'en') {
      // Replace the current locale prefix with the new one
      const pathWithoutCurrentLocale = location.pathname.replace(`/${currentLocale}`, '');
      return `/${locale}${pathWithoutCurrentLocale}`;
    }
    
    return location.pathname; // fallback
  };

  if (isCurrentLocale) {
    return (
      <button 
        className="translation-button translation-button-current"
        disabled
        style={{
          padding: '8px 16px',
          margin: '0 4px',
          backgroundColor: '#e9ecef',
          border: '1px solid #ced4da',
          borderRadius: '4px',
          cursor: 'not-allowed',
          opacity: 0.6
        }}
      >
        {label}
      </button>
    );
  }

  return (
    <Link 
      to={getTranslatedUrl()}
      style={{
        textDecoration: 'none'
      }}
    >
      <button 
        className="translation-button"
        style={{
          padding: '8px 16px',
          margin: '0 4px',
          backgroundColor: '#007bff',
          color: 'white',
          border: '1px solid #007bff',
          borderRadius: '4px',
          cursor: 'pointer'
        }}
        onMouseOver={(e) => {
          e.target.style.backgroundColor = '#0056b3';
          e.target.style.borderColor = '#0056b3';
        }}
        onMouseOut={(e) => {
          e.target.style.backgroundColor = '#007bff';
          e.target.style.borderColor = '#007bff';
        }}
      >
        {label}
      </button>
    </Link>
  );
};

// Main Translations component
const Translations = () => {
  const { i18n } = useDocusaurusContext();
  const currentLocale = i18n.currentLocale;

  // Define available locales with their labels
  const locales = [
    { locale: 'en', label: <Translate id="translation.english" description="English">English</Translate> },
    { locale: 'ur-PK', label: <Translate id="translation.urdu" description="Roman Urdu">Roman Urdu</Translate> },
    { locale: 'zh-CN', label: <Translate id="translation.chinese" description="Chinese">中文（简体）</Translate> },
  ];

  return (
    <div className="translations-container" style={{ 
      display: 'flex', 
      flexDirection: 'row', 
      flexWrap: 'wrap',
      alignItems: 'center',
      justifyContent: 'center',
      gap: '10px',
      padding: '15px 0',
      margin: '15px 0',
      borderTop: '1px solid #e0e0e0',
      borderBottom: '1px solid #e0e0e0'
    }}>
      <span style={{ fontWeight: 'bold', marginRight: '10px' }}>
        <Translate id="translation.selectLanguage" description="Select Language">Translate to:</Translate>
      </span>
      <div className="translation-buttons" style={{ display: 'flex', flexWrap: 'wrap', gap: '5px' }}>
        {locales.map(({ locale, label }) => (
          <TranslationButton
            key={locale}
            locale={locale}
            label={label}
            currentLocale={currentLocale}
          />
        ))}
      </div>
    </div>
  );
};

export default Translations;
