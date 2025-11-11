import React, { useState } from 'react';
import Link from '@docusaurus/Link';
import Translate from '@docusaurus/Translate';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useLocation } from '@docusaurus/router';

const LanguageSelector = ({ position = 'inline' }) => {
  const { i18n } = useDocusaurusContext();
  const location = useLocation();
  const currentLocale = i18n.currentLocale;

  // Define available locales with their labels and flags
  const locales = [
    { 
      locale: 'en', 
      label: <Translate id="translation.english" description="English">English</Translate>,
      flag: '🇺🇸',
      short: 'EN'
    },
    { 
      locale: 'ur-PK', 
      label: <Translate id="translation.urdu" description="Roman Urdu">Roman Urdu</Translate>, 
      flag: '🇵🇰',
      short: 'UR'
    },
    { 
      locale: 'zh-CN', 
      label: <Translate id="translation.chinese" description="Chinese">中文（简体）</Translate>,
      flag: '🇨🇳',
      short: 'ZH'
    },
  ];

  // Function to get the translated URL
  const getTranslatedUrl = (targetLocale) => {
    if (targetLocale === 'en') {
      // For default locale, remove any existing locale prefix from URL
      const pathWithoutLocale = location.pathname.replace(/^\/(ur-PK|zh-CN)/, '');
      return pathWithoutLocale === '' ? '/' : pathWithoutLocale;
    }
    
    // If we're on the default locale path and want to switch to another locale
    if (!location.pathname.startsWith('/ur-PK') && !location.pathname.startsWith('/zh-CN')) {
      // Add the target locale prefix to the current path
      const pathWithLocale = `/${targetLocale}${location.pathname}`;
      return pathWithLocale === `/${targetLocale}/` ? `/${targetLocale}` : pathWithLocale;
    }
    
    // If we're switching between non-default locales
    if (currentLocale !== 'en') {
      // Replace the current locale prefix with the target locale
      const pathWithoutCurrentLocale = location.pathname.replace(`/${currentLocale}`, '');
      return `/${targetLocale}${pathWithoutCurrentLocale}`;
    }
    
    // Fallback
    return `/${targetLocale}${location.pathname}`;
  };

  const currentLocaleInfo = locales.find(loc => loc.locale === currentLocale);

  // For floating/overlay positioning
  if (position === 'floating') {
    const [isOpen, setIsOpen] = useState(false);
    
    return (
      <div className={`language-selector-floating ${isOpen ? 'open' : ''}`}>
        <button
          className="language-selector-toggle"
          onClick={() => setIsOpen(!isOpen)}
          title={Translate({
            id: 'translation.toggleLanguage',
            message: 'Select Language',
            description: 'Toggle language selector button title'
          })}
          style={{
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            width: '50px',
            height: '50px',
            borderRadius: '50%',
            backgroundColor: '#007bff',
            color: 'white',
            border: 'none',
            fontSize: '24px',
            cursor: 'pointer',
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            zIndex: 1000,
            boxShadow: '0 4px 8px rgba(0,0,0,0.2)',
          }}
        >
          {currentLocaleInfo?.flag || '🌐'}
        </button>
        
        {isOpen && (
          <div 
            className="language-selector-dropdown"
            style={{
              position: 'fixed',
              bottom: '80px',
              right: '20px',
              backgroundColor: 'white',
              borderRadius: '8px',
              padding: '10px',
              boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
              zIndex: 1001,
              minWidth: '200px',
            }}
          >
            <div style={{ fontWeight: 'bold', marginBottom: '10px', textAlign: 'center' }}>
              <Translate id="translation.selectLanguage" description="Select Language">Language</Translate>:
            </div>
            {locales.map(({ locale, label, flag }) => {
              const isCurrent = locale === currentLocale;
              
              if (isCurrent) {
                return (
                  <div 
                    key={locale}
                    className="current-language"
                    style={{
                      display: 'flex',
                      alignItems: 'center',
                      padding: '8px 12px',
                      backgroundColor: '#f8f9fa',
                      color: '#6c757d',
                      borderRadius: '4px',
                      fontStyle: 'italic',
                    }}
                  >
                    <span style={{ marginRight: '8px' }}>{flag}</span>
                    {label}
                  </div>
                );
              }
              
              return (
                <Link
                  key={locale}
                  to={getTranslatedUrl(locale)}
                  style={{
                    display: 'flex',
                    alignItems: 'center',
                    padding: '8px 12px',
                    textDecoration: 'none',
                    color: 'inherit',
                    borderRadius: '4px',
                  }}
                  onMouseEnter={(e) => {
                    e.currentTarget.style.backgroundColor = '#e9ecef';
                  }}
                  onMouseLeave={(e) => {
                    e.currentTarget.style.backgroundColor = 'transparent';
                  }}
                >
                  <span style={{ marginRight: '8px' }}>{flag}</span>
                  <span>{label}</span>
                </Link>
              );
            })}
          </div>
        )}
      </div>
    );
  }

  // For inline positioning (default)
  return (
    <div className={`language-selector language-selector-${position}`}>
      <div 
        className="language-selector-header"
        style={{ 
          display: 'flex', 
          alignItems: 'center',
          justifyContent: 'center',
          marginBottom: '10px',
          padding: '8px 0',
        }}
      >
        <span className="language-selector-label" style={{ fontWeight: 'bold', marginRight: '10px' }}>
          <Translate id="translation.selectLanguage" description="Select Language">Translate to:</Translate>
        </span>
      </div>
      
      <div 
        className="language-selector-buttons" 
        style={{ 
          display: 'flex', 
          flexWrap: 'wrap', 
          justifyContent: 'center', 
          gap: '8px',
          alignItems: 'center',
        }}
      >
        {locales.map(({ locale, label, flag, short }) => {
          const isCurrent = locale === currentLocale;

          if (isCurrent) {
            return (
              <button
                key={locale}
                className="language-button language-button-current"
                disabled
                style={{
                  padding: '8px 16px',
                  backgroundColor: '#e9ecef',
                  border: '1px solid #ced4da',
                  borderRadius: '20px',
                  cursor: 'not-allowed',
                  opacity: 0.7,
                  display: 'flex',
                  alignItems: 'center',
                  gap: '5px',
                }}
              >
                <span>{flag}</span>
                <span>{short}</span>
              </button>
            );
          }

          return (
            <Link
              key={locale}
              to={getTranslatedUrl(locale)}
              style={{
                textDecoration: 'none',
              }}
            >
              <button
                className="language-button"
                style={{
                  padding: '8px 16px',
                  backgroundColor: '#007bff',
                  color: 'white',
                  border: '1px solid #007bff',
                  borderRadius: '20px',
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  gap: '5px',
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
                <span>{flag}</span>
                <span>{short}</span>
              </button>
            </Link>
          );
        })}
      </div>
    </div>
  );
};

export default LanguageSelector;