import React, { useEffect } from 'react';
import Head from '@docusaurus/Head'; 

function GoogleTranslate() {

  // Initialize Google Translate when the component loads
  useEffect(() => {
    // Avoid loading the script multiple times
    if (document.getElementById('google-translate-script-element')) {
        return;
    }

    // Define the Google Translate initialization function
    window.googleTranslateElementInit = function() {
      new window.google.translate.TranslateElement(
        { 
          pageLanguage: 'en', // Default language
          layout: window.google.translate.TranslateElement.InlineLayout.SIMPLE, // Simple dropdown layout
          autoDisplay: false 
        }, 
        'google_translate_element_id' // ID of the div where the widget will be rendered
      );
    };
  }, []);

  return (
    <>
      {/* Load Google Translate script */}
      <Head>
        <script 
          type="text/javascript" 
          src="//translate.google.com/translate_a/element.js?cb=googleTranslateElementInit"
          key="google-translate-script-element" 
        />
      </Head>
      
      {/* Hidden div - required for initialization */}
      <div style={{ display: 'none' }} />
    </>
  );
}

export default GoogleTranslate;
