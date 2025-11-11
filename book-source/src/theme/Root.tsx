/**
 * Docusaurus Root Component
 * * This component wraps the entire site with the AnalyticsTracker and the Google Translate script loader,
 * ensuring the translation functionality is available globally.
 */

import React from 'react';
import { AnalyticsTracker } from '@/components/AnalyticsTracker';
import LanguageSelector from '@/components/LanguageSelector';
// Import Google Translate component
import GoogleTranslate from '../components/GoogleTranslate'; 

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <AnalyticsTracker>
      {children}
      <LanguageSelector position="floating" />
      {/*  Render Google Translate globally */}
      <GoogleTranslate />
    </AnalyticsTracker>
  );
}