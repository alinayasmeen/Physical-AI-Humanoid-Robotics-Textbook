import React from 'react';
import TranslateToUrdu from '.';

// This component wraps the page content with translation functionality
const PageWithTranslation: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  return <TranslateToUrdu>{children}</TranslateToUrdu>;
};

export default PageWithTranslation;