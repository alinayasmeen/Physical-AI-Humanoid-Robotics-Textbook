import React from 'react';

interface TranslationComponentProps {
  children?: React.ReactNode;
}

const TranslationComponent: React.FC<TranslationComponentProps> = ({ children }) => {
  // Removed translation functionality - just render children directly
  return <>{children}</>;
};

export default TranslationComponent;