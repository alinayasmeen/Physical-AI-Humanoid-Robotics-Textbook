import React, { useState, useEffect } from 'react';
import TranslationButton from '../../chatbot/TranslationButton';

interface TranslationComponentProps {
  children?: React.ReactNode;
}

const TranslationComponent: React.FC<TranslationComponentProps> = ({ children }) => {
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  const handleTranslationComplete = (content: string) => {
    setTranslatedContent(content);
  };

  if (!isClient) {
    // Don't render on the server to avoid hydration issues
    return <div>{children}</div>;
  }

  return (
    <div>
      <div className="translationButtonContainer">
        <TranslationButton onTranslationComplete={handleTranslationComplete} />
      </div>
      {translatedContent ? (
        <div dangerouslySetInnerHTML={{ __html: translatedContent }} />
      ) : (
        children
      )}
    </div>
  );
};

export default TranslationComponent;