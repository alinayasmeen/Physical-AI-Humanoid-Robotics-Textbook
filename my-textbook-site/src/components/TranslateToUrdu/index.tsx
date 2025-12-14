import React from 'react';
import TranslationButton from '../../chatbot/TranslationButton';
import styles from '../../chatbot/styles.module.css';

interface TranslateToUrduProps {
  children?: React.ReactNode;
  containerSelector?: string;
}

const TranslateToUrdu: React.FC<TranslateToUrduProps> = ({
  children,
  containerSelector = 'main .container .row .col'
}) => {
  return (
    <div>
      <div className={styles.translationButtonContainer}>
        <TranslationButton containerSelector={containerSelector} />
      </div>
      {children}
    </div>
  );
};

export default TranslateToUrdu;