import React, { useState } from 'react';
import LoginForm from './LoginForm';
import RegisterForm from './RegisterForm';
import styles from './AuthWrapper.module.css';

interface AuthWrapperProps {
  onAuthSuccess: () => void;
}

const AuthWrapper: React.FC<AuthWrapperProps> = ({ onAuthSuccess }) => {
  const [isLoginView, setIsLoginView] = useState(true);

  const handleAuthSuccess = () => {
    onAuthSuccess();
  };

  return (
    <div className={styles.authWrapper}>
      <div className={styles.authCard}>
        {isLoginView ? (
          <LoginForm
            onSwitchToRegister={() => setIsLoginView(false)}
          />
        ) : (
          <RegisterForm
            onSwitchToLogin={() => setIsLoginView(true)}
          />
        )}
      </div>
    </div>
  );
};

export default AuthWrapper;