import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';

export default function Root({ children }) {
  return <AuthProvider>{children}</AuthProvider>;
}