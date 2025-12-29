import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import API_BASE_URL from '../config/apiConfig';

interface User {
  id: string;
  email: string;
  full_name: string;
}

interface AuthContextType {
  user: User | null;
  login: (email: string, password: string) => Promise<{ success: boolean; error?: string }>;
  register: (email: string, password: string, fullName: string) => Promise<{ success: boolean; error?: string }>;
  logout: () => void;
  isAuthenticated: boolean;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  useEffect(() => {
    // Check if user is already logged in on component mount
    const token = localStorage.getItem('token');
    if (token) {
      // Verify token and get user info
      fetchUserInfo(token);
    }
  }, []);

  const fetchUserInfo = async (token: string) => {
    try {
      console.log('Fetching user info from:', `${API_BASE_URL}/users/me`);
      const response = await fetch(`${API_BASE_URL}/users/me`, {
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      if (response.ok) {
        let userData;
        try {
          userData = await response.json();
        } catch (e) {
          // If response is not JSON, handle gracefully
          console.error('Failed to parse user data as JSON:', e);
          userData = { id: null, email: 'unknown', full_name: 'Unknown User' };
        }
        setUser(userData);
        setIsAuthenticated(true);
      } else {
        // Token might be invalid, clear it
        localStorage.removeItem('token');
        setUser(null);
        setIsAuthenticated(false);
      }
    } catch (error: any) {
      console.error('Error fetching user info:', error);
      console.error('Error details:', {
        message: error.message,
        name: error.name,
        stack: error.stack
      });
      // Check if it's a CORS error or network error
      if (error instanceof TypeError && (error.message.includes('fetch') || error.message.includes('CORS'))) {
        // Network error - keep token, just mark as not authenticated
        // This prevents clearing the token just because of CORS issues
        setUser(null);
        setIsAuthenticated(false);
      } else {
        // Other network error - keep token, just mark as not authenticated
        setUser(null);
        setIsAuthenticated(false);
      }
    }
  };

  const login = async (email: string, password: string) => {
    try {
      console.log('Attempting login to:', `${API_BASE_URL}/token`);
      const response = await fetch(`${API_BASE_URL}/token`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: new URLSearchParams({
          username: email,
          password: password,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        localStorage.setItem('token', data.access_token);

        // Fetch user info after successful login
        await fetchUserInfo(data.access_token);

        return { success: true };
      } else {
        // Try to parse error response, but handle if it's not JSON
        let errorData;
        try {
          errorData = await response.json();
        } catch (e) {
          // If response is not JSON, use status text
          errorData = { detail: `HTTP ${response.status}: ${response.statusText}` };
        }
        return { success: false, error: errorData.detail || 'Login failed' };
      }
    } catch (error: any) {
      console.error('Login error:', error);
      console.error('Error details:', {
        message: error.message,
        name: error.name,
        stack: error.stack
      });
      // Check if it's a CORS error or network error
      if (error instanceof TypeError && (error.message.includes('fetch') || error.message.includes('CORS'))) {
        return {
          success: false,
          error: 'Network error - please check your connection or contact the administrator. This might be a CORS configuration issue. Ensure the backend is properly configured to allow requests from ' + window.location.origin
        };
      }
      return { success: false, error: 'Network error: ' + error.message };
    }
  };

  const register = async (email: string, password: string, fullName: string) => {
    try {
      console.log('Attempting registration to:', `${API_BASE_URL}/register`);
      const response = await fetch(`${API_BASE_URL}/register`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password,
          full_name: fullName,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        localStorage.setItem('token', data.access_token);

        // Fetch user info after successful registration
        await fetchUserInfo(data.access_token);

        return { success: true };
      } else {
        // Try to parse error response, but handle if it's not JSON
        let errorData;
        try {
          errorData = await response.json();
        } catch (e) {
          // If response is not JSON, use status text
          errorData = { detail: `HTTP ${response.status}: ${response.statusText}` };
        }
        return { success: false, error: errorData.detail || 'Registration failed' };
      }
    } catch (error: any) {
      console.error('Registration error:', error);
      console.error('Error details:', {
        message: error.message,
        name: error.name,
        stack: error.stack
      });
      // Check if it's a CORS error or network error
      if (error instanceof TypeError && (error.message.includes('fetch') || error.message.includes('CORS'))) {
        return {
          success: false,
          error: 'Network error - please check your connection or contact the administrator. This might be a CORS configuration issue. Ensure the backend is properly configured to allow requests from ' + window.location.origin
        };
      }
      return { success: false, error: 'Network error: ' + error.message };
    }
  };

  const logout = () => {
    localStorage.removeItem('token');
    setUser(null);
    setIsAuthenticated(false);
  };

  return (
    <AuthContext.Provider value={{ user, login, register, logout, isAuthenticated }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};