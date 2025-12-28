// API Configuration
// Check if we're running in development mode (localhost)
const isDevelopment = typeof window !== 'undefined' &&
  (window.location.hostname === 'localhost' ||
   window.location.hostname === '127.0.0.1');

// Use different API URLs based on environment to handle CORS
// For development, you can run the backend locally on port 8000, or use the remote server
// To run backend locally: uvicorn main:app --reload --host 0.0.0.0 --port 8000
const API_BASE_URL = isDevelopment
  ? process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000'  // Local backend for development (can be overridden)
  : 'https://physical-ai-humanoid-robotics-textbook-fcve.onrender.com'; // Production URL

export default API_BASE_URL;