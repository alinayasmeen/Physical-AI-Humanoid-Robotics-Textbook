// API Configuration
// Check if we're running in development mode (localhost)
const isDevelopment = typeof window !== 'undefined' &&
  (window.location.hostname === 'localhost' ||
   window.location.hostname === '127.0.0.1');

// Use different API URLs based on environment to handle CORS
const API_BASE_URL = isDevelopment
  ? 'https://physical-ai-humanoid-robotics-textbook-fcve.onrender.com'  // Same as production but may have CORS issues
  : 'https://physical-ai-humanoid-robotics-textbook-fcve.onrender.com'; // Production URL

export default API_BASE_URL;