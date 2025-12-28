// API Client with CORS handling
import API_BASE_URL from '../config/apiConfig';

class ApiClient {
  private baseUrl: string;

  constructor() {
    this.baseUrl = API_BASE_URL;
  }

  // Check if we're in development mode to handle CORS differently
  private isDevelopment(): boolean {
    return typeof window !== 'undefined' &&
      window.location.hostname === 'localhost' ||
      window.location.hostname === '127.0.0.1';
  }

  // For development, we can use a proxy approach or handle differently
  private getApiUrl(endpoint: string): string {
    // In development, we might want to proxy through the dev server
    if (this.isDevelopment()) {
      // For Docusaurus development, we can't easily set up proxy like CRA
      // So we'll just use the original URL and rely on backend CORS
      return `${this.baseUrl}${endpoint}`;
    }
    // For production, use the configured base URL
    return `${this.baseUrl}${endpoint}`;
  }

  async request(endpoint: string, options: RequestInit = {}) {
    const url = this.getApiUrl(endpoint);

    const config: RequestInit = {
      ...options,
      headers: {
        'Content-Type': 'application/json',
        ...options.headers,
      },
    };

    try {
      const response = await fetch(url, config);
      return response;
    } catch (error) {
      console.error(`API request failed: ${url}`, error);
      throw error;
    }
  }

  async get(endpoint: string, headers: HeadersInit = {}) {
    return this.request(endpoint, { method: 'GET', headers });
  }

  async post(endpoint: string, data: any, headers: HeadersInit = {}) {
    return this.request(endpoint, {
      method: 'POST',
      headers,
      body: JSON.stringify(data),
    });
  }

  async postForm(endpoint: string, formData: FormData, headers: HeadersInit = {}) {
    // Don't set Content-Type for form data to let browser set it with boundary
    const { 'Content-Type': _, ...otherHeaders } = headers;
    return this.request(endpoint, {
      method: 'POST',
      headers: otherHeaders,
      body: formData,
    });
  }
}

export default new ApiClient();