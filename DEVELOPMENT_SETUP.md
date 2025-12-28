# Development Setup Guide

This guide explains how to set up the development environment for both the frontend and backend to avoid CORS issues during development.

## Backend Setup (Local Development)

### Prerequisites
- Python 3.8 or higher
- pip package manager
- Git

### Steps

1. **Clone the repository** (if not already done):
   ```bash
   git clone <repository-url>
   cd Physical-AI-Humanoid-Robotics-Textbook
   ```

2. **Navigate to the backend directory**:
   ```bash
   cd rag-chatbot-backend
   ```

3. **Create a virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

4. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   # Or if requirements.txt doesn't exist, install the necessary packages:
   pip install fastapi uvicorn python-multipart python-jose[cryptography] passlib[bcrypt] psycopg[pool] python-dotenv qdrant-client fastembed google-generativeai
   ```

5. **Create a .env file** with your configuration:
   ```env
   SECRET_KEY=your-secret-key-here
   QDRANT_URL=your-qdrant-url
   QDRANT_API_KEY=your-qdrant-api-key
   GEMINI_API_KEY=your-gemini-api-key
   NEON_DATABASE_URL=your-neon-db-url
   FRONTEND_URL=http://localhost:3000
   RENDER_EXTERNAL_URL=https://physical-ai-humanoid-robotics-textbook-fcve.onrender.com
   ```

6. **Run the backend server**:
   ```bash
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

## Frontend Setup

### Prerequisites
- Node.js 14 or higher
- npm or yarn package manager

### Steps

1. **Navigate to the frontend directory**:
   ```bash
   cd my-textbook-site
   ```

2. **Install dependencies**:
   ```bash
   npm install
   ```

3. **Run the development server**:
   ```bash
   npm start
   ```

## Environment Variables for Frontend Development

To override the API base URL during development, you can create a `.env` file in the `my-textbook-site` directory:

```env
REACT_APP_API_BASE_URL=http://localhost:8000
```

This will make the frontend use your local backend instead of the remote one, avoiding CORS issues.

## Running with Local Backend

1. Start the backend server first:
   ```bash
   cd rag-chatbot-backend
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

2. In a new terminal, start the frontend:
   ```bash
   cd my-textbook-site
   npm start
   ```

3. The frontend will be available at `http://localhost:3000` and will communicate with the backend at `http://localhost:8000`.

## Troubleshooting CORS Issues

If you still encounter CORS issues:

1. **Check that your backend is running** and accessible at the configured URL
2. **Verify the CORS configuration** in `rag-chatbot-backend/main.py`
3. **Ensure your environment variables** are properly set
4. **Check browser console** for specific error messages

## Production Deployment

For production, the frontend and backend are configured to work together with proper CORS headers. The production backend is deployed on Render and the frontend can be deployed to any static hosting service (like Vercel, Netlify, or GitHub Pages).