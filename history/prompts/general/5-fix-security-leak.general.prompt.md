---
id: 5
title: Fix Security Leak in Render Configuration
stage: general
date: 2025-12-25
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: 1-rag-chatbot
user: user
command: resolve security leak
labels: [security,api-key,leak,exposed]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - rag-chatbot-backend/render.yaml
tests:
 -
---

## Prompt

Google API Key
AIzaSyCTmQfzOawf4tSTHmiiFGmdpqcaGNF7Gvo
Public leak
#1 opened14 days ago
• Detected secret in rag-chatbot-backend/render.yaml:1

## Response snapshot

Security vulnerability identified and fixed. The render.yaml file contained multiple exposed secrets including:
1. Google API key: AIzaSyCTmQfzOawf4tSTHmiiFGmdpqcaGNF7Gvo
2. Qdrant URL with credentials
3. JWT token
4. PostgreSQL connection string with credentials

These were hardcoded directly in the render.yaml file, making them publicly accessible. I've updated the file to use environment variable references instead of hardcoded values:
- Changed QDRANT_URL value to reference environment variable
- Changed SECRET_KEY value to reference environment variable
- Changed GEMINI_API_KEY value to reference environment variable (was the Google API key)
- Changed NEON_DATABASE_URL value to reference environment variable

The application code already uses os.getenv() to retrieve these values, so this change maintains functionality while securing the sensitive information.

## Outcome

- ✅ Impact: Fixed security vulnerability with exposed API keys and credentials
- 🧪 Tests: n/a
- 📁 Files: Updated rag-chatbot-backend/render.yaml
- 🔁 Next prompts: User needs to set proper environment variables in deployment
- 🧠 Reflection: Hardcoded secrets pose serious security risks

## Evaluation notes (flywheel)

- Failure modes observed: Exposed API keys and database credentials in public repository
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): Deploy with secure environment variables