
● Physical AI & Humanoid Robotics Textbook

  LICENSE
  https://www.python.org/
  https://docusaurus.io/
  https://github.com/alinayasmeen/Physical-AI-Humanoid-Robotics-Textbook

  An in-depth guide to the future of AI and robotics, focusing on Physical AI and Humanoid Robotics. This comprehensive textbook combines theoretical foundations with practical applications, providing students and researchers with the knowledge needed to develop intelligent humanoid robots.

  🚀 Features

  - Comprehensive Curriculum: Covers all aspects of humanoid robotics from basic principles to advanced topics
  - Interactive Learning: Includes hands-on labs, projects, and practical exercises
  - Multilingual Support: Available in English and Urdu for broader accessibility
  - Modern Technologies: Focuses on cutting-edge technologies like NVIDIA Isaac, ROS 2, and AI models
  - Real-world Applications: Emphasizes practical implementation and real-world deployment

  📚 Table of Contents

  1. ./my-textbook-site/docs/intro.md
  2. ./my-textbook-site/docs/chapters/chapter1.md
  3. ./my-textbook-site/docs/chapters/chapter2.md
  4. ./my-textbook-site/docs/chapters/chapter3.md
  5. ./my-textbook-site/docs/chapters/chapter4.md
  6. ./my-textbook-site/docs/chapters/hardware-requirements.md
  7. ./my-textbook-site/docs/chapters/capstone-intro.md
  8. ./my-textbook-site/docs/chapters/quarter-overview.md
  9. ./my-textbook-site/docs/chapters/labs-intro.md

  🏗️ Architecture

  This textbook project is built using a modern tech stack:

  - Frontend: Docusaurus v3 for documentation and learning materials
  - Backend: FastAPI for the RAG chatbot and translation services
  - AI/ML: Integration with Gemini, Qwen, and other AI models
  - Vector Database: Qdrant for efficient document retrieval
  - Database: PostgreSQL with Neon for data persistence
  - Authentication: JWT-based authentication system

  🛠️ Tech Stack

  - Languages: Python, TypeScript, Markdown
  - Frameworks: Docusaurus, FastAPI, React
  - AI/ML: Google Gemini, OpenAI, ONNX Runtime
  - Databases: PostgreSQL, Qdrant
  - Other: Docker, Render deployment

  📋 Prerequisites

  Hardware Requirements

  This course is technically demanding, sitting at the intersection of three heavy computational loads:
  - Physics Simulation (e.g., Isaac Sim/Gazebo)
  - Visual Perception (e.g., SLAM/Computer Vision)
  - Generative AI (e.g., LLMs/VLA)

  For detailed hardware requirements, see ./my-textbook-site/docs/chapters/hardware-requirements.md.

  Software Requirements

  - Python 3.10+
  - Node.js 20+
  - NVIDIA GPU with RTX capabilities (for simulation)
  - Ubuntu 22.04 LTS (recommended)

  🚀 Getting Started

  Frontend (Documentation Site)

  1. Navigate to the my-textbook-site directory
  2. Install dependencies: npm install
  3. Start the development server: npm start
  4. Open http://localhost:3000 in your browser

  Backend (RAG Chatbot)

  1. Navigate to the rag-chatbot-backend directory
  2. Install dependencies: pip install -r requirements.txt
  3. Set up environment variables (see .env template)
  4. Start the server: uvicorn main:app --reload
  5. Backend will be available at http://localhost:8000

  🔐 Authentication

  The system includes a complete authentication system with:
  - User registration and login
  - JWT-based token authentication
  - Protected API endpoints
  - User session management

  🌐 Translation Services

  Built-in translation capabilities:
  - Urdu translation support
  - AI-powered translation using multiple models
  - Document translation functionality
  - Real-time translation for interactive learning

  🤖 RAG Chatbot

  The integrated RAG (Retrieval-Augmented Generation) chatbot provides:
  - Context-aware responses based on textbook content
  - Semantic search through course materials
  - Intelligent question answering
  - Multi-modal support

  📊 Modules

  Module 1: The Robotic Nervous System (ROS 2)

  Focuses on middleware for robot control, including ROS 2 nodes, topics, and services.

  Module 2: The Digital Twin (Gazebo & Unity)

  Covers physics simulation and environment building for safe and efficient development.

  Module 3: The AI-Robot Brain (NVIDIA Isaac™)

  Advanced perception and training using NVIDIA Isaac Sim and Isaac ROS.

  Module 4: Vision-Language-Action (VLA)

  Integration of LLMs with robotics for natural language understanding and cognitive planning.

  🚧 Development Status

  This project is actively maintained and under continuous development. We welcome contributions from the community.

  🤝 Contributing

  1. Fork the repository
  2. Create a feature branch (git checkout -b feature/amazing-feature)
  3. Commit your changes (git commit -m 'Add some amazing feature')
  4. Push to the branch (git push origin feature/amazing-feature)
  5. Open a Pull Request

  📄 License

  This project is licensed under the MIT License - see the LICENSE file for details.

  📞 Support

  For support, please open an issue in the GitHub repository or contact the maintainers.

  🙏 Acknowledgments

  - Thanks to NVIDIA for Isaac Sim and robotics frameworks
  - Google for Gemini AI integration
  - The Docusaurus team for excellent documentation tools
  - All contributors who help make this project better

  📈 Roadmap

  - Enhanced multi-language support
  - More interactive learning modules
  - Advanced simulation environments
  - Mobile application development
  - Integration with additional AI models

  ---
  Maintained by: Hafiza Alina Yasmeen
  Project Status: Active Development
  Last Updated: December 2025
