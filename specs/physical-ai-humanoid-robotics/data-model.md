# Data Models for Physical AI & Humanoid Robotics Textbook Advanced Features

## 1. RAG-Powered Chatbot Data Models

### 1.1. `TextChunk`
Represents a segment of the textbook content (e.g., a paragraph, a section) that is suitable for embedding and retrieval.
- **Fields:**
    - `id`: string (Unique identifier for the chunk)
    - `content`: string (The raw text content of the chunk)
    - `source_file`: string (Path to the original Docusaurus markdown file)
    - `section_heading`: string (Heading of the section the chunk belongs to, for context)
    - `page_url`: string (URL of the Docusaurus page where the chunk originates)
    - `metadata`: object (Additional metadata, e.g., chapter, module, topic tags)

### 1.2. `VectorEmbedding`
Represents the high-dimensional numerical vector embedding of a `TextChunk` or a `UserQuery`.
- **Fields:**
    - `id`: string (References the `TextChunk.id` or `UserQuery.id`)
    - `vector`: array of floats (The embedding vector)
    - `model_id`: string (Identifier for the embedding model used)

### 1.3. `UserQuery`
Represents an incoming query from the Docusaurus frontend to the RAG chatbot.
- **Fields:**
    - `id`: string (Unique identifier for the query session/message)
    - `text`: string (The raw text of the user's question)
    - `timestamp`: datetime (Time when the query was received)
    - `session_id`: string (Identifier for the user's chat session)
    - `context_page_url`: string (URL of the Docusaurus page from which the query was made, for additional context)

### 1.4. `ChatResponse`
Represents the generated response from the RAG chatbot.
- **Fields:**
    - `id`: string (Unique identifier for the response message)
    - `query_id`: string (References the `UserQuery.id`)
    - `answer_text`: string (The generated natural language answer)
    - `retrieved_chunks_ids`: array of strings (References `TextChunk.id`s that were used as context)
    - `confidence_score`: float (Optional: score indicating confidence in the answer)
    - `timestamp`: datetime (Time when the response was generated)
    - `source_links`: array of objects (Links to relevant Docusaurus pages or sections)
        - `title`: string
        - `url`: string

## 2. Robotics Skill Agent Data Models

### 2.1. `RoboticsQuery`
Represents an incoming query from the Docusaurus frontend to the Robotics Skill Agent.
- **Fields:**
    - `id`: string (Unique identifier for the query session/message)
    - `text`: string (The raw text of the user's question, e.g., \"How do I create a ROS 2 node?\")
    - `timestamp`: datetime
    - `session_id`: string
    - `context_page_url`: string (URL of the Docusaurus page from which the query was made, for additional context on the topic)
    - `robotics_framework_hint`: string (Optional: e.g., \"ROS 2\", \"Gazebo\", \"Isaac Sim\")
    - `version_hint`: string (Optional: e.g., \"Humble\", \"Ignition Fortress\")

### 2.2. `KnowledgeSnippet`
Represents a retrieved piece of information from the structured robotics knowledge base or RAG system.
- **Fields:**
    - `id`: string (Unique identifier for the snippet)
    - `content`: string (The textual content of the knowledge snippet)
    - `source_type`: enum (e.g., \"TEXTBOOK_RAG\", \"ROS_WIKI\", \"ISAAC_DOCS\", \"GAZEBO_TUTORIAL\")
    - `source_url`: string (URL to the original source of the snippet)
    - `tags`: array of strings (Keywords or topics associated with the snippet, e.g., \"ROS 2 nodes\", \"Gazebo physics\")
    - `relevance_score`: float (Score indicating relevance to the query)

### 2.3. `SkillAgentResponse`
Represents the generated response from the Robotics Skill Agent.
- **Fields:**
    - `id`: string
    - `query_id`: string (References the `RoboticsQuery.id`)
    - `answer_text`: string (The generated natural language answer, including code snippets, formatted markdown)
    - `retrieved_snippets_ids`: array of strings (References `KnowledgeSnippet.id`s used as context)
    - `tool_calls`: array of objects (Optional: details if the agent invoked any external tools)
        - `tool_name`: string
        - `tool_arguments`: object
        - `tool_output`: string
    - `confidence_score`: float
    - `timestamp`: datetime
    - `suggested_next_steps`: array of strings (e.g., \"Try running `ros2 node list`\", \"Refer to Chapter 3 for URDF modeling\")

### 2.4. `ToolCall`
Represents a potential action the Skill Agent might take using external tools (e.g., a ROS 2 command executor, a simulator API wrapper).
- **Fields:**
    - `tool_name`: string (e.g., \"ROS2_CLI_Executor\", \"Gazebo_Simulator_API\")
    - `arguments`: object (Parameters for the tool, e.g., `{"command": "ros2 node list"}`)
    - `expected_output_format`: string (e.g., \"text\", \"JSON\")
    - `execution_status`: enum (\"PENDING\", \"SUCCESS\", \"FAILED\")
    - `output`: string (The result of the tool's execution)
