# ADR-0003: RAG-Powered Chatbot Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-05
- **Feature:** physical-ai-humanoid-robotics
- **Context:** The textbook requires an advanced feature: a RAG-powered chatbot for Q&A on textbook content. This involves a backend for data processing, retrieval, and response generation, integrated with the Docusaurus frontend.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? Yes
     2) Alternatives: Multiple viable options considered with tradeoffs? Yes
     3) Scope: Cross-cutting concern (not an isolated detail)? Yes
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement the RAG chatbot backend using FastAPI for the API layer, Qdrant for vector storage, and OpenAI embeddings for text-to-vector conversion. Orchestrate the RAG pipeline using a framework like LangChain.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Provides immediate, contextually aware answers grounded in the textbook content, significantly enhancing interactive learning.
- High-performance and scalable backend achievable with FastAPI and Qdrant for efficient data processing and retrieval.
- Leverages state-of-the-art embedding models from OpenAI for semantic understanding.
- Modular architecture allows for future improvements, model swapping, and flexibility in integrating other AI services.

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- Requires external API calls to OpenAI, incurring costs and potential latency. May involve vendor lock-in to OpenAI's ecosystem.
- Requires careful and continuous content processing and chunking to maintain accuracy and prevent hallucinations.
- Complexity of managing and updating the vector database (Qdrant) and ensuring embedding consistency with new content or model updates.
- Initial setup and integration with the Docusaurus frontend will require custom development and maintenance of frontend-backend communication.

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

-   **Fully Client-Side RAG (using local embeddings/models):**
    -   **Pros:** No backend needed, potentially lower operational costs if models are open-source and run efficiently client-side, enhanced user data privacy.
    -   **Cons:** Limited model size and complexity due to browser constraints, slower inference on less powerful client devices, challenging to dynamically update the knowledge base, no persistent memory across user sessions.
-   **Serverless RAG (e.g., AWS Lambda, Azure Functions):**
    -   **Pros:** Scalability on demand, reduced operational overhead with a pay-per-use pricing model.
    -   **Cons:** Potential cold start latencies affecting user experience, increased complexity in managing vector database connections from ephemeral serverless functions, potential vendor lock-in to a specific cloud provider's serverless ecosystem.

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: `specs/physical-ai-humanoid-robotics/spec.md`
- Implementation Plan: `specs/physical-ai-humanoid-robotics/plan.md`
- Data Model: `specs/physical-ai-humanoid-robotics/data-model.md`
- API Contract: `specs/physical-ai-humanoid-robotics/contracts/rag_chatbot_api.yaml`
- Related ADRs: N/A
- Evaluator Evidence: N/A <!-- link to eval notes/PHR showing graders and outcomes -->
