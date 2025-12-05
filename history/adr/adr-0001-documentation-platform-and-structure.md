# ADR-0001: Documentation Platform and Structure

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-05
- **Feature:** physical-ai-humanoid-robotics
- **Context:** The project requires a comprehensive, multi-chapter textbook website. Key requirements include distinct sections for course, labs, and capstone, integrated search, dark/light mode, mobile-friendliness, and GitHub Pages deployment.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? Yes
     2) Alternatives: Multiple viable options considered with tradeoffs? Yes
     3) Scope: Cross-cutting concern (not an isolated detail)? Yes
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Utilize Docusaurus v3 with multiple documentation instances (`/docs`, `/course`, `/labs`, `/capstone`). Integrate Algolia DocSearch for search functionality, leverage Docusaurus's built-in theme features for dark/light mode and mobile-friendly UI, and implement GitHub Actions for automated deployment to GitHub Pages.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Clear content separation for different parts of the textbook (course, labs, capstone).
- Strong community support and extensive documentation for Docusaurus.
- Rich feature set out-of-the-box, including navigation, search, and theming.
- Efficient static site generation for fast loading times and security.
- Streamlined deployment process using GitHub Actions.
- Mobile-friendly UI with built-in responsiveness.

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- Initial learning curve for Docusaurus specifics, especially multi-instance configuration.
- Potential for complex customization beyond basic theming.
- Reliance on Algolia for optimal search, requiring application and configuration.
- Markdown-centric content creation might be restrictive for highly interactive elements without custom React components.

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

- **Custom React Application:**
    - **Pros:** Full control over UI/UX, maximum flexibility for custom interactive elements.
    - **Cons:** High development overhead, manual implementation of common documentation features (search, navigation, markdown parsing), increased maintenance burden, slower initial setup.
- **MkDocs:**
    - **Pros:** Simpler setup for basic documentation, Python-based for easier integration with Python toolchains.
    - **Cons:** Less feature-rich than Docusaurus for complex multi-chapter textbook structures, limited extensibility for advanced features (RAG chatbot, Skill Agent) without significant custom work, less polished out-of-the-box UI/UX.

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: `specs/physical-ai-humanoid-robotics/spec.md`
- Implementation Plan: `specs/physical-ai-humanoid-robotics/plan.md`
- Related ADRs: N/A
- Evaluator Evidence: N/A <!-- link to eval notes/PHR showing graders and outcomes -->
