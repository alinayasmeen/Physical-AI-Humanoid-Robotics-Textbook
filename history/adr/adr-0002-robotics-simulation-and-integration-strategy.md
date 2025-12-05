# ADR-0002: Robotics Simulation and Integration Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-05
- **Feature:** physical-ai-humanoid-robotics
- **Context:** The textbook requires a comprehensive approach to robotics simulation, integrating multiple platforms (ROS 2, Gazebo, Unity, NVIDIA Isaac Sim) to cover various aspects of physical AI and humanoid robotics.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? Yes
     2) Alternatives: Multiple viable options considered with tradeoffs? Yes
     3) Scope: Cross-cutting concern (not an isolated detail)? Yes
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Adopt a multi-simulator strategy, leveraging ROS 2 as the middleware. Utilize `ros_gz_bridge` for Gazebo integration, and dedicated ROS 2 bridge extensions for Unity and NVIDIA Isaac Sim. Emphasize the new ROS simulation standard for interoperability and future-proofing.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Comprehensive coverage of diverse simulation capabilities, allowing students to learn different tools.
- Optimized environments for specific tasks (e.g., Unity for visualization, Isaac Sim for ML-optimized physics and synthetic data).
- Enhanced learning experience by exposing students to multiple industry-relevant tools and their integration.
- Reduced long-term migration costs and improved interoperability through standardized ROS simulation interfaces.

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- Increased complexity in initial setup and configuration due to the integration of multiple distinct platforms.
- Potential for synchronization challenges and debugging overhead when operating across different simulators.
- Steeper learning curve for students to master the intricacies of all integrated tools and their communication protocols.
- Overhead of maintaining and updating multiple simulation environments as they evolve.

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

-   **Single-Simulator Approach (e.g., solely Gazebo or Isaac Sim):**
    -   **Pros:** Simpler setup and maintenance, reduced learning curve focused on one platform.
    -   **Cons:** Limited capabilities compared to a multi-simulator approach (e.g., Gazebo lacks advanced photorealism of Unity, Isaac Sim might not have the same breadth of community support as ROS-Gazebo), may not meet all textbook requirements for diverse topics in physical AI and humanoid robotics.
-   **Custom Simulation Environment:**
    -   **Pros:** Complete control and optimization tailored for extremely specific research or educational needs.
    -   **Cons:** Extremely high development cost and time, lack of community support and existing tooling, not suitable for a broad educational textbook aiming to introduce industry-standard tools.

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
