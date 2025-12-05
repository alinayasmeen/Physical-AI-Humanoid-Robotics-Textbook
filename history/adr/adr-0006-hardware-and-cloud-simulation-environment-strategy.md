# ADR-0006: Hardware and Cloud Simulation Environment Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-05
- **Feature:** physical-ai-humanoid-robotics
- **Context:** The textbook requires guiding students on setting up both physical hardware and scalable cloud environments for learning physical AI and humanoid robotics. This involves defining workstation, edge device, sensor, and robot requirements, as well as cloud simulation best practices.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? Yes
     2) Alternatives: Multiple viable options considered with tradeoffs? Yes
     3) Scope: Cross-cutting concern (not an isolated detail)? Yes
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Provide detailed hardware requirements for high-performance workstations (NVIDIA RTX GPUs like 4070Ti–4090), edge brains (NVIDIA Jetson Orin Nano/NX), perception sensors (Intel RealSense D435i/D455), and examples of humanoid/quadruped robots (e.g., Unitree Go2/G1). Complement this with a comprehensive guide for cloud-based simulation setup using AWS g5/g6e instances, emphasizing performance optimization, cost-effectiveness, and ease of setup for students.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Provides comprehensive guidance for students with diverse resources, enabling hands-on learning with both physical and virtual robots.
- High-performance hardware recommendations support demanding AI and robotics workloads, ensuring students can effectively train and run models.
- Cloud simulation offers unparalleled flexibility, scalability, and cost-effectiveness (through pay-as-you-go and Spot Instances), making advanced robotics accessible to a broader audience.
- Exposes students to industry-standard physical hardware and cloud computing platforms, enhancing their practical skills.

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- High upfront cost for recommended physical hardware, potentially limiting accessibility for some students.
- Complexity in setting up and maintaining multiple hardware and software environments, requiring significant technical effort.
- Requires students to understand cloud infrastructure concepts and AWS services for effective cloud-based simulation, adding to the learning burden.
- Rapid evolution of hardware and cloud offerings may lead to outdated recommendations over time, necessitating continuous updates to the textbook content.
- Potential for the \"sim-to-real gap\" if students exclusively rely on simulation without understanding the nuances of physical robot behavior.

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

-   **Focus Solely on Physical Hardware:**
    -   **Pros:** Provides direct, tangible experience with real robots, which is invaluable for understanding physical constraints.
    -   **Cons:** Extremely high cost for acquiring and maintaining robots and specialized hardware, limited scalability for large classes or research groups, difficult for remote learners, safety concerns with physical robots without proper supervision.
-   **Focus Solely on Cloud Simulation:**
    -   **Pros:** Highly scalable and accessible from anywhere with an internet connection, generally more cost-effective (pay-as-you-go model).
    -   **Cons:** Lacks hands-on experience with physical hardware, potential for a significant \"sim-to-real gap\" where simulated behaviors do not perfectly translate to the physical world, may require strong internet connectivity and understanding of cloud networking.

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
