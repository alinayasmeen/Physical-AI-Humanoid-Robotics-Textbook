# ADR-0005: LLM Cognitive Planning and Voice Command Integration

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-05
- **Feature:** physical-ai-humanoid-robotics
- **Context:** The textbook aims to demonstrate how high-level natural language instructions can be translated into ROS 2 actions for humanoid robots, combining Whisper for speech-to-text and LLMs for cognitive planning.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? Yes
     2) Alternatives: Multiple viable options considered with tradeoffs? Yes
     3) Scope: Cross-cutting concern (not an isolated detail)? Yes
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Integrate Whisper-based speech-to-text for voice commands with a Large Language Model (LLM) serving as the cognitive core for semantic planning. The LLM will interpret high-level natural language instructions, decompose them into a sequence of actionable sub-goals, which are then translated into executable ROS 2 actions via specialized ROS 2 agents.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Enables intuitive and natural human-robot interaction through voice commands, making robotics more accessible.
- Facilitates autonomous task execution by bridging the semantic gap between abstract human language and concrete robot capabilities.
- Provides a powerful demonstration of embodied AI and cognitive robotics, aligning with the textbook's vision.
- LLM's generalization capabilities can lead to more flexible and adaptive robot behaviors for novel tasks.

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- Challenges in achieving real-time synchronization and low latency from voice input to robot execution, which can impact user experience.
- Potential for model misalignment between the LLM's abstract reasoning and the robot's physical constraints, sensor data, and real-world physics.
- Complexity in managing scalable memory and context for long-horizon tasks and multi-turn conversations.
- Issues with grounding ambiguous natural language concepts (e.g., "tidy," "carefully") to concrete robot perceptions and actions.
- Computational overhead of running large LLMs and speech recognition models on robotic platforms or requiring cloud inference.

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

-   **Pre-programmed Command Set (no LLM):**
    -   **Pros:** Deterministic and predictable robot behavior, simpler to implement and debug, potentially lower latency.
    -   **Cons:** Limited flexibility, cannot understand novel commands or adapt to new situations, lacks cognitive planning capabilities, not aligned with the textbook's focus on advanced AI.
-   **Text-based Command Interface (no Whisper):**
    -   **Pros:** Avoids the complexities and latency inherent in speech-to-text processing.
    -   **Cons:** Less natural and intuitive for human interaction, limits accessibility for users who prefer voice commands, does not fully explore the potential of voice-controlled robotics.

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
