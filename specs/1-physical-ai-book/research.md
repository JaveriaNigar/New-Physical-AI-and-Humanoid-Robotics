# Research: Physical AI & Humanoid Robotics Textbook

## Summary of Research Findings

This research phase focused on clarifying initial technical environment details for the Docusaurus textbook project.

## Decisions and Rationale

### Docusaurus Version
- **Decision**: Latest stable (v3.x)
- **Rationale**: Utilizing the latest stable version ensures access to current features, performance improvements, and ongoing community support, minimizing potential compatibility issues with newer web technologies. This aligns with standard best practices for new documentation projects.
- **Alternatives considered**: Specific older versions (rejected to avoid legacy issues); latest beta/RC (rejected due to potential instability).

### Node.js Version
- **Decision**: Latest LTS (Long Term Support) release (e.g., 18.x, 20.x)
- **Rationale**: Targeting the latest LTS release for Node.js provides a stable and well-supported development environment. This reduces the risk of encountering transient bugs or compatibility problems, which is crucial for a textbook aimed at reproducibility by students. It ensures a consistent build experience over time.
- **Alternatives considered**: Latest current release (rejected due to potential for rapid changes and less stability); specific older versions (rejected to avoid legacy issues).
