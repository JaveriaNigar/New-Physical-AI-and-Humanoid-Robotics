qw---
description: "Updated Task list: Sidebar structure for Physical AI & Humanoid Robotics Textbook"
---

# Tasks: Updated Sidebar Structure for Physical AI & Humanoid Robotics Textbook

**Input**: Sidebar format requirements from user input
**Feature**: Create a clean, uniform sidebar matching the specified structure

**Tests**: No explicit testing requirements for sidebar structure, so tests are not included.

**Organization**: Tasks are grouped by implementation phases to enable systematic development.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: `docs/`, `static/`, `src/`, `blog/` at repository root
- **Configuration files**: `docusaurus.config.js`, `sidebars.js` at repository root
- Paths adjusted based on plan.md structure for Docusaurus textbook

<!--
  ============================================================================
  IMPORTANT: These are the actual tasks generated based on:
  - User stories from spec.md (with their priorities P1, P2, P3...)
  - Feature requirements from plan.md
  - Research findings from research.md
  - Sidebar structure requirements from user input

  Tasks are organized by user story so each story can be:
  - Implemented independently
  - Tested independently
  - Delivered as an MVP increment
  ============================================================================
-->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify project infrastructure for sidebar implementation

- [X] T001 Verify Docusaurus project structure exists in physical-ai-book directory
- [X] T002 Confirm sidebars.js file exists at repository root with current structure
- [X] T003 Ensure all weekly content directories exist (week-1-2, week-3-5, week-6-7, week-8-10, week-11-13)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before sidebar restructuring

**⚠️ CRITICAL**: Sidebar work can begin only after this phase is complete

- [X] T004 Backup existing sidebars.js file to prevent accidental loss
- [X] T005 Analyze current content structure in docs/ directory
- [X] T006 Map existing content files to required sidebar structure
- [ ] T007 Create category files (_category_.json) for each module if they don't match the required format

**Checkpoint**: Foundation ready - Sidebar implementation can now begin

---

## Phase 3: User Story 1 - Module 1 Structure (Priority: P1) 🎯 MVP

**Goal**: Ensure sidebar structure for Module 1 Weeks 1-2: Introduction to Physical AI matches the required format

**Independent Test**: Students can navigate to Module 1 content and see structured categories for Humanoid Robotics Overview, Physical AI and Embodied Intelligence, and Sensors in Physical AI Systems.

### Implementation for User Story 1

- [ ] T008 [P] [US1] Update _category_.json in docs/week-1-2/ with title "Module 1: Introduction to Physical AI"
- [ ] T009 [P] [US1] Verify content file 'humanoid-robots.md' exists in docs/week-1-2/ (maps to Humanoid Robotics Overview)
- [ ] T010 [P] [US1] Verify content file 'physical-ai-intro.md' exists in docs/week-1-2/ (maps to Physical AI and Embodied Intelligence)
- [ ] T011 [US1] Verify content file 'sensors-overview.md' exists in docs/week-1-2/ (maps to Sensors in Physical AI Systems)
- [ ] T012 [US1] Verify all Module 1 content files have proper Docusaurus frontmatter
- [ ] T013 [US1] Ensure content files are properly organized in sidebar structure

**Checkpoint**: At this point, Module 1 structure should be fully functional and testable

---

## Phase 4: User Story 2 - Module 2 Structure (Priority: P2)

**Goal**: Ensure sidebar structure for Module 2 Weeks 3-5: ROS 2 Fundamentals and Weeks 6-7: Robot Simulation with Gazebo matches the required format

**Independent Test**: Students can navigate to Module 2 content and see structured categories for ROS 2 Fundamentals and Gazebo simulation.

### Implementation for User Story 2

- [ ] T014 [P] [US2] Update _category_.json in docs/week-3-5/ with title "Module 2: ROS 2 Fundamentals"
- [ ] T015 [P] [US2] Create/update ROS 2 content files in docs/week-3-5/ directory to match required topics
- [ ] T016 [P] [US2] Create content file for Robot Data Flow in docs/week-3-5/robot-data-flow.md
- [ ] T017 [US2] Create content file for ROS 2 Architecture in docs/week-3-5/ros2-architecture-nodes-topics-services-actions.md
- [ ] T018 [US2] Create content file for ROS 2 with Python in docs/week-3-5/ros2-with-python-rclpy.md
- [ ] T019 [US2] Update _category_.json in docs/week-6-7/ with title "Weeks 6-7: Robot Simulation with Gazebo"
- [ ] T020 [P] [US2] Create content file for Gazebo Physics in docs/week-6-7/gazebo-physics-collisions-environment-design.md
- [ ] T021 [US2] Create content file for Sensor Simulation in docs/week-6-7/sensor-simulation-lidar-depth-imu.md
- [ ] T022 [US2] Add Docusaurus frontmatter to all Module 2 content files
- [ ] T023 [US2] Verify all Module 2 content files have proper titles and descriptions

**Checkpoint**: At this point, Modules 1 AND 2 structures should both work independently

---

## Phase 5: User Story 3 - Module 3 Structure (Priority: P3)

**Goal**: Ensure sidebar structure for Module 3 Weeks 8-10: The AI-Robot Brain (NVIDIA Isaac™) matches the required format

**Independent Test**: Students can navigate to Module 3 content and see structured categories for NVIDIA Isaac components.

### Implementation for User Story 3

- [ ] T024 [P] [US3] Update _category_.json in docs/week-8-10/ with title "Module 3: The AI-Robot Brain (NVIDIA Isaac™)"
- [ ] T025 [P] [US3] Create content file for AI-Robot Brain Overview in docs/week-8-10/ai-robot-brain-overview.md
- [ ] T026 [P] [US3] Create content file for NVIDIA Isaac Architecture in docs/week-8-10/nvidia-isaac-architecture.md
- [ ] T027 [US3] Create content file for Cognitive Planning in docs/week-8-10/cognitive-planning-actions.md
- [ ] T028 [US3] Add Docusaurus frontmatter to all Module 3 content files
- [ ] T029 [US3] Verify all Module 3 content files have proper titles and descriptions

**Checkpoint**: At this point, Modules 1, 2 AND 3 structures should all work independently

---

## Phase 6: User Story 4 - Module 4 Structure (Priority: P4)

**Goal**: Ensure sidebar structure for Module 4 Weeks 11-12: Humanoid Development & VLA Systems matches the required format

**Independent Test**: Students can navigate to Module 4 content and see structured categories for Humanoid Development and VLA Systems.

### Implementation for User Story 4

- [ ] T030 [P] [US4] Update _category_.json in docs/week-11-13/ with title "Module 4: Humanoid Development & VLA Systems"
- [ ] T031 [P] [US4] Create content file for Humanoid Development Overview in docs/week-11-13/humanoid-development-overview.md
- [ ] T032 [P] [US4] Create content file for Vision-Language-Action Systems in docs/week-11-13/vision-language-action-systems.md
- [ ] T033 [US4] Create content file for Context-Aware Behavior in docs/week-11-13/context-aware-behavior.md
- [ ] T034 [US4] Add Docusaurus frontmatter to all Module 4 content files
- [ ] T035 [US4] Verify all Module 4 content files have proper titles and descriptions

**Checkpoint**: At this point, Modules 1-4 should all work independently

---

## Phase 7: User Story 5 - Capstone Structure (Priority: P5)

**Goal**: Ensure sidebar structure for the Capstone Project (Week 13) matches the required format

**Independent Test**: Students can navigate to the Capstone project content with proper structure.

### Implementation for User Story 5

- [ ] T036 [P] [US5] Update _category_.json in docs/capstone/ with title "Capstone: Autonomous Humanoid Project"
- [ ] T037 [P] [US5] Create content file for Capstone Overview in docs/capstone/capstone-overview.md
- [ ] T038 [US5] Create content file for Voice Command Integration in docs/capstone/voice-command-integration.md
- [ ] T039 [US5] Create content file for Path Planning in docs/capstone/path-planning-nav2.md
- [ ] T040 [US5] Create content file for Navigation and Object Interaction in docs/capstone/navigation-object-interaction.md
- [ ] T041 [US5] Create content file for Complete System Integration in docs/capstone/complete-system-integration.md
- [ ] T042 [US5] Add Docusaurus frontmatter to all Capstone content files
- [ ] T043 [US5] Verify all Capstone content files have proper titles and descriptions

**Checkpoint**: At this point, all modules should be structured with proper sidebar navigation

---

## Phase 8: User Story 6 - Sidebar Configuration (Priority: P6)

**Goal**: Update the sidebars.js file to reflect the new structured modules with the exact format specified

**Independent Test**: The sidebar displays the clean, uniform format as specified by the user.

### Implementation for User Story 6

- [ ] T044 [P] [US6] Replace existing sidebars.js with structured format for all modules
- [ ] T045 [P] [US6] Define Module 1 (Weeks 1-2) structure in sidebars.js with exact required format
- [ ] T046 [P] [US6] Define Module 2 (Weeks 3-5 and 6-7) structure in sidebars.js with exact required format
- [ ] T047 [P] [US6] Define Module 3 (Weeks 8-10) structure in sidebars.js with exact required format
- [ ] T048 [US6] Define Module 4 (Weeks 11-12) structure in sidebars.js with exact required format
- [ ] T050 [US6] Add proper type and title properties to all sidebar entries
- [ ] T051 [US6] Test sidebar structure renders correctly in Docusaurus

**Checkpoint**: Sidebar now reflects the required structure and displays properly

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect the entire textbook sidebar including consistency, navigation, and overall quality

- [ ] T052 [P] Verify all content files have consistent Docusaurus frontmatter
- [ ] T053 [P] Ensure all content files include proper learning objectives and exercises
- [ ] T054 [P] Add citations to all technical claims throughout content
- [ ] T055 [P] Verify that at least 40% of citations are from peer-reviewed or academic sources
- [ ] T056 [P] Review and improve readability to maintain Flesch-Kincaid grade level 10-12
- [ ] T057 [P] Run Docusaurus build process to ensure site generates without errors
- [ ] T058 [P] Test sidebar navigation works correctly in local development
- [ ] T059 [P] Verify all links in sidebar are valid and point to correct content
- [ ] T060 [P] Final review of sidebar structure for consistency and formatting

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in priority order (P1 → P2 → P3 → P4 → P5 → P6)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May use concepts from US1
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May use concepts from US1-US2
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - May use concepts from US1-US3
- **User Story 5 (P5)**: Can start after Foundational (Phase 2) - Integrates concepts from US1-US4
- **User Story 6 (P6)**: Can start after Foundational (Phase 2) - Depends on content from US1-US5

### Within Each User Story

- Core content implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- All tasks within each user story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members (though sequential order is recommended for consistency)

### Parallel Example: User Story 1

```bash
# Launch all Module 1 structure tasks together:
Task: "Update _category_.json in docs/week-1-2/ with title \"Module 1: Introduction to Physical AI\""
Task: "Verify content file 'humanoid-robots.md' exists in docs/week-1-2/ (maps to Humanoid Robotics Overview)"
Task: "Verify content file 'physical-ai-intro.md' exists in docs/week-1-2/ (maps to Physical AI and Embodied Intelligence)"
Task: "Verify content file 'sensors-overview.md' exists in docs/week-1-2/ (maps to Sensors in Physical AI Systems)"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 - Module 1 structure
4. **STOP and VALIDATE**: Test Module 1 structure independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 (Module 1) → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 (Module 2) → Test independently → Deploy/Demo
4. Add User Story 3 (Module 3) → Test independently → Deploy/Demo
5. Add User Story 4 (Module 4) → Test independently → Deploy/Demo
6. Add User Story 5 (Capstone) → Test independently → Deploy/Demo
7. Add User Story 6 (Sidebar Config) → Test independently → Deploy/Demo
8. Polish & Cross-Cutting Concerns → Final deploy (Complete Textbook!)

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Stories 1 (Module 1)
   - Developer B: User Story 2 (Module 2)
   - Developer C: User Story 3 (Module 3)
3. After US1-3 are complete:
   - Developer A: User Story 4 (Module 4)
   - Developer B: User Story 5 (Capstone)
   - Developer C: User Story 6 (Sidebar Configuration)
4. All developers: Polish & Cross-Cutting Concerns

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence