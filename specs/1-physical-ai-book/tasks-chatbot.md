---

description: "Task list for AI Chatbot UI feature implementation"
---

# Tasks: AI Chatbot UI for Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/1-physical-ai-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: `src/`, `static/`, `docs/` at repository root
- **Components**: `src/components/` for React components
- **CSS**: `src/css/custom.css` for global styles
- **Pages**: `src/pages/` for Docusaurus page overrides

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for chatbot component

- [X] T001 Create project structure for AI chatbot component in physical-ai-book/
- [X] T002 [P] Install additional dependencies for chat UI components: react-icons, framer-motion (for animations)
- [X] T003 [P] Configure linting and formatting tools for React components

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core chatbot infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Create base styling variables for pink/white theme in src/css/custom.css
- [X] T005 [P] Define color palette: primary (pink variants), secondary (white), accent (black/dark gray)
- [X] T006 [P] Set up responsive layout foundation for chatbot UI in src/css/chatbot-global.css
- [X] T007 Create base chatbot component structure in src/components/AIChatbot/
- [X] T008 Define chat message data structure and types in src/types/chat.d.ts
- [X] T009 Set up state management foundation for chat interface in src/context/ChatContext.tsx

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Floating Chat Icon (Priority: P1) 🎯 MVP

**Goal**: Implement a floating "Ask AI" chatbot icon at the bottom-right of the page

**Independent Test**: Floating chat icon appears at bottom-right of page and is visible on all routes

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

- [ ] T010 [P] [US1] Visual regression test for floating icon across different screen sizes
- [ ] T011 [P] [US1] Click interaction test for floating icon in browser automation

### Implementation for User Story 1

- [X] T012 [P] [US1] Create floating chat button component in src/components/AIChatbot/FloatingButton.tsx
- [X] T013 [P] [US1] Implement pink/white styling for chat icon in src/components/AIChatbot/FloatingButton.module.css
- [X] T014 [US1] Position chat icon at bottom-right of viewport with CSS positioning
- [X] T015 [US1] Add smooth animation for hover state in src/components/AIChatbot/FloatingButton.module.css
- [X] T016 [US1] Ensure icon remains visible on scroll in all pages
- [X] T017 [US1] Make icon responsive across different device sizes
- [X] T018 [US1] Integrate floating button into Docusaurus layout in src/theme/Layout/index.js

**Checkpoint**: At this point, the floating chat icon should appear at bottom-right of the page and remain visible on all routes

---

## Phase 4: User Story 2 - Popup Chat Interface (Priority: P2)

**Goal**: On clicking the floating icon, a small popup chatbot opens with modern, clean design

**Independent Test**: Clicking floating icon opens a chat interface with header, message area, and input

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T019 [P] [US2] Modal open/close functionality test in src/components/AIChatbot/__tests__/ChatModal.test.tsx
- [ ] T020 [P] [US2] State management test for chat visibility in src/components/AIChatbot/__tests__/ChatModal.test.tsx

### Implementation for User Story 2

- [ ] T021 [P] [US2] Create modal/popup component structure in src/components/AIChatbot/ChatModal.tsx
- [ ] T022 [P] [US2] Implement pink/white styling for chat modal in src/components/AIChatbot/ChatModal.module.css
- [ ] T023 [US2] Create header with "AI Assistant" text in src/components/AIChatbot/ChatModal.tsx
- [ ] T024 [US2] Implement chat messages area with scrollable container in src/components/AIChatbot/ChatModal.tsx
- [ ] T025 [US2] Add input box with send button in src/components/AIChatbot/ChatModal.tsx
- [ ] T026 [US2] Handle click interaction between floating button and modal open/close
- [ ] T027 [US2] Add smooth entrance/exit animations for the chat modal
- [ ] T028 [US2] Ensure modal closes when clicking outside the chat window
- [ ] T029 [US2] Make the chat interface responsive and mobile-friendly

**Checkpoint**: At this point, clicking the floating icon should open a modern-looking chat interface with header text "AI Assistant", message area, and input box with send button

---

## Phase 5: User Story 3 - Basic Chat Functionality (Priority: P3)

**Goal**: Enable basic interaction with the chat interface (typing and displaying messages)

**Independent Test**: User can type messages in the input box, submit them, and see them displayed in the chat area

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T030 [P] [US3] Message submission test in src/components/AIChatbot/__tests__/ChatInterface.test.tsx
- [ ] T31 [P] [US3] Message display test in src/components/AIChatbot/__tests__/ChatInterface.test.tsx

### Implementation for User Story 3

- [ ] T032 [P] [US3] Implement state management for chat messages in src/components/AIChatbot/ChatInterface.tsx
- [ ] T033 [P] [US3] Create functionality to add user messages to chat history in src/components/AIChatbot/ChatInterface.tsx
- [ ] T034 [US3] Display messages in the chat messages area with proper styling
- [ ] T035 [US3] Implement input handling for user typing in the input box
- [ ] T036 [US3] Connect send button functionality to submit input
- [ ] T037 [US3] Create basic UI differentiation between user and bot messages
- [ ] T038 [US3] Add auto-scroll to bottom when new messages are added
- [ ] T039 [US3] Implement keyboard submit (Enter key) functionality
- [ ] T040 [US3] Add placeholder text in the input box

**Checkpoint**: At this point, users should be able to type messages, submit them via button or Enter key, and see them displayed in the chat area

---

## Phase 6: User Story 4 - UI Polish & Responsiveness (Priority: P4)

**Goal**: Enhance the UI with proper styling, improve responsiveness, and ensure lightweight performance

**Independent Test**: Chat interface looks polished with consistent pink/white theme and performs well on all device sizes

### Tests for User Story 4 (OPTIONAL - only if tests requested) ⚠️

- [ ] T041 [P] [US4] Responsive design validation across different screen sizes
- [ ] T042 [P] [US4] Performance test to ensure lightweight UI

### Implementation for User Story 4

- [ ] T043 [P] [US4] Refine color scheme to ensure accessibility compliance in src/css/custom.css
- [ ] T044 [P] [US4] Optimize component structure for lightweight performance in src/components/AIChatbot/
- [ ] T045 [US4] Add loading indicators for message sending/receiving in src/components/AIChatbot/ChatInterface.tsx
- [ ] T046 [US4] Implement smooth scrolling behavior for chat history in src/components/AIChatbot/ChatInterface.tsx
- [ ] T047 [US4] Add accessibility attributes (aria labels, keyboard navigation) to all chat elements
- [ ] T048 [US4] Fine-tune animations and transitions for better UX in src/components/AIChatbot/ChatModal.module.css
- [ ] T049 [US4] Optimize images and assets (if any) for fast loading in static/img/
- [ ] T050 [US4] Implement proper focus management for keyboard users in src/components/AIChatbot/ChatInterface.tsx
- [ ] T051 [US4] Ensure consistent styling across all Docusaurus themes in src/css/custom.css
- [ ] T052 [US4] Add subtle visual enhancements that fit the pink/white theme

**Checkpoint**: At this point, the chat interface should be polished, responsive, lightweight, and accessible with consistent pink/white styling

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T053 [P] Documentation updates in docs/ for the new chatbot feature
- [ ] T054 Finalize accessibility compliance across all chatbot components
- [ ] T055 [P] Cross-browser compatibility testing for the chat interface
- [ ] T056 [P] Mobile responsiveness validation for all chatbot components
- [ ] T057 Add analytics/tracking for chatbot usage (optional enhancement)
- [ ] T058 Update homepage layout to mention the new AI assistant feature
- [ ] T059 Run quickstart.md validation to ensure the site still builds properly

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Depends on User Story 1 completion (needs the floating button to interact with)
- **User Story 3 (P3)**: Depends on User Story 2 completion (needs the modal interface to add functionality to)
- **User Story 4 (P4)**: Depends on User Story 3 completion (polishes the fully functional chat)

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Component structure before styling
- Basic functionality before polish
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, user stories with no dependencies can start in parallel
- All tests for a user story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members (when no dependencies exist)

---

## Parallel Example: User Story 2

```bash
# Launch all styling tasks for User Story 2 together:
Task: "Implement pink/white styling for chat modal in src/components/AIChatbot/ChatModal.module.css"
Task: "Create header with "AI Assistant" text in src/components/AIChatbot/ChatModal.tsx"
Task: "Add smooth entrance/exit animations for the chat modal"
```

---

## Implementation Strategy

### MVP First (User Stories 1 and 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Floating chat icon)
4. Complete Phase 4: User Story 2 (Popup chat interface)
5. **STOP and VALIDATE**: Test the basic functionality independently
6. Deploy/demo if ready (users can see and open the chat interface)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo
3. Add User Story 2 → Test independently → Deploy/Demo (MVP - UI is working!)
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: Starts User Story 2 (after US1 is done)
   - Developer C: Starts User Story 3 (after US2 is done)
3. Stories build upon each other sequentially but can have overlapping work

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence