---
description: "Task list for UI/UX Improvements & Internationalization"
---

# Tasks: UI/UX Improvements & Internationalization

**Input**: Design documents from `/specs/002-ui-ux-improvements-and-i18n/`
**Prerequisites**: spec.md (required for user stories)

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `docusaurus/` at repository root
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Landing Page Redesign (US1)

**Purpose**: Modern, elegant landing page with custom navigation

- [x] T001 [US1] Redesign landing page at `docusaurus/src/pages/index.tsx` with modern hero section
- [x] T002 [US1] Create custom navigation bar with logo, tabs, and theme toggle
- [x] T003 [US1] Implement scroll effects for navigation bar in `docusaurus/src/pages/index.module.css`
- [x] T004 [US1] Add animated background orbs and gradient text effects
- [x] T005 [US1] Create learning path modules section with hover effects
- [x] T006 [US1] Implement features section with responsive grid (2 cards per row)
- [x] T007 [US1] Add CTA section with call-to-action buttons
- [x] T008 [US1] Create footer component with social links and navigation
- [x] T009 [US1] Implement manual theme toggle (without useColorMode hook)
- [x] T010 [US1] Hide default Docusaurus navbar on landing page
- [x] T011 [US1] Add comprehensive dark mode support for all sections
- [x] T012 [US1] Ensure light theme displays white backgrounds correctly

## Phase 2: Internationalization - Urdu Support (US2)

**Purpose**: Complete Urdu translation with RTL support

- [x] T013 [US2] Configure i18n in `docusaurus/docusaurus.config.ts` for English and Urdu
- [x] T014 [US2] Add locale dropdown to navbar configuration
- [x] T015 [US2] Create Urdu translation directory structure: `i18n/ur/docusaurus-plugin-content-docs/current/`
- [x] T016 [US2] Translate Module 1 files (5 files: intro, index, ros-nodes, topics-services, rclpy-bridge, urdf-humanoids)
- [x] T017 [US2] Translate Module 2 files (5 files: index, physics-simulation, collisions, rendering, sensors)
- [x] T018 [US2] Translate Module 3 files (4 files: index, isaac-sim, vslam-navigation, nav2-bipedal)
- [x] T019 [US2] Translate Module 4 files (4 files: index, voice-to-action, llm-planning, capstone-project)
- [x] T020 [US2] Translate Appendix files (3 files: faq, troubleshooting, glossary)
- [x] T021 [US2] Create Urdu translations for landing page: `i18n/ur/docusaurus-plugin-content-pages/index.json`
- [x] T022 [US2] Create Urdu navbar translations: `i18n/ur/docusaurus-theme-classic/navbar.json`
- [x] T023 [US2] Create Urdu footer translations: `i18n/ur/docusaurus-theme-classic/footer.json`
- [x] T024 [US2] Update landing page to use `<Translate>` components for all text
- [x] T025 [US2] Verify RTL layout works correctly for Urdu content
- [x] T026 [US2] Test automatic locale switching in production build

## Phase 3: Chatbot Source Links Fix (US3)

**Purpose**: Accurate source links displayed at bottom of chat

- [x] T027 [US3] Fix URL generation in `backend/app/services/document_service.py` to include baseUrl
- [x] T028 [US3] Remove file extensions from generated URLs (.md, .mdx)
- [x] T029 [US3] Add title extraction from frontmatter in `backend/app/utils/document_parser.py`
- [x] T030 [US3] Move sources display from inline to bottom section in `docusaurus/src/components/Chatbot/Chatbot.tsx`
- [x] T031 [US3] Add `currentSources` state management for bottom display
- [x] T032 [US3] Implement sources clearing on new message send
- [x] T033 [US3] Filter invalid sources (empty URL/title)
- [x] T034 [US3] Add bottom sources section styling in `docusaurus/src/components/Chatbot/Chatbot.css`
- [x] T035 [US3] Ensure sources display above input area
- [x] T036 [US3] Add dark mode support for bottom sources section
- [x] T037 [US3] Make sources section expandable with source count

## Phase 4: SSR Compatibility & Bug Fixes

**Purpose**: Ensure all components work with server-side rendering

- [x] T038 [P] Wrap Chatbot component in BrowserOnly for SSR compatibility
- [x] T039 [P] Add window/document checks in `docusaurus/src/pages/index.tsx`
- [x] T040 [P] Fix SSR issues in Chatbot component (window.getSelection)
- [x] T041 [P] Disable showLastUpdateAuthor and showLastUpdateTime in docs config
- [x] T042 [P] Test build process with all locales
- [x] T043 [P] Verify production build works correctly

## Testing Tasks

- [x] T044 Test landing page in light theme
- [x] T045 Test landing page in dark theme
- [x] T046 Test responsive design on mobile, tablet, desktop
- [x] T047 Test theme toggle functionality
- [x] T048 Test Urdu locale switching
- [x] T049 Test all 21 Urdu translations display correctly
- [x] T050 Test chatbot source links accuracy
- [x] T051 Test sources display at bottom
- [x] T052 Test sources clear on new message

---

**Status**: ✅ All tasks completed
**Last Updated**: 2025-01-02

