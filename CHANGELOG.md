# Changelog

All notable changes to the Physical AI & Humanoid Robotics documentation site will be documented in this file.

## [1.1.0] - 2025-01-02

### Enhanced User Interface & Experience

This release focuses on significant UI/UX improvements to provide a more modern, polished, and user-friendly experience across the documentation site.

#### Landing Page Improvements

- **Modern Hero Section**
  - Implemented animated gradient background with floating orbs for visual depth
  - Enhanced typography with gradient text effects for the main title
  - Added comprehensive hero description to better communicate value proposition
  - Introduced dual call-to-action buttons with smooth hover animations
  - Added statistics section showcasing key learning metrics (4 modules, 20+ projects, 100% practical learning)
  - Improved responsive behavior for mobile and tablet devices

- **Feature Cards Redesign**
  - Redesigned feature cards with modern card-based layout
  - Implemented gradient icon backgrounds for visual distinction
  - Updated content to be more specific to robotics curriculum
  - Added smooth hover animations with elevation effects
  - Optimized card sizing for better content density
  - Improved spacing and alignment using CSS Grid layout
  - Enhanced visual hierarchy with refined typography

#### Chatbot UI Enhancements

- **Modernized Design**
  - Simplified and refined chatbot interface for better usability
  - Reduced visual clutter while maintaining functionality
  - Improved button sizing and spacing for better touch targets
  - Enhanced header design with icon and subtitle
  - Refined message bubbles with better contrast and readability
  - Optimized container dimensions (350x450px) for better screen real estate usage

- **Speech Recognition Integration**
  - Integrated Web Speech API for voice input functionality
  - Added microphone button with visual feedback during recording
  - Implemented real-time speech-to-text transcription
  - Added browser compatibility detection
  - Enhanced error handling for microphone permissions
  - Visual indicators for active listening state

- **Improved User Experience**
  - Auto-resizing textarea for better input handling
  - Enhanced context preview with remove functionality
  - Improved loading states with spinner animations
  - Better message animations for smoother interactions
  - Refined scrollbar styling for modern appearance
  - Enhanced accessibility with proper ARIA labels

#### Responsive Design

- **Grid Layout System**
  - Implemented CSS Grid for reliable responsive layouts
  - Optimized for 2 cards per row on large screens (≥997px)
  - Maintains 2 cards per row on tablets (768px-996px)
  - Single column layout on mobile devices (<768px)
  - Consistent spacing across all breakpoints

- **Mobile Optimization**
  - Improved touch targets for mobile interaction
  - Optimized chatbot widget for mobile screens
  - Enhanced card readability on smaller screens
  - Better spacing and padding for mobile devices

#### Dark Mode Support

- **Comprehensive Dark Theme**
  - Full dark mode support across all components
  - Proper color contrast for accessibility
  - Smooth theme transitions
  - Dark mode optimized scrollbars
  - Enhanced readability in dark theme
  - Consistent theming across landing page, features, and chatbot

#### Performance & Accessibility

- **Performance Improvements**
  - Optimized CSS animations for better performance
  - Reduced layout shifts with proper sizing
  - Improved font rendering with anti-aliasing
  - Better scroll performance

- **Accessibility Enhancements**
  - Added proper ARIA labels to interactive elements
  - Improved keyboard navigation
  - Enhanced focus indicators
  - Better color contrast ratios
  - Screen reader friendly structure

### Technical Details

- **CSS Architecture**: Migrated to CSS Grid for more reliable layouts
- **Animation Framework**: Implemented smooth transitions using CSS cubic-bezier functions
- **Browser Support**: Full support for modern browsers with graceful degradation
- **Component Structure**: Improved component organization and reusability

### Bug Fixes

- Fixed responsive grid layout issues
- Resolved spacing inconsistencies between cards
- Fixed dark mode color inconsistencies
- Improved cross-browser compatibility

---

## [1.0.0] - 2024-12-18

### Initial Release

- RAG Chatbot implementation with OpenRouter API integration
- Qwen3 embedding models for semantic search
- Qdrant Cloud vector database integration
- Basic chatbot widget with text input
- Context selection from page content
- FastAPI backend with LangChain RAG pipeline

