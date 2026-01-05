# Data Model: Physical AI & Humanoid Robotics Documentation

## Documentation Page Entity
- **Fields**:
  - id (string): unique identifier for the page
  - title (string): page title for display
  - slug (string): URL-friendly identifier
  - content (string): MDX content of the page
  - module (enum): Module 1-4 or appendix
  - order (integer): position within the module
  - prerequisites (array of strings): IDs of prerequisite pages
  - learningObjectives (array of strings): what the user should learn
  - difficulty (enum): beginner, intermediate, advanced
  - estimatedTime (integer): minutes to complete
  - tags (array of strings): technical tags for search
- **Validation**: Title and content are required, slug must be unique
- **Relationships**: Belongs to one module, may reference other pages

## Module Entity
- **Fields**:
  - id (string): unique identifier (module-1, module-2, etc.)
  - title (string): module title
  - description (string): brief overview of the module
  - pages (array of DocumentationPage): ordered list of pages
  - order (integer): sequence of the module (1-4)
  - prerequisites (array of strings): IDs of prerequisite modules
  - learningOutcomes (array of strings): what user should achieve
- **Validation**: Title is required, order must be 1-4
- **Relationships**: Contains many DocumentationPage entities

## Code Example Entity
- **Fields**:
  - id (string): unique identifier
  - title (string): descriptive title
  - description (string): purpose of the code example
  - code (string): the actual code snippet
  - language (string): programming language identifier
  - explanation (string): line-by-line or section explanations
  - expectedOutput (string): what the code should produce
  - relatedPage (string): ID of DocumentationPage this belongs to
  - downloadable (boolean): whether it's available as a file download
- **Validation**: Code and language are required
- **Relationships**: Belongs to one DocumentationPage

## Visual Aid Entity
- **Fields**:
  - id (string): unique identifier
  - title (string): descriptive title
  - type (enum): diagram, image, chart, video
  - altText (string): accessibility description
  - description (string): what the visual aid explains
  - src (string): path to the asset
  - relatedPage (string): ID of DocumentationPage this belongs to
  - mermaidCode (string): if type is diagram and using Mermaid
- **Validation**: Alt text is required for accessibility (WCAG compliance)
- **Relationships**: Belongs to one DocumentationPage

## Learning Path Entity
- **Fields**:
  - id (string): unique identifier
  - name (string): name of the learning path
  - description (string): what the path covers
  - pages (array of strings): ordered list of page IDs
  - estimatedDuration (integer): total time in minutes
  - targetAudience (string): who the path is for
- **Validation**: Must have at least one page
- **Relationships**: References many DocumentationPage entities

## User Assessment Entity
- **Fields**:
  - id (string): unique identifier
  - relatedPage (string): ID of DocumentationPage this belongs to
  - question (string): assessment question
  - type (enum): multiple-choice, short-answer, practical
  - options (array of strings): for multiple-choice questions
  - correctAnswer (string): the correct answer
  - explanation (string): why this is the correct answer
- **Validation**: Question and correct answer are required
- **Relationships**: Belongs to one DocumentationPage