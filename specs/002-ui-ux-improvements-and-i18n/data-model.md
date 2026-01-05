# Data Model: UI/UX Improvements & Internationalization

## Locale Configuration Entity
- **Fields**:
  - locale (string): Language identifier (en, ur)
  - label (string): Display name for the locale
  - direction (enum): Text direction (ltr, rtl)
  - htmlLang (string): HTML lang attribute value
  - calendar (string): Calendar system (gregory)
- **Validation**: Locale must be unique, direction must be ltr or rtl
- **Relationships**: Contains many TranslatedContent entities

## TranslatedContent Entity
- **Fields**:
  - id (string): Unique identifier
  - locale (string): Language of the content
  - file_path (string): Path to translated file
  - original_path (string): Path to original English file
  - content (string): Translated content
  - metadata (object): Translation metadata (translator, date, etc.)
- **Validation**: locale and file_path are required
- **Relationships**: Belongs to one LocaleConfiguration, references one OriginalDocument

## SourceLink Entity
- **Fields**:
  - url (string): Docusaurus documentation URL
  - title (string): Document title
  - relevance_score (float): Relevance score (0.0-1.0)
  - document_id (string): Reference to source document
- **Validation**: url and title are required, relevance_score must be 0.0-1.0
- **Relationships**: Belongs to one ChatResponse

## ChatResponse Entity
- **Fields**:
  - response (string): Bot response text
  - sources (array of SourceLink): Array of source references
  - session_id (string): Chat session identifier
  - latency (float): Response time in seconds
- **Validation**: response and sources are required
- **Relationships**: Contains many SourceLink entities

## ThemeState Entity
- **Fields**:
  - current_theme (enum): Current theme (light, dark)
  - stored_preference (string): User's stored preference
  - system_preference (string): System preference from media query
- **Validation**: current_theme must be light or dark
- **Relationships**: Singleton entity for application state

