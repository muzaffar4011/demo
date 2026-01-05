import re
from pathlib import Path
from typing import List, Dict, Optional
import markdown


class DocumentParser:
    def __init__(self):
        pass

    def parse_document(self, file_path: Path) -> str:
        """
        Parse a document file and extract its text content
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()

        # Different handling based on file extension
        if file_path.suffix.lower() == '.md':
            return self._parse_markdown(content)
        elif file_path.suffix.lower() == '.mdx':
            return self._parse_mdx(content)
        else:
            # For other text files, just return the content
            return content

    def extract_frontmatter(self, file_path: Path) -> Optional[Dict[str, str]]:
        """
        Extract frontmatter (title, sidebar_label, etc.) from markdown file
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()

            if content.startswith('---'):
                parts = content.split('---', 2)
                if len(parts) >= 3:
                    frontmatter_text = parts[1].strip()
                    frontmatter = {}
                    for line in frontmatter_text.split('\n'):
                        if ':' in line:
                            key, value = line.split(':', 1)
                            frontmatter[key.strip()] = value.strip().strip('"').strip("'")
                    return frontmatter
        except Exception as e:
            print(f"Warning: Could not extract frontmatter from {file_path}: {e}")
        return None

    def _parse_markdown(self, content: str) -> str:
        """
        Parse markdown content, extracting text while preserving structure
        """
        # Remove frontmatter if present
        content = self._remove_frontmatter(content)

        # Convert markdown to plain text while preserving structure
        try:
            # For this implementation, we'll just clean up the markdown
            # Remove markdown formatting but keep the text content
            text = self._clean_markdown(content)
            return text
        except:
            # Fallback to basic cleaning
            return self._clean_markdown(content)

    def _parse_mdx(self, content: str) -> str:
        """
        Parse MDX content (Markdown with JSX)
        """
        # Remove JSX components but keep the markdown content
        content = self._remove_frontmatter(content)
        content = self._remove_jsx_components(content)

        # Clean the remaining markdown content
        text = self._clean_markdown(content)
        return text

    def _remove_frontmatter(self, content: str) -> str:
        """
        Remove YAML frontmatter from content
        """
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                return parts[2].strip()
        return content

    def _remove_jsx_components(self, content: str) -> str:
        """
        Remove JSX components from MDX content while preserving text
        """
        # Remove JSX opening and closing tags (simplified approach)
        # This is a basic implementation - in a real app you might want to use a proper parser
        content = re.sub(r'<[^>]*>', '', content)
        return content

    def _clean_markdown(self, content: str) -> str:
        """
        Clean markdown syntax to plain text
        """
        # Remove markdown formatting but preserve the text
        # Headers
        content = re.sub(r'^#+\s+', '', content, flags=re.MULTILINE)
        # Bold and italic
        content = re.sub(r'\*{1,2}([^*]+)\*{1,2}', r'\1', content)
        content = re.sub(r'_{1,2}([^_]+)_{1,2}', r'\1', content)
        # Code blocks
        content = re.sub(r'`{3}[\s\S]*?`{3}', '', content)
        # Inline code
        content = re.sub(r'`([^`]+)`', r'\1', content)
        # Links
        content = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', content)
        # Images
        content = re.sub(r'!\[([^\]]*)\]\([^)]+\)', r'\1', content)
        # Lists
        content = re.sub(r'^\s*[\*\-\+]\s+', '', content, flags=re.MULTILINE)
        content = re.sub(r'^\s*\d+\.\s+', '', content, flags=re.MULTILINE)
        # Blockquotes
        content = re.sub(r'^\s*>\s+', '', content, flags=re.MULTILINE)

        # Clean up extra whitespace
        content = re.sub(r'\n\s+\n', '\n\n', content)
        content = content.strip()

        return content

    def chunk_content(self, content: str, chunk_size: int = 1000, chunk_overlap: int = 200) -> List[str]:
        """
        Split content into chunks of specified size with overlap
        """
        if len(content) <= chunk_size:
            return [content]

        chunks = []
        start = 0

        while start < len(content):
            # Determine the end position
            end = start + chunk_size

            # If we're near the end, include the rest
            if end >= len(content):
                chunks.append(content[start:])
                break

            # Try to break at sentence boundary
            chunk = content[start:end]

            # Find the last sentence boundary within the chunk
            sentence_end = max(
                chunk.rfind('. '),
                chunk.rfind('! '),
                chunk.rfind('? '),
                chunk.rfind('\n'),
                chunk.rfind('.\n'),
                chunk.rfind('!'),
                chunk.rfind('?')
            )

            # If we found a good sentence boundary and it's not too close to the start
            if sentence_end > chunk_size // 2:
                actual_end = start + sentence_end + 1
                chunks.append(content[start:actual_end])
                start = actual_end - chunk_overlap
            else:
                # If no good sentence boundary found, just take the full chunk
                chunks.append(content[start:end])
                start = end - chunk_overlap

        # Remove empty chunks
        chunks = [chunk for chunk in chunks if chunk.strip()]

        return chunks