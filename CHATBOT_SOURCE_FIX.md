# Chatbot Source Citation Fix

## Problem
The chatbot was not providing accurate source citations from the book. URLs were pointing to incorrect paths (e.g., `/docs/module-1/index.md` instead of `/docs/module-1/`) and titles were not user-friendly.

## Solution
Fixed the source citation logic in the backend to:
1. **Generate correct Docusaurus URLs** - Remove `.md`/`.mdx` extensions and handle `index` files properly
2. **Extract proper titles** - Read titles from markdown frontmatter instead of using filenames
3. **Improve URL formatting** - Convert backslashes to forward slashes and ensure proper path structure

## Changes Made

### 1. Fixed URL Generation (backend/app/services/document_service.py:89-110)
- Removes file extensions (`.md`, `.mdx`) from URLs
- Handles `index.md` files correctly (e.g., `module-1/index` → `module-1/`)
- Ensures URLs start with `/docs/`
- Converts Windows backslashes to forward slashes

### 2. Enhanced Title Extraction (backend/app/services/document_service.py:65-74)
- Extracts titles from markdown frontmatter
- Falls back to filename if frontmatter is not available
- Checks multiple frontmatter fields: `title`, `sidebar_label`, `id`

### 3. Added Frontmatter Parser (backend/app/utils/document_parser.py:27-47)
- New method `extract_frontmatter()` to parse YAML frontmatter
- Safely handles missing or malformed frontmatter

## How to Apply the Fix

### Step 1: Ensure Backend is Running
```bash
cd backend
uv run uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### Step 2: Re-ingest Documents
Run the re-ingestion script to update the vector database with correct URLs and titles:

```bash
cd backend
uv run python reingest_docs.py
```

Or manually via the API:
```bash
curl -X POST "http://localhost:8000/api/ingest" \
  -H "Content-Type: application/json" \
  -d '{
    "source": "../docusaurus/docs",
    "recursive": true,
    "filters": [".md", ".mdx"],
    "chunk_size": 1000,
    "chunk_overlap": 200
  }'
```

### Step 3: Test the Chatbot
1. Open your application in the browser
2. Click on the chatbot widget
3. Ask a question related to your book content (e.g., "What is ROS2?")
4. Check that the sources at the bottom show:
   - Proper titles (e.g., "ROS2 Foundations" instead of "index")
   - Clickable URLs that work (e.g., `/docs/module-1/` instead of `/docs/module-1/index.md`)

## Example

### Before:
```json
{
  "title": "index",
  "url": "/docs/module-1/index.md",
  "relevance_score": 0.85
}
```

### After:
```json
{
  "title": "ROS2 Foundations",
  "url": "/docs/module-1/",
  "relevance_score": 0.85
}
```

## Verification Checklist
- [ ] Backend is running without errors
- [ ] Documents re-ingested successfully
- [ ] Chatbot displays sources at the bottom
- [ ] Source titles are readable (not filenames)
- [ ] Source URLs are clickable and navigate to correct pages
- [ ] URLs don't have file extensions (`.md`, `.mdx`)

## Troubleshooting

### Issue: "Collection not found" error
**Solution:** The collection needs to be created first. Run the re-ingestion script or call the `/api/ingest` endpoint.

### Issue: Sources still showing incorrect URLs
**Solution:** Make sure you re-ingested the documents AFTER applying the code changes. The old data needs to be replaced.

### Issue: No sources showing at all
**Solution:**
1. Check if Qdrant is running and accessible
2. Verify your `.env` file has correct `QDRANT_URL` and `QDRANT_API_KEY`
3. Check backend logs for errors

### Issue: Titles are still showing as filenames
**Solution:** Your markdown files might not have frontmatter. Add frontmatter to your docs:
```markdown
---
title: Your Page Title
---

Your content here...
```

## Files Modified
1. `backend/app/services/document_service.py` - Fixed URL generation and title extraction
2. `backend/app/utils/document_parser.py` - Added frontmatter extraction
3. `backend/reingest_docs.py` - Created re-ingestion helper script (new file)

## Need Help?
If you encounter any issues, check the backend logs:
```bash
cd backend
uv run uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

The logs will show detailed error messages and processing information.
