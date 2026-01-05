# Chatbot Troubleshooting Guide

## Quick Fix: If You See "Index required" Error

If you see this error in your backend logs:
```
Error: Index required but not found for "session_id"
```

**Solution:** Run the fix script to recreate collections with proper indexes:

```bash
cd backend
uv run python fix_chat_collections.py
```

This will:
- Delete old chat collections
- Create new ones with proper configuration
- Add required indexes for filtering

**Note:** This will clear any existing chat history.

---

## Common Issues & Solutions

### Issue 1: "Collection already exists" Error

**Error Message:**
```
Error retrieving session: Unexpected Response: 409 (Conflict)
Collection `chat_messages` already exists!
```

**Cause:** Collections were created without proper indexes.

**Solution:**
```bash
cd backend
uv run python fix_chat_collections.py
```

---

### Issue 2: Source Links Not Working

**Symptoms:**
- Clicking source links doesn't navigate
- Links open in new tab to wrong page
- Stays on landing page

**Solutions:**

1. **Clear browser cache:**
   - Press `Ctrl+Shift+Delete` (Chrome/Edge)
   - Select "Cached images and files"
   - Click "Clear data"

2. **Hard refresh:**
   - Press `Ctrl+Shift+R` (or `Cmd+Shift+R` on Mac)

3. **Check if frontend was rebuilt:**
   ```bash
   cd docusaurus
   npm run build
   ```

4. **Verify URL format in console:**
   - Open DevTools (F12)
   - Click a source link
   - Check console - should see navigation to `/docs/...` (not `/docs/.../index.md`)

---

### Issue 3: Chat History Not Loading

**Symptoms:**
- Messages disappear when reopening chatbot
- Empty chat on page refresh
- "Session not found" error

**Solutions:**

1. **Check if backend is running:**
   ```bash
   cd backend
   uv run uvicorn app.main:app --reload --port 8000
   ```

2. **Check browser console for errors:**
   - Press F12 to open DevTools
   - Look for red error messages
   - Common errors:
     - Network error: Backend not reachable
     - 404 error: Session ID doesn't exist
     - 500 error: Backend database issue

3. **Verify Qdrant is accessible:**
   - Check `.env` file in backend directory
   - Ensure `QDRANT_URL` and `QDRANT_API_KEY` are correct
   - Test connection: Try opening Qdrant dashboard

4. **Clear localStorage and start fresh:**
   - Open DevTools (F12)
   - Go to Application tab → Local Storage
   - Delete `chatbot_session_id` key
   - Refresh page and try again

5. **Re-run collection fix:**
   ```bash
   cd backend
   uv run python fix_chat_collections.py
   ```

---

### Issue 4: 404 "Session not found" Error

**Error Message:**
```
GET /api/chat/history/{session_id} HTTP/1.1" 404 Not Found
```

**Cause:** Session ID in localStorage doesn't exist in database.

**Solution:**
1. Open DevTools (F12)
2. Go to Application → Local Storage → `http://localhost:3000` (or your domain)
3. Find `chatbot_session_id` key
4. Delete it
5. Refresh page
6. New session will be created automatically on first message

---

### Issue 5: Messages Not Persisting After Restart

**Symptoms:**
- Messages save during session
- After closing/reopening chatbot, history is gone

**Solutions:**

1. **Check if session ID is being saved:**
   - Open DevTools (F12) → Application → Local Storage
   - Look for `chatbot_session_id` key
   - Should contain a UUID like: `67b0ca20-c7dc-4880-8ae5-444e5eac8e54`
   - If missing, localStorage might be disabled

2. **Enable localStorage in browser:**
   - Chrome: Settings → Privacy → Cookies → Allow all cookies
   - Check if you're in Incognito/Private mode (localStorage disabled)

3. **Check backend logs:**
   - Look for "Save user message" or "Save assistant message"
   - Should see successful upsert operations
   - If errors appear, check Qdrant connection

---

### Issue 6: Multiple Sessions Not Separating

**Symptoms:**
- Old messages showing up in new conversations
- Can't start fresh chat

**Solution:**
Clear localStorage to start new session:
1. DevTools (F12) → Application → Local Storage
2. Delete `chatbot_session_id`
3. Refresh page
4. Send new message → New session created

---

### Issue 7: Backend Crashes on Startup

**Error in logs:**
```
ModuleNotFoundError: No module named 'app.services.chat_history_service'
```

**Solution:**
Restart backend with proper path:
```bash
cd backend
uv run uvicorn app.main:app --reload --port 8000
```

---

### Issue 8: Source Citations Missing

**Symptoms:**
- Chatbot responds but no sources shown
- Sources section empty or not visible

**Solutions:**

1. **Re-ingest documents with correct URLs:**
   ```bash
   cd backend
   uv run python reingest_docs.py
   ```

2. **Check if Qdrant has documents:**
   - Verify `physical_ai_book` collection exists
   - Should have 117+ chunks from re-ingestion

3. **Check backend logs for RAG errors:**
   - Look for "Error searching in vector database"
   - If found, verify Qdrant connection in `.env`

---

## Verification Checklist

After applying fixes, verify everything works:

- [ ] Backend running without errors on port 8000
- [ ] Frontend accessible at http://localhost:3000
- [ ] Collections exist: `physical_ai_book`, `chat_sessions`, `chat_messages`
- [ ] Index exists on `chat_messages.session_id`
- [ ] Can send message and get response
- [ ] Sources appear at bottom of chatbot
- [ ] Source links navigate to correct pages
- [ ] Close and reopen chatbot → History persists
- [ ] Refresh page → History still there
- [ ] Clear localStorage → New session starts

---

## Debug Commands

### Check Qdrant Collections
```bash
# Using qdrant-client
python -c "
from app.core.database import get_qdrant_client
import asyncio

async def check():
    client = get_qdrant_client()
    collections = await client.get_collections()
    for c in collections.collections:
        print(f'Collection: {c.name}')
        info = await client.get_collection(c.name)
        print(f'  Points: {info.points_count}')

asyncio.run(check())
"
```

### Test Chat History API
```bash
# Get history for a session
curl http://localhost:8000/api/chat/history/YOUR-SESSION-ID

# Send a test message
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Test message"}'
```

### Check Browser Console
```javascript
// In browser DevTools console:

// Check localStorage
console.log('Session ID:', localStorage.getItem('chatbot_session_id'));

// Test API endpoint
fetch('http://localhost:8000/api/chat/history/' + localStorage.getItem('chatbot_session_id'))
  .then(r => r.json())
  .then(console.log);
```

---

## Emergency Reset

If nothing works, do a complete reset:

```bash
# 1. Stop backend (Ctrl+C)

# 2. Fix collections
cd backend
uv run python fix_chat_collections.py

# 3. Re-ingest documents
uv run python reingest_docs.py

# 4. Restart backend
uv run uvicorn app.main:app --reload --port 8000

# 5. Clear browser data
# - DevTools (F12) → Application → Clear Storage → Clear site data

# 6. Rebuild frontend
cd ../docusaurus
npm run build

# 7. Test
# Open http://localhost:3000 and test chatbot
```

---

## Getting Help

If issues persist after trying all solutions:

1. **Check backend logs** - Look for error messages and stack traces
2. **Check browser console** - Look for network errors or JavaScript errors
3. **Verify `.env` configuration** - Ensure all API keys and URLs are correct
4. **Test Qdrant connection** - Make sure Qdrant is accessible
5. **Check file modifications** - Ensure all code changes were applied correctly

**Files to check:**
- `backend/app/services/chat_history_service.py` - Chat history logic
- `backend/app/api/chat.py` - Chat endpoints
- `docusaurus/src/components/Chatbot/Chatbot.tsx` - Frontend chatbot
- `backend/.env` - Configuration

---

## Success Indicators

You'll know everything is working when:

1. ✅ No errors in backend logs
2. ✅ Chatbot opens and shows history (if exists)
3. ✅ Can send messages and get responses
4. ✅ Sources appear and links work
5. ✅ History persists after closing/reopening
6. ✅ History survives page refresh
7. ✅ Can start new session by clearing localStorage
