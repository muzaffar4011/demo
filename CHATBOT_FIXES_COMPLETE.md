# Chatbot Complete Fixes - Source Links & Chat History

## Issues Fixed

### 1. Source Links Not Navigating to Book Content
**Problem:** When clicking on source links in the chatbot, they remained on the landing page instead of navigating to the actual book content.

**Root Cause:** Links had `target="_blank"` which was opening in new tabs, and the relative URLs weren't being handled properly by the browser.

**Solution:** Changed the link behavior to use `window.location.href` for internal navigation instead of opening in new tabs.

### 2. Chat History Not Persisting
**Problem:** When reopening the chatbot, previous conversations were lost. Chat history wasn't being saved to the database.

**Root Cause:** No chat history persistence layer was implemented. Messages were only stored in React state.

**Solution:** Implemented complete chat history system with:
- Database storage using Qdrant collections
- Session management
- Message persistence
- Automatic history loading on chatbot open

---

## Files Modified

### Backend Changes

#### 1. New File: `backend/app/services/chat_history_service.py`
Created new service to manage chat sessions and messages:
- `ChatHistoryService` class with methods:
  - `save_session()` - Save/update chat sessions
  - `get_session()` - Retrieve session by ID
  - `save_message()` - Save individual messages
  - `get_session_messages()` - Get all messages for a session
- Uses Qdrant collections as key-value store for persistence

#### 2. Modified: `backend/app/api/chat.py`
Enhanced chat endpoint with history persistence:
- Import chat history service and models
- Create/retrieve session on each chat request
- Save user messages before processing
- Save assistant responses after generation
- Added new endpoint: `GET /api/chat/history/{session_id}` to retrieve chat history

**New Models Added:**
```python
class ChatHistoryMessage(BaseModel):
    id: str
    role: str
    content: str
    sources: Optional[List[Source]] = []
    created_at: str
    latency: Optional[float] = None

class ChatHistoryResponse(BaseModel):
    session_id: str
    messages: List[ChatHistoryMessage]
```

### Frontend Changes

#### Modified: `docusaurus/src/components/Chatbot/Chatbot.tsx`

**1. Fixed Source Links (lines 308-337):**
```typescript
// Before:
<a href={url} target="_blank" rel="noopener noreferrer">

// After:
<a
  href={url}
  onClick={(e) => {
    e.preventDefault();
    window.location.href = url;
  }}
>
```

**2. Added Chat History Loading (lines 139-176):**
- New `useEffect` hook to load history when chatbot opens
- New `loadChatHistory()` function that:
  - Fetches history from backend API
  - Converts to frontend message format
  - Restores sources from last assistant message

---

## How It Works

### Chat History Flow

1. **First Message:**
   - User opens chatbot and sends first message
   - Backend creates new session with unique ID
   - Session ID saved to localStorage
   - User and assistant messages saved to Qdrant

2. **Subsequent Messages:**
   - Session ID retrieved from localStorage
   - Messages associated with existing session
   - All messages persisted to database

3. **Reopening Chatbot:**
   - Frontend checks for session ID in localStorage
   - Calls `/api/chat/history/{session_id}` endpoint
   - Backend retrieves all messages from Qdrant
   - Frontend displays full conversation history

### Source Link Navigation

1. User clicks on source link
2. `onClick` handler prevents default behavior
3. `window.location.href` navigates to internal URL
4. Browser loads the book content page
5. User can read the source, then return to continue chatting

---

## Testing Instructions

### Test Source Link Navigation

1. Open your application (http://localhost:3000)
2. Click on chatbot widget
3. Ask: "What is ROS2?" or any book-related question
4. Wait for response with sources
5. Click on any source link at the bottom
6. **Verify:** Browser navigates to the actual book content page (not staying on landing page)
7. **Verify:** URL changes to something like `/docs/module-1/`

### Test Chat History Persistence

1. Open chatbot and send a message: "Hello, what is ROS2?"
2. Wait for bot response
3. Send another message: "Tell me more about nodes"
4. **Close the chatbot** (click X button)
5. **Reopen the chatbot** (click the widget)
6. **Verify:** All previous messages are still visible
7. **Verify:** You can continue the conversation
8. **Refresh the entire page** (F5)
9. Open chatbot again
10. **Verify:** Chat history is still preserved

### Test Multiple Sessions

1. Open browser DevTools (F12) → Application → Local Storage
2. Delete `chatbot_session_id` key
3. Send a new message
4. **Verify:** New session started (new messages, no old history)
5. Close and reopen chatbot
6. **Verify:** Only new session messages visible

---

## Database Schema

### Chat Sessions Collection (`chat_sessions`)
```json
{
  "id": "uuid",
  "user_id": null,
  "title": "First message preview...",
  "created_at": "2026-01-03T...",
  "updated_at": "2026-01-03T...",
  "is_active": true
}
```

### Chat Messages Collection (`chat_messages`)
```json
{
  "id": "uuid",
  "session_id": "uuid",
  "role": "user" | "assistant",
  "content": "Message text...",
  "context": null,
  "sources": [{...}],
  "created_at": "2026-01-03T...",
  "latency": 1.23
}
```

---

## API Endpoints

### POST /api/chat
Send a message and get a response (unchanged behavior, now saves to DB)

**Request:**
```json
{
  "message": "What is ROS2?",
  "session_id": "optional-uuid",
  "context": null,
  "llm_config": {}
}
```

**Response:**
```json
{
  "response": "ROS2 is...",
  "sources": [...],
  "session_id": "uuid",
  "latency": 1.23
}
```

### GET /api/chat/history/{session_id}
Retrieve chat history for a session (NEW)

**Response:**
```json
{
  "session_id": "uuid",
  "messages": [
    {
      "id": "uuid",
      "role": "user",
      "content": "What is ROS2?",
      "sources": [],
      "created_at": "2026-01-03T...",
      "latency": null
    },
    {
      "id": "uuid",
      "role": "assistant",
      "content": "ROS2 is...",
      "sources": [{...}],
      "created_at": "2026-01-03T...",
      "latency": 1.23
    }
  ]
}
```

---

## Troubleshooting

### Issue: Source links still not working
**Check:**
1. Clear browser cache (Ctrl+Shift+Delete)
2. Hard refresh (Ctrl+Shift+R)
3. Verify URL format in browser console (should be `/docs/...`, not `/docs/.../index.md`)

### Issue: Chat history not loading
**Check:**
1. Backend is running: `cd backend && uv run uvicorn app.main:app --reload`
2. Check browser console for errors
3. Verify Qdrant is accessible (check `.env` file)
4. Check backend logs for error messages

### Issue: Chat history from wrong session
**Solution:**
1. Open DevTools → Application → Local Storage
2. Delete `chatbot_session_id`
3. Refresh page
4. Start new conversation

### Issue: "Session not found" error
**Cause:** Session ID in localStorage doesn't exist in database
**Solution:**
1. Clear localStorage (delete `chatbot_session_id`)
2. Refresh page
3. New session will be created automatically

---

## Benefits

### For Users:
- ✅ Seamless navigation to source content
- ✅ Conversation history preserved across sessions
- ✅ Can continue conversations later
- ✅ Better research workflow

### For Development:
- ✅ Clean separation of concerns (history service)
- ✅ Scalable database design
- ✅ Easy to add features (multi-user, search history, etc.)
- ✅ Full audit trail of all conversations

---

## Next Steps (Optional Enhancements)

1. **Add "New Chat" button** - Let users start fresh conversations
2. **Chat history sidebar** - Show list of previous sessions
3. **Search chat history** - Find old conversations
4. **Export conversations** - Download as PDF/text
5. **Multi-user support** - Add authentication and user-specific history
6. **Message editing** - Let users edit their messages
7. **Conversation titles** - Auto-generate meaningful titles
8. **Delete conversations** - Let users remove old chats

---

## Files Summary

**New Files:**
- `backend/app/services/chat_history_service.py` (108 lines)

**Modified Files:**
- `backend/app/api/chat.py` (+100 lines)
- `docusaurus/src/components/Chatbot/Chatbot.tsx` (+44 lines, modified link handler)

**Total Changes:** ~252 lines of new code

---

## Conclusion

Both issues are now completely resolved:

1. ✅ **Source links navigate correctly** - Users can click and read the actual book content
2. ✅ **Chat history persists** - Conversations are saved and restored automatically

The chatbot now provides a seamless experience with persistent conversations and working source citations!
