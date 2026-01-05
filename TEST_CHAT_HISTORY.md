# Testing Chat History - Step by Step

## Step 1: Clear Old Session

1. Open your browser
2. Press **F12** to open Developer Tools
3. Go to **Application** tab (Chrome/Edge) or **Storage** tab (Firefox)
4. Find **Local Storage** → `http://localhost:3000` (or your domain)
5. Delete the `chatbot_session_id` key if it exists
6. **Close DevTools**

## Step 2: Start Fresh Conversation

1. **Refresh the page** (F5)
2. Open the **chatbot widget** (bottom-right corner)
3. Send a message: `"Hello, what is ROS2?"`
4. Wait for the bot's response

## Step 3: Verify Session Created

1. Open **DevTools** again (F12)
2. Go to **Application** → **Local Storage**
3. You should see `chatbot_session_id` with a new UUID value
4. **Copy this UUID** - you'll need it for verification

## Step 4: Continue Conversation

1. Send another message: `"Tell me more about nodes"`
2. Wait for response
3. Send one more: `"How do I create a node?"`
4. You should now have **3 user messages** and **3 bot responses**

## Step 5: Test Persistence - Close & Reopen

1. **Close the chatbot** (click the X button)
2. **Reopen the chatbot** (click the widget)
3. **Verify:** All 6 messages should still be visible ✅

## Step 6: Test Persistence - Page Refresh

1. **Refresh the entire page** (F5)
2. **Open the chatbot** again
3. **Verify:** All 6 messages should still be there ✅

## Step 7: Verify in Backend Logs

Look at your backend terminal. You should see logs like:

```
[CHAT] Saved user message for session {your-session-id}
[CHAT] Saved assistant message for session {your-session-id}
[CHAT] Updated session {your-session-id}
```

If you see these logs, **messages are being saved correctly** ✅

## Step 8: Test History API Directly

Open a new browser tab and test the API:

```
http://localhost:8000/api/chat/history/{your-session-id}
```

Replace `{your-session-id}` with the UUID from Step 3.

You should see a JSON response with all your messages.

## Step 9: Test Source Links

1. In the chatbot, check if sources appear at the bottom
2. Click on any source link
3. **Verify:** Browser navigates to the book content page ✅
4. **Verify:** URL changes to something like `/docs/module-1/` ✅

## Troubleshooting

### If history doesn't persist:

**Check Backend Logs:**
```
Look for:
- "[CHAT] Saved user message..."
- "[CHAT] Saved assistant message..."

If you DON'T see these, the backend isn't saving messages.
```

**Check Browser Console:**
```
1. Open DevTools (F12)
2. Go to Console tab
3. Look for red errors
4. Common issues:
   - "Session not found" → Old session ID, fixed automatically
   - Network error → Backend not running
   - 500 error → Backend database issue
```

**Verify Collections Exist:**
```bash
cd backend
python -c "
from app.core.database import get_qdrant_client
import asyncio

async def check():
    client = get_qdrant_client()
    collections = await client.get_collections()
    names = [c.name for c in collections.collections]
    print('Collections:', names)
    if 'chat_sessions' in names and 'chat_messages' in names:
        print('✅ Chat collections exist')
    else:
        print('❌ Missing chat collections - run fix_chat_collections.py')

asyncio.run(check())
"
```

### If collections are missing:

```bash
cd backend
uv run python fix_chat_collections.py
```

## Expected Results

After completing all steps, you should have:

- ✅ 6 messages in the chat (3 from you, 3 from bot)
- ✅ Messages persist when closing/reopening chatbot
- ✅ Messages persist after page refresh
- ✅ Session ID saved in localStorage
- ✅ Backend logs showing saved messages
- ✅ API endpoint returning chat history
- ✅ Source links navigate to book content

## Quick Verification Commands

### Test from terminal:

```bash
# Test sending a message
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS2?", "session_id": "test-session-123"}'

# Check if message was saved
curl http://localhost:8000/api/chat/history/test-session-123
```

If both commands return JSON responses, the system is working! ✅

## Success!

If all steps pass, your chat history system is **fully functional**! 🎉

You can now:
- Have conversations that persist across sessions
- Close and reopen the chatbot without losing history
- Refresh the page and continue conversations
- Click source links to read book content
- Start new conversations by clearing localStorage
