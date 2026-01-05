# Browser Testing Instructions - Follow These Steps

## Step 1: Clear Old Session (30 seconds)

1. Open your browser to: **http://localhost:3000**
2. Press **F12** to open Developer Tools
3. Click on **Application** tab (Chrome/Edge) or **Storage** tab (Firefox)
4. In the left sidebar, expand **Local Storage**
5. Click on **http://localhost:3000**
6. Find the key: `chatbot_session_id`
7. **Right-click** on it and select **Delete**
8. **Close DevTools** (X button or F12 again)

## Step 2: Refresh and Start New Chat (1 minute)

1. **Refresh the page** (Press F5)
2. Find the **chatbot widget** in the bottom-right corner
3. **Click** to open the chatbot
4. Type a message: **"What is ROS2?"**
5. Press **Enter** or click Send
6. **Wait for the response** (10-20 seconds)
7. You should see:
   - Bot's response about ROS2
   - Sources at the bottom of the chatbot

## Step 3: Send Another Message (1 minute)

1. Type: **"Tell me about nodes"**
2. Press **Enter**
3. **Wait for response**
4. You should now see **4 messages total**:
   - Your first question about ROS2
   - Bot's first response
   - Your second question about nodes
   - Bot's second response

## Step 4: Test Persistence - Close & Reopen (30 seconds)

1. **Close the chatbot** (click the X button in top-right of chatbot)
2. **Wait 2 seconds**
3. **Reopen the chatbot** (click the widget in bottom-right)
4. **VERIFY:** All 4 messages should still be visible ✅
   - If you see all messages → **History is working!** 🎉
   - If chatbot is empty → See troubleshooting below

## Step 5: Test Persistence - Page Refresh (30 seconds)

1. **Keep the chatbot open** with your 4 messages visible
2. **Refresh the entire page** (Press F5)
3. **Wait for page to reload**
4. **Open the chatbot** again
5. **VERIFY:** All 4 messages should still be there ✅
   - If you see all messages → **Persistence works!** 🎉
   - If chatbot is empty → See troubleshooting below

## Step 6: Test Source Links (30 seconds)

1. Look at the **Sources** section at the bottom of the chatbot
2. You should see links like:
   - "ROS2 Foundations"
   - "Introduction"
   - etc.
3. **Click on any source link**
4. **VERIFY:** Browser navigates to the book content page ✅
5. **VERIFY:** URL changes to something like `/docs/module-1/` ✅
   - If it works → **Source links fixed!** 🎉
   - If you stay on same page → See troubleshooting below

## Step 7: Check Backend Logs

Look at your backend terminal window. You should see logs like:

```
[CHAT] Saved user message for session {uuid}
[CHAT] Saved assistant message for session {uuid}
[CHAT] Updated session {uuid}
INFO: 127.0.0.1 - "POST /api/chat HTTP/1.1" 200 OK
```

If you see these logs → **Backend is saving messages correctly!** ✅

---

## Quick Verification Checklist

After completing all steps, check these boxes:

- [ ] Old session cleared from localStorage
- [ ] New conversation started successfully
- [ ] Received responses from chatbot
- [ ] Sources visible at bottom
- [ ] Messages persist when closing/reopening chatbot
- [ ] Messages persist after page refresh
- [ ] Source links navigate to book content
- [ ] Backend logs show saved messages

If ALL boxes checked → **Everything is working perfectly!** 🎉

---

## Troubleshooting

### If messages don't persist:

**Check Browser Console:**
1. Press F12 → Console tab
2. Look for red errors
3. Common issues:
   - "Session not found" → This is normal once, then should work
   - "Failed to fetch" → Backend not running
   - "Network error" → Check backend is on port 8000

**Check localStorage:**
1. F12 → Application → Local Storage
2. You should see: `chatbot_session_id` with a UUID value
3. If missing → Not being saved, check browser console for errors

**Check Backend:**
1. Look at backend terminal
2. Should see: `[CHAT] Saved user message...`
3. If missing → Backend not saving, check for errors

### If source links don't work:

**Clear Browser Cache:**
1. Press Ctrl+Shift+Delete
2. Select "Cached images and files"
3. Click "Clear data"
4. Refresh page (Ctrl+Shift+R)

**Check URL:**
1. Right-click source link → Inspect
2. Check href attribute
3. Should be: `/docs/module-1/` (not `/docs/module-1/index.md`)

---

## Test Results Recording

**Document your test results here:**

Test Date/Time: _______________

| Test | Result | Notes |
|------|--------|-------|
| Clear old session | ✅ / ❌ | |
| Send first message | ✅ / ❌ | |
| Send second message | ✅ / ❌ | |
| Close & reopen - messages persist | ✅ / ❌ | |
| Page refresh - messages persist | ✅ / ❌ | |
| Source links navigate correctly | ✅ / ❌ | |
| Backend logs show saves | ✅ / ❌ | |

**Overall Result:** ✅ PASS / ❌ FAIL

**Issues Encountered:**
_____________________________________
_____________________________________
_____________________________________

---

## Expected Session ID

If you followed the test script, your session ID should be:
**0d862d2e-f254-4367-aecc-28d1037c5347**

You can manually set this in localStorage to see the test messages:
1. F12 → Application → Local Storage
2. Add key: `chatbot_session_id`
3. Value: `0d862d2e-f254-4367-aecc-28d1037c5347`
4. Refresh page
5. Open chatbot → You'll see the 2 test messages from the script!

---

## Need Help?

If any test fails:
1. Check `CHATBOT_TROUBLESHOOTING.md`
2. Verify backend is running on port 8000
3. Check browser console for errors (F12 → Console)
4. Check backend terminal for error logs
5. Try clearing localStorage and starting fresh

**Most common issue:** Old session ID from before we fixed the collections.
**Solution:** Just delete `chatbot_session_id` from localStorage and start fresh!
