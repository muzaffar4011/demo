# Quick Browser Test - Your Current Setup

## Your Frontend URL:
**http://localhost:3002/physical-ai-humanoid-robotics/**

(Port may vary: 3000, 3001, or 3002 depending on what's available)

---

## Quick Test (2 Minutes)

### Step 1: Clear Old Session
1. Open: **http://localhost:3002/physical-ai-humanoid-robotics/**
2. Press **F12** (Developer Tools)
3. Click **Application** tab
4. Expand **Local Storage** in left sidebar
5. Click on **http://localhost:3002**
6. Find `chatbot_session_id` key
7. **Right-click → Delete** (if exists)
8. **Close DevTools** (press F12 again)

### Step 2: Start New Chat
1. **Refresh page** (press F5)
2. **Open chatbot** (widget in bottom-right corner)
3. **Type**: "What is ROS2?"
4. **Press Enter**
5. **Wait** for response (10-20 seconds)

### Step 3: Test Persistence
1. **Close chatbot** (click X button)
2. **Wait 2 seconds**
3. **Reopen chatbot** (click widget)
4. **CHECK**: Your message and bot's response should still be there! ✅

### Step 4: Test Page Refresh
1. **Refresh entire page** (F5)
2. **Open chatbot** again
3. **CHECK**: Messages should STILL be there! ✅

### Step 5: Test Source Links
1. Look at **Sources** section at bottom of chatbot
2. **Click** any source link
3. **CHECK**: Browser navigates to book content (e.g., `/physical-ai-humanoid-robotics/docs/module-1/`) ✅

---

## Alternative: Use Test Session

Want to see existing test messages immediately?

1. **F12** → Application → Local Storage
2. **Add key**: `chatbot_session_id`
3. **Value**: `0d862d2e-f254-4367-aecc-28d1037c5347`
4. **Close DevTools**
5. **Refresh page** (F5)
6. **Open chatbot**
7. You'll see 2 test messages about ROS2 and nodes!

---

## Troubleshooting

### If chatbot doesn't appear:
- Check bottom-right corner of the page
- Try scrolling down
- Check browser console (F12 → Console) for errors

### If messages don't persist:
1. Check browser console for errors
2. Verify localStorage shows `chatbot_session_id`
3. Check backend logs (should see "[CHAT] Saved user message...")

### If wrong port:
Check which port your frontend is actually on:
- Try: http://localhost:3000/physical-ai-humanoid-robotics/
- Try: http://localhost:3001/physical-ai-humanoid-robotics/
- Try: http://localhost:3002/physical-ai-humanoid-robotics/

---

## What to Look For

**Success indicators:**
- ✅ Chatbot opens and responds
- ✅ Sources appear at bottom
- ✅ Messages persist when closing/reopening
- ✅ Messages persist after page refresh
- ✅ Source links navigate to content
- ✅ Backend logs show "[CHAT] Saved user message..."

**You have 8 messages already saved in the database, so the system is working!**

---

## Current System Status

```
Backend: Running on port 8000 ✅
Frontend: Running on port 3002 ✅
Database:
  - 3 chat sessions stored ✅
  - 8 messages saved ✅
  - 117 book content chunks ✅
```

Everything is ready - just test it! 🚀
