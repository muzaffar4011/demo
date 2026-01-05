# Quick Start: Deploy Backend to Render

## 🚀 Fast Deployment Steps

### 1. Create Render Account
- Go to [render.com](https://render.com) and sign up (free)

### 2. Deploy from GitHub
1. In Render dashboard, click **"New +"** → **"Web Service"**
2. Connect your GitHub repo: `muzaffar4011/demo`
3. Use these settings:
   - **Name**: `rag-chatbot-backend`
   - **Environment**: `Python 3`
   - **Build Command**: `pip install -r backend/requirements.txt`
   - **Start Command**: `cd backend && uvicorn app.main:app --host 0.0.0.0 --port $PORT`

### 3. Add Environment Variables

**⚠️ IMPORTANT: Add these in Render dashboard, NOT GitHub!**

1. In Render dashboard, go to your service
2. Click **"Environment"** tab (left sidebar)
3. Click **"Add Environment Variable"** button
4. Add each variable:

**Required Variables:**
- **Key**: `OPENROUTER_API_KEY` → **Value**: Your OpenRouter API key
- **Key**: `QDRANT_URL` → **Value**: Your Qdrant Cloud URL
- **Key**: `QDRANT_API_KEY` → **Value**: Your Qdrant API key

**Optional (has defaults):**
- **Key**: `ALLOWED_ORIGINS` → **Value**: `https://muzaffar4011.github.io`

### 4. Deploy!
Click **"Create Web Service"** and wait 2-5 minutes.

### 5. Get Your Backend URL
After deployment, you'll get a URL like:
```
https://rag-chatbot-backend.onrender.com
```

### 6. Update Frontend (Optional)
The frontend is already configured to use `https://rag-chatbot-backend.onrender.com` by default.

If your Render service has a different name, update `docusaurus/docusaurus.config.ts`:
```typescript
innerHTML: `window.CHATBOT_API_URL = "https://your-service-name.onrender.com";`,
```

### 7. Ingest Documents
After deployment, ingest your docs:
```bash
curl -X POST "https://your-backend-url.onrender.com/api/ingest" \
  -H "Content-Type: application/json" \
  -d '{
    "source": "../docusaurus/docs",
    "recursive": true,
    "filters": [".md", ".mdx"]
  }'
```

**Note**: You need to run this from a machine with access to your docs folder, or use Render Shell.

### 8. Test!
- Visit: `https://muzaffar4011.github.io/demo/`
- The chatbot should work! 🎉

## 📝 Full Documentation
See `RENDER_DEPLOYMENT.md` for detailed instructions and troubleshooting.

## ⚠️ Important Notes

- **Free Tier**: Service spins down after 15 min inactivity (first request may be slow)
- **CORS**: Already configured for your GitHub Pages domain
- **Environment Variables**: Must be set in Render dashboard
- **Documents**: Need to be ingested after first deployment

