# Render Deployment Guide

This guide will help you deploy the RAG Chatbot backend to Render.

## Prerequisites

1. A [Render](https://render.com) account (free tier works)
2. Your OpenRouter API key
3. Your Qdrant Cloud credentials (URL and API key)

## Step 1: Create a New Web Service on Render

1. Go to [Render Dashboard](https://dashboard.render.com)
2. Click **"New +"** → **"Web Service"**
3. Connect your GitHub repository: `muzaffar4011/demo`
4. Configure the service:
   - **Name**: `rag-chatbot-backend` (or any name you prefer)
   - **Region**: Choose the closest region to your users
   - **Branch**: `main`
   - **Root Directory**: Leave empty (we'll handle this in the build command)
   - **Environment**: `Python 3`
   - **Build Command**: `pip install -r backend/requirements.txt`
   - **Start Command**: `cd backend && uvicorn app.main:app --host 0.0.0.0 --port $PORT`

## Step 2: Configure Environment Variables

In the Render dashboard, go to your service → **Environment** tab and add these variables:

### Required Variables:

```
OPENROUTER_API_KEY=your_openrouter_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

### Optional Variables (with defaults):

```
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
QDRANT_COLLECTION_NAME=physical_ai_book
EMBEDDING_MODEL=openai/text-embedding-ada-002
LLM_MODEL=qwen/qwen3-max
RATE_LIMIT_REQUESTS=10
RATE_LIMIT_WINDOW=60
DEFAULT_CHUNK_SIZE=1000
DEFAULT_CHUNK_OVERLAP=200
BACKEND_API_KEY=your_optional_api_key_for_auth
ALLOWED_ORIGINS=https://muzaffar4011.github.io
```

### How to Get Your Keys:

#### OpenRouter API Key:
1. Go to [OpenRouter](https://openrouter.ai/)
2. Sign up or log in
3. Navigate to **Keys** section
4. Create a new API key
5. Copy the key and paste it in Render

#### Qdrant Cloud Credentials:
1. Go to [Qdrant Cloud](https://cloud.qdrant.io/)
2. Sign up or log in
3. Create a new cluster (Free tier works)
4. Copy the **Cluster URL** and **API Key**
5. Paste them in Render

## Step 3: Deploy

1. Click **"Create Web Service"**
2. Render will automatically:
   - Clone your repository
   - Install dependencies
   - Start your backend
3. Wait for the deployment to complete (usually 2-5 minutes)

## Step 4: Get Your Backend URL

Once deployed, Render will provide you with a URL like:
```
https://rag-chatbot-backend.onrender.com
```

**Note**: On the free tier, your service will spin down after 15 minutes of inactivity. The first request after spin-down may take 30-60 seconds to respond.

## Step 5: Update Frontend Configuration

Update your Docusaurus frontend to use the Render backend URL:

1. In your GitHub Pages deployment, the frontend will automatically use the environment variable
2. Or update `docusaurus/docusaurus.config.ts` to set the backend URL:

```typescript
headTags: [
  {
    tagName: 'script',
    attributes: {
      type: 'text/javascript',
    },
    innerHTML: `window.CHATBOT_API_URL = "${process.env.CHATBOT_API_URL || 'https://your-backend-name.onrender.com'}";`,
  },
],
```

## Step 6: Ingest Documents

After deployment, you need to ingest your documents into Qdrant:

1. Get your backend URL from Render
2. Send a POST request to ingest documents:

```bash
curl -X POST "https://your-backend-name.onrender.com/api/ingest" \
  -H "Content-Type: application/json" \
  -d '{
    "source": "../docusaurus/docs",
    "recursive": true,
    "filters": [".md", ".mdx"],
    "chunk_size": 1000,
    "chunk_overlap": 200
  }'
```

**Note**: Since the backend is running on Render, you'll need to either:
- Run this from a machine that has access to your local `docusaurus/docs` folder
- Or upload your docs to a cloud storage and provide that URL
- Or use the Render shell to run the ingestion script

### Alternative: Use Render Shell

1. In Render dashboard, go to your service
2. Click **"Shell"** tab
3. Run:
```bash
cd backend
python reingest_docs.py
```

## Step 7: Test Your Deployment

1. Visit your backend health endpoint:
   ```
   https://your-backend-name.onrender.com/api/health
   ```

2. Test the chat endpoint:
   ```bash
   curl -X POST "https://your-backend-name.onrender.com/api/chat" \
     -H "Content-Type: application/json" \
     -d '{
       "message": "What is ROS2?",
       "conversation_id": "test-123"
     }'
   ```

3. Visit your frontend at:
   ```
   https://muzaffar4011.github.io/demo/
   ```

4. Test the chatbot widget - it should connect to your Render backend!

## Troubleshooting

### Service Won't Start

- Check the **Logs** tab in Render dashboard
- Verify all required environment variables are set
- Ensure `requirements.txt` has all dependencies

### CORS Errors

- Make sure `ALLOWED_ORIGINS` includes your GitHub Pages URL
- Check that your frontend is using the correct backend URL

### Slow First Request

- This is normal on Render free tier (cold start)
- Consider upgrading to a paid plan for always-on service

### Documents Not Ingesting

- Verify Qdrant credentials are correct
- Check that your Qdrant cluster is running
- Review backend logs for errors

## Upgrading to Always-On (Optional)

To prevent your service from spinning down:

1. Go to your service settings
2. Enable **"Auto-Deploy"** (already enabled by default)
3. For always-on service, upgrade to a paid plan ($7/month)

## Monitoring

- **Logs**: View real-time logs in the Render dashboard
- **Metrics**: Monitor CPU, memory, and request metrics
- **Alerts**: Set up email alerts for service issues

## Security Recommendations

1. **Set BACKEND_API_KEY**: Add authentication to protect your API
2. **Use HTTPS**: Render provides HTTPS by default
3. **Rate Limiting**: Already configured (10 requests/minute by default)
4. **CORS**: Only allow your frontend domain

## Next Steps

- Set up automatic deployments from your main branch
- Configure custom domain (optional)
- Set up monitoring and alerts
- Consider adding API authentication for production use

