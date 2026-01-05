# Environment Variables Setup Guide

## 🎯 Where to Add Environment Variables

### For Render Backend Deployment (Main Setup)

**Environment variables go in Render dashboard, NOT GitHub!**

#### Step-by-Step:

1. **Deploy your backend on Render** (if not done yet):
   - Go to [render.com](https://render.com)
   - Create a new Web Service
   - Connect your GitHub repo

2. **Add Environment Variables in Render**:
   - In Render dashboard, go to your service
   - Click on **"Environment"** tab (in the left sidebar)
   - Click **"Add Environment Variable"** button
   - Add each variable one by one:

#### Required Variables for Render:

```
OPENROUTER_API_KEY
```
- **Value**: Your OpenRouter API key
- **How to get**: [OpenRouter Dashboard](https://openrouter.ai/keys) → Create new key

```
QDRANT_URL
```
- **Value**: Your Qdrant Cloud cluster URL
- **How to get**: [Qdrant Cloud](https://cloud.qdrant.io/) → Your cluster → Copy URL

```
QDRANT_API_KEY
```
- **Value**: Your Qdrant Cloud API key
- **How to get**: [Qdrant Cloud](https://cloud.qdrant.io/) → Your cluster → Copy API Key

#### Optional Variables (with defaults):

```
ALLOWED_ORIGINS
```
- **Value**: `https://muzaffar4011.github.io`
- **Purpose**: Allows your GitHub Pages frontend to access the backend

```
BACKEND_API_KEY
```
- **Value**: (Optional) A secret key for API authentication
- **Purpose**: Protect your API endpoints (leave empty for public access)

#### Visual Guide:

```
Render Dashboard
├── Your Service (rag-chatbot-backend)
    ├── Environment Tab ← CLICK HERE
        ├── Add Environment Variable
            ├── Key: OPENROUTER_API_KEY
            ├── Value: sk-or-v1-xxxxx
            └── Save
        ├── Add Environment Variable
            ├── Key: QDRANT_URL
            ├── Value: https://xxxxx.qdrant.io
            └── Save
        └── ... (repeat for each variable)
```

---

### For GitHub Actions (Frontend - Optional)

If you need to set environment variables for GitHub Actions (for frontend builds), you can add them in GitHub:

#### Step-by-Step:

1. Go to your GitHub repository: `https://github.com/muzaffar4011/demo`

2. Click **"Settings"** (top menu)

3. In left sidebar, click **"Secrets and variables"** → **"Actions"**

4. Click **"New repository secret"** or **"Variables"** tab

5. Add variables if needed (usually not required for this project)

#### When You Might Need GitHub Variables:

- If you want to override the backend URL during build
- If you have build-time secrets
- For this project, you typically **don't need** GitHub variables because:
  - The backend URL is hardcoded in `docusaurus.config.ts`
  - Or you can set it via Render's public URL

---

## 📋 Complete Variable List

### Render Backend (Required):

| Variable | Description | Where to Get |
|----------|-------------|--------------|
| `OPENROUTER_API_KEY` | Your OpenRouter API key | [OpenRouter](https://openrouter.ai/keys) |
| `QDRANT_URL` | Qdrant Cloud cluster URL | [Qdrant Cloud](https://cloud.qdrant.io/) |
| `QDRANT_API_KEY` | Qdrant Cloud API key | [Qdrant Cloud](https://cloud.qdrant.io/) |

### Render Backend (Optional):

| Variable | Default Value | Purpose |
|----------|---------------|---------|
| `ALLOWED_ORIGINS` | - | CORS allowed origins (comma-separated) |
| `BACKEND_API_KEY` | - | API authentication key |
| `OPENROUTER_BASE_URL` | `https://openrouter.ai/api/v1` | OpenRouter API base URL |
| `QDRANT_COLLECTION_NAME` | `physical_ai_book` | Vector database collection name |
| `EMBEDDING_MODEL` | `openai/text-embedding-ada-002` | Embedding model name |
| `LLM_MODEL` | `qwen/qwen3-max` | LLM model name |
| `RATE_LIMIT_REQUESTS` | `10` | Requests per minute |
| `RATE_LIMIT_WINDOW` | `60` | Rate limit window in seconds |

---

## 🔍 How to Verify Variables Are Set

### In Render:

1. Go to your service → **Environment** tab
2. You should see all your variables listed
3. Values are hidden (showing as `••••••`)

### Test Your Backend:

```bash
# Test health endpoint
curl https://your-backend-name.onrender.com/api/health

# Should return: {"status": "healthy", ...}
```

---

## ⚠️ Important Notes

1. **Never commit secrets to GitHub!**
   - Don't add `.env` files to git
   - Don't hardcode API keys in code
   - Use environment variables in Render

2. **Render vs GitHub:**
   - **Render**: For backend environment variables ✅
   - **GitHub**: Usually NOT needed for this setup

3. **Security:**
   - Keep your API keys secret
   - Don't share them publicly
   - Rotate keys if exposed

---

## 🚀 Quick Checklist

- [ ] Created Render account
- [ ] Deployed backend service on Render
- [ ] Added `OPENROUTER_API_KEY` in Render Environment tab
- [ ] Added `QDRANT_URL` in Render Environment tab
- [ ] Added `QDRANT_API_KEY` in Render Environment tab
- [ ] (Optional) Added `ALLOWED_ORIGINS` in Render
- [ ] Tested backend health endpoint
- [ ] Updated frontend with backend URL (if different from default)

---

## 📞 Need Help?

If you're stuck:
1. Check Render logs: Service → **Logs** tab
2. Verify all required variables are set
3. Test the health endpoint
4. Check CORS settings if frontend can't connect

