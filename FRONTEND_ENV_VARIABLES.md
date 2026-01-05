# Frontend Environment Variables Setup (GitHub)

## 🎯 Where to Add Environment Variables for Frontend

### Step-by-Step Guide:

#### 1. Go to Your GitHub Repository

1. Visit: `https://github.com/muzaffar4011/demo`
2. Click on **"Settings"** (top menu bar)

#### 2. Navigate to Secrets and Variables

1. In the left sidebar, click **"Secrets and variables"**
2. Click **"Actions"**

#### 3. Add Variables

You have two options:

**Option A: Variables (Recommended for public values)**
- Click on **"Variables"** tab
- Click **"New repository variable"**
- Add your backend URL:

| Name | Value |
|------|-------|
| `CHATBOT_API_URL` | `https://your-backend-name.onrender.com` |

**Option B: Secrets (For sensitive data)**
- Click on **"Secrets"** tab
- Click **"New repository secret"**
- Use this if you have sensitive API keys (usually not needed for frontend)

#### 4. Visual Guide

```
GitHub Repository (muzaffar4011/demo)
└── Settings (top menu)
    └── Secrets and variables (left sidebar)
        └── Actions
            └── Variables tab ← CLICK HERE
                └── New repository variable
                    ├── Name: CHATBOT_API_URL
                    ├── Value: https://rag-chatbot-backend.onrender.com
                    └── Add variable
```

---

## 📋 Required Variable

### `CHATBOT_API_URL`

- **Purpose**: Tells the frontend where to find your backend API
- **Value**: Your Render backend URL (e.g., `https://rag-chatbot-backend.onrender.com`)
- **Where to Add**: GitHub → Settings → Secrets and variables → Actions → Variables

---

## 🔄 How It Works

1. **You add the variable** in GitHub repository settings
2. **GitHub Actions workflow** reads it during build
3. **Docusaurus config** uses it to set `window.CHATBOT_API_URL`
4. **Frontend** connects to your backend using this URL

---

## ✅ After Adding the Variable

1. **Commit and push** any changes (if you modified the workflow)
2. **GitHub Actions will automatically run** on your next push
3. **The build will use** your environment variable
4. **Your frontend will connect** to the specified backend URL

---

## 🧪 Testing

After deployment:

1. Visit: `https://muzaffar4011.github.io/demo/`
2. Open browser console (F12)
3. Type: `window.CHATBOT_API_URL`
4. You should see your backend URL

---

## 📝 Current Default

If you **don't add** the variable, the frontend will use:
```
https://rag-chatbot-backend.onrender.com
```

This is already set as the default in `docusaurus.config.ts`.

---

## ⚠️ Important Notes

1. **Variables vs Secrets**:
   - Use **Variables** for public values (like backend URLs)
   - Use **Secrets** for sensitive data (API keys, passwords)

2. **Case Sensitive**:
   - Variable names are case-sensitive
   - Use exactly: `CHATBOT_API_URL`

3. **No Quotes Needed**:
   - Don't add quotes around the URL value
   - Just paste: `https://rag-chatbot-backend.onrender.com`

4. **Automatic Deployment**:
   - After adding the variable, push a new commit to trigger rebuild
   - Or manually trigger the workflow from Actions tab

---

## 🔍 Verify It's Working

### Check the Build Logs:

1. Go to **Actions** tab in your repository
2. Click on the latest workflow run
3. Expand **"Build website"** step
4. Check that it's using your variable

### Check the Deployed Site:

1. Visit your site: `https://muzaffar4011.github.io/demo/`
2. Open browser DevTools (F12)
3. Go to **Console** tab
4. Type: `window.CHATBOT_API_URL`
5. Should show your backend URL

---

## 🚀 Quick Checklist

- [ ] Go to GitHub repository settings
- [ ] Click "Secrets and variables" → "Actions"
- [ ] Click "Variables" tab
- [ ] Click "New repository variable"
- [ ] Name: `CHATBOT_API_URL`
- [ ] Value: Your Render backend URL
- [ ] Click "Add variable"
- [ ] Push a commit to trigger rebuild
- [ ] Test the chatbot on your site

---

## 💡 Pro Tip

If your Render backend URL changes:
1. Update the variable in GitHub
2. Push a new commit (or manually trigger workflow)
3. The frontend will automatically use the new URL

