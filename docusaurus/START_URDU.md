# How to View Urdu Version

## Problem
When you switch to Urdu using the dropdown, you get a "Page Not Found" error.

## Solution

### Option 1: Development Mode with Urdu (Recommended for Testing)

**Important**: In development mode, Docusaurus only supports **one locale at a time**. You must restart the server with the Urdu locale flag.

1. **Stop the current server** (Press `Ctrl+C` in the terminal where it's running)

2. **Clear the cache**:
   ```bash
   npm run clear
   ```

3. **Start with Urdu locale**:
   ```bash
   npm run start -- --locale ur
   ```

4. **Access the Urdu version**:
   - Open: `http://localhost:3000/ur/docs/intro`
   - The URL will automatically have `/ur/` prefix

### Option 2: Production Build (Automatic Switching Works)

For automatic locale switching to work, you need to build and serve:

1. **Build with all locales**:
   ```bash
   npm run build
   ```

2. **Serve the built site**:
   ```bash
   npm run serve
   ```

3. **Access both languages**:
   - English: `http://localhost:3000/en/docs/intro`
   - Urdu: `http://localhost:3000/ur/docs/intro`
   - The dropdown will automatically switch between languages

## Why This Happens

- **Development mode**: Only one locale at a time (faster for development)
- **Production build**: All locales available (automatic switching works)

## Quick Fix Right Now

If you want to see Urdu immediately:

```bash
# Stop current server (Ctrl+C)
npm run clear
npm run start -- --locale ur
```

Then visit: `http://localhost:3000/ur/docs/intro`

