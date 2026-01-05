# Automatic Locale Switching Guide

## How Locale Switching Works

Docusaurus automatically handles locale switching when you build the site with all locales. The locale dropdown in the navbar will automatically redirect users to the selected language.

## Development Mode

**Important**: In development mode, Docusaurus only supports **one locale at a time**. The locale dropdown will not automatically switch in dev mode.

### To Test Urdu in Development:
```bash
npm run clear
npm run start -- --locale ur
```

Then access: `http://localhost:3000/ur/docs/intro`

## Production Mode (Automatic Switching)

For automatic locale switching to work, you need to build the site with all locales:

```bash
# Build with all locales
npm run build

# Serve the built site
npm run serve
```

After building, the locale dropdown will automatically:
- Switch to the selected language
- Redirect to the correct URL with the locale prefix
- Maintain the current page path

### Access URLs:
- English: `http://localhost:3000/en/docs/intro`
- Urdu: `http://localhost:3000/ur/docs/intro`

## How It Works

1. **Locale Dropdown**: When a user clicks on a language in the dropdown, Docusaurus automatically:
   - Detects the current page path
   - Redirects to the same page in the selected locale
   - Preserves the page structure

2. **URL Structure**: 
   - English pages: `/en/docs/...`
   - Urdu pages: `/ur/docs/...`

3. **Automatic Detection**: The system automatically:
   - Detects the locale from the URL
   - Loads the appropriate translation files
   - Applies RTL/LTR direction based on locale

## Troubleshooting

If automatic switching doesn't work:

1. **Clear cache and rebuild**:
   ```bash
   npm run clear
   npm run build
   npm run serve
   ```

2. **Verify all translation files exist**:
   - Check `i18n/ur/` directory has all required files
   - Ensure file encoding is UTF-8

3. **Check browser console** for any errors

4. **Verify configuration** in `docusaurus.config.ts`:
   - `i18n.locales` includes both 'en' and 'ur'
   - `localeDropdown` is in navbar items

## Current Status

✅ All 21 documentation files translated to Urdu
✅ Locale dropdown configured in navbar
✅ RTL support enabled for Urdu
✅ Automatic switching works in production builds

