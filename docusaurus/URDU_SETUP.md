# Urdu Language Setup Instructions

## Important: Development Mode Limitation

In development mode, Docusaurus only supports **one locale at a time**. You cannot switch between locales using the dropdown in dev mode.

## To Test Urdu Locale:

### Option 1: Start Dev Server with Urdu Locale
```bash
# Stop current server (Ctrl+C)
npm run clear
npm run start -- --locale ur
```

Then access: `http://localhost:3000/ur/docs/intro`

### Option 2: Build and Serve (Supports All Locales)
```bash
npm run build
npm run serve
```

Then you can access both:
- English: `http://localhost:3000/en/docs/intro`
- Urdu: `http://localhost:3000/ur/docs/intro`

## File Structure

The Urdu translations are located in:
```
i18n/ur/
├── docusaurus-plugin-content-docs/current/
│   └── intro.md (Urdu translation)
├── docusaurus-plugin-content-pages/
│   └── index.json (Landing page translations)
└── docusaurus-theme-classic/
    ├── navbar.json (Navbar translations)
    └── footer.json (Footer translations)
```

## Adding More Urdu Translations

To translate more pages, copy the English file and translate it:

```bash
# Example: Translate module-1/index.md
cp docs/module-1/index.md i18n/ur/docusaurus-plugin-content-docs/current/module-1/index.md
```

Then edit the Urdu file and translate the content.

## Current Status

✅ Urdu locale configured
✅ Urdu intro page translated
✅ Urdu navbar and footer translations
✅ RTL (right-to-left) support enabled
✅ Locale dropdown in navbar

## Troubleshooting

If pages still don't load:
1. Clear the build cache: `npm run clear`
2. Restart the dev server with the locale flag: `npm run start -- --locale ur`
3. Check that files are in the correct directory structure
4. Verify file encoding is UTF-8

