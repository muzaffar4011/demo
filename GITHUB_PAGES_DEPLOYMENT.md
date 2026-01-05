# GitHub Pages Deployment Guide

This project is configured to automatically deploy to GitHub Pages using GitHub Actions.

## Repository Configuration

- **Repository**: `muzaffar4011/demo`
- **GitHub Pages URL**: `https://muzaffar4011.github.io/demo/`
- **Base URL**: `/demo/`

## Automatic Deployment

The site is automatically deployed when you push to the `main` branch. The GitHub Actions workflow (`.github/workflows/deploy.yml`) will:

1. Check out the code
2. Install dependencies
3. Build the Docusaurus site
4. Deploy to GitHub Pages

## Manual Deployment

If you need to deploy manually:

1. **Build the site**:
   ```bash
   cd docusaurus
   npm install
   npm run build
   ```

2. **Deploy using Docusaurus CLI**:
   ```bash
   cd docusaurus
   npm run deploy
   ```

   This will push the built site to the `gh-pages` branch.

## Setting Up GitHub Pages

1. Go to your repository settings on GitHub
2. Navigate to **Pages** in the left sidebar
3. Under **Source**, select:
   - **Source**: `GitHub Actions` (recommended)
   - OR **Source**: `Deploy from a branch` → Branch: `gh-pages` → Folder: `/ (root)`

## Important Notes

- **Base URL**: The site is configured with `baseUrl: '/demo/'` to match your repository name
- **Routing**: Docusaurus handles client-side routing automatically. All URLs will work correctly on GitHub Pages
- **Assets**: All static assets (images, CSS, JS) are automatically handled with the correct paths
- **404 Page**: Docusaurus includes a custom 404 page that works with GitHub Pages

## Troubleshooting

### URLs not working after deployment

1. Verify the `baseUrl` in `docusaurus/docusaurus.config.ts` matches your repository name
2. Ensure the GitHub Actions workflow completed successfully
3. Check that GitHub Pages is enabled in repository settings

### Build fails

1. Check Node.js version (requires >= 20.0)
2. Verify all dependencies are installed: `cd docusaurus && npm install`
3. Check the GitHub Actions logs for specific errors

### Site shows 404

1. Wait a few minutes after deployment (GitHub Pages can take time to update)
2. Clear your browser cache
3. Verify the deployment branch is correct in repository settings

## Testing Locally

To test the production build locally:

```bash
cd docusaurus
npm run build
npm run serve
```

Visit `http://localhost:3000/demo/` to see how it will appear on GitHub Pages.

## Configuration Files

- **Docusaurus Config**: `docusaurus/docusaurus.config.ts`
- **GitHub Actions Workflow**: `.github/workflows/deploy.yml`
- **Package.json**: `docusaurus/package.json`

## Next Steps

1. Push your changes to the `main` branch
2. The GitHub Actions workflow will automatically deploy your site
3. Your site will be available at: `https://muzaffar4011.github.io/demo/`

