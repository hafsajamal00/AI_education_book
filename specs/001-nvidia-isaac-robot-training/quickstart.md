# Quickstart: NVIDIA Isaac™ Robot Training Module

## Prerequisites

- Node.js v18 or higher
- npm or yarn package manager
- Basic knowledge of Markdown and JavaScript

## Setup

1. **Clone the repository** (if not already done):
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Navigate to the module directory**:
   ```bash
   cd module-3-ai-robot-brain
   ```

3. **Install dependencies**:
   ```bash
   npm install
   # or
   yarn install
   ```

4. **Start the development server**:
   ```bash
   npm start
   # or
   yarn start
   ```
   
   This command starts a local development server and opens the documentation site in your browser at `http://localhost:3000`.

## Project Structure

```
module-3-ai-robot-brain/
├── docs/
│   ├── chapter-1-nvidia-isaac-sim-fundamentals.md
│   ├── chapter-2-isaac-ros-perception-navigation.md
│   └── chapter-3-nav2-bipedal-humanoid-movement.md
├── src/
│   ├── components/
│   ├── pages/
│   └── css/
├── static/
├── docusaurus.config.js
├── package.json
├── sidebars.js
└── README.md
```

## Adding Content

1. **Create new documentation pages** in the `docs/` folder as `.md` files
2. **Update sidebar navigation** in `sidebars.js` to include new pages
3. **Customize styles** in `src/css/custom.css`
4. **Add static assets** (images, etc.) to the `static/` folder

## Building for Production

To build a static version of the site for deployment:

```bash
npm run build
# or
yarn build
```

This will generate a `build/` folder with the static site that can be deployed to any web server.

## Deployment

The site can be deployed using:
- GitHub Pages
- Netlify
- Vercel
- Any standard web server

For GitHub Pages, run `npm run deploy` after configuring your repository settings.