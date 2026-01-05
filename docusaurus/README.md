# Physical AI & Humanoid Robotics: Embodied Intelligence

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator. It serves as comprehensive documentation for the Physical AI & Humanoid Robotics course, bridging the gap between digital AI and physical bodies.

## Overview

This documentation covers four comprehensive modules:

1. **The Robotic Nervous System (ROS 2)**: Master nodes, topics, services, bridging Python agents via rclpy, and creating humanoid robot models with URDF.

2. **The Digital Twin (Gazebo & Unity)**: Understand physics simulation, collisions, high-fidelity rendering, and sensor integration including LiDAR, depth cameras, and IMUs.

3. **The AI-Robot Brain (NVIDIA Isaac™)**: Explore Isaac Sim for photorealistic simulation and synthetic data generation, Isaac ROS for VSLAM/navigation, and Nav2 for bipedal path planning.

4. **Vision-Language-Action (VLA)**: Implement voice-to-action systems using OpenAI Whisper and LLM cognitive planning for natural language commands.

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## GitHub Pages Deployment

This site is configured for deployment to GitHub Pages using GitHub Actions. The workflow is defined in `.github/workflows/deploy.yml` and will automatically deploy the site when changes are pushed to the main branch.

To manually deploy:

```bash
npm run deploy
```

This command builds the website and pushes the static files to the `gh-pages` branch for GitHub Pages hosting.

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add some amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.
