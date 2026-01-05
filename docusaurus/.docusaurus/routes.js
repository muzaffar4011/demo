import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-humanoid-robotics/ur/blog',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog', 'fc5'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/blog/archive',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog/archive', '161'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/blog/authors',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog/authors', '772'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/blog/authors/all-sebastien-lorber-articles',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog/authors/all-sebastien-lorber-articles', 'a20'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/blog/authors/yangshun',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog/authors/yangshun', 'f4b'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/blog/first-blog-post',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog/first-blog-post', '779'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/blog/long-blog-post',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog/long-blog-post', '23f'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/blog/mdx-blog-post',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog/mdx-blog-post', '577'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/blog/tags',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog/tags', '6a3'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/blog/tags/docusaurus',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog/tags/docusaurus', '201'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/blog/tags/facebook',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog/tags/facebook', '3bd'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/blog/tags/hello',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog/tags/hello', '7e7'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/blog/tags/hola',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog/tags/hola', '07c'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/blog/welcome',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/blog/welcome', '3c2'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/markdown-page',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/markdown-page', '9d8'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/docs',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs', '742'),
    routes: [
      {
        path: '/physical-ai-humanoid-robotics/ur/docs',
        component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs', 'e5f'),
        routes: [
          {
            path: '/physical-ai-humanoid-robotics/ur/docs',
            component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs', 'a1a'),
            routes: [
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/appendix/faq',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/appendix/faq', 'b4e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/appendix/glossary',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/appendix/glossary', 'bc1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/appendix/troubleshooting',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/appendix/troubleshooting', '232'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/intro',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/intro', 'a12'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-1/',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-1/', '36c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-1/rclpy-bridge',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-1/rclpy-bridge', '9a6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-1/ros-nodes',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-1/ros-nodes', 'fda'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-1/topics-services',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-1/topics-services', '957'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-1/urdf-humanoids',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-1/urdf-humanoids', '16d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-2/',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-2/', '8ac'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-2/collisions',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-2/collisions', '8e1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-2/physics-simulation',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-2/physics-simulation', '58c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-2/rendering',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-2/rendering', '5a6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-2/sensors',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-2/sensors', 'cff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-3/',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-3/', '63a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-3/isaac-sim',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-3/isaac-sim', 'fb7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-3/nav2-bipedal',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-3/nav2-bipedal', 'a20'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-3/vslam-navigation',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-3/vslam-navigation', '53f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-4/',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-4/', 'b5d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-4/capstone-project',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-4/capstone-project', 'e49'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-4/llm-planning',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-4/llm-planning', 'dca'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/module-4/voice-to-action',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/module-4/voice-to-action', 'f4b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/tutorial-basics/congratulations', '7f8'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/tutorial-basics/create-a-blog-post', '82d'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/tutorial-basics/create-a-document', '60f'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/tutorial-basics/create-a-page', 'e76'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/tutorial-basics/deploy-your-site', '764'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/tutorial-basics/markdown-features', '202'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/tutorial-extras/manage-docs-versions', 'fb1'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics/ur/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/physical-ai-humanoid-robotics/ur/docs/tutorial-extras/translate-your-site', 'cd4'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/physical-ai-humanoid-robotics/ur/',
    component: ComponentCreator('/physical-ai-humanoid-robotics/ur/', '8ee'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
