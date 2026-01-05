import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/demo/blog',
    component: ComponentCreator('/demo/blog', '9cb'),
    exact: true
  },
  {
    path: '/demo/blog/archive',
    component: ComponentCreator('/demo/blog/archive', '373'),
    exact: true
  },
  {
    path: '/demo/blog/authors',
    component: ComponentCreator('/demo/blog/authors', '74f'),
    exact: true
  },
  {
    path: '/demo/blog/authors/all-sebastien-lorber-articles',
    component: ComponentCreator('/demo/blog/authors/all-sebastien-lorber-articles', 'f99'),
    exact: true
  },
  {
    path: '/demo/blog/authors/yangshun',
    component: ComponentCreator('/demo/blog/authors/yangshun', 'aa4'),
    exact: true
  },
  {
    path: '/demo/blog/first-blog-post',
    component: ComponentCreator('/demo/blog/first-blog-post', '02a'),
    exact: true
  },
  {
    path: '/demo/blog/long-blog-post',
    component: ComponentCreator('/demo/blog/long-blog-post', '3ad'),
    exact: true
  },
  {
    path: '/demo/blog/mdx-blog-post',
    component: ComponentCreator('/demo/blog/mdx-blog-post', '294'),
    exact: true
  },
  {
    path: '/demo/blog/tags',
    component: ComponentCreator('/demo/blog/tags', '9e0'),
    exact: true
  },
  {
    path: '/demo/blog/tags/docusaurus',
    component: ComponentCreator('/demo/blog/tags/docusaurus', '64b'),
    exact: true
  },
  {
    path: '/demo/blog/tags/facebook',
    component: ComponentCreator('/demo/blog/tags/facebook', 'ffe'),
    exact: true
  },
  {
    path: '/demo/blog/tags/hello',
    component: ComponentCreator('/demo/blog/tags/hello', '326'),
    exact: true
  },
  {
    path: '/demo/blog/tags/hola',
    component: ComponentCreator('/demo/blog/tags/hola', '230'),
    exact: true
  },
  {
    path: '/demo/blog/welcome',
    component: ComponentCreator('/demo/blog/welcome', 'dfb'),
    exact: true
  },
  {
    path: '/demo/markdown-page',
    component: ComponentCreator('/demo/markdown-page', '3dc'),
    exact: true
  },
  {
    path: '/demo/docs',
    component: ComponentCreator('/demo/docs', '3ed'),
    routes: [
      {
        path: '/demo/docs',
        component: ComponentCreator('/demo/docs', '6e1'),
        routes: [
          {
            path: '/demo/docs',
            component: ComponentCreator('/demo/docs', '2b7'),
            routes: [
              {
                path: '/demo/docs/appendix/faq',
                component: ComponentCreator('/demo/docs/appendix/faq', '513'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/appendix/glossary',
                component: ComponentCreator('/demo/docs/appendix/glossary', '637'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/appendix/troubleshooting',
                component: ComponentCreator('/demo/docs/appendix/troubleshooting', 'f25'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/intro',
                component: ComponentCreator('/demo/docs/intro', '721'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-1/',
                component: ComponentCreator('/demo/docs/module-1/', '4d5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-1/rclpy-bridge',
                component: ComponentCreator('/demo/docs/module-1/rclpy-bridge', 'ca4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-1/ros-nodes',
                component: ComponentCreator('/demo/docs/module-1/ros-nodes', 'f78'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-1/topics-services',
                component: ComponentCreator('/demo/docs/module-1/topics-services', '582'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-1/urdf-humanoids',
                component: ComponentCreator('/demo/docs/module-1/urdf-humanoids', '38f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-2/',
                component: ComponentCreator('/demo/docs/module-2/', '673'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-2/collisions',
                component: ComponentCreator('/demo/docs/module-2/collisions', '5af'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-2/physics-simulation',
                component: ComponentCreator('/demo/docs/module-2/physics-simulation', '7ab'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-2/rendering',
                component: ComponentCreator('/demo/docs/module-2/rendering', 'cf0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-2/sensors',
                component: ComponentCreator('/demo/docs/module-2/sensors', '6e2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-3/',
                component: ComponentCreator('/demo/docs/module-3/', '770'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-3/isaac-sim',
                component: ComponentCreator('/demo/docs/module-3/isaac-sim', 'e5f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-3/nav2-bipedal',
                component: ComponentCreator('/demo/docs/module-3/nav2-bipedal', '2cb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-3/vslam-navigation',
                component: ComponentCreator('/demo/docs/module-3/vslam-navigation', '41d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-4/',
                component: ComponentCreator('/demo/docs/module-4/', '364'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-4/capstone-project',
                component: ComponentCreator('/demo/docs/module-4/capstone-project', 'ebb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-4/llm-planning',
                component: ComponentCreator('/demo/docs/module-4/llm-planning', '4bd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/module-4/voice-to-action',
                component: ComponentCreator('/demo/docs/module-4/voice-to-action', '09e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/demo/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/demo/docs/tutorial-basics/congratulations', 'b04'),
                exact: true
              },
              {
                path: '/demo/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/demo/docs/tutorial-basics/create-a-blog-post', 'a91'),
                exact: true
              },
              {
                path: '/demo/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/demo/docs/tutorial-basics/create-a-document', '5ef'),
                exact: true
              },
              {
                path: '/demo/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/demo/docs/tutorial-basics/create-a-page', '1c4'),
                exact: true
              },
              {
                path: '/demo/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/demo/docs/tutorial-basics/deploy-your-site', '3ff'),
                exact: true
              },
              {
                path: '/demo/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/demo/docs/tutorial-basics/markdown-features', '68c'),
                exact: true
              },
              {
                path: '/demo/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/demo/docs/tutorial-extras/manage-docs-versions', '436'),
                exact: true
              },
              {
                path: '/demo/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/demo/docs/tutorial-extras/translate-your-site', '6dc'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/demo/',
    component: ComponentCreator('/demo/', '220'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
