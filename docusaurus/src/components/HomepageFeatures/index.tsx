import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  icon: string;
  description: ReactNode;
  gradient: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS 2 Mastery',
    icon: '🤖',
    gradient: 'linear-gradient(135deg, #3b82f6 0%, #8b5cf6 100%)',
    description: (
      <>
        Master the robotic nervous system with ROS 2. Learn nodes, topics, services, 
        and create humanoid robot models with URDF. Build real-world robotics applications.
      </>
    ),
  },
  {
    title: 'Digital Twin Simulation',
    icon: '🌐',
    gradient: 'linear-gradient(135deg, #10b981 0%, #06b6d4 100%)',
    description: (
      <>
        Create high-fidelity digital twins with Gazebo & Unity. Understand physics simulation, 
        collisions, rendering, and integrate sensors including LiDAR, depth cameras, and IMUs.
      </>
    ),
  },
  {
    title: 'NVIDIA Isaac Integration',
    icon: '🚀',
    gradient: 'linear-gradient(135deg, #ec4899 0%, #f59e0b 100%)',
    description: (
      <>
        Leverage NVIDIA Isaac Sim for photorealistic simulation and synthetic data generation. 
        Implement VSLAM navigation and Nav2 for bipedal path planning.
      </>
    ),
  },
  {
    title: 'Voice-to-Action Systems',
    icon: '🎤',
    gradient: 'linear-gradient(135deg, #8b5cf6 0%, #ec4899 100%)',
    description: (
      <>
        Build natural language interfaces using OpenAI Whisper and LLM cognitive planning. 
        Transform voice commands into robotic actions with real-time processing.
      </>
    ),
  },
  {
    title: 'Hands-on Projects',
    icon: '⚡',
    gradient: 'linear-gradient(135deg, #f59e0b 0%, #ef4444 100%)',
    description: (
      <>
        Apply your knowledge through 20+ hands-on projects. From basic ROS nodes to 
        complete humanoid robot control systems. Learn by building real solutions.
      </>
    ),
  },
  {
    title: 'Production Ready',
    icon: '🏆',
    gradient: 'linear-gradient(135deg, #06b6d4 0%, #3b82f6 100%)',
    description: (
      <>
        Learn industry best practices and production-ready patterns. Deploy your robots 
        with confidence using proven architectures and deployment strategies.
      </>
    ),
  },
];

function Feature({title, icon, description, gradient}: FeatureItem) {
  return (
    <div className={clsx('col col--4', styles.featureCard)}>
      <div className={styles.featureIcon} style={{background: gradient}}>
        <span className={styles.iconEmoji}>{icon}</span>
      </div>
      <div className={styles.featureContent}>
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <p className={styles.featureDescription}>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            What You'll Master
          </Heading>
          <p className={styles.sectionSubtitle}>
            A comprehensive curriculum designed to take you from beginner to expert
          </p>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
