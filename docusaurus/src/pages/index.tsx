import type {ReactNode} from 'react';
import { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Translate from '@docusaurus/Translate';
import { Cpu, Brain, Box, Zap, Users, Bot, Search, Menu, X, ChevronRight, ArrowRight, Code2, Sparkles, Star, Award, MessageSquare, Send, Layers, Sun, Moon, Github, Twitter, Linkedin, Mail } from 'lucide-react';
import styles from './index.module.css';

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  const [activeModule, setActiveModule] = useState<number | null>(null);
  const [scrolled, setScrolled] = useState(false);
  const [mobileMenu, setMobileMenu] = useState(false);
  const [colorMode, setColorMode] = useState<'light' | 'dark'>('light');

  useEffect(() => {
    if (typeof window === 'undefined') return;
    const handleScroll = () => setScrolled(window.scrollY > 20);
    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  // Get current theme and listen for changes
  useEffect(() => {
        const getCurrentTheme = (): 'light' | 'dark' => {
          if (typeof window === 'undefined') return 'light';
          const html = document.documentElement;
          const theme = html.getAttribute('data-theme') as 'light' | 'dark';
          return theme || (window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');
        };

    setColorMode(getCurrentTheme());

    // Listen for theme changes
    const observer = new MutationObserver(() => {
      setColorMode(getCurrentTheme());
    });

    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ['data-theme']
    });

    return () => observer.disconnect();
  }, []);

  // Hide Docusaurus navbar on landing page
  useEffect(() => {
    const navbar = document.querySelector('.navbar') as HTMLElement;
    if (navbar) {
      navbar.style.display = 'none';
    }
    return () => {
      // Restore navbar when leaving the page
      if (navbar) {
        navbar.style.display = '';
      }
    };
  }, []);

  // Toggle theme function
  const toggleTheme = () => {
    const html = document.documentElement;
    const currentTheme = html.getAttribute('data-theme') || 'light';
    const newTheme = currentTheme === 'dark' ? 'light' : 'dark';
    
    // Set theme attribute
    html.setAttribute('data-theme', newTheme);
    
    // Save to localStorage (Docusaurus way)
    try {
      localStorage.setItem('theme', newTheme);
    } catch (e) {
      // Ignore localStorage errors
    }

    // Trigger Docusaurus theme change event if available
    if (typeof window !== 'undefined') {
      const event = new CustomEvent('theme-change', { detail: { theme: newTheme } });
      window.dispatchEvent(event);
    }

    setColorMode(newTheme);
  };

  const modules = [
    {num:"1", title:"ROS2 Foundations", desc:"Master the nervous system of modern robots with nodes, topics, services, and actions", icon:<Layers style={{width: '28px', height: '28px'}}/>, color:"from-violet-600 to-purple-600", shadow:"shadow-violet-500/20", path:"/docs/module-1/"},
    {num:"2", title:"Simulation & Digital Twins", desc:"Build and test robots safely in Gazebo, Unity, and Isaac Sim environments", icon:<Box style={{width: '28px', height: '28px'}}/>, color:"from-blue-600 to-cyan-600", shadow:"shadow-blue-500/20", path:"/docs/module-2/"},
    {num:"3", title:"Hardware Foundations", desc:"Learn motors, actuators, sensors, and embedded systems for real robots", icon:<Cpu style={{width: '28px', height: '28px'}}/>, color:"from-amber-600 to-orange-600", shadow:"shadow-amber-500/20", path:"/docs/module-3/"},
    {num:"4", title:"VLA Systems", desc:"Master Vision-Language-Action architectures for intelligent robotics", icon:<Brain style={{width: '28px', height: '28px'}}/>, color:"from-emerald-600 to-teal-600", shadow:"shadow-emerald-500/20", path:"/docs/module-4/"},
    {num:"5", title:"Advanced AI & Control", desc:"Dive into reinforcement learning, motion planning, and trajectory optimization", icon:<Zap style={{width: '28px', height: '28px'}}/>, color:"from-rose-600 to-pink-600", shadow:"shadow-rose-500/20", path:"/docs/module-4/"},
    {num:"6", title:"Humanoid Design", desc:"Design complete humanoid robots from mechanical systems to AI movement", icon:<Users style={{width: '28px', height: '28px'}}/>, color:"from-indigo-600 to-purple-600", shadow:"shadow-indigo-500/20", path:"/docs/module-4/"}
  ];

  const features = [
    {icon:<Code2 style={{width: '24px', height: '24px'}}/>, title:"Practical & Hands-On", desc:"Real-world examples, code snippets, and simulations in every module"},
    {icon:<Award style={{width: '24px', height: '24px'}}/>, title:"Industry-Aligned", desc:"Technologies used by Tesla, Figure AI, and leading robotics companies"},
    {icon:<Star style={{width: '24px', height: '24px'}}/>, title:"Beginner Friendly", desc:"Start from basics and progress to advanced humanoid systems"},
    {icon:<Brain style={{width: '24px', height: '24px'}}/>, title:"AI-Native", desc:"Built around modern AI workflows including LLMs and VLA systems"}
  ];

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics: Embodied Intelligence - Bridging Digital AI with Physical Bodies"
      wrapperClassName={styles.layoutWrapper}
      noFooter
    >
      <div className={styles.pageContainer}>
        {/* Background Effects */}
        <div className={styles.backgroundEffects}>
          <div className={styles.backgroundOrb1}></div>
          <div className={styles.backgroundOrb2}></div>
          <div className={styles.backgroundOrb3}></div>
        </div>

        {/* Navigation */}
        <nav className={`${styles.nav} ${scrolled ? styles.navScrolled : ''}`}>
          <div className={styles.navContainer}>
            <div className={styles.navBrand}>
              <div className={styles.navBrandIconWrapper}>
                <div className={styles.navBrandIconGlow}></div>
                <div className={styles.navBrandIcon}>
                  <Bot style={{width: '28px', height: '28px'}} />
                </div>
              </div>
              <div className={styles.navBrandText}>
                <div className={styles.navBrandTitle}>Physical AI & Humanoid Robotics</div>
                <div className={styles.navBrandSubtitle}>Advanced Learning Platform</div>
              </div>
            </div>
            <div className={styles.navLinks}>
              <Link to="/" className={styles.navLinkActive}>
                <Translate id="nav.home">Home</Translate>
              </Link>
              <Link to="/docs/intro" className={styles.navLink}>
                <Translate id="nav.textbook">Textbook</Translate>
              </Link>
            </div>
            <div className={styles.navActions}>
              <button 
                onClick={toggleTheme}
                className={styles.navActionButton}
                aria-label="Toggle theme"
                title={colorMode === 'dark' ? 'Switch to light mode' : 'Switch to dark mode'}
              >
                {colorMode === 'dark' ? (
                  <Sun style={{width: '20px', height: '20px'}} />
                ) : (
                  <Moon style={{width: '20px', height: '20px'}} />
                )}
              </button>
              <button className={styles.navActionButton}>
                <Search style={{width: '20px', height: '20px'}} />
              </button>
              <button 
                onClick={() => setMobileMenu(!mobileMenu)} 
                className={`${styles.navActionButton} ${styles.navMobileMenu}`}
              >
                {mobileMenu ? <X style={{width: '20px', height: '20px'}} /> : <Menu style={{width: '20px', height: '20px'}} />}
              </button>
            </div>
          </div>
        </nav>

        <main className={styles.main}>
          {/* Hero Section */}
          <section className={styles.heroSection}>
            <div className={styles.heroContainer}>
              <div className={styles.heroContent}>
          
                <div className={styles.heroText}>
                  <h1 className={styles.heroTitle}>
                    <span className={styles.heroTitleLine}>
                      <Translate id="hero.title.line1">Build the Future of</Translate>
                    </span>
                    <span className={`${styles.heroTitleLine} ${styles.heroTitleGradient}`}>
                      <Translate id="hero.title.line2">Intelligent Robotics</Translate>
                    </span>
                  </h1>
                  <p className={styles.heroDescription}>
                    <Translate id="hero.description">
                      A comprehensive textbook covering ROS2, AI systems, VLA architectures, digital twins, 
                      and humanoid design — everything you need to master physical AI
                    </Translate>
                  </p>
                </div>
                <div className={styles.heroButtons}>
                  <Link
                    to="/docs/intro"
                    className={styles.heroButtonPrimary}
                  >
                    <span><Translate id="hero.button.getStarted">Get Started</Translate></span>
                    <ArrowRight style={{width: '24px', height: '24px'}} />
                  </Link>
                  <Link
                    to="/docs/module-1"
                    className={styles.heroButtonSecondary}
                  >
                    <Translate id="hero.button.exploreModules">Explore Modules</Translate>
                  </Link>
                </div>
              </div>
            </div>
          </section>

          {/* Learning Path */}
          <section className={styles.learningPathSection}>
            <div className={styles.learningPathContainer}>
              <div className={styles.learningPathHeader}>
                <h2 className={styles.learningPathTitle}>
                  <Translate id="learningPath.title">Learning Path</Translate>
                </h2>
                <p className={styles.learningPathSubtitle}>
                  <Translate id="learningPath.subtitle">
                    Master physical AI through our structured 6-module curriculum
                  </Translate>
                </p>
              </div>
              <div className={styles.modulesGrid}>
                {modules.map((module, idx) => (
                  <div
                    key={idx}
                    onMouseEnter={() => setActiveModule(idx)}
                    onMouseLeave={() => setActiveModule(null)}
                    className={`${styles.moduleCard} ${
                      activeModule === idx ? styles.moduleCardActive : ''
                    }`}
                  >
                    <div className={`${styles.moduleNumber} ${styles[`moduleNumber${module.color.replace(/\s/g, '')}`]}`}>
                      {module.num}
                    </div>
                    <div className={styles.moduleContent}>
                      <div className={`${styles.moduleIcon} ${styles[`moduleIcon${module.color.replace(/\s/g, '')}`]}`}>
                        {module.icon}
                      </div>
                      <div className={styles.moduleText}>
                        <h3 className={styles.moduleTitle}>{module.title}</h3>
                        <p className={styles.moduleDesc}>{module.desc}</p>
                      </div>
                      <Link
                        to={module.path}
                        className={`${styles.moduleButton} ${
                          activeModule === idx ? styles.moduleButtonActive : ''
                        }`}
                      >
                        <span><Translate id="module.button.start">Start Module</Translate></span>
                        <ChevronRight style={{width: '20px', height: '20px'}} className={activeModule === idx ? styles.moduleButtonIconActive : ''} />
                      </Link>
                    </div>
                  </div>
                ))}
              </div>
            </div>
          </section>

          {/* Features */}
          <section className={styles.featuresSection}>
            <div className={styles.featuresContainer}>
              <div className={styles.featuresGrid}>
                {features.map((feat, idx) => (
                  <div key={idx} className={styles.featureCard}>
                    <div className={styles.featureIcon}>
                      {feat.icon}
                    </div>
                    <h3 className={styles.featureTitle}>{feat.title}</h3>
                    <p className={styles.featureDesc}>{feat.desc}</p>
                  </div>
                ))}
              </div>
            </div>
          </section>

          {/* CTA */}
          <section className={styles.ctaSection}>
            <div className={styles.ctaContainer}>
              <h2 className={styles.ctaTitle}>
                <Translate id="cta.title">Start Your Robotics Journey</Translate>
              </h2>
              <p className={styles.ctaSubtitle}>
                <Translate id="cta.subtitle">
                  Master the technologies powering the next generation of intelligent robots
                </Translate>
              </p>
              <Link
                to="/docs/intro"
                className={styles.ctaButton}
              >
                <Translate id="cta.button">Begin Learning</Translate>
              </Link>
            </div>
          </section>
        </main>

        {/* Footer */}
        <footer className={styles.footer}>
          <div className={styles.footerContainer}>
            <div className={styles.footerContent}>
              <div className={styles.footerBrand}>
                <div className={styles.footerBrandIcon}>
                  <Bot style={{width: '32px', height: '32px'}} />
                </div>
                <div className={styles.footerBrandText}>
                  <h3 className={styles.footerBrandTitle}>Physical AI & Humanoid Robotics</h3>
                  <p className={styles.footerBrandSubtitle}>Advanced Learning Platform</p>
                </div>
              </div>

              <div className={styles.footerLinks}>
                <div className={styles.footerColumn}>
                  <h4 className={styles.footerColumnTitle}>Learn</h4>
                  <ul className={styles.footerLinkList}>
                    <li><Link to="/docs/intro" className={styles.footerLink}>Documentation</Link></li>
                    <li><Link to="/docs/module-1" className={styles.footerLink}>Modules</Link></li>
                    <li><Link to="/blog" className={styles.footerLink}>Blog</Link></li>
                    <li><Link to="/docs/intro" className={styles.footerLink}>Tutorials</Link></li>
                  </ul>
                </div>

                <div className={styles.footerColumn}>
                  <h4 className={styles.footerColumnTitle}>Resources</h4>
                  <ul className={styles.footerLinkList}>
                    <li><a href="https://github.com/muzaffar4011/demo" target="_blank" rel="noopener noreferrer" className={styles.footerLink}>GitHub</a></li>
                    <li><a href="#" className={styles.footerLink}>Community</a></li>
                    <li><a href="#" className={styles.footerLink}>Support</a></li>
                    <li><a href="#" className={styles.footerLink}>FAQ</a></li>
                  </ul>
                </div>

                <div className={styles.footerColumn}>
                  <h4 className={styles.footerColumnTitle}>Connect</h4>
                  <ul className={styles.footerLinkList}>
                    <li><a href="#" className={styles.footerLink}>About Us</a></li>
                    <li><a href="#" className={styles.footerLink}>Contact</a></li>
                    <li><a href="#" className={styles.footerLink}>Privacy Policy</a></li>
                    <li><a href="#" className={styles.footerLink}>Terms of Service</a></li>
                  </ul>
                </div>
              </div>
            </div>

            <div className={styles.footerBottom}>
              <div className={styles.footerSocial}>
                <a href="https://github.com/muzaffar4011/demo" target="_blank" rel="noopener noreferrer" className={styles.footerSocialLink} aria-label="GitHub">
                  <Github style={{width: '20px', height: '20px'}} />
                </a>
                <a href="#" className={styles.footerSocialLink} aria-label="Twitter">
                  <Twitter style={{width: '20px', height: '20px'}} />
                </a>
                <a href="#" className={styles.footerSocialLink} aria-label="LinkedIn">
                  <Linkedin style={{width: '20px', height: '20px'}} />
                </a>
                <a href="#" className={styles.footerSocialLink} aria-label="Email">
                  <Mail style={{width: '20px', height: '20px'}} />
                </a>
              </div>
              <p className={styles.footerCopyright}>
                © {new Date().getFullYear()} Physical AI & Humanoid Robotics. All rights reserved.
              </p>
            </div>
          </div>
        </footer>
      </div>
    </Layout>
  );
}
