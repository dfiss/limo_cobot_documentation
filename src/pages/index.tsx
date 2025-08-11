import type {ReactNode} from 'react';
import { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import AnimatedRobot from '@site/src/components/AnimatedRobot';
import ScrollAnimation from '@site/src/components/ScrollAnimation';
import ScrollProgress from '@site/src/components/ScrollProgress';
import TechBackground from '@site/src/components/TechBackground';
import StatusDisplay from '@site/src/components/StatusDisplay';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroLeft}>
            <Heading as="h1" className={styles.heroTitle}>
              {siteConfig.title}
            </Heading>
            <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Get Started 🚀
              </Link>
              <Link
                className="button button--outline button--secondary button--lg"
                to="/docs/intro"
                style={{marginLeft: '1rem'}}>
                View Documentation
              </Link>
            </div>
          </div>
          <div className={styles.heroRight}>
            <AnimatedRobot animationStage={5} />
            <StatusDisplay stage={5} />
          </div>
        </div>
      </div>
    </header>
  );
}

function RobotStorySection() {
  const [robotStage, setRobotStage] = useState(0);
  const [robotPosition, setRobotPosition] = useState(0);
  const cardRefs = useRef([]);
  const containerRef = useRef(null);

  useEffect(() => {
    const handleScroll = () => {
      if (!containerRef.current) return;

      const containerRect = containerRef.current.getBoundingClientRect();
      const containerTop = containerRect.top;
      const viewportHeight = window.innerHeight;
      
      // Find which card is most visible
      let mostVisibleCard = -1;
      let maxVisibility = 0;

      cardRefs.current.forEach((card, index) => {
        if (!card) return;
        
        const cardRect = card.getBoundingClientRect();
        const cardTop = cardRect.top;
        const cardBottom = cardRect.bottom;
        const cardHeight = cardRect.height;
        
        // Calculate how much of the card is visible
        const visibleTop = Math.max(cardTop, 0);
        const visibleBottom = Math.min(cardBottom, viewportHeight);
        const visibleHeight = Math.max(0, visibleBottom - visibleTop);
        const visibilityRatio = visibleHeight / cardHeight;
        
        if (visibilityRatio > maxVisibility) {
          maxVisibility = visibilityRatio;
          mostVisibleCard = index;
        }
      });

      if (mostVisibleCard >= 0) {
        setRobotStage(mostVisibleCard);
        
        // Calculate robot position to align with the most visible card
        const targetCard = cardRefs.current[mostVisibleCard];
        if (targetCard) {
          const cardRect = targetCard.getBoundingClientRect();
          const cardCenter = cardRect.top + cardRect.height / 2;
          
          // Convert to position relative to container, then to vh units
          const relativePosition = (cardCenter - containerTop) / viewportHeight * 100;
          
          // Adjust to center robot with card (subtract robot height offset)
          setRobotPosition(Math.max(0, relativePosition - 20)); // 20vh offset to center robot
        }
      }
    };

    // Listen to scroll events
    window.addEventListener('scroll', handleScroll);
    window.addEventListener('resize', handleScroll);
    
    // Initial calculation
    handleScroll();

    return () => {
      window.removeEventListener('scroll', handleScroll);
      window.removeEventListener('resize', handleScroll);
    };
  }, []);

  const handleProgressChange = (_progress: number) => {
    // Keep for backward compatibility
  };

  return (
    <div ref={containerRef} className={styles.storyContainer}>
      <ScrollProgress onProgressChange={handleProgressChange} />
      
      <div className={styles.storyLayout}>
      <div className={styles.robotColumn}>
        <div 
        className={styles.robotWrapper}
        style={{
          transform: `translateX(-50%) translateY(${robotPosition}vh)`
        }}
        >
        <AnimatedRobot animationStage={robotStage} />
        <StatusDisplay stage={robotStage} />
        </div>
      </div>
      
      <div className={styles.contentColumn}>
        {/* Stage 1: System Initialization */}
        <ScrollAnimation animation="slideInRight" triggerPoint={0.3}>
        <div 
          className={styles.storySection}
          ref={el => { cardRefs.current[0] = el; }}
        >
          <Heading as="h2" className={styles.storyTitle}>
          🟢 System Initialization
          </Heading>
          <p className={styles.storyText}>
          Seamlessly set up and launch the complete ROS2 workspace for LIMO Cobot with a single command. Hardware, drivers, and environment configuration are automated for a plug-and-play experience.
          </p>
          <div className={styles.featureList}>
          <div className={styles.feature}>🔗 One-Click Launch</div>
          <div className={styles.feature}>🖥️ Automated Hardware Detection</div>
          <div className={styles.feature}>⚙️ Clean Modular Workspace</div>
          </div>
        </div>
        </ScrollAnimation>

        {/* Stage 2: Autonomous Exploration */}
        <ScrollAnimation animation="slideInRight" triggerPoint={0.3} delay={200}>
        <div 
          className={styles.storySection}
          ref={el => { cardRefs.current[1] = el; }}
        >
          <Heading as="h2" className={styles.storyTitle}>
          🚗 Autonomous Exploration
          </Heading>
          <p className={styles.storyText}>
          LIMO robot explores unknown environments using real-time SLAM, custom waypoints, and robust mapping. It navigates safely, performing rotational scans to ensure complete area coverage.
          </p>
          <div className={styles.featureList}>
          <div className={styles.feature}>🗺️ SLAM Mapping & Cartographer</div>
          <div className={styles.feature}>🔄 Rotational Scanning</div>
          <div className={styles.feature}>🤖 Dynamic Path Planning</div>
          </div>
        </div>
        </ScrollAnimation>

        {/* Stage 3: Real-Time 3D Object Detection */}
        <ScrollAnimation animation="slideInRight" triggerPoint={0.3} delay={400}>
        <div 
          className={styles.storySection}
          ref={el => { cardRefs.current[2] = el; }}
        >
          <Heading as="h2" className={styles.storyTitle}>
          🟦 Real-Time 3D Object Detection
          </Heading>
          <p className={styles.storyText}>
          Integrated YOLOv8 object detection with depth cameras for precise 3D localization. Detected objects are transformed to map frame for actionable robotic responses.
          </p>
          <div className={styles.featureList}>
          <div className={styles.feature}>🟡 YOLOv8 with Custom Dataset</div>
          <div className={styles.feature}>🔢 Depth-Based Localization</div>
          <div className={styles.feature}>📡 Map Frame Integration</div>
          </div>
        </div>
        </ScrollAnimation>

        {/* Stage 4: Intelligent Manipulation */}
        <ScrollAnimation animation="slideInRight" triggerPoint={0.3} delay={600}>
        <div 
          className={styles.storySection}
          ref={el => { cardRefs.current[3] = el; }}
        >
          <Heading as="h2" className={styles.storyTitle}>
          ✋ Intelligent Manipulation
          </Heading>
          <p className={styles.storyText}>
          MyCobot arm executes robust, wireless pick-and-place tasks with automatic pose detection and error handling. Full integration with voice feedback and timeout recovery ensures reliability.
          </p>
          <div className={styles.featureList}>
          <div className={styles.feature}>📶 Wireless Arm Control</div>
          <div className={styles.feature}>🎯 Precise Pick & Drop</div>
          <div className={styles.feature}>🔊 Voice Feedback Integration</div>
          </div>
        </div>
        </ScrollAnimation>

        {/* Stage 5: Fully Autonomous Mission Logic */}
        <ScrollAnimation animation="slideInRight" triggerPoint={0.3} delay={800}>
        <div 
          className={styles.storySection}
          ref={el => { cardRefs.current[4] = el; }}
        >
          <Heading as="h2" className={styles.storyTitle}>
          🚀 Fully Autonomous Mission Logic
          </Heading>
          <p className={styles.storyText}>
          LIMO robot autonomously links all modules: explores, detects, navigates, picks, returns, and drops—no human intervention needed. Robust recovery and advanced launch sequencing make it production-ready.
          </p>
          <div className={styles.featureList}>
          <div className={styles.feature}>🤖 State Machine Orchestration</div>
          <div className={styles.feature}>💡 Recovery & Error Handling</div>
          <div className={styles.feature}>🛠️ Production-Grade Automation</div>
          </div>
          <div className={styles.ctaSection}>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Explore Full Workflow
          </Link>
          <Link
            className="button button--outline button--secondary button--lg"
            to="/docs/intro"
            style={{marginLeft: '1rem'}}>
            Technical Documentation
          </Link>
          </div>
        </div>
        </ScrollAnimation>
      </div>
      </div>
    </div>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title} - Documentation`}
      description="Interactive documentation for LIMO collaborative robotics platform"
      wrapperClassName={styles.mainWrapper}>
      <TechBackground />
      <HomepageHeader />
      <main>
        <RobotStorySection />
      </main>
    </Layout>
  );
}
