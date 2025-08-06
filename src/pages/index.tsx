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
                to="/docs/tutorial-basics/create-a-document"
                style={{marginLeft: '1rem'}}>
                View Documentation
              </Link>
            </div>
          </div>
          <div className={styles.heroRight}>
            <AnimatedRobot animationStage={0} />
            <StatusDisplay stage={0} />
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

  useEffect(() => {
    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            const cardIndex = cardRefs.current.indexOf(entry.target);
            if (cardIndex !== -1) {
              // Position robot to align with each card's position - 3x movement
              // Card 0: 30vh, Card 1: 90vh, Card 2: 150vh, Card 3: 210vh, Card 4: 270vh
              const robotPositions = [30, 90, 150, 210, 270];
              setRobotPosition(robotPositions[cardIndex]);
              setRobotStage(Math.min(cardIndex, 4)); // Keep stage max at 4
            }
          }
        });
      },
      {
        threshold: 0.3, // Trigger when 30% of card is visible
        rootMargin: '-20% 0px -20% 0px' // Only trigger when card is well within viewport
      }
    );

    cardRefs.current.forEach((card) => {
      if (card) observer.observe(card);
    });

    return () => observer.disconnect();
  }, []);

  const handleProgressChange = (_progress: number) => {
    // Keep for backward compatibility but main logic is now in intersection observer
  };

  return (
    <div className={styles.storyContainer}>
      <ScrollProgress onProgressChange={handleProgressChange} />
      
      <div className={styles.storyLayout}>
        <div className={styles.robotColumn}>
          <div 
            className={styles.robotWrapper}
            style={{
              transform: `translateY(${robotPosition}vh)`
            }}
          >
            <AnimatedRobot animationStage={robotStage} />
            <StatusDisplay stage={robotStage} />
          </div>
        </div>
        
        <div className={styles.contentColumn}>
          {/* Stage 1: Robot Awakens */}
          <ScrollAnimation animation="slideInRight" triggerPoint={0.3}>
            <div 
              className={styles.storySection}
              ref={el => { cardRefs.current[0] = el; }}
            >
              <Heading as="h2" className={styles.storyTitle}>
                🔋 Robot Awakens
              </Heading>
              <p className={styles.storyText}>
                LIMO Cobot comes to life with advanced AI systems. Watch as the eyes light up 
                and the power systems initialize, ready for autonomous operation.
              </p>
              <div className={styles.featureList}>
                <div className={styles.feature}>✨ AI-Powered Intelligence</div>
                <div className={styles.feature}>🔋 Advanced Power Management</div>
                <div className={styles.feature}>👀 Computer Vision Systems</div>
              </div>
            </div>
          </ScrollAnimation>

          {/* Stage 2: Autonomous Movement */}
          <ScrollAnimation animation="slideInRight" triggerPoint={0.3} delay={200}>
            <div 
              className={styles.storySection}
              ref={el => { cardRefs.current[1] = el; }}
            >
              <Heading as="h2" className={styles.storyTitle}>
                🚶 Autonomous Movement
              </Heading>
              <p className={styles.storyText}>
                Advanced locomotion systems engage. LIMO Cobot begins its journey with 
                precise navigation and obstacle avoidance capabilities.
              </p>
              <div className={styles.featureList}>
                <div className={styles.feature}>🗺️ SLAM Navigation</div>
                <div className={styles.feature}>🚧 Obstacle Avoidance</div>
                <div className={styles.feature}>📍 Precision Positioning</div>
              </div>
            </div>
          </ScrollAnimation>

          {/* Stage 3: Robotic Manipulation */}
          <ScrollAnimation animation="slideInRight" triggerPoint={0.3} delay={400}>
            <div 
              className={styles.storySection}
              ref={el => { cardRefs.current[2] = el; }}
            >
              <Heading as="h2" className={styles.storyTitle}>
                🤖 Robotic Manipulation
              </Heading>
              <p className={styles.storyText}>
                Sophisticated arm control systems activate. Watch the robot demonstrate 
                precise manipulation capabilities for complex tasks.
              </p>
              <div className={styles.featureList}>
                <div className={styles.feature}>🦾 6-DOF Manipulation</div>
                <div className={styles.feature}>🎯 Precise Control</div>
                <div className={styles.feature}>🔄 Task Automation</div>
              </div>
            </div>
          </ScrollAnimation>

          {/* Stage 4: Full Integration */}
          <ScrollAnimation animation="slideInRight" triggerPoint={0.3} delay={600}>
            <div 
              className={styles.storySection}
              ref={el => { cardRefs.current[3] = el; }}
            >
              <Heading as="h2" className={styles.storyTitle}>
                🌟 Full Integration
              </Heading>
              <p className={styles.storyText}>
                All systems fully operational! LIMO Cobot is now ready for complex 
                collaborative tasks with seamless human-robot interaction.
              </p>
              <div className={styles.featureList}>
                <div className={styles.feature}>🤝 Human-Robot Collaboration</div>
                <div className={styles.feature}>📡 Wireless Communication</div>
                <div className={styles.feature}>🔮 Predictive Analytics</div>
              </div>
              <div className={styles.ctaSection}>
                <Link
                  className="button button--primary button--lg"
                  to="/docs/intro">
                  Start Building with LIMO Cobot
                </Link>
                <Link
                  className="button button--outline button--secondary button--lg"
                  to="/docs/tutorial-basics/create-a-document"
                  style={{marginLeft: '1rem'}}>
                  View Documentation
                </Link>
              </div>
            </div>
          </ScrollAnimation>

          {/* Stage 5: Advanced Operations */}
          <ScrollAnimation animation="slideInRight" triggerPoint={0.3} delay={800}>
            <div 
              className={styles.storySection}
              ref={el => { cardRefs.current[4] = el; }}
            >
              <Heading as="h2" className={styles.storyTitle}>
                🚀 Advanced Operations
              </Heading>
              <p className={styles.storyText}>
                LIMO Cobot reaches peak performance with advanced AI capabilities, 
                real-time decision making, and seamless integration with industrial systems.
              </p>
              <div className={styles.featureList}>
                <div className={styles.feature}>🧠 Advanced AI Decision Making</div>
                <div className={styles.feature}>⚡ Real-time Processing</div>
                <div className={styles.feature}>🏭 Industrial Integration</div>
              </div>
              <div className={styles.ctaSection}>
                <Link
                  className="button button--primary button--lg"
                  to="/docs/intro">
                  Explore Advanced Features
                </Link>
                <Link
                  className="button button--outline button--secondary button--lg"
                  to="/docs/tutorial-basics/create-a-document"
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
