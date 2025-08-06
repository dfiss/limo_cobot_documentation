import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Autonomous Exploration',
    Svg: require('@site/static/img/rtab.svg').default,
    description: (
      <>
        Real-time mapping, navigation, and dynamic waypoint exploration using ROS2 SLAM and Nav2. The robot explores new environments autonomously, with robust mapping and localization.
      </>
    ),
  },
  {
    title: '3D Object Detection',
    Svg: require('@site/static/img/yolo.svg').default,
    description: (
      <>
        YOLO-based visual perception with depth integration for real-world object localization. Detects objects, transforms coordinates to map frame, and triggers intelligent robot behavior.
      </>
    ),
  },
  {
    title: 'Robotic Manipulation',
    Svg: require('@site/static/img/arm.svg').default,
    description: (
      <>
        Full MyCobot arm control via wireless setup. Executes modular pick-and-drop routines with voice feedback and robust error handling—seamlessly integrated with detection and navigation.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className={styles.featureCard}>
        <div className={clsx('text--center', styles.featureSvgContainer)}>
          <Svg className={styles.featureSvg} role="img" />
        </div>
        <div className="text--center">
          <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
          <p className={styles.featureDescription}>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className={styles.animatedBackground}></div>
      <div className={styles.animatedBackground}></div>
      <div className={styles.animatedBackground}></div>
      <div className="container">
        <div className={styles.sectionTitle}>
          <Heading as="h2">Powerful Features</Heading>
          <p className={styles.sectionSubtitle}>
            Advanced robotics capabilities designed for the future of automation
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
