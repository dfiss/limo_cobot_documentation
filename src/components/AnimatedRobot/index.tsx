import React, { useEffect, useState, useRef } from 'react';
import styles from './styles.module.css';

interface AnimatedRobotProps {
  className?: string;
  animationStage?: number;
}

export default function AnimatedRobot({ className = '', animationStage = 0 }: AnimatedRobotProps) {
  const [currentStage, setCurrentStage] = useState(0);
  const robotRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    setCurrentStage(animationStage);
  }, [animationStage]);

  return (
    <div ref={robotRef} className={`${styles.robotContainer} ${className}`}>
      {/* Main Robot Image */}
      <div className={styles.robotImageWrapper}>
        <img
          src="/img/limo_cobot.svg"
          alt="LIMO Cobot Robot"
          className={`${styles.robotImage} ${styles[`stage${currentStage}`]}`}
        />
      </div>

      {/* Animation Overlays */}
      <div className={styles.animationOverlays}>
        {/* Glowing Eyes Effect */}
        {currentStage >= 1 && (
          <div className={`${styles.eyeGlow} ${styles.leftEye}`}></div>
        )}
        {currentStage >= 1 && (
          <div className={`${styles.eyeGlow} ${styles.rightEye}`}></div>
        )}

        {/* Power Status Indicator */}
        {currentStage >= 1 && (
          <div className={styles.powerIndicator}>
            <div className={styles.powerLight}></div>
          </div>
        )}

        {/* System Status Bars */}
        {currentStage >= 2 && (
          <div className={styles.statusPanel}>
            <div className={`${styles.statusBar} ${styles.bar1}`}></div>
            <div className={`${styles.statusBar} ${styles.bar2} ${currentStage >= 3 ? styles.active : ''}`}></div>
            <div className={`${styles.statusBar} ${styles.bar3} ${currentStage >= 4 ? styles.active : ''}`}></div>
          </div>
        )}

        {/* Antenna Beacon */}
        {currentStage >= 4 && (
          <div className={styles.antennaBeacon}>
            <div className={styles.beacon}></div>
            <div className={styles.beaconPulse}></div>
          </div>
        )}

        {/* Particle Effects */}
        {currentStage >= 4 && (
          <div className={styles.particles}>
            <div className={`${styles.particle} ${styles.particle1}`}></div>
            <div className={`${styles.particle} ${styles.particle2}`}></div>
            <div className={`${styles.particle} ${styles.particle3}`}></div>
            <div className={`${styles.particle} ${styles.particle4}`}></div>
            <div className={`${styles.particle} ${styles.particle5}`}></div>
          </div>
        )}

        {/* Holographic Elements */}
        {currentStage >= 3 && (
          <div className={styles.holographicUI}>
            <div className={`${styles.hologram} ${styles.leftHolo}`}>
              <div className={styles.holoText}>NAV</div>
              <div className={styles.holoIndicator}></div>
            </div>
            <div className={`${styles.hologram} ${styles.rightHolo}`}>
              <div className={styles.holoText}>ARM</div>
              <div className={styles.holoIndicator}></div>
            </div>
          </div>
        )}

        {/* Energy Field */}
        {currentStage >= 4 && (
          <div className={styles.energyField}>
            <div className={styles.energyRing}></div>
            <div className={`${styles.energyRing} ${styles.ring2}`}></div>
          </div>
        )}
      </div>
    </div>
  );
}
