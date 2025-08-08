import React, { useEffect, useState, useRef } from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

interface AnimatedRobotProps {
  className?: string;
  animationStage?: number;
}

export default function AnimatedRobot({ className = '', animationStage = 0 }: AnimatedRobotProps) {
  const [currentStage, setCurrentStage] = useState(0);
  const robotRef = useRef<HTMLDivElement>(null);
  const logoUrl = useBaseUrl('/img/limo_cobot.svg');

  useEffect(() => {
    setCurrentStage(animationStage);
  }, [animationStage]);

  return (
    <div ref={robotRef} className={`${styles.robotContainer} ${className}`}>
      {/* Futuristic Background Grid */}
      <div className={styles.quantumGrid}>
        <div className={styles.gridLine}></div>
        <div className={styles.gridLine}></div>
        <div className={styles.gridLine}></div>
        <div className={styles.gridLine}></div>
      </div>

      {/* Plasma Energy Field */}
      <div className={styles.plasmaField}>
        <div className={styles.plasmaOrb}></div>
        <div className={`${styles.plasmaOrb} ${styles.orb2}`}></div>
        <div className={`${styles.plasmaOrb} ${styles.orb3}`}></div>
      </div>

      {/* Main Robot Image */}
      <div className={styles.robotImageWrapper}>
        <img
          src={logoUrl}
          alt="LIMO Cobot Robot"
          className={`${styles.robotImage} ${styles[`stage${currentStage}`]}`}
        />
        
        {/* Futuristic Scanner Lines */}
        {currentStage >= 1 && (
          <div className={styles.scannerLines}>
            <div className={styles.scanLine}></div>
            <div className={`${styles.scanLine} ${styles.scanLine2}`}></div>
          </div>
        )}
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

        {/* Neural Network Visualization */}
        {currentStage >= 3 && (
          <div className={styles.neuralNetwork}>
            <div className={styles.neuralNode}></div>
            <div className={`${styles.neuralNode} ${styles.node2}`}></div>
            <div className={`${styles.neuralNode} ${styles.node3}`}></div>
            <div className={`${styles.neuralNode} ${styles.node4}`}></div>
            <div className={styles.neuralConnection}></div>
            <div className={`${styles.neuralConnection} ${styles.connection2}`}></div>
          </div>
        )}

        {/* Antenna Beacon */}
        {currentStage >= 4 && (
          <div className={styles.antennaBeacon}>
            <div className={styles.beacon}></div>
            <div className={styles.beaconPulse}></div>
          </div>
        )}

        {/* Enhanced Particle Effects */}
        {currentStage >= 4 && (
          <div className={styles.particles}>
            <div className={`${styles.particle} ${styles.particle1}`}></div>
            <div className={`${styles.particle} ${styles.particle2}`}></div>
            <div className={`${styles.particle} ${styles.particle3}`}></div>
            <div className={`${styles.particle} ${styles.particle4}`}></div>
            <div className={`${styles.particle} ${styles.particle5}`}></div>
            <div className={`${styles.particle} ${styles.particle6}`}></div>
            <div className={`${styles.particle} ${styles.particle7}`}></div>
            <div className={`${styles.particle} ${styles.particle8}`}></div>
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
            <div className={`${styles.hologram} ${styles.topHolo}`}>
              <div className={styles.holoText}>AI</div>
              <div className={styles.holoIndicator}></div>
            </div>
          </div>
        )}

        {/* Energy Field */}
        {currentStage >= 4 && (
          <div className={styles.energyField}>
            <div className={styles.energyRing}></div>
            <div className={`${styles.energyRing} ${styles.ring2}`}></div>
            <div className={`${styles.energyRing} ${styles.ring3}`}></div>
          </div>
        )}

        {/* Quantum Effects */}
        {currentStage >= 4 && (
          <div className={styles.quantumEffects}>
            <div className={styles.quantumParticle}></div>
            <div className={`${styles.quantumParticle} ${styles.qp2}`}></div>
            <div className={`${styles.quantumParticle} ${styles.qp3}`}></div>
          </div>
        )}
      </div>
    </div>
  );
}
