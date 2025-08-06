import React from 'react';
import styles from './styles.module.css';

const TechBackground: React.FC = () => {
  return (
    <div className={styles.techBackground}>
      <div className={styles.gridPattern}></div>
      <div className={styles.scanlines}></div>
      <div className={styles.floatingElements}>
        {Array.from({ length: 20 }, (_, i) => (
          <div
            key={i}
            className={styles.floatingDot}
            style={{
              left: `${Math.random() * 100}%`,
              top: `${Math.random() * 100}%`,
              animationDelay: `${Math.random() * 10}s`,
              animationDuration: `${8 + Math.random() * 4}s`,
            }}
          />
        ))}
      </div>
      <div className={styles.dataStreams}>
        <div className={styles.dataStream}></div>
        <div className={styles.dataStream}></div>
        <div className={styles.dataStream}></div>
      </div>
    </div>
  );
};

export default TechBackground;
