import React, { useEffect, useState } from 'react';
import styles from './styles.module.css';

interface ScrollProgressProps {
  onProgressChange?: (progress: number) => void;
}

export default function ScrollProgress({ onProgressChange }: ScrollProgressProps) {
  const [scrollProgress, setScrollProgress] = useState(0);

  useEffect(() => {
    const handleScroll = () => {
      const totalHeight = document.documentElement.scrollHeight - window.innerHeight;
      const progress = (window.scrollY / totalHeight) * 100;
      setScrollProgress(progress);
      
      if (onProgressChange) {
        onProgressChange(progress);
      }
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, [onProgressChange]);

  return (
    <div className={styles.scrollProgress}>
      <div 
        className={styles.progressBar}
        style={{ width: `${scrollProgress}%` }}
      />
    </div>
  );
}
