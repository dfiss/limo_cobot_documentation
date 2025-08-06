import React, { useEffect, useRef, useState } from 'react';
import styles from './styles.module.css';

interface ScrollAnimationProps {
  children: React.ReactNode;
  triggerPoint?: number;
  animation?: 'fadeInUp' | 'slideInLeft' | 'slideInRight' | 'scaleIn' | 'rotateIn';
  delay?: number;
  duration?: number;
}

export default function ScrollAnimation({
  children,
  triggerPoint = 0.7,
  animation = 'fadeInUp',
  delay = 0,
  duration = 800
}: ScrollAnimationProps) {
  const [isVisible, setIsVisible] = useState(false);
  const elementRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setTimeout(() => {
            setIsVisible(true);
          }, delay);
        }
      },
      {
        threshold: triggerPoint,
        rootMargin: '0px 0px -50px 0px'
      }
    );

    if (elementRef.current) {
      observer.observe(elementRef.current);
    }

    return () => {
      if (elementRef.current) {
        observer.unobserve(elementRef.current);
      }
    };
  }, [triggerPoint, delay]);

  return (
    <div
      ref={elementRef}
      className={`${styles.scrollAnimation} ${styles[animation]} ${
        isVisible ? styles.visible : styles.hidden
      }`}
      style={{
        animationDuration: `${duration}ms`,
        animationDelay: `${delay}ms`
      }}
    >
      {children}
    </div>
  );
}
