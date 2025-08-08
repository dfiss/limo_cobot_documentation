import React from 'react';
import styles from './styles.module.css';

interface StatusDisplayProps {
  stage: number;
}

const statusMessages = [
  "🔌 System Offline - Awaiting Initialization",
  "⚡ Powering Up - Core Systems Online",
  "🔄 Initializing Navigation Systems",
  "🤖 Robotic Systems Active",
  "🌟 All Systems Operational - Ready for Collaboration",
  "🚀 Ultimate Mode - Maximum Performance Achieved"
];

const StatusDisplay: React.FC<StatusDisplayProps> = ({ stage }) => {
  return (
    <div className={styles.statusDisplay}>
      <div className={styles.statusHeader}>
        <div className={styles.statusIndicator}>
          <div className={`${styles.statusLight} ${stage > 0 ? styles.active : ''}`}></div>
        </div>
        <div className={styles.statusTitle}>LIMO COBOT STATUS</div>
      </div>
      <div className={styles.statusMessage}>
        {statusMessages[stage] || statusMessages[0]}
      </div>
      <div className={styles.systemMetrics}>
        <div className={styles.metric}>
          <span className={styles.metricLabel}>Power:</span>
          <div className={styles.metricBar}>
            <div 
              className={styles.metricFill} 
              style={{ width: `${stage >= 5 ? 100 : Math.min(stage * 25, 100)}%` }}
            ></div>
          </div>
          <span className={styles.metricValue}>{stage >= 5 ? 100 : Math.min(stage * 25, 100)}%</span>
        </div>
        <div className={styles.metric}>
          <span className={styles.metricLabel}>Systems:</span>
          <div className={styles.metricBar}>
            <div 
              className={styles.metricFill} 
              style={{ width: `${stage >= 5 ? 100 : Math.min((stage - 1) * 33.33, 100)}%` }}
            ></div>
          </div>
          <span className={styles.metricValue}>{stage >= 5 ? 100 : Math.max(Math.min((stage - 1) * 33.33, 100), 0).toFixed(0)}%</span>
        </div>
      </div>
    </div>
  );
};

export default StatusDisplay;
