import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

interface NavItem {
  id: string;
  href: string;
  label: string;
  icon: string;
}

const navigationItems: NavItem[] = [
  { id: 'intro', href: '/docs/intro', label: 'Introduction', icon: '🚀' },
  { id: 'overview', href: '/docs/project-overview', label: 'Project Overview', icon: '📋' },
  { id: 'hardware', href: '/docs/hardware-setup', label: 'Hardware Setup', icon: '🔧' },
  { id: 'software', href: '/docs/software-stack', label: 'Software Stack', icon: '💻' },
  { id: 'control', href: '/docs/control-algorithms', label: 'Control Systems', icon: '⚙️' },
  { id: 'perception', href: '/docs/perception-systems', label: 'Perception', icon: '👁️' },
  { id: 'planning', href: '/docs/path-planning', label: 'Path Planning', icon: '🗺️' },
  { id: 'integration', href: '/docs/system-integration', label: 'Integration', icon: '🔗' },
  { id: 'testing', href: '/docs/testing-validation', label: 'Testing', icon: '🧪' },
  { id: 'deployment', href: '/docs/deployment', label: 'Deployment', icon: '🚢' },
  { id: 'monitoring', href: '/docs/monitoring-maintenance', label: 'Monitoring', icon: '📊' },
  { id: 'troubleshooting', href: '/docs/troubleshooting', label: 'Troubleshooting', icon: '🔍' },
  { id: 'appendices', href: '/docs/appendices', label: 'Appendices', icon: '📚' },
];

const MobilePhoneSidebar: React.FC = () => {
  const location = useLocation();
  const [isVisible, setIsVisible] = useState(true);

  const isItemActive = (item: NavItem): boolean => {
    return location.pathname.includes(item.href);
  };

  // Hide on homepage
  useEffect(() => {
    setIsVisible(location.pathname.startsWith('/docs'));
  }, [location.pathname]);

  if (!isVisible) return null;

  return (
    <div className={styles.sidebarContainer}>
      <div className={styles.phoneFrame}>
        <div className={styles.phoneScreen}>
          {/* Status Bar */}
          <div className={styles.statusBar}>
            <div className={styles.statusLeft}>
              <span className={styles.time}>9:41</span>
            </div>
            <div className={styles.statusRight}>
              <div className={styles.signal}></div>
              <div className={styles.wifi}></div>
              <div className={styles.battery}>
                <div className={styles.batteryLevel}></div>
              </div>
            </div>
          </div>

          {/* App Header */}
          <div className={styles.appHeader}>
            <div className={styles.appIcon}>🤖</div>
            <div className={styles.appTitle}>LIMO Cobot</div>
            <div className={styles.appSubtitle}>Documentation</div>
          </div>

          {/* Navigation List */}
          <div className={styles.navList}>
            {navigationItems.map((item) => (
              <Link
                key={item.id}
                to={item.href}
                className={`${styles.navItem} ${isItemActive(item) ? styles.navItemActive : ''}`}
              >
                <div className={styles.navIcon}>{item.icon}</div>
                <div className={styles.navLabel}>{item.label}</div>
                <div className={styles.navChevron}>›</div>
              </Link>
            ))}
          </div>

          {/* Home Indicator */}
          <div className={styles.homeIndicator}></div>
        </div>
      </div>
    </div>
  );
};

export default MobilePhoneSidebar;
