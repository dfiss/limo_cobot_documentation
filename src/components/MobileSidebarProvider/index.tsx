import React from 'react';
import { MobilePhoneSidebar } from '@site/src/components/MobilePhoneSidebar';

// Global style to adjust content when mobile sidebar is active
const sidebarStyles = `
  .docs-wrapper {
    margin-left: 300px !important;
  }
  
  @media (max-width: 996px) {
    .docs-wrapper {
      margin-left: 0 !important;
    }
  }
`;

const MobileSidebarProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  React.useEffect(() => {
    // Add global styles for content adjustment
    const style = document.createElement('style');
    style.textContent = sidebarStyles;
    document.head.appendChild(style);

    // Clean up on unmount
    return () => {
      if (style.parentNode) {
        style.parentNode.removeChild(style);
      }
    };
  }, []);

  return (
    <>
      <MobilePhoneSidebar />
      {children}
    </>
  );
};

export default MobileSidebarProvider;
