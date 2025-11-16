import React, { createContext, useContext, useState, useCallback, ReactNode } from 'react';

interface SidebarContextType {
  collapseSidebar: () => void;
  expandSidebar: () => void;
  isSidebarCollapsed: boolean;
}

const SidebarContext = createContext<SidebarContextType | undefined>(undefined);

export const SidebarProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [isSidebarCollapsed, setIsSidebarCollapsed] = useState(false);

  const collapseSidebar = useCallback(() => {
    setIsSidebarCollapsed(true);
    // Dispatch custom event to trigger sidebar collapse
    window.dispatchEvent(new CustomEvent('collapseSidebar'));
  }, []);

  const expandSidebar = useCallback(() => {
    setIsSidebarCollapsed(false);
    // Dispatch custom event to trigger sidebar expand
    window.dispatchEvent(new CustomEvent('expandSidebar'));
  }, []);

  return (
    <SidebarContext.Provider value={{ collapseSidebar, expandSidebar, isSidebarCollapsed }}>
      {children}
    </SidebarContext.Provider>
  );
};

export const useSidebarControl = (): SidebarContextType => {
  const context = useContext(SidebarContext);
  if (context === undefined) {
    throw new Error('useSidebarControl must be used within a SidebarProvider');
  }
  return context;
};
