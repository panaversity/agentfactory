"use client"

import React, { createContext, useContext, useState, useEffect, ReactNode } from "react"
import { Tabs, TabsList, TabsTrigger, TabsContent } from "@/components/ui/tabs"
import { cn } from "@/lib/utils"

// Icons for each OS
const WindowsIcon = () => (
  <svg className="w-4 h-4" viewBox="0 0 24 24" fill="currentColor">
    <path d="M0 3.449L9.75 2.1v9.451H0m10.949-9.602L24 0v11.4H10.949M0 12.6h9.75v9.451L0 20.699M10.949 12.6H24V24l-12.9-1.801"/>
  </svg>
)

const AppleIcon = () => (
  <svg className="w-4 h-4" viewBox="0 0 24 24" fill="currentColor">
    <path d="M18.71 19.5c-.83 1.24-1.71 2.45-3.05 2.47-1.34.03-1.77-.79-3.29-.79-1.53 0-2 .77-3.27.82-1.31.05-2.3-1.32-3.14-2.53C4.25 17 2.94 12.45 4.7 9.39c.87-1.52 2.43-2.48 4.12-2.51 1.28-.02 2.5.87 3.29.87.78 0 2.26-1.07 3.81-.91.65.03 2.47.26 3.64 1.98-.09.06-2.17 1.28-2.15 3.81.03 3.02 2.65 4.03 2.68 4.04-.03.07-.42 1.44-1.38 2.83M13 3.5c.73-.83 1.94-1.46 2.94-1.5.13 1.17-.34 2.35-1.04 3.19-.69.85-1.83 1.51-2.95 1.42-.15-1.15.41-2.35 1.05-3.11z"/>
  </svg>
)

const LinuxIcon = () => (
  <svg className="w-4 h-4" viewBox="0 0 24 24" fill="currentColor">
    <path d="M12.504 0c-.155 0-.315.008-.48.021-4.226.333-3.105 4.807-3.17 6.298-.076 1.092-.3 1.953-1.05 3.02-.885 1.051-2.127 2.75-2.716 4.521-.278.832-.41 1.684-.287 2.489a.424.424 0 00-.11.135c-.26.268-.45.6-.663.839-.199.199-.485.267-.797.4-.313.136-.658.269-.864.68-.09.189-.136.394-.132.602 0 .199.027.4.055.536.058.399.116.728.04.97-.249.68-.28 1.145-.106 1.484.174.334.535.47.94.601.81.2 1.91.135 2.774.6.926.466 1.866.67 2.616.47.526-.116.97-.464 1.208-.946.587-.003 1.23-.269 2.26-.334.699-.058 1.574.267 2.577.2.025.134.063.198.114.333l.003.003c.391.778 1.113 1.132 1.884 1.071.771-.06 1.592-.536 2.257-1.306.631-.765 1.683-1.084 2.378-1.503.348-.199.629-.469.649-.853.023-.4-.2-.811-.714-1.376v-.097l-.003-.003c-.17-.2-.25-.535-.338-.926-.085-.401-.182-.786-.492-1.046h-.003c-.059-.054-.123-.067-.188-.135a.357.357 0 00-.19-.064c.431-1.278.264-2.55-.173-3.694-.533-1.41-1.465-2.638-2.175-3.483-.796-1.005-1.576-1.957-1.56-3.368.026-2.152.236-6.133-3.544-6.139zm.529 3.405h.013c.213 0 .396.062.584.198.19.135.33.332.438.533.105.259.158.459.166.724 0-.02.006-.04.006-.06v.105a.086.086 0 01-.004-.021l-.004-.024a1.807 1.807 0 01-.15.706.953.953 0 01-.213.335.71.71 0 00-.088-.042c-.104-.045-.198-.064-.284-.133a1.312 1.312 0 00-.22-.066c.05-.06.146-.133.183-.198.053-.128.082-.264.088-.402v-.02a.96.96 0 00-.026-.238c-.028-.129-.074-.249-.137-.36-.063-.105-.14-.2-.225-.282a.964.964 0 00-.32-.186.932.932 0 00-.382-.064c.146-.048.296-.06.45-.06zm-3.023.096c.154-.001.307.012.458.06a.932.932 0 00-.382.064.964.964 0 00-.32.186c-.085.082-.162.177-.225.282a1.15 1.15 0 00-.137.36.96.96 0 00-.026.238v.02c.006.138.035.274.088.402.037.065.133.138.183.198a1.312 1.312 0 00-.22.066c-.086.069-.18.088-.284.133a.71.71 0 00-.088.042.953.953 0 01-.213-.335 1.807 1.807 0 01-.15-.706l-.004.024a.086.086 0 01-.004.021v-.105c0 .02.006.04.006.06.008-.265.061-.465.166-.724.108-.2.248-.398.438-.533.188-.136.371-.198.584-.198h.013zm-4.167 6.69c-.025.143.003.298.068.433.134.27.354.47.564.63.211.165.416.323.541.55a.48.48 0 01.057.196v.003c-.004.063-.034.105-.074.135-.06.056-.166.082-.295.074-.259-.013-.553-.146-.749-.378-.097-.117-.181-.292-.234-.456-.053-.165-.076-.32-.062-.413zm8.302 0c.014.093-.009.248-.062.413-.053.164-.137.339-.234.456-.196.232-.49.365-.749.378-.129.008-.235-.018-.295-.074-.04-.03-.07-.072-.074-.135v-.003a.48.48 0 01.057-.196c.125-.227.33-.385.541-.55.21-.16.43-.36.564-.63.065-.135.093-.29.068-.433zM3.847 12.8c.059-.003.119 0 .18.007.3.039.596.195.87.409.273.213.523.477.727.77.204.292.36.614.45.96.09.35.104.725.012 1.087-.033.13-.08.254-.143.373.164.043.32.11.472.183.153.074.3.164.427.303.14.152.242.362.293.614.05.252.05.552-.007.888-.02.12-.048.238-.084.357-.168.547-.442 1.1-.783 1.621-.17.26-.36.512-.559.746-.395.47-.82.899-1.283 1.237a5.433 5.433 0 01-.766.48c-.333.17-.65.287-.937.338-.29.05-.556.033-.802-.06-.247-.093-.472-.258-.674-.478-.201-.22-.38-.492-.53-.81a6.89 6.89 0 01-.454-1.18 8.924 8.924 0 01-.292-1.308c-.055-.47-.066-.956-.026-1.423.04-.468.133-.925.287-1.335a3.3 3.3 0 01.574-.96c.246-.282.548-.517.9-.68a2.6 2.6 0 011.158-.264h.018zm16.306 0h.018a2.6 2.6 0 011.158.264c.352.163.654.398.9.68.247.283.437.607.574.96.154.41.247.867.287 1.335.04.467.029.953-.026 1.423a8.924 8.924 0 01-.292 1.307 6.89 6.89 0 01-.454 1.18c-.15.32-.329.592-.53.812-.202.22-.427.385-.674.478-.246.093-.512.11-.802.06-.287-.051-.604-.168-.937-.338a5.433 5.433 0 01-.766-.48c-.463-.338-.888-.767-1.283-1.237a8.16 8.16 0 01-.559-.746c-.341-.521-.615-1.074-.783-1.621a3.03 3.03 0 01-.084-.357c-.057-.336-.057-.636-.007-.888.051-.252.153-.462.293-.614.127-.14.274-.229.427-.303.152-.073.308-.14.472-.183a1.78 1.78 0 01-.143-.373c-.092-.362-.078-.738.012-1.088.09-.345.246-.667.45-.96.204-.292.454-.556.727-.769.274-.214.57-.37.87-.409.061-.007.121-.01.18-.007z"/>
  </svg>
)

// Context for syncing tabs across the page
interface TabsSyncContextType {
  activeTab: string
  setActiveTab: (tab: string) => void
}

const TabsSyncContext = createContext<TabsSyncContextType | null>(null)

// Provider component to wrap the page
export function OSTabsProvider({ children, defaultValue = "windows" }: { children: ReactNode, defaultValue?: string }) {
  const [activeTab, setActiveTab] = useState(defaultValue)

  // Persist selection in localStorage
  useEffect(() => {
    const saved = localStorage.getItem("preferred-os")
    if (saved) {
      setActiveTab(saved)
    }
  }, [])

  useEffect(() => {
    localStorage.setItem("preferred-os", activeTab)
  }, [activeTab])

  return (
    <TabsSyncContext.Provider value={{ activeTab, setActiveTab }}>
      {children}
    </TabsSyncContext.Provider>
  )
}

// Hook to use synced tabs
function useTabsSync() {
  const context = useContext(TabsSyncContext)
  if (!context) {
    // Fallback for when not wrapped in provider
    const [activeTab, setActiveTab] = useState("windows")
    return { activeTab, setActiveTab, synced: false }
  }
  return { ...context, synced: true }
}

// Main OSTabs component
interface OSTabsProps {
  children: ReactNode
  className?: string
}

export function OSTabs({ children, className }: OSTabsProps) {
  const { activeTab, setActiveTab } = useTabsSync()

  return (
    <Tabs value={activeTab} onValueChange={setActiveTab} className={cn("w-full", className)}>
      <TabsList className="grid w-full grid-cols-3 mb-4">
        <TabsTrigger value="windows" className="flex items-center gap-2">
          <WindowsIcon />
          <span>Windows</span>
        </TabsTrigger>
        <TabsTrigger value="macos" className="flex items-center gap-2">
          <AppleIcon />
          <span>macOS</span>
        </TabsTrigger>
        <TabsTrigger value="linux" className="flex items-center gap-2">
          <LinuxIcon />
          <span>Linux</span>
        </TabsTrigger>
      </TabsList>
      {children}
    </Tabs>
  )
}

// Tab content components
interface OSTabContentProps {
  children: ReactNode
  className?: string
}

export function WindowsContent({ children, className }: OSTabContentProps) {
  return (
    <TabsContent value="windows" className={className}>
      {children}
    </TabsContent>
  )
}

export function MacOSContent({ children, className }: OSTabContentProps) {
  return (
    <TabsContent value="macos" className={className}>
      {children}
    </TabsContent>
  )
}

export function LinuxContent({ children, className }: OSTabContentProps) {
  return (
    <TabsContent value="linux" className={className}>
      {children}
    </TabsContent>
  )
}

export { TabsContent as OSTabContent }
