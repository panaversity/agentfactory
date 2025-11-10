/**
 * TutorAgent Container Component
 *
 * Coordinates the floating icon, selection popover, and existing TutorChat
 * All actions now use the existing TutorChat component
 */

import React, { useState, useCallback, useEffect } from 'react';
import AgentSidebar from './AgentSidebar';
import SelectionPopover from './SelectionPopover';
import type { ChatMessage } from '@/utils/agentApi';

const TutorAgent: React.FC = () => {
  const [isMounted, setIsMounted] = useState(false);

  // Only render on client-side to avoid SSR issues
  useEffect(() => {
    setIsMounted(true);
  }, []);

  // Handle opening TutorChat
  const handleOpenChat = useCallback((text: string, autoSend: boolean = false) => {
    // Dispatch custom event to open TutorChat
    const event = new CustomEvent('openTutorChat', {
      detail: { text, autoSend }
    });
    window.dispatchEvent(event);
  }, []);

  // Handle new messages - send to TutorChat with auto-send
  const handleNewMessage = useCallback((message: ChatMessage) => {
    // For Summary, Explain, etc., open TutorChat and auto-send
    handleOpenChat(message.content, true);
  }, [handleOpenChat]);

  // Don't render on server-side
  if (!isMounted) {
    return null;
  }

  return (
    <>
      {/* Floating Icon - Opens TutorChat when clicked */}
      <AgentSidebar onClick={() => handleOpenChat('')} />

      {/* Selection Popover - Appears when text is selected */}
      <SelectionPopover
        onOpenChat={handleOpenChat}
        onNewMessage={handleNewMessage}
      />
    </>
  );
};

export default TutorAgent;
