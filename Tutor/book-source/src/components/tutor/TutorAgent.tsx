/**
 * TutorAgent Container Component
 *
 * Main container that coordinates the sidebar, selection popover, and chat
 * Manages shared state between all tutor components
 */

import React, { useState, useCallback, useEffect } from 'react';
import AgentSidebar from './AgentSidebar';
import SelectionPopover from './SelectionPopover';
import type { ChatMessage } from '@/utils/agentApi';

const TutorAgent: React.FC = () => {
  const [isMounted, setIsMounted] = useState(false);
  const [chatMessages, setChatMessages] = useState<ChatMessage[]>([]);
  const [prefillText, setPrefillText] = useState<string>('');

  // Only render on client-side to avoid SSR issues
  useEffect(() => {
    setIsMounted(true);
  }, []);

  // Handle new messages from various sources
  const handleNewMessage = useCallback((message: ChatMessage) => {
    setChatMessages((prev) => [...prev, message]);
  }, []);

  // Handle opening chat with prefilled text from selection popover
  const handleOpenChat = useCallback((text: string) => {
    setPrefillText(text);
    // The prefill text will be used in the ChatWindow component
    // We'll need to pass this down through the AgentSidebar
  }, []);

  // Don't render on server-side
  if (!isMounted) {
    return null;
  }

  return (
    <>
      {/* Agent Sidebar - Fixed on the left */}
      <AgentSidebar onChatMessage={(message) => console.log('Chat message:', message)} />

      {/* Selection Popover - Appears when text is selected */}
      <SelectionPopover onOpenChat={handleOpenChat} onNewMessage={handleNewMessage} />
    </>
  );
};

export default TutorAgent;
