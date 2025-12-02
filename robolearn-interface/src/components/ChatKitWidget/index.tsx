/**
 * ChatKit Floating Widget Component
 * 
 * A floating chat button that opens/closes ChatKit like standard chat agents.
 * Features:
 * - Floating button icon (bottom-right corner)
 * - ChatKit opens/closes on click
 * - Text selection support (Select Text and ASK)
 * - Page context awareness
 * - Script loading detection (matches starter app pattern)
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import { useAuth } from '@/contexts/AuthContext';
import { getOAuthAuthorizationUrl } from '@/lib/auth-client';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

const isBrowser = typeof window !== 'undefined';

interface ChatKitWidgetProps {
  backendUrl?: string;
}

export function ChatKitWidget({ backendUrl }: ChatKitWidgetProps): React.ReactElement {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedText, setSelectedText] = useState<string>('');
  const [selectionPosition, setSelectionPosition] = useState<{ x: number; y: number } | null>(null);
  const [showPersonalizeMenu, setShowPersonalizeMenu] = useState(false);
  const [scriptStatus, setScriptStatus] = useState<'pending' | 'ready' | 'error'>(
    isBrowser && window.customElements?.get('openai-chatkit') ? 'ready' : 'pending'
  );
  const selectionRef = useRef<HTMLDivElement>(null);
  const personalizeMenuRef = useRef<HTMLDivElement>(null);
  const isMountedRef = useRef(true);
  const { session, isLoading: authLoading } = useAuth();
  const { siteConfig } = useDocusaurusContext();
  
  // Get backend URL from props, siteConfig, or default
  // Note: process.env is not available in browser, use siteConfig.customFields instead
  const effectiveBackendUrl = backendUrl || 
    (siteConfig.customFields?.backendUrl as string) || 
    'http://localhost:8000';
  
  // Get auth URL and OAuth client ID for login redirect
  const authUrl = (siteConfig.customFields?.authUrl as string) || 'http://localhost:3001';
  const oauthClientId = (siteConfig.customFields?.oauthClientId as string) || 'robolearn-interface';
  
  // Domain key for ChatKit (required for whitelabeled domains)
  // Read from siteConfig.customFields (set via CHATKIT_DOMAIN_KEY env var in deployment)
  // Trim whitespace and fallback to dummy value if not set
  const domainKey = (siteConfig.customFields?.chatkitDomainKey as string)?.trim() || 'domain_pk_local_dev';
  
  // Check if user is logged in
  const isLoggedIn = !!session?.user?.id;
  
  // Handle login redirect
  const handleLogin = useCallback(async () => {
    const oauthConfig = {
      authUrl,
      clientId: oauthClientId,
    };
    const authorizationUrl = await getOAuthAuthorizationUrl('signin', oauthConfig);
    // Small delay to ensure localStorage write is flushed before navigation
    await new Promise(resolve => setTimeout(resolve, 50));
    window.location.href = authorizationUrl;
  }, [authUrl, oauthClientId]);
  
  // Check if ChatKit script is loaded (matches starter app pattern)
  useEffect(() => {
    if (!isBrowser) return;

    let timeoutId: number | undefined;

    const handleLoaded = () => {
      if (!isMountedRef.current) return;
      setScriptStatus('ready');
    };

    const handleError = (event: Event) => {
      console.error('Failed to load chatkit.js', event);
      if (!isMountedRef.current) return;
      setScriptStatus('error');
      const detail = (event as CustomEvent<unknown>)?.detail ?? 'unknown error';
      console.error('ChatKit script error:', detail);
    };

    window.addEventListener('chatkit-script-loaded', handleLoaded);
    window.addEventListener('chatkit-script-error', handleError as EventListener);

    if (window.customElements?.get('openai-chatkit')) {
      handleLoaded();
    } else if (scriptStatus === 'pending') {
      timeoutId = window.setTimeout(() => {
        if (!window.customElements?.get('openai-chatkit')) {
          handleError(
            new CustomEvent('chatkit-script-error', {
              detail: 'ChatKit web component is unavailable. Verify that the script URL is reachable.',
            })
          );
        }
      }, 5000);
    }

    return () => {
      window.removeEventListener('chatkit-script-loaded', handleLoaded);
      window.removeEventListener('chatkit-script-error', handleError as EventListener);
      if (timeoutId) {
        window.clearTimeout(timeoutId);
      }
    };
  }, [scriptStatus]);

  useEffect(() => {
    return () => {
      isMountedRef.current = false;
    };
  }, []);

  // Get current page context - enhanced for better awareness
  const getPageContext = useCallback(() => {
    if (typeof window === 'undefined') return null;
    
    // Extract page metadata
    const metaDescription = document.querySelector('meta[name="description"]')?.getAttribute('content') || '';
    const metaKeywords = document.querySelector('meta[name="keywords"]')?.getAttribute('content') || '';
    
    // Get main content (try to find article or main content area)
    const mainContent = document.querySelector('article') || 
                       document.querySelector('main') || 
                       document.querySelector('[role="main"]') ||
                       document.body;
    
    // Extract headings for context
    const headings = Array.from(mainContent.querySelectorAll('h1, h2, h3'))
      .slice(0, 5)
      .map(h => h.textContent?.trim())
      .filter(Boolean)
      .join(', ');
    
    return {
      url: window.location.href,
      title: document.title,
      path: window.location.pathname,
      description: metaDescription,
      keywords: metaKeywords,
      headings: headings,
      timestamp: new Date().toISOString(),
    };
  }, []);

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (!selection || selection.rangeCount === 0) {
        setSelectedText('');
        setSelectionPosition(null);
        return;
      }

      const selectedText = selection.toString().trim();
      if (selectedText.length > 0) {
        setSelectedText(selectedText);
        
        // Get selection position for "Ask" button placement
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        setSelectionPosition({
          x: rect.left + rect.width / 2,
          y: rect.top - 10, // Position above selection
        });
      } else {
        setSelectedText('');
        setSelectionPosition(null);
      }
    };

    document.addEventListener('selectionchange', handleSelection);
    document.addEventListener('mouseup', handleSelection);
    
    return () => {
      document.removeEventListener('selectionchange', handleSelection);
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // ChatKit configuration
  const { control, sendUserMessage } = useChatKit({
    api: {
      // Custom backend URL - ChatKit will send requests here
      url: `${effectiveBackendUrl}/chatkit`,
      // Domain key (required by API type, but not used for custom backends)
      domainKey: domainKey,
      
      // Custom fetch to inject authentication headers and page context
      fetch: async (url: string, options: RequestInit) => {
        // Require login - don't allow anonymous users
        if (!isLoggedIn) {
          throw new Error('User must be logged in to use chat');
        }
        
        const userId = session!.user!.id; // Safe to use ! since we checked isLoggedIn
        const user = session!.user!;
        const pageContext = getPageContext();
        
        // Build user info for agent awareness
        const userInfo = {
          id: user.id,
          name: user.name || user.email || 'User',
          email: user.email,
          role: user.role,
          softwareBackground: user.softwareBackground,
          hardwareTier: user.hardwareTier,
        };
        
        // Modify request body to add metadata if needed
        let modifiedOptions = { ...options };
        if (modifiedOptions.body && typeof modifiedOptions.body === 'string') {
          try {
            const parsed = JSON.parse(modifiedOptions.body);
            if (parsed.type === 'threads.create' && parsed.params?.input) {
              parsed.params.input.metadata = {
                userId: userId,
                userInfo: userInfo, // Add user info for agent awareness
                pageContext: pageContext, // Add page context to metadata
                ...parsed.params.input.metadata,
              };
              modifiedOptions.body = JSON.stringify(parsed);
            } else if (parsed.type === 'threads.run' && parsed.params?.input) {
              // Add user info and page context to run requests as well
              if (!parsed.params.input.metadata) {
                parsed.params.input.metadata = {};
              }
              parsed.params.input.metadata.userInfo = userInfo;
              parsed.params.input.metadata.pageContext = pageContext;
              modifiedOptions.body = JSON.stringify(parsed);
            }
          } catch (error) {
            // Ignore if not JSON
          }
        }
        
        return fetch(url, {
          ...modifiedOptions,
          headers: {
            ...modifiedOptions.headers,
            'X-User-ID': userId,
            'X-Page-URL': pageContext?.url || '',
            'X-Page-Title': pageContext?.title || '',
            'X-Page-Path': pageContext?.path || '',
            'Content-Type': 'application/json',
          },
        });
      },
    },
    theme: {
      colorScheme: 'dark',
    },
    startScreen: {
      greeting: 'Welcome to RoboLearn AI Assistant!',
      prompts: [
        {
          label: 'Learn ROS 2',
          prompt: 'Help me understand ROS 2 fundamentals',
          icon: 'sparkle',
        },
        {
          label: 'Simulation Help',
          prompt: 'Explain Gazebo or Isaac Sim',
          icon: 'square-code',
        },
        {
          label: 'Hardware Setup',
          prompt: 'Help with hardware tier setup',
          icon: 'circle-question',
        },
      ],
    },
    composer: {
      placeholder: 'Ask anything about robotics and Physical AI...',
    },
    onError: ({ error }) => {
      console.error('ChatKit error:', error);
    },
  });

  // Handle "Ask" button click for selected text - Simplified to send directly to chat
  const handleAskSelectedText = useCallback(async () => {
    if (!selectedText || !isLoggedIn) return;

    // Open chat first if not already open
    if (!isOpen) {
      setIsOpen(true);
      // Wait for ChatKit to initialize
      await new Promise(resolve => setTimeout(resolve, 300));
    }

    // Build simple message with selected text
    const pageContext = getPageContext();
    let messageText = '';
    
    // Simpler, more natural message format
    if (pageContext) {
      messageText = `Can you explain this from "${pageContext.title}":\n\n"${selectedText}"`;
    } else {
      messageText = `Can you explain this:\n\n"${selectedText}"`;
    }

    // Send message using sendUserMessage hook
    try {
      if (sendUserMessage) {
        await sendUserMessage({
          text: messageText,
          newThread: false, // Continue current thread
        });
        
        // Clear selection after successful send
        setTimeout(() => {
          window.getSelection()?.removeAllRanges();
          setSelectedText('');
          setSelectionPosition(null);
        }, 200);
      } else {
        // Fallback: Insert text into ChatKit input field directly
        await new Promise(resolve => setTimeout(resolve, 500)); // Wait for ChatKit to render
        
        const chatkitElement = document.querySelector('openai-chatkit');
        if (chatkitElement) {
          // Try shadow DOM first
          let input: HTMLTextAreaElement | HTMLInputElement | null = null;
          
          if (chatkitElement.shadowRoot) {
            input = chatkitElement.shadowRoot.querySelector('textarea, input[type="text"]') as HTMLTextAreaElement | HTMLInputElement;
          }
          
          // If not in shadow DOM, try regular DOM
          if (!input) {
            input = chatkitElement.querySelector('textarea, input[type="text"]') as HTMLTextAreaElement | HTMLInputElement;
          }
          
          if (input) {
            // Set value and trigger input event
            input.value = messageText;
            input.dispatchEvent(new Event('input', { bubbles: true }));
            input.dispatchEvent(new Event('change', { bubbles: true }));
            
            // Try to submit the form
            const form = input.closest('form');
            if (form) {
              const submitEvent = new Event('submit', { bubbles: true, cancelable: true });
              form.dispatchEvent(submitEvent);
              
              // Also try clicking send button
              const sendButton = chatkitElement.shadowRoot?.querySelector('button[type="submit"], button[aria-label*="send" i]') as HTMLButtonElement;
              if (sendButton) {
                sendButton.click();
              }
            }
            
            // Clear selection after inserting
            setTimeout(() => {
              window.getSelection()?.removeAllRanges();
              setSelectedText('');
              setSelectionPosition(null);
            }, 200);
          } else {
            console.warn('Could not find ChatKit input field');
          }
        }
      }
    } catch (error) {
      console.error('Failed to send message:', error);
      // Keep selection visible so user can try again
    }
  }, [selectedText, isOpen, isLoggedIn, control, sendUserMessage, getPageContext]);

  // Close ChatKit when clicking outside (but not when clicking Ask button)
  useEffect(() => {
    if (!isOpen) return;

    const handleClickOutside = (event: MouseEvent) => {
      const target = event.target as HTMLElement;
      
      // Don't close if clicking inside ChatKit or the button
      if (
        target.closest(`.${styles.chatKitContainer}`) ||
        target.closest(`.${styles.chatButton}`)
      ) {
        return;
      }
      
      setIsOpen(false);
    };

    // Small delay to avoid immediate close on open
    const timeoutId = setTimeout(() => {
      document.addEventListener('mousedown', handleClickOutside);
    }, 100);

    return () => {
      clearTimeout(timeoutId);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  // Close on outside click for text selection
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (selectionRef.current && !selectionRef.current.contains(event.target as Node)) {
        // Don't close if clicking inside ChatKit
        const target = event.target as HTMLElement;
        if (target.closest(`.${styles.chatKitContainer}`)) {
          return;
        }
        setSelectedText('');
        setSelectionPosition(null);
      }
    };

    if (selectedText) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => document.removeEventListener('mousedown', handleClickOutside);
    }
  }, [selectedText]);

  // Hide scroll percentage indicator ("Scroll 0%") after ChatKit renders - AGGRESSIVE
  useEffect(() => {
    if (!isOpen || scriptStatus !== 'ready') return;

    // Note: We rely on JavaScript to find and hide the element since CSS :has-text() doesn't exist

    const hideScrollIndicator = () => {
      const container = document.querySelector(`.${styles.chatKitContainer}`);
      if (!container) return;

      const findAndHide = () => {
        // Find all elements within the container (including shadow DOM if accessible)
        const allElements = container.querySelectorAll('*');
        
        allElements.forEach((el) => {
          const htmlEl = el as HTMLElement;
          const text = (htmlEl.textContent || '').trim();
          const innerHTML = htmlEl.innerHTML || '';
          
          // Match "Scroll 0%" or similar patterns (case insensitive, with or without colon)
          const scrollPattern = /Scroll\s*:?\s*\d+%?/i;
          const hasScrollText = scrollPattern.test(text) || 
                               scrollPattern.test(innerHTML) ||
                               (text.includes('Scroll') && text.includes('%')) ||
                               (innerHTML.includes('Scroll') && innerHTML.includes('%'));
          
          if (hasScrollText) {
            // Multiple hiding strategies
            htmlEl.style.cssText = `
              display: none !important;
              visibility: hidden !important;
              opacity: 0 !important;
              height: 0 !important;
              width: 0 !important;
              overflow: hidden !important;
              position: absolute !important;
              left: -9999px !important;
              pointer-events: none !important;
              font-size: 0 !important;
              line-height: 0 !important;
            `;
            htmlEl.setAttribute('aria-hidden', 'true');
            
            // Also try to remove the element entirely
            try {
              htmlEl.remove();
            } catch (e) {
              // Ignore if can't remove
            }
          }
          
          // Also check for common scroll indicator class/id patterns
          const className = String(htmlEl.className || '');
          const id = String(htmlEl.id || '');
          if (
            /scroll/i.test(className) || 
            /progress/i.test(className) ||
            /indicator/i.test(className) ||
            /scroll/i.test(id) ||
            /progress/i.test(id) ||
            /indicator/i.test(id)
          ) {
            htmlEl.style.cssText += `
              display: none !important;
              visibility: hidden !important;
              opacity: 0 !important;
            `;
          }
        });
        
        // Also check text nodes directly using TreeWalker
        const walker = document.createTreeWalker(
          container,
          NodeFilter.SHOW_TEXT,
          {
            acceptNode: (node) => {
              const text = node.textContent || '';
              if (/Scroll\s*:?\s*\d+%?/i.test(text) || (text.includes('Scroll') && text.includes('%'))) {
                return NodeFilter.FILTER_ACCEPT;
              }
              return NodeFilter.FILTER_REJECT;
            }
          }
        );
        
        let textNode;
        while ((textNode = walker.nextNode())) {
          const parent = textNode.parentElement;
          if (parent) {
            parent.style.cssText = `
              display: none !important;
              visibility: hidden !important;
              opacity: 0 !important;
              height: 0 !important;
              width: 0 !important;
            `;
            try {
              parent.remove();
            } catch (e) {
              // Ignore
            }
          }
        }
      };

      // Multiple checks at different intervals
      const timeouts = [
        setTimeout(findAndHide, 50),
        setTimeout(findAndHide, 100),
        setTimeout(findAndHide, 200),
        setTimeout(findAndHide, 300),
        setTimeout(findAndHide, 500),
        setTimeout(findAndHide, 1000),
        setTimeout(findAndHide, 2000),
        setTimeout(findAndHide, 3000),
      ];

      // Use MutationObserver to catch dynamically added elements
      const observer = new MutationObserver(() => {
        findAndHide();
      });
      
      observer.observe(container, {
        childList: true,
        subtree: true,
        characterData: true,
        attributes: true,
        attributeFilter: ['class', 'id', 'aria-label', 'title', 'style'],
      });

      // Also observe the ChatKit web component shadow DOM if accessible
      const chatkitElement = container.querySelector('openai-chatkit');
      if (chatkitElement && chatkitElement.shadowRoot) {
        const shadowObserver = new MutationObserver(() => {
          findAndHide();
        });
        shadowObserver.observe(chatkitElement.shadowRoot, {
          childList: true,
          subtree: true,
          characterData: true,
        });
        
        return () => {
          timeouts.forEach(clearTimeout);
          observer.disconnect();
          shadowObserver.disconnect();
        };
      }

      return () => {
        timeouts.forEach(clearTimeout);
        observer.disconnect();
      };
    };

    return hideScrollIndicator();
  }, [isOpen, scriptStatus, styles.chatKitContainer]);

  // Handle chat button click - check login first
  const handleChatButtonClick = useCallback(() => {
    if (!isLoggedIn) {
      // Redirect to sign in if not logged in
      handleLogin();
      return;
    }
    
    // Toggle chat
    setIsOpen(!isOpen);
    setShowPersonalizeMenu(false); // Close menu when toggling chat
  }, [isLoggedIn, isOpen, handleLogin]);

  // Handle right-click on chat button for personalization menu
  const handleChatButtonContextMenu = useCallback((e: React.MouseEvent) => {
    e.preventDefault();
    if (isLoggedIn) {
      setShowPersonalizeMenu(!showPersonalizeMenu);
    }
  }, [isLoggedIn, showPersonalizeMenu]);

  // Handle personalize menu actions
  const handlePersonalize = useCallback(() => {
    setShowPersonalizeMenu(false);
    setIsOpen(true);
    // Send a message asking about personalization
    setTimeout(async () => {
      if (sendUserMessage) {
        await sendUserMessage({
          text: "How can I personalize my learning experience?",
          newThread: false,
        });
      }
    }, 500);
  }, [sendUserMessage]);

  const handleUpdateProfile = useCallback(() => {
    setShowPersonalizeMenu(false);
    setIsOpen(true);
    // Send a message about updating profile
    setTimeout(async () => {
      if (sendUserMessage) {
        await sendUserMessage({
          text: "Help me update my profile and preferences",
          newThread: false,
        });
      }
    }, 500);
  }, [sendUserMessage]);

  // Close personalize menu when clicking outside
  useEffect(() => {
    if (!showPersonalizeMenu) return;

    const handleClickOutside = (event: MouseEvent) => {
      if (
        personalizeMenuRef.current &&
        !personalizeMenuRef.current.contains(event.target as Node) &&
        !(event.target as HTMLElement).closest(`.${styles.chatButton}`)
      ) {
        setShowPersonalizeMenu(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [showPersonalizeMenu]);

  return (
    <>
      {/* Floating Chat Button and Settings */}
      <div className={styles.chatButtonContainer}>
        {/* Settings/Personalize Button - Visible when logged in */}
        {isLoggedIn && (
          <button
            className={styles.settingsButton}
            onClick={() => setShowPersonalizeMenu(!showPersonalizeMenu)}
            aria-label="Personalize Assistant"
            title="Personalize Assistant"
          >
            <svg
              width="18"
              height="18"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            >
              <circle cx="12" cy="12" r="3" />
              <path d="M12 1v6m0 6v6M5.64 5.64l4.24 4.24m4.24 4.24l4.24 4.24M1 12h6m6 0h6M5.64 18.36l4.24-4.24m4.24-4.24l4.24-4.24" />
            </svg>
          </button>
        )}

        <button
          className={styles.chatButton}
          onClick={handleChatButtonClick}
          aria-label={isLoggedIn ? "Open AI Assistant" : "Login to use AI Assistant"}
          title={isLoggedIn ? "Open AI Assistant" : "Login to use AI Assistant"}
        >
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="#ffffff"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        </button>

        {/* Personalize Menu - Shows when clicking settings button */}
        {showPersonalizeMenu && isLoggedIn && (
          <div
            ref={personalizeMenuRef}
            className={styles.personalizeMenu}
          >
            <div className={styles.personalizeMenuHeader}>
              <h4>Personalize Assistant</h4>
              <button
                className={styles.closeMenuButton}
                onClick={() => setShowPersonalizeMenu(false)}
                aria-label="Close menu"
              >
                ×
              </button>
            </div>
            <div className={styles.personalizeMenuContent}>
              <p>Customize your AI assistant:</p>
              <button
                className={styles.personalizeButton}
                onClick={handlePersonalize}
              >
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M12 2L2 7l10 5 10-5-10-5z" />
                  <path d="M2 17l10 5 10-5" />
                  <path d="M2 12l10 5 10-5" />
                </svg>
                Set Learning Preferences
              </button>
              <button
                className={styles.personalizeButton}
                onClick={handleUpdateProfile}
              >
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
                  <circle cx="12" cy="7" r="4" />
                </svg>
                Update Profile
              </button>
              <button
                className={styles.personalizeButton}
                onClick={() => {
                  setShowPersonalizeMenu(false);
                  setIsOpen(true);
                  setTimeout(async () => {
                    if (sendUserMessage) {
                      await sendUserMessage({
                        text: "What can you help me with?",
                        newThread: false,
                      });
                    }
                  }, 500);
                }}
              >
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <circle cx="12" cy="12" r="10" />
                  <path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3" />
                  <path d="M12 17h.01" />
                </svg>
                What Can You Help With?
              </button>
              <button
                className={styles.personalizeButton}
                onClick={() => {
                  setShowPersonalizeMenu(false);
                  setIsOpen(true);
                  setTimeout(async () => {
                    if (sendUserMessage) {
                      await sendUserMessage({
                        text: "Show me my learning progress and recommendations",
                        newThread: false,
                      });
                    }
                  }, 500);
                }}
              >
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <polyline points="22 12 18 12 15 21 9 3 6 12 2 12" />
                </svg>
                Learning Progress
              </button>
            </div>
          </div>
        )}
      </div>

      {/* ChatKit Component - Floating widget that opens/closes */}
      {/* Only show if user is logged in */}
      {isOpen && scriptStatus === 'ready' && isLoggedIn && (
        <div className={styles.chatKitContainer}>
          <ChatKit 
            control={control} 
            className={styles.chatKit}
            style={{
              scrollbarWidth: 'none',
              msOverflowStyle: 'none',
            }}
          />
        </div>
      )}
      
      {/* Login Prompt Overlay - Show if user tries to open chat but isn't logged in */}
      {isOpen && !isLoggedIn && !authLoading && (
        <div className={styles.loginPrompt}>
          <div className={styles.loginPromptContent}>
            <h3>Login Required</h3>
            <p>Please log in to use the AI Assistant and get personalized help with your robotics learning journey.</p>
            <button 
              className={styles.loginButton}
              onClick={handleLogin}
            >
              Log In
            </button>
            <button 
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
            >
              Close
            </button>
          </div>
        </div>
      )}

      {/* "Ask" Button for Selected Text - Only show if logged in */}
      {selectedText && selectionPosition && isLoggedIn && (
        <div
          ref={selectionRef}
          className={styles.askButton}
          style={{
            left: `${selectionPosition.x}px`,
            top: `${selectionPosition.y}px`,
            transform: 'translateX(-50%)',
          }}
          onClick={(e) => {
            e.stopPropagation();
            e.preventDefault();
            handleAskSelectedText();
          }}
          onMouseDown={(e) => {
            e.stopPropagation();
            e.preventDefault();
          }}
        >
          <svg
            width="14"
            height="14"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
          <span>Ask</span>
        </div>
      )}
    </>
  );
}

