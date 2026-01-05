import React, { useState, useEffect, useRef } from 'react';
import { Bot, X, Send, MessageSquare } from 'lucide-react';
import './Chatbot.css';

// Type declarations for Web Speech API
declare global {
  interface Window {
    SpeechRecognition: typeof SpeechRecognition;
    webkitSpeechRecognition: typeof SpeechRecognition;
  }
}

interface SpeechRecognition extends EventTarget {
  continuous: boolean;
  interimResults: boolean;
  lang: string;
  onstart: (() => void) | null;
  onresult: ((event: SpeechRecognitionEvent) => void) | null;
  onerror: ((event: SpeechRecognitionErrorEvent) => void) | null;
  onend: (() => void) | null;
  start(): void;
  stop(): void;
}

interface SpeechRecognitionEvent extends Event {
  results: SpeechRecognitionResultList;
}

interface SpeechRecognitionErrorEvent extends Event {
  error: string;
}

interface SpeechRecognitionResultList {
  [index: number]: SpeechRecognitionResult;
  length: number;
}

interface SpeechRecognitionResult {
  [index: number]: SpeechRecognitionAlternative;
  length: number;
  isFinal: boolean;
}

interface SpeechRecognitionAlternative {
  transcript: string;
  confidence: number;
}

interface Message {
  id: number;
  text: string;
  sender: 'user' | 'bot';
  sources?: Array<{ url: string; title: string; relevance_score: number }>;
  latency?: number;
  timestamp: Date;
}

const Chatbot = ({ apiUrl = 'http://localhost:8000', position = 'bottom-right' }: { apiUrl?: string; position?: string }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [isListening, setIsListening] = useState(false);
  const [isSupported, setIsSupported] = useState(false);
  const [currentSources, setCurrentSources] = useState<Array<{ url: string; title: string; relevance_score: number }>>([]);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const recognitionRef = useRef<SpeechRecognition | null>(null);
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  // Check for speech recognition support
  useEffect(() => {
    if (typeof window === 'undefined') return;
    const SpeechRecognitionConstructor = (window as any).SpeechRecognition || (window as any).webkitSpeechRecognition;
    if (SpeechRecognitionConstructor) {
      setIsSupported(true);
      const recognition = new SpeechRecognitionConstructor() as SpeechRecognition;
      recognition.continuous = false;
      recognition.interimResults = false;
      recognition.lang = 'en-US';

      recognition.onstart = () => {
        setIsListening(true);
      };

      recognition.onresult = (event: SpeechRecognitionEvent) => {
        const transcript = event.results[0][0].transcript;
        setInputValue(prev => prev + (prev ? ' ' : '') + transcript);
        setIsListening(false);
      };

      recognition.onerror = (event: SpeechRecognitionErrorEvent) => {
        console.error('Speech recognition error:', event.error);
        setIsListening(false);
        if (event.error === 'not-allowed') {
          alert('Microphone access denied. Please enable microphone permissions.');
        }
      };

      recognition.onend = () => {
        setIsListening(false);
      };

      recognitionRef.current = recognition;
    }
  }, []);

  // Function to get selected text from the page
  useEffect(() => {
    const handleSelection = () => {
      const text = typeof window !== 'undefined' ? window.getSelection()?.toString().trim() || '' : '';
      if (text) {
        setSelectedText(text);
      }
    };

    if (typeof window !== 'undefined' && typeof document !== 'undefined') {
      document.addEventListener('mouseup', handleSelection);
      return () => {
        document.removeEventListener('mouseup', handleSelection);
      };
    }
  }, []);

  // Auto-resize textarea
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = `${textareaRef.current.scrollHeight}px`;
    }
  }, [inputValue]);

  // Auto-scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Load chat history when chatbot opens
  useEffect(() => {
    if (isOpen && messages.length === 0) {
      loadChatHistory();
    }
  }, [isOpen]);

  const loadChatHistory = async () => {
    const sessionId = localStorage.getItem('chatbot_session_id');
    if (!sessionId) return;

    try {
      const response = await fetch(`${apiUrl}/api/chat/history/${sessionId}`);

      if (response.status === 404) {
        // Session doesn't exist in database (might have been cleared)
        console.log('Session not found, starting fresh conversation');
        localStorage.removeItem('chatbot_session_id');
        return;
      }

      if (response.ok) {
        const data = await response.json();

        // Convert history messages to our message format
        const historyMessages: Message[] = data.messages.map((msg: any) => ({
          id: parseInt(msg.id.substring(0, 8), 16), // Convert UUID to number
          text: msg.content,
          sender: msg.role === 'user' ? 'user' : 'bot',
          sources: msg.sources || [],
          latency: msg.latency,
          timestamp: new Date(msg.created_at)
        }));

        setMessages(historyMessages);

        // Set sources from last assistant message if available
        const lastAssistantMsg = [...historyMessages].reverse().find(m => m.sender === 'bot');
        if (lastAssistantMsg && lastAssistantMsg.sources) {
          setCurrentSources(lastAssistantMsg.sources);
        }
      }
    } catch (error) {
      console.error('Error loading chat history:', error);
      // If error, remove invalid session ID and start fresh
      localStorage.removeItem('chatbot_session_id');
    }
  };

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setCurrentSources([]); // Clear previous sources when sending new message
    setCurrentSources([]); // Clear previous sources when sending new message

    try {
      // Prepare the context from selected text if available
      const context = selectedText || undefined;

      // Send message to backend
      const response = await fetch(`${apiUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: userMessage.text,
          context: context,
          session_id: localStorage.getItem('chatbot_session_id') || null
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Store session ID for future messages
      if (data.session_id) {
        localStorage.setItem('chatbot_session_id', data.session_id);
      }

      // Add bot response to chat (without sources in message)
      const botMessage: Message = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'bot',
        latency: data.latency,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
      
      // Store sources separately for bottom display
      if (data.sources && data.sources.length > 0) {
        setCurrentSources(data.sources);
      } else {
        setCurrentSources([]);
      }
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: Message = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error. Please try again.',
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
      setCurrentSources([]); // Clear sources on error
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear selected text after sending
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const toggleListening = () => {
    if (!isSupported || !recognitionRef.current) {
      alert('Speech recognition is not supported in your browser.');
      return;
    }

    if (isListening) {
      recognitionRef.current.stop();
      setIsListening(false);
    } else {
      try {
        recognitionRef.current.start();
      } catch (error) {
        console.error('Error starting recognition:', error);
        setIsListening(false);
      }
    }
  };

  return (
    <div className={`chatbot-widget chatbot-${position}`}>
      {isOpen ? (
        <div className="chatbot-container-new">
          <div className="chatbot-header-new">
            <div className="chatbot-header-content-new">
              <div className="chatbot-header-icon-new">
                <Bot style={{width: '28px', height: '28px'}} />
              </div>
              <div>
                <div className="chatbot-title-new">AI Assistant</div>
                <div className="chatbot-subtitle-new">
                  <div className="chatbot-status-dot"></div>
                  Online
                </div>
              </div>
            </div>
            <button className="chatbot-close-new" onClick={toggleChat} aria-label="Close chat">
              <X style={{width: '20px', height: '20px'}} />
            </button>
          </div>

          <div className="chatbot-messages-new">
            {messages.length === 0 ? (
              <div className="chatbot-welcome-new">
                <p>Welcome! I can help you navigate the Physical AI curriculum. What would you like to learn?</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`chatbot-message-new chatbot-message-${message.sender}-new`}
                >
                  <div className="chatbot-message-text-new">{message.text}</div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="chatbot-message-new chatbot-message-bot-new">
                <div className="chatbot-typing-indicator-new">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Sources section at the bottom */}
          {currentSources && currentSources.length > 0 && (
            <div className="chatbot-sources-bottom-new">
              <details open>
                <summary>
                  <span className="sources-icon">📚</span>
                  Sources ({currentSources.length})
                </summary>
                <ul>
                  {currentSources
                    .filter(source => source.url && source.title)
                    .map((source, index) => {
                      const url = source.url.startsWith('http')
                        ? source.url
                        : source.url.startsWith('/')
                          ? source.url
                          : `/${source.url}`;

                      return (
                        <li key={index}>
                          <a
                            href={url}
                            className="source-link"
                            onClick={(e) => {
                              e.preventDefault();
                              // Use window.location for internal navigation
                              window.location.href = url;
                            }}
                          >
                            {source.title || 'Untitled Document'}
                          </a>
                          {source.relevance_score && (
                            <span className="relevance-score">
                              {(source.relevance_score * 100).toFixed(0)}% relevant
                            </span>
                          )}
                        </li>
                      );
                    })}
                </ul>
              </details>
            </div>
          )}

          <div className="chatbot-input-area-new">
            {selectedText && (
              <div className="chatbot-context-preview-new">
                <span>Context: "{selectedText.substring(0, 80)}{selectedText.length > 80 ? '...' : ''}"</span>
                <button 
                  className="chatbot-context-remove-new"
                  onClick={() => setSelectedText('')}
                  aria-label="Remove context"
                >
                  ×
                </button>
              </div>
            )}
            <div className="chatbot-input-wrapper-new">
              <input
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask anything..."
                className="chatbot-input-new"
              />
              <button
                onClick={sendMessage}
                disabled={isLoading || !inputValue.trim()}
                className="chatbot-send-button-new"
                aria-label="Send message"
              >
                <Send style={{width: '24px', height: '24px'}} />
              </button>
            </div>
          </div>
        </div>
      ) : (
        <button className="chatbot-toggle-new" onClick={toggleChat} aria-label="Open chat">
          <MessageSquare style={{width: '32px', height: '32px'}} />
        </button>
      )}
    </div>
  );
};

export default Chatbot;