"use client";

import { useState, useRef, useEffect } from "react";
import { useLanguage } from "@/contexts/language-context";
import { Sidebar } from "@/components/sidebar";
import { MobileNav } from "@/components/mobile-nav";
import { Header } from "@/components/header";
import { GlassCard } from "@/components/ui/glass-card";
import { Brain, Send, Sparkles, Loader2 } from "lucide-react";

interface Message {
  id: number;
  role: "user" | "ai";
  content: string;
  timestamp: Date;
}

export default function AIAssistant() {
  const { t } = useLanguage();
  const [messages, setMessages] = useState<Message[]>([
    {
      id: 1,
      role: "ai",
      content: t("aiAssistant.welcomeMessage"),
      timestamp: new Date(),
    },
  ]);
  const [input, setInput] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const quickSuggestions = [
    t("aiAssistant.suggestion1"),
    t("aiAssistant.suggestion2"),
    t("aiAssistant.suggestion3"),
    t("aiAssistant.suggestion4"),
  ];

  // Auto scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  const handleSend = async () => {
    if (!input.trim() || isLoading) return;

    const userInput = input.trim();
    setInput("");
    setIsLoading(true);

    // Add user message
    const userMessage: Message = {
      id: Date.now(),
      role: "user",
      content: userInput,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);

    try {
      // Call API
      const response = await fetch("/api/ai-chat", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          messages: [
            ...messages.map((m) => ({
              role: m.role === "ai" ? "assistant" : "user",
              content: m.content,
            })),
            { role: "user", content: userInput },
          ],
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.message || "API request failed");
      }

      // Add AI response
      const aiMessage: Message = {
        id: Date.now() + 1,
        role: "ai",
        content: data.message,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, aiMessage]);
    } catch (error: any) {
      console.error("Error:", error);

      // Add error message
      const errorMessage: Message = {
        id: Date.now() + 1,
        role: "ai",
        content: `Xin lỗi, tôi đang gặp sự cố kỹ thuật. Vui lòng thử lại sau. 🔧\n\nLỗi: ${error.message}`,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSuggestionClick = (suggestion: string) => {
    setInput(suggestion);
  };

  return (
    <div className="flex flex-col h-screen bg-bg-primary overflow-hidden">
      <Header />
      <div className="flex flex-1 overflow-hidden">
        <MobileNav />
        <Sidebar />

      <main className="flex-1 overflow-auto flex flex-col">
        {/* Header */}
        <div className="p-4 sm:p-6 lg:p-8 border-b border-white/10">
          <div className="flex items-center gap-4">
            <div className="relative">
              <div
                className="w-12 h-12 rounded-full glass-card flex items-center justify-center"
                style={{
                  borderColor: "#ff7a1a",
                  boxShadow: "0 0 15px rgba(255, 122, 26, 0.5)",
                }}
              >
                <Brain className="w-6 h-6 text-neon-cyan" />
              </div>
              <div
                className="absolute -top-1 -right-1 w-3 h-3 bg-[#10b981] rounded-full animate-pulse"
                style={{ boxShadow: "0 0 10px #10b981" }}
              />
            </div>
            <div>
              <h1 className="text-2xl font-bold text-[#ff7a1a] tracking-wider">
                {t("aiAssistant.title")}
              </h1>

              <p className="text-sm text-fg-secondary">
                {t("aiAssistant.subtitle")}
              </p>
            </div>
          </div>
        </div>

        {/* Messages Container */}
        <div className="flex-1 overflow-y-auto p-4 sm:p-6 lg:p-8 space-y-4">
          {messages.map((message) => (
            <div
              key={message.id}
              className={`flex ${message.role === "user" ? "justify-end" : "justify-start"} animate-fadeIn`}
            >
              <div
                className={`flex gap-3 max-w-[80%] ${message.role === "user" ? "flex-row-reverse" : "flex-row"}`}
              >
                {/* Avatar */}
                <div
                  className={`w-10 h-10 rounded-full flex items-center justify-center shrink-0 ${
                    message.role === "ai"
                      ? "glass-card glow-cyan"
                      : "glass-card border-neon-green/30"
                  }`}
                >
                  {message.role === "ai" ? (
                    <Brain className="w-5 h-5 text-neon-cyan" />
                  ) : (
                    <div
                      className="w-5 h-5 rounded-full bg-neon-green"
                      style={{ boxShadow: "0 0 10px var(--neon-green)" }}
                    />
                  )}
                </div>

                {/* Message Bubble */}
                <GlassCard
                  glow={message.role === "ai" ? "cyan" : "none"}
                  className={`p-4 ${message.role === "user" ? "border-neon-green/30" : ""}`}
                >
                  <p className="text-fg-primary text-sm leading-relaxed whitespace-pre-wrap">
                    {message.content}
                  </p>
                  <p className="text-xs text-fg-muted mt-2">
                    {message.timestamp.toLocaleTimeString()}
                  </p>
                </GlassCard>
              </div>
            </div>
          ))}

          {/* Loading indicator */}
          {isLoading && (
            <div className="flex justify-start animate-fadeIn">
              <div className="flex gap-3 max-w-[80%]">
                <div className="w-10 h-10 rounded-full flex items-center justify-center shrink-0 glass-card glow-cyan">
                  <Brain className="w-5 h-5 text-neon-cyan" />
                </div>
                <GlassCard glow="cyan" className="p-4">
                  <div className="flex items-center gap-2">
                    <Loader2 className="w-4 h-4 text-neon-cyan animate-spin" />
                    <p className="text-fg-secondary text-sm">
                      Đang suy nghĩ...
                    </p>
                  </div>
                </GlassCard>
              </div>
            </div>
          )}

          <div ref={messagesEndRef} />
        </div>

        {/* Quick Suggestions */}
        <div className="px-4 sm:px-6 lg:px-8 py-3 border-t border-white/10">
          <div className="flex flex-wrap gap-2">
            {quickSuggestions.map((suggestion, idx) => (
              <button
                key={idx}
                onClick={() => handleSuggestionClick(suggestion)}
                disabled={isLoading}
                className="glass-card px-3 py-2 text-xs text-neon-cyan hover:glow-cyan transition-all duration-300 border border-transparent hover:border-neon-cyan/50 disabled:opacity-50 disabled:cursor-not-allowed"
              >
                <Sparkles className="w-3 h-3 inline mr-1" />
                {suggestion}
              </button>
            ))}
          </div>
        </div>

        {/* Input Area */}
        <div className="p-4 sm:p-6 lg:p-8 border-t border-white/10">
          <div className="flex gap-3">
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={(e) =>
                e.key === "Enter" && !isLoading && handleSend()
              }
              placeholder={t("aiAssistant.inputPlaceholder")}
              disabled={isLoading}
              className="flex-1 glass-card px-4 py-3 text-fg-primary placeholder:text-fg-muted focus:glow-cyan transition-all duration-300 disabled:opacity-50 disabled:cursor-not-allowed"
            />
            <button
              onClick={handleSend}
              disabled={!input.trim() || isLoading}
              className="btn-neon px-6 disabled:opacity-50 disabled:cursor-not-allowed flex items-center gap-2"
            >
              {isLoading ? (
                <Loader2 className="w-5 h-5 animate-spin" />
              ) : (
                <Send className="w-5 h-5" />
              )}
            </button>
          </div>

          {/* Info Note */}
        </div>
      </main>
      </div>
    </div>
  );
}
