"use client"

import type React from "react"

import { useState, useRef, useEffect } from "react"
import { Sidebar } from "@/components/sidebar"
import { Button } from "@/components/ui/button"
import { Card } from "@/components/ui/card"
import { Send, Loader } from "lucide-react"

interface Message {
  id: string
  role: "user" | "assistant"
  content: string
  timestamp: string
}

const mockResponses = [
  "Dựa trên dữ liệu lái xe của bạn, tôi khuyên bạn nên tăng khoảng cách an toàn với xe phía trước.",
  "Bạn đã lái xe an toàn trong 5 ngày liên tiếp. Hãy tiếp tục duy trì thói quen tốt này!",
  "Tôi phát hiện bạn thường phân tán vào lúc 2-3 chiều. Hãy cân nhắc nghỉ ngơi vào thời gian này.",
  "Tốc độ trung bình của bạn là 55km/h, nằm trong giới hạn an toàn. Tuyệt vời!",
  "Bạn có thể cải thiện kỹ năng lái xe bằng cách tập trung hơn vào đường phía trước.",
]

export default function AIAssistant() {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: "1",
      role: "assistant",
      content:
        "Xin chào! Tôi là trợ lý AI của ADAS. Tôi có thể giúp bạn cải thiện kỹ năng lái xe và cung cấp lời khuyên về an toàn giao thông. Bạn có câu hỏi gì không?",
      timestamp: new Date().toLocaleTimeString("vi-VN"),
    },
  ])
  const [input, setInput] = useState("")
  const [isLoading, setIsLoading] = useState(false)
  const messagesEndRef = useRef<HTMLDivElement>(null)

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" })
  }

  useEffect(() => {
    scrollToBottom()
  }, [messages])

  const handleSendMessage = async () => {
    if (!input.trim()) return

    const userMessage: Message = {
      id: Date.now().toString(),
      role: "user",
      content: input,
      timestamp: new Date().toLocaleTimeString("vi-VN"),
    }

    setMessages((prev) => [...prev, userMessage])
    setInput("")
    setIsLoading(true)

    // Simulate AI response delay
    setTimeout(() => {
      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: "assistant",
        content: mockResponses[Math.floor(Math.random() * mockResponses.length)],
        timestamp: new Date().toLocaleTimeString("vi-VN"),
      }
      setMessages((prev) => [...prev, assistantMessage])
      setIsLoading(false)
    }, 1000)
  }

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault()
      handleSendMessage()
    }
  }

  return (
    <div className="flex h-screen bg-background">
      <Sidebar />

      <main className="flex-1 overflow-auto flex flex-col">
        <div className="p-8 pb-4">
          <div className="mb-4">
            <h1 className="text-3xl font-bold text-foreground mb-2">Trợ Lý AI</h1>
            <p className="text-foreground/60">Nhận lời khuyên và hỗ trợ từ trợ lý AI thông minh</p>
          </div>
        </div>

        <div className="flex-1 overflow-auto px-8">
          <div className="space-y-4 max-w-2xl">
            {messages.map((message) => (
              <div key={message.id} className={`flex ${message.role === "user" ? "justify-end" : "justify-start"}`}>
                <Card
                  className={`max-w-md p-4 ${
                    message.role === "user"
                      ? "bg-primary text-primary-foreground border-primary"
                      : "bg-card border-border text-foreground"
                  }`}
                >
                  <p className="text-sm mb-1">{message.content}</p>
                  <p
                    className={`text-xs ${message.role === "user" ? "text-primary-foreground/70" : "text-foreground/60"}`}
                  >
                    {message.timestamp}
                  </p>
                </Card>
              </div>
            ))}
            {isLoading && (
              <div className="flex justify-start">
                <Card className="bg-card border-border p-4">
                  <div className="flex items-center gap-2">
                    <Loader className="w-4 h-4 animate-spin text-primary" />
                    <p className="text-sm text-foreground/60">Đang suy nghĩ...</p>
                  </div>
                </Card>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
        </div>

        <div className="p-8 pt-4 border-t border-border">
          <div className="max-w-2xl flex gap-3">
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Nhập câu hỏi của bạn..."
              className="flex-1 bg-input border border-border rounded-lg p-3 text-foreground placeholder-foreground/50 resize-none focus:outline-none focus:ring-2 focus:ring-primary"
              rows={3}
            />
            <Button
              onClick={handleSendMessage}
              disabled={!input.trim() || isLoading}
              className="bg-primary hover:bg-primary/90 text-primary-foreground self-end"
            >
              <Send className="w-4 h-4" />
            </Button>
          </div>
        </div>
      </main>
    </div>
  )
}
