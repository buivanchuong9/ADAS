"use client"

import { Sidebar } from "@/components/sidebar"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert"
import { Info } from "lucide-react"

// TODO: Integrate with real AI backend
// Needed:
// - POST /api/ai-chat - for chat completions
// - WebSocket /ws/ai-chat - for streaming responses
// - GET /api/ai-chat/history - for chat history

export default function AIAssistant() {
  return (
    <div className="flex h-screen bg-gradient-to-br from-blue-50 via-purple-50 to-pink-50">
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div className="p-8">
          <div className="mb-8">
            <h1 className="text-3xl font-bold text-foreground mb-2">Trợ Lý AI</h1>
            <p className="text-foreground/60">Nhận lời khuyên và hỗ trợ từ trợ lý AI thông minh</p>
          </div>

          {/* Warning about missing real API */}
          <Alert className="mb-6 border-yellow-300 bg-yellow-50">
            <Info className="h-4 w-4 text-yellow-600" />
            <AlertTitle className="text-yellow-800 font-semibold">Cần tích hợp AI Backend</AlertTitle>
            <AlertDescription className="text-yellow-700">
              <p className="mb-2">Tính năng này cần tích hợp với AI model (GPT/Claude/Llama):</p>
              <ul className="list-disc list-inside text-sm space-y-1">
                <li><code className="bg-yellow-100 px-1 rounded">POST /api/ai-chat</code> - Gửi message và nhận response</li>
                <li><code className="bg-yellow-100 px-1 rounded">WebSocket /ws/ai-chat</code> - Streaming responses</li>
                <li><code className="bg-yellow-100 px-1 rounded">GET /api/ai-chat/history</code> - Lịch sử chat</li>
              </ul>
              <p className="mt-3 text-xs">Có thể dùng: OpenAI API, Anthropic Claude, hoặc local LLM (Llama, Mistral)</p>
            </AlertDescription>
          </Alert>

          {/* Placeholder */}
          <Card className="border-indigo-200 bg-indigo-50/30">
            <CardHeader>
              <CardTitle className="text-indigo-700">Chờ AI Integration</CardTitle>
            </CardHeader>
            <CardContent className="space-y-3">
              <p className="text-sm text-indigo-600">
                Tính năng AI Assistant sẽ cung cấp:
              </p>
              <ul className="list-disc list-inside text-sm text-indigo-600 space-y-1">
                <li>Phân tích hành vi lái xe và đưa ra lời khuyên</li>
                <li>Giải thích các cảnh báo an toàn</li>
                <li>Đề xuất cải thiện kỹ năng lái xe</li>
                <li>Trả lời câu hỏi về ADAS features</li>
              </ul>
            </CardContent>
          </Card>
        </div>
      </main>
    </div>
  )
}
