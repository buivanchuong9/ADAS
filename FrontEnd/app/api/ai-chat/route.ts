import { NextRequest, NextResponse } from 'next/server'
import { GoogleGenerativeAI } from '@google/generative-ai'

// Gemini API Key
const GEMINI_API_KEY = process.env.GEMINI_API_KEY || ''

// System prompt - AI sẽ KHÔNG BAO GIỜ tự giới thiệu model
const SYSTEM_PROMPT = `Bạn là trợ lý ảo thông minh của hệ thống ADAS (Advanced Driver Assistance System).

QUY TẮC QUAN TRỌNG:
- KHÔNG BAO GIỜ tự giới thiệu bạn là GPT, ChatGPT, OpenAI, Gemini, Claude hay bất kỳ tên model AI nào
- KHÔNG BAO GIỜ đề cập đến việc bạn được phát triển bởi Google, OpenAI hay công ty nào
- Khi được hỏi "bạn là ai", chỉ trả lời: "Tôi là trợ lý ảo của hệ thống ADAS"
- Khi được hỏi về model/công nghệ, chỉ nói: "Tôi sử dụng công nghệ AI tiên tiến để hỗ trợ bạn"
- KHÔNG BAO GIỜ nói "tôi là Gemini" hay "tôi được Google phát triển"

VAI TRÒ CỦA BẠN:
- Bạn là chuyên gia về hệ thống ADAS (Advanced Driver Assistance System)
- Bạn hiểu rõ về: phát hiện làn đường, cảnh báo va chạm, giám sát tài xế, phát hiện đối tượng
- Bạn có thể phân tích dữ liệu lái xe, đưa ra khuyến nghị an toàn
- Bạn thân thiện, chuyên nghiệp và luôn sẵn sàng giúp đỡ

CÁCH TRẢ LỜI:
- Trả lời bằng tiếng Việt nếu người dùng hỏi bằng tiếng Việt
- Trả lời bằng tiếng Anh nếu người dùng hỏi bằng tiếng Anh
- Ngắn gọn, súc tích nhưng đầy đủ thông tin
- Sử dụng emoji phù hợp để tăng tính thân thiện
- Đưa ra ví dụ cụ thể khi cần thiết

KIẾN THỨC CHUYÊN MÔN:
- Lane Departure Warning (LDW): Cảnh báo khi xe lệch làn
- Forward Collision Warning (FCW): Cảnh báo nguy cơ va chạm phía trước
- Driver Monitoring System (DMS): Giám sát trạng thái tài xế (mệt mỏi, mất tập trung)
- Object Detection: Phát hiện xe cộ, người đi bộ, biển báo
- Blind Spot Monitoring (BSM): Giám sát điểm mù
- Adaptive Cruise Control (ACC): Điều khiển tốc độ tự động

Hãy trả lời một cách thông minh, tự nhiên như một trợ lý ảo chuyên nghiệp!`

interface ChatMessage {
    role: 'user' | 'assistant' | 'system'
    content: string
}

interface RequestBody {
    messages: ChatMessage[]
}

// API Route Handler
export async function POST(request: NextRequest) {
    try {
        const body: RequestBody = await request.json()
        const { messages } = body

        if (!messages || !Array.isArray(messages)) {
            return NextResponse.json(
                { error: 'Invalid request: messages array required' },
                { status: 400 }
            )
        }

        if (!GEMINI_API_KEY) {
            return NextResponse.json(
                { error: 'No API key configured' },
                { status: 500 }
            )
        }

        // Initialize Gemini
        const genAI = new GoogleGenerativeAI(GEMINI_API_KEY)
        const model = genAI.getGenerativeModel({
            model: 'gemini-2.5-flash',
        })

        // Get the last user message
        const lastMessage = messages[messages.length - 1]
        if (!lastMessage || lastMessage.role !== 'user') {
            return NextResponse.json(
                { error: 'Last message must be from user' },
                { status: 400 }
            )
        }

        // Convert messages to Gemini format (excluding system messages and the last message)
        const conversationHistory = messages
            .slice(0, -1) // Exclude the last message
            .filter(m => m.role !== 'system') // Remove system messages
            .map(m => ({
                role: m.role === 'assistant' ? 'model' : 'user',
                parts: [{ text: m.content }],
            }))

        // Ensure the first message in history is from 'user' role
        // If history starts with 'model', remove it to prevent API error
        while (conversationHistory.length > 0 && conversationHistory[0].role === 'model') {
            conversationHistory.shift()
        }

        // Prepend system prompt to the first user message if this is the first message
        const isFirstMessage = conversationHistory.length === 0
        const messageToSend = isFirstMessage
            ? `${SYSTEM_PROMPT}\n\n---\n\nUser: ${lastMessage.content}`
            : lastMessage.content

        // Start chat with history
        const chat = model.startChat({
            history: conversationHistory,
        })

        // Send the message
        const result = await chat.sendMessage(messageToSend)
        const response = result.response.text()

        // Return response
        return NextResponse.json({
            message: response,
            success: true,
        })

    } catch (error: any) {
        console.error('API Error:', error)
        return NextResponse.json(
            {
                error: 'Internal server error',
                message: error.message
            },
            { status: 500 }
        )
    }
}
