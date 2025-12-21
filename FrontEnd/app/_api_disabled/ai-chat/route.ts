import { type NextRequest, NextResponse } from "next/server"

export async function POST(request: NextRequest) {
  try {
    const data = await request.json()

    if (!data.message) {
      return NextResponse.json({ error: "Message is required" }, { status: 400 })
    }

    // TODO: Integrate with AI service (OpenAI, Claude, etc.)
    // Mock response for now
    const mockResponses = [
      "Dựa trên dữ liệu lái xe của bạn, tôi khuyên bạn nên tăng khoảng cách an toàn với xe phía trước.",
      "Bạn đã lái xe an toàn trong 5 ngày liên tiếp. Hãy tiếp tục duy trì thói quen tốt này!",
      "Tôi phát hiện bạn thường phân tán vào lúc 2-3 chiều. Hãy cân nhắc nghỉ ngơi vào thời gian này.",
      "Tốc độ trung bình của bạn là 55km/h, nằm trong giới hạn an toàn. Tuyệt vời!",
    ]

    const response = mockResponses[Math.floor(Math.random() * mockResponses.length)]

    return NextResponse.json({
      success: true,
      message: data.message,
      response,
      timestamp: new Date().toISOString(),
    })
  } catch (error) {
    console.error("AI Chat API error:", error)
    return NextResponse.json({ error: "Internal server error" }, { status: 500 })
  }
}
