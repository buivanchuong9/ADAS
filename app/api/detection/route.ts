import { type NextRequest, NextResponse } from "next/server"

const BACKEND_URL = process.env.BACKEND_URL || "http://localhost:8000"

export async function POST(request: NextRequest) {
  try {
    const data = await request.json()

    // Proxy to backend if needed, or handle WebSocket detections
    // Most detections come through WebSocket, this is for REST fallback
    
    return NextResponse.json({
      success: true,
      message: "Detection data received. Use WebSocket for real-time inference.",
      websocket_url: "ws://localhost:8000/ws/infer"
    })
  } catch (error) {
    console.error("Detection API error:", error)
    return NextResponse.json({ error: "Internal server error" }, { status: 500 })
  }
}

export async function GET() {
  try {
    // Fetch recent detections from backend
    const response = await fetch(`${BACKEND_URL}/api/events/list?type=detection&limit=10`, {
      headers: {
        'Content-Type': 'application/json',
      },
    })

    if (!response.ok) {
      throw new Error(`Backend error: ${response.statusText}`)
    }

    const detections = await response.json()
    return NextResponse.json(detections)
  } catch (error) {
    console.error("Detection API error:", error)
    return NextResponse.json({ error: "Internal server error" }, { status: 500 })
  }
}
