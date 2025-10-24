import { type NextRequest, NextResponse } from "next/server"

export async function POST(request: NextRequest) {
  try {
    const data = await request.json()

    // Validate detection data
    if (!data.objects || !Array.isArray(data.objects)) {
      return NextResponse.json({ error: "Invalid detection data" }, { status: 400 })
    }

    // Process detection data
    const detections = data.objects.map((obj: any) => ({
      type: obj.type,
      confidence: obj.confidence,
      bbox: obj.bbox,
      timestamp: new Date().toISOString(),
    }))

    // TODO: Store detections in database
    // TODO: Trigger alerts if necessary

    return NextResponse.json({
      success: true,
      detections,
      message: "Detection data processed successfully",
    })
  } catch (error) {
    console.error("Detection API error:", error)
    return NextResponse.json({ error: "Internal server error" }, { status: 500 })
  }
}

export async function GET() {
  try {
    // TODO: Fetch recent detections from database
    const detections = [
      {
        id: "1",
        type: "vehicle",
        confidence: 0.95,
        timestamp: new Date().toISOString(),
      },
      {
        id: "2",
        type: "person",
        confidence: 0.87,
        timestamp: new Date().toISOString(),
      },
    ]

    return NextResponse.json({ success: true, detections })
  } catch (error) {
    console.error("Detection API error:", error)
    return NextResponse.json({ error: "Internal server error" }, { status: 500 })
  }
}
