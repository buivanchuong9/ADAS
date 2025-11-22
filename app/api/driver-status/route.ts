import { type NextRequest, NextResponse } from "next/server"

const BACKEND_URL = process.env.BACKEND_URL || "http://localhost:8000"

export async function POST(request: NextRequest) {
  try {
    const data = await request.json()

    // Send driver status to backend (stored as event or driver_status)
    const response = await fetch(`${BACKEND_URL}/api/events`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        type: 'driver_status',
        severity: data.fatigueLevel > 70 ? 'critical' : data.fatigueLevel > 40 ? 'warning' : 'info',
        description: `Fatigue: ${data.fatigueLevel}%, Distraction: ${data.distractionLevel}%`,
        metadata: JSON.stringify(data)
      }),
    })

    if (!response.ok) {
      throw new Error(`Backend error: ${response.statusText}`)
    }

    const result = await response.json()
    return NextResponse.json(result)
  } catch (error) {
    console.error("Driver status API error:", error)
    return NextResponse.json({ error: "Internal server error" }, { status: 500 })
  }
}

export async function GET() {
  try {
    // Fetch recent driver status from backend events
    const response = await fetch(`${BACKEND_URL}/api/events/list?type=driver_status&limit=1`, {
      headers: {
        'Content-Type': 'application/json',
      },
    })

    if (!response.ok) {
      throw new Error(`Backend error: ${response.statusText}`)
    }

    const events = await response.json()
    const latestStatus = events.length > 0 ? JSON.parse(events[0].metadata || '{}') : {
      fatigueLevel: 0,
      distractionLevel: 0,
      eyesClosed: false,
      timestamp: new Date().toISOString()
    }

    return NextResponse.json({ success: true, status: latestStatus })
  } catch (error) {
    console.error("Driver status API error:", error)
    return NextResponse.json({ error: "Internal server error" }, { status: 500 })
  }
}
