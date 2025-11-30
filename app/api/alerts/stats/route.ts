import { type NextRequest, NextResponse } from "next/server"

const BACKEND_URL = process.env.BACKEND_URL || "http://localhost:8000"

export async function GET(request: NextRequest) {
  try {
    const searchParams = request.nextUrl.searchParams
    const hours = searchParams.get('hours') || '24'
    
    // Fetch alert stats from backend (Phase 1)
    const response = await fetch(`${BACKEND_URL}/api/alerts/stats?hours=${hours}`, {
      headers: {
        'Content-Type': 'application/json',
      },
    })

    if (!response.ok) {
      throw new Error(`Backend error: ${response.statusText}`)
    }

    const data = await response.json()
    return NextResponse.json(data)
  } catch (error) {
    console.error("Alert stats API error:", error)
    return NextResponse.json({ 
      total_alerts: 0,
      severity_breakdown: {},
      type_breakdown: {},
      error: "Failed to fetch alert stats"
    }, { status: 500 })
  }
}
