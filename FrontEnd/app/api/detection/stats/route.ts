import { type NextRequest, NextResponse } from "next/server"

const BACKEND_URL = process.env.BACKEND_URL || "http://localhost:8000"

export async function GET(request: NextRequest) {
  try {
    const searchParams = request.nextUrl.searchParams
    const hours = searchParams.get('hours') || '24'
    
    // Fetch detection stats from backend
    const response = await fetch(`${BACKEND_URL}/api/detections/stats?hours=${hours}`, {
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
    console.error("Detection stats API error:", error)
    return NextResponse.json({ 
      success: false,
      error: "Failed to fetch detection stats",
      total_detections: 0,
      by_class: []
    }, { status: 500 })
  }
}
