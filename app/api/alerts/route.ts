import { type NextRequest, NextResponse } from "next/server"

const BACKEND_URL = process.env.BACKEND_URL || "http://localhost:8000"

export async function GET(request: NextRequest) {
  try {
    const searchParams = request.nextUrl.searchParams
    const limit = searchParams.get('limit') || '10'
    const severity = searchParams.get('severity') || ''
    const unplayed_only = searchParams.get('unplayed_only') || 'false'
    
    // Build query
    let query = `limit=${limit}`
    if (severity) query += `&severity=${severity}`
    if (unplayed_only === 'true') query += `&unplayed_only=true`
    
    // Fetch latest alerts from backend (Phase 1)
    const response = await fetch(`${BACKEND_URL}/api/alerts/latest?${query}`, {
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
    console.error("Alerts API error:", error)
    return NextResponse.json({ 
      success: false,
      alerts: [],
      error: "Failed to fetch alerts"
    }, { status: 500 })
  }
}
