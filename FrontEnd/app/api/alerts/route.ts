import { type NextRequest, NextResponse } from "next/server"

import { getApiUrl } from "@/lib/api-config"
import { API_ENDPOINTS } from "@/lib/api-endpoints"

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
    
    // Fetch latest alerts from ADAS backend (mapped to admin statistics/alerts; path assumed)
    // Mapped to admin statistics until dedicated alerts endpoint is provided
    const response = await fetch(`${getApiUrl(API_ENDPOINTS.ADMIN_STATISTICS)}?${query}`, {
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
