import { type NextRequest, NextResponse } from "next/server"

import { getApiUrl } from "@/lib/api-config"
import { API_ENDPOINTS } from "@/lib/api-endpoints"

export async function GET(request: NextRequest) {
  try {
    const searchParams = request.nextUrl.searchParams
    const hours = searchParams.get('hours') || '24'
    
    // Fetch alert stats from ADAS backend (mapped to admin statistics; path assumed)
    // Use admin statistics endpoint for alert stats
    const response = await fetch(`${getApiUrl(API_ENDPOINTS.ADMIN_STATISTICS)}?hours=${hours}`, {
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
