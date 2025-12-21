import { type NextRequest, NextResponse } from "next/server"

import { getApiUrl } from "@/lib/api-config"
import { API_ENDPOINTS } from "@/lib/api-endpoints"

export async function GET(request: NextRequest) {
  try {
    const searchParams = request.nextUrl.searchParams
    const hours = searchParams.get('hours') || '24'
    
    // Fetch detection stats via admin statistics
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
    console.error("Detection stats API error:", error)
    return NextResponse.json({ 
      success: false,
      error: "Failed to fetch detection stats",
      total_detections: 0,
      by_class: []
    }, { status: 500 })
  }
}
