import { type NextRequest, NextResponse } from "next/server"

import { getApiUrl } from "@/lib/api-config"
import { API_ENDPOINTS } from "@/lib/api-endpoints"

export async function POST(request: NextRequest) {
  try {
    const data = await request.json()

    // Forward to ADAS detections save (assumed equivalent path)
    const response = await fetch(getApiUrl(API_ENDPOINTS.DETECTIONS_SAVE), {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(data)
    })

    if (!response.ok) {
      const error = await response.json()
      throw new Error(error.detail || "Failed to save detection")
    }
    
    const result = await response.json()
    return NextResponse.json(result)
  } catch (error) {
    console.error("Detection save error:", error)
    return NextResponse.json({ 
      success: false,
      error: error instanceof Error ? error.message : "Failed to save detection" 
    }, { status: 500 })
  }
}

export async function GET(request: NextRequest) {
  try {
    // Get query params
    const searchParams = request.nextUrl.searchParams
    const limit = searchParams.get('limit') || '20'
    const camera_id = searchParams.get('camera_id') || ''
    const class_name = searchParams.get('class_name') || ''
    
    // Build query string
    let query = `limit=${limit}`
    if (camera_id) query += `&camera_id=${camera_id}`
    if (class_name) query += `&class_name=${class_name}`
    
    // Fetch recent detections from ADAS endpoint (assumed equivalent path)
    const response = await fetch(`${getApiUrl(API_ENDPOINTS.DETECTIONS_RECENT)}?${query}`, {
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
    console.error("Detection API error:", error)
    return NextResponse.json({ 
      success: false,
      error: "Failed to fetch detections",
      detections: [] 
    }, { status: 500 })
  }
}
