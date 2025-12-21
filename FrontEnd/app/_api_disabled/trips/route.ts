import { type NextRequest, NextResponse } from "next/server"

import { getApiUrl } from "@/lib/api-config"
import { API_ENDPOINTS } from "@/lib/api-endpoints"

export async function POST(request: NextRequest) {
  try {
    const data = await request.json()

    // Send trip to ADAS backend
    const response = await fetch(getApiUrl(API_ENDPOINTS.TRIPS), {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(data),
    })

    if (!response.ok) {
      throw new Error(`Backend error: ${response.statusText}`)
    }

    const result = await response.json()
    return NextResponse.json(result)
  } catch (error) {
    console.error("Trips API error:", error)
    return NextResponse.json({ error: "Internal server error" }, { status: 500 })
  }
}

export async function GET(request: NextRequest) {
  try {
    const searchParams = request.nextUrl.searchParams
    const limit = searchParams.get("limit")

    // Build query params
    const params = new URLSearchParams()
    if (limit) params.append('limit', limit)

    // Fetch trips from ADAS backend
    const response = await fetch(`${getApiUrl(API_ENDPOINTS.TRIPS_LIST)}?${params.toString()}`, {
      headers: {
        'Content-Type': 'application/json',
      },
    })

    if (!response.ok) {
      throw new Error(`Backend error: ${response.statusText}`)
    }

    const trips = await response.json()
    return NextResponse.json(trips)
  } catch (error) {
    console.error("Trips API error:", error)
    return NextResponse.json({ error: "Internal server error" }, { status: 500 })
  }
}
