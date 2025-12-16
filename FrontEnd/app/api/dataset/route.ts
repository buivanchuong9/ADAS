import { NextRequest, NextResponse } from "next/server"

import { getApiUrl } from "@/lib/api-config"
import { API_ENDPOINTS } from "@/lib/api-endpoints"

export async function GET() {
  try {
    // Forward to ADAS backend dataset (assumed equivalent)
    const response = await fetch(getApiUrl(API_ENDPOINTS.DATASET), {
      method: "GET",
      headers: {
        "Content-Type": "application/json"
      }
    })

    if (!response.ok) {
      throw new Error("Failed to fetch dataset")
    }

    const data = await response.json()
    return NextResponse.json(data)
  } catch (error) {
    console.error("Dataset fetch error:", error)
    return NextResponse.json(
      { error: "Failed to fetch dataset" },
      { status: 500 }
    )
  }
}

export async function POST(request: NextRequest) {
  try {
    const formData = await request.formData()
    
    // Forward to ADAS backend dataset upload (assumed equivalent)
    const response = await fetch(getApiUrl(API_ENDPOINTS.DATASET), {
      method: "POST",
      body: formData
    })

    if (!response.ok) {
      const error = await response.json()
      throw new Error(error.detail || "Failed to create dataset item")
    }

    const data = await response.json()
    return NextResponse.json(data)
  } catch (error) {
    console.error("Dataset creation error:", error)
    return NextResponse.json(
      { error: error instanceof Error ? error.message : "Failed to create dataset item" },
      { status: 500 }
    )
  }
}
