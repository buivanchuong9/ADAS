import { NextRequest, NextResponse } from "next/server"

import { getApiUrl } from "@/lib/api-config"
import { API_ENDPOINTS } from "@/lib/api-endpoints"

export async function GET() {
  try {
    // TODO: Backend chưa có endpoint /api/dataset
    // Trả về empty data để không bị crash
    return NextResponse.json({
      success: true,
      data: [],
      message: "Dataset endpoint chưa được implement ở backend"
    })
    
    // Code này sẽ dùng khi backend ready:
    // const response = await fetch(getApiUrl(API_ENDPOINTS.DATASET), {
    //   method: "GET",
    //   headers: { "Content-Type": "application/json" }
    // })
    // if (!response.ok) throw new Error("Failed to fetch dataset")
    // const data = await response.json()
    // return NextResponse.json(data)
  } catch (error) {
    console.error("Dataset fetch error:", error)
    return NextResponse.json(
      { success: false, data: [], error: "Failed to fetch dataset" },
      { status: 200 } // Return 200 to avoid errors
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
