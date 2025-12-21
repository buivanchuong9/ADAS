import { NextRequest, NextResponse } from "next/server"

import { getApiUrl } from "@/lib/api-config"
import { API_ENDPOINTS } from "@/lib/api-endpoints"

export async function DELETE(
  request: NextRequest,
  { params }: { params: Promise<{ id: string }> }
) {
  const { id } = await params
  try {
    // Forward to ADAS backend (assumed equivalent endpoint)
    const response = await fetch(`${getApiUrl(API_ENDPOINTS.DATASET)}/${id}`, {
      method: "DELETE"
    })

    if (!response.ok) {
      throw new Error("Failed to delete dataset item")
    }

    const data = await response.json()
    return NextResponse.json(data)
  } catch (error) {
    console.error("Dataset deletion error:", error)
    return NextResponse.json(
      { error: "Failed to delete dataset item" },
      { status: 500 }
    )
  }
}
