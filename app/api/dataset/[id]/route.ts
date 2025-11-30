import { NextRequest, NextResponse } from "next/server"

export async function DELETE(
  request: NextRequest,
  { params }: { params: { id: string } }
) {
  try {
    const { id } = params
    
    // Forward to FastAPI backend
    const response = await fetch(`http://localhost:8000/api/dataset/${id}`, {
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
