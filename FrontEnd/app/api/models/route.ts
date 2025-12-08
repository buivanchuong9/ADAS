import { NextRequest, NextResponse } from 'next/server';

const BACKEND_URL = process.env.NEXT_PUBLIC_BACKEND_URL || 'http://localhost:8000';

export async function GET(request: NextRequest) {
  try {
    const response = await fetch(`${BACKEND_URL}/api/models/available`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      throw new Error(`Backend error: ${response.status}`);
    }

    const data = await response.json();
    
    // Backend returns {success, models, total_models}
    // Convert to expected format {success, models}
    return NextResponse.json({
      success: data.success || true,
      models: data.models || []
    });
  } catch (error) {
    console.error('Models API error:', error);
    return NextResponse.json(
      { 
        success: false, 
        error: error instanceof Error ? error.message : 'Failed to fetch models',
        models: []
      },
      { status: 500 }
    );
  }
}
