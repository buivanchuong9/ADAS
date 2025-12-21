import { NextRequest, NextResponse } from 'next/server';

import { getApiUrl } from "@/lib/api-config";
import { API_ENDPOINTS } from "@/lib/api-endpoints";

export async function GET(request: NextRequest) {
  try {
    const response = await fetch(getApiUrl(API_ENDPOINTS.MODELS_AVAILABLE), {
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
