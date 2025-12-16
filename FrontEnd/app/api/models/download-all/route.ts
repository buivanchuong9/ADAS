import { NextRequest, NextResponse } from 'next/server';

import { getApiUrl } from "@/lib/api-config";
import { API_ENDPOINTS } from "@/lib/api-endpoints";

export async function POST(request: NextRequest) {
  try {
    const response = await fetch(getApiUrl(API_ENDPOINTS.MODELS_DOWNLOAD_ALL), {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      throw new Error(`Backend error: ${response.status}`);
    }

    const data = await response.json();
    return NextResponse.json(data);
  } catch (error) {
    console.error('Download all models error:', error);
    return NextResponse.json(
      { 
        success: false, 
        error: error instanceof Error ? error.message : 'Failed to download models'
      },
      { status: 500 }
    );
  }
}
