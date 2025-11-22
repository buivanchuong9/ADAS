// API Configuration
export const API_CONFIG = {
  BASE_URL: process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000',
  ENDPOINTS: {
    // Analytics
    ANALYTICS_DASHBOARD: '/api/analytics/dashboard',
    
    // Cameras
    CAMERAS_LIST: '/api/cameras/list',
    CAMERAS_CREATE: '/api/cameras',
    CAMERAS_GET: (id: number) => `/api/cameras/${id}`,
    CAMERAS_UPDATE: (id: number) => `/api/cameras/${id}`,
    CAMERAS_DELETE: (id: number) => `/api/cameras/${id}`,
    CAMERAS_STATUS: (id: number) => `/api/cameras/${id}/status`,
    
    // Trips
    TRIPS_LIST: '/api/trips/list',
    TRIPS_CREATE: '/api/trips',
    TRIPS_GET: (id: number) => `/api/trips/${id}`,
    TRIPS_UPDATE: (id: number) => `/api/trips/${id}`,
    TRIPS_END: (id: number) => `/api/trips/${id}/end`,
    
    // Events
    EVENTS_LIST: '/api/events/list',
    EVENTS_CREATE: '/api/events',
    EVENTS_DELETE: (id: number) => `/api/events/${id}`,
    
    // Drivers
    DRIVERS_LIST: '/api/drivers/list',
    DRIVERS_CREATE: '/api/drivers',
    DRIVERS_GET: (id: number) => `/api/drivers/${id}`,
    DRIVERS_UPDATE: (id: number) => `/api/drivers/${id}`,
    
    // Models
    MODELS_LIST: '/api/models/list',
    MODELS_DOWNLOAD: (id: number) => `/api/models/${id}/download`,
    MODELS_ACTIVATE: (id: number) => `/api/models/${id}/activate`,
    
    // WebSocket
    WS_INFERENCE: 'ws://localhost:8000/ws/infer',
    
    // Health
    HEALTH: '/health',
  }
}

// Helper function to fetch from API
export async function apiFetch<T>(
  endpoint: string,
  options?: RequestInit
): Promise<T> {
  const url = `${API_CONFIG.BASE_URL}${endpoint}`
  
  const response = await fetch(url, {
    ...options,
    headers: {
      'Content-Type': 'application/json',
      ...options?.headers,
    },
  })
  
  if (!response.ok) {
    throw new Error(`API Error: ${response.statusText}`)
  }
  
  return response.json()
}
