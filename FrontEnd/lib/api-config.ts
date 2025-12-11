// API Configuration for ADAS Platform
// Tự động detect môi trường và cấu hình URL phù hợp

// Determine the environment
const isDevelopment = process.env.NODE_ENV === 'development'
const isClient = typeof window !== 'undefined'

// Backend API URL
const getBaseUrl = (): string => {
  // Trong môi trường development
  if (isDevelopment) {
    // Nếu đang chạy trên client (browser)
    if (isClient) {
      return 'http://localhost:8000'
    }
    // Nếu đang chạy trên server (SSR)
    return process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000'
  }

  // Trong môi trường production
  if (isClient) {
    // Sử dụng window.location để tự động detect
    return process.env.NEXT_PUBLIC_API_URL || `${window.location.protocol}//${window.location.hostname}:8000`
  }

  // SSR trong production
  return process.env.NEXT_PUBLIC_API_URL || 'http://backend:8000'
}

// WebSocket URL
const getWsBaseUrl = (): string => {
  const baseUrl = getBaseUrl()
  
  // Chuyển http/https thành ws/wss
  if (baseUrl.startsWith('https://')) {
    return baseUrl.replace('https://', 'wss://')
  } else if (baseUrl.startsWith('http://')) {
    return baseUrl.replace('http://', 'ws://')
  }
  
  // Default fallback
  return `ws://localhost:8000`
}

// API Configuration Object
export const API_CONFIG = {
  BASE_URL: getBaseUrl(),
  WS_BASE_URL: getWsBaseUrl(),
  
  // API Endpoints
  ENDPOINTS: {
    // System
    STATUS: '/api/status',
    HEALTH: '/health',
    
    // ADAS
    ADAS_STREAM: '/ws/adas/stream',
    ADAS_CONFIG: '/api/adas/config',
    
    // Alerts
    ALERTS: '/api/alerts',
    ALERTS_STATS: '/api/alerts/stats',
    
    // Detections
    DETECTIONS: '/api/detections',
    DETECTIONS_STATS: '/api/detections/stats',
    
    // Video Upload
    VIDEO_UPLOAD: '/api/upload/video',
    VIDEO_INFERENCE: '/ws/inference/video',
    
    // Models
    MODELS: '/api/models',
    MODELS_WEBCAM: '/ws/models/webcam',
    
    // Analytics
    ANALYTICS: '/api/analytics',
    
    // Driver Monitoring
    DRIVER_MONITOR: '/api/driver-monitoring',
    
    // Dataset
    DATASET: '/api/dataset',
  },
  
  // Timeout settings
  TIMEOUT: {
    REQUEST: 30000, // 30 seconds
    WS_RECONNECT: 3000, // 3 seconds
  },
} as const

// Helper function to get WebSocket URL
export const getWebSocketUrl = (endpoint: string): string => {
  const wsBaseUrl = API_CONFIG.WS_BASE_URL
  
  // Remove leading slash if exists
  const cleanEndpoint = endpoint.startsWith('/') ? endpoint.slice(1) : endpoint
  
  // Construct full WebSocket URL
  return `${wsBaseUrl}/${cleanEndpoint}`
}

// Helper function to get API URL
export const getApiUrl = (endpoint: string): string => {
  const baseUrl = API_CONFIG.BASE_URL
  
  // Remove leading slash if exists
  const cleanEndpoint = endpoint.startsWith('/') ? endpoint.slice(1) : endpoint
  
  // Construct full API URL
  return `${baseUrl}/${cleanEndpoint}`
}

// Export type for TypeScript
export type ApiEndpoint = keyof typeof API_CONFIG.ENDPOINTS
