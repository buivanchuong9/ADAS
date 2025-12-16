// API Configuration for ADAS Platform
// Tự động detect môi trường và cấu hình URL phù hợp

import { API_ENDPOINTS } from './api-endpoints'

// Determine the environment
const isDevelopment = process.env.NODE_ENV === 'development'
const isClient = typeof window !== 'undefined'

// Backend API URL
const getBaseUrl = (): string => {
  const configured = process.env.NEXT_PUBLIC_API_URL
  if (configured) return configured

  // Default to official ADAS backend; no localhost fallback allowed
  return 'https://adas-api.aiotlab.edu.vn'
}

export const API_BASE_URL = getBaseUrl()

// WebSocket URL
const getWsBaseUrl = (): string => {
  const baseUrl = API_BASE_URL
  
  // Chuyển http/https thành ws/wss
  if (baseUrl.startsWith('https://')) {
    return baseUrl.replace('https://', 'wss://')
  } else if (baseUrl.startsWith('http://')) {
    return baseUrl.replace('http://', 'ws://')
  }
  
  // Default to secure websocket on ADAS host
  return 'wss://adas-api.aiotlab.edu.vn'
}

// API Configuration Object
export const API_CONFIG = {
  BASE_URL: API_BASE_URL,
  WS_BASE_URL: getWsBaseUrl(),
  ENDPOINTS: API_ENDPOINTS,
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
