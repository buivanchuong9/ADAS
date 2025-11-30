// API Configuration
// To use Cloudflare Tunnel for remote access:
// 1. Start backend: cd backend-python && python main.py
// 2. Start tunnel: ./start-tunnel.sh
// 3. Copy the generated URL and set it in .env.local:
//    NEXT_PUBLIC_API_URL=https://your-tunnel-url.trycloudflare.com
export const API_CONFIG = {
  BASE_URL: process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000',
  ENDPOINTS: {
    // ============ Core ADAS APIs ============
    
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
    
    // ============ AI/ML APIs ============
    
    // Models Management
    MODELS_LIST: '/api/models/list',
    MODELS_AVAILABLE: '/api/models/available',
    MODELS_INFO: (id: string) => `/api/models/info/${id}`,
    MODELS_DOWNLOAD: (id: string) => `/api/models/download/${id}`,
    MODELS_DOWNLOAD_ALL: '/api/models/download-all',
    MODELS_DELETE: (id: string) => `/api/models/delete/${id}`,
    MODELS_ACTIVATE: (id: number) => `/api/models/${id}/activate`,
    
    // Inference/Detection
    INFERENCE_VIDEO: '/api/inference/video',
    INFERENCE_IMAGE: '/api/inference/image',
    INFER_MODEL: (modelName: string) => `/infer/${modelName}`,
    
    // Detections
    DETECTIONS_RECENT: '/api/detections/recent',
    DETECTIONS_SAVE: '/api/detections/save',
    DETECTIONS_STATS: '/api/detections/stats',
    DETECTIONS_DELETE: (id: number) => `/api/detections/${id}`,
    
    // Alerts
    ALERTS_LATEST: '/api/alerts/latest',
    ALERTS_AUDIO: (id: number) => `/api/alerts/audio/${id}`,
    ALERTS_MARK_PLAYED: (id: number) => `/api/alerts/mark-played/${id}`,
    ALERTS_STATS: '/api/alerts/stats',
    ALERTS_CLEAR_OLD: '/api/alerts/clear-old',
    
    // ============ Dataset & Training APIs ============
    
    // Dataset Management
    DATASET_VIDEOS: '/api/dataset/videos',
    DATASET_VIDEO: (id: number) => `/api/dataset/videos/${id}`,
    DATASET_VIDEO_LABELS: (id: number) => `/api/dataset/videos/${id}/labels`,
    DATASET_VIDEO_DELETE: (id: number) => `/api/dataset/videos/${id}`,
    DATASET_STATS: '/api/dataset/stats',
    
    // Upload
    UPLOAD_VIDEO: '/api/upload/video',
    UPLOAD_VIDEO_STATUS: (id: number) => `/api/upload/video/${id}/status`,
    
    // Training
    TRAINING_START: '/api/training/start',
    TRAINING_STATUS: (id: string) => `/api/training/status/${id}`,
    TRAINING_LIST: '/api/training/list',
    TRAINING_ACTIVATE: (id: number) => `/api/training/activate/${id}`,
    
    // Auto Learning
    AUTO_LEARNING_STATS: '/api/auto-learning/stats',
    AUTO_LEARNING_TRAIN: '/api/auto-learning/train-incremental',
    AUTO_LEARNING_STATUS: (id: string) => `/api/auto-learning/training-status/${id}`,
    AUTO_LEARNING_CLEAR: '/api/auto-learning/clear-collection',
    
    // ============ WebSocket ============
    WS_INFERENCE: '/ws/infer',
    
    // ============ Health Check ============
    HEALTH: '/health',
    API_DOCS: '/docs',
  }
}

// Helper function to get WebSocket URL
export function getWebSocketUrl(endpoint: string): string {
  const baseUrl = API_CONFIG.BASE_URL
  const wsProtocol = baseUrl.startsWith('https') ? 'wss' : 'ws'
  const urlWithoutProtocol = baseUrl.replace(/^https?:\/\//, '')
  return `${wsProtocol}://${urlWithoutProtocol}${endpoint}`
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
