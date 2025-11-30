/**
 * ADAS Frontend - API Client
 * Type-safe API client with error handling and retry logic
 */

import { API_CONFIG } from './api-config'

// ============ Types ============

export interface APIError {
  message: string
  code?: string
  status?: number
}

export interface APIResponse<T = any> {
  status: 'success' | 'error' | 'warning'
  data?: T
  message?: string
  error?: string
  error_code?: string
  timestamp: string
}

export interface PaginatedResponse<T> {
  status: 'success' | 'error'
  data: T[]
  total: number
  page: number
  page_size: number
  total_pages: number
  has_next: boolean
  has_prev: boolean
}

// ============ API Client Class ============

class APIClient {
  private baseURL: string
  private defaultHeaders: HeadersInit

  constructor(baseURL?: string) {
    this.baseURL = baseURL || API_CONFIG.BASE_URL
    this.defaultHeaders = {
      'Content-Type': 'application/json',
    }
  }

  /**
   * Generic fetch with error handling
   */
  private async fetch<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<T> {
    const url = `${this.baseURL}${endpoint}`
    
    try {
      const response = await fetch(url, {
        ...options,
        headers: {
          ...this.defaultHeaders,
          ...options.headers,
        },
      })

      // Handle HTTP errors
      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}))
        throw {
          message: errorData.error || errorData.message || `HTTP ${response.status}: ${response.statusText}`,
          code: errorData.error_code,
          status: response.status,
        } as APIError
      }

      const data = await response.json()
      return data
    } catch (error) {
      // Network or parsing errors
      if (error && typeof error === 'object' && 'message' in error) {
        throw error
      }
      throw {
        message: error instanceof Error ? error.message : 'Network error occurred',
        code: 'NETWORK_ERROR',
      } as APIError
    }
  }

  /**
   * GET request
   */
  async get<T>(endpoint: string, params?: Record<string, string | number>): Promise<T> {
    let url = endpoint
    if (params) {
      const searchParams = new URLSearchParams()
      Object.entries(params).forEach(([key, value]) => {
        searchParams.append(key, String(value))
      })
      url += `?${searchParams.toString()}`
    }

    return this.fetch<T>(url, { method: 'GET' })
  }

  /**
   * POST request
   */
  async post<T>(endpoint: string, body?: any): Promise<T> {
    return this.fetch<T>(endpoint, {
      method: 'POST',
      body: body ? JSON.stringify(body) : undefined,
    })
  }

  /**
   * PUT request
   */
  async put<T>(endpoint: string, body?: any): Promise<T> {
    return this.fetch<T>(endpoint, {
      method: 'PUT',
      body: body ? JSON.stringify(body) : undefined,
    })
  }

  /**
   * DELETE request
   */
  async delete<T>(endpoint: string): Promise<T> {
    return this.fetch<T>(endpoint, { method: 'DELETE' })
  }

  /**
   * Upload file
   */
  async upload<T>(endpoint: string, file: File, additionalData?: Record<string, any>): Promise<T> {
    const formData = new FormData()
    formData.append('file', file)
    
    if (additionalData) {
      Object.entries(additionalData).forEach(([key, value]) => {
        formData.append(key, String(value))
      })
    }

    return this.fetch<T>(endpoint, {
      method: 'POST',
      body: formData,
      headers: {}, // Let browser set Content-Type for FormData
    })
  }

  /**
   * Update base URL (e.g., when tunnel URL changes)
   */
  setBaseURL(url: string) {
    this.baseURL = url
  }
}

// ============ API Service Functions ============

export const apiClient = new APIClient()

// Health & Status
export const healthCheck = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.HEALTH)

export const getAPIStatus = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.API_DOCS || '/api/status')

// Cameras
export const getCameras = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.CAMERAS_LIST)

export const getCamera = (id: number) => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.CAMERAS_GET(id))

export const createCamera = (data: any) => 
  apiClient.post<APIResponse>(API_CONFIG.ENDPOINTS.CAMERAS_CREATE, data)

export const updateCamera = (id: number, data: any) => 
  apiClient.put<APIResponse>(API_CONFIG.ENDPOINTS.CAMERAS_UPDATE(id), data)

export const deleteCamera = (id: number) => 
  apiClient.delete<APIResponse>(API_CONFIG.ENDPOINTS.CAMERAS_DELETE(id))

// Detections
export const getRecentDetections = (limit?: number) => 
  apiClient.get<APIResponse>(
    API_CONFIG.ENDPOINTS.DETECTIONS_RECENT,
    limit ? { limit } : undefined
  )

export const getDetectionStats = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.DETECTIONS_STATS)

export const saveDetection = (data: any) => 
  apiClient.post<APIResponse>(API_CONFIG.ENDPOINTS.DETECTIONS_SAVE, data)

// Alerts
export const getLatestAlerts = (limit?: number) => 
  apiClient.get<APIResponse>(
    API_CONFIG.ENDPOINTS.ALERTS_LATEST,
    limit ? { limit } : undefined
  )

export const getAlertStats = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.ALERTS_STATS)

export const markAlertPlayed = (id: number) => 
  apiClient.post<APIResponse>(API_CONFIG.ENDPOINTS.ALERTS_MARK_PLAYED(id))

// Models
export const getModels = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.MODELS_LIST)

export const getAvailableModels = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.MODELS_AVAILABLE)

export const downloadModel = (id: string) => 
  apiClient.post<APIResponse>(API_CONFIG.ENDPOINTS.MODELS_DOWNLOAD(id))

export const deleteModel = (id: string) => 
  apiClient.delete<APIResponse>(API_CONFIG.ENDPOINTS.MODELS_DELETE(id))

// Dataset
export const getDatasetVideos = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.DATASET_VIDEOS)

export const getDatasetStats = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.DATASET_STATS)

export const uploadVideo = (file: File) => 
  apiClient.upload<APIResponse>(API_CONFIG.ENDPOINTS.UPLOAD_VIDEO, file)

// Training
export const startTraining = (config: any) => 
  apiClient.post<APIResponse>(API_CONFIG.ENDPOINTS.TRAINING_START, config)

export const getTrainingStatus = (id: string) => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.TRAINING_STATUS(id))

export const getTrainingList = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.TRAINING_LIST)

// Auto Learning
export const getAutoLearningStats = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.AUTO_LEARNING_STATS)

export const triggerIncrementalTraining = () => 
  apiClient.post<APIResponse>(API_CONFIG.ENDPOINTS.AUTO_LEARNING_TRAIN)

// Analytics
export const getDashboardStats = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.ANALYTICS_DASHBOARD)

// Trips
export const getTrips = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.TRIPS_LIST)

export const createTrip = (data: any) => 
  apiClient.post<APIResponse>(API_CONFIG.ENDPOINTS.TRIPS_CREATE, data)

export const endTrip = (id: number) => 
  apiClient.post<APIResponse>(API_CONFIG.ENDPOINTS.TRIPS_END(id))

// Events
export const getEvents = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.EVENTS_LIST)

export const createEvent = (data: any) => 
  apiClient.post<APIResponse>(API_CONFIG.ENDPOINTS.EVENTS_CREATE, data)

// Drivers
export const getDrivers = () => 
  apiClient.get<APIResponse>(API_CONFIG.ENDPOINTS.DRIVERS_LIST)

export const createDriver = (data: any) => 
  apiClient.post<APIResponse>(API_CONFIG.ENDPOINTS.DRIVERS_CREATE, data)

// ============ WebSocket Client ============

export class WebSocketClient {
  private ws: WebSocket | null = null
  private reconnectAttempts = 0
  private maxReconnectAttempts = 5
  private reconnectDelay = 1000
  private messageHandlers: Map<string, (data: any) => void> = new Map()

  constructor(private endpoint: string) {}

  connect() {
    const wsUrl = API_CONFIG.BASE_URL.replace(/^http/, 'ws') + this.endpoint
    
    this.ws = new WebSocket(wsUrl)

    this.ws.onopen = () => {
      console.log('WebSocket connected')
      this.reconnectAttempts = 0
    }

    this.ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data)
        const handler = this.messageHandlers.get(message.type)
        if (handler) {
          handler(message.data)
        }
      } catch (error) {
        console.error('WebSocket message parse error:', error)
      }
    }

    this.ws.onerror = (error) => {
      console.error('WebSocket error:', error)
    }

    this.ws.onclose = () => {
      console.log('WebSocket closed')
      this.attemptReconnect()
    }
  }

  private attemptReconnect() {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++
      setTimeout(() => {
        console.log(`Reconnecting... (attempt ${this.reconnectAttempts})`)
        this.connect()
      }, this.reconnectDelay * this.reconnectAttempts)
    }
  }

  on(messageType: string, handler: (data: any) => void) {
    this.messageHandlers.set(messageType, handler)
  }

  send(type: string, data: any) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({ type, data }))
    }
  }

  disconnect() {
    if (this.ws) {
      this.ws.close()
      this.ws = null
    }
    this.messageHandlers.clear()
  }
}

export default apiClient
