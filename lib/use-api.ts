/**
 * ADAS Frontend - React Hooks for API Data
 * Custom hooks with automatic refresh, loading states, and error handling
 */

import { useState, useEffect, useCallback, useRef } from 'react'
import * as api from './api-client'

// ============ Types ============

interface UseDataOptions {
  refreshInterval?: number // milliseconds
  enabled?: boolean
  onError?: (error: any) => void
  onSuccess?: (data: any) => void
}

interface UseDataReturn<T> {
  data: T | null
  loading: boolean
  error: Error | null
  refetch: () => Promise<void>
  isRefetching: boolean
}

// ============ Generic Data Hook ============

function useData<T>(
  fetcher: () => Promise<any>,
  options: UseDataOptions = {}
): UseDataReturn<T> {
  const {
    refreshInterval,
    enabled = true,
    onError,
    onSuccess,
  } = options

  const [data, setData] = useState<T | null>(null)
  const [loading, setLoading] = useState(true)
  const [isRefetching, setIsRefetching] = useState(false)
  const [error, setError] = useState<Error | null>(null)
  
  const intervalRef = useRef<NodeJS.Timeout>()
  const mountedRef = useRef(true)

  const fetchData = useCallback(async (isRefetch = false) => {
    if (!enabled) return

    try {
      if (isRefetch) {
        setIsRefetching(true)
      } else {
        setLoading(true)
      }
      
      setError(null)
      
      const response = await fetcher()
      
      if (!mountedRef.current) return
      
      // Handle APIResponse format
      const responseData = response.data !== undefined ? response.data : response
      
      setData(responseData)
      onSuccess?.(responseData)
    } catch (err) {
      if (!mountedRef.current) return
      
      const error = err instanceof Error ? err : new Error(String(err))
      setError(error)
      onError?.(error)
      console.error('Data fetch error:', error)
    } finally {
      if (mountedRef.current) {
        setLoading(false)
        setIsRefetching(false)
      }
    }
  }, [fetcher, enabled, onSuccess, onError])

  useEffect(() => {
    mountedRef.current = true
    fetchData()

    return () => {
      mountedRef.current = false
    }
  }, [fetchData])

  // Auto-refresh
  useEffect(() => {
    if (refreshInterval && enabled) {
      intervalRef.current = setInterval(() => {
        fetchData(true)
      }, refreshInterval)

      return () => {
        if (intervalRef.current) {
          clearInterval(intervalRef.current)
        }
      }
    }
  }, [refreshInterval, enabled, fetchData])

  const refetch = useCallback(async () => {
    await fetchData(true)
  }, [fetchData])

  return { data, loading, error, refetch, isRefetching }
}

// ============ Specific Hooks ============

export function useHealth(options?: UseDataOptions) {
  return useData(api.healthCheck, options)
}

export function useCameras(options?: UseDataOptions) {
  return useData(api.getCameras, { refreshInterval: 5000, ...options })
}

export function useCamera(id: number, options?: UseDataOptions) {
  return useData(() => api.getCamera(id), options)
}

export function useDetections(limit?: number, options?: UseDataOptions) {
  return useData(
    () => api.getRecentDetections(limit),
    { refreshInterval: 3000, ...options }
  )
}

export function useDetectionStats(options?: UseDataOptions) {
  return useData(
    api.getDetectionStats,
    { refreshInterval: 5000, ...options }
  )
}

export function useAlerts(limit?: number, options?: UseDataOptions) {
  return useData(
    () => api.getLatestAlerts(limit),
    { refreshInterval: 2000, ...options }
  )
}

export function useAlertStats(options?: UseDataOptions) {
  return useData(
    api.getAlertStats,
    { refreshInterval: 5000, ...options }
  )
}

export function useModels(options?: UseDataOptions) {
  return useData(api.getAvailableModels, { refreshInterval: 10000, ...options })
}

export function useDatasetStats(options?: UseDataOptions) {
  return useData(api.getDatasetStats, { refreshInterval: 10000, ...options })
}

export function useTrainingList(options?: UseDataOptions) {
  return useData(api.getTrainingList, { refreshInterval: 5000, ...options })
}

export function useTrainingStatus(id: string, options?: UseDataOptions) {
  return useData(
    () => api.getTrainingStatus(id),
    { refreshInterval: 2000, ...options }
  )
}

export function useAutoLearningStats(options?: UseDataOptions) {
  return useData(
    api.getAutoLearningStats,
    { refreshInterval: 10000, ...options }
  )
}

export function useDashboardStats(options?: UseDataOptions) {
  return useData(
    api.getDashboardStats,
    { refreshInterval: 5000, ...options }
  )
}

export function useTrips(options?: UseDataOptions) {
  return useData(api.getTrips, { refreshInterval: 5000, ...options })
}

export function useEvents(options?: UseDataOptions) {
  return useData(api.getEvents, { refreshInterval: 5000, ...options })
}

export function useDrivers(options?: UseDataOptions) {
  return useData(api.getDrivers, options)
}

// ============ WebSocket Hook ============

interface UseWebSocketOptions {
  endpoint: string
  onMessage?: (type: string, data: any) => void
  autoConnect?: boolean
}

export function useWebSocket(options: UseWebSocketOptions) {
  const { endpoint, onMessage, autoConnect = true } = options
  
  const wsRef = useRef<api.WebSocketClient | null>(null)
  const [connected, setConnected] = useState(false)
  const [lastMessage, setLastMessage] = useState<any>(null)

  useEffect(() => {
    if (!autoConnect) return

    const ws = new api.WebSocketClient(endpoint)
    wsRef.current = ws

    ws.on('detection', (data) => {
      setLastMessage({ type: 'detection', data })
      onMessage?.('detection', data)
    })

    ws.on('alert', (data) => {
      setLastMessage({ type: 'alert', data })
      onMessage?.('alert', data)
    })

    ws.on('status', (data) => {
      setConnected(true)
      setLastMessage({ type: 'status', data })
      onMessage?.('status', data)
    })

    ws.connect()

    return () => {
      ws.disconnect()
      wsRef.current = null
      setConnected(false)
    }
  }, [endpoint, autoConnect, onMessage])

  const send = useCallback((type: string, data: any) => {
    wsRef.current?.send(type, data)
  }, [])

  return { connected, lastMessage, send }
}

// ============ Mutation Hooks ============

interface UseMutationOptions<T> {
  onSuccess?: (data: T) => void
  onError?: (error: Error) => void
}

export function useMutation<T = any, P = any>(
  mutationFn: (params: P) => Promise<T>,
  options?: UseMutationOptions<T>
) {
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState<Error | null>(null)
  const [data, setData] = useState<T | null>(null)

  const mutate = useCallback(
    async (params: P) => {
      try {
        setLoading(true)
        setError(null)
        const result = await mutationFn(params)
        setData(result)
        options?.onSuccess?.(result)
        return result
      } catch (err) {
        const error = err instanceof Error ? err : new Error(String(err))
        setError(error)
        options?.onError?.(error)
        throw error
      } finally {
        setLoading(false)
      }
    },
    [mutationFn, options]
  )

  return { mutate, loading, error, data }
}

// Specific mutation hooks
export function useCreateCamera(options?: UseMutationOptions<any>) {
  return useMutation(api.createCamera, options)
}

export function useUpdateCamera(options?: UseMutationOptions<any>) {
  return useMutation(
    ({ id, data }: { id: number; data: any }) => api.updateCamera(id, data),
    options
  )
}

export function useDeleteCamera(options?: UseMutationOptions<any>) {
  return useMutation((id: number) => api.deleteCamera(id), options)
}

export function useStartTraining(options?: UseMutationOptions<any>) {
  return useMutation(api.startTraining, options)
}

export function useUploadVideo(options?: UseMutationOptions<any>) {
  return useMutation((file: File) => api.uploadVideo(file), options)
}

export function useMarkAlertPlayed(options?: UseMutationOptions<any>) {
  return useMutation((id: number) => api.markAlertPlayed(id), options)
}

// ============ Polling Hook ============

export function usePolling<T>(
  fetcher: () => Promise<T>,
  interval: number,
  condition?: (data: T | null) => boolean
) {
  const [data, setData] = useState<T | null>(null)
  const [isPolling, setIsPolling] = useState(false)

  useEffect(() => {
    if (condition && data && !condition(data)) {
      return // Stop polling if condition is met
    }

    setIsPolling(true)
    const poll = async () => {
      try {
        const result = await fetcher()
        setData(result)
      } catch (error) {
        console.error('Polling error:', error)
      }
    }

    poll() // Initial fetch
    const intervalId = setInterval(poll, interval)

    return () => {
      clearInterval(intervalId)
      setIsPolling(false)
    }
  }, [fetcher, interval, data, condition])

  return { data, isPolling }
}
