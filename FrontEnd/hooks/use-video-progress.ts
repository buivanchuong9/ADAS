import { useEffect, useState, useRef } from 'react'
import { getWebSocketUrl } from '@/lib/api-config'

type ProgressData = {
    type: 'connected' | 'progress' | 'finished' | 'error'
    job_id?: string
    status?: string
    progress_percent?: number
    result_path?: string | null
    error_message?: string | null
    processing_time_seconds?: number
    message?: string
}

type UseVideoProgressReturn = {
    progress: number
    status: string
    isFinished: boolean
    error: string | null
    processingTime: number | null
    isConnected: boolean
}

/**
 * Custom hook to monitor video processing progress via WebSocket
 * @param jobId - The job ID returned from video upload
 * @param enabled - Whether to start the WebSocket connection (default: true)
 * @returns Progress data and connection status
 */
export function useVideoProgress(
    jobId: string | null,
    enabled: boolean = true
): UseVideoProgressReturn {
    const [progress, setProgress] = useState(0)
    const [status, setStatus] = useState('pending')
    const [isFinished, setIsFinished] = useState(false)
    const [error, setError] = useState<string | null>(null)
    const [processingTime, setProcessingTime] = useState<number | null>(null)
    const [isConnected, setIsConnected] = useState(false)

    const wsRef = useRef<WebSocket | null>(null)
    const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null)

    useEffect(() => {
        // Don't connect if no job ID or disabled
        if (!jobId || !enabled) {
            return
        }

        // Reset state when job ID changes
        setProgress(0)
        setStatus('pending')
        setIsFinished(false)
        setError(null)
        setProcessingTime(null)
        setIsConnected(false)

        // Use the centralized WebSocket URL helper
        const wsUrl = getWebSocketUrl(`ws/video/progress/${jobId}`)
        console.log('🔌 [WebSocket] Connecting to:', wsUrl)

        try {
            const ws = new WebSocket(wsUrl)
            wsRef.current = ws

            ws.onopen = () => {
                console.log('✅ [WebSocket] Connected')
                setIsConnected(true)
            }

            ws.onmessage = (event) => {
                try {
                    const data: ProgressData = JSON.parse(event.data)
                    console.log('📨 [WebSocket] Message:', data)

                    switch (data.type) {
                        case 'connected':
                            console.log('✅ [WebSocket] Connection confirmed:', data.message)
                            break

                        case 'progress':
                            if (data.progress_percent !== undefined) {
                                setProgress(data.progress_percent)
                            }
                            if (data.status) {
                                setStatus(data.status)
                            }
                            if (data.processing_time_seconds !== undefined) {
                                setProcessingTime(data.processing_time_seconds)
                            }
                            break

                        case 'finished':
                            console.log('✅ [WebSocket] Processing finished:', data.status)
                            setIsFinished(true)
                            if (data.status) {
                                setStatus(data.status)
                            }
                            // Close the WebSocket connection
                            ws.close()
                            break

                        case 'error':
                            console.error('❌ [WebSocket] Error:', data.message)
                            setError(data.message || 'Unknown error')
                            ws.close()
                            break

                        default:
                            console.warn('⚠️ [WebSocket] Unknown message type:', data)
                    }
                } catch (err) {
                    console.error('❌ [WebSocket] Failed to parse message:', err)
                }
            }

            ws.onerror = (event) => {
                console.error('❌ [WebSocket] Connection error:', event)
                setError('WebSocket connection error')
                setIsConnected(false)
            }

            ws.onclose = (event) => {
                console.log('🔌 [WebSocket] Connection closed:', event.code, event.reason)
                setIsConnected(false)

                // Don't reconnect if finished or error
                if (!isFinished && !error && enabled) {
                    console.log('🔄 [WebSocket] Attempting to reconnect in 3s...')
                    reconnectTimeoutRef.current = setTimeout(() => {
                        // This will trigger a re-render and reconnection
                        setStatus('reconnecting')
                    }, 3000)
                }
            }
        } catch (err) {
            console.error('❌ [WebSocket] Failed to create connection:', err)
            setError('Failed to create WebSocket connection')
        }

        // Cleanup function
        return () => {
            console.log('🧹 [WebSocket] Cleaning up connection')
            if (reconnectTimeoutRef.current) {
                clearTimeout(reconnectTimeoutRef.current)
            }
            if (wsRef.current) {
                wsRef.current.close()
                wsRef.current = null
            }
        }
    }, [jobId, enabled, isFinished, error])

    return {
        progress,
        status,
        isFinished,
        error,
        processingTime,
        isConnected,
    }
}
