import { supabase } from '@/lib/auth/supabase-client'
import { API_BASE_URL } from '../api-config'

export interface ApiClientOptions extends RequestInit {
    skipAuth?: boolean // Skip adding Authorization header
}

/**
 * HTTP Client with automatic authentication
 * Automatically attaches JWT token to all requests
 */
export async function apiClient(
    endpoint: string,
    options: ApiClientOptions = {}
): Promise<Response> {
    const { skipAuth = false, ...fetchOptions } = options

    // Get current session
    const { data: { session } } = await supabase.auth.getSession()

    // Prepare headers
    const headers: Record<string, string> = {
        'Content-Type': 'application/json',
        ...(fetchOptions.headers as Record<string, string>),
    }

    // Add Authorization header if user is authenticated and not skipped
    if (!skipAuth && session?.access_token) {
        headers['Authorization'] = `Bearer ${session.access_token}`
    }

    // Construct full URL
    const url = endpoint.startsWith('http')
        ? endpoint
        : `${API_BASE_URL}${endpoint.startsWith('/') ? endpoint : `/${endpoint}`}`

    // Make request
    const response = await fetch(url, {
        ...fetchOptions,
        headers,
    })

    return response
}

/**
 * Convenience methods for common HTTP verbs
 */
export const api = {
    async get(endpoint: string, options?: ApiClientOptions) {
        return apiClient(endpoint, { ...options, method: 'GET' })
    },

    async post(endpoint: string, body?: any, options?: ApiClientOptions) {
        return apiClient(endpoint, {
            ...options,
            method: 'POST',
            body: body ? JSON.stringify(body) : undefined,
        })
    },

    async put(endpoint: string, body?: any, options?: ApiClientOptions) {
        return apiClient(endpoint, {
            ...options,
            method: 'PUT',
            body: body ? JSON.stringify(body) : undefined,
        })
    },

    async delete(endpoint: string, options?: ApiClientOptions) {
        return apiClient(endpoint, { ...options, method: 'DELETE' })
    },

    async patch(endpoint: string, body?: any, options?: ApiClientOptions) {
        return apiClient(endpoint, {
            ...options,
            method: 'PATCH',
            body: body ? JSON.stringify(body) : undefined,
        })
    },
}
