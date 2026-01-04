import { getSupabase } from './supabase-client'
import { API_BASE_URL } from '../api-config'

export interface UserInfo {
    id: number // Integer ID from backend
    auth_id: string // UUID from Supabase
    email: string
    role?: string
    username?: string
}

export interface AuthResponse {
    success: boolean
    message: string
    data?: any
}

/**
 * Authentication Service
 * Handles all authentication operations via Supabase Auth
 */
export const authService = {
    /**
     * Sign up a new user via Supabase Auth
     */
    async signUp(email: string, password: string): Promise<AuthResponse> {
        try {
            const supabase = getSupabase()
            if (!supabase) {
                return {
                    success: false,
                    message: 'Supabase client not available',
                }
            }

            const { data, error } = await supabase.auth.signUp({
                email,
                password,
            })

            if (error) {
                return {
                    success: false,
                    message: error.message,
                }
            }

            return {
                success: true,
                message: 'Đăng ký thành công! Vui lòng đăng nhập',
                data,
            }
        } catch (err: any) {
            return {
                success: false,
                message: err.message || 'Đã xảy ra lỗi khi đăng ký',
            }
        }
    },

    /**
     * Sign in an existing user via Supabase Auth
     */
    async signIn(email: string, password: string): Promise<AuthResponse> {
        try {
            const supabase = getSupabase()
            if (!supabase) {
                return {
                    success: false,
                    message: 'Supabase client not available',
                }
            }

            const { data, error } = await supabase.auth.signInWithPassword({
                email,
                password,
            })

            if (error) {
                return {
                    success: false,
                    message: 'Email hoặc mật khẩu không đúng',
                }
            }

            return {
                success: true,
                message: 'Đăng nhập thành công',
                data,
            }
        } catch (err: any) {
            return {
                success: false,
                message: err.message || 'Đã xảy ra lỗi khi đăng nhập',
            }
        }
    },

    /**
     * Sign out the current user
     */
    async signOut(): Promise<void> {
        const supabase = getSupabase()
        if (supabase) {
            await supabase.auth.signOut()
        }
    },

    /**
     * Get the current Supabase session
     */
    async getSession() {
        const supabase = getSupabase()
        if (!supabase) return null

        const { data } = await supabase.auth.getSession()
        return data.session
    },

    /**
     * Get user info from backend API
     * This maps Supabase auth_id to the backend's integer user ID
     */
    async getUserInfo(accessToken: string): Promise<UserInfo | null> {
        try {
            const response = await fetch(`${API_BASE_URL}/api/auth/me`, {
                headers: {
                    Authorization: `Bearer ${accessToken}`,
                    'Content-Type': 'application/json',
                },
            })

            if (!response.ok) {
                throw new Error('Failed to fetch user info')
            }

            const result = await response.json()

            if (result.success && result.user) {
                return result.user
            }

            return null
        } catch (err) {
            console.error('Error fetching user info:', err)
            return null
        }
    },

    /**
     * Listen to auth state changes
     */
    onAuthStateChange(callback: (event: string, session: any) => void) {
        const supabase = getSupabase()
        if (!supabase) {
            return { data: { subscription: { unsubscribe: () => { } } } }
        }
        return supabase.auth.onAuthStateChange(callback)
    },
}
