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
     * After successful registration, automatically signs in the user
     * so backend can create user record on first /api/auth/me call
     */
    async signUp(email: string, password: string): Promise<AuthResponse> {
        try {
            console.log('🔵 [SignUp] Starting registration for:', email)

            const supabase = getSupabase()
            if (!supabase) {
                console.error('❌ [SignUp] Supabase client not available')
                return {
                    success: false,
                    message: 'Dịch vụ xác thực không khả dụng',
                }
            }

            // Create new user directly - Supabase will handle duplicate email validation
            console.log('🔵 [SignUp] Creating new user...')
            const { data: signUpData, error: signUpError } = await supabase.auth.signUp({
                email,
                password,
            })

            if (signUpError) {
                console.error('❌ [SignUp] Supabase signup error:', signUpError.message)

                // Handle specific error cases with user-friendly messages
                if (signUpError.message.includes('already registered') ||
                    signUpError.message.includes('User already registered')) {
                    return {
                        success: false,
                        message: 'Email đã được đăng ký. Vui lòng đăng nhập.',
                    }
                }

                if (signUpError.message.includes('Password')) {
                    return {
                        success: false,
                        message: 'Mật khẩu không hợp lệ. Vui lòng thử lại.',
                    }
                }

                // Generic error message to avoid exposing internal details
                return {
                    success: false,
                    message: 'Đã xảy ra lỗi khi đăng ký. Vui lòng thử lại.',
                }
            }

            if (!signUpData.user) {
                console.error('❌ [SignUp] No user data returned')
                return {
                    success: false,
                    message: 'Không thể tạo tài khoản. Vui lòng thử lại.',
                }
            }

            console.log('✅ [SignUp] User created:', {
                userId: signUpData.user.id,
                email: signUpData.user.email,
            })

            // Sign out immediately to ensure user must log in manually
            console.log('🔵 [SignUp] Signing out to require manual login...')
            await supabase.auth.signOut()

            // Don't return sensitive data - only success message
            return {
                success: true,
                message: 'Đăng ký thành công! Vui lòng đăng nhập.',
            }
        } catch (err: any) {
            console.error('❌ [SignUp] Unexpected error:', err)
            // Don't expose internal error details to user
            return {
                success: false,
                message: 'Đã xảy ra lỗi khi đăng ký. Vui lòng thử lại sau.',
            }
        }
    },

    /**
     * Sign in an existing user via Supabase Auth
     */
    async signIn(email: string, password: string): Promise<AuthResponse> {
        try {
            console.log('🔵 [SignIn] Starting login for:', email)

            const supabase = getSupabase()
            if (!supabase) {
                console.error('❌ [SignIn] Supabase client not available')
                return {
                    success: false,
                    message: 'Dịch vụ xác thực không khả dụng',
                }
            }

            console.log('🔵 [SignIn] Calling Supabase signInWithPassword...')
            const { data, error } = await supabase.auth.signInWithPassword({
                email,
                password,
            })

            if (error) {
                console.error('❌ [SignIn] Supabase error:', error.message)
                // Don't expose internal error details - use generic message
                return {
                    success: false,
                    message: 'Email hoặc mật khẩu không đúng',
                }
            }

            console.log('✅ [SignIn] Login successful:', {
                userId: data?.user?.id,
                email: data?.user?.email,
                hasSession: !!data?.session,
            })

            // Don't return sensitive data - session is already stored in Supabase client
            return {
                success: true,
                message: 'Đăng nhập thành công',
            }
        } catch (err: any) {
            console.error('❌ [SignIn] Unexpected error:', err)
            // Don't expose internal error details to user
            return {
                success: false,
                message: 'Đã xảy ra lỗi khi đăng nhập. Vui lòng thử lại.',
            }
        }
    },

    /**
     * Sign out the current user
     * Clears Supabase session and all local storage
     */
    async signOut(): Promise<void> {
        try {
            console.log('🔵 [SignOut] Starting sign out...')

            const supabase = getSupabase()
            if (supabase) {
                // Sign out from Supabase
                await supabase.auth.signOut()
                console.log('✅ [SignOut] Supabase auth.signOut() completed')
            }

            // Clear all auth-related localStorage items
            if (typeof window !== 'undefined') {
                // Clear Supabase auth storage
                localStorage.removeItem('adas-supabase-auth')

                // Clear any other Supabase keys (fallback)
                Object.keys(localStorage).forEach(key => {
                    if (key.includes('supabase') || key.includes('auth')) {
                        localStorage.removeItem(key)
                    }
                })

                console.log('✅ [SignOut] LocalStorage cleared')
            }
        } catch (error) {
            console.error('❌ [SignOut] Error during sign out:', error)
            // Even if there's an error, try to clear localStorage
            if (typeof window !== 'undefined') {
                localStorage.removeItem('adas-supabase-auth')
            }
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
            console.log('🔵 [GetUserInfo] Fetching user info from backend...')
            console.log('🔵 [GetUserInfo] API URL:', `${API_BASE_URL}/api/auth/me`)

            const response = await fetch(`${API_BASE_URL}/api/auth/me`, {
                headers: {
                    Authorization: `Bearer ${accessToken}`,
                    'Content-Type': 'application/json',
                },
            })

            console.log('🔵 [GetUserInfo] Response status:', response.status)

            if (!response.ok) {
                console.error('❌ [GetUserInfo] Failed to fetch user info:', {
                    status: response.status,
                    statusText: response.statusText,
                })
                throw new Error(`Failed to fetch user info: ${response.status}`)
            }

            const result = await response.json()

            if (result.success && result.user) {
                console.log('✅ [GetUserInfo] User info retrieved:', {
                    id: result.user.id,
                    email: result.user.email,
                    role: result.user.role,
                })
                return result.user
            }

            console.warn('⚠️ [GetUserInfo] No user data in response')
            return null
        } catch (err) {
            console.error('❌ [GetUserInfo] Error:', err)
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
