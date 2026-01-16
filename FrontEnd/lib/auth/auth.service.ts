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
                    message: 'Supabase client không khả dụng',
                }
            }

            // Step 1: Check if user already exists
            console.log('🔵 [SignUp] Checking if user exists...')
            const { data: existingUser } = await supabase.auth.signInWithPassword({
                email,
                password,
            })

            if (existingUser?.user) {
                console.log('⚠️ [SignUp] User already exists, signing them in')
                return {
                    success: true,
                    message: 'Tài khoản đã tồn tại, đang đăng nhập...',
                    data: existingUser,
                }
            }

            // Step 2: Create new user
            console.log('🔵 [SignUp] Creating new user...')
            const { data: signUpData, error: signUpError } = await supabase.auth.signUp({
                email,
                password,
            })

            if (signUpError) {
                console.error('❌ [SignUp] Supabase signup error:', signUpError)

                // Handle specific error cases
                if (signUpError.message.includes('already registered')) {
                    return {
                        success: false,
                        message: 'Email đã được đăng ký. Vui lòng đăng nhập.',
                    }
                }

                return {
                    success: false,
                    message: signUpError.message || 'Đã xảy ra lỗi khi đăng ký',
                }
            }

            if (!signUpData.user) {
                console.error('❌ [SignUp] No user data returned')
                return {
                    success: false,
                    message: 'Không thể tạo tài khoản',
                }
            }

            console.log('✅ [SignUp] User created:', {
                userId: signUpData.user.id,
                email: signUpData.user.email,
            })

            // Sign out immediately to ensure user must log in manually
            console.log('🔵 [SignUp] Signing out to require manual login...')
            await supabase.auth.signOut()

            return {
                success: true,
                message: 'Đăng ký thành công! Vui lòng đăng nhập.',
                data: signUpData,
            }
        } catch (err: any) {
            console.error('❌ [SignUp] Unexpected error:', err)
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
            console.log('🔵 [SignIn] Starting login for:', email)

            const supabase = getSupabase()
            if (!supabase) {
                console.error('❌ [SignIn] Supabase client not available')
                return {
                    success: false,
                    message: 'Supabase client not available',
                }
            }

            console.log('🔵 [SignIn] Calling Supabase signInWithPassword...')
            const { data, error } = await supabase.auth.signInWithPassword({
                email,
                password,
            })

            if (error) {
                console.error('❌ [SignIn] Supabase error:', error)
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

            return {
                success: true,
                message: 'Đăng nhập thành công',
                data,
            }
        } catch (err: any) {
            console.error('❌ [SignIn] Unexpected error:', err)
            return {
                success: false,
                message: err.message || 'Đã xảy ra lỗi khi đăng nhập',
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
            console.log('🔵 [GetUserInfo] Response data:', result)

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
