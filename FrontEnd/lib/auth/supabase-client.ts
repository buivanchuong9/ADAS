import { createClient, SupabaseClient } from '@supabase/supabase-js'

/**
 * Supabase Client - Lazy Initialization Pattern
 * 
 * This implementation ensures Supabase client is ONLY created in the browser,
 * never during build, SSR, or prerendering. This prevents CI/CD failures
 * while maintaining full client-side functionality.
 * 
 * Why this works:
 * 1. No top-level client creation = no build-time crashes
 * 2. Lazy initialization = client created only when needed in browser
 * 3. Singleton pattern = only one client instance created
 * 4. Type-safe = returns SupabaseClient | null
 */

let supabaseInstance: SupabaseClient | null = null

/**
 * Get Supabase client instance (browser-only)
 * Returns null during build/SSR, returns client in browser
 */
export function getSupabase(): SupabaseClient | null {
    // During build/SSR: return null immediately
    if (typeof window === 'undefined') {
        return null
    }

    // In browser: create client if not exists (singleton)
    if (!supabaseInstance) {
        const supabaseUrl = process.env.NEXT_PUBLIC_SUPABASE_URL
        const supabaseAnonKey = process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY

        if (!supabaseUrl || !supabaseAnonKey) {
            console.error(
                '❌ Supabase configuration missing!\n' +
                'Please set NEXT_PUBLIC_SUPABASE_URL and NEXT_PUBLIC_SUPABASE_ANON_KEY\n' +
                'in your environment variables or .env.local file.'
            )
            return null
        }

        supabaseInstance = createClient(supabaseUrl, supabaseAnonKey, {
            auth: {
                autoRefreshToken: true,
                persistSession: true,
                detectSessionInUrl: true,
                storage: typeof window !== 'undefined' ? window.localStorage : undefined,
                storageKey: 'adas-supabase-auth',
            },
        })

        // Add error listener for auth errors
        supabaseInstance.auth.onAuthStateChange((event, session) => {
            if (event === 'TOKEN_REFRESHED') {
                console.log('✅ Token refreshed successfully')
            } else if (event === 'SIGNED_OUT') {
                console.log('🔵 User signed out')
            }
        })
    }

    return supabaseInstance
}
