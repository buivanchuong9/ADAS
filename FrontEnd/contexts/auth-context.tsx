"use client";

import { createContext, useContext, useState, useEffect, ReactNode } from "react";
import { authService, UserInfo } from "@/lib/auth/auth.service";

interface AuthContextType {
  user: UserInfo | null;
  isAuthenticated: boolean;
  loading: boolean;
  login: (email: string, password: string) => Promise<{ success: boolean; message: string }>;
  register: (email: string, password: string) => Promise<{ success: boolean; message: string }>;
  logout: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<UserInfo | null>(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [loading, setLoading] = useState(true);

  // Initialize auth state from Supabase session
  useEffect(() => {
    // Check for existing session
    const initializeAuth = async () => {
      try {
        const session = await authService.getSession();

        if (session?.access_token) {
          // Fetch user info from backend
          const userInfo = await authService.getUserInfo(session.access_token);

          if (userInfo) {
            setUser(userInfo);
            setIsAuthenticated(true);
          }
        }
      } catch (error: any) {
        // Handle refresh token errors silently
        if (error?.message?.includes('refresh_token_not_found') ||
          error?.message?.includes('Invalid Refresh Token')) {
          console.log('🔵 [AuthContext] Clearing invalid session...');
          await authService.signOut();
        } else {
          console.error("Error initializing auth:", error);
        }
      } finally {
        setLoading(false);
      }
    };

    initializeAuth();

    // Listen to auth state changes
    const { data: { subscription } } = authService.onAuthStateChange(
      async (event, session) => {
        if (event === 'SIGNED_IN' && session?.access_token) {
          const userInfo = await authService.getUserInfo(session.access_token);
          if (userInfo) {
            setUser(userInfo);
            setIsAuthenticated(true);
          }
        } else if (event === 'SIGNED_OUT') {
          setUser(null);
          setIsAuthenticated(false);
        } else if (event === 'TOKEN_REFRESHED') {
          console.log('✅ [AuthContext] Token refreshed successfully');
        }
      }
    );

    return () => {
      subscription.unsubscribe();
    };
  }, []);

  const login = async (
    email: string,
    password: string
  ): Promise<{ success: boolean; message: string }> => {
    try {
      console.log('🔵 [AuthContext] Login attempt for:', email)

      const result = await authService.signIn(email, password);

      if (!result.success) {
        console.error('❌ [AuthContext] Login failed:', result.message)
        return result;
      }

      console.log('🔵 [AuthContext] Login successful, fetching user info...')

      // Fetch user info from backend
      const session = await authService.getSession();
      if (session?.access_token) {
        console.log('🔵 [AuthContext] Session found, access token available')

        const userInfo = await authService.getUserInfo(session.access_token);

        if (userInfo) {
          console.log('✅ [AuthContext] User info retrieved, setting auth state')
          setUser(userInfo);
          setIsAuthenticated(true);
          return {
            success: true,
            message: "Đăng nhập thành công",
          };
        } else {
          console.error('❌ [AuthContext] Failed to get user info from backend')
          return {
            success: false,
            message: "Không thể lấy thông tin người dùng từ hệ thống",
          };
        }
      }

      console.error('❌ [AuthContext] No session or access token found')
      return {
        success: false,
        message: "Đã xảy ra lỗi khi đăng nhập",
      };
    } catch (error: any) {
      console.error('❌ [AuthContext] Login error:', error)
      return {
        success: false,
        message: error.message || "Đã xảy ra lỗi khi đăng nhập",
      };
    }
  };

  const register = async (
    email: string,
    password: string
  ): Promise<{ success: boolean; message: string }> => {
    try {
      console.log('🔵 [AuthContext] Registration attempt for:', email)

      // Validate password length
      if (password.length < 6) {
        console.warn('⚠️ [AuthContext] Password too short')
        return {
          success: false,
          message: "Mật khẩu phải có ít nhất 6 ký tự",
        };
      }

      const result = await authService.signUp(email, password);

      if (result.success) {
        console.log('✅ [AuthContext] Registration successful - user must now log in manually')
      } else {
        console.error('❌ [AuthContext] Registration failed:', result.message)
      }

      return result;
    } catch (error: any) {
      console.error('❌ [AuthContext] Registration error:', error)
      return {
        success: false,
        message: error.message || "Đã xảy ra lỗi khi đăng ký",
      };
    }
  };

  const logout = async () => {
    try {
      console.log('🔵 [AuthContext] Starting logout...');

      // Sign out from Supabase
      await authService.signOut();
      console.log('✅ [AuthContext] Supabase signOut completed');

      // Clear local state
      setUser(null);
      setIsAuthenticated(false);
      console.log('✅ [AuthContext] Local state cleared');

      // Small delay to ensure state is updated
      await new Promise(resolve => setTimeout(resolve, 100));

      // ✅ Redirect to login page
      console.log('🔵 [AuthContext] Redirecting to /login...');
      window.location.href = '/login';
    } catch (error) {
      console.error('❌ [AuthContext] Logout error:', error);
      // Even if there's an error, clear local state and redirect
      setUser(null);
      setIsAuthenticated(false);
      window.location.href = '/login';
    }
  };

  return (
    <AuthContext.Provider
      value={{
        user,
        isAuthenticated,
        loading,
        login,
        register,
        logout,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error("useAuth must be used within an AuthProvider");
  }
  return context;
}

