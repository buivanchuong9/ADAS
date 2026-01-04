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
      } catch (error) {
        console.error("Error initializing auth:", error);
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
      const result = await authService.signIn(email, password);

      if (!result.success) {
        return result;
      }

      // Fetch user info from backend
      const session = await authService.getSession();
      if (session?.access_token) {
        const userInfo = await authService.getUserInfo(session.access_token);

        if (userInfo) {
          setUser(userInfo);
          setIsAuthenticated(true);
          return {
            success: true,
            message: "Đăng nhập thành công",
          };
        } else {
          return {
            success: false,
            message: "Không thể lấy thông tin người dùng từ hệ thống",
          };
        }
      }

      return {
        success: false,
        message: "Đã xảy ra lỗi khi đăng nhập",
      };
    } catch (error: any) {
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
      // Validate password length
      if (password.length < 6) {
        return {
          success: false,
          message: "Mật khẩu phải có ít nhất 6 ký tự",
        };
      }

      const result = await authService.signUp(email, password);
      return result;
    } catch (error: any) {
      return {
        success: false,
        message: error.message || "Đã xảy ra lỗi khi đăng ký",
      };
    }
  };

  const logout = async () => {
    await authService.signOut();
    setUser(null);
    setIsAuthenticated(false);
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

