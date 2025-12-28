"use client";

import { createContext, useContext, useState, useEffect, ReactNode } from "react";

interface User {
  id: string;
  username: string;
  email: string;
  password: string; // In real app, this would be hashed
}

interface AuthContextType {
  user: User | null;
  isAuthenticated: boolean;
  login: (username: string, password: string) => Promise<{ success: boolean; message: string }>;
  register: (username: string, email: string, password: string) => Promise<{ success: boolean; message: string }>;
  logout: () => void;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

const STORAGE_KEY_USERS = "adas_users";
const STORAGE_KEY_CURRENT_USER = "adas_current_user";

export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<User | null>(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  // Load user from localStorage on mount
  useEffect(() => {
    const storedUser = localStorage.getItem(STORAGE_KEY_CURRENT_USER);
    if (storedUser) {
      try {
        const userData = JSON.parse(storedUser);
        setUser(userData);
        setIsAuthenticated(true);
      } catch (error) {
        console.error("Error parsing stored user:", error);
        localStorage.removeItem(STORAGE_KEY_CURRENT_USER);
      }
    }
  }, []);

  // Initialize users array if it doesn't exist
  const getUsers = (): User[] => {
    const stored = localStorage.getItem(STORAGE_KEY_USERS);
    if (!stored) {
      return [];
    }
    try {
      return JSON.parse(stored);
    } catch {
      return [];
    }
  };

  const saveUsers = (users: User[]) => {
    localStorage.setItem(STORAGE_KEY_USERS, JSON.stringify(users));
  };

  const login = async (
    username: string,
    password: string
  ): Promise<{ success: boolean; message: string }> => {
    const users = getUsers();
    const foundUser = users.find(
      (u) => u.username === username && u.password === password
    );

    if (!foundUser) {
      return {
        success: false,
        message: "Tên đăng nhập hoặc mật khẩu không đúng",
      };
    }

    // Remove password from user object before storing
    const { password: _, ...userWithoutPassword } = foundUser;
    const userToStore = { ...foundUser };

    localStorage.setItem(STORAGE_KEY_CURRENT_USER, JSON.stringify(userToStore));
    setUser(foundUser);
    setIsAuthenticated(true);

    return {
      success: true,
      message: "Đăng nhập thành công",
    };
  };

  const register = async (
    username: string,
    email: string,
    password: string
  ): Promise<{ success: boolean; message: string }> => {
    const users = getUsers();

    // Check if username already exists
    if (users.some((u) => u.username === username)) {
      return {
        success: false,
        message: "Tên đăng nhập đã tồn tại",
      };
    }

    // Check if email already exists
    if (users.some((u) => u.email === email)) {
      return {
        success: false,
        message: "Email đã được sử dụng",
      };
    }

    // Validate password length
    if (password.length < 6) {
      return {
        success: false,
        message: "Mật khẩu phải có ít nhất 6 ký tự",
      };
    }

    // Create new user
    const newUser: User = {
      id: Date.now().toString(),
      username,
      email,
      password, // In production, hash this!
    };

    users.push(newUser);
    saveUsers(users);

    // Auto login after registration
    localStorage.setItem(STORAGE_KEY_CURRENT_USER, JSON.stringify(newUser));
    setUser(newUser);
    setIsAuthenticated(true);

    return {
      success: true,
      message: "Đăng ký thành công",
    };
  };

  const logout = () => {
    localStorage.removeItem(STORAGE_KEY_CURRENT_USER);
    setUser(null);
    setIsAuthenticated(false);
  };

  return (
    <AuthContext.Provider
      value={{
        user,
        isAuthenticated,
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

