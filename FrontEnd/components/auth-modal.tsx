"use client";

import { useState, useEffect } from "react";
import { motion, AnimatePresence } from "framer-motion";
import { useAuth } from "@/contexts/auth-context";
import { X, LogIn, UserPlus, Mail, Lock, User as UserIcon } from "lucide-react";

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
}

export function AuthModal({ isOpen, onClose }: AuthModalProps) {
  const { isAuthenticated, login, register } = useAuth();
  const [mode, setMode] = useState<"login" | "register">("login");
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");
  const [success, setSuccess] = useState("");

  // Form states
  const [loginUsername, setLoginUsername] = useState("");
  const [loginPassword, setLoginPassword] = useState("");
  const [registerUsername, setRegisterUsername] = useState("");
  const [registerEmail, setRegisterEmail] = useState("");
  const [registerPassword, setRegisterPassword] = useState("");
  const [registerConfirmPassword, setRegisterConfirmPassword] = useState("");

  // Close modal if user becomes authenticated or if already authenticated
  useEffect(() => {
    if (isAuthenticated) {
      onClose();
      setSuccess("Đăng nhập thành công!");
      setTimeout(() => setSuccess(""), 3000);
    }
  }, [isAuthenticated, onClose]);

  // Prevent modal from opening if already authenticated
  useEffect(() => {
    if (isOpen && isAuthenticated) {
      onClose();
    }
  }, [isOpen, isAuthenticated, onClose]);

  // Reset form when switching modes
  useEffect(() => {
    setError("");
    setSuccess("");
    setLoginUsername("");
    setLoginPassword("");
    setRegisterUsername("");
    setRegisterEmail("");
    setRegisterPassword("");
    setRegisterConfirmPassword("");
  }, [mode]);

  // Prevent body scroll when modal is open
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = "hidden";
    } else {
      document.body.style.overflow = "unset";
    }
    return () => {
      document.body.style.overflow = "unset";
    };
  }, [isOpen]);

  const handleLogin = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setSuccess("");

    if (!loginUsername || !loginPassword) {
      setError("Vui lòng điền đầy đủ thông tin");
      return;
    }

    setLoading(true);
    try {
      const result = await login(loginUsername, loginPassword);
      if (result.success) {
        setSuccess(result.message);
        setTimeout(() => {
          onClose();
        }, 1000);
      } else {
        setError(result.message);
      }
    } catch (err) {
      setError("Đã xảy ra lỗi. Vui lòng thử lại.");
    } finally {
      setLoading(false);
    }
  };

  const handleRegister = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setSuccess("");

    if (!registerUsername || !registerEmail || !registerPassword || !registerConfirmPassword) {
      setError("Vui lòng điền đầy đủ thông tin");
      return;
    }

    if (registerPassword !== registerConfirmPassword) {
      setError("Mật khẩu xác nhận không khớp");
      return;
    }

    setLoading(true);
    try {
      const result = await register(registerUsername, registerEmail, registerPassword);
      if (result.success) {
        setSuccess(result.message);
        setTimeout(() => {
          onClose();
        }, 1000);
      } else {
        setError(result.message);
      }
    } catch (err) {
      setError("Đã xảy ra lỗi. Vui lòng thử lại.");
    } finally {
      setLoading(false);
    }
  };

  if (!isOpen) return null;

  return (
    <AnimatePresence>
      {isOpen && (
        <>
          {/* Backdrop */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            onClick={onClose}
            className="fixed inset-0 z-50 glass-overlay flex items-center justify-center p-4"
            style={{
              backgroundColor: 'rgba(0, 0, 0, 0.95)',
              backdropFilter: 'blur(20px)',
              WebkitBackdropFilter: 'blur(20px)',
            }}
          >
            {/* Modal */}
            <motion.div
              initial={{ opacity: 0, scale: 0.9, y: 20 }}
              animate={{ opacity: 1, scale: 1, y: 0 }}
              exit={{ opacity: 0, scale: 0.9, y: 20 }}
              onClick={(e) => e.stopPropagation()}
              className="relative w-full max-w-md glass-card scan-lines p-8 rounded-3xl border-neon-cyan/50 glow-cyan"
            >
              {/* Close button */}
              <button
                onClick={onClose}
                className="absolute top-4 right-4 w-8 h-8 flex items-center justify-center rounded-lg glass-card border border-white/10 hover:border-neon-cyan/50 text-fg-secondary hover:text-neon-cyan transition-all"
              >
                <X className="w-5 h-5" />
              </button>

              {/* Animated gradient orbs */}
              <motion.div
                className="absolute top-0 right-0 w-64 h-64 bg-neon-cyan/10 rounded-full blur-3xl"
                animate={{
                  scale: [1, 1.2, 1],
                  opacity: [0.2, 0.4, 0.2],
                }}
                transition={{
                  duration: 8,
                  repeat: Infinity,
                  ease: "easeInOut",
                }}
              />

              <div className="relative z-10">
                {/* Header */}
                <div className="text-center mb-8">
                  <motion.div
                    initial={{ scale: 0 }}
                    animate={{ scale: 1 }}
                    transition={{ delay: 0.2, type: "spring" }}
                    className="w-16 h-16 mx-auto mb-4 rounded-2xl bg-gradient-to-br from-primary to-accent flex items-center justify-center shadow-lg shadow-primary/30"
                  >
                    {mode === "login" ? (
                      <LogIn className="w-8 h-8 text-white" />
                    ) : (
                      <UserPlus className="w-8 h-8 text-white" />
                    )}
                  </motion.div>
                  <h2 className="text-2xl font-bold text-neon-cyan mb-2">
                    {mode === "login" ? "Đăng Nhập" : "Đăng Ký"}
                  </h2>
                  <p className="text-sm text-fg-secondary">
                    {mode === "login"
                      ? "Nhập thông tin để truy cập hệ thống"
                      : "Tạo tài khoản mới để bắt đầu"}
                  </p>
                </div>

                {/* Error/Success messages */}
                {error && (
                  <motion.div
                    initial={{ opacity: 0, y: -10 }}
                    animate={{ opacity: 1, y: 0 }}
                    className="mb-4 p-3 rounded-xl glass-card border border-neon-red/50 text-neon-red text-sm"
                  >
                    {error}
                  </motion.div>
                )}
                {success && (
                  <motion.div
                    initial={{ opacity: 0, y: -10 }}
                    animate={{ opacity: 1, y: 0 }}
                    className="mb-4 p-3 rounded-xl glass-card border border-neon-green/50 text-neon-green text-sm"
                  >
                    {success}
                  </motion.div>
                )}

                {/* Forms */}
                {mode === "login" ? (
                  <form onSubmit={handleLogin} className="space-y-4">
                    <div>
                      <label className="block text-sm font-medium text-fg-secondary mb-2">
                        Tên đăng nhập
                      </label>

                      <div className="login-input-wrapper">
                        <div className="login-input-icon">
                          <UserIcon className="w-5 h-5 text-neon-cyan/50" />
                        </div>

                        <input
                          type="text"
                          value={loginUsername}
                          onChange={(e) => setLoginUsername(e.target.value)}
                          className="login-input-field"
                          placeholder="Nhập tên đăng nhập"
                          autoFocus
                        />
                      </div>
                    </div>

                    <div>
                      <label className="block text-sm font-medium text-fg-secondary mb-2">
                        Mật khẩu
                      </label>

                      <div className="login-input-wrapper">
                        {/* phần bên trái chứa icon */}
                        <div className="login-input-icon">
                          <Lock className="w-5 h-5 text-neon-cyan/50" />
                        </div>

                        {/* phần bên phải là input */}
                        <input
                          type="password"
                          value={loginPassword}
                          onChange={(e) => setLoginPassword(e.target.value)}
                          className="login-input-field"
                          placeholder="Nhập mật khẩu"
                        />
                      </div>
                    </div>

                    <motion.button
                      type="submit"
                      disabled={loading}
                      whileHover={{ scale: 1.02 }}
                      whileTap={{ scale: 0.98 }}
                      className="w-full btn-neon py-3 text-base font-semibold disabled:opacity-50 disabled:cursor-not-allowed"
                    >
                      {loading ? "Đang xử lý..." : "Đăng Nhập"}
                    </motion.button>

                    <div className="text-center">
                      <button
                        type="button"
                        onClick={() => setMode("register")}
                        className="text-sm text-neon-cyan hover:text-neon-cyan/80 transition-colors"
                      >
                        Chưa có tài khoản? Đăng ký ngay
                      </button>
                    </div>
                  </form>
                ) : (
                  <form onSubmit={handleRegister} className="space-y-4">
                    {/* TÊN ĐĂNG KÝ */}
<div>
  <label className="block text-sm font-medium text-fg-secondary mb-2">
    Tên đăng nhập
  </label>
  <div className="login-input-wrapper">
    <div className="login-input-icon">
      <UserIcon className="w-5 h-5 text-neon-cyan/50" />
    </div>
    <input
      type="text"
      value={registerUsername}
      onChange={(e) => setRegisterUsername(e.target.value)}
      className="login-input-field"
      placeholder="Nhập tên đăng nhập"
      autoFocus
    />
  </div>
</div>

{/* EMAIL */}
<div>
  <label className="block text-sm font-medium text-fg-secondary mb-2">
    Email
  </label>
  <div className="login-input-wrapper">
    <div className="login-input-icon">
      <Mail className="w-5 h-5 text-neon-cyan/50" />
    </div>
    <input
      type="email"
      value={registerEmail}
      onChange={(e) => setRegisterEmail(e.target.value)}
      className="login-input-field"
      placeholder="Nhập email"
    />
  </div>
</div>

{/* MẬT KHẨU */}
<div>
  <label className="block text-sm font-medium text-fg-secondary mb-2">
    Mật khẩu
  </label>
  <div className="login-input-wrapper">
    <div className="login-input-icon">
      <Lock className="w-5 h-5 text-neon-cyan/50" />
    </div>
    <input
      type="password"
      value={registerPassword}
      onChange={(e) => setRegisterPassword(e.target.value)}
      className="login-input-field"
      placeholder="Nhập mật khẩu (tối thiểu 6 ký tự)"
    />
  </div>
</div>

{/* XÁC NHẬN MẬT KHẨU */}
<div>
  <label className="block text-sm font-medium text-fg-secondary mb-2">
    Xác nhận mật khẩu
  </label>
  <div className="login-input-wrapper">
    <div className="login-input-icon">
      <Lock className="w-5 h-5 text-neon-cyan/50" />
    </div>
    <input
      type="password"
      value={registerConfirmPassword}
      onChange={(e) => setRegisterConfirmPassword(e.target.value)}
      className="login-input-field"
      placeholder="Nhập lại mật khẩu"
    />
  </div>
</div>

                    <motion.button
                      type="submit"
                      disabled={loading}
                      whileHover={{ scale: 1.02 }}
                      whileTap={{ scale: 0.98 }}
                      className="w-full btn-neon py-3 text-base font-semibold disabled:opacity-50 disabled:cursor-not-allowed"
                    >
                      {loading ? "Đang xử lý..." : "Đăng Ký"}
                    </motion.button>

                    <div className="text-center">
                      <button
                        type="button"
                        onClick={() => setMode("login")}
                        className="text-sm text-neon-cyan hover:text-neon-cyan/80 transition-colors"
                      >
                        Đã có tài khoản? Đăng nhập
                      </button>
                    </div>
                  </form>
                )}
              </div>
            </motion.div>
          </motion.div>
        </>
      )}
    </AnimatePresence>
  );
}

