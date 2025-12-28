"use client";

import { useState, useEffect } from "react";
import { motion, AnimatePresence } from "framer-motion";
import { useAuth } from "@/contexts/auth-context";
import { X, UserPlus } from "lucide-react";
import { ShieldCheck } from "lucide-react";

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
}

export function AuthModal({ isOpen, onClose }: AuthModalProps) {
  const { isAuthenticated, login, register } = useAuth();
  const [mode, setMode] = useState<"login" | "register">("login");
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");

  const [username, setUsername] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [confirm, setConfirm] = useState("");

  useEffect(() => {
    if (isAuthenticated) onClose();
  }, [isAuthenticated, onClose]);

  if (!isOpen) return null;

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setLoading(true);

    if (mode === "login") {
      const res = await login(username, password);
      if (!res.success) setError(res.message);
    } else {
      if (password !== confirm) {
        setError("Mật khẩu xác nhận không khớp");
        setLoading(false);
        return;
      }
      const res = await register(username, email, password);
      if (!res.success) setError(res.message);
    }

    setLoading(false);
  };

  return (
    <AnimatePresence>
      <motion.div
        className="fixed inset-0 z-50 flex items-center justify-center"
        style={{ background: "#000000" }}
      >
        <motion.div
          className="relative w-[90vw] h-[85vh] grid grid-cols-2 rounded-3xl overflow-hidden glass-card"
          initial={{ scale: 0.95, opacity: 0 }}
          animate={{ scale: 1, opacity: 1 }}
        >
          {/* LEFT - IMAGE */}
          <div
            className="hidden md:block bg-cover bg-center"
            style={{
              backgroundImage:
                "url(https://images.pexels.com/photos/35364379/pexels-photo-35364379.jpeg)",
            }}
          >
            
          </div>

          {/* RIGHT - FORM */}
          <div className="relative flex items-center justify-center p-12">
            <button
              onClick={onClose}
              className="absolute top-6 right-6 w-9 h-9 rounded-full glass-card flex items-center justify-center"
            >
              <X />
            </button>

            <form
              onSubmit={handleSubmit}
              className="w-full max-w-md"
            ><div className="w-full h-full  flex flex-col items-center justify-center text-center px-10">
              <h1 className="text-4xl font-bold text-neon-cyan mb-3">
                ADAS SYSTEM
              </h1>
              <p className="text-fg-secondary">
                Advanced Driver Assistance System
              </p>
            </div>
              {/* TITLE */}
              <div className="text-center">
                <div className="w-14 h-14 mx-auto mb-4 rounded-full bg-gradient-to-br from-primary to-accent flex items-center justify-center">
                  {mode === "login" ? (
                            <ShieldCheck className="w-7 h-7 text-white" />
                          ) : (
                            <UserPlus className="w-7 h-7 text-white" />
                          )}

                </div>
                <h2 className="text-2xl font-bold text-neon-cyan">
                  {mode === "login" ? "Đăng Nhập" : "Đăng Ký"}
                </h2>
              </div>

              {/* ERROR */}
              {error && (
                <div className="mt-6 px-6 py-3 rounded-full border border-red-500 text-red-400 text-sm text-center">
                  {error}
                </div>
              )}

              {/* INPUTS */}
              <div className="mt-8 space-y-6">
                <input
                  placeholder="Tên đăng nhập"
                  className="w-full h-12 px-6 rounded-full bg-white text-black focus:outline-none"
                  value={username}
                  onChange={(e) => setUsername(e.target.value)}
                />

                {mode === "register" && (
                  <input
                    placeholder="Email"
                    className="w-full h-12 px-6 rounded-full bg-white text-black"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                  />
                )}

                <input
                  type="password"
                  placeholder="Mật khẩu"
                  className="w-full h-12 px-6 rounded-full bg-white text-black"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                />

                {mode === "register" && (
                  <input
                    type="password"
                    placeholder="Xác nhận mật khẩu"
                    className="w-full h-12 px-6 rounded-full bg-white text-black"
                    value={confirm}
                    onChange={(e) => setConfirm(e.target.value)}
                  />
                )}
              </div>

              {/* BUTTON */}
              <button className="w-full mt-8 btn-neon py-3 rounded-full">
                {loading
                  ? "Đang xử lý..."
                  : mode === "login"
                  ? "ĐĂNG NHẬP"
                  : "ĐĂNG KÝ"}
              </button>

              {/* SWITCH */}
              <div className="mt-6 text-center">
                <button
                  type="button"
                  onClick={() =>
                    setMode(mode === "login" ? "register" : "login")
                  }
                  className="text-neon-cyan text-sm"
                >
                  {mode === "login"
                    ? "Chưa có tài khoản? Đăng ký"
                    : "Đã có tài khoản? Đăng nhập"}
                  <div className="my-6 flex items-center gap-3">
  <div className="flex-1 h-px bg-gray-600" />
  <span className="text-xs text-gray-400 whitespace-nowrap">HOẶC</span>
  <div className="flex-1 h-px bg-gray-600" />
</div>

<div className="grid grid-cols-3 gap-3">
  {/* Google */}
  <button
    type="button"
    className="flex items-center justify-center gap-2 py-3 rounded-xl border border-gray-600 hover:border-cyan-400 transition hover:bg-white/5"
  >
    <img
      src="https://www.svgrepo.com/show/475656/google-color.svg
"
      className="w-5 h-5"
    />
  </button>

  {/* Facebook */}
  <button
    type="button"
    className="flex items-center justify-center gap-2 py-3 rounded-xl border border-gray-600 hover:border-cyan-400 transition hover:bg-white/5"
  >
    <img
      src="https://www.svgrepo.com/show/475647/facebook-color.svg"
      className="w-5 h-5"
    />
  </button>

 {/* Apple */}
<button
  type="button"
  className="flex items-center justify-center py-3 rounded-xl border border-gray-600 hover:border-cyan-400 transition hover:bg-white/5"
>
  <img
    src="https://upload.wikimedia.org/wikipedia/commons/f/fa/Apple_logo_black.svg"
    alt="Apple"
    className="w-5 h-5 invert"
  />
</button>
</div>

                </button>
              </div>
            </form>
          </div>
        </motion.div>
      </motion.div>
    </AnimatePresence>
  );
}
