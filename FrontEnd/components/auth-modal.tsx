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

  useEffect(() => {
  setError("");
}, [mode]);

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
          className="relative w-[95vw] h-[85vh] grid grid-cols-2 rounded-3xl overflow-hidden glass-card"
          initial={{ scale: 0.95, opacity: 0 }}
          animate={{ scale: 1, opacity: 1 }}
        >
          {/* LEFT - IMAGE */}
          <div className="hidden md:block relative overflow-hidden">
  <AnimatePresence mode="wait">
    <motion.div
      key={mode} // quan trọng: đổi mode là đổi ảnh
      className="absolute inset-0 bg-cover bg-center"
      style={{
        backgroundImage:
          mode === "login"
            ? "url(/AnhDangNhap.jpg)"
            : "url(/AnhDangKy.jpg)",
      }}
      initial={{ x: 80, opacity: 0 }}
animate={{ x: 0, opacity: 1 }}
exit={{ x: -80, opacity: 0 }}
transition={{ duration: 0.6, ease: "easeOut" }}


    />
  </AnimatePresence>
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
            ><div className="w-full flex flex-col items-center text-center px-6 mb-4">

              <h1 className="text-4xl font-bold text-neon-cyan">
                ADAS SYSTEM
              </h1>
              
            </div>
              {/* TITLE */}
              <div className="text-center">
                
                <h2 className="text-xl font-bold text-neon-cyan">
                  {mode === "login" ? "Đăng Nhập" : "Đăng Ký"}
                </h2>
              </div>

              {/* ERROR */}
              {error && (
                <div className="mt-2 rounded-full border border-red-500 text-red-400 text-sm text-center">
                  {error}
                </div>
              )}

              {/* INPUTS */}
              <div className="mt-4 space-y-6">
                <input
                  placeholder="Tên đăng nhập"
                  className="w-full h-8 px-6 rounded-full bg-white text-black focus:outline-none"
                  value={username}
                  onChange={(e) => setUsername(e.target.value)}
                />

                {mode === "register" && (
                  <input
                    placeholder="Email"
                    className="w-full h-7 px-6 rounded-full bg-white text-black"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                  />
                )}

                <input
                  type="password"
                  placeholder="Mật khẩu"
                  className="w-full h-8 px-6 rounded-full bg-white text-black"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                />

                {mode === "register" && (
                  <input
                    type="password"
                    placeholder="Xác nhận mật khẩu"
                    className="w-full h-8 px-6 rounded-full bg-white text-black"
                    value={confirm}
                    onChange={(e) => setConfirm(e.target.value)}
                  />
                )}
              </div>

              {/* BUTTON */}
              <button className="w-full mt-5 btn-neon py-1 rounded-full">
                {loading
                  ? "Đang xử lý..."
                  : mode === "login"
                  ? "ĐĂNG NHẬP"
                  : "ĐĂNG KÝ"}
              </button>

              {/* SWITCH MODE */}
                  <div className="mt-5 text-center">
                    <button
                    type="button"
                    onClick={() =>
                      
                      setMode(mode === "login" ? "register" : "login")
                      
                    }
                    className="text-sm"
                  >
                    {mode === "login" ? (
                      <>
                        <span>Chưa có tài khoản? </span>
                        <span
                          style={{
                            fontWeight: 600,
                            background: "linear-gradient(90deg, #00ffff, #00ff99, #00ffff)",
                            backgroundSize: "200% auto",
                            color: "transparent",
                            WebkitBackgroundClip: "text",
                            backgroundClip: "text",
                            animation: "ledMove 2s linear infinite",
                          }}
                        >
                          Đăng ký
                        </span>
                      </>
                    ) : (
                      <>
                        <span>Đã có tài khoản? </span>
                        <span
                          style={{
                            fontWeight: 600,
                            background: "linear-gradient(90deg, #00ffff, #00ff99, #00ffff)",
                            backgroundSize: "200% auto",
                            color: "transparent",
                            WebkitBackgroundClip: "text",
                            backgroundClip: "text",
                            animation: "ledMove 2s linear infinite",
                          }}
                        >
                          Đăng nhập
                        </span>
                      </>
                    )}
                  </button>



</div>

{/* DIVIDER */}
<div className="my-3 flex items-center gap-3">
  <div className="flex-1 h-px bg-gray-600" />
  <span className="text-xs text-gray-400 whitespace-nowrap">HOẶC</span>
  <div className="flex-1 h-px bg-gray-600" />
</div>

{/* SOCIAL LOGIN */}
<div className="grid grid-cols-3 gap-3">
  <button
    type="button"
    className="flex items-center justify-center py-3 rounded-xl border border-gray-600 hover:border-cyan-400 transition hover:bg-white/5"
  >
    <img
      src="https://www.svgrepo.com/show/475656/google-color.svg"
      className="w-5 h-5"
      alt="Google"
    />
  </button>

  <button
    type="button"
    className="flex items-center justify-center py-3 rounded-xl border border-gray-600 hover:border-cyan-400 transition hover:bg-white/5"
  >
    <img
      src="https://www.svgrepo.com/show/475647/facebook-color.svg"
      className="w-5 h-5"
      alt="Facebook"
    />
  </button>

  <button
    type="button"
    className="flex items-center justify-center py-3 rounded-xl border border-gray-600 hover:border-cyan-400 transition hover:bg-white/5"
  >
    <img
      src="https://upload.wikimedia.org/wikipedia/commons/f/fa/Apple_logo_black.svg"
      className="w-5 h-5 invert"
      alt="Apple"
    />
  </button>
</div>



            </form>
          </div>
        </motion.div>
      </motion.div>
    </AnimatePresence>
  );
}

