"use client";

import { useState, useEffect } from "react";
import { motion, AnimatePresence } from "framer-motion";
import { useAuth } from "@/contexts/auth-context";
import { useRouter } from "next/navigation";
import Link from "next/link";
import { AnimatedPasswordInput } from "@/components/animated-password-input";
import { Sparkles, ArrowRight, Shield } from "lucide-react";

export default function RegisterPage() {
    const { isAuthenticated, register } = useAuth();
    const router = useRouter();
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState("");
    const [successMessage, setSuccessMessage] = useState("");
    const [mounted, setMounted] = useState(false);

    const [email, setEmail] = useState("");
    const [password, setPassword] = useState("");
    const [confirm, setConfirm] = useState("");

    // Mount check for client-side only rendering
    useEffect(() => {
        setMounted(true);
    }, []);

    // Redirect if already authenticated
    useEffect(() => {
        if (isAuthenticated) {
            router.push("/dashboard");
        }
    }, [isAuthenticated, router]);

    const handleSubmit = async (e: React.FormEvent) => {
        e.preventDefault();
        setError("");
        setSuccessMessage("");

        if (password !== confirm) {
            setError("Mật khẩu không khớp!");
            return;
        }

        setLoading(true);

        const res = await register(email, password);
        if (!res.success) {
            setError(res.message);
        } else {
            setSuccessMessage("Đăng ký thành công! Đang chuyển hướng...");
            setTimeout(() => {
                router.push("/dashboard");
            }, 1000);
            return;
        }

        setLoading(false);
    };

    return (
        <div className="min-h-screen w-full overflow-y-auto text-white">

            {/* Animated Gradient Background */}
            <div className="absolute inset-0 bg-linear-to-br from-slate-950 via-blue-950 to-slate-900">
                {/* Animated gradient orbs */}
                <motion.div
                    animate={{
                        scale: [1, 1.2, 1],
                        opacity: [0.3, 0.5, 0.3],
                    }}
                    transition={{
                        duration: 8,
                        repeat: Infinity,
                        ease: "easeInOut",
                    }}
                    className="absolute top-0 right-0 w-96 h-96 bg-blue-500/30 rounded-full blur-3xl"
                />
                <motion.div
                    animate={{
                        scale: [1, 1.3, 1],
                        opacity: [0.2, 0.4, 0.2],
                    }}
                    transition={{
                        duration: 10,
                        repeat: Infinity,
                        ease: "easeInOut",
                        delay: 1,
                    }}
                    className="absolute bottom-0 left-0 w-96 h-96 bg-cyan-500/20 rounded-full blur-3xl"
                />
                <motion.div
                    animate={{
                        scale: [1, 1.1, 1],
                        opacity: [0.2, 0.3, 0.2],
                    }}
                    transition={{
                        duration: 12,
                        repeat: Infinity,
                        ease: "easeInOut",
                        delay: 2,
                    }}
                    className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-96 h-96 bg-blue-500/20 rounded-full blur-3xl"
                />
            </div>

            {/* Floating particles - Client-side only */}
            {mounted && [...Array(20)].map((_, i) => {
                const startX = Math.random() * window.innerWidth;
                const startY = Math.random() * window.innerHeight;
                const endX = Math.random() * window.innerWidth;
                const endY = Math.random() * window.innerHeight;
                const duration = Math.random() * 10 + 10;

                return (
                    <motion.div
                        key={i}
                        className="absolute w-1 h-1 bg-cyan-400 rounded-full"
                        initial={{ x: startX, y: startY }}
                        animate={{ y: endY, x: endX }}
                        transition={{
                            duration,
                            repeat: Infinity,
                            ease: "linear",
                        }}
                    />
                );
            })}

            {/* Main Content */}
            <div className="relative z-10 flex items-center justify-center min-h-screen p-4 sm:p-6 lg:p-8">
                <motion.div
                    initial={{ scale: 0.9, opacity: 0 }}
                    animate={{ scale: 1, opacity: 1 }}
                    transition={{ duration: 0.5 }}
                    className="w-full max-w-6xl grid grid-cols-1 lg:grid-cols-2 gap-4 sm:gap-6 lg:gap-8"
                >
                    {/* LEFT - Image Card */}
                    <motion.div
                        initial={{ x: -50, opacity: 0 }}
                        animate={{ x: 0, opacity: 1 }}
                        transition={{ duration: 0.6, delay: 0.2 }}
                        className="hidden lg:block relative rounded-2xl lg:rounded-3xl overflow-hidden min-h-[600px] border border-white/10"

                    >
                        {/* Image - Using background for better compatibility */}
                        <div
                            className="absolute inset-0 bg-cover bg-center"
                            style={{ backgroundImage: 'url(/AnhDangKy.jpg)' }}
                        >
                            {/* Subtle overlay gradient for text readability */}
                            <div className="absolute inset-0 bg-linear-to-t from-black/70 via-black/20 to-transparent" />
                        </div>

                        {/* Content overlay */}
                        <div className="relative h-full flex flex-col justify-end p-6 sm:p-8 lg:p-12 z-20">
                            <motion.div
                                initial={{ y: 20, opacity: 0 }}
                                animate={{ y: 0, opacity: 1 }}
                                transition={{ delay: 0.8 }}
                                className="space-y-2 sm:space-y-4"
                            >
                                <div className="inline-flex items-center gap-2 px-3 sm:px-4 py-1.5 sm:py-2 rounded-full bg-cyan-500/20 border-cyan-500/30 backdrop-blur-sm">
                                    <Shield className="w-3 sm:w-4 h-3 sm:h-4 text-cyan-400" />
                                    <span className="text-xs sm:text-sm text-cyan-300 font-medium">Secure & Trusted</span>
                                </div>
                                <h3 className="text-2xl sm:text-3xl lg:text-4xl font-bold !text-white">
                                    Tham gia ADAS
                                </h3>
                                <p className="text-white/80 text-sm sm:text-base lg:text-lg">
                                    Trải nghiệm công nghệ hỗ trợ lái xe tiên tiến
                                </p>
                            </motion.div>
                        </div>
                    </motion.div>

                    {/* RIGHT - Form Card */}
                    <motion.div
                        initial={{ x: 50, opacity: 0 }}
                        animate={{ x: 0, opacity: 1 }}
                        transition={{ duration: 0.6, delay: 0.2 }}
                        className="relative"
                    >
                        {/* Glass card */}
                        <div className="relative rounded-3xl bg-white/5 backdrop-blur-2xl border border-white/10 p-8 md:p-12 shadow-2xl">
                            {/* Close button */}
                            <Link
                                href="/overview"
                                className="absolute top-6 right-6 w-10 h-10 rounded-full bg-white/5 backdrop-blur-sm border border-white/10 flex items-center justify-center hover:bg-white/10 transition-all duration-300 group"
                            >
                                <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" className="text-white/60 group-hover:text-white transition-colors">
                                    <line x1="18" y1="6" x2="6" y2="18"></line>
                                    <line x1="6" y1="6" x2="18" y2="18"></line>
                                </svg>
                            </Link>

                            <form onSubmit={handleSubmit} className="space-y-6">
                                {/* Header */}
                                <motion.div
                                    initial={{ y: 20, opacity: 0 }}
                                    animate={{ y: 0, opacity: 1 }}
                                    transition={{ delay: 0.4 }}
                                    className="text-center space-y-2"
                                >
                                    <div className="inline-flex items-center gap-2 px-4 py-2 rounded-full bg-cyan-500/10 border-cyan-500/20 mb-4">
                                        <Sparkles className="w-4 h-4 text-cyan-400" />
                                        <span className="text-sm text-cyan-400 font-medium">ADAS Platform</span>
                                    </div>
                                    <h1 className="text-4xl font-bold !text-white">
                                            Đăng Ký
                                        </h1>
                                    <p className="text-white/50">Tạo tài khoản mới</p>
                                </motion.div>

                                {/* Error & Success Messages */}
                                <AnimatePresence>
                                    {error && (
                                        <motion.div
                                            initial={{ opacity: 0, y: -10 }}
                                            animate={{ opacity: 1, y: 0 }}
                                            exit={{ opacity: 0, y: -10 }}
                                            className="rounded-xl bg-red-500/10 border border-red-500/20 text-red-400 text-sm text-center py-3 px-4 backdrop-blur-sm"
                                        >
                                            {error}
                                        </motion.div>
                                    )}
                                    {successMessage && (
                                        <motion.div
                                            initial={{ opacity: 0, y: -10 }}
                                            animate={{ opacity: 1, y: 0 }}
                                            exit={{ opacity: 0, y: -10 }}
                                            className="rounded-xl bg-green-500/10 border border-green-500/20 text-green-400 text-sm text-center py-3 px-4 backdrop-blur-sm"
                                        >
                                            {successMessage}
                                        </motion.div>
                                    )}
                                </AnimatePresence>

                                {/* Inputs */}
                                <motion.div
                                    initial={{ y: 20, opacity: 0 }}
                                    animate={{ y: 0, opacity: 1 }}
                                    transition={{ delay: 0.5 }}
                                >
                                    <AnimatedPasswordInput
                                        mode="register"
                                        email={email}
                                        password={password}
                                        confirmPassword={confirm}
                                        onEmailChange={setEmail}
                                        onPasswordChange={setPassword}
                                        onConfirmPasswordChange={setConfirm}
                                    />
                                </motion.div>

                                {/* Submit Button */}
                                <motion.button
                                    initial={{ y: 20, opacity: 0 }}
                                    animate={{ y: 0, opacity: 1 }}
                                    transition={{ delay: 0.6 }}
                                    whileHover={{ scale: 1.02 }}
                                    whileTap={{ scale: 0.98 }}
                                    type="submit"
                                    disabled={loading}
                                    className="w-full h-14 rounded-xl bg-linear-to-r from-cyan-500 to-blue-500 
hover:from-cyan-400 hover:to-blue-400
shadow-cyan-500/25
hover:shadow-cyan-500/40 transition-all duration-300 flex items-center justify-center gap-2 group disabled:opacity-50 disabled:cursor-not-allowed"
                                >
                                    {loading ? (
                                        <div className="w-5 h-5 border-2 border-white/30 border-t-white rounded-full animate-spin" />
                                    ) : (
                                        <>
                                            <span>Tạo Tài Khoản</span>
                                            <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
                                        </>
                                    )}
                                </motion.button>

                                {/* Divider */}
                                <motion.div
                                    initial={{ opacity: 0 }}
                                    animate={{ opacity: 1 }}
                                    transition={{ delay: 0.7 }}
                                    className="flex items-center gap-4"
                                >
                                    <div className="flex-1 h-px bg-linear-to-r from-transparent via-white/20 to-transparent" />
                                    <span className="text-xs text-white/40 font-medium">HOẶC</span>
                                    <div className="flex-1 h-px bg-linear-to-r from-transparent via-white/20 to-transparent" />
                                </motion.div>

                                {/* Social Login */}
                                <motion.div
                                    initial={{ y: 20, opacity: 0 }}
                                    animate={{ y: 0, opacity: 1 }}
                                    transition={{ delay: 0.8 }}
                                    className="grid grid-cols-3 gap-3"
                                >
                                    {[
                                        { icon: "https://www.svgrepo.com/show/475656/google-color.svg", name: "Google" },
                                        { icon: "https://www.svgrepo.com/show/475647/facebook-color.svg", name: "Facebook" },
                                        { icon: "https://upload.wikimedia.org/wikipedia/commons/f/fa/Apple_logo_black.svg", name: "Apple", invert: true },
                                    ].map((social) => (
                                        <motion.button
                                            key={social.name}
                                            whileHover={{ scale: 1.05, y: -2 }}
                                            whileTap={{ scale: 0.95 }}
                                            type="button"
                                            className="h-12 rounded-xl bg-white/5 backdrop-blur-sm border border-white/10 hover:bg-white/10 hover:border-white/20 transition-all duration-300 flex items-center justify-center group"
                                        >
                                            <img
                                                src={social.icon}
                                                className={`w-5 h-5 ${social.invert ? 'invert' : ''} group-hover:scale-110 transition-transform`}
                                                alt={social.name}
                                            />
                                        </motion.button>
                                    ))}
                                </motion.div>

                                {/* Login Link */}
                                <motion.div
                                    initial={{ opacity: 0 }}
                                    animate={{ opacity: 1 }}
                                    transition={{ delay: 0.9 }}
                                    className="text-center"
                                >
                                    <span className="text-white">Đã có tài khoản? </span>
                                    <Link
                                        href="/login"
                                        className="text-cyan-400 hover:text-cyan-300 font-medium transition-colors"
                                    >
                                        Đăng nhập
                                    </Link>
                                </motion.div>
                            </form>
                        </div>
                    </motion.div>
                </motion.div>
            </div>
        </div>
    );
}
