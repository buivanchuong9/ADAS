"use client";

import { useState, useEffect } from "react";
import { motion, AnimatePresence } from "framer-motion";
import { useAuth } from "@/contexts/auth-context";
import { useRouter } from "next/navigation";
import Link from "next/link";
import { AnimatedPasswordInput } from "@/components/animated-password-input";
import { Sparkles, ArrowRight } from "lucide-react";

export default function LoginPage() {
    const { isAuthenticated, login } = useAuth();
    const router = useRouter();
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState("");
    const [successMessage, setSuccessMessage] = useState("");
    const [mounted, setMounted] = useState(false);

    const [email, setEmail] = useState("");
    const [password, setPassword] = useState("");

    useEffect(() => {
        setMounted(true);
    }, []);

    useEffect(() => {
        if (isAuthenticated) {
            router.push("/dashboard");
        }
    }, [isAuthenticated, router]);

    const handleSubmit = async (e: React.FormEvent) => {
        e.preventDefault();
        setError("");
        setSuccessMessage("");
        setLoading(true);

        const res = await login(email, password);
        if (!res.success) {
            setError(res.message);
        } else {
            setSuccessMessage("Đăng nhập thành công!");
            setTimeout(() => {
                router.push("/dashboard");
            }, 500);
            return;
        }

        setLoading(false);
    };

    return (
        <div className="min-h-screen w-full overflow-y-auto text-white [&_*]:!text-white">


            {/* Background */}
            <div className="absolute inset-0 bg-gradient-to-br from-slate-950 via-blue-950 to-slate-900" />

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
                        transition={{ duration, repeat: Infinity, ease: "linear" }}
                    />
                );
            })}

            <div className="relative z-10 flex items-center justify-center min-h-screen p-4 sm:p-6 lg:p-8">
                <motion.div
                    initial={{ scale: 0.9, opacity: 0 }}
                    animate={{ scale: 1, opacity: 1 }}
                    transition={{ duration: 0.5 }}
                    className="w-full max-w-6xl grid grid-cols-1 lg:grid-cols-2 gap-6"
                >

                    {/* LEFT */}
                    <motion.div
                        initial={{ x: -50, opacity: 0 }}
                        animate={{ x: 0, opacity: 1 }}
                        transition={{ duration: 0.6, delay: 0.2 }}
                        className="hidden lg:block relative rounded-3xl overflow-hidden min-h-[600px] border border-white/20"

                    >
                        <div
                            className="absolute inset-0 bg-cover bg-center"
                            style={{ backgroundImage: 'url(/AnhDangNhap.jpg)' }}
                        >
                            <div className="absolute inset-0 bg-gradient-to-t from-black/90 via-black/60 to-black/20" />
                        </div>

                        <div className="relative h-full flex flex-col justify-end p-12 z-20">
                            <h3 className="text-4xl font-extrabold text-white drop-shadow-lg mb-4">
                                Welcome to ADAS
                            </h3>
                            <p className="text-lg font-semibold text-white drop-shadow-md">
                                Advanced Driver Assistance System
                            </p>
                        </div>
                    </motion.div>

                    {/* RIGHT */}
                    <motion.div
                        initial={{ x: 50, opacity: 0 }}
                        animate={{ x: 0, opacity: 1 }}
                        transition={{ duration: 0.6, delay: 0.2 }}
                    >
                        <div className="relative rounded-3xl bg-slate-900/80 backdrop-blur-xl border border-white/20 p-12 shadow-2xl text-white">

                            <Link
                                href="/overview"
                                className="absolute top-6 right-6 w-10 h-10 rounded-full bg-white/10 border border-white/20 flex items-center justify-center hover:bg-white/20 transition-all"
                            >
                                ✕
                            </Link>

                            <form onSubmit={handleSubmit} className="space-y-8">

                                {/* Header */}
                                <div className="text-center space-y-3">
                                    <div className="inline-flex items-center gap-2 px-4 py-2 rounded-full bg-cyan-600 border border-cyan-400">
                                        <Sparkles className="w-4 h-4 text-white" />
                                        <span className="text-sm font-bold text-white">
                                            ADAS Platform
                                        </span>
                                    </div>

                                    <h1 className="text-4xl font-extrabold text-white drop-shadow-lg">
                                        Đăng Nhập
                                    </h1>

                                    <p className="text-base font-semibold text-white">
                                        Chào mừng trở lại!
                                    </p>
                                </div>

                                {/* Messages */}
                                <AnimatePresence>
                                    {error && (
                                        <motion.div
                                            initial={{ opacity: 0, y: -10 }}
                                            animate={{ opacity: 1, y: 0 }}
                                            exit={{ opacity: 0, y: -10 }}
                                            className="rounded-xl bg-red-600 border border-red-400 text-white font-semibold text-center py-3 px-4"
                                        >
                                            {error}
                                        </motion.div>
                                    )}

                                    {successMessage && (
                                        <motion.div
                                            initial={{ opacity: 0, y: -10 }}
                                            animate={{ opacity: 1, y: 0 }}
                                            exit={{ opacity: 0, y: -10 }}
                                            className="rounded-xl bg-green-600 border border-green-400 text-white font-semibold text-center py-3 px-4"
                                        >
                                            {successMessage}
                                        </motion.div>
                                    )}
                                </AnimatePresence>

                                <AnimatedPasswordInput
                                    mode="login"
                                    email={email}
                                    password={password}
                                    onEmailChange={setEmail}
                                    onPasswordChange={setPassword}
                                />

                                <motion.button
                                    whileHover={{ scale: 1.02 }}
                                    whileTap={{ scale: 0.98 }}
                                    type="submit"
                                    disabled={loading}
                                    className="w-full h-14 rounded-xl bg-gradient-to-r from-cyan-500 to-blue-500 text-white font-bold shadow-lg transition-all duration-300 flex items-center justify-center gap-2 disabled:opacity-50"
                                >
                                    {loading ? (
                                        <div className="w-5 h-5 border-2 border-white border-t-transparent rounded-full animate-spin" />
                                    ) : (
                                        <>
                                            <span>Đăng Nhập</span>
                                            <ArrowRight className="w-5 h-5" />
                                        </>
                                    )}
                                </motion.button>

                                <div className="flex items-center gap-4">
                                    <div className="flex-1 h-px bg-white/30" />
                                    <span className="text-sm font-bold text-white">
                                        HOẶC
                                    </span>
                                    <div className="flex-1 h-px bg-white/30" />
                                </div>

                                <div className="text-center">
                                    <span className="font-semibold text-white">
                                        Chưa có tài khoản?{" "}
                                    </span>
                                    <Link
                                        href="/register"
                                        className="text-cyan-400 font-bold hover:text-cyan-300"
                                    >
                                        Đăng ký ngay
                                    </Link>
                                </div>

                            </form>
                        </div>
                    </motion.div>

                </motion.div>
            </div>
        </div>
    );
}
