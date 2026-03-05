"use client";

import { useState } from "react";
import { motion, AnimatePresence } from "framer-motion";
import { Eye, EyeOff, Mail, Lock, CheckCircle2, XCircle } from "lucide-react";

interface AnimatedPasswordInputProps {
    email: string;
    password: string;
    confirmPassword?: string;
    onEmailChange: (value: string) => void;
    onPasswordChange: (value: string) => void;
    onConfirmPasswordChange?: (value: string) => void;
    mode: "login" | "register";
}

export function AnimatedPasswordInput({
    email,
    password,
    confirmPassword,
    onEmailChange,
    onPasswordChange,
    onConfirmPasswordChange,
    mode,
}: AnimatedPasswordInputProps) {
    const [showPassword, setShowPassword] = useState(false);
    const [showConfirmPassword, setShowConfirmPassword] = useState(false);
    const [emailFocused, setEmailFocused] = useState(false);
    const [passwordFocused, setPasswordFocused] = useState(false);
    const [confirmPasswordFocused, setConfirmPasswordFocused] = useState(false);

    // Email validation
    const isEmailValid = /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
    const showEmailValidation = email.length > 0;

    // Password strength calculation
    const getPasswordStrength = () => {
        let strength = 0;
        if (password.length >= 8) strength++;
        if (password.length >= 12) strength++;
        if (/[A-Z]/.test(password)) strength++;
        if (/[a-z]/.test(password)) strength++;
        if (/[0-9]/.test(password)) strength++;
        if (/[^A-Za-z0-9]/.test(password)) strength++;
        return strength;
    };

    const passwordStrength = getPasswordStrength();
    const getStrengthColor = () => {
        if (passwordStrength <= 2) return "from-red-500 to-red-600";
        if (passwordStrength <= 4) return "from-yellow-500 to-orange-500";
        return "from-green-500 to-emerald-500";
    };

    const getStrengthText = () => {
        if (passwordStrength <= 2) return "Weak";
        if (passwordStrength <= 4) return "Medium";
        return "Strong";
    };

    // Password match validation
    const passwordsMatch = confirmPassword && password === confirmPassword;
    const showPasswordMatch = confirmPassword && confirmPassword.length > 0;

    return (
        <div className="space-y-6">
            {/* Email Input with Framer Motion */}
            <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.4 }}
                className="relative"
            >
                {/* Glow effect on focus */}
                <AnimatePresence>
                    {emailFocused && (
                        <motion.div
                            initial={{ opacity: 0, scale: 0.95 }}
                            animate={{ opacity: 1, scale: 1 }}
                            exit={{ opacity: 0, scale: 0.95 }}
                            className="absolute -inset-1 bg-linear-to-r from-cyan-500/30 via-blue-500/30 to-purple-500/30 rounded-2xl blur-xl"
                        />
                    )}
                </AnimatePresence>

                <div className="relative">
                    {/* Animated Icon */}
                    <motion.div
                        animate={{
                            x: emailFocused ? -5 : 0,
                            scale: emailFocused ? 1.1 : 1,
                        }}
                        transition={{ duration: 0.3 }}
                        className="absolute left-5 top-1/2 -translate-y-1/2 pointer-events-none z-10"
                    >
                        <Mail className={`w-5 h-5 transition-colors duration-300 ${emailFocused ? 'text-cyan-400' : 'text-white/40'}`} />
                    </motion.div>

                    {/* Input */}
                    <input
                        type="email"
                        placeholder="Email address"
                        className="w-full h-12 sm:h-14 px-12 sm:px-14 rounded-xl bg-white/10 backdrop-blur-xl border border-white/20 text-white text-sm sm:text-base text-center placeholder-white/40 placeholder:text-center focus:outline-none focus:border-cyan-400/60 focus:bg-white/15 transition-all duration-300"
                        value={email}
                        onChange={(e) => onEmailChange(e.target.value)}
                        onFocus={() => setEmailFocused(true)}
                        onBlur={() => setEmailFocused(false)}
                        required
                    />

                    {/* Validation Icon */}
                    <AnimatePresence>
                        {showEmailValidation && (
                            <motion.div
                                initial={{ opacity: 0, scale: 0 }}
                                animate={{ opacity: 1, scale: 1 }}
                                exit={{ opacity: 0, scale: 0 }}
                                className="absolute right-4 top-1/2 -translate-y-1/2"
                            >
                                {isEmailValid ? (
                                    <CheckCircle2 className="w-5 h-5 text-green-400" />
                                ) : (
                                    <XCircle className="w-5 h-5 text-red-400" />
                                )}
                            </motion.div>
                        )}
                    </AnimatePresence>
                </div>

                {/* Floating Label */}
                <AnimatePresence>
                    {email && (
                        <motion.span
                            initial={{ opacity: 0, y: 10 }}
                            animate={{ opacity: 1, y: 0 }}
                            exit={{ opacity: 0, y: 10 }}
                            className="absolute -top-2.5 left-10 px-2 text-xs font-medium text-cyan-400 bg-black/80 backdrop-blur-sm rounded-full"
                        >
                            Email
                        </motion.span>
                    )}
                </AnimatePresence>
            </motion.div>

            {/* Password Input with Framer Motion */}
            <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.4, delay: 0.1 }}
                className="relative"
            >
                {/* Glow effect on focus */}
                <AnimatePresence>
                    {passwordFocused && (
                        <motion.div
                            initial={{ opacity: 0, scale: 0.95 }}
                            animate={{ opacity: 1, scale: 1 }}
                            exit={{ opacity: 0, scale: 0.95 }}
                            className="absolute -inset-1 bg-linear-to-r from-cyan-500/30 via-blue-500/30 to-purple-500/30 rounded-2xl blur-xl"
                        />
                    )}
                </AnimatePresence>

                <div className="relative">
                    {/* Animated Icon */}
                    <motion.div
                        animate={{
                            x: passwordFocused ? -5 : 0,
                            scale: passwordFocused ? 1.1 : 1,
                            rotate: passwordFocused ? -5 : 0,
                        }}
                        transition={{ duration: 0.3 }}
                        className="absolute left-5 top-1/2 -translate-y-1/2 pointer-events-none z-10"
                    >
                        <Lock className={`w-5 h-5 transition-colors duration-300 ${passwordFocused ? 'text-cyan-400' : 'text-white/40'}`} />
                    </motion.div>

                    {/* Input */}
                    <input
                        type={showPassword ? "text" : "password"}
                        placeholder="Password"
                        className="w-full h-12 sm:h-14 px-12 sm:px-14 rounded-xl bg-white/10 backdrop-blur-xl border border-white/20 text-white text-sm sm:text-base text-center placeholder-white/40 placeholder:text-center focus:outline-none focus:border-cyan-400/60 focus:bg-white/15 transition-all duration-300"
                        value={password}
                        onChange={(e) => onPasswordChange(e.target.value)}
                        onFocus={() => setPasswordFocused(true)}
                        onBlur={() => setPasswordFocused(false)}
                        required
                    />

                    {/* Toggle Password Visibility */}
                    <motion.button
                        type="button"
                        whileHover={{ scale: 1.1 }}
                        whileTap={{ scale: 0.95 }}
                        onClick={() => setShowPassword(!showPassword)}
                        className="absolute right-4 sm:right-5 top-1/2 -translate-y-1/2 p-1.5 rounded-lg text-white/60 hover:text-cyan-400 hover:bg-white/10 transition-all duration-200 z-10"
                    >
                        {showPassword ? (
                            <Eye className="w-4 h-4 sm:w-5 sm:h-5" />
                        ) : (
                            <EyeOff className="w-4 h-4 sm:w-5 sm:h-5" />
                        )}
                    </motion.button>
                </div>

                {/* Floating Label */}
                <AnimatePresence>
                    {password && (
                        <motion.span
                            initial={{ opacity: 0, y: 10 }}
                            animate={{ opacity: 1, y: 0 }}
                            exit={{ opacity: 0, y: 10 }}
                            className="absolute -top-2.5 left-10 px-2 text-xs font-medium text-cyan-400 bg-black/80 backdrop-blur-sm rounded-full"
                        >
                            Password
                        </motion.span>
                    )}
                </AnimatePresence>
            </motion.div>

            {/* Password Strength Indicator */}
            <AnimatePresence>
                {password && (
                    <motion.div
                        initial={{ opacity: 0, height: 0 }}
                        animate={{ opacity: 1, height: "auto" }}
                        exit={{ opacity: 0, height: 0 }}
                        className="space-y-2"
                    >
                        {/* Strength Bar */}
                        <div className="flex gap-1.5">
                            {[1, 2, 3, 4, 5, 6].map((level) => (
                                <motion.div
                                    key={level}
                                    initial={{ scaleX: 0 }}
                                    animate={{ scaleX: 1 }}
                                    transition={{ delay: level * 0.05 }}
                                    className="h-1.5 flex-1 rounded-full bg-white/10 overflow-hidden"
                                >
                                    <motion.div
                                        initial={{ width: 0 }}
                                        animate={{ width: passwordStrength >= level ? "100%" : "0%" }}
                                        transition={{ duration: 0.3 }}
                                        className={`h-full bg-linear-to-r ${getStrengthColor()}`}
                                    />
                                </motion.div>
                            ))}
                        </div>

                        {/* Strength Text */}
                        <motion.div
                            initial={{ opacity: 0 }}
                            animate={{ opacity: 1 }}
                            className="flex items-center justify-between text-xs"
                        >
                            <span className="text-white/50">Password strength:</span>
                            <span className={`font-medium ${passwordStrength <= 2 ? "text-red-400" :
                                passwordStrength <= 4 ? "text-yellow-400" :
                                    "text-green-400"
                                }`}>
                                {getStrengthText()}
                            </span>
                        </motion.div>
                    </motion.div>
                )}
            </AnimatePresence>

            {/* Confirm Password Input (Register mode only) */}
            {mode === "register" && onConfirmPasswordChange && (
                <motion.div
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.4, delay: 0.2 }}
                    className="relative"
                >
                    {/* Glow effect on focus */}
                    <AnimatePresence>
                        {confirmPasswordFocused && (
                            <motion.div
                                initial={{ opacity: 0, scale: 0.95 }}
                                animate={{ opacity: 1, scale: 1 }}
                                exit={{ opacity: 0, scale: 0.95 }}
                                className="absolute -inset-1 bg-linear-to-r from-cyan-500/30 via-blue-500/30 to-purple-500/30 rounded-2xl blur-xl"
                            />
                        )}
                    </AnimatePresence>

                    <div className="relative">
                        {/* Animated Icon */}
                        <motion.div
                            animate={{
                                x: confirmPasswordFocused ? -5 : 0,
                                scale: confirmPasswordFocused ? 1.1 : 1,
                                rotate: confirmPasswordFocused ? 5 : 0,
                            }}
                            transition={{ duration: 0.3 }}
                            className="absolute left-5 top-1/2 -translate-y-1/2 pointer-events-none z-10"
                        >
                            <Lock className={`w-5 h-5 transition-colors duration-300 ${confirmPasswordFocused ? 'text-cyan-400' : 'text-white/40'}`} />
                        </motion.div>

                        {/* Input */}
                        <input
                            type={showConfirmPassword ? "text" : "password"}
                            placeholder="Confirm password"
                            className="w-full h-12 sm:h-14 px-12 sm:px-14 rounded-xl bg-white/10 backdrop-blur-xl border border-white/20 text-white text-sm sm:text-base text-center placeholder-white/40 placeholder:text-center focus:outline-none focus:border-cyan-400/60 focus:bg-white/15 transition-all duration-300"
                            value={confirmPassword || ""}
                            onChange={(e) => onConfirmPasswordChange(e.target.value)}
                            onFocus={() => setConfirmPasswordFocused(true)}
                            onBlur={() => setConfirmPasswordFocused(false)}
                            required
                        />

                        {/* Toggle Password Visibility */}
                        <motion.button
                            type="button"
                            whileHover={{ scale: 1.1 }}
                            whileTap={{ scale: 0.95 }}
                            onClick={() => setShowConfirmPassword(!showConfirmPassword)}
                            className="absolute right-4 sm:right-5 top-1/2 -translate-y-1/2 p-1.5 rounded-lg text-white/60 hover:text-cyan-400 hover:bg-white/10 transition-all duration-200 z-10"
                        >
                            {showConfirmPassword ? (
                                <Eye className="w-4 h-4 sm:w-5 sm:h-5" />
                            ) : (
                                <EyeOff className="w-4 h-4 sm:w-5 sm:h-5" />
                            )}
                        </motion.button>

                        {/* Validation Icon */}
                        <AnimatePresence>
                            {showPasswordMatch && (
                                <motion.div
                                    initial={{ opacity: 0, scale: 0 }}
                                    animate={{ opacity: 1, scale: 1 }}
                                    exit={{ opacity: 0, scale: 0 }}
                                    className="absolute right-12 sm:right-14 top-1/2 -translate-y-1/2 z-10"
                                >
                                    {passwordsMatch ? (
                                        <CheckCircle2 className="w-4 h-4 sm:w-5 sm:h-5 text-green-400" />
                                    ) : (
                                        <XCircle className="w-4 h-4 sm:w-5 sm:h-5 text-red-400" />
                                    )}
                                </motion.div>
                            )}
                        </AnimatePresence>
                    </div>

                    {/* Floating Label */}
                    <AnimatePresence>
                        {confirmPassword && (
                            <motion.span
                                initial={{ opacity: 0, y: 10 }}
                                animate={{ opacity: 1, y: 0 }}
                                exit={{ opacity: 0, y: 10 }}
                                className="absolute -top-2.5 left-10 px-2 text-xs font-medium text-cyan-400 bg-black/80 backdrop-blur-sm rounded-full"
                            >
                                Confirm Password
                            </motion.span>
                        )}
                    </AnimatePresence>
                </motion.div>
            )}
        </div>
    );
}
