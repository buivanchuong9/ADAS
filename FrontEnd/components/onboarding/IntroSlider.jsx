"use client";

import { useState } from "react";
import { motion, AnimatePresence } from "framer-motion";
import { introSlides } from "@/lib/introData";
import styles from "./IntroSlider.module.scss";

// Helper function to convert hex to RGB
const hexToRgb = (hex) => {
    const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
    return result
        ? `${parseInt(result[1], 16)}, ${parseInt(result[2], 16)}, ${parseInt(result[3], 16)}`
        : "0, 229, 255";
};

export default function IntroSlider({ onComplete }) {
    const [currentSlide, setCurrentSlide] = useState(0);
    const slide = introSlides[currentSlide];
    const Icon = slide.icon;

    const handleNext = () => {
        if (slide.isFinal) {
            onComplete();
        } else {
            setCurrentSlide((prev) => Math.min(prev + 1, introSlides.length - 1));
        }
    };

    const handleSkip = () => {
        onComplete();
    };

    const handleDotClick = (index) => {
        setCurrentSlide(index);
    };

    // Slide animation variants
    const slideVariants = {
        enter: (direction) => ({
            x: direction > 0 ? 1000 : -1000,
            opacity: 0,
        }),
        center: {
            x: 0,
            opacity: 1,
        },
        exit: (direction) => ({
            x: direction < 0 ? 1000 : -1000,
            opacity: 0,
        }),
    };

    // Stagger animation for features
    const containerVariants = {
        hidden: { opacity: 0 },
        visible: {
            opacity: 1,
            transition: {
                staggerChildren: 0.1,
                delayChildren: 0.7,
            },
        },
    };

    const featureVariants = {
        hidden: { opacity: 0, x: -30 },
        visible: {
            opacity: 1,
            x: 0,
            transition: {
                type: "spring",
                stiffness: 100,
            },
        },
    };

    return (
        <div
            className={styles.introSlider}
            style={{
                "--theme-color": slide.themeColor,
                "--theme-color-rgb": hexToRgb(slide.themeColor),
                "--gradient-from": slide.gradientFrom || slide.themeColor,
                "--gradient-to": slide.gradientTo || slide.themeColor,
            }}
        >
            {/* Animated Background Overlay */}
            <div className={styles.backgroundOverlay} />

            {/* Skip Button */}
            <button className={styles.skipButton} onClick={handleSkip}>
                Bỏ qua
            </button>

            {/* Slide Content */}
            <AnimatePresence mode="wait" custom={1}>
                <motion.div
                    key={currentSlide}
                    custom={1}
                    variants={slideVariants}
                    initial="enter"
                    animate="center"
                    exit="exit"
                    transition={{
                        x: { type: "spring", stiffness: 300, damping: 30 },
                        opacity: { duration: 0.3 },
                    }}
                    className={styles.slideContainer}
                >
                    <div className={styles.contentWrapper}>
                        {/* Icon Container */}
                        <motion.div
                            className={styles.iconContainer}
                            initial={{ scale: 0, rotate: -180 }}
                            animate={{ scale: 1, rotate: 0 }}
                            transition={{
                                type: "spring",
                                stiffness: 200,
                                damping: 15,
                                delay: 0.2,
                            }}
                        >
                            <Icon />
                        </motion.div>

                        {/* Title */}
                        <motion.h1
                            className={styles.title}
                            initial={{ opacity: 0, y: 30 }}
                            animate={{ opacity: 1, y: 0 }}
                            transition={{ delay: 0.4, duration: 0.6 }}
                        >
                            {slide.title}
                        </motion.h1>

                        {/* Subtitle */}
                        <motion.p
                            className={styles.subtitle}
                            initial={{ opacity: 0, y: 20 }}
                            animate={{ opacity: 1, y: 0 }}
                            transition={{ delay: 0.5, duration: 0.6 }}
                        >
                            {slide.subtitle}
                        </motion.p>

                        {/* Description */}
                        <motion.p
                            className={styles.description}
                            initial={{ opacity: 0, y: 20 }}
                            animate={{ opacity: 1, y: 0 }}
                            transition={{ delay: 0.6, duration: 0.6 }}
                        >
                            {slide.description}
                        </motion.p>

                        {/* Features List */}
                        <motion.div
                            className={styles.features}
                            variants={containerVariants}
                            initial="hidden"
                            animate="visible"
                        >
                            {slide.features.map((feature, index) => (
                                <motion.div
                                    key={index}
                                    className={styles.featureItem}
                                    variants={featureVariants}
                                >
                                    {feature}
                                </motion.div>
                            ))}
                        </motion.div>

                        {/* Next/CTA Button */}
                        <motion.button
                            className={styles.nextButton}
                            onClick={handleNext}
                            initial={{ opacity: 0, scale: 0.8 }}
                            animate={{ opacity: 1, scale: 1 }}
                            transition={{ delay: 0.9, duration: 0.4 }}
                            whileHover={{ scale: 1.05 }}
                            whileTap={{ scale: 0.95 }}
                        >
                            <span>{slide.buttonText}</span>
                        </motion.button>
                    </div>
                </motion.div>
            </AnimatePresence>

            {/* Pagination Dots */}
            <div className={styles.pagination}>
                {introSlides.map((_, index) => (
                    <motion.div
                        key={index}
                        className={`${styles.dot} ${index === currentSlide ? styles.active : ""}`}
                        onClick={() => handleDotClick(index)}
                        whileHover={{ scale: 1.2 }}
                        whileTap={{ scale: 0.9 }}
                    />
                ))}
            </div>
        </div>
    );
}
