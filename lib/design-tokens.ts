/**
 * ADAS Professional Design System - Color Tokens
 * Centralized color definitions for consistent theming
 */

export const colors = {
    // Primary palette - Professional blues
    primary: {
        50: 'hsl(217, 91%, 95%)',
        100: 'hsl(217, 91%, 85%)',
        200: 'hsl(217, 91%, 75%)',
        300: 'hsl(217, 91%, 65%)',
        400: 'hsl(217, 91%, 60%)', // Main primary
        500: 'hsl(217, 91%, 55%)',
        600: 'hsl(217, 91%, 50%)',
        700: 'hsl(217, 91%, 45%)',
        800: 'hsl(217, 91%, 40%)',
        900: 'hsl(217, 91%, 35%)',
    },

    // Accent palette - Vibrant orange for alerts
    accent: {
        50: 'hsl(25, 95%, 95%)',
        100: 'hsl(25, 95%, 85%)',
        200: 'hsl(25, 95%, 75%)',
        300: 'hsl(25, 95%, 65%)',
        400: 'hsl(25, 95%, 53%)', // Main accent
        500: 'hsl(25, 95%, 48%)',
        600: 'hsl(25, 95%, 43%)',
        700: 'hsl(25, 95%, 38%)',
        800: 'hsl(25, 95%, 33%)',
        900: 'hsl(25, 95%, 28%)',
    },

    // Semantic colors
    success: {
        light: 'hsl(142, 76%, 45%)',
        main: 'hsl(142, 76%, 36%)',
        dark: 'hsl(142, 76%, 28%)',
    },

    warning: {
        light: 'hsl(38, 92%, 60%)',
        main: 'hsl(38, 92%, 50%)',
        dark: 'hsl(38, 92%, 40%)',
    },

    error: {
        light: 'hsl(0, 84%, 70%)',
        main: 'hsl(0, 84%, 60%)',
        dark: 'hsl(0, 84%, 50%)',
    },

    info: {
        light: 'hsl(199, 89%, 58%)',
        main: 'hsl(199, 89%, 48%)',
        dark: 'hsl(199, 89%, 38%)',
    },

    // Neutral grays
    gray: {
        50: 'hsl(210, 40%, 98%)',
        100: 'hsl(210, 40%, 96%)',
        200: 'hsl(214, 32%, 91%)',
        300: 'hsl(213, 27%, 84%)',
        400: 'hsl(215, 20%, 65%)',
        500: 'hsl(215, 16%, 47%)',
        600: 'hsl(215, 19%, 35%)',
        700: 'hsl(215, 25%, 27%)',
        800: 'hsl(217, 33%, 17%)',
        900: 'hsl(222, 47%, 11%)',
    },

    // Chart colors for analytics
    chart: {
        blue: 'hsl(217, 91%, 60%)',
        green: 'hsl(142, 76%, 36%)',
        orange: 'hsl(25, 95%, 53%)',
        purple: 'hsl(280, 89%, 60%)',
        pink: 'hsl(340, 82%, 52%)',
        teal: 'hsl(173, 80%, 40%)',
        yellow: 'hsl(48, 96%, 53%)',
        red: 'hsl(0, 84%, 60%)',
    },
} as const

export type ColorPalette = typeof colors
