'use client'

import { useTheme as useNextTheme } from 'next-themes'

type Theme = 'light' | 'dark'

export function useTheme() {
  const { theme, setTheme } = useNextTheme()

  // Nếu theme chưa có (undefined) thì coi như 'dark'
  const safeTheme: Theme = theme === 'light' ? 'light' : 'dark'

  const toggleTheme = () => {
    const nextTheme: Theme = safeTheme === 'dark' ? 'light' : 'dark'
    setTheme(nextTheme)
  }

  return {
    theme: safeTheme,
    setTheme: setTheme as (t: Theme) => void,
    toggleTheme,
  }
}