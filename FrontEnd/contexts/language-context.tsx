"use client";

import React, { createContext, useContext, useState, useEffect, ReactNode, useMemo } from "react";
import viTranslations from "@/locales/vi";
import enTranslations from "@/locales/en";

export type Language = "vi" | "en";

interface LanguageContextType {
  language: Language;
  setLanguage: (lang: Language) => void;
  t: (key: string, params?: Record<string, string | number>) => string;
}

const LanguageContext = createContext<LanguageContextType | undefined>(undefined);

const STORAGE_KEY = "adas-language";

export function LanguageProvider({ children }: { children: ReactNode }) {
  const [language, setLanguageState] = useState<Language>("vi");

  // Load language from localStorage on mount
  useEffect(() => {
    if (typeof window !== "undefined") {
      const saved = localStorage.getItem(STORAGE_KEY) as Language | null;
      if (saved === "vi" || saved === "en") {
        setLanguageState(saved);
      }
    }
  }, []);

  // Save language to localStorage when it changes
  const setLanguage = (lang: Language) => {
    setLanguageState(lang);
    if (typeof window !== "undefined") {
      localStorage.setItem(STORAGE_KEY, lang);
      // Update HTML lang attribute
      document.documentElement.lang = lang;
    }
  };

  // Get translations based on current language
  const translations = useMemo(() => {
    return language === "vi" ? viTranslations : enTranslations;
  }, [language]);

  // Translation function
  const t = (key: string, params?: Record<string, string | number>): string => {
    try {
      // Navigate through nested keys (e.g., "home.title")
      const keys = key.split(".");
      let value: any = translations;
      
      for (const k of keys) {
        if (value && typeof value === "object" && k in value) {
          value = value[k];
        } else {
          // Fallback to key if translation not found
          console.warn(`Translation missing for key: ${key}`);
          return key;
        }
      }

      // If value is a string, replace placeholders
      if (typeof value === "string" && params) {
        return value.replace(/\{(\w+)\}/g, (match, paramKey) => {
          return params[paramKey]?.toString() || match;
        });
      }

      return typeof value === "string" ? value : key;
    } catch (error) {
      console.error(`Error loading translation for key: ${key}`, error);
      return key;
    }
  };

  // Update HTML lang attribute when language changes
  useEffect(() => {
    if (typeof window !== "undefined") {
      document.documentElement.lang = language;
    }
  }, [language]);

  return (
    <LanguageContext.Provider value={{ language, setLanguage, t }}>
      {children}
    </LanguageContext.Provider>
  );
}

export function useLanguage() {
  const context = useContext(LanguageContext);
  if (context === undefined) {
    throw new Error("useLanguage must be used within a LanguageProvider");
  }
  return context;
}
