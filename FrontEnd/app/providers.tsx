"use client";

import React from "react";
import { LanguageProvider } from "@/contexts/language-context";
import { AuthProvider } from "@/contexts/auth-context";
import { ThemeProvider } from "@/components/theme-provider";
import { IntroGuard } from "@/components/intro-guard";

export function Providers({ children }: { children: React.ReactNode }) {
  return (
    <ThemeProvider
      attribute="data-theme"
      defaultTheme="light"
      enableSystem={false}
      forcedTheme="light"
    >
      <LanguageProvider>
        <AuthProvider>
          <IntroGuard>{children}</IntroGuard>
        </AuthProvider>
      </LanguageProvider>
    </ThemeProvider>
  );
}
