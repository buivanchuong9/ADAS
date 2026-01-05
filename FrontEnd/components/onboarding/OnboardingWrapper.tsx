"use client";

import type { ReactNode } from "react";
import { useEffect, useState } from "react";
import { useRouter } from "next/navigation";
import IntroSlider from "./IntroSlider";

const ONBOARDING_KEY = "adas_onboarding_completed";

const FORCE_SHOW_INTRO = true;
// const FORCE_SHOW_INTRO = process.env.NODE_ENV !== "production";

type OnboardingWrapperProps = {
  children: ReactNode;
};

export default function OnboardingWrapper({ children }: OnboardingWrapperProps) {
  const [isOnboardingComplete, setIsOnboardingComplete] = useState<boolean | null>(null);
  const router = useRouter();

  useEffect(() => {
    if (FORCE_SHOW_INTRO) {
      // Luôn xem intro mỗi lần refresh/reload
      sessionStorage.removeItem(ONBOARDING_KEY);
      setIsOnboardingComplete(false);
      return;
    }

    // Chế độ: xem 1 lần / session
    const completed = sessionStorage.getItem(ONBOARDING_KEY);
    setIsOnboardingComplete(completed === "true");
  }, []);

  const handleOnboardingComplete = (): void => {
    if (!FORCE_SHOW_INTRO) {
      sessionStorage.setItem(ONBOARDING_KEY, "true");
    }
    setIsOnboardingComplete(true);
    router.push("/overview");
  };

  // Tránh flash khi chưa biết trạng thái
  if (isOnboardingComplete === null) return null;

  // Chưa complete thì show intro
  if (!isOnboardingComplete) {
    return <IntroSlider onComplete={handleOnboardingComplete} />;
  }

  // Complete rồi thì show app
  return <>{children}</>;
}