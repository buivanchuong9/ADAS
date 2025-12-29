"use client";

import { useState, useEffect } from "react";
import IntroSlider from "./IntroSlider";

const ONBOARDING_KEY = "adas_onboarding_completed";

export default function OnboardingWrapper({ children }) {
    const [isOnboardingComplete, setIsOnboardingComplete] = useState(null);

    useEffect(() => {
        // Check if onboarding has been completed in this session
        const completed = sessionStorage.getItem(ONBOARDING_KEY);
        setIsOnboardingComplete(completed === "true");
    }, []);

    const handleOnboardingComplete = () => {
        sessionStorage.setItem(ONBOARDING_KEY, "true");
        setIsOnboardingComplete(true);
    };

    // Show nothing while checking sessionStorage (prevents flash)
    if (isOnboardingComplete === null) {
        return null;
    }

    // Show onboarding if not completed
    if (!isOnboardingComplete) {
        return <IntroSlider onComplete={handleOnboardingComplete} />;
    }

    // Show main app if onboarding is complete
    return <>{children}</>;
}
