'use client';

import { useState } from 'react';
import { useRouter } from 'next/navigation';
import IntroSlider from '@/components/onboarding/IntroSlider';
import { useAuth } from '@/contexts/auth-context';

const INTRO_KEY = 'adas_intro_completed';

export default function IntroPage() {
    const [isTransitioning, setIsTransitioning] = useState(false);
    const router = useRouter();
    const { isAuthenticated } = useAuth();

    const handleComplete = () => {
        setIsTransitioning(true);

        // Mark intro as completed
        sessionStorage.setItem(INTRO_KEY, 'true');

        // Wait for fade out animation before navigating
        setTimeout(() => {
            // Logged-in users go to dashboard, guests go to overview
            if (isAuthenticated) {
                router.push('/dashboard');
            } else {
                router.push('/overview');
            }
        }, 300);
    };

    // Show transitioning state
    if (isTransitioning) {
        return (
            <div
                style={{
                    position: 'fixed',
                    inset: 0,
                    backgroundColor: '#000',
                    opacity: 0,
                    animation: 'fadeInSimple 300ms ease-in forwards',
                    zIndex: 9999
                }}
            />
        );
    }

    return <IntroSlider onComplete={handleComplete} />;
}
