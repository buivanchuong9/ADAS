'use client';

import { useEffect, useState } from 'react';
import { useRouter, usePathname } from 'next/navigation';

const INTRO_KEY = 'adas_intro_completed';
const ALLOWED_PATHS = ['/intro', '/login', '/register'];

/**
 * IntroGuard ensures users see the intro screen first
 * before accessing any other pages in the app
 */
export function IntroGuard({ children }: { children: React.ReactNode }) {
    const router = useRouter();
    const pathname = usePathname();
    const [isChecking, setIsChecking] = useState(true);
    const [shouldRender, setShouldRender] = useState(false);

    useEffect(() => {
        // Skip check if already on allowed paths
        if (ALLOWED_PATHS.includes(pathname)) {
            setShouldRender(true);
            setIsChecking(false);
            return;
        }

        // Check if intro has been completed
        const introCompleted = sessionStorage.getItem(INTRO_KEY);

        if (!introCompleted) {
            console.log('🔵 [IntroGuard] Redirecting to /intro - intro not completed');
            router.replace('/intro'); // Use replace instead of push to avoid back button issues
            setShouldRender(false);
        } else {
            setShouldRender(true);
        }

        setIsChecking(false);
    }, [pathname, router]);

    // Don't render anything while checking or if should redirect
    if (isChecking || !shouldRender) {
        return null;
    }

    return <>{children}</>;
}
