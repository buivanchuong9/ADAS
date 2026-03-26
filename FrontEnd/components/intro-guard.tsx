'use client';

import { useEffect, useState } from 'react';
import { useRouter, usePathname } from 'next/navigation';
import { useAuth } from '@/contexts/auth-context';

const AUTH_PUBLIC_PATHS = ['/', '/intro', '/overview', '/login', '/register'];
const AUTH_PAGES_WHEN_LOGGED_IN = ['/login', '/register'];

/**
 * Auth guard + intro/public route whitelist:
 * - Logged-in users can access all pages (except /login & /register -> redirect /dashboard).
 * - Not logged in users can access only /intro, /overview, /login, /register.
 */
export function IntroGuard({ children }: { children: React.ReactNode }) {
    const router = useRouter();
    const pathname = usePathname();
    const { isAuthenticated, loading } = useAuth();
    const [isChecking, setIsChecking] = useState(true);
    const [shouldRender, setShouldRender] = useState(false);

    useEffect(() => {
        // Chờ auth context khởi tạo xong
        if (loading) return;

        // Logged-in user: never redirect them to /intro based on introCompleted.
        if (isAuthenticated) {
            // Auth pages should bounce to dashboard when already logged in.
            if (AUTH_PAGES_WHEN_LOGGED_IN.includes(pathname)) {
                router.replace('/dashboard');
                setShouldRender(false);
                setIsChecking(false);
                return;
            }

            setShouldRender(true);
            setIsChecking(false);
            return;
        }

        // Not authenticated: only allow whitelisted public routes.
        if (!AUTH_PUBLIC_PATHS.includes(pathname)) {
            router.replace('/login');
            setShouldRender(false);
            setIsChecking(false);
            return;
        }

        setShouldRender(true);
        setIsChecking(false);
    }, [pathname, router, isAuthenticated, loading]);

    // Không render gì khi đang kiểm tra hoặc cần redirect
    if (isChecking || !shouldRender) {
        return null;
    }

    return <>{children}</>;
}
