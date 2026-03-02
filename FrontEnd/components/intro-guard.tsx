'use client';

import { useEffect, useState } from 'react';
import { useRouter, usePathname } from 'next/navigation';
import { useAuth } from '@/contexts/auth-context';

const INTRO_KEY = 'adas_intro_completed';
const INTRO_ALLOWED_PATHS = ['/intro', '/login', '/register'];
const AUTH_PUBLIC_PATHS = ['/intro', '/overview', '/login', '/register'];

/**
 * IntroGuard ensures users thấy intro trước
 * và đồng thời chặn truy cập các tab khi chưa đăng nhập
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

        // 1. Kiểm tra Intro trước
        if (!INTRO_ALLOWED_PATHS.includes(pathname)) {
            const introCompleted = sessionStorage.getItem(INTRO_KEY);

            if (!introCompleted) {
                console.log('🔵 [IntroGuard] Redirecting to /intro - intro not completed');
                router.replace('/intro');
                setShouldRender(false);
                setIsChecking(false);
                return;
            }
        }

        // 2. Kiểm tra đăng nhập
        if (!isAuthenticated && !AUTH_PUBLIC_PATHS.includes(pathname)) {
            console.log('🔵 [IntroGuard] Redirecting to /login - not authenticated');
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
