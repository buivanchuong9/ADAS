'use client';

import type { Metadata } from 'next';
import Navigation from '@/components/overview/Navigation';
import Hero from '@/components/overview/Hero';
import VisionGallery from '@/components/overview/VisionGallery';
import VideoDemo from '@/components/overview/VideoDemo';
import Features from '@/components/overview/Features';
import Footer from '@/components/overview/Footer';
import styles from './overview.module.css';
import './dark-theme.css';

export default function OverviewPage() {
    return (
        <div className={styles.overviewPage}>
            <Navigation />
            <Hero />
            <VisionGallery />
            <VideoDemo />
            <Features />
            <Footer />
        </div>
    );
}
