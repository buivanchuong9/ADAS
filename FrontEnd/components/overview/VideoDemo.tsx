'use client';

import { useEffect, useRef } from 'react';
import styles from './VideoDemo.module.css';

export default function VideoDemo() {
    const videoRef = useRef<HTMLVideoElement>(null);

    useEffect(() => {
        // Force video to play when component mounts
        const playVideo = async () => {
            if (videoRef.current) {
                try {
                    // Ensure video is muted for autoplay to work
                    videoRef.current.muted = true;
                    await videoRef.current.play();
                } catch (error) {
                    console.log('Video autoplay failed:', error);
                }
            }
        };

        playVideo();

        // Add event listener to replay when video ends (backup for loop attribute)
        const handleEnded = () => {
            if (videoRef.current) {
                videoRef.current.currentTime = 0;
                videoRef.current.play();
            }
        };

        const video = videoRef.current;
        if (video) {
            video.addEventListener('ended', handleEnded);
        }

        return () => {
            if (video) {
                video.removeEventListener('ended', handleEnded);
            }
        };
    }, []);

    return (
        <section className={styles.videoSection} id="demo">
            <div className={styles.videoContainer}>
                <div className={styles.headerBar}>
                    <h2>THỬ NGHIỆM THỰC TẾ</h2>
                    <div className={styles.liveIndicator}>● LIVE RECORDING</div>
                </div>

                <div className={styles.videoFrame}>
                    <video
                        ref={videoRef}
                        autoPlay
                        loop
                        muted
                        playsInline
                        controls
                    >
                        <source
                            src="/adas-demo.mp4"
                            type="video/mp4"
                        />
                        Your browser does not support the video tag.
                    </video>
                </div>

                <div className={styles.videoStatsBar}>
                    <div className={styles.vStat}>
                        MODEL: <strong>Ultra Fast Lane Detection (UFLD)</strong>
                    </div>
                    <div className={styles.vStat}>
                        RESOLUTION: <strong>1920x1080</strong>
                    </div>
                    <div className={styles.vStat}>
                        FPS: <strong>58.4</strong>
                    </div>
                    <div className={styles.vStat}>
                        DEVICE: <strong>Jetson Orin Nano</strong>
                    </div>
                </div>
            </div>
        </section>
    );
}
