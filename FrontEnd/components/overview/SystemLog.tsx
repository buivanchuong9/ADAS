'use client';

import { useEffect, useRef, useState } from 'react';
import styles from './SystemLog.module.css';

const LOG_MESSAGES = [
    { text: '[INFO] Initializing System Core...', type: 'info' },
    { text: '[INFO] Loading Ultra Fast Lane Detection (UFLD) weights (6.4MB)...', type: 'info' },
    { text: '[OK] Model loaded successfully in 0.4s', type: 'ok' },
    { text: '[INFO] Connecting to Camera Input...', type: 'info' },
    { text: '[OK] Video Stream Established: 1080p', type: 'ok' },
    { text: '[INFO] Starting Inference Loop...', type: 'info' },
    { text: '> Object: Car (conf: 0.92)', type: 'output' },
    { text: '> Object: Motorbike (conf: 0.88)', type: 'output' },
    { text: '[WARN] Lane Departure Detected!', type: 'warn' },
    { text: '[INFO] Syncing to Cloud...', type: 'info' },
    { text: '[OK] Data Synced.', type: 'ok' },
];

export default function SystemLog() {
    const [logs, setLogs] = useState<Array<{ text: string; type: string }>>([]);
    const [lineIndex, setLineIndex] = useState(0);
    const logContainerRef = useRef<HTMLDivElement>(null);

    useEffect(() => {
        const addLogLine = () => {
            setLogs((prevLogs) => {
                const newLogs = [...prevLogs, LOG_MESSAGES[lineIndex]];
                // Keep only last 15 lines
                return newLogs.slice(-15);
            });

            setLineIndex((prev) => (prev + 1) % LOG_MESSAGES.length);
        };

        const interval = setInterval(addLogLine, Math.random() * 800 + 200);

        return () => clearInterval(interval);
    }, [lineIndex]);

    useEffect(() => {
        // Auto-scroll to bottom
        if (logContainerRef.current) {
            logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
        }
    }, [logs]);

    const getLogColor = (type: string) => {
        switch (type) {
            case 'ok':
                return '#00ff00';
            case 'warn':
                return '#ffcc00';
            case 'output':
                return '#ccc';
            default:
                return '#00ff00';
        }
    };

    return (
        <div className={styles.sysLog} ref={logContainerRef}>
            <div className={styles.header}>TERMINAL &gt; BE-ADAS.py</div>
            <div className={styles.logContent}>
                {logs.map((log, index) => (
                    <div
                        key={`${log.text}-${index}`}
                        className={styles.logLine}
                        style={{ color: getLogColor(log.type) }}
                    >
                        {log.text}
                    </div>
                ))}
            </div>
        </div>
    );
}
