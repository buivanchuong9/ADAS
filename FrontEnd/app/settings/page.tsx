"use client";

import { useState } from "react";
import { Sidebar } from "@/components/sidebar";
import { MobileNav } from "@/components/mobile-nav";
import { Header } from "@/components/header";
import { GlassCard } from "@/components/ui/glass-card";
import { Badge } from "@/components/ui/badge";
import { useLanguage } from "@/contexts/language-context";
import { Settings, Brain, Sliders, Gauge, RotateCcw, ChevronDown } from "lucide-react";

export default function SettingsPage() {
  const { language, setLanguage, t } = useLanguage();

  // System Settings
  const [theme, setTheme] = useState("dark");
  const [timezone, setTimezone] = useState("Asia/Ho_Chi_Minh");
  const [autoSave, setAutoSave] = useState(true);

  // Notification Settings
  const [alertSound, setAlertSound] = useState(true);
  const [warningThreshold, setWarningThreshold] = useState(75);
  const [emailNotif, setEmailNotif] = useState(false);
  const [pushNotif, setPushNotif] = useState(true);

  // AI Assistant Settings
  const [aiEnabled, setAiEnabled] = useState(true);
  const [aiLanguage, setAiLanguage] = useState("vi");
  const [voiceFeedback, setVoiceFeedback] = useState(false);
  const [autoSuggestions, setAutoSuggestions] = useState(true);

  // Detection Settings
  const [confidenceThreshold, setConfidenceThreshold] = useState(70);
  const [frameSkip, setFrameSkip] = useState(0);
  const [recordingQuality, setRecordingQuality] = useState("high");

  // Advanced Settings
  const [perfMonitoring, setPerfMonitoring] = useState(true);
  const [dataRetention, setDataRetention] = useState(30);

  const [hasChanges, setHasChanges] = useState(false);

  const handleReset = () => {
    setLanguage("vi");
    setTheme("dark");
    setAlertSound(true);
    setWarningThreshold(75);
    setAiEnabled(true);
    setConfidenceThreshold(70);
    setHasChanges(false);
  };

  return (
   <div className="flex flex-col h-screen bg-bg-primary overflow-hidden">
      <Header />
      <div className="flex flex-1 overflow-hidden">
      <MobileNav />
      <Sidebar />

      <main className="flex-1 overflow-auto">
        <div
          className="p-4 sm:p-6 lg:p-8 space-y-6"
          style={{ fontFamily: "var(--font-inter)" }}
        >
          {/* Header */}
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-4xl font-extrabold text-neon-cyan tracking-wide">

                {t("settings.title")}
              </h1>
              <p className="text-sm text-fg-secondary mt-1 font-medium">
                {t("settings.subtitle")}
              </p>
            </div>
            <div className="flex gap-3">
              <button
                onClick={handleReset}
                className="glass-card border-neon-yellow/50 text-neon-yellow px-4 py-2 rounded-lg hover:glow-yellow transition-all flex items-center gap-2 font-semibold"
              >
                <RotateCcw className="w-4 h-4" />
                <span className="hidden sm:inline">{t("settings.reset")}</span>
              </button>
            </div>
          </div>

          {/* System Settings */}
          <GlassCard scanLines className="p-6">
            <div className="flex items-center gap-3 mb-6">
              <Settings className="w-6 h-6 text-neon-cyan" />
              <h2 className="text-xl font-bold text-neon-cyan tracking-wide">
                {t("settings.system")}
              </h2>
            </div>
            <div className="space-y-6">
              <div className="space-y-2">
                <label className="text-sm font-semibold text-fg-primary">
                  {t("settings.language")}
                </label>
                <div className="relative">
                  <select
                    value={language}
                    onChange={(e) => {
                      const newLang = e.target.value as "vi" | "en";
                      setLanguage(newLang);
                    }}
                    className="w-full glass-card border-neon-cyan/30 text-fg-primary px-4 py-3 rounded-lg appearance-none cursor-pointer hover:border-neon-cyan/50 transition-all font-medium"
                  >
                    <option value="vi">{t("settings.vietnamese")}</option>
                    <option value="en">{t("settings.english")}</option>
                  </select>
                  <ChevronDown className="absolute right-3 top-1/2 -translate-y-1/2 w-5 h-5 text-fg-secondary pointer-events-none" />
                </div>
                <p className="text-xs text-fg-muted font-medium">
                  {t("settings.languageDesc")}
                </p>
              </div>
            </div>
          </GlassCard>

          {/* AI Assistant Settings */}
          <GlassCard scanLines className="p-6">
            <div className="flex items-center gap-3 mb-6">
              <Brain className="w-6 h-6 text-neon-cyan" />
              <h2 className="text-xl font-bold text-neon-cyan tracking-wide">
                {t("settings.aiAssistant")}
              </h2>
            </div>
            <div className="grid gap-6 md:grid-cols-2">
              <div className="space-y-2">
                <div className="flex items-center justify-between">
                  <div>
                    <label className="text-sm font-semibold text-fg-primary">
                      {t("settings.enableAI")}
                    </label>
                    <p className="text-xs text-fg-muted mt-1 font-medium">
                      {t("settings.enableAIDesc")}
                    </p>
                  </div>
                  <button
                    onClick={() => {
                      setAiEnabled(!aiEnabled);
                      setHasChanges(true);
                    }}
                    className={`relative w-14 h-7 rounded-full transition-all ${
                      aiEnabled ? "bg-[#ff7a1a]/30" : "bg-gray-300"
                    }`}
                  >
                    <div
                      className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${
                        aiEnabled ? "translate-x-7 bg-[#ff7a1a]" : "bg-gray-500"
                      }`}
                      style={aiEnabled ? { boxShadow: "0 0 10px #ff7a1a" } : {}}
                    />
                  </button>
                </div>
              </div>

              <div className="space-y-2">
                <label className="text-sm font-semibold text-fg-primary">
                  {t("settings.aiResponseLanguage")}
                </label>
                <div className="relative">
                  <select
                    value={aiLanguage}
                    onChange={(e) => {
                      setAiLanguage(e.target.value);
                      setHasChanges(true);
                    }}
                    className="w-full glass-card border-neon-cyan/30 text-fg-primary px-4 py-3 rounded-lg appearance-none cursor-pointer hover:border-neon-cyan/50 transition-all font-medium"
                    disabled={!aiEnabled}
                  >
                    <option value="vi">{t("settings.vietnamese")}</option>
                    <option value="en">{t("settings.english")}</option>
                    <option value="auto">{t("settings.auto")}</option>
                  </select>
                  <ChevronDown className="absolute right-3 top-1/2 -translate-y-1/2 w-5 h-5 text-fg-secondary pointer-events-none" />
                </div>
                <p className="text-xs text-fg-muted font-medium">
                  {t("settings.aiResponseLanguageDesc")}
                </p>
              </div>

              <div className="space-y-2">
                <div className="flex items-center justify-between">
                  <div>
                    <label className="text-sm font-semibold text-fg-primary">
                      {t("settings.voiceFeedback")}
                    </label>
                    <p className="text-xs text-fg-muted mt-1 font-medium">
                      {t("settings.voiceFeedbackDesc")}
                    </p>
                  </div>
                  <button
                    onClick={() => {
                      setVoiceFeedback(!voiceFeedback);
                      setHasChanges(true);
                    }}
                    className={`relative w-14 h-7 rounded-full transition-all ${
                      voiceFeedback ? "bg-[#ff7a1a]/30" : "bg-gray-300"
                    }`}
                    disabled={!aiEnabled}
                  >
                    <div
                      className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${
                        voiceFeedback
                          ? "translate-x-7 bg-[#ff7a1a]"
                          : "bg-gray-500"
                      }`}
                      style={
                        voiceFeedback ? { boxShadow: "0 0 10px #ff7a1a" } : {}
                      }
                    />
                  </button>
                </div>
              </div>
            </div>
          </GlassCard>

          {/* Advanced Settings */}
          <GlassCard scanLines className="p-6">
            <div className="flex items-center gap-3 mb-6">
              <Sliders className="w-6 h-6 text-neon-cyan" />
              <h2 className="text-xl font-bold text-neon-cyan tracking-wide">
                {t("settings.advanced")}
              </h2>
            </div>
            <div className="grid gap-6 md:grid-cols-2">
              <div className="space-y-2">
                <div className="flex items-center justify-between">
                  <div>
                    <label className="text-sm font-semibold text-fg-primary">
                      {t("settings.performanceMonitoring")}
                    </label>
                    <p className="text-xs text-fg-muted mt-1 font-medium">
                      {t("settings.performanceMonitoringDesc")}
                    </p>
                  </div>
                  <button
                    onClick={() => {
                      setPerfMonitoring(!perfMonitoring);
                      setHasChanges(true);
                    }}
                    className={`relative w-14 h-7 rounded-full transition-all ${
                      perfMonitoring ? "bg-[#ff7a1a]/30" : "bg-gray-300"
                    }`}
                  >
                    <div
                      className={`absolute top-1 left-1 w-5 h-5 rounded-full transition-all ${
                        perfMonitoring
                          ? "translate-x-7 bg-[#ff7a1a]"
                          : "bg-gray-500"
                      }`}
                      style={
                        perfMonitoring ? { boxShadow: "0 0 10px #ff7a1a" } : {}
                      }
                    />
                  </button>
                </div>
              </div>

              <div className="space-y-3">
                <div className="flex items-center justify-between">
                  <label className="text-sm font-semibold text-fg-primary">
                    {t("settings.dataRetention")}
                  </label>
                  <span className="text-sm font-bold text-neon-cyan digital-number">
                    {t("settings.days", { count: dataRetention })}
                  </span>
                </div>
                <input
                  type="range"
                  min="7"
                  max="90"
                  step="7"
                  value={dataRetention}
                  onChange={(e) => {
                    setDataRetention(Number(e.target.value));
                    setHasChanges(true);
                  }}
                  className="w-full h-2 bg-white/10 rounded-full appearance-none cursor-pointer [&::-webkit-slider-thumb]:appearance-none [&::-webkit-slider-thumb]:w-4 [&::-webkit-slider-thumb]:h-4 [&::-webkit-slider-thumb]:rounded-full [&::-webkit-slider-thumb]:bg-neon-cyan [&::-webkit-slider-thumb]:cursor-pointer"
                  style={{
                    background: `linear-gradient(to right, var(--neon-cyan) 0%, var(--neon-cyan) ${((dataRetention - 7) / 83) * 100}%, rgba(255,255,255,0.1) ${((dataRetention - 7) / 83) * 100}%, rgba(255,255,255,0.1) 100%)`,
                  }}
                />
                <p className="text-xs text-fg-muted font-medium">
                  {t("settings.dataRetentionDesc")}
                </p>
              </div>
            </div>
          </GlassCard>

          {/* Info Card */}
          <GlassCard className="p-6 border-neon-cyan/30">
            <div className="flex items-start gap-4">
              <div className="w-10 h-10 rounded-full glass-card border-neon-cyan/50 flex items-center justify-center flex-shrink-0">
                <Gauge className="w-5 h-5 text-neon-cyan" />
              </div>
              <div className="flex-1">
                <h3 className="text-sm font-bold text-neon-cyan mb-2">
                  {t("settings.note")}
                </h3>
                <p className="text-xs text-fg-secondary leading-relaxed font-medium">
                  {t("settings.noteText")}
                </p>
              </div>
            </div>
          </GlassCard>
        </div>
      </main>
      </div>
    </div>
  );
}
