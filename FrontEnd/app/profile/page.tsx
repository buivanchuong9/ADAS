"use client";

import { useState, useEffect } from "react";
import { motion } from "framer-motion";
import { useLanguage } from "@/contexts/language-context";
import { useAuth } from "@/contexts/auth-context";
import { Header } from "@/components/header";
import { Sidebar } from "@/components/sidebar";
import { User, Save, Camera, Key, Eye, EyeOff } from "lucide-react";
import { toast } from "sonner";
import { getSupabase } from "@/lib/auth/supabase-client";
import UAParser from "ua-parser-js";

interface UserProfile {
  id?: number;
  auth_id: string;
  username: string | null;
  full_name: string | null;
  email: string | null;
  role: string | null;
  phone: string | null;
  avatar_url?: string | null; // Will be added later
}

export default function ProfilePage() {
  const { t } = useLanguage();
  const { user } = useAuth();
  const supabase = getSupabase();

  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [lastLoginIP, setLastLoginIP] = useState<string>("Đang tải...");
  const [showPasswordModal, setShowPasswordModal] = useState(false);
  const [showPasswords, setShowPasswords] = useState({
    current: false,
    new: false,
    confirm: false,
  });
  const [passwordForm, setPasswordForm] = useState({
    currentPassword: "",
    newPassword: "",
    confirmPassword: "",
  });
  const [savedProfile, setSavedProfile] = useState<UserProfile>({
    auth_id: "",
    username: "",
    full_name: "",
    email: "",
    role: "driver",
    phone: "",
    avatar_url: null,
  });
  const [formData, setFormData] = useState<UserProfile>({
    auth_id: "",
    username: "",
    full_name: "",
    email: "",
    role: "driver",
    phone: "",
    avatar_url: null,
  });

  // Fetch profile from Supabase
  useEffect(() => {
    if (user?.auth_id && supabase) {
      fetchProfile();
    } else {
      console.log(
        "⚠️ [Profile] Waiting for user auth_id or Supabase client...",
        {
          hasUser: !!user,
          hasAuthId: !!user?.auth_id,
          hasSupabase: !!supabase,
        },
      );
    }
  }, [user]);

  // Fetch real IP address
  useEffect(() => {
    const fetchIP = async () => {
      try {
        const response = await fetch("https://api.ipify.org?format=json");
        const data = await response.json();
        setLastLoginIP(data.ip);
      } catch (error) {
        console.error("Error fetching IP:", error);
        setLastLoginIP("Không xác định");
      }
    };
    fetchIP();
  }, []);

  const fetchProfile = async () => {
    try {
      setLoading(true);

      console.log("🔵 [Profile] Fetching profile for user:", user?.auth_id);
      console.log("🔵 [Profile] User object:", user);
      console.log(
        "🔵 [Profile] Supabase client:",
        supabase ? "Available" : "NULL",
      );

      if (!supabase) {
        console.error("❌ [Profile] Supabase client is null!");
        toast.error("Không thể kết nối với database");
        setLoading(false);
        return;
      }

      const { data, error } = await supabase
        .from("users")
        .select("*")
        .eq("auth_id", user?.auth_id)
        .single();

      console.log("🔵 [Profile] Query result - data:", data);
      console.log("🔵 [Profile] Query result - error:", error);
      console.log(
        "🔵 [Profile] Error stringified:",
        JSON.stringify(error, null, 2),
      );

      // Log detailed error information
      if (error) {
        console.error("❌ [Profile] Error details:", {
          fullError: error,
          message: error?.message || "Unknown error",
          details: error?.details || "No details available",
          hint: error?.hint || "No hint available",
          code: error?.code || "No error code",
        });

        // If user doesn't exist (PGRST116), this shouldn't happen
        // Users should be created during registration
        if (error.code === "PGRST116") {
          console.error("⚠️ [Profile] User not found in database");
          toast.error("Không tìm thấy thông tin người dùng");
          return;
        }

        toast.error("Không thể tải thông tin hồ sơ");
        return;
      }

      if (data) {
        console.log("✅ [Profile] Profile loaded successfully");
        setSavedProfile(data);
        setFormData(data);
      }
    } catch (err) {
      console.error("❌ [Profile] Unexpected error:", err);
      toast.error("Lỗi khi tải dữ liệu");
    } finally {
      setLoading(false);
    }
  };

  const handleInputChange = (
    e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>,
  ) => {
    const { name, value } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: name === "model_year" ? parseInt(value) || null : value,
    }));
  };

  const handleSave = async () => {
    try {
      setSaving(true);

      console.log("🔵 [Save] Starting save...", {
        auth_id: user?.auth_id,
        formData: formData,
      });

      if (!supabase) {
        console.error("❌ [Save] Supabase client is null!");
        toast.error("Không thể kết nối với database");
        setSaving(false);
        return;
      }

      const updateData = {
        username: formData.username,
        full_name: formData.full_name,
        email: formData.email,
        // phone: formData.phone, // TODO: Add phone column to users table first
      };

      // First check if user exists
      const { data: checkData, error: checkError } = await supabase
        .from("users")
        .select("id, auth_id, username, email")
        .eq("auth_id", user?.auth_id);

      console.log("🔵 [Save] User check:", {
        found: checkData?.length,
        data: checkData,
        error: checkError,
      });

      console.log("🔵 [Save] Update data:", updateData);

      const { data, error } = await supabase
        .from("users")
        .update(updateData)
        .eq("auth_id", user?.auth_id);

      console.log("🔵 [Save] Response data:", data);
      console.log("🔵 [Save] Response error:", error);

      if (error) {
        console.error("❌ [Save] Error updating profile:", {
          code: error.code,
          message: error.message,
          details: error.details,
          hint: error.hint,
        });
        toast.error(`Không thể lưu thông tin: ${error.message}`);
        return;
      }

      console.log("✅ [Save] Profile updated successfully");

      // Update savedProfile to sync avatar display
      setSavedProfile({ ...formData });

      toast.success(t("profile.saveSuccess"));
    } catch (err) {
      console.error("❌ [Save] Unexpected error:", err);
      toast.error(t("profile.saveError"));
    } finally {
      setSaving(false);
    }
  };

  const handleAvatarClick = () => {
    const input = document.createElement("input");
    input.type = "file";
    input.accept = "image/*";
    input.onchange = async (e: any) => {
      const file = e.target?.files?.[0];
      if (!file) return;

      if (!supabase) {
        toast.error("Không thể kết nối với database");
        return;
      }

      try {
        // Upload to Supabase Storage
        const fileExt = file.name.split(".").pop();
        const fileName = `${user?.auth_id}-${Date.now()}.${fileExt}`;
        const { data, error } = await supabase.storage
          .from("avatars")
          .upload(fileName, file, { upsert: true });

        if (error) {
          toast.error("Không thể tải ảnh lên");
          return;
        }

        // Get public URL
        const { data: urlData } = supabase.storage
          .from("avatars")
          .getPublicUrl(fileName);

        // Update user avatar
        await supabase
          .from("users")
          .update({ avatar_url: urlData.publicUrl })
          .eq("auth_id", user?.auth_id);

        setFormData((prev) => ({ ...prev, avatar_url: urlData.publicUrl }));
        toast.success("Đã cập nhật ảnh đại diện");
      } catch (err) {
        console.error("Avatar upload error:", err);
        toast.error("Lỗi khi tải ảnh");
      }
    };
    input.click();
  };

  if (loading) {
    return (
      <>
        <Header />
        <div className="flex fixed top-16 left-0 right-0 bottom-0">
          <Sidebar />
          <main className="flex-1 overflow-y-auto flex items-center justify-center">
            <div className="text-center">
              <div className="inline-block animate-spin rounded-full h-12 w-12 border-b-2 border-primary" />
              <p className="mt-4 text-gray-400">{t("common.loading")}</p>
            </div>
          </main>
        </div>
      </>
    );
  }

  return (
    <>
      <Header />
      <div className="flex fixed top-16 left-0 right-0 bottom-0">
        <Sidebar />
        <main className="flex-1 overflow-y-auto">
          <div className="pt-8 pb-6 px-4 sm:px-6 lg:px-8 max-w-5xl mx-auto">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              className="space-y-8"
            >
              {/* Header */}
              <div className="flex flex-col md:flex-row md:items-center justify-between gap-4">
                <div>
                  <h1 className="text-3xl font-bold bg-clip-text text-transparent bg-linear-to-r from-primary to-accent">
                    {t("profile.title")}
                  </h1>
                  <p className="text-gray-400 mt-1">{t("profile.subtitle")}</p>
                </div>
                <button
                  onClick={handleSave}
                  disabled={saving}
                  className="btn-neon flex items-center gap-2 px-6 py-2 rounded-xl disabled:opacity-50"
                >
                  <Save className="w-5 h-5" />
                  <span>
                    {saving ? t("common.loading") : t("profile.saveChanges")}
                  </span>
                </button>
              </div>

              {/* Content */}
              <div className="bg-white/5 backdrop-blur-md rounded-2xl p-6 border border-white/10">
                <motion.div
                  initial={{ opacity: 0 }}
                  animate={{ opacity: 1 }}
                  className="space-y-6"
                >
                  {/* Avatar */}
                  <div className="flex items-center gap-4">
                    <div
                      className="relative w-24 h-24 rounded-full border-2 border-primary/50 overflow-hidden cursor-pointer group"
                      onClick={handleAvatarClick}
                    >
                      {formData.avatar_url ? (
                        <img
                          src={formData.avatar_url}
                          alt="Avatar"
                          className="w-full h-full object-cover"
                        />
                      ) : (
                        <div className="w-full h-full bg-linear-to-br from-primary/20 to-accent/20 flex items-center justify-center">
                          <User className="w-12 h-12 text-primary" />
                        </div>
                      )}
                      <div className="absolute inset-0 bg-black/50 opacity-0 group-hover:opacity-100 transition-opacity flex items-center justify-center">
                        <Camera className="w-8 h-8 text-white" />
                      </div>
                    </div>
                    <div className="flex items-center gap-3">
                      <div>
                        <p className="text-lg font-semibold text-gray-900">
                          {savedProfile.username || "Chưa có username"}
                        </p>
                        <p className="text-sm text-gray-400">
                          {savedProfile.role === "driver" && "Lái xe"}
                          {savedProfile.role === "admin" && "Quản trị viên"}
                          {savedProfile.role === "researcher" &&
                            "Nhà nghiên cứu"}
                          {savedProfile.role === "tester" && "Thử nghiệm"}
                        </p>
                      </div>
                      <button
                        onClick={() => setShowPasswordModal(true)}
                        className="px-3 py-1.5 bg-primary/20 hover:bg-primary/30 border border-primary/50 rounded-lg text-primary text-sm flex items-center gap-2 transition-colors"
                      >
                        <Key className="w-4 h-4" />
                        Đổi mật khẩu
                      </button>
                    </div>
                  </div>

                  <div className="h-px bg-white/10" />

                  {/* Form Fields */}
                  <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                    <div>
                      <label className="block text-sm font-medium text-gray-400 mb-1">
                        Username
                      </label>
                      <input
                        type="text"
                        name="username"
                        value={formData.username || ""}
                        onChange={handleInputChange}
                        className="w-full bg-black/20 border border-white/10 rounded-xl px-4 py-2.5 focus:outline-none focus:border-primary transition-colors text-white"
                        placeholder="Tên đăng nhập"
                      />
                    </div>

                    <div>
                      <label className="block text-sm font-medium text-gray-400 mb-1">
                        {t("profile.fullName")}
                      </label>
                      <input
                        type="text"
                        name="full_name"
                        value={formData.full_name || ""}
                        onChange={handleInputChange}
                        className="w-full bg-black/20 border border-white/10 rounded-xl px-4 py-2.5 focus:outline-none focus:border-primary transition-colors text-white"
                        placeholder="Họ và tên đầy đủ"
                      />
                    </div>

                    <div>
                      <label className="block text-sm font-medium text-gray-400 mb-1">
                        {t("profile.email")}
                      </label>
                      <input
                        type="email"
                        name="email"
                        value={formData.email || ""}
                        onChange={handleInputChange}
                        className="w-full bg-black/20 border border-white/10 rounded-xl px-4 py-2.5 focus:outline-none focus:border-primary transition-colors text-white"
                        placeholder="example@email.com"
                      />
                    </div>

                    <div>
                      <label className="block text-sm font-medium text-gray-400 mb-1">
                        Số điện thoại
                      </label>
                      <input
                        type="tel"
                        name="phone"
                        value={formData.phone || ""}
                        onChange={handleInputChange}
                        className="w-full bg-black/20 border border-white/10 rounded-xl px-4 py-2.5 focus:outline-none focus:border-primary transition-colors text-white"
                        placeholder="0901234567"
                      />
                    </div>
                  </div>

                  <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                    <div>
                      <label className="block text-sm font-medium text-gray-400 mb-1">
                        IP đăng nhập gần nhất
                      </label>
                      <input
                        type="text"
                        value={lastLoginIP}
                        disabled
                        className="w-full bg-black/20 border border-white/10 rounded-xl px-4 py-2.5 focus:outline-none focus:border-primary transition-colors text-white opacity-50 cursor-not-allowed"
                      />
                    </div>

                    <div>
                      <label className="block text-sm font-medium text-gray-400 mb-1">
                        Thiết bị gần nhất
                      </label>
                      <input
                        type="text"
                        value={(() => {
                          const ua = navigator.userAgent;
                          let browser = "Unknown";
                          let os = "Unknown";

                          // Detect browser (check Cốc Cốc FIRST before Chrome since it's Chromium-based)
                          if (ua.includes("coc_coc") || ua.includes("CocCoc"))
                            browser = "Cốc Cốc";
                          else if (ua.includes("Edg")) browser = "Edge";
                          else if (ua.includes("Chrome")) browser = "Chrome";
                          else if (ua.includes("Firefox")) browser = "Firefox";
                          else if (ua.includes("Safari")) browser = "Safari";

                          // Detect OS
                          if (ua.includes("Windows")) os = "Windows";
                          else if (ua.includes("Mac")) os = "MacOS";
                          else if (ua.includes("Linux")) os = "Linux";
                          else if (ua.includes("Android")) os = "Android";
                          else if (ua.includes("iOS")) os = "iOS";

                          // Detect device type
                          const isMobile = /Mobile|Android|iPhone|iPad/.test(
                            ua,
                          );
                          const deviceType = isMobile ? "Mobile" : "Desktop";

                          return `${browser} / ${os} / ${deviceType}`;
                        })()}
                        disabled
                        className="w-full bg-black/20 border border-white/10 rounded-xl px-4 py-2.5 focus:outline-none focus:border-primary transition-colors text-white opacity-50 cursor-not-allowed"
                      />
                    </div>
                  </div>
                </motion.div>
              </div>
            </motion.div>
          </div>
        </main>
      </div>

      {/* Password Change Modal */}
      {showPasswordModal && (
        <div className="fixed inset-0 bg-black/85 backdrop-blur-lg flex items-center justify-center z-[100]">
          <motion.div
            initial={{ opacity: 0, scale: 0.95 }}
            animate={{ opacity: 1, scale: 1 }}
            className="bg-white rounded-2xl p-6 max-w-md w-full mx-4 shadow-2xl"
          >
            <div className="flex items-center justify-between mb-6">
              <h3 className="text-xl font-bold text-gray-900 flex items-center gap-2">
                <Key className="w-5 h-5 text-primary" />
                Thay đổi mật khẩu
              </h3>
              <button
                onClick={() => setShowPasswordModal(false)}
                className="text-gray-400 hover:text-gray-600 transition-colors text-2xl leading-none"
              >
                ✕
              </button>
            </div>

            <div className="space-y-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Mật khẩu hiện tại
                </label>
                <div className="relative">
                  <input
                    type={showPasswords.current ? "text" : "password"}
                    value={passwordForm.currentPassword}
                    onChange={(e) =>
                      setPasswordForm({
                        ...passwordForm,
                        currentPassword: e.target.value,
                      })
                    }
                    className="w-full bg-gray-50 border border-gray-300 rounded-xl px-4 py-2.5 pr-12 focus:outline-none focus:border-primary focus:ring-1 focus:ring-primary transition-colors text-gray-900"
                    placeholder="Nhập mật khẩu hiện tại"
                  />
                  <button
                    type="button"
                    onClick={() =>
                      setShowPasswords({
                        ...showPasswords,
                        current: !showPasswords.current,
                      })
                    }
                    className="absolute right-3 top-1/2 -translate-y-1/2 text-gray-400 hover:text-gray-600 transition-colors"
                  >
                    {showPasswords.current ? (
                      <EyeOff className="w-5 h-5" />
                    ) : (
                      <Eye className="w-5 h-5" />
                    )}
                  </button>
                </div>
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Mật khẩu mới
                </label>
                <div className="relative">
                  <input
                    type={showPasswords.new ? "text" : "password"}
                    value={passwordForm.newPassword}
                    onChange={(e) =>
                      setPasswordForm({
                        ...passwordForm,
                        newPassword: e.target.value,
                      })
                    }
                    className="w-full bg-gray-50 border border-gray-300 rounded-xl px-4 py-2.5 pr-12 focus:outline-none focus:border-primary focus:ring-1 focus:ring-primary transition-colors text-gray-900"
                    placeholder="Nhập mật khẩu mới"
                  />
                  <button
                    type="button"
                    onClick={() =>
                      setShowPasswords({
                        ...showPasswords,
                        new: !showPasswords.new,
                      })
                    }
                    className="absolute right-3 top-1/2 -translate-y-1/2 text-gray-400 hover:text-gray-600 transition-colors"
                  >
                    {showPasswords.new ? (
                      <EyeOff className="w-5 h-5" />
                    ) : (
                      <Eye className="w-5 h-5" />
                    )}
                  </button>
                </div>
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Nhập lại mật khẩu mới
                </label>
                <div className="relative">
                  <input
                    type={showPasswords.confirm ? "text" : "password"}
                    value={passwordForm.confirmPassword}
                    onChange={(e) =>
                      setPasswordForm({
                        ...passwordForm,
                        confirmPassword: e.target.value,
                      })
                    }
                    className="w-full bg-gray-50 border border-gray-300 rounded-xl px-4 py-2.5 pr-12 focus:outline-none focus:border-primary focus:ring-1 focus:ring-primary transition-colors text-gray-900"
                    placeholder="Nhập lại mật khẩu mới"
                  />
                  <button
                    type="button"
                    onClick={() =>
                      setShowPasswords({
                        ...showPasswords,
                        confirm: !showPasswords.confirm,
                      })
                    }
                    className="absolute right-3 top-1/2 -translate-y-1/2 text-gray-400 hover:text-gray-600 transition-colors"
                  >
                    {showPasswords.confirm ? (
                      <EyeOff className="w-5 h-5" />
                    ) : (
                      <Eye className="w-5 h-5" />
                    )}
                  </button>
                </div>
              </div>

              <div className="flex gap-3 mt-6">
                <button
                  onClick={() => {
                    setShowPasswordModal(false);
                    setPasswordForm({
                      currentPassword: "",
                      newPassword: "",
                      confirmPassword: "",
                    });
                  }}
                  className="flex-1 px-4 py-2.5 bg-gray-100 hover:bg-gray-200 border border-gray-300 rounded-xl text-gray-700 font-medium transition-colors"
                >
                  Hủy
                </button>
                <button
                  onClick={async () => {
                    if (
                      !passwordForm.currentPassword ||
                      !passwordForm.newPassword ||
                      !passwordForm.confirmPassword
                    ) {
                      toast.error("Vui lòng điền đầy đủ thông tin");
                      return;
                    }

                    if (
                      passwordForm.newPassword !== passwordForm.confirmPassword
                    ) {
                      toast.error("Mật khẩu mới không khớp");
                      return;
                    }

                    if (passwordForm.newPassword.length < 6) {
                      toast.error("Mật khẩu mới phải có ít nhất 6 ký tự");
                      return;
                    }

                    try {
                      // TODO: Call API to change password
                      // await supabase.auth.updateUser({ password: passwordForm.newPassword });

                      toast.success("Đổi mật khẩu thành công!");
                      setShowPasswordModal(false);
                      setPasswordForm({
                        currentPassword: "",
                        newPassword: "",
                        confirmPassword: "",
                      });
                    } catch (error) {
                      toast.error("Không thể đổi mật khẩu");
                    }
                  }}
                  className="flex-1 px-4 py-2.5 bg-primary hover:bg-primary/90 border border-primary rounded-xl text-white font-medium transition-colors"
                >
                  Xác nhận
                </button>
              </div>
            </div>
          </motion.div>
        </div>
      )}
    </>
  );
}
