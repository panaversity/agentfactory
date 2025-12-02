"use client";

import { useState } from "react";

export default function ProfileForm({
  user,
  redirectUrl
}: {
  user: any;
  redirectUrl: string | null;
}) {
  const [loading, setLoading] = useState(false);
  const [formData, setFormData] = useState({
    // OIDC Standard Claims (editable)
    name: user.name || "",
    givenName: user.givenName || "",
    familyName: user.familyName || "",

    // Custom Fields (editable)
    softwareBackground: user.softwareBackground || "beginner",
    hardwareTier: user.hardwareTier || "tier1",

    // Optional: Preferences
    locale: user.locale || "en-US",
    zoneinfo: user.zoneinfo || "America/New_York",
  });

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);

    try {
      const response = await fetch("/api/account/profile", {
        method: "PATCH",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(formData),
      });

      if (response.ok) {
        if (redirectUrl) {
          // Redirect back to client app
          window.location.href = redirectUrl;
        } else {
          // No redirect URL, just reload current page
          window.location.reload();
        }
      } else {
        const error = await response.json();
        alert(`Error: ${error.message}`);
      }
    } catch (error) {
      alert("Failed to update profile");
    } finally {
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className="space-y-8">
      {/* Account Information Section */}
      <section className="space-y-5">
        <div className="border-b-2 border-slate-200 pb-3">
          <h3 className="text-lg font-bold text-slate-900 tracking-tight">Account Information</h3>
          <p className="text-sm text-slate-600 mt-1">Your verified account details</p>
        </div>

        {/* Read-only Email */}
        <div className="space-y-3">
          <label className="block text-sm font-semibold text-slate-700">
            Email Address
          </label>
          <div className="relative">
            <input
              type="email"
              value={user.email}
              disabled
              className="w-full px-4 py-3.5 border-2 border-slate-200 rounded-xl bg-gradient-to-br from-slate-50 to-slate-100/50 text-slate-600 font-medium cursor-not-allowed shadow-inner"
            />
            <div className="absolute right-4 top-1/2 -translate-y-1/2">
              <svg className="w-5 h-5 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M5 13l4 4L19 7" />
              </svg>
            </div>
          </div>
          <p className="text-xs text-slate-500 leading-relaxed flex items-center gap-1.5">
            <svg className="w-3.5 h-3.5 text-slate-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 15v2m-6 4h12a2 2 0 002-2v-6a2 2 0 00-2-2H6a2 2 0 00-2 2v6a2 2 0 002 2zm10-10V7a4 4 0 00-8 0v4h8z" />
            </svg>
            Email changes require verification — contact support if needed
          </p>
        </div>
      </section>

      {/* Personal Details Section */}
      <section className="space-y-5">
        <div className="border-b-2 border-slate-200 pb-3">
          <h3 className="text-lg font-bold text-slate-900 tracking-tight">Personal Details</h3>
          <p className="text-sm text-slate-600 mt-1">How you'd like to be identified</p>
        </div>

        {/* Full Name */}
        <div className="space-y-3">
          <label className="block text-sm font-semibold text-slate-700">
            Full Name
          </label>
          <input
            type="text"
            value={formData.name}
            onChange={(e) => setFormData({...formData, name: e.target.value})}
            placeholder="John Doe"
            className="w-full px-4 py-3.5 border-2 border-slate-200 rounded-xl focus:border-indigo-500 focus:ring-4 focus:ring-indigo-500/10 transition-all duration-200 bg-white hover:border-slate-300"
          />
        </div>

        {/* First & Last Name Grid */}
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-5">
          <div className="space-y-3">
            <label className="block text-sm font-semibold text-slate-700">
              First Name
            </label>
            <input
              type="text"
              value={formData.givenName}
              onChange={(e) => setFormData({...formData, givenName: e.target.value})}
              placeholder="John"
              className="w-full px-4 py-3.5 border-2 border-slate-200 rounded-xl focus:border-indigo-500 focus:ring-4 focus:ring-indigo-500/10 transition-all duration-200 bg-white hover:border-slate-300"
            />
          </div>
          <div className="space-y-3">
            <label className="block text-sm font-semibold text-slate-700">
              Last Name
            </label>
            <input
              type="text"
              value={formData.familyName}
              onChange={(e) => setFormData({...formData, familyName: e.target.value})}
              placeholder="Doe"
              className="w-full px-4 py-3.5 border-2 border-slate-200 rounded-xl focus:border-indigo-500 focus:ring-4 focus:ring-indigo-500/10 transition-all duration-200 bg-white hover:border-slate-300"
            />
          </div>
        </div>
      </section>

      {/* Learning Profile Section */}
      <section className="space-y-5">
        <div className="border-b-2 border-slate-200 pb-3">
          <h3 className="text-lg font-bold text-slate-900 tracking-tight">Learning Profile</h3>
          <p className="text-sm text-slate-600 mt-1">Personalize your educational experience</p>
        </div>

        {/* Software Background */}
        <div className="space-y-3">
          <label className="block text-sm font-semibold text-slate-700">
            Software Background
          </label>
          <select
            value={formData.softwareBackground}
            onChange={(e) => setFormData({...formData, softwareBackground: e.target.value})}
            className="w-full px-4 py-3.5 border-2 border-slate-200 rounded-xl focus:border-indigo-500 focus:ring-4 focus:ring-indigo-500/10 transition-all duration-200 bg-white hover:border-slate-300 cursor-pointer appearance-none bg-[url('data:image/svg+xml;charset=UTF-8,%3csvg xmlns=%27http://www.w3.org/2000/svg%27 viewBox=%270 0 24 24%27 fill=%27none%27 stroke=%27currentColor%27 stroke-width=%272%27 stroke-linecap=%27round%27 stroke-linejoin=%27round%27%3e%3cpolyline points=%276 9 12 15 18 9%27%3e%3c/polyline%3e%3c/svg%3e')] bg-[length:1.25rem] bg-[right_0.75rem_center] bg-no-repeat"
          >
            <option value="beginner">Beginner — Just starting out</option>
            <option value="intermediate">Intermediate — Some experience</option>
            <option value="advanced">Advanced — Professional level</option>
          </select>
        </div>

        {/* Hardware Tier */}
        <div className="space-y-3">
          <label className="block text-sm font-semibold text-slate-700">
            Hardware Tier
          </label>
          <select
            value={formData.hardwareTier}
            onChange={(e) => setFormData({...formData, hardwareTier: e.target.value})}
            className="w-full px-4 py-3.5 border-2 border-slate-200 rounded-xl focus:border-indigo-500 focus:ring-4 focus:ring-indigo-500/10 transition-all duration-200 bg-white hover:border-slate-300 cursor-pointer appearance-none bg-[url('data:image/svg+xml;charset=UTF-8,%3csvg xmlns=%27http://www.w3.org/2000/svg%27 viewBox=%270 0 24 24%27 fill=%27none%27 stroke=%27currentColor%27 stroke-width=%272%27 stroke-linecap=%27round%27 stroke-linejoin=%27round%27%3e%3cpolyline points=%276 9 12 15 18 9%27%3e%3c/polyline%3e%3c/svg%3e')] bg-[length:1.25rem] bg-[right_0.75rem_center] bg-no-repeat"
          >
            <option value="tier1">Tier 1 — Laptop/Cloud (Browser-based)</option>
            <option value="tier2">Tier 2 — RTX GPU (Local simulation)</option>
            <option value="tier3">Tier 3 — Jetson Edge (Real sensors)</option>
            <option value="tier4">Tier 4 — Physical Robot (Unitree Go2/G1)</option>
          </select>
          <p className="text-xs text-slate-500 leading-relaxed flex items-center gap-1.5">
            <svg className="w-3.5 h-3.5 text-slate-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
            Your tier determines which content and features are available
          </p>
        </div>
      </section>

      {/* Preferences Section */}
      <section className="space-y-5">
        <div className="border-b-2 border-slate-200 pb-3">
          <h3 className="text-lg font-bold text-slate-900 tracking-tight">Preferences</h3>
          <p className="text-sm text-slate-600 mt-1">Localization and regional settings</p>
        </div>

        {/* Language & Timezone Grid */}
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-5">
          <div className="space-y-3">
            <label className="block text-sm font-semibold text-slate-700">
              Language
            </label>
            <select
              value={formData.locale}
              onChange={(e) => setFormData({...formData, locale: e.target.value})}
              className="w-full px-4 py-3.5 border-2 border-slate-200 rounded-xl focus:border-indigo-500 focus:ring-4 focus:ring-indigo-500/10 transition-all duration-200 bg-white hover:border-slate-300 cursor-pointer appearance-none bg-[url('data:image/svg+xml;charset=UTF-8,%3csvg xmlns=%27http://www.w3.org/2000/svg%27 viewBox=%270 0 24 24%27 fill=%27none%27 stroke=%27currentColor%27 stroke-width=%272%27 stroke-linecap=%27round%27 stroke-linejoin=%27round%27%3e%3cpolyline points=%276 9 12 15 18 9%27%3e%3c/polyline%3e%3c/svg%3e')] bg-[length:1.25rem] bg-[right_0.75rem_center] bg-no-repeat"
            >
              <option value="en-US">English (US)</option>
              <option value="en-GB">English (UK)</option>
              <option value="ur-PK">اردو (Urdu)</option>
              <option value="es-ES">Español (Spanish)</option>
              <option value="fr-FR">Français (French)</option>
            </select>
          </div>

          <div className="space-y-3">
            <label className="block text-sm font-semibold text-slate-700">
              Timezone
            </label>
            <select
              value={formData.zoneinfo}
              onChange={(e) => setFormData({...formData, zoneinfo: e.target.value})}
              className="w-full px-4 py-3.5 border-2 border-slate-200 rounded-xl focus:border-indigo-500 focus:ring-4 focus:ring-indigo-500/10 transition-all duration-200 bg-white hover:border-slate-300 cursor-pointer appearance-none bg-[url('data:image/svg+xml;charset=UTF-8,%3csvg xmlns=%27http://www.w3.org/2000/svg%27 viewBox=%270 0 24 24%27 fill=%27none%27 stroke=%27currentColor%27 stroke-width=%272%27 stroke-linecap=%27round%27 stroke-linejoin=%27round%27%3e%3cpolyline points=%276 9 12 15 18 9%27%3e%3c/polyline%3e%3c/svg%3e')] bg-[length:1.25rem] bg-[right_0.75rem_center] bg-no-repeat"
            >
              <option value="America/New_York">Eastern Time (US)</option>
              <option value="America/Chicago">Central Time (US)</option>
              <option value="America/Denver">Mountain Time (US)</option>
              <option value="America/Los_Angeles">Pacific Time (US)</option>
              <option value="Asia/Karachi">Pakistan Standard Time</option>
              <option value="Europe/London">London (GMT)</option>
              <option value="Asia/Dubai">Dubai (GST)</option>
            </select>
          </div>
        </div>
      </section>

      {/* Action Buttons */}
      <div className="pt-4 flex gap-4 border-t-2 border-slate-200">
        <button
          type="submit"
          disabled={loading}
          className="flex-1 relative py-4 px-6 bg-gradient-to-r from-indigo-600 to-indigo-700 text-white font-bold rounded-xl shadow-lg shadow-indigo-500/30 hover:shadow-xl hover:shadow-indigo-500/40 hover:scale-[1.02] focus:outline-none focus:ring-4 focus:ring-indigo-500/50 transition-all duration-200 disabled:opacity-50 disabled:cursor-not-allowed disabled:hover:scale-100 overflow-hidden group"
        >
          {loading ? (
            <span className="flex items-center justify-center">
              <svg className="animate-spin -ml-1 mr-3 h-5 w-5 text-white" fill="none" viewBox="0 0 24 24">
                <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" />
                <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z" />
              </svg>
              Saving Changes...
            </span>
          ) : (
            <>
              <span className="relative z-10 flex items-center justify-center">
                Save Changes
                <svg className="ml-2 w-5 h-5 group-hover:translate-x-1 transition-transform" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M5 13l4 4L19 7" />
                </svg>
              </span>
              <div className="absolute inset-0 bg-gradient-to-r from-indigo-700 to-indigo-800 opacity-0 group-hover:opacity-100 transition-opacity" />
            </>
          )}
        </button>
      </div>
    </form>
  );
}
