"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";

export default function ProfileForm({ user }: { user: any }) {
  const router = useRouter();
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
        // Refresh the page to get updated session data
        window.location.reload();
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
    <form onSubmit={handleSubmit} className="space-y-6">
      {/* Read-only Email (show, but don't allow editing) */}
      <div>
        <label className="block text-sm font-medium mb-2">
          Email (verified)
        </label>
        <input
          type="email"
          value={user.email}
          disabled
          className="w-full px-3 py-2 border rounded-md bg-gray-100 cursor-not-allowed"
        />
        <p className="text-xs text-gray-500 mt-1">
          To change your email, contact support or use account settings
        </p>
      </div>

      {/* Editable: Full Name */}
      <div>
        <label className="block text-sm font-medium mb-2">
          Full Name
        </label>
        <input
          type="text"
          value={formData.name}
          onChange={(e) => setFormData({...formData, name: e.target.value})}
          placeholder="John Doe"
          className="w-full px-3 py-2 border rounded-md"
        />
      </div>

      {/* Editable: First & Last Name */}
      <div className="grid grid-cols-2 gap-4">
        <div>
          <label className="block text-sm font-medium mb-2">
            First Name
          </label>
          <input
            type="text"
            value={formData.givenName}
            onChange={(e) => setFormData({...formData, givenName: e.target.value})}
            placeholder="John"
            className="w-full px-3 py-2 border rounded-md"
          />
        </div>
        <div>
          <label className="block text-sm font-medium mb-2">
            Last Name
          </label>
          <input
            type="text"
            value={formData.familyName}
            onChange={(e) => setFormData({...formData, familyName: e.target.value})}
            placeholder="Doe"
            className="w-full px-3 py-2 border rounded-md"
          />
        </div>
      </div>


      {/* Editable: Software Background */}
      <div>
        <label className="block text-sm font-medium mb-2">
          Software Background
        </label>
        <select
          value={formData.softwareBackground}
          onChange={(e) => setFormData({...formData, softwareBackground: e.target.value})}
          className="w-full px-3 py-2 border rounded-md"
        >
          <option value="beginner">Beginner - Just starting out</option>
          <option value="intermediate">Intermediate - Some experience</option>
          <option value="advanced">Advanced - Professional level</option>
        </select>
      </div>

      {/* Editable: Hardware Tier */}
      <div>
        <label className="block text-sm font-medium mb-2">
          Hardware Tier
        </label>
        <select
          value={formData.hardwareTier}
          onChange={(e) => setFormData({...formData, hardwareTier: e.target.value})}
          className="w-full px-3 py-2 border rounded-md"
        >
          <option value="tier1">Tier 1 - Laptop/Cloud (Browser-based)</option>
          <option value="tier2">Tier 2 - RTX GPU (Local simulation)</option>
          <option value="tier3">Tier 3 - Jetson Edge (Real sensors)</option>
          <option value="tier4">Tier 4 - Physical Robot (Unitree Go2/G1)</option>
        </select>
        <p className="text-xs text-gray-500 mt-1">
          This determines what content you can access
        </p>
      </div>

      {/* Optional: Locale */}
      <div>
        <label className="block text-sm font-medium mb-2">
          Language
        </label>
        <select
          value={formData.locale}
          onChange={(e) => setFormData({...formData, locale: e.target.value})}
          className="w-full px-3 py-2 border rounded-md"
        >
          <option value="en-US">English (US)</option>
          <option value="en-GB">English (UK)</option>
          <option value="ur-PK">اردو (Urdu)</option>
          <option value="es-ES">Español (Spanish)</option>
          <option value="fr-FR">Français (French)</option>
        </select>
      </div>

      {/* Optional: Timezone */}
      <div>
        <label className="block text-sm font-medium mb-2">
          Timezone
        </label>
        <select
          value={formData.zoneinfo}
          onChange={(e) => setFormData({...formData, zoneinfo: e.target.value})}
          className="w-full px-3 py-2 border rounded-md"
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

      {/* Submit Button */}
      <button
        type="submit"
        disabled={loading}
        className="w-full bg-blue-600 text-white py-2 px-4 rounded-md hover:bg-blue-700 disabled:opacity-50 disabled:cursor-not-allowed"
      >
        {loading ? "Saving..." : "Save Changes"}
      </button>
    </form>
  );
}
