"use client";

import { useState } from "react";
import { authClient } from "@/lib/auth-client";
import { Toast } from "@/components/toast";
import PhoneInput, { isValidPhoneNumber } from "react-phone-number-input";
import "react-phone-number-input/style.css";
import { CountrySelect } from "react-country-state-city";
import "react-country-state-city/dist/react-country-state-city.css";

export default function ProfileForm({
  user,
  redirectUrl
}: {
  user: any;
  redirectUrl: string | null;
}) {
  const [loading, setLoading] = useState(false);
  const [phoneError, setPhoneError] = useState("");
  const [toast, setToast] = useState<{ show: boolean; message: string; type: "success" | "error" | "info" }>({
    show: false,
    message: "",
    type: "success"
  });
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

    // Additional Information (003-user-profile-fields)
    phoneNumber: user.phoneNumber || "",
    gender: user.gender || "",
    fatherName: user.fatherName || "",
    city: user.city || "",
    country: user.country || "",
  });


  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    // Validate phone number if provided
    if (formData.phoneNumber && !isValidPhoneNumber(formData.phoneNumber)) {
      setPhoneError("Please enter a valid phone number");
      return;
    }
    setPhoneError("");
    setLoading(true);

    try {
      // Use Better Auth's official updateUser method
      const { data, error } = await authClient.updateUser({
        name: formData.name,
        givenName: formData.givenName,
        familyName: formData.familyName,
        softwareBackground: formData.softwareBackground,
        hardwareTier: formData.hardwareTier,
        locale: formData.locale,
        zoneinfo: formData.zoneinfo,
        // Additional fields (003-user-profile-fields)
        phoneNumber: formData.phoneNumber,
        gender: formData.gender,
        fatherName: formData.fatherName,
        city: formData.city,
        country: formData.country,
      });

      if (error) {
        setToast({ show: true, message: error.message || "Failed to update profile", type: "error" });
        setLoading(false);
        return;
      }

      // Show success toast
      setToast({ show: true, message: "Profile updated successfully!", type: "success" });
      setLoading(false);

      // Better Auth automatically refreshes the session!
      // Delay redirect/reload to let user see the toast
      setTimeout(() => {
        if (redirectUrl) {
          // Validate redirect URL to prevent open redirect vulnerability
          const isValidRedirect = (url: string): boolean => {
            try {
              const parsed = new URL(url, window.location.origin);
              return parsed.origin === window.location.origin;
            } catch {
              return url.startsWith('/'); // Allow relative URLs
            }
          };

          if (isValidRedirect(redirectUrl)) {
            // Add cache-busting parameter for client app
            const redirectWithRefresh = redirectUrl.includes("?")
              ? `${redirectUrl}&refresh=${Date.now()}`
              : `${redirectUrl}?refresh=${Date.now()}`;
            window.location.href = redirectWithRefresh;
          } else {
            // Invalid redirect URL, just reload
            window.location.reload();
          }
        } else {
          // Reload to reflect changes in UI
          window.location.reload();
        }
      }, 1500);
    } catch (error) {
      setToast({ show: true, message: "Failed to update profile. Please try again.", type: "error" });
      setLoading(false);
    }
  };

  // Common input class for consistency
  const inputClass = "w-full px-4 py-3.5 border-2 border-slate-200 rounded-xl focus:border-pana-500 focus:ring-4 focus:ring-pana-500/10 transition-all duration-200 bg-white hover:border-slate-300";
  const selectClass = `${inputClass} cursor-pointer appearance-none bg-[url('data:image/svg+xml;charset=UTF-8,%3csvg xmlns=%27http://www.w3.org/2000/svg%27 viewBox=%270 0 24 24%27 fill=%27none%27 stroke=%27currentColor%27 stroke-width=%272%27 stroke-linecap=%27round%27 stroke-linejoin=%27round%27%3e%3cpolyline points=%276 9 12 15 18 9%27%3e%3c/polyline%3e%3c/svg%3e')] bg-[length:1.25rem] bg-[right_0.75rem_center] bg-no-repeat`;

  return (
    <>
      {/* Toast Notification */}
      {toast.show && (
        <Toast
          message={toast.message}
          type={toast.type}
          onClose={() => setToast({ ...toast, show: false })}
        />
      )}

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
              <svg className="w-5 h-5 text-pana-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
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
            className={inputClass}
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
              className={inputClass}
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
              className={inputClass}
            />
          </div>
        </div>
      </section>

      {/* Demographics & Contact Section (003-user-profile-fields) */}
      <section className="space-y-5">
        <div className="border-b-2 border-slate-200 pb-3">
          <h3 className="text-lg font-bold text-slate-900 tracking-tight">Demographics & Contact</h3>
          <p className="text-sm text-slate-600 mt-1">Optional information to personalize your experience</p>
        </div>

        {/* Contact Subsection */}
        <div className="pt-2">
          <p className="text-xs font-semibold text-pana-600 uppercase tracking-wider mb-4">Contact</p>
        </div>
        <div className="space-y-3">
          <label htmlFor="phone-number" className="block text-sm font-semibold text-slate-700">
            Phone Number
          </label>
          <div className="phone-input-wrapper">
            <PhoneInput
              id="phone-number"
              international
              defaultCountry="PK"
              value={formData.phoneNumber}
              onChange={(value) => {
                const newValue = value || "";
                setFormData({...formData, phoneNumber: newValue});
                // Clear error when user enters a valid phone number
                if (phoneError && newValue && isValidPhoneNumber(newValue)) {
                  setPhoneError("");
                }
              }}
              className="w-full"
              inputClassName={inputClass}
              aria-describedby="phone-help"
            />
          </div>
          {phoneError ? (
            <p id="phone-help" className="text-xs text-red-600 leading-relaxed flex items-center gap-1.5">
              <svg className="w-3.5 h-3.5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
              </svg>
              {phoneError}
            </p>
          ) : (
            <p id="phone-help" className="text-xs text-slate-500 leading-relaxed flex items-center gap-1.5">
              <svg className="w-3.5 h-3.5 text-slate-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M3 5a2 2 0 012-2h3.28a1 1 0 01.948.684l1.498 4.493a1 1 0 01-.502 1.21l-2.257 1.13a11.042 11.042 0 005.516 5.516l1.13-2.257a1 1 0 011.21-.502l4.493 1.498a1 1 0 01.684.949V19a2 2 0 01-2 2h-1C9.716 21 3 14.284 3 6V5z" />
              </svg>
              Select your country code and enter phone number
            </p>
          )}
        </div>

        {/* Identity Subsection */}
        <div className="pt-2">
          <p className="text-xs font-semibold text-pana-600 uppercase tracking-wider mb-4">Identity</p>
        </div>
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-5">
          <div className="space-y-3">
            <label htmlFor="gender" className="block text-sm font-semibold text-slate-700">
              Gender
            </label>
            <select
              id="gender"
              value={formData.gender}
              onChange={(e) => setFormData({...formData, gender: e.target.value})}
              className={selectClass}
            >
              <option value="">Select gender...</option>
              <option value="male">Male</option>
              <option value="female">Female</option>
              <option value="other">Other</option>
              <option value="prefer_not_to_say">Prefer not to say</option>
            </select>
          </div>
          <div className="space-y-3">
            <label htmlFor="father-name" className="block text-sm font-semibold text-slate-700">
              Father's Name
            </label>
            <input
              id="father-name"
              type="text"
              value={formData.fatherName}
              onChange={(e) => setFormData({...formData, fatherName: e.target.value})}
              placeholder="Enter father's name"
              maxLength={100}
              className={inputClass}
            />
          </div>
        </div>

        {/* Location Subsection */}
        <div className="pt-2">
          <p className="text-xs font-semibold text-pana-600 uppercase tracking-wider mb-4">Location</p>
        </div>
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-5">
          <div className="space-y-3">
            <label className="block text-sm font-semibold text-slate-700">
              Country
            </label>
            <CountrySelect
              // eslint-disable-next-line @typescript-eslint/no-explicit-any
              onChange={(e: any) => {
                setFormData({...formData, country: e?.name || ""});
              }}
              placeHolder={formData.country || "Select country..."}
              containerClassName="country-select-container"
            />
          </div>
          <div className="space-y-3">
            <label htmlFor="city" className="block text-sm font-semibold text-slate-700">
              City
            </label>
            <input
              id="city"
              type="text"
              value={formData.city}
              onChange={(e) => setFormData({...formData, city: e.target.value})}
              placeholder="Enter your city..."
              maxLength={100}
              className={inputClass}
            />
          </div>
        </div>

        <p className="text-xs text-slate-500 leading-relaxed flex items-center gap-1.5 bg-pana-50/50 p-3 rounded-lg border border-pana-100">
          <svg className="w-4 h-4 text-pana-600 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
          </svg>
          <span>All additional information fields are optional and help personalize your experience across Panaversity platforms.</span>
        </p>
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
            className={selectClass}
          >
            <option value="beginner">Beginner — Just starting out</option>
            <option value="intermediate">Intermediate — Some experience</option>
            <option value="advanced">Advanced — Professional level</option>
          </select>
        </div>

        {/* Hardware & OS */}
        <div className="space-y-3">
          <label className="block text-sm font-semibold text-slate-700">
            Hardware & Operating System
          </label>
          <select
            value={formData.hardwareTier}
            onChange={(e) => setFormData({...formData, hardwareTier: e.target.value})}
            className={selectClass}
          >
            <option value="tier1">Windows PC — Desktop or laptop</option>
            <option value="tier2">Mac — MacBook or iMac</option>
            <option value="tier3">Linux — Ubuntu, Fedora, etc.</option>
            <option value="tier4">Chromebook/Web — Browser-based only</option>
          </select>
          <p className="text-xs text-slate-500 leading-relaxed flex items-center gap-1.5">
            <svg className="w-3.5 h-3.5 text-slate-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9.75 17L9 20l-1 1h8l-1-1-.75-3M3 13h18M5 17h14a2 2 0 002-2V5a2 2 0 00-2-2H5a2 2 0 00-2 2v10a2 2 0 002 2z" />
            </svg>
            Your hardware and OS setup for development
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
              className={selectClass}
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
              className={selectClass}
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
          className="flex-1 relative py-4 px-6 bg-gradient-to-r from-pana-600 to-pana-700 text-white font-bold rounded-xl shadow-lg shadow-pana-500/30 hover:shadow-xl hover:shadow-pana-500/40 hover:scale-[1.02] focus:outline-none focus:ring-4 focus:ring-pana-500/50 transition-all duration-200 disabled:opacity-50 disabled:cursor-not-allowed disabled:hover:scale-100 overflow-hidden group"
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
              <div className="absolute inset-0 bg-gradient-to-r from-pana-700 to-pana-800 opacity-0 group-hover:opacity-100 transition-opacity" />
            </>
          )}
        </button>
      </div>

      {/* Custom styles for third-party components */}
      <style jsx global>{`
        /* Phone Input Styling */
        .PhoneInput {
          display: flex;
          align-items: center;
        }
        .PhoneInputCountry {
          margin-right: 0.75rem;
        }
        .PhoneInputCountrySelect {
          padding: 0.875rem;
          border: 2px solid #e2e8f0;
          border-radius: 0.75rem;
          background: white;
          cursor: pointer;
        }
        .PhoneInputCountrySelect:focus {
          border-color: #1cd98e;
          outline: none;
          box-shadow: 0 0 0 4px rgba(28, 217, 142, 0.1);
        }
        .PhoneInputInput {
          flex: 1;
          padding: 0.875rem 1rem;
          border: 2px solid #e2e8f0;
          border-radius: 0.75rem;
          background: white;
          transition: all 0.2s;
        }
        .PhoneInputInput:hover {
          border-color: #cbd5e1;
        }
        .PhoneInputInput:focus {
          border-color: #1cd98e;
          outline: none;
          box-shadow: 0 0 0 4px rgba(28, 217, 142, 0.1);
        }

        /* Country Select Styling */
        .country-select-container {
          position: relative;
        }
        .country-select-container input {
          width: 100%;
          padding: 0.875rem 1rem;
          border: 2px solid #e2e8f0;
          border-radius: 0.75rem;
          background: white;
          transition: all 0.2s;
          cursor: pointer;
        }
        .country-select-container input:hover {
          border-color: #cbd5e1;
        }
        .country-select-container input:focus {
          border-color: #1cd98e;
          outline: none;
          box-shadow: 0 0 0 4px rgba(28, 217, 142, 0.1);
        }
        .country-select-container ul {
          position: absolute;
          top: 100%;
          left: 0;
          right: 0;
          max-height: 200px;
          overflow-y: auto;
          background: white;
          border: 2px solid #e2e8f0;
          border-radius: 0.75rem;
          margin-top: 0.25rem;
          z-index: 50;
          box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.1);
        }
        .country-select-container li {
          padding: 0.75rem 1rem;
          cursor: pointer;
          transition: background 0.15s;
        }
        .country-select-container li:hover {
          background: #f0fdf4;
        }
      `}</style>
      </form>
    </>
  );
}
