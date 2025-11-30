"use client";

import { useState } from "react";
import { useRouter, useSearchParams } from "next/navigation";
import { signUp } from "@/lib/auth-client";
import { BackgroundSelect } from "./background-select";
import { HardwareTierSelect } from "./hardware-tier-select";
import { SoftwareBackground, HardwareTier } from "@/lib/db/schema";

interface FormErrors {
  email?: string;
  password?: string;
  confirmPassword?: string;
  background?: string;
  hardwareTier?: string;
  general?: string;
}

type Step = 1 | 2;

export function SignUpForm() {
  const router = useRouter();
  const searchParams = useSearchParams();
  const [currentStep, setCurrentStep] = useState<Step>(1);
  const [isLoading, setIsLoading] = useState(false);
  const [errors, setErrors] = useState<FormErrors>({});
  const [focusedField, setFocusedField] = useState<string | null>(null);

  // Check for OAuth parameters or redirect param
  const clientId = searchParams.get("client_id");
  const redirectUri = searchParams.get("redirect_uri");
  const responseType = searchParams.get("response_type");
  const scope = searchParams.get("scope");
  const state = searchParams.get("state");
  const codeChallenge = searchParams.get("code_challenge");
  const codeChallengeMethod = searchParams.get("code_challenge_method");
  const redirectParam = searchParams.get("redirect");

  const [formData, setFormData] = useState({
    email: "",
    password: "",
    confirmPassword: "",
    name: "",
    softwareBackground: "beginner" as SoftwareBackground,
    hardwareTier: "" as HardwareTier | "",
  });

  const validateStep1 = (): boolean => {
    const newErrors: FormErrors = {};
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    
    if (!formData.email) {
      newErrors.email = "Email is required";
    } else if (!emailRegex.test(formData.email)) {
      newErrors.email = "Please enter a valid email address";
    }

    if (!formData.password) {
      newErrors.password = "Password is required";
    } else if (formData.password.length < 8) {
      newErrors.password = "Password must be at least 8 characters";
    }

    if (formData.password !== formData.confirmPassword) {
      newErrors.confirmPassword = "Passwords do not match";
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const validateStep2 = (): boolean => {
    const newErrors: FormErrors = {};
    
    if (!formData.hardwareTier) {
      newErrors.hardwareTier = "Please select your hardware tier";
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleStep1Next = (e: React.FormEvent) => {
    e.preventDefault();
    if (validateStep1()) {
      setCurrentStep(2);
      setErrors({});
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validateStep2()) return;

    setIsLoading(true);
    setErrors({});

    try {
      const isOAuthFlow = clientId && redirectUri && responseType;
      if (isOAuthFlow) {
        localStorage.setItem("oauth_pending_params", JSON.stringify({
          client_id: clientId,
          redirect_uri: redirectUri,
          response_type: responseType,
          scope: scope || undefined,
          state: state || undefined,
          code_challenge: codeChallenge || undefined,
          code_challenge_method: codeChallengeMethod || undefined,
        }));
      }

      const result = await signUp.email({
        email: formData.email,
        password: formData.password,
        name: formData.name || "",
        callbackURL: isOAuthFlow ? "/auth/verify-callback" : "/",
      });

      if (result.error) {
        if (result.error.message?.includes("already exists") || result.error.message?.includes("already registered")) {
          setErrors({ 
            email: "This email is already registered. Try signing in instead.",
            general: "An account with this email already exists. Please sign in instead."
          });
          setCurrentStep(1);
        } else {
          setErrors({ general: result.error.message || "Registration failed. Please try again." });
        }
        setIsLoading(false);
        return;
      }

      // Create profile directly during signup (simpler and more deterministic)
      if (result.data?.user?.id) {
        try {
          const profileResponse = await fetch("/api/profile", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
              userId: result.data.user.id,
              softwareBackground: formData.softwareBackground,
              hardwareTier: formData.hardwareTier,
            }),
          });

          if (!profileResponse.ok) {
            const errorData = await profileResponse.json().catch(() => ({}));
            console.error("Failed to create profile:", errorData);
            // Don't fail signup if profile creation fails - user can update later
          }
        } catch (err) {
          console.error("Error creating profile:", err);
          // Don't fail signup if profile creation fails - user can update later
        }
      }

      if (clientId && redirectUri && responseType) {
        const oauthParams = new URLSearchParams({
          client_id: clientId,
          redirect_uri: redirectUri,
          response_type: responseType,
          ...(scope && { scope }),
          ...(state && { state }),
          ...(codeChallenge && { code_challenge: codeChallenge }),
          ...(codeChallengeMethod && { code_challenge_method: codeChallengeMethod }),
        });
        window.location.href = `/api/auth/oauth2/authorize?${oauthParams.toString()}`;
        return;
      }

      if (redirectParam) {
        window.location.href = redirectParam;
        return;
      }

      window.location.href = "/";
    } catch (error) {
      setErrors({ general: "An unexpected error occurred. Please try again." });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="w-full">
      {/* Progress indicator */}
      <div className="mb-8">
        <div className="flex items-center justify-between mb-3">
          <div className={`flex-1 h-1.5 rounded-full transition-all duration-500 ${
            currentStep >= 1 ? "bg-gradient-to-r from-indigo-600 to-indigo-500" : "bg-slate-200"
          }`} />
          <div className={`mx-3 w-10 h-10 rounded-full flex items-center justify-center text-sm font-bold transition-all duration-500 shadow-lg ${
            currentStep >= 1 
              ? "bg-gradient-to-br from-indigo-600 to-indigo-700 text-white scale-110 shadow-indigo-500/50" 
              : "bg-slate-200 text-slate-400"
          }`}>
            1
          </div>
          <div className={`flex-1 h-1.5 rounded-full transition-all duration-500 ${
            currentStep >= 2 ? "bg-gradient-to-r from-indigo-600 to-indigo-500" : "bg-slate-200"
          }`} />
          <div className={`mx-3 w-10 h-10 rounded-full flex items-center justify-center text-sm font-bold transition-all duration-500 shadow-lg ${
            currentStep >= 2 
              ? "bg-gradient-to-br from-indigo-600 to-indigo-700 text-white scale-110 shadow-indigo-500/50" 
              : "bg-slate-200 text-slate-400"
          }`}>
            2
          </div>
          <div className={`flex-1 h-1.5 rounded-full transition-all duration-500 ${
            currentStep >= 2 ? "bg-gradient-to-r from-indigo-600 to-indigo-500" : "bg-slate-200"
          }`} />
        </div>
        <div className="flex justify-between text-xs font-semibold mt-2">
          <span className={`transition-colors duration-300 ${
            currentStep === 1 ? "text-indigo-600" : "text-slate-400"
          }`}>
            Account
          </span>
          <span className={`transition-colors duration-300 ${
            currentStep === 2 ? "text-indigo-600" : "text-slate-400"
          }`}>
            Background
          </span>
        </div>
      </div>

      <form onSubmit={currentStep === 1 ? handleStep1Next : handleSubmit} className="space-y-6">
        {errors.general && (
          <div className="p-4 bg-red-50 border-l-4 border-red-500 rounded-r-xl animate-in slide-in-from-top">
            <p className="text-sm font-semibold text-red-800">{errors.general}</p>
          </div>
        )}

        {/* Step 1: Account Creation */}
        <div className={`transition-all duration-500 ease-in-out ${
          currentStep === 1 
            ? "opacity-100 translate-x-0 pointer-events-auto" 
            : "opacity-0 absolute translate-x-full pointer-events-none"
        }`}>
          <div className="mb-6">
            <h2 className="text-2xl font-semibold text-slate-900 mb-2">Create your account</h2>
            <p className="text-sm text-slate-600">Get started with RoboLearn in seconds</p>
          </div>

          <div className="space-y-1.5">
            <label htmlFor="name" className="block text-sm font-semibold text-slate-700">
              Name <span className="text-slate-400 font-normal text-xs">(optional)</span>
            </label>
            <input
              id="name"
              type="text"
              value={formData.name}
              onChange={(e) => setFormData({ ...formData, name: e.target.value })}
              onFocus={() => setFocusedField("name")}
              onBlur={() => setFocusedField(null)}
              className={`w-full px-4 py-3 border rounded-xl transition-all duration-200 bg-white/50 backdrop-blur-sm ${
                focusedField === "name"
                  ? "border-indigo-400 focus:border-indigo-500 focus:ring-2 focus:ring-indigo-500/20 shadow-sm shadow-indigo-500/10"
                  : "border-slate-200 focus:border-indigo-400 focus:ring-2 focus:ring-indigo-500/20"
              }`}
              placeholder="Your name"
            />
          </div>

          <div className="space-y-1.5">
            <label htmlFor="email" className="block text-sm font-semibold text-slate-700">
              Email address <span className="text-red-500">*</span>
            </label>
            <div className="relative">
              <input
                id="email"
                type="email"
                required
                value={formData.email}
                onChange={(e) => setFormData({ ...formData, email: e.target.value })}
                onFocus={() => setFocusedField("email")}
                onBlur={() => setFocusedField(null)}
                className={`w-full px-4 py-3 border rounded-xl transition-all duration-200 bg-white/50 backdrop-blur-sm ${
                  errors.email 
                    ? "border-red-300 focus:border-red-500 focus:ring-2 focus:ring-red-500/20" 
                    : focusedField === "email"
                    ? "border-indigo-400 focus:border-indigo-500 focus:ring-2 focus:ring-indigo-500/20 shadow-sm shadow-indigo-500/10"
                    : "border-slate-200 focus:border-indigo-400 focus:ring-2 focus:ring-indigo-500/20"
                }`}
                placeholder="you@example.com"
              />
              {focusedField === "email" && (
                <div className="absolute inset-0 rounded-xl border-2 border-indigo-500 pointer-events-none animate-in scale-in opacity-50" />
              )}
            </div>
            {errors.email && (
              <p className="text-sm text-red-600 animate-in slide-in-from-top">{errors.email}</p>
            )}
          </div>

          <div className="space-y-1.5">
            <label htmlFor="password" className="block text-sm font-semibold text-slate-700">
              Password <span className="text-red-500">*</span>
            </label>
            <div className="relative">
              <input
                id="password"
                type="password"
                required
                value={formData.password}
                onChange={(e) => setFormData({ ...formData, password: e.target.value })}
                onFocus={() => setFocusedField("password")}
                onBlur={() => setFocusedField(null)}
                className={`w-full px-4 py-3 border rounded-xl transition-all duration-200 bg-white/50 backdrop-blur-sm ${
                  errors.password 
                    ? "border-red-300 focus:border-red-500 focus:ring-2 focus:ring-red-500/20" 
                    : focusedField === "password"
                    ? "border-indigo-400 focus:border-indigo-500 focus:ring-2 focus:ring-indigo-500/20 shadow-sm shadow-indigo-500/10"
                    : "border-slate-200 focus:border-indigo-400 focus:ring-2 focus:ring-indigo-500/20"
                }`}
                placeholder="At least 8 characters"
              />
              {focusedField === "password" && (
                <div className="absolute inset-0 rounded-xl border-2 border-indigo-500 pointer-events-none animate-in scale-in opacity-50" />
              )}
            </div>
            {errors.password && (
              <p className="text-sm text-red-600 animate-in slide-in-from-top">{errors.password}</p>
            )}
          </div>

          <div className="space-y-1.5">
            <label htmlFor="confirmPassword" className="block text-sm font-semibold text-slate-700">
              Confirm Password <span className="text-red-500">*</span>
            </label>
            <div className="relative">
              <input
                id="confirmPassword"
                type="password"
                required
                value={formData.confirmPassword}
                onChange={(e) => setFormData({ ...formData, confirmPassword: e.target.value })}
                onFocus={() => setFocusedField("confirmPassword")}
                onBlur={() => setFocusedField(null)}
                className={`w-full px-4 py-3 border rounded-xl transition-all duration-200 bg-white/50 backdrop-blur-sm ${
                  errors.confirmPassword 
                    ? "border-red-300 focus:border-red-500 focus:ring-2 focus:ring-red-500/20" 
                    : focusedField === "confirmPassword"
                    ? "border-indigo-400 focus:border-indigo-500 focus:ring-2 focus:ring-indigo-500/20 shadow-sm shadow-indigo-500/10"
                    : "border-slate-200 focus:border-indigo-400 focus:ring-2 focus:ring-indigo-500/20"
                }`}
                placeholder="Confirm your password"
              />
              {focusedField === "confirmPassword" && (
                <div className="absolute inset-0 rounded-xl border-2 border-indigo-500 pointer-events-none animate-in scale-in opacity-50" />
              )}
            </div>
            {errors.confirmPassword && (
              <p className="text-sm text-red-600 animate-in slide-in-from-top">{errors.confirmPassword}</p>
            )}
          </div>

          <button
            type="submit"
            className="w-full relative py-3.5 px-4 mt-6 bg-gradient-to-r from-indigo-600 to-indigo-700 text-white font-semibold rounded-xl shadow-lg shadow-indigo-500/30 hover:shadow-xl hover:shadow-indigo-500/40 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-indigo-500 transition-all duration-200 overflow-hidden group"
          >
            <span className="flex items-center justify-center">
              Continue
              <svg className="ml-2 w-5 h-5 group-hover:translate-x-1 transition-transform" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5l7 7-7 7" />
              </svg>
            </span>
            <div className="absolute inset-0 shimmer opacity-0 group-hover:opacity-100 transition-opacity" />
          </button>
        </div>

        {/* Step 2: Background Questions */}
        <div className={`transition-all duration-500 ease-in-out ${
          currentStep === 2 
            ? "opacity-100 translate-x-0 pointer-events-auto" 
            : "opacity-0 absolute translate-x-full pointer-events-none"
        }`}>
          <div className="mb-6">
            <button
              type="button"
              onClick={() => {
                setCurrentStep(1);
                setErrors({});
              }}
              className="flex items-center text-sm font-medium text-slate-600 hover:text-slate-900 mb-4 transition-colors group"
            >
              <svg className="w-4 h-4 mr-1.5 group-hover:-translate-x-1 transition-transform" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 19l-7-7 7-7" />
              </svg>
              Back
            </button>
            <h2 className="text-2xl font-semibold text-slate-900 mb-2">Tell us about yourself</h2>
            <p className="text-sm text-slate-600">Help us personalize your learning experience</p>
          </div>

          <BackgroundSelect
            value={formData.softwareBackground}
            onChange={(value) => setFormData({ ...formData, softwareBackground: value })}
            error={errors.background}
          />

          <HardwareTierSelect
            value={formData.hardwareTier}
            onChange={(value) => setFormData({ ...formData, hardwareTier: value })}
            error={errors.hardwareTier}
          />

          <button
            type="submit"
            disabled={isLoading}
            className="w-full relative py-3.5 px-4 mt-6 bg-gradient-to-r from-indigo-600 to-indigo-700 text-white font-semibold rounded-xl shadow-lg shadow-indigo-500/30 hover:shadow-xl hover:shadow-indigo-500/40 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-indigo-500 transition-all duration-200 disabled:opacity-50 disabled:cursor-not-allowed overflow-hidden group"
          >
            {isLoading ? (
              <span className="flex items-center justify-center">
                <svg className="animate-spin -ml-1 mr-3 h-5 w-5 text-white" fill="none" viewBox="0 0 24 24">
                  <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" />
                  <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z" />
                </svg>
                Creating account...
              </span>
            ) : (
              <>
                <span className="relative z-10">Create account</span>
                <div className="absolute inset-0 shimmer opacity-0 group-hover:opacity-100 transition-opacity" />
              </>
            )}
          </button>
        </div>

        <div className="relative pt-6">
          <div className="absolute inset-0 flex items-center">
            <div className="w-full border-t border-slate-200" />
          </div>
          <div className="relative flex justify-center text-sm">
            <span className="px-4 bg-white text-slate-500">Already have an account?</span>
          </div>
        </div>

        <a
          href={`/auth/sign-in${searchParams.toString() ? `?${searchParams.toString()}` : ""}`}
          className="block w-full text-center py-3 px-4 border-2 border-slate-200 text-slate-700 font-semibold rounded-xl hover:border-indigo-300 hover:text-indigo-700 hover:bg-indigo-50/50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-indigo-500 transition-all duration-200"
        >
          Sign in
        </a>
      </form>
    </div>
  );
}
