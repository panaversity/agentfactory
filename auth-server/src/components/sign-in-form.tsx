"use client";

import { useState } from "react";
import { useRouter, useSearchParams } from "next/navigation";
import { signIn } from "@/lib/auth-client";

interface FormErrors {
  email?: string;
  password?: string;
  general?: string;
  needsVerification?: boolean;
}

// Eye icons for password visibility toggle
const EyeIcon = () => (
  <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z" />
  </svg>
);

const EyeOffIcon = () => (
  <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13.875 18.825A10.05 10.05 0 0112 19c-4.478 0-8.268-2.943-9.543-7a9.97 9.97 0 011.563-3.029m5.858.908a3 3 0 114.243 4.243M9.878 9.878l4.242 4.242M9.88 9.88l-3.29-3.29m7.532 7.532l3.29 3.29M3 3l3.59 3.59m0 0A9.953 9.953 0 0112 5c4.478 0 8.268 2.943 9.543 7a10.025 10.025 0 01-4.132 5.411m0 0L21 21" />
  </svg>
);

export function SignInForm() {
  const router = useRouter();
  const searchParams = useSearchParams();
  const [isLoading, setIsLoading] = useState(false);
  const [errors, setErrors] = useState<FormErrors>({});
  const [focusedField, setFocusedField] = useState<string | null>(null);
  const [showPassword, setShowPassword] = useState(false);

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
  });

  const validateForm = (): boolean => {
    const newErrors: FormErrors = {};

    if (!formData.email) {
      newErrors.email = "Email is required";
    }

    if (!formData.password) {
      newErrors.password = "Password is required";
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validateForm()) return;

    setIsLoading(true);
    setErrors({});

    try {
      const result = await signIn.email({
        email: formData.email,
        password: formData.password,
        callbackURL: "/",
      });

      if (result.error) {
        console.error("[SignIn] Error:", result.error.status, result.error.message);
        
        if (result.error.status === 403 || result.error.message?.toLowerCase().includes("verify") || result.error.message?.toLowerCase().includes("verification") || result.error.message?.toLowerCase().includes("not verified")) {
          setErrors({
            general: "Please verify your email address before signing in. Check your inbox for the verification email.",
            needsVerification: true,
          });
        } else if (result.error.message?.toLowerCase().includes("password") || result.error.message?.toLowerCase().includes("incorrect") || result.error.message?.toLowerCase().includes("invalid password")) {
          setErrors({ general: "Invalid password. Please check your password and try again." });
        } else if (result.error.message?.toLowerCase().includes("not found") || result.error.message?.toLowerCase().includes("does not exist") || result.error.message?.toLowerCase().includes("no account")) {
          setErrors({ general: "No account found with this email. Please sign up first." });
        } else {
          const errorMsg = result.error.message || "Invalid credentials. Please check your email and password.";
          setErrors({ general: errorMsg });
        }
        setIsLoading(false);
        return;
      }

      // Check if this is part of an OAuth flow
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

      // Check if we have a redirect param
      if (redirectParam) {
        window.location.href = redirectParam;
        return;
      }

      // Stay on auth server after direct login
      window.location.href = "/";
    } catch (error) {
      setErrors({ general: "An unexpected error occurred. Please try again." });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="w-full">
      <div className="mb-8">
        <h2 className="text-2xl font-semibold text-slate-900 mb-2">Welcome back</h2>
        <p className="text-sm text-slate-600">Sign in to your account</p>
      </div>

      <form onSubmit={handleSubmit} className="space-y-5">
        {errors.general && (
          <div className={`p-4 rounded-xl border-l-4 animate-in slide-in-from-top ${
            errors.needsVerification 
              ? "bg-amber-50 border-amber-400 text-amber-800" 
              : "bg-red-50 border-red-400 text-red-800"
          }`}>
            <p className="text-sm font-medium">{errors.general}</p>
            {errors.needsVerification && (
              <a
                href={`/auth/resend-verification?email=${encodeURIComponent(formData.email)}${searchParams.toString() ? `&${searchParams.toString()}` : ""}`}
                className="mt-2 inline-block text-sm font-semibold text-amber-700 hover:text-amber-800 underline underline-offset-2 transition-colors"
              >
                Resend verification email
              </a>
            )}
          </div>
        )}

        <div className="space-y-3">
          <label htmlFor="email" className="block text-sm font-semibold text-slate-700">
            Email address
          </label>
          <div className="relative">
            <input
              id="email"
              type="email"
              required
              autoComplete="email"
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

        <div className="space-y-3">
          <label htmlFor="password" className="block text-sm font-semibold text-slate-700">
            Password
          </label>
          <div className="relative">
            <input
              id="password"
              type={showPassword ? "text" : "password"}
              required
              autoComplete="current-password"
              value={formData.password}
              onChange={(e) => setFormData({ ...formData, password: e.target.value })}
              onFocus={() => setFocusedField("password")}
              onBlur={() => setFocusedField(null)}
              className={`w-full px-4 py-3 pr-12 border rounded-xl transition-all duration-200 bg-white/50 backdrop-blur-sm ${
                errors.password
                  ? "border-red-300 focus:border-red-500 focus:ring-2 focus:ring-red-500/20"
                  : focusedField === "password"
                  ? "border-indigo-400 focus:border-indigo-500 focus:ring-2 focus:ring-indigo-500/20 shadow-sm shadow-indigo-500/10"
                  : "border-slate-200 focus:border-indigo-400 focus:ring-2 focus:ring-indigo-500/20"
              }`}
              placeholder="Enter your password"
            />
            <button
              type="button"
              onClick={() => setShowPassword(!showPassword)}
              className="absolute right-3 top-1/2 -translate-y-1/2 text-slate-400 hover:text-slate-600 transition-colors p-1"
              tabIndex={-1}
              aria-label={showPassword ? "Hide password" : "Show password"}
            >
              {showPassword ? <EyeOffIcon /> : <EyeIcon />}
            </button>
            {focusedField === "password" && (
              <div className="absolute inset-0 rounded-xl border-2 border-indigo-500 pointer-events-none animate-in scale-in opacity-50" />
            )}
          </div>
          {errors.password && (
            <p className="text-sm text-red-600 animate-in slide-in-from-top">{errors.password}</p>
          )}
        </div>

        <div className="flex items-center justify-between pt-1">
          <a 
            href="/auth/forgot-password" 
            className="text-sm font-medium text-indigo-600 hover:text-indigo-700 transition-colors"
          >
            Forgot password?
          </a>
        </div>

        <button
          type="submit"
          disabled={isLoading}
          className="w-full relative py-3.5 px-4 bg-gradient-to-r from-indigo-600 to-indigo-700 text-white font-semibold rounded-xl shadow-lg shadow-indigo-500/30 hover:shadow-xl hover:shadow-indigo-500/40 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-indigo-500 transition-all duration-200 disabled:opacity-50 disabled:cursor-not-allowed disabled:hover:shadow-lg overflow-hidden group"
        >
          {isLoading ? (
            <span className="flex items-center justify-center">
              <svg className="animate-spin -ml-1 mr-3 h-5 w-5 text-white" fill="none" viewBox="0 0 24 24">
                <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" />
                <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z" />
              </svg>
              Signing in...
            </span>
          ) : (
            <>
              <span className="relative z-10">Sign in</span>
              <div className="absolute inset-0 shimmer opacity-0 group-hover:opacity-100 transition-opacity" />
            </>
          )}
        </button>

        <div className="relative pt-6">
          <div className="absolute inset-0 flex items-center">
            <div className="w-full border-t border-slate-200" />
          </div>
          <div className="relative flex justify-center text-sm">
            <span className="px-4 bg-white text-slate-500">Don't have an account?</span>
          </div>
        </div>

        <a
          href={`/auth/sign-up${searchParams.toString() ? `?${searchParams.toString()}` : ""}`}
          className="block w-full text-center py-3 px-4 border-2 border-slate-200 text-slate-700 font-semibold rounded-xl hover:border-indigo-300 hover:text-indigo-700 hover:bg-indigo-50/50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-indigo-500 transition-all duration-200"
        >
          Create an account
        </a>
      </form>
    </div>
  );
}
