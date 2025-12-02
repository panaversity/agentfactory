"use client";

import type { SoftwareBackground } from "@/types/profile";

interface BackgroundSelectProps {
  value: SoftwareBackground;
  onChange: (value: SoftwareBackground) => void;
  error?: string;
}

const backgrounds: { value: SoftwareBackground; label: string; description: string }[] = [
  {
    value: "beginner",
    label: "Beginner",
    description: "New to programming or robotics",
  },
  {
    value: "intermediate",
    label: "Intermediate",
    description: "Some experience with Python or similar languages",
  },
  {
    value: "advanced",
    label: "Advanced",
    description: "Professional developer or robotics engineer",
  },
];

export function BackgroundSelect({ value, onChange, error }: BackgroundSelectProps) {
  return (
    <div className="space-y-3">
      <label className="block mt-6 px-4 py-3 bg-gradient-to-r from-indigo-50 to-indigo-50/50 border-l-4 border-indigo-500 rounded-r-lg text-sm font-bold text-indigo-900">
        What's your software background?
      </label>
      <div className="grid grid-cols-3 gap-3">
        {backgrounds.map((bg) => (
          <button
            key={bg.value}
            type="button"
            onClick={() => onChange(bg.value)}
            className={`group relative p-4 border-2 rounded-xl cursor-pointer transition-all duration-200 text-left ${
              value === bg.value
                ? "border-indigo-500 bg-gradient-to-br from-indigo-50 to-indigo-50/50 shadow-md shadow-indigo-500/20"
                : "border-slate-200 hover:border-indigo-200 hover:bg-slate-50/50 bg-white/50 backdrop-blur-sm"
            }`}
          >
            <div className="flex flex-col items-center text-center space-y-2">
              <div className={`w-6 h-6 rounded-full border-2 flex items-center justify-center transition-all duration-200 ${
                value === bg.value
                  ? "border-indigo-600 bg-indigo-600"
                  : "border-slate-300 group-hover:border-indigo-400"
              }`}>
                {value === bg.value && (
                  <div className="w-2.5 h-2.5 rounded-full bg-white animate-in scale-in" />
                )}
              </div>
              <span className={`block text-sm font-semibold transition-colors ${
                value === bg.value ? "text-indigo-900" : "text-slate-900"
              }`}>
                {bg.label}
              </span>
              <span className={`block text-xs transition-colors leading-tight ${
                value === bg.value ? "text-indigo-700" : "text-slate-600"
              }`}>
                {bg.description}
              </span>
            </div>
            {value === bg.value && (
              <div className="absolute top-2 right-2">
                <svg className="w-4 h-4 text-indigo-600 animate-in scale-in" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M5 13l4 4L19 7" />
                </svg>
              </div>
            )}
          </button>
        ))}
      </div>
      {error && (
        <p className="text-sm text-red-600 animate-in slide-in-from-top">{error}</p>
      )}
    </div>
  );
}
