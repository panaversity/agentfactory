"use client";

import { HardwareTier } from "@/lib/db/schema";

interface HardwareTierSelectProps {
  value: HardwareTier | "";
  onChange: (value: HardwareTier) => void;
  error?: string;
}

const hardwareTiers: { value: HardwareTier; label: string; shortLabel: string; description: string }[] = [
  {
    value: "tier1",
    label: "Tier 1: Laptop/Cloud",
    shortLabel: "Laptop/Cloud",
    description: "Browser-based learning",
  },
  {
    value: "tier2",
    label: "Tier 2: RTX GPU Workstation",
    shortLabel: "RTX GPU",
    description: "Local Isaac Sim & Gazebo",
  },
  {
    value: "tier3",
    label: "Tier 3: Jetson Edge Device",
    shortLabel: "Jetson Edge",
    description: "Real sensors & edge deployment",
  },
  {
    value: "tier4",
    label: "Tier 4: Physical Robot",
    shortLabel: "Physical Robot",
    description: "Real-world testing",
  },
];

export function HardwareTierSelect({ value, onChange, error }: HardwareTierSelectProps) {
  return (
    <div className="space-y-3">
      <label className="block mt-6 px-4 py-3 bg-gradient-to-r from-indigo-50 to-indigo-50/50 border-l-4 border-indigo-500 rounded-r-lg text-sm font-bold text-indigo-900">
        What hardware do you have access to?
      </label>
      <div className="grid grid-cols-2 gap-3">
        {hardwareTiers.map((tier) => (
          <button
            key={tier.value}
            type="button"
            onClick={() => onChange(tier.value)}
            className={`group relative p-4 border-2 rounded-xl cursor-pointer transition-all duration-200 text-left ${
              value === tier.value
                ? "border-indigo-500 bg-gradient-to-br from-indigo-50 to-indigo-50/50 shadow-md shadow-indigo-500/20"
                : "border-slate-200 hover:border-indigo-200 hover:bg-slate-50/50 bg-white/50 backdrop-blur-sm"
            }`}
          >
            <div className="flex flex-col space-y-1.5">
              <div className="flex items-center justify-between">
                <div className={`w-5 h-5 rounded-full border-2 flex items-center justify-center transition-all duration-200 ${
                  value === tier.value
                    ? "border-indigo-600 bg-indigo-600"
                    : "border-slate-300 group-hover:border-indigo-400"
                }`}>
                  {value === tier.value && (
                    <div className="w-2 h-2 rounded-full bg-white animate-in scale-in" />
                  )}
                </div>
                {value === tier.value && (
                  <svg className="w-4 h-4 text-indigo-600 animate-in scale-in" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M5 13l4 4L19 7" />
                  </svg>
                )}
              </div>
              <span className={`block text-sm font-semibold transition-colors ${
                value === tier.value ? "text-indigo-900" : "text-slate-900"
              }`}>
                {tier.shortLabel}
              </span>
              <span className={`block text-xs transition-colors leading-tight ${
                value === tier.value ? "text-indigo-700" : "text-slate-600"
              }`}>
                {tier.description}
              </span>
            </div>
          </button>
        ))}
      </div>
      {error && (
        <p className="text-sm text-red-600 animate-in slide-in-from-top">{error}</p>
      )}
    </div>
  );
}
