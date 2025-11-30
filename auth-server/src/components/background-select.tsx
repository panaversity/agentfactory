"use client";

import { SoftwareBackground } from "@/lib/db/schema";

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
      <label className="block text-sm font-medium text-gray-700">
        What's your software background?
      </label>
      <div className="space-y-2">
        {backgrounds.map((bg) => (
          <label
            key={bg.value}
            className={`flex items-start p-3 border rounded-lg cursor-pointer transition-colors ${
              value === bg.value
                ? "border-blue-500 bg-blue-50"
                : "border-gray-200 hover:border-gray-300"
            }`}
          >
            <input
              type="radio"
              name="softwareBackground"
              value={bg.value}
              checked={value === bg.value}
              onChange={(e) => onChange(e.target.value as SoftwareBackground)}
              className="mt-1 h-4 w-4 text-blue-600 border-gray-300 focus:ring-blue-500"
            />
            <div className="ml-3">
              <span className="block text-sm font-medium text-gray-900">
                {bg.label}
              </span>
              <span className="block text-sm text-gray-500">
                {bg.description}
              </span>
            </div>
          </label>
        ))}
      </div>
      {error && <p className="text-sm text-red-600">{error}</p>}
    </div>
  );
}
