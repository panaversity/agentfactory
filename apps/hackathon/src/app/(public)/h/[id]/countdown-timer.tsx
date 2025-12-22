"use client";

import { useEffect, useState } from "react";

interface CountdownTimerProps {
  targetDate: string;
}

interface TimeLeft {
  days: number;
  hours: number;
  minutes: number;
  seconds: number;
}

export function CountdownTimer({ targetDate }: CountdownTimerProps) {
  const [timeLeft, setTimeLeft] = useState<TimeLeft>({
    days: 0,
    hours: 0,
    minutes: 0,
    seconds: 0,
  });
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);

    const calculateTimeLeft = () => {
      const difference = new Date(targetDate).getTime() - new Date().getTime();

      if (difference > 0) {
        setTimeLeft({
          days: Math.floor(difference / (1000 * 60 * 60 * 24)),
          hours: Math.floor((difference / (1000 * 60 * 60)) % 24),
          minutes: Math.floor((difference / 1000 / 60) % 60),
          seconds: Math.floor((difference / 1000) % 60),
        });
      } else {
        setTimeLeft({ days: 0, hours: 0, minutes: 0, seconds: 0 });
      }
    };

    calculateTimeLeft();
    const timer = setInterval(calculateTimeLeft, 1000);

    return () => clearInterval(timer);
  }, [targetDate]);

  if (!mounted) {
    return (
      <div className="flex gap-4">
        {["Days", "Hours", "Minutes", "Seconds"].map((label) => (
          <div key={label} className="text-center">
            <div className="relative">
              <div className="w-20 h-24 md:w-28 md:h-32 rounded-2xl bg-white/[0.03] border border-white/10 flex items-center justify-center">
                <span className="text-4xl md:text-5xl font-bold text-white tabular-nums">
                  --
                </span>
              </div>
            </div>
            <p className="text-xs md:text-sm text-white/40 mt-2 uppercase tracking-widest">
              {label}
            </p>
          </div>
        ))}
      </div>
    );
  }

  return (
    <div className="flex gap-3 md:gap-4">
      <TimeUnit value={timeLeft.days} label="Days" />
      <div className="flex items-center text-white/20 text-3xl font-light self-start mt-8">
        :
      </div>
      <TimeUnit value={timeLeft.hours} label="Hours" />
      <div className="flex items-center text-white/20 text-3xl font-light self-start mt-8">
        :
      </div>
      <TimeUnit value={timeLeft.minutes} label="Minutes" />
      <div className="flex items-center text-white/20 text-3xl font-light self-start mt-8">
        :
      </div>
      <TimeUnit value={timeLeft.seconds} label="Seconds" pulse />
    </div>
  );
}

function TimeUnit({
  value,
  label,
  pulse,
}: {
  value: number;
  label: string;
  pulse?: boolean;
}) {
  return (
    <div className="text-center">
      <div className="relative group">
        {/* Glow effect */}
        <div
          className={`absolute inset-0 bg-[#1cd98e]/20 rounded-2xl blur-xl transition-opacity ${
            pulse ? "animate-pulse opacity-50" : "opacity-0 group-hover:opacity-30"
          }`}
        />
        {/* Card */}
        <div
          className={`relative w-16 h-20 md:w-24 md:h-28 rounded-2xl bg-gradient-to-b from-white/[0.08] to-white/[0.02] border border-white/10 flex items-center justify-center overflow-hidden ${
            pulse ? "border-[#1cd98e]/30" : ""
          }`}
        >
          {/* Inner shine */}
          <div className="absolute inset-0 bg-gradient-to-br from-white/5 to-transparent" />
          {/* Value */}
          <span
            className={`relative text-3xl md:text-5xl font-bold tabular-nums transition-colors ${
              pulse ? "text-[#1cd98e]" : "text-white"
            }`}
          >
            {value.toString().padStart(2, "0")}
          </span>
        </div>
      </div>
      <p className="text-[10px] md:text-xs text-white/40 mt-2 uppercase tracking-[0.2em]">
        {label}
      </p>
    </div>
  );
}
