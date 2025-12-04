import type { Config } from "tailwindcss";

const config: Config = {
  darkMode: 'class', // Disable system-preference dark mode, only apply if 'dark' class present
  content: [
    "./src/pages/**/*.{js,ts,jsx,tsx,mdx}",
    "./src/components/**/*.{js,ts,jsx,tsx,mdx}",
    "./src/app/**/*.{js,ts,jsx,tsx,mdx}",
  ],
  theme: {
    extend: {
      colors: {
        // Panaversity Brand
        pana: {
          50: '#edfdf6',
          100: '#d3f9e8',
          200: '#aaf2d4',
          300: '#72e6ba',
          400: '#4eeaab',
          500: '#1cd98e', // Primary
          600: '#15a776',
          700: '#148a63',
          800: '#156d51',
          900: '#145944',
          950: '#053226',
        },
        // Dark green accent (same family as pana)
        accent: {
          50: '#ecfdf5',
          100: '#d1fae5',
          200: '#a7f3d0',
          300: '#6ee7b7',
          400: '#34d399',
          500: '#0d9668', // Dark Emerald
          600: '#065f46', // Darker
          700: '#064e3b',
          800: '#053a2d',
          900: '#042f24',
        },
      },
      fontFamily: {
        sans: ['Plus Jakarta Sans', 'system-ui', 'sans-serif'],
        mono: ['JetBrains Mono', 'monospace'],
      },
      boxShadow: {
        'glow': '0 0 20px 2px rgba(28, 217, 142, 0.2)',
        'glow-lg': '0 0 40px 4px rgba(28, 217, 142, 0.15)',
        'card': '0 4px 24px rgba(0, 0, 0, 0.06)',
        'card-hover': '0 8px 32px rgba(28, 217, 142, 0.1)',
      },
      animation: {
        'fade-in': 'fadeIn 0.3s ease-out',
        'slide-up': 'slideUp 0.4s ease-out',
        'scale-in': 'scaleIn 0.2s ease-out',
        'pulse-soft': 'pulseSoft 2s ease-in-out infinite',
      },
      keyframes: {
        fadeIn: {
          '0%': { opacity: '0' },
          '100%': { opacity: '1' },
        },
        slideUp: {
          '0%': { opacity: '0', transform: 'translateY(10px)' },
          '100%': { opacity: '1', transform: 'translateY(0)' },
        },
        scaleIn: {
          '0%': { opacity: '0', transform: 'scale(0.95)' },
          '100%': { opacity: '1', transform: 'scale(1)' },
        },
        pulseSoft: {
          '0%, 100%': { opacity: '1' },
          '50%': { opacity: '0.7' },
        },
      },
    },
  },
  plugins: [],
};

export default config;
