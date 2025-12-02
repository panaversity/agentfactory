/** @type {import('tailwindcss').Config} */
module.exports = {
    content: [
      "./src/**/*.{js,jsx,ts,tsx}",
      "./docs/**/*.{md,mdx}",
    ],
    darkMode: ['class', '[data-theme="dark"]'],
    theme: {
      extend: {},
    },
    corePlugins: {
      preflight: false,
    },
    plugins: [],
  }
  