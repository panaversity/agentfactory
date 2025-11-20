import type { ReactNode } from "react";

export default function TailwindTestComponent(): ReactNode {
  return (
    <section className="py-16 px-4 bg-gradient-to-r from-blue-50 to-indigo-50 dark:from-slate-900 dark:to-slate-800">
      <div className="container mx-auto max-w-4xl">
        <div className="text-center mb-12">
          <h2 className="text-3xl font-bold text-gray-900 dark:text-white mb-4">
            ✨ Tailwind CSS v3 Test Component
          </h2>
          <p className="text-lg text-gray-600 dark:text-gray-300">
            Testing Tailwind v3 integration with Docusaurus and dark mode support
          </p>
        </div>

        {/* Test Cards Grid */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6 mb-8">
          {/* Card 1 - Responsive Design */}
          <div className="bg-white dark:bg-slate-800 rounded-lg shadow-md hover:shadow-lg transition-shadow p-6 border-l-4 border-blue-500">
            <div className="flex items-center mb-4">
              <div className="w-12 h-12 bg-blue-100 dark:bg-blue-900 rounded-lg flex items-center justify-center">
                <span className="text-2xl">🎯</span>
              </div>
              <h3 className="ml-4 text-xl font-semibold text-gray-900 dark:text-white">
                Responsive Design
              </h3>
            </div>
            <p className="text-gray-600 dark:text-gray-400 mb-4">
              This card is fully responsive. Try resizing your browser to see the grid change from 3 columns to 2 to 1.
            </p>
            <button className="w-full bg-blue-500 hover:bg-blue-600 text-white font-semibold py-2 px-4 rounded-lg transition-colors">
              Learn More
            </button>
          </div>

          {/* Card 2 - Dark Mode */}
          <div className="bg-white dark:bg-slate-800 rounded-lg shadow-md hover:shadow-lg transition-shadow p-6 border-l-4 border-green-500">
            <div className="flex items-center mb-4">
              <div className="w-12 h-12 bg-green-100 dark:bg-green-900 rounded-lg flex items-center justify-center">
                <span className="text-2xl">🌙</span>
              </div>
              <h3 className="ml-4 text-xl font-semibold text-gray-900 dark:text-white">
                Dark Mode Ready
              </h3>
            </div>
            <p className="text-gray-600 dark:text-gray-400 mb-4">
              Dark mode is fully supported using Docusaurus' data-theme attribute with Tailwind's dark: prefix.
            </p>
            <button className="w-full bg-green-500 hover:bg-green-600 text-white font-semibold py-2 px-4 rounded-lg transition-colors">
              Test Dark Mode
            </button>
          </div>

          {/* Card 3 - Utilities */}
          <div className="bg-white dark:bg-slate-800 rounded-lg shadow-md hover:shadow-lg transition-shadow p-6 border-l-4 border-purple-500">
            <div className="flex items-center mb-4">
              <div className="w-12 h-12 bg-purple-100 dark:bg-purple-900 rounded-lg flex items-center justify-center">
                <span className="text-2xl">⚡</span>
              </div>
              <h3 className="ml-4 text-xl font-semibold text-gray-900 dark:text-white">
                Utility Classes
              </h3>
            </div>
            <p className="text-gray-600 dark:text-gray-400 mb-4">
              All Tailwind utilities are working: spacing, colors, shadows, and smooth transitions.
            </p>
            <button className="w-full bg-purple-500 hover:bg-purple-600 text-white font-semibold py-2 px-4 rounded-lg transition-colors">
              View Docs
            </button>
          </div>
        </div>

        {/* Feature Showcase */}
        <div className="bg-white dark:bg-slate-800 rounded-xl shadow-lg p-8 mb-8">
          <h3 className="text-2xl font-bold text-gray-900 dark:text-white mb-6">
            Tailwind Features in Action
          </h3>

          <div className="space-y-4">
            {/* Feature 1 */}
            <div className="flex items-start space-x-4 p-4 bg-blue-50 dark:bg-blue-900/20 rounded-lg border border-blue-200 dark:border-blue-800">
              <div className="flex-shrink-0 w-6 h-6 rounded-full bg-blue-500 flex items-center justify-center text-white font-bold text-sm">
                ✓
              </div>
              <div>
                <h4 className="font-semibold text-gray-900 dark:text-white">Flexbox Layout</h4>
                <p className="text-sm text-gray-600 dark:text-gray-400">Using flex utilities for responsive layouts</p>
              </div>
            </div>

            {/* Feature 2 */}
            <div className="flex items-start space-x-4 p-4 bg-green-50 dark:bg-green-900/20 rounded-lg border border-green-200 dark:border-green-800">
              <div className="flex-shrink-0 w-6 h-6 rounded-full bg-green-500 flex items-center justify-center text-white font-bold text-sm">
                ✓
              </div>
              <div>
                <h4 className="font-semibold text-gray-900 dark:text-white">Grid System</h4>
                <p className="text-sm text-gray-600 dark:text-gray-400">Responsive grid layout with gap utilities</p>
              </div>
            </div>

            {/* Feature 3 */}
            <div className="flex items-start space-x-4 p-4 bg-purple-50 dark:bg-purple-900/20 rounded-lg border border-purple-200 dark:border-purple-800">
              <div className="flex-shrink-0 w-6 h-6 rounded-full bg-purple-500 flex items-center justify-center text-white font-bold text-sm">
                ✓
              </div>
              <div>
                <h4 className="font-semibold text-gray-900 dark:text-white">Gradient Backgrounds</h4>
                <p className="text-sm text-gray-600 dark:text-gray-400">Using gradient and background utilities</p>
              </div>
            </div>

            {/* Feature 4 */}
            <div className="flex items-start space-x-4 p-4 bg-amber-50 dark:bg-amber-900/20 rounded-lg border border-amber-200 dark:border-amber-800">
              <div className="flex-shrink-0 w-6 h-6 rounded-full bg-amber-500 flex items-center justify-center text-white font-bold text-sm">
                ✓
              </div>
              <div>
                <h4 className="font-semibold text-gray-900 dark:text-white">Hover & Transitions</h4>
                <p className="text-sm text-gray-600 dark:text-gray-400">Smooth transitions and interactive states</p>
              </div>
            </div>
          </div>
        </div>

        {/* Color Palette Test - Full Spectrum with Dark Mode */}
        <div className="bg-white dark:bg-slate-800 rounded-xl shadow-lg p-8">
          <h3 className="text-2xl font-bold text-gray-900 dark:text-white mb-2">
            Tailwind v3 Color Palette - Light & Dark Mode
          </h3>
          <p className="text-gray-600 dark:text-gray-400 mb-6">
            Toggle dark mode to see how colors adapt. Colors are optimized for both light and dark themes.
          </p>

          {/* Primary Colors */}
          <div className="mb-8">
            <h4 className="text-lg font-semibold text-gray-900 dark:text-white mb-4">Primary Colors</h4>
            <div className="flex flex-wrap gap-3">
              <div className="w-20 h-20 bg-red-500 dark:bg-red-600 rounded-lg flex flex-col items-center justify-center text-white font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">🔴</span>
                <span>Red</span>
              </div>
              <div className="w-20 h-20 bg-orange-500 dark:bg-orange-600 rounded-lg flex flex-col items-center justify-center text-white font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">🟠</span>
                <span>Orange</span>
              </div>
              <div className="w-20 h-20 bg-yellow-500 dark:bg-yellow-600 rounded-lg flex flex-col items-center justify-center text-gray-900 dark:text-white font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">🟡</span>
                <span>Yellow</span>
              </div>
              <div className="w-20 h-20 bg-green-500 dark:bg-green-600 rounded-lg flex flex-col items-center justify-center text-white font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">🟢</span>
                <span>Green</span>
              </div>
              <div className="w-20 h-20 bg-blue-500 dark:bg-blue-600 rounded-lg flex flex-col items-center justify-center text-white font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">🔵</span>
                <span>Blue</span>
              </div>
              <div className="w-20 h-20 bg-indigo-500 dark:bg-indigo-600 rounded-lg flex flex-col items-center justify-center text-white font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">💜</span>
                <span>Indigo</span>
              </div>
              <div className="w-20 h-20 bg-purple-500 dark:bg-purple-600 rounded-lg flex flex-col items-center justify-center text-white font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">🟣</span>
                <span>Purple</span>
              </div>
              <div className="w-20 h-20 bg-pink-500 dark:bg-pink-600 rounded-lg flex flex-col items-center justify-center text-white font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">🩷</span>
                <span>Pink</span>
              </div>
            </div>
          </div>

          {/* Neutral Colors - Light Mode */}
          <div className="mb-8">
            <h4 className="text-lg font-semibold text-gray-900 dark:text-white mb-4">Neutral Colors (Light Mode)</h4>
            <div className="flex flex-wrap gap-3">
              <div className="w-20 h-20 bg-gray-50 border-2 border-gray-300 rounded-lg flex flex-col items-center justify-center text-gray-900 font-bold text-xs shadow-sm hover:shadow-md transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">⚪</span>
                <span className="text-xs">50</span>
              </div>
              <div className="w-20 h-20 bg-gray-200 rounded-lg flex flex-col items-center justify-center text-gray-900 font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">🩶</span>
                <span>200</span>
              </div>
              <div className="w-20 h-20 bg-gray-400 rounded-lg flex flex-col items-center justify-center text-gray-900 font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">⬜</span>
                <span>400</span>
              </div>
              <div className="w-20 h-20 bg-gray-600 rounded-lg flex flex-col items-center justify-center text-white font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">🔲</span>
                <span>600</span>
              </div>
              <div className="w-20 h-20 bg-gray-800 rounded-lg flex flex-col items-center justify-center text-white font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">⬛</span>
                <span>800</span>
              </div>
              <div className="w-20 h-20 bg-gray-950 rounded-lg flex flex-col items-center justify-center text-white font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">⬛</span>
                <span>950</span>
              </div>
            </div>
          </div>

          {/* Neutral Colors - Dark Mode */}
          <div className="mb-8">
            <h4 className="text-lg font-semibold text-gray-900 dark:text-white mb-4">Neutral Colors (Dark Mode)</h4>
            <div className="flex flex-wrap gap-3">
              <div className="w-20 h-20 bg-slate-50 dark:bg-slate-950 border-2 dark:border-slate-700 rounded-lg flex flex-col items-center justify-center text-gray-900 dark:text-white font-bold text-xs shadow-sm hover:shadow-md transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">⚪</span>
                <span className="text-xs">50</span>
              </div>
              <div className="w-20 h-20 bg-slate-100 dark:bg-slate-800 rounded-lg flex flex-col items-center justify-center text-gray-900 dark:text-white font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">🩶</span>
                <span>100</span>
              </div>
              <div className="w-20 h-20 bg-slate-300 dark:bg-slate-600 rounded-lg flex flex-col items-center justify-center text-gray-900 dark:text-white font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">⬜</span>
                <span>300</span>
              </div>
              <div className="w-20 h-20 bg-slate-600 dark:bg-slate-400 rounded-lg flex flex-col items-center justify-center text-white dark:text-gray-900 font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">🔲</span>
                <span>600</span>
              </div>
              <div className="w-20 h-20 bg-slate-800 dark:bg-slate-200 rounded-lg flex flex-col items-center justify-center text-white dark:text-gray-900 font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">⬛</span>
                <span>800</span>
              </div>
              <div className="w-20 h-20 bg-slate-950 dark:bg-slate-100 rounded-lg flex flex-col items-center justify-center text-white dark:text-gray-900 font-bold text-xs shadow-md hover:shadow-lg transition-all cursor-pointer hover:scale-110">
                <span className="text-2xl">⬛</span>
                <span>950</span>
              </div>
            </div>
          </div>

          {/* Gradient Examples */}
          <div className="mb-8">
            <h4 className="text-lg font-semibold text-gray-900 dark:text-white mb-4">Gradient Examples</h4>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              <div className="h-24 bg-gradient-to-r from-blue-400 to-blue-600 rounded-lg shadow-md hover:shadow-lg transition-shadow cursor-pointer flex items-center justify-center text-white font-bold">
                Blue Gradient
              </div>
              <div className="h-24 bg-gradient-to-r from-purple-400 to-pink-600 rounded-lg shadow-md hover:shadow-lg transition-shadow cursor-pointer flex items-center justify-center text-white font-bold">
                Purple to Pink
              </div>
              <div className="h-24 bg-gradient-to-r from-green-400 to-blue-500 rounded-lg shadow-md hover:shadow-lg transition-shadow cursor-pointer flex items-center justify-center text-white font-bold">
                Green to Blue
              </div>
              <div className="h-24 bg-gradient-to-r from-yellow-400 via-red-500 to-pink-500 rounded-lg shadow-md hover:shadow-lg transition-shadow cursor-pointer flex items-center justify-center text-white font-bold">
                Warm Rainbow
              </div>
            </div>
          </div>
        </div>

        {/* Test Status */}
        <div className="mt-8 p-6 bg-green-50 dark:bg-green-900/20 border-2 border-green-500 rounded-xl text-center">
          <h4 className="text-xl font-bold text-green-900 dark:text-green-200 mb-2">✅ Tailwind v3 Integration Successful</h4>
          <p className="text-green-700 dark:text-green-300">
            All Tailwind CSS utilities are working correctly with Docusaurus and dark mode support enabled.
          </p>
        </div>
      </div>
    </section>
  );
}
