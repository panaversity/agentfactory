export default function AuthLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <div className="min-h-screen flex items-center justify-center py-12 px-4 sm:px-6 lg:px-8">
      <div className="max-w-md w-full">
        <div className="text-center mb-8">
          <h1 className="text-3xl font-bold text-gray-900">RoboLearn</h1>
          <p className="mt-2 text-sm text-gray-600">
            Physical AI & Humanoid Robotics
          </p>
        </div>
        <div className="bg-white py-8 px-6 shadow-lg rounded-xl">
          {children}
        </div>
      </div>
    </div>
  );
}
