export default function HackathonsLoading() {
  return (
    <div className="relative overflow-hidden min-h-screen">
      {/* Background Effects */}
      <div className="absolute inset-0 overflow-hidden pointer-events-none">
        <div className="absolute top-0 left-1/4 w-[800px] h-[800px] bg-[#1cd98e]/5 rounded-full blur-[120px] -translate-y-1/2" />
      </div>

      {/* Hero Skeleton */}
      <section className="relative pt-16 pb-12 px-4 sm:px-6 lg:px-8">
        <div className="mx-auto max-w-7xl">
          <div className="text-center max-w-3xl mx-auto">
            <div className="h-10 w-64 rounded-full bg-white/5 animate-pulse mx-auto mb-6" />
            <div className="h-16 w-3/4 rounded-lg bg-white/5 animate-pulse mx-auto mb-4" />
            <div className="h-6 w-2/3 rounded bg-white/5 animate-pulse mx-auto mb-8" />
            <div className="h-12 w-40 rounded-xl bg-[#1cd98e]/20 animate-pulse mx-auto" />
          </div>
        </div>
      </section>

      {/* Grid Skeleton */}
      <section className="relative py-12 px-4 sm:px-6 lg:px-8">
        <div className="mx-auto max-w-7xl">
          <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
            {[1, 2, 3, 4, 5, 6].map((i) => (
              <div
                key={i}
                className="p-6 rounded-2xl bg-white/[0.02] border border-white/5"
              >
                <div className="flex justify-between mb-4">
                  <div className="h-6 w-32 rounded-full bg-white/5 animate-pulse" />
                  <div className="h-5 w-5 rounded bg-white/5 animate-pulse" />
                </div>
                <div className="h-7 w-3/4 rounded bg-white/5 animate-pulse mb-2" />
                <div className="h-4 w-full rounded bg-white/5 animate-pulse mb-1" />
                <div className="h-4 w-2/3 rounded bg-white/5 animate-pulse mb-6" />
                <div className="grid grid-cols-2 gap-4 mb-6">
                  <div className="h-5 w-24 rounded bg-white/5 animate-pulse" />
                  <div className="h-5 w-20 rounded bg-white/5 animate-pulse" />
                </div>
                <div className="pt-4 border-t border-white/5">
                  <div className="h-4 w-32 rounded bg-white/5 animate-pulse" />
                </div>
              </div>
            ))}
          </div>
        </div>
      </section>
    </div>
  );
}
