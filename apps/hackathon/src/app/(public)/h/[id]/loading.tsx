export default function HackathonLoading() {
  return (
    <div className="relative overflow-hidden">
      {/* Background Effects */}
      <div className="absolute inset-0 overflow-hidden pointer-events-none">
        <div className="absolute top-0 left-1/4 w-[800px] h-[800px] bg-[#1cd98e]/5 rounded-full blur-[120px] -translate-y-1/2" />
      </div>

      {/* Hero Skeleton */}
      <section className="relative pt-12 pb-20 px-4 sm:px-6 lg:px-8">
        <div className="mx-auto max-w-7xl">
          {/* Status Badge */}
          <div className="h-8 w-40 rounded-full bg-white/5 animate-pulse mb-6" />

          {/* Title */}
          <div className="space-y-3 mb-6">
            <div className="h-14 w-3/4 rounded-lg bg-white/5 animate-pulse" />
            <div className="h-14 w-1/2 rounded-lg bg-white/5 animate-pulse" />
          </div>

          {/* Description */}
          <div className="space-y-2 mb-10 max-w-3xl">
            <div className="h-5 w-full rounded bg-white/5 animate-pulse" />
            <div className="h-5 w-5/6 rounded bg-white/5 animate-pulse" />
            <div className="h-5 w-4/6 rounded bg-white/5 animate-pulse" />
          </div>

          {/* Quick Stats */}
          <div className="flex gap-6 mb-12">
            <div className="h-6 w-48 rounded bg-white/5 animate-pulse" />
            <div className="h-6 w-48 rounded bg-white/5 animate-pulse" />
          </div>

          {/* CTA Buttons */}
          <div className="flex gap-4 mb-16">
            <div className="h-14 w-40 rounded-xl bg-[#1cd98e]/20 animate-pulse" />
            <div className="h-14 w-40 rounded-xl bg-white/5 animate-pulse" />
          </div>

          {/* Countdown */}
          <div className="mb-16">
            <div className="h-4 w-40 rounded bg-white/5 animate-pulse mb-4" />
            <div className="flex gap-4">
              {[1, 2, 3, 4].map((i) => (
                <div key={i} className="text-center">
                  <div className="w-24 h-28 rounded-2xl bg-white/5 animate-pulse" />
                  <div className="h-3 w-16 rounded bg-white/5 animate-pulse mt-2 mx-auto" />
                </div>
              ))}
            </div>
          </div>
        </div>
      </section>

      {/* Stats Section Skeleton */}
      <section className="relative py-16 px-4 sm:px-6 lg:px-8 border-y border-white/5">
        <div className="mx-auto max-w-7xl">
          <div className="grid grid-cols-2 md:grid-cols-4 gap-6">
            {[1, 2, 3, 4].map((i) => (
              <div key={i} className="p-6 rounded-2xl bg-white/[0.02] border border-white/5">
                <div className="h-6 w-6 rounded bg-white/10 animate-pulse mb-4" />
                <div className="h-10 w-24 rounded bg-white/5 animate-pulse mb-2" />
                <div className="h-4 w-20 rounded bg-white/5 animate-pulse" />
              </div>
            ))}
          </div>
        </div>
      </section>

      {/* Prizes Skeleton */}
      <section className="relative py-20 px-4 sm:px-6 lg:px-8">
        <div className="mx-auto max-w-7xl">
          <div className="text-center mb-12">
            <div className="h-10 w-64 rounded bg-white/5 animate-pulse mx-auto mb-4" />
            <div className="h-5 w-96 rounded bg-white/5 animate-pulse mx-auto" />
          </div>
          <div className="grid md:grid-cols-3 gap-6">
            {[1, 2, 3].map((i) => (
              <div key={i} className="p-8 rounded-2xl bg-white/[0.02] border border-white/5">
                <div className="h-10 w-10 rounded bg-white/10 animate-pulse mb-4" />
                <div className="h-4 w-20 rounded bg-white/5 animate-pulse mb-2" />
                <div className="h-7 w-32 rounded bg-white/5 animate-pulse mb-2" />
                <div className="h-9 w-24 rounded bg-white/5 animate-pulse" />
              </div>
            ))}
          </div>
        </div>
      </section>
    </div>
  );
}
