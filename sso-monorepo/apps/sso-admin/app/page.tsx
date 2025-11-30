import { Button } from "@repo/ui";

export default function Home() {
  return (
    <div className="flex min-h-screen items-center justify-center p-8 bg-slate-50">
      <div className="max-w-2xl space-y-8">
        <div>
          <h1 className="text-4xl font-bold mb-2 text-slate-900">SSO Admin Dashboard</h1>
          <p className="text-muted-foreground">Testing shared shadcn/ui components from @repo/ui</p>
        </div>

        <div className="space-y-4">
          <h2 className="text-2xl font-semibold">Button Variants</h2>
          <div className="flex flex-wrap gap-4">
            <Button>Default</Button>
            <Button variant="secondary">Secondary</Button>
            <Button variant="destructive">Destructive</Button>
            <Button variant="outline">Outline</Button>
            <Button variant="ghost">Ghost</Button>
            <Button variant="link">Link</Button>
          </div>
        </div>

        <div className="space-y-4">
          <h2 className="text-2xl font-semibold">Button Sizes</h2>
          <div className="flex flex-wrap items-center gap-4">
            <Button size="sm">Small</Button>
            <Button size="default">Default</Button>
            <Button size="lg">Large</Button>
            <Button size="icon">⚙️</Button>
          </div>
        </div>

        <div className="space-y-4">
          <h2 className="text-2xl font-semibold">Disabled State</h2>
          <div className="flex flex-wrap gap-4">
            <Button disabled>Disabled</Button>
            <Button variant="outline" disabled>Disabled Outline</Button>
          </div>
        </div>

        <div className="mt-8 p-4 bg-green-50 border border-green-200 rounded-lg">
          <p className="text-sm text-green-800">
            ✅ If you can see styled buttons above, the shared UI package is working correctly!
          </p>
        </div>
      </div>
    </div>
  );
}
