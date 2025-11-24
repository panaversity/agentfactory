# @repo/ui

Shared UI component library for the SSO monorepo, built with shadcn/ui and Tailwind CSS.

## Overview

This package provides a centralized collection of reusable UI components that can be shared across all apps in the monorepo (`sso-client`, `sso-admin`, etc.).

## Project Structure

```
packages/ui/
├── components/          # UI components
│   └── button.tsx      # Example: Button component
├── lib/                # Utilities
│   └── utils.ts        # cn() utility for className merging
├── components.json     # shadcn/ui configuration
├── globals.css         # Global styles with shadcn CSS variables
├── tailwind.config.ts  # Tailwind configuration
├── tsconfig.json       # TypeScript configuration
├── index.tsx           # Main export file
└── package.json        # Package dependencies
```

## Using Components in Apps

### 1. Import Components

```tsx
import { Button } from "@repo/ui";

export default function MyPage() {
  return <Button variant="default">Click me</Button>;
}
```

### 2. Import Styles

Make sure your app imports the UI package's global styles:

```css
/* In your app's globals.css */
@import "@repo/ui/globals.css";
```

### 3. Configure Tailwind

Your app's `tailwind.config.ts` should include the UI package's components:

```ts
content: [
  "./app/**/*.{js,ts,jsx,tsx,mdx}",
  "../../packages/ui/components/**/*.{js,ts,jsx,tsx,mdx}", // Important!
],
```

## Adding New Components

### Method 1: Manual Creation (Recommended for Monorepos)

1. **Create the component file** in `components/`:

```tsx
// components/card.tsx
import * as React from "react";
import { cn } from "../lib/utils";

export interface CardProps extends React.HTMLAttributes<HTMLDivElement> {}

const Card = React.forwardRef<HTMLDivElement, CardProps>(
  ({ className, ...props }, ref) => (
    <div
      ref={ref}
      className={cn(
        "rounded-lg border bg-card text-card-foreground shadow-sm",
        className
      )}
      {...props}
    />
  )
);
Card.displayName = "Card";

export { Card };
```

2. **Add dependencies** if needed:

```bash
cd packages/ui
pnpm add @radix-ui/react-dialog  # Example for Dialog component
```

3. **Export the component** in `index.tsx`:

```tsx
// index.tsx
export { Button, buttonVariants } from "./components/button";
export type { ButtonProps } from "./components/button";
export { Card } from "./components/card";  // Add new export
export type { CardProps } from "./components/card";

export { cn } from "./lib/utils";
```

4. **Use in your apps**:

```tsx
import { Card } from "@repo/ui";
```

### Method 2: Using shadcn/ui References

You can reference the official shadcn/ui components and copy their implementation:

1. Visit https://ui.shadcn.com/docs/components
2. Find the component you want (e.g., "Input")
3. Copy the component code
4. Paste into `packages/ui/components/input.tsx`
5. Adjust imports to use relative paths (`../lib/utils`)
6. Add to exports in `index.tsx`

### Method 3: Using shadcn MCP (Advanced)

If you have the shadcn MCP configured, you can search and view components:

```bash
# In packages/ui directory
pnpm dlx shadcn@latest add <component-name>
```

Note: This may require adjusting the generated code for monorepo structure.

## Available Components

- ✅ **Button** - Various button styles and sizes

## Common Component Patterns

### Using the cn() Utility

Always use the `cn()` utility for merging class names:

```tsx
import { cn } from "../lib/utils";

<div className={cn("base-classes", "more-classes", className)} />
```

### Forwarding Refs

Use `React.forwardRef` for components that need DOM refs:

```tsx
const MyComponent = React.forwardRef<HTMLDivElement, MyComponentProps>(
  (props, ref) => <div ref={ref} {...props} />
);
MyComponent.displayName = "MyComponent";
```

### Variant Props with CVA

Use `class-variance-authority` for variant-based styling:

```tsx
import { cva, type VariantProps } from "class-variance-authority";

const variants = cva("base-classes", {
  variants: {
    variant: {
      default: "variant-default-classes",
      secondary: "variant-secondary-classes",
    },
    size: {
      sm: "size-sm-classes",
      lg: "size-lg-classes",
    },
  },
  defaultVariants: {
    variant: "default",
    size: "sm",
  },
});
```

## Component Dependencies

Common dependencies for shadcn components:

```json
{
  "@radix-ui/react-slot": "Button, many components",
  "@radix-ui/react-dialog": "Dialog, AlertDialog",
  "@radix-ui/react-dropdown-menu": "DropdownMenu",
  "@radix-ui/react-select": "Select",
  "@radix-ui/react-tooltip": "Tooltip",
  "class-variance-authority": "Variant styling",
  "clsx": "Class name utilities",
  "tailwind-merge": "Tailwind class merging"
}
```

## Styling and Theming

### CSS Variables

The package uses CSS variables for theming (defined in `globals.css`):

- `--background`, `--foreground`
- `--primary`, `--primary-foreground`
- `--secondary`, `--secondary-foreground`
- `--muted`, `--muted-foreground`
- `--accent`, `--accent-foreground`
- `--destructive`, `--destructive-foreground`
- `--border`, `--input`, `--ring`
- `--radius` (border radius)

### Dark Mode

Dark mode is supported via the `.dark` class. Components automatically adapt when the dark class is applied to a parent element.

## Testing Components

Create a test page in any app to verify your components:

```tsx
// app/test-ui/page.tsx
import { Button, Card } from "@repo/ui";

export default function TestUI() {
  return (
    <div className="p-8 space-y-4">
      <h1>UI Component Tests</h1>
      <Button>Test Button</Button>
      <Card>Test Card</Card>
    </div>
  );
}
```

## Troubleshooting

### Components not styling correctly

1. Check that `globals.css` is imported in your app
2. Verify Tailwind config includes UI package in `content` array
3. Run `pnpm install` at the monorepo root

### TypeScript errors

1. Check that path aliases are set up in app's `tsconfig.json`
2. Ensure `@repo/ui` is in the app's dependencies
3. Rebuild the workspace: `pnpm install`

### Import errors

Make sure components are exported in `index.tsx`:

```tsx
export { YourComponent } from "./components/your-component";
```

## Best Practices

1. **Keep components generic** - Don't add app-specific logic
2. **Use TypeScript** - Provide proper types for all props
3. **Export types** - Make component prop types available
4. **Document props** - Add JSDoc comments for complex components
5. **Test in multiple apps** - Verify components work across apps
6. **Follow shadcn patterns** - Maintain consistency with shadcn/ui

## Adding More Apps to the Monorepo

When adding a new app to the monorepo that needs to use shadcn/ui components, follow these steps:

### Step 1: Add Dependency

Add `@repo/ui` to your new app's `package.json`:

```json
{
  "name": "your-new-app",
  "dependencies": {
    "@repo/ui": "workspace:*",
    "next": "16.0.3",
    "react": "19.2.0",
    "react-dom": "19.2.0"
  },
  "devDependencies": {
    "@types/node": "^20",
    "@types/react": "^19",
    "@types/react-dom": "^19",
    "autoprefixer": "^10.4.20",
    "eslint": "^9",
    "eslint-config-next": "16.0.3",
    "postcss": "^8.4.49",
    "tailwindcss": "^3.4.17",
    "typescript": "^5"
  }
}
```

### Step 2: Configure Tailwind CSS

Create `tailwind.config.ts` with the shadcn theme configuration:

```ts
// apps/your-new-app/tailwind.config.ts
import type { Config } from "tailwindcss";

const config: Config = {
  darkMode: ["class"],
  content: [
    "./pages/**/*.{js,ts,jsx,tsx,mdx}",
    "./components/**/*.{js,ts,jsx,tsx,mdx}",
    "./app/**/*.{js,ts,jsx,tsx,mdx}",
    // IMPORTANT: Include UI package components
    "../../packages/ui/components/**/*.{js,ts,jsx,tsx,mdx}",
  ],
  theme: {
    extend: {
      borderRadius: {
        lg: "var(--radius)",
        md: "calc(var(--radius) - 2px)",
        sm: "calc(var(--radius) - 4px)",
      },
      colors: {
        background: "hsl(var(--background))",
        foreground: "hsl(var(--foreground))",
        card: {
          DEFAULT: "hsl(var(--card))",
          foreground: "hsl(var(--card-foreground))",
        },
        popover: {
          DEFAULT: "hsl(var(--popover))",
          foreground: "hsl(var(--popover-foreground))",
        },
        primary: {
          DEFAULT: "hsl(var(--primary))",
          foreground: "hsl(var(--primary-foreground))",
        },
        secondary: {
          DEFAULT: "hsl(var(--secondary))",
          foreground: "hsl(var(--secondary-foreground))",
        },
        muted: {
          DEFAULT: "hsl(var(--muted))",
          foreground: "hsl(var(--muted-foreground))",
        },
        accent: {
          DEFAULT: "hsl(var(--accent))",
          foreground: "hsl(var(--accent-foreground))",
        },
        destructive: {
          DEFAULT: "hsl(var(--destructive))",
          foreground: "hsl(var(--destructive-foreground))",
        },
        border: "hsl(var(--border))",
        input: "hsl(var(--input))",
        ring: "hsl(var(--ring))",
        chart: {
          "1": "hsl(var(--chart-1))",
          "2": "hsl(var(--chart-2))",
          "3": "hsl(var(--chart-3))",
          "4": "hsl(var(--chart-4))",
          "5": "hsl(var(--chart-5))",
        },
      },
    },
  },
  plugins: [],
};

export default config;
```

### Step 3: Configure PostCSS

Create `postcss.config.mjs`:

```js
// apps/your-new-app/postcss.config.mjs
const config = {
  plugins: {
    tailwindcss: {},
    autoprefixer: {},
  },
};

export default config;
```

### Step 4: Import Global Styles

In your app's main CSS file (e.g., `app/globals.css`), import the UI package styles:

```css
/* apps/your-new-app/app/globals.css */
@import "@repo/ui/globals.css";
```

That's it! No need to duplicate all the Tailwind directives and CSS variables.

### Step 5: Update Layout

Make sure your root layout imports the globals.css:

```tsx
// apps/your-new-app/app/layout.tsx
import "./globals.css"; // This imports @repo/ui/globals.css

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
```

### Step 6: Install Dependencies

Run the install command from the monorepo root:

```bash
# From monorepo root
pnpm install
```

### Step 7: Test Components

Create a test page to verify the setup:

```tsx
// apps/your-new-app/app/page.tsx
import { Button } from "@repo/ui";

export default function Home() {
  return (
    <div className="flex min-h-screen items-center justify-center p-8">
      <div className="space-y-4">
        <h1 className="text-4xl font-bold">Your New App</h1>
        <div className="flex gap-4">
          <Button>Default</Button>
          <Button variant="secondary">Secondary</Button>
          <Button variant="outline">Outline</Button>
        </div>
      </div>
    </div>
  );
}
```

### Step 8: Start Development Server

```bash
cd apps/your-new-app
pnpm run dev
```

### Checklist for New Apps

- [ ] Added `@repo/ui` to dependencies
- [ ] Created `tailwind.config.ts` with UI package in content array
- [ ] Created `postcss.config.mjs` with tailwindcss and autoprefixer
- [ ] Created `app/globals.css` importing `@repo/ui/globals.css`
- [ ] Root layout imports globals.css
- [ ] Ran `pnpm install` from monorepo root
- [ ] Tested that components render with proper styling
- [ ] Verified dark mode works (if needed)

### Common Issues

**Components not styling:**
- Ensure UI package is in `tailwind.config.ts` content array
- Check that `@repo/ui/globals.css` is imported
- Run `pnpm install` again

**Import errors:**
- Verify `@repo/ui` is in package.json dependencies
- Check that workspace is using `workspace:*` protocol
- Restart your dev server

**Type errors:**
- Ensure TypeScript can find the types from `@repo/ui`
- Check that `node_modules` is properly linked
- Try deleting `node_modules` and reinstalling

## Resources

- [shadcn/ui Documentation](https://ui.shadcn.com)
- [Radix UI Documentation](https://www.radix-ui.com)
- [Tailwind CSS Documentation](https://tailwindcss.com)
- [CVA Documentation](https://cva.style)

## License

This is an internal package for the SSO monorepo.
