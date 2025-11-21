# SSO Monorepo Setup

## Prerequisites
- Node.js 20+ installed
- pnpm installed (`npm install -g pnpm`)

## Step 1: Initialize Monorepo with Turborepo

Create the monorepo structure using Turborepo:
```bash
# Create project directory
mkdir sso-monorepo && cd sso-monorepo

# Initialize with Turborepo
pnpm dlx create-turbo@latest . --package-manager pnpm

# When prompted:
# - Where would you like to create your turborepo? . (current directory)
# - Which package manager? pnpm
```

After completion, clean up the default apps and packages:
```bash
# Remove default example apps and packages
rm -rf apps/* packages/*

# Your structure should now be:
# sso-monorepo/
# ├── turbo.json
# ├── package.json
# ├── pnpm-workspace.yaml
# └── (empty apps and packages folders)
```