# Docusaurus PanaversityFS Plugin

Fetch educational content from PanaversityFS MCP server at build time.

## Usage

Add to `docusaurus.config.ts`:

```typescript
plugins: [
  [
    './plugins/docusaurus-panaversityfs-plugin',
    {
      bookId: 'ai-native-software-development',
      enabled: true,
      useMockData: true,  // false to connect to real MCP server
    }
  ]
]
```

## Options

| Option | Default | Description |
|--------|---------|-------------|
| `bookId` | `'ai-native-software-development'` | Book identifier |
| `enabled` | `false` | Enable/disable plugin |
| `useMockData` | `true` | Use mock data or real MCP server |
| `serverPath` | `'../../panaversity-fs'` | Path to PanaversityFS |
| `storageBackend` | `'fs'` | Storage backend (fs/s3/supabase) |
| `storageRoot` | `'/tmp/panaversity-test'` | Storage root path |

## Testing

```bash
# Build with plugin
cd apps/learn-app
npm run build 2>&1 | grep PanaversityFS

# Expected output (mock mode)
[PanaversityFS] Using mock data for POC
[PanaversityFS] Content loaded: 1 lessons
```

## Files

```
docusaurus-panaversityfs-plugin/
├── index.js       # Plugin implementation
├── mcp-client.js  # MCP client for Node.js
├── package.json   # Metadata
└── README.md      # This file
```

## How It Works

1. **loadContent()**: Fetches lessons from PanaversityFS (or mock data)
2. **contentLoaded()**: Creates data files in `.docusaurus/`
3. Build uses data files to render content

## License

MIT
