# AI Provider Authentication

## Overview

Your workspace is created, but agents can't do work yet—they need credentials to access AI models. This section covers configuring three AI providers with Antigravity:

- **Google AI (Gemini)**: Free tier, fast responses
- **Anthropic (Claude)**: High-quality reasoning, Antigravity's preferred provider
- **OpenAI (GPT-4/GPT-4-mini)**: Reliable, mature API

Pick at least one. If you have multiple API keys, configure multiple providers—Antigravity lets you choose per agent.

**Time estimate**: 10-15 minutes (assuming you already have API keys)

---

## Prerequisites

You should have API keys from at least one provider. If not:

| Provider | Sign-up | Free Tier? | Get Key |
|----------|---------|-----------|---------|
| **Google AI** | https://ai.google.dev | Yes (50 reqs/min) | API Keys tab |
| **Anthropic** | https://console.anthropic.com | Yes (with limits) | Dashboard → API Keys |
| **OpenAI** | https://platform.openai.com | Yes ($5 credits) | Settings → API Keys |

**Time-saving tip**: Have your API key copied to clipboard before starting this section.

---

## Configuring AI Providers in Antigravity

### Access AI Provider Settings

1. Open Antigravity and navigate to your workspace
2. Click **Settings ⚙️** (top-right of Agent Manager)
3. Click **"AI Providers"** tab

You see a list:
- Google AI (Gemini)
- Anthropic (Claude)
- OpenAI (GPT-4)
- Ollama (local models)
- Custom (enterprise)

---

## Option 1: Google AI (Gemini)

### Step 1: Get API Key

1. Visit https://ai.google.dev
2. Click **"Get API Key"**
3. Select or create Google Cloud project
4. Copy the generated API key (looks like: `AIza...`)
5. Keep this page open

### Step 2: Input Key in Antigravity

1. In **AI Providers** settings, click **"Google AI (Gemini)"**
2. Paste API key into **"API Key"** field
3. Click **"Test Connection"** button

**Expected responses**:
- ✅ "Connection successful" → Key is valid, you're set!
- ❌ "Invalid API key" → Copy key again (ensure no spaces at start/end)
- ❌ "Quota exceeded" → You've hit free tier limit (Google limits free keys to 50 requests/minute)

### Step 3: Save Configuration

Click **"Save"** to persist your Google AI credentials.

You can now:
- Check **"Set as default"** to use Gemini for new agents automatically
- Leave unchecked to choose per agent

---

## Option 2: Anthropic (Claude)

### Step 1: Get API Key

1. Visit https://console.anthropic.com/
2. Sign in with your Anthropic account
3. Click **"API Keys"** (left sidebar)
4. Click **"Create Key"**
5. Copy the generated key (looks like: `sk-ant-...`)
6. Give it a name like "Antigravity Development"

### Step 2: Input Key in Antigravity

1. In **AI Providers** settings, click **"Anthropic (Claude)"**
2. Paste API key into **"API Key"** field
3. Select model version:
   - **Claude Sonnet** (recommended): Fast, excellent reasoning
   - **Claude Opus**: More powerful, slower, higher cost
   - **Claude Haiku**: Fast, lower cost, simpler tasks

4. Click **"Test Connection"**

**Expected**:
- ✅ "Connection successful" → Ready to use!
- ❌ "Invalid API key" → Verify key copied completely
- ❌ "Insufficient credits" → Check Anthropic dashboard for remaining balance

### Step 3: Save Configuration

Click **"Save"** and optionally check **"Set as default"** (recommended for this lesson).

---

## Option 3: OpenAI (GPT-4 or GPT-4-mini)

### Step 1: Get API Key

1. Visit https://platform.openai.com/api/keys
2. Sign in with your OpenAI account
3. Click **"Create new secret key"**
4. Copy the key (looks like: `sk-proj-...`)
5. Do NOT share this key

### Step 2: Input Key in Antigravity

1. In **AI Providers** settings, click **"OpenAI"**
2. Paste API key into **"API Key"** field
3. Select model:
   - **GPT-4 Turbo**: Powerful, slower (~30sec response)
   - **GPT-4-mini**: Fast (~5sec response), cheaper
   - **GPT-4o**: Latest, optimized for agents (if available)

4. Click **"Test Connection"**

**Expected**:
- ✅ "Connection successful" → Ready!
- ❌ "Invalid API key" → Check key format (must start with `sk-`)
- ❌ "Quota exceeded" → Out of free credits, add payment method

### Step 3: Save Configuration

Click **"Save"** and optionally set as default.

---

## Configuring Multiple Providers

### Why Use Multiple?

Different providers have different strengths:
- **Google Gemini**: Fastest, good for simple tasks
- **Anthropic Claude**: Best reasoning, preferred for complex work
- **OpenAI GPT-4**: Mature, reliable, multi-modal (code + images)

Antigravity lets you assign different providers to different agents.

### How to Configure Multiple

Repeat the steps above for each provider you want to use. In Antigravity:
1. Add Google AI key
2. Click **"Save"**, don't check "set as default"
3. Go back to **AI Providers**
4. Add Anthropic key
5. Optionally check **"Set as default"**
6. Go back to **AI Providers**
7. Add OpenAI key
8. Don't check "set as default"

Now you have all three available. When creating agents later, you'll choose which provider each agent uses.

---

## Testing Your Configuration

### Quick Test in Agent Manager

1. Close Settings
2. You're back in Agent Manager
3. Click **"Create Agent"** button
4. In agent creation dialog, under **"AI Provider"**, you should see all configured providers

If you see your providers listed, authentication is working!

**Don't create agent yet**—that's next section. Just verify the dropdown shows your providers, then close this dialog.

---

## Troubleshooting Provider Configuration

### Problem: "Invalid API Key"
**Solution**:
1. Go to provider's website and copy key again (character by character)
2. Ensure no leading/trailing spaces
3. Verify key hasn't been revoked in provider's dashboard
4. Check that you're using the right key for the provider (Anthropic key won't work for OpenAI)

### Problem: "Connection test failed but I'm sure the key is valid"
**Solution**:
1. Check your internet connection
2. Verify your firewall isn't blocking Antigravity
3. Wait a moment and try again (provider might be temporarily down)
4. Check provider status:
   - Google: https://status.cloud.google.com
   - Anthropic: https://status.anthropic.com
   - OpenAI: https://status.openai.com

### Problem: "Model not available"
**Solution**:
1. Some models require payment history (OpenAI GPT-4)
2. Free tier may have limited model access
3. Add payment method to provider account
4. Re-test connection

### Problem: "Quota exceeded" (free tier limit)
**Solution**:
1. Free tiers have rate limits (Google: 50 req/min, OpenAI: varies)
2. Either add payment method or use a different provider
3. Or wait until quota resets (usually next calendar day)

---

## API Key Management Best Practices

### Security

- **Never share your API keys** (not in code, not in messages, not in screenshots)
- **Don't commit keys to git** (even private repos are at risk)
- **Rotate keys regularly** (delete old ones, create new ones)
- **Use least-privilege keys** (if provider allows, restrict by IP/domain)

### Organization

- **Label keys meaningfully**: "Antigravity Development", "Learning IDE", not just "key-1"
- **Set usage limits** (if provider supports): Cap monthly spend per key
- **Monitor usage**: Check provider dashboard monthly for unexpected charges
- **Use different keys for different purposes**: One for development, one for production later

### In Antigravity

API keys are stored in:
- **macOS/Linux**: `~/.antigravity/workspaces/[name]/config.json` (encrypted at rest)
- **Windows**: `%USERPROFILE%\.antigravity\workspaces\[name]\config.json`

Antigravity encrypts keys using your system's credential storage, but never commit config.json to public repos.

---

## Verification Checkpoint

Before proceeding to agent creation, verify:

- [ ] At least one AI provider configured
- [ ] "Test Connection" returned success for that provider
- [ ] Provider appears in agent creation dialog dropdown

**All verified?** Great! Next section covers creating your first agent and using the three surfaces.

---

## Cost Implications

### Free Tier Usage (This Lesson)

For this lesson (creating one simple agent), costs are negligible:

| Provider | Estimated Cost | Notes |
|----------|-----------------|-------|
| **Google AI** | $0 | Free tier: 50 req/min |
| **Anthropic** | ~$0.01-0.05 | Free trial: $5 credits |
| **OpenAI** | ~$0.01-0.10 | Free trial: $5 credits |

You'll use far less than free tier limits for this lesson.

### Future Lessons

As you create more complex agents and run longer tasks, costs will increase slightly but remain very low for learning purposes. Typical monthly cost for learning (small agents, simple tasks): <$10 across all providers.

---

## What's Next?

You've configured AI providers! Your agents now have access to AI models.

**Next step**: [04-three-surfaces.md](./04-three-surfaces.md) - Understand Antigravity's three-surface architecture

**Time check**: Setup + authentication ~30-35 minutes. You have 25-30 minutes remaining.

---

## Quick Reference: API Key Formats

| Provider | Key Format | Where to Get |
|----------|-----------|--------------|
| **Google AI** | `AIza...` (40+ chars) | https://ai.google.dev |
| **Anthropic** | `sk-ant-...` (varies) | https://console.anthropic.com/api/keys |
| **OpenAI** | `sk-proj-...` (varies) | https://platform.openai.com/api/keys |

Each provider's key is unique to that provider. Mix-matching won't work.

---

## Advanced: Custom/Enterprise Providers

If your organization uses a custom LLM provider (self-hosted Ollama, enterprise Azure OpenAI, etc.), Antigravity also supports custom providers. This is covered in advanced lessons.

For now, stick to Google/Anthropic/OpenAI.
