# 📊 Analytics Implementation - AI Native Software Development

## Overview

This contains the analytics setup for tracking learner engagement across the curriculum.

**Setup:** Configure your GA4 Measurement ID via environment variable (see `docs/ANALYTICS_SETUP.md`)

## Files

### Core Analytics

- **`src/utils/analytics.ts`** - Analytics event tracking functions
  - `trackChapterView()` - Track chapter views
  - `trackResourceClick()` - Track external link clicks
  - `trackScrollDepth()` - Track reading engagement
  - `trackTimeOnPage()` - Track time spent on page
  - `trackCodeInteraction()` - Track code copy/view events
  - `trackQuizInteraction()` - Track quiz attempts
  - `trackDownload()` - Track file downloads
  - `trackLearnerSegment()` - Segment learners by type
  - `trackCustomEvent()` - Custom event tracking

- **`src/components/AnalyticsTracker.tsx`** - React component for automatic tracking
  - Automatically tracks page views on navigation
  - Tracks scroll depth as users read
  - Tracks external link clicks
  - Tracks time spent on page

- **`src/analytics/dashboard-config.ts`** - Dashboard configuration
  - Pre-configured dashboards for key metrics
  - Alert rules for monitoring
  - Weekly monitoring checklist

## Quick Start

### 1. Get Your Measurement ID

- Go to [Google Analytics 4](https://analytics.google.com)
- Create a property
- Copy your **Measurement ID** (format: `G-XXXXXXXXXX`)

### 2. Set Environment Variable

```bash
export GA4_MEASUREMENT_ID=G-XXXXXXXXXX
```

### 3. Verify Installation

GA4 script in `docusaurus.config.ts` uses the environment variable:

```typescript
...(process.env.GA4_MEASUREMENT_ID ? [
  {
    tagName: 'script',
    attributes: {
      async: 'true',
      src: `https://www.googletagmanager.com/gtag/js?id=${process.env.GA4_MEASUREMENT_ID}`,
    },
  },
  {
    tagName: 'script',
    innerHTML: `
      window.dataLayer = window.dataLayer || [];
      function gtag(){dataLayer.push(arguments);}
      gtag('js', new Date());
      gtag('config', '${process.env.GA4_MEASUREMENT_ID}', {
        'anonymize_ip': true,
        'allow_google_signals': false,
        'allow_ad_personalization_signals': false
      });
    `,
  },
] : []),
```

### 2. Integrate Analytics Tracker

Wrap your main layout with the `AnalyticsTracker` component:

```tsx
import { AnalyticsTracker } from '@/components/AnalyticsTracker';

export function DocLayout({ children }) {
  return (
    <AnalyticsTracker>
      {children}
    </AnalyticsTracker>
  );
}
```

### 3. Test Locally

1. Run `npm start`
2. Open [Google Analytics DebugView](https://analytics.google.com)
3. Navigate your site
4. Verify events appear in real-time

### 4. Access Dashboard

1. Go to [analytics.google.com](https://analytics.google.com)
2. Select property: "ai-native.panaversity.org"
3. Start monitoring! 🎉

## Tracked Events

### Automatic Events (No Code Required)

- ✅ Page views
- ✅ Session duration
- ✅ Device info (mobile/desktop/tablet)
- ✅ Browser type
- ✅ Traffic source
- ✅ Geographic location

### Custom Events (Implemented)

- `chapter_view` - When learners view a chapter
- `scroll_depth` - Reading engagement (25%, 50%, 75%, 100%)
- `resource_click` - External link clicks (GitHub, YouTube, docs)
- `page_engagement` - Time spent on page

### Custom Events (Ready to Implement)

- `code_interaction` - Code copy/expand/view
- `quiz_interaction` - Quiz start/complete/answer
- `file_download` - Downloads initiated
- `learner_segment` - Segment learner type

## Usage Examples

### Track Chapter View

```typescript
import { trackChapterView } from '@/utils/analytics';

trackChapterView(3, 'Billion Dollar AI', 'Part 1');
```

### Track Resource Click

```typescript
import { trackResourceClick } from '@/utils/analytics';

trackResourceClick('github', 'https://github.com/panaversity/...', 'Example Repo');
```

### Track Quiz Completion

```typescript
import { trackQuizInteraction } from '@/utils/analytics';

trackQuizInteraction('complete', 'Part 1 Quiz', 85, 'Part 1');
```

### Track Code Copy

```typescript
import { trackCodeInteraction } from '@/utils/analytics';

const handleCopyCode = () => {
  // Copy code...
  trackCodeInteraction('copy', 'python', 'Chapter 5');
};
```

## Key Dashboards to Monitor

### 1. **Learning Overview** (Weekly)
- Total users and new vs returning
- Engagement rate
- Average session duration

### 2. **Chapter Performance** (Weekly)
- Top viewed chapters
- Drop-off analysis
- Scroll depth by chapter

### 3. **Learning Progression** (Weekly)
- Part completion rates
- Where students drop off
- Progression funnels

### 4. **Resource Engagement** (Weekly)
- GitHub clicks
- YouTube clicks
- Most clicked resources

### 5. **Traffic Analysis** (Weekly)
- Organic search traffic
- Referral sources
- Social media traffic

See **`docs/ANALYTICS.md`** for detailed dashboard setup.

## Privacy & GDPR

Analytics is configured with privacy-first settings:

- ✅ IP anonymization enabled
- ✅ No cross-site tracking
- ✅ No ad personalization
- ✅ Compliant with privacy regulations

## Troubleshooting

### Events Not Appearing?

1. Check GA4 DebugView: https://analytics.google.com
2. Verify Measurement ID: `G-M8S81FYRFT`
3. Open browser DevTools console - should see `📊 Analytics:` logs
4. Check ad blocker isn't blocking analytics
5. Wait 24 hours for data to fully process

### Data Not Updating?

1. Verify events are firing in DebugView
2. Check filters in reports (sometimes data is filtered out)
3. Confirm date range includes today
4. Clear GA4 cache

## Google Analytics 4 Setup

This site automatically tracks learner engagement using Google Analytics 4 (GA4).

## What Gets Tracked

✅ **Page views** - Which chapters learners visit  
✅ **Scroll depth** - How far students read (25%, 50%, 75%, 100%)  
✅ **External links** - GitHub, YouTube, documentation clicks  
✅ **Time on page** - How long students spend per chapter  
✅ **Device & location** - Mobile vs desktop, learner locations  

## How to Enable

### 1. Create GA4 Property

- Go to [analytics.google.com](https://analytics.google.com)
- Click **Admin** → **Create Property**
- Fill in property name and settings
- Click **Create**

### 2. Get Measurement ID

- Go to **Admin** → **Data Streams**
- Select your web stream
- Copy the **Measurement ID** (format: `G-XXXXXXXXXX`)

### 3. Set Environment Variable

**Local Development:**
```bash
export GA4_MEASUREMENT_ID=G-XXXXXXXXXX
npm run build
npm run start
```

**Production (GitHub):**
- Go to your GitHub repo
- **Settings** → **Secrets and variables** → **Actions**
- Add secret: `GA4_MEASUREMENT_ID` = `G-XXXXXXXXXX`
- The deploy workflow will automatically use it

### 4. Verify It Works

1. Run your site: `npm start`
2. Open [analytics.google.com](https://analytics.google.com)
3. Go to **Reports** → **Realtime**
4. Navigate through your site
5. You should see **1 active user** and **events firing**

## Privacy

✅ IP anonymization enabled  
✅ No cross-site tracking  
✅ No ad personalization  
✅ GDPR compliant  

## View Your Data

Once tracking is active, visit [analytics.google.com](https://analytics.google.com) to see:

- **Realtime** - Active users right now
- **Reports** → **Acquisition** - Where traffic comes from
- **Reports** → **Engagement** - How students interact
- **Reports** → **Retention** - Return visitor patterns

## Troubleshooting

**GA4 not receiving data?**
- Check: `echo $GA4_MEASUREMENT_ID`
- Verify Measurement ID starts with `G-`
- Open DevTools console - should see `📊 Analytics:` messages
- Wait 24 hours for data to appear in reports

**To disable analytics:**
- Don't set the `GA4_MEASUREMENT_ID` environment variable

## Resources

- **[GA4 Documentation](https://support.google.com/analytics)** - Official Google Analytics docs
- **[GA4 Events Reference](https://support.google.com/analytics/answer/9322688)** - Event naming conventions
- **[GA4 for Education](https://support.google.com/analytics/answer/11064533)** - Education-specific guidance
- **[Analytics Guide](docs/ANALYTICS.md)** - Internal guide for team

## Next Steps

1. ✅ Install GA4 (done!)
2. ✅ Add AnalyticsTracker component to layouts (do this)
3. ✅ Test in development with DebugView
4. ✅ Deploy to production
5. ✅ Monitor dashboards weekly
6. 🔜 Add custom events as content is created
7. 🔜 Set up goals/conversions for learning milestones
8. 🔜 Create automated email reports

