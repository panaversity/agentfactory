/**
 * Analytics Dashboard Configuration
 * 
 * This defines key metrics and dashboards for monitoring learner engagement
 * Can be used to create custom dashboards in GA4 or visualize exported data
 * 
 * Key dashboards to monitor:
 * 1. Learning Overview - High-level engagement metrics
 * 2. Chapter Performance - Which chapters engage most
 * 3. Learner Retention - Who comes back and how often
 * 4. Resource Engagement - Which external resources matter
 * 5. Drop-off Analysis - Where do learners exit
 */

export interface DashboardMetric {
  id: string;
  title: string;
  description: string;
  metric: string;
  dimension?: string;
  filters?: Record<string, string | number>;
  sortBy?: 'asc' | 'desc';
  target?: number; // Optional target value
}

export interface Dashboard {
  id: string;
  title: string;
  description: string;
  metrics: DashboardMetric[];
  updateFrequency: 'daily' | 'weekly' | 'monthly';
}

// ==========================================
// DASHBOARD 1: Learning Overview
// ==========================================

export const DASHBOARD_LEARNING_OVERVIEW: Dashboard = {
  id: 'learning-overview',
  title: 'Learning Overview',
  description: 'High-level engagement and growth metrics',
  updateFrequency: 'daily',
  metrics: [
    {
      id: 'total-users',
      title: 'Total Unique Users',
      description: 'All visitors to the platform',
      metric: 'active_users',
      target: 1000, // Example target: 1000 learners/month
    },
    {
      id: 'new-users',
      title: 'New Users',
      description: 'First-time visitors',
      metric: 'new_users',
    },
    {
      id: 'returning-users',
      title: 'Returning Users',
      description: 'Users who visited more than once',
      metric: 'returning_users',
      target: 300, // Example target: 30% of users return
    },
    {
      id: 'avg-engagement-time',
      title: 'Avg. Engagement Time',
      description: 'Average session duration in minutes',
      metric: 'average_session_duration',
      target: 15, // 15 minutes target
    },
    {
      id: 'engagement-rate',
      title: 'Engagement Rate',
      description: 'Percentage of sessions with interactions',
      metric: 'engagement_rate',
      target: 75, // 75% target
    },
    {
      id: 'avg-sessions-per-user',
      title: 'Avg Sessions per User',
      description: 'How many times users visit',
      metric: 'sessions_per_user',
    },
  ],
};

// ==========================================
// DASHBOARD 2: Chapter Performance
// ==========================================

export const DASHBOARD_CHAPTER_PERFORMANCE: Dashboard = {
  id: 'chapter-performance',
  title: 'Chapter Performance',
  description: 'Engagement metrics by chapter/lesson',
  updateFrequency: 'weekly',
  metrics: [
    {
      id: 'top-chapters-by-views',
      title: 'Top Chapters by Views',
      description: 'Most visited chapters',
      metric: 'screen_views',
      dimension: 'page_title',
      sortBy: 'desc',
    },
    {
      id: 'top-chapters-by-engagement',
      title: 'Top Chapters by Engagement',
      description: 'Chapters with longest session duration',
      metric: 'average_session_duration',
      dimension: 'page_title',
      sortBy: 'desc',
    },
    {
      id: 'chapters-with-highest-bounce',
      title: 'Chapters with Highest Bounce Rate',
      description: 'Chapters where users leave without engaging',
      metric: 'bounce_rate',
      dimension: 'page_title',
      sortBy: 'desc',
    },
    {
      id: 'scroll-depth-by-chapter',
      title: 'Avg Scroll Depth by Chapter',
      description: 'How far students read on average',
      metric: 'scroll_depth',
      dimension: 'page_title',
      sortBy: 'desc',
      target: 75, // Target 75% scroll depth
    },
  ],
};

// ==========================================
// DASHBOARD 3: Learning Progression
// ==========================================

export const DASHBOARD_LEARNING_PROGRESSION: Dashboard = {
  id: 'learning-progression',
  title: 'Learning Progression',
  description: 'How learners progress through curriculum',
  updateFrequency: 'weekly',
  metrics: [
    {
      id: 'part1-completion',
      title: 'Part 1 Completion Rate',
      description: 'Users who completed Part 1',
      metric: 'part1_completions',
      target: 80, // 80% of users complete Part 1
    },
    {
      id: 'part2-completion',
      title: 'Part 2 Completion Rate',
      description: 'Users who completed Part 2',
      metric: 'part2_completions',
      target: 60, // 60% complete Part 2
    },
    {
      id: 'part3-completion',
      title: 'Part 3+ Completion Rate',
      description: 'Users who reach advanced parts',
      metric: 'part3_completions',
      target: 40, // 40% reach advanced content
    },
    {
      id: 'avg-parts-completed',
      title: 'Avg Parts Completed',
      description: 'Average how far learners progress',
      metric: 'average_parts_completed',
      target: 2, // Target: average learner completes 2 parts
    },
    {
      id: 'drop-off-analysis',
      title: 'Drop-off Point',
      description: 'Where most learners stop',
      metric: 'drop_off_point',
      dimension: 'part_number',
    },
  ],
};

// ==========================================
// DASHBOARD 4: Resource Engagement
// ==========================================

export const DASHBOARD_RESOURCE_ENGAGEMENT: Dashboard = {
  id: 'resource-engagement',
  title: 'Resource Engagement',
  description: 'Which external resources students use',
  updateFrequency: 'weekly',
  metrics: [
    {
      id: 'github-clicks',
      title: 'GitHub Clicks',
      description: 'Clicks to GitHub repositories',
      metric: 'resource_clicks',
      filters: { resource_type: 'github' },
    },
    {
      id: 'youtube-clicks',
      title: 'YouTube Clicks',
      description: 'Clicks to YouTube videos',
      metric: 'resource_clicks',
      filters: { resource_type: 'youtube' },
    },
    {
      id: 'docs-clicks',
      title: 'External Docs Clicks',
      description: 'Clicks to external documentation',
      metric: 'resource_clicks',
      filters: { resource_type: 'docs' },
    },
    {
      id: 'total-resource-clicks',
      title: 'Total Resource Clicks',
      description: 'All external resource interactions',
      metric: 'resource_clicks',
    },
    {
      id: 'resources-per-session',
      title: 'Resources per Session',
      description: 'Avg resources clicked per session',
      metric: 'resources_per_session',
    },
  ],
};

// ==========================================
// DASHBOARD 5: Traffic & Acquisition
// ==========================================

export const DASHBOARD_TRAFFIC: Dashboard = {
  id: 'traffic-analysis',
  title: 'Traffic & Acquisition',
  description: 'Where learners come from',
  updateFrequency: 'weekly',
  metrics: [
    {
      id: 'organic-search',
      title: 'Organic Search',
      description: 'Traffic from search engines',
      metric: 'users',
      filters: { source: 'google' },
    },
    {
      id: 'direct-traffic',
      title: 'Direct Traffic',
      description: 'Users typing URL directly',
      metric: 'users',
      filters: { source: 'direct' },
    },
    {
      id: 'referral-traffic',
      title: 'Referral Traffic',
      description: 'Users from other websites',
      metric: 'users',
      filters: { source: 'referral' },
    },
    {
      id: 'social-traffic',
      title: 'Social Media Traffic',
      description: 'Users from social platforms',
      metric: 'users',
      filters: { source: 'social' },
    },
    {
      id: 'top-traffic-sources',
      title: 'Top Traffic Sources',
      description: 'Most common referral sources',
      metric: 'users',
      dimension: 'source',
      sortBy: 'desc',
    },
  ],
};

// ==========================================
// DASHBOARD 6: Device & Technology
// ==========================================

export const DASHBOARD_DEVICE_ANALYSIS: Dashboard = {
  id: 'device-analysis',
  title: 'Device & Technology',
  description: 'How learners access the platform',
  updateFrequency: 'weekly',
  metrics: [
    {
      id: 'desktop-users',
      title: 'Desktop Users',
      description: 'Users on desktop computers',
      metric: 'users',
      filters: { device_category: 'desktop' },
    },
    {
      id: 'mobile-users',
      title: 'Mobile Users',
      description: 'Users on mobile devices',
      metric: 'users',
      filters: { device_category: 'mobile' },
    },
    {
      id: 'tablet-users',
      title: 'Tablet Users',
      description: 'Users on tablets',
      metric: 'users',
      filters: { device_category: 'tablet' },
    },
    {
      id: 'desktop-engagement',
      title: 'Desktop Engagement',
      description: 'Desktop avg session duration',
      metric: 'average_session_duration',
      filters: { device_category: 'desktop' },
    },
    {
      id: 'mobile-engagement',
      title: 'Mobile Engagement',
      description: 'Mobile avg session duration',
      metric: 'average_session_duration',
      filters: { device_category: 'mobile' },
    },
    {
      id: 'top-browsers',
      title: 'Top Browsers',
      description: 'Most common browsers',
      metric: 'users',
      dimension: 'browser',
      sortBy: 'desc',
    },
  ],
};

// ==========================================
// ALL DASHBOARDS COLLECTION
// ==========================================

export const ALL_DASHBOARDS: Dashboard[] = [
  DASHBOARD_LEARNING_OVERVIEW,
  DASHBOARD_CHAPTER_PERFORMANCE,
  DASHBOARD_LEARNING_PROGRESSION,
  DASHBOARD_RESOURCE_ENGAGEMENT,
  DASHBOARD_TRAFFIC,
  DASHBOARD_DEVICE_ANALYSIS,
];

// ==========================================
// UTILITY FUNCTIONS
// ==========================================

/**
 * Get dashboard by ID
 */
export const getDashboard = (dashboardId: string): Dashboard | undefined => {
  return ALL_DASHBOARDS.find((d) => d.id === dashboardId);
};

/**
 * Get all metric IDs from a dashboard
 */
export const getDashboardMetricIds = (dashboard: Dashboard): string[] => {
  return dashboard.metrics.map((m) => m.id);
};

/**
 * Export dashboard config for GA4 integration
 * Returns a JSON format suitable for GA4 API
 */
export const exportDashboardConfig = (dashboard: Dashboard) => {
  return {
    dashboard_id: dashboard.id,
    title: dashboard.title,
    description: dashboard.description,
    update_frequency: dashboard.updateFrequency,
    metrics_count: dashboard.metrics.length,
    metrics: dashboard.metrics.map((m) => ({
      id: m.id,
      title: m.title,
      metric: m.metric,
      dimension: m.dimension,
      filters: m.filters,
      target: m.target,
    })),
  };
};

// ==========================================
// ALERTS & THRESHOLDS
// ==========================================

export interface AlertRule {
  id: string;
  name: string;
  description: string;
  metric: string;
  condition: 'above' | 'below' | 'equals';
  threshold: number;
  action: string;
}

export const ALERT_RULES: AlertRule[] = [
  {
    id: 'high-bounce-rate',
    name: 'High Bounce Rate Alert',
    description: 'Alert when bounce rate exceeds 60%',
    metric: 'bounce_rate',
    condition: 'above',
    threshold: 60,
    action: 'Investigate chapter content quality',
  },
  {
    id: 'low-engagement',
    name: 'Low Engagement Alert',
    description: 'Alert when avg session duration drops below 5 minutes',
    metric: 'average_session_duration',
    condition: 'below',
    threshold: 5,
    action: 'Review content for disengaging sections',
  },
  {
    id: 'drop-off-spike',
    name: 'Drop-off Spike Alert',
    description: 'Alert when drop-off rate increases >10%',
    metric: 'drop_off_rate',
    condition: 'above',
    threshold: 10,
    action: 'Check for technical issues or content problems',
  },
  {
    id: 'low-returning-users',
    name: 'Low Retention Alert',
    description: 'Alert when returning user rate drops below 20%',
    metric: 'returning_user_rate',
    condition: 'below',
    threshold: 20,
    action: 'Consider adding community features or email engagement',
  },
];

// ==========================================
// WEEKLY MONITORING CHECKLIST
// ==========================================

export const WEEKLY_CHECKLIST = [
  {
    day: 'Monday',
    task: 'Check Learning Overview dashboard',
    description: 'Review overall engagement metrics and traffic',
  },
  {
    day: 'Tuesday',
    task: 'Analyze Chapter Performance',
    description: 'Find top chapters and chapters with high bounce rate',
  },
  {
    day: 'Wednesday',
    task: 'Review Learning Progression',
    description: 'Check drop-off points and completion rates',
  },
  {
    day: 'Thursday',
    task: 'Assess Resource Engagement',
    description: 'See which external resources are most valuable',
  },
  {
    day: 'Friday',
    task: 'Generate Weekly Report',
    description: 'Export data and share insights with team',
  },
];
