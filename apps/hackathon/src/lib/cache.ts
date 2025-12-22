import { unstable_cache } from "next/cache";
import { revalidateTag } from "next/cache";

/**
 * Cache tags for the hackathon platform
 * Using a structured naming convention: entity:scope:id
 */
export const CacheTags = {
  // Hackathon tags
  hackathons: () => "hackathons",
  hackathon: (id: string) => `hackathon:${id}`,
  hackathonsByOrg: (orgId: string) => `hackathons:org:${orgId}`,
  publishedHackathons: () => "hackathons:published",

  // Team tags
  teams: (hackathonId: string) => `teams:${hackathonId}`,
  team: (teamId: string) => `team:${teamId}`,

  // Submission tags
  submissions: (hackathonId: string) => `submissions:${hackathonId}`,
  submission: (submissionId: string) => `submission:${submissionId}`,

  // Judging criteria tags
  criteria: (hackathonId: string) => `criteria:${hackathonId}`,

  // Hackathon roles tags
  roles: (hackathonId: string) => `roles:${hackathonId}`,

  // Role/stats tags
  hackathonStats: (hackathonId: string) => `stats:${hackathonId}`,
} as const;

/**
 * Revalidate cache tags after mutations
 * Using 'max' cacheLife profile for stale-while-revalidate behavior (Next.js 16+)
 */
export const invalidateCache = {
  // When a hackathon is created
  onHackathonCreate: (orgId: string) => {
    revalidateTag(CacheTags.hackathons(), "max");
    revalidateTag(CacheTags.hackathonsByOrg(orgId), "max");
  },

  // When a hackathon is updated
  onHackathonUpdate: (id: string, orgId: string) => {
    revalidateTag(CacheTags.hackathon(id), "max");
    revalidateTag(CacheTags.hackathons(), "max");
    revalidateTag(CacheTags.hackathonsByOrg(orgId), "max");
    revalidateTag(CacheTags.hackathonStats(id), "max");
  },

  // When a hackathon is published/unpublished
  onHackathonPublish: (id: string, orgId: string) => {
    revalidateTag(CacheTags.hackathon(id), "max");
    revalidateTag(CacheTags.publishedHackathons(), "max");
    revalidateTag(CacheTags.hackathonsByOrg(orgId), "max");
  },

  // When a hackathon is deleted
  onHackathonDelete: (orgId: string) => {
    revalidateTag(CacheTags.hackathons(), "max");
    revalidateTag(CacheTags.hackathonsByOrg(orgId), "max");
    revalidateTag(CacheTags.publishedHackathons(), "max");
  },

  // When a team is created or someone joins
  onTeamChange: (hackathonId: string) => {
    revalidateTag(CacheTags.teams(hackathonId), "max");
    revalidateTag(CacheTags.hackathonStats(hackathonId), "max");
    revalidateTag(CacheTags.hackathon(hackathonId), "max");
  },

  // When a submission is created or updated
  onSubmissionChange: (hackathonId: string, submissionId?: string) => {
    revalidateTag(CacheTags.submissions(hackathonId), "max");
    if (submissionId) {
      revalidateTag(CacheTags.submission(submissionId), "max");
    }
  },

  // When judging criteria change
  onCriteriaChange: (hackathonId: string) => {
    revalidateTag(CacheTags.criteria(hackathonId), "max");
  },

  // When hackathon roles change
  onRoleChange: (hackathonId: string) => {
    revalidateTag(CacheTags.roles(hackathonId), "max");
  },
};

/**
 * Create a cached query with proper tags
 * Usage:
 *   const getHackathon = cachedQuery(
 *     ['hackathon', id],
 *     () => db.select()...,
 *     [CacheTags.hackathon(id)]
 *   );
 */
export function cachedQuery<T>(
  keyParts: string[],
  fn: () => Promise<T>,
  tags: string[],
  revalidate?: number | false
) {
  return unstable_cache(fn, keyParts, {
    tags,
    revalidate: revalidate ?? false, // Default: cache indefinitely, invalidate via tags
  });
}
