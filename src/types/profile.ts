/**
 * Profile Types - Single Source of Truth
 *
 * TODO: Migrate to member.metadata in Proposal 001 (tenant-specific fields)
 * For now, these are stored in user.additionalFields for single-tenant use
 */

export type SoftwareBackground = "beginner" | "intermediate" | "advanced";

export type HardwareTier = "tier1" | "tier2" | "tier3" | "tier4";

/**
 * Gender options for profile (003-user-profile-fields)
 */
export type Gender = "male" | "female" | "other" | "prefer_not_to_say";

/**
 * Profile data structure used in signup and profile management
 */
export interface ProfileData {
  softwareBackground: SoftwareBackground;
  hardwareTier: HardwareTier;
  // Additional profile fields (003-user-profile-fields)
  gender?: Gender;
  fatherName?: string;
  city?: string;
  country?: string;
  phoneNumber?: string;
}
