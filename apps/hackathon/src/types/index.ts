import type { RoleType, Permission } from "@/lib/auth/permissions";

/**
 * User data from SSO JWT claims
 */
export interface UserFromSession {
  id: string;
  email: string;
  name: string;
  organizationId: string;
  organizationName?: string;
}

/**
 * Session data structure
 */
export interface SessionData {
  isLoggedIn: boolean;
  user?: UserFromSession;
  accessToken?: string;
  idToken?: string;
  expiresAt?: number;
  oauthState?: string;
  oauthCodeVerifier?: string;
}

/**
 * User with their hackathon-specific roles
 */
export interface UserWithRoles extends UserFromSession {
  roles: RoleType[];
  permissions: Permission[];
}

/**
 * Hackathon status enum
 */
export type HackathonStatus =
  | "draft"
  | "open"
  | "active"
  | "judging"
  | "completed";

/**
 * Team status enum
 */
export type TeamStatus = "forming" | "ready" | "submitted" | "disqualified";

/**
 * Submission status enum
 */
export type SubmissionStatus = "submitted" | "under_review" | "scored";

/**
 * Team member role in team
 */
export type TeamMemberRole = "leader" | "member";

/**
 * API response wrapper
 */
export interface ApiResponse<T> {
  data?: T;
  error?: string;
  message?: string;
}

/**
 * Pagination params
 */
export interface PaginationParams {
  page?: number;
  limit?: number;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  data: T[];
  total: number;
  page: number;
  limit: number;
  hasMore: boolean;
}

/**
 * Hackathon with computed status
 */
export interface HackathonWithStatus {
  id: string;
  title: string;
  slug: string;
  description: string;
  status: HackathonStatus;
  published: boolean;
  startDate: Date;
  endDate: Date;
  registrationDeadline: Date;
  submissionDeadline: Date;
  minTeamSize: number;
  maxTeamSize: number;
  // Computed
  isRegistrationOpen: boolean;
  isSubmissionOpen: boolean;
  daysUntilStart: number;
  daysUntilEnd: number;
}

/**
 * Team with member count
 */
export interface TeamWithMemberCount {
  id: string;
  name: string;
  description: string | null;
  status: TeamStatus;
  inviteCode: string;
  leaderId: string;
  hackathonId: string;
  memberCount: number;
  maxSize: number;
  minSize: number;
  isReady: boolean;
}
