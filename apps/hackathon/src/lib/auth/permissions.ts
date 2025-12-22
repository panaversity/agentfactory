/**
 * Role types for hackathon participants
 */
export type RoleType = "organizer" | "manager" | "judge" | "mentor" | "participant";

/**
 * Permission types for hackathon actions
 */
export type Permission =
  | "create_hackathon"
  | "edit_hackathon"
  | "delete_hackathon"
  | "publish_hackathon"
  | "assign_roles"
  | "edit_teams"
  | "judge_submissions"
  | "mentor_teams"
  | "submit_projects"
  | "view_analytics"
  | "view_all_submissions"
  | "manage_criteria";

/**
 * Role permissions mapping
 */
export const ROLE_PERMISSIONS: Record<RoleType, Permission[]> = {
  organizer: [
    "create_hackathon",
    "edit_hackathon",
    "delete_hackathon",
    "publish_hackathon",
    "assign_roles",
    "edit_teams",
    "judge_submissions",
    "mentor_teams",
    "view_analytics",
    "view_all_submissions",
    "manage_criteria",
  ],
  manager: [
    "edit_hackathon",
    "assign_roles",
    "edit_teams",
    "judge_submissions",
    "mentor_teams",
    "view_analytics",
    "view_all_submissions",
    "manage_criteria",
  ],
  judge: ["judge_submissions", "view_all_submissions"],
  mentor: ["mentor_teams"],
  participant: ["submit_projects"],
};

/**
 * Check if a role has a specific permission
 */
export function hasPermission(role: RoleType, permission: Permission): boolean {
  return ROLE_PERMISSIONS[role]?.includes(permission) ?? false;
}

/**
 * Check if any of the user's roles has the permission
 */
export function can(roles: RoleType[], permission: Permission): boolean {
  return roles.some((role) => hasPermission(role, permission));
}

/**
 * Get all permissions for a role
 */
export function getPermissions(role: RoleType): Permission[] {
  return ROLE_PERMISSIONS[role] ?? [];
}

/**
 * Get all permissions for multiple roles (merged)
 */
export function getMergedPermissions(roles: RoleType[]): Permission[] {
  const permissions = new Set<Permission>();
  for (const role of roles) {
    for (const permission of getPermissions(role)) {
      permissions.add(permission);
    }
  }
  return Array.from(permissions);
}

/**
 * Validate role type
 */
export function isValidRole(role: string): role is RoleType {
  return ["organizer", "manager", "judge", "mentor", "participant"].includes(role);
}

/**
 * Roles that can be assigned (organizer is auto-assigned to creator)
 */
export const ASSIGNABLE_ROLES: RoleType[] = [
  "manager",
  "judge",
  "mentor",
  "participant",
];
