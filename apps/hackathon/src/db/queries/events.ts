import { and, eq, asc, gte, lte, desc } from "drizzle-orm";
import { db } from "@/db";
import { hackathonEvents, type NewHackathonEvent } from "@/db/schema";

/**
 * Event types for the hackathon schedule
 */
export type EventType =
  | "kickoff"
  | "workshop"
  | "qa_session"
  | "submission_deadline"
  | "judging"
  | "awards"
  | "livestream"
  | "networking"
  | "other";

/**
 * Get all events for a hackathon (ordered by start time)
 */
export async function getEventsByHackathon(hackathonId: string) {
  return db
    .select()
    .from(hackathonEvents)
    .where(eq(hackathonEvents.hackathonId, hackathonId))
    .orderBy(asc(hackathonEvents.startTime), asc(hackathonEvents.order));
}

/**
 * Get only published events for a hackathon (for public display)
 */
export async function getPublishedEvents(hackathonId: string) {
  return db
    .select()
    .from(hackathonEvents)
    .where(
      and(
        eq(hackathonEvents.hackathonId, hackathonId),
        eq(hackathonEvents.published, true)
      )
    )
    .orderBy(asc(hackathonEvents.startTime), asc(hackathonEvents.order));
}

/**
 * Get upcoming events (events starting from now)
 */
export async function getUpcomingEvents(hackathonId: string) {
  const now = new Date();
  return db
    .select()
    .from(hackathonEvents)
    .where(
      and(
        eq(hackathonEvents.hackathonId, hackathonId),
        eq(hackathonEvents.published, true),
        gte(hackathonEvents.startTime, now)
      )
    )
    .orderBy(asc(hackathonEvents.startTime), asc(hackathonEvents.order));
}

/**
 * Get past events with recordings
 */
export async function getPastEventsWithRecordings(hackathonId: string) {
  const now = new Date();
  return db
    .select()
    .from(hackathonEvents)
    .where(
      and(
        eq(hackathonEvents.hackathonId, hackathonId),
        eq(hackathonEvents.published, true),
        lte(hackathonEvents.startTime, now)
      )
    )
    .orderBy(desc(hackathonEvents.startTime));
}

/**
 * Get events by type
 */
export async function getEventsByType(hackathonId: string, eventType: EventType) {
  return db
    .select()
    .from(hackathonEvents)
    .where(
      and(
        eq(hackathonEvents.hackathonId, hackathonId),
        eq(hackathonEvents.eventType, eventType)
      )
    )
    .orderBy(asc(hackathonEvents.startTime));
}

/**
 * Get a single event by ID
 */
export async function getEventById(eventId: string) {
  const result = await db
    .select()
    .from(hackathonEvents)
    .where(eq(hackathonEvents.id, eventId))
    .limit(1);

  return result[0] || null;
}

/**
 * Create a new event
 */
export async function createEvent(data: NewHackathonEvent) {
  const result = await db.insert(hackathonEvents).values(data).returning();
  return result[0];
}

/**
 * Update an event
 */
export async function updateEvent(
  eventId: string,
  data: Partial<Omit<NewHackathonEvent, "id" | "hackathonId" | "organizationId" | "createdBy" | "createdAt">>
) {
  const result = await db
    .update(hackathonEvents)
    .set(data)
    .where(eq(hackathonEvents.id, eventId))
    .returning();

  return result[0];
}

/**
 * Delete an event
 */
export async function deleteEvent(eventId: string) {
  await db.delete(hackathonEvents).where(eq(hackathonEvents.id, eventId));
}

/**
 * Bulk create events (for importing schedule)
 */
export async function bulkCreateEvents(events: NewHackathonEvent[]) {
  if (events.length === 0) return [];
  const result = await db.insert(hackathonEvents).values(events).returning();
  return result;
}

/**
 * Get the next upcoming livestream
 */
export async function getNextLivestream(hackathonId: string) {
  const now = new Date();
  const result = await db
    .select()
    .from(hackathonEvents)
    .where(
      and(
        eq(hackathonEvents.hackathonId, hackathonId),
        eq(hackathonEvents.published, true),
        gte(hackathonEvents.startTime, now),
        eq(hackathonEvents.eventType, "livestream")
      )
    )
    .orderBy(asc(hackathonEvents.startTime))
    .limit(1);

  return result[0] || null;
}
