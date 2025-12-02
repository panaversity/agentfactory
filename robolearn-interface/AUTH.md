# Using Authentication in RoboLearn Interface

## Accessing User Information After Login

After a user logs in, you can access their information using the `useAuth()` hook anywhere in your React components.

### Basic Usage

```tsx
import { useAuth } from '@/contexts/AuthContext';

function MyComponent() {
  const { session, isLoading, signOut } = useAuth();

  // Check if user is logged in
  if (isLoading) {
    return <div>Loading...</div>;
  }

  if (!session) {
    return <div>Please log in</div>;
  }

  // Access user information
  const userId = session.user.id;           // User ID (from OAuth sub claim)
  const email = session.user.email;         // User email
  const name = session.user.name;           // User name (optional)
  const role = session.user.role;           // User role: "admin" | "user" (optional)
  const softwareBackground = session.user.softwareBackground; // Custom field (optional)
  const accessToken = session.accessToken;  // OAuth access token (if needed for API calls)

  return (
    <div>
      <h1>Welcome, {name || email}!</h1>
      <p>User ID: {userId}</p>
      <p>Email: {email}</p>
      {role && <p>Role: {role}</p>}
      <button onClick={() => signOut()}>Sign Out</button>
    </div>
  );
}
```

### User Object Structure

```typescript
interface User {
  id: string;                    // User ID (OAuth sub claim)
  email: string;                // User email
  name?: string;                // User's display name (optional)
  role?: string;                // User role: "admin" | "user" (optional)
  softwareBackground?: string | null; // Custom field from user profile (optional)
}

interface Session {
  user: User;
  accessToken?: string;         // OAuth access token
}
```

### Common Patterns

#### 1. Conditional Rendering Based on Auth State

```tsx
import { useAuth } from '@/contexts/AuthContext';

function ProtectedContent() {
  const { session, isLoading } = useAuth();

  if (isLoading) return <div>Checking authentication...</div>;
  if (!session) return <div>Please log in to view this content</div>;

  return <div>Protected content for {session.user.email}</div>;
}
```

#### 2. Role-Based Access Control

```tsx
import { useAuth } from '@/contexts/AuthContext';

function AdminPanel() {
  const { session, isLoading } = useAuth();

  if (isLoading) return <div>Loading...</div>;
  if (!session) return <div>Please log in</div>;
  if (session.user.role !== 'admin') {
    return <div>Access denied. Admin only.</div>;
  }

  return <div>Admin Panel - User ID: {session.user.id}</div>;
}
```

#### 3. Using Access Token for API Calls

```tsx
import { useAuth } from '@/contexts/AuthContext';

function ApiExample() {
  const { session } = useAuth();

  const callProtectedApi = async () => {
    if (!session?.accessToken) {
      console.error('No access token available');
      return;
    }

    const response = await fetch('https://api.example.com/protected', {
      headers: {
        'Authorization': `Bearer ${session.accessToken}`,
      },
    });

    const data = await response.json();
    console.log('API Response:', data);
  };

  return <button onClick={callProtectedApi}>Call Protected API</button>;
}
```

#### 4. Display User Profile Information

```tsx
import { useAuth } from '@/contexts/AuthContext';

function UserProfile() {
  const { session } = useAuth();

  if (!session) return null;

  const { user } = session;

  return (
    <div>
      <h2>Profile</h2>
      <p><strong>User ID:</strong> {user.id}</p>
      <p><strong>Email:</strong> {user.email}</p>
      {user.name && <p><strong>Name:</strong> {user.name}</p>}
      {user.role && <p><strong>Role:</strong> {user.role}</p>}
      {user.softwareBackground && (
        <p><strong>Software Background:</strong> {user.softwareBackground}</p>
      )}
    </div>
  );
}
```

#### 5. Sign Out Options

```tsx
import { useAuth } from '@/contexts/AuthContext';

function SignOutButton() {
  const { signOut } = useAuth();

  return (
    <div>
      {/* Local sign out - user stays logged in at auth server */}
      <button onClick={() => signOut(false)}>Sign Out (Local)</button>

      {/* Global sign out - logs out from all apps */}
      <button onClick={() => signOut(true)}>Sign Out (Global)</button>
    </div>
  );
}
```

#### 6. Refresh User Data (After Profile Update)

```tsx
import { useAuth } from '@/contexts/AuthContext';
import { useEffect, useState } from 'react';

function ProfilePage() {
  const { session, refreshUserData } = useAuth();
  const [refreshing, setRefreshing] = useState(false);

  // Option A: Auto-refresh on page load
  useEffect(() => {
    refreshUserData();
  }, []);

  // Option B: Manual refresh button
  const handleRefresh = async () => {
    setRefreshing(true);
    const success = await refreshUserData();
    if (success) {
      alert('Profile data refreshed!');
    } else {
      alert('Failed to refresh profile data');
    }
    setRefreshing(false);
  };

  if (!session) return <div>Please log in</div>;

  return (
    <div>
      <h1>Profile</h1>
      <p><strong>Name:</strong> {session.user.name}</p>
      <p><strong>Email:</strong> {session.user.email}</p>
      <p><strong>Software Background:</strong> {session.user.softwareBackground}</p>
      <p><strong>Hardware Tier:</strong> {session.user.hardwareTier}</p>

      <button onClick={handleRefresh} disabled={refreshing}>
        {refreshing ? 'Refreshing...' : 'Refresh Profile Data'}
      </button>

      <a href={`${process.env.NEXT_PUBLIC_AUTH_URL}/account/profile`}>
        Edit Profile on Auth Server
      </a>
    </div>
  );
}
```

**Use Case**: After a user updates their profile on the auth server, call `refreshUserData()` to fetch the latest information without requiring sign out/in.

### Available Properties

The `useAuth()` hook returns:

- **`session`**: `Session | null` - Current session object containing user info and access token
- **`isLoading`**: `boolean` - Whether authentication state is being checked
- **`signOut`**: `(global?: boolean) => void` - Function to sign out the user
- **`refreshUserData`**: `() => Promise<boolean>` - Fetches fresh user data from auth server (returns true on success)

### Notes

- User information is automatically verified using JWKS (client-side) for better performance
- The session is checked on component mount and automatically refreshed if needed
- User data comes from the OAuth ID token (verified) or userinfo endpoint (fallback)
- The `accessToken` can be used to make authenticated API calls to protected endpoints

