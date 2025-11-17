/**
 * Login Page - Dummy authentication with user profiling
 * 
 * Route: /login
 * Query params: ?returnTo=<url> (optional return URL after login)
 */
import React from 'react';
import Layout from '@theme/Layout';
import DummyLoginWithProfile from '../components/ContentTabs/DummyLoginWithProfile';

export default function LoginPage(): React.ReactElement {
  return (
    <Layout
      title="Login"
      description="Authenticate to access personalized content and AI summaries"
    >
      <DummyLoginWithProfile />
    </Layout>
  );
}
