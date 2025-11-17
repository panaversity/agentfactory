/**
 * DummyLoginWithProfile Component - Enhanced authentication with user profiling
 * 
 * Collects user information and proficiency levels during login:
 * - Name
 * - Email
 * - Programming Experience (Novice, Beginner, Intermediate, Expert)
 * - AI Proficiency (Novice, Beginner, Intermediate, Expert)
 */
import React, { useState, FormEvent } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import { UserProfile, ProficiencyLevel } from '../../types/contentTabs';
import * as authService from '../../services/authService';
import styles from './styles.module.css';

const PROFICIENCY_LEVELS: ProficiencyLevel[] = ['Novice', 'Beginner', 'Intermediate', 'Expert'];

interface FormErrors {
  name?: string;
  email?: string;
  programmingExperience?: string;
  aiProficiency?: string;
}

export default function DummyLoginWithProfile(): React.ReactElement {
  const history = useHistory();
  const location = useLocation();
  
  // T024: 4-field form state
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [programmingExperience, setProgrammingExperience] = useState<ProficiencyLevel | ''>('');
  const [aiProficiency, setAiProficiency] = useState<ProficiencyLevel | ''>('');
  
  // T027: Inline error messages
  const [errors, setErrors] = useState<FormErrors>({});
  
  // T030: Loading state
  const [isLoading, setIsLoading] = useState(false);
  const [submitError, setSubmitError] = useState<string | null>(null);

  // T026: Form validation
  const validateForm = (): boolean => {
    const newErrors: FormErrors = {};
    
    // Email regex from spec: ^[\w\.-]+@[\w\.-]+\.\w+$
    const emailRegex = /^[\w\.-]+@[\w\.-]+\.\w+$/;
    
    if (!name.trim()) {
      newErrors.name = 'Name is required';
    }
    
    if (!email.trim()) {
      newErrors.email = 'Email is required';
    } else if (!emailRegex.test(email)) {
      newErrors.email = 'Invalid email format';
    }
    
    if (!programmingExperience) {
      newErrors.programmingExperience = 'Programming experience is required';
    }
    
    if (!aiProficiency) {
      newErrors.aiProficiency = 'AI proficiency is required';
    }
    
    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  // T028: Form submission handler
  const handleSubmit = async (e: FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    setSubmitError(null);
    
    // T027: Prevent submission until valid
    if (!validateForm()) {
      return;
    }
    
    // T030: Disable submit button, show spinner
    setIsLoading(true);
    
    try {
      // T028: Call POST /api/v1/auth/dummy-login-with-profile
      const apiUrl = typeof window !== 'undefined'
        ? (window as any).REACT_APP_API_BASE_URL || 'http://localhost:8000'
        : 'http://localhost:8000';
      
      const response = await fetch(`${apiUrl}/api/v1/auth/dummy-login-with-profile`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          name: name.trim(),
          email: email.trim(),
          programmingExperience,
          aiProficiency,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Login failed');
      }

      const data = await response.json();

      // T029: Save session and redirect
      // Map backend response (snake_case) to frontend format (camelCase)
      const profile: UserProfile = {
        name: data.user.name,
        email: data.user.email,
        programmingExperience: data.user.programming_experience || data.user.programmingExperience,
        aiProficiency: data.user.ai_proficiency || data.user.aiProficiency,
      };
      
      authService.saveSession(data.token, profile);

      // T036: Redirect to returnTo URL or home
      const searchParams = new URLSearchParams(location.search);
      const returnTo = searchParams.get('returnTo') || '/';
      history.push(returnTo);
      
    } catch (error) {
      console.error('Login error:', error);
      setSubmitError(error instanceof Error ? error.message : 'Login failed. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.loginContainer}>
      <div className={styles.loginBox}>
        <h2>Welcome! Let's Personalize Your Learning</h2>
        <p className={styles.loginDescription}>
          Tell us about your experience so we can tailor content to your level.
        </p>

        <form onSubmit={handleSubmit} className={styles.loginForm}>
          {/* Name field */}
          <div className={styles.formGroup}>
            <label htmlFor="name">
              Name <span className={styles.required}>*</span>
            </label>
            <input
              id="name"
              type="text"
              value={name}
              onChange={(e) => setName(e.target.value)}
              className={errors.name ? styles.inputError : ''}
              disabled={isLoading}
              placeholder="Enter your name"
            />
            {errors.name && <span className={styles.errorMessage}>{errors.name}</span>}
          </div>

          {/* Email field */}
          <div className={styles.formGroup}>
            <label htmlFor="email">
              Email <span className={styles.required}>*</span>
            </label>
            <input
              id="email"
              type="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              className={errors.email ? styles.inputError : ''}
              disabled={isLoading}
              placeholder="your.email@example.com"
            />
            {errors.email && <span className={styles.errorMessage}>{errors.email}</span>}
          </div>

          {/* T025: Programming Experience dropdown */}
          <div className={styles.formGroup}>
            <label htmlFor="programmingExperience">
              Programming Experience <span className={styles.required}>*</span>
            </label>
            <select
              id="programmingExperience"
              value={programmingExperience}
              onChange={(e) => setProgrammingExperience(e.target.value as ProficiencyLevel)}
              className={errors.programmingExperience ? styles.inputError : ''}
              disabled={isLoading}
            >
              <option value="">Select your level</option>
              {PROFICIENCY_LEVELS.map((level) => (
                <option key={level} value={level}>
                  {level}
                </option>
              ))}
            </select>
            {errors.programmingExperience && (
              <span className={styles.errorMessage}>{errors.programmingExperience}</span>
            )}
          </div>

          {/* T025: AI Proficiency dropdown */}
          <div className={styles.formGroup}>
            <label htmlFor="aiProficiency">
              AI Proficiency <span className={styles.required}>*</span>
            </label>
            <select
              id="aiProficiency"
              value={aiProficiency}
              onChange={(e) => setAiProficiency(e.target.value as ProficiencyLevel)}
              className={errors.aiProficiency ? styles.inputError : ''}
              disabled={isLoading}
            >
              <option value="">Select your level</option>
              {PROFICIENCY_LEVELS.map((level) => (
                <option key={level} value={level}>
                  {level}
                </option>
              ))}
            </select>
            {errors.aiProficiency && (
              <span className={styles.errorMessage}>{errors.aiProficiency}</span>
            )}
          </div>

          {/* Submit error message */}
          {submitError && (
            <div className={styles.submitError} role="alert">
              {submitError}
            </div>
          )}

          {/* T030: Submit button with loading state */}
          <button
            type="submit"
            className={styles.submitButton}
            disabled={isLoading}
          >
            {isLoading ? (
              <>
                <span className={styles.spinner} />
                Logging in...
              </>
            ) : (
              'Login (Demo)'
            )}
          </button>

          <p className={styles.disclaimer}>
            Note: This is a temporary dummy authentication for demonstration purposes.
            <br />
            SSO integration will be implemented in a future update.
          </p>
        </form>
      </div>
    </div>
  );
}
