# Testing Guide - Personalization Feature

## Prerequisites Setup

### 1. Set Up API Key

```bash
# Navigate to API directory
cd api

# Copy environment template
cp .env.example .env

# Edit .env and add your Google API key
# GOOGLE_API_KEY=your_actual_google_api_key_here
```

**Get Google API Key:**
1. Go to https://aistudio.google.com/apikey
2. Click "Create API Key"
3. Copy the key and paste it in `api/.env`

### 2. Verify Dependencies

```bash
# Ensure you're in the api directory with venv activated
cd api
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies if needed
pip install -r requirements.txt
```

## Backend Testing

### Start the Backend Server

```bash
# From api directory with venv activated
python -m uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload

# You should see:
# INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
# INFO:     Started reloader process
```

### Test Endpoints with cURL or Browser

#### 1. Health Check
```bash
curl http://localhost:8000/health
# Expected: {"status":"healthy","service":"content-personalization-api"}
```

#### 2. API Documentation
Open in browser: http://localhost:8000/docs

You'll see interactive Swagger UI with all endpoints.

#### 3. Test Authentication Endpoint

**Create a dummy login:**
```bash
curl -X POST "http://localhost:8000/api/v1/auth/dummy-login-with-profile" \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Test User",
    "email": "test@example.com",
    "programmingExperience": "Beginner",
    "aiProficiency": "Novice"
  }'

# Expected response:
# {
#   "token": "dummy_token_abc123...",
#   "profile": {
#     "name": "Test User",
#     "email": "test@example.com",
#     "programmingExperience": "Beginner",
#     "aiProficiency": "Novice"
#   }
# }
```

**Copy the token from the response for next tests.**

#### 4. Test Summary Endpoint (SSE Stream)

```bash
# Replace YOUR_TOKEN with the token from step 3
curl -N "http://localhost:8000/api/v1/summarize?pageId=test-page&token=YOUR_TOKEN&content=This%20is%20a%20long%20article%20about%20artificial%20intelligence%20and%20machine%20learning.%20It%20covers%20various%20topics%20including%20neural%20networks%2C%20deep%20learning%2C%20and%20natural%20language%20processing.%20The%20content%20explains%20how%20AI%20systems%20work%20and%20their%20applications%20in%20modern%20technology."

# Expected: Streaming response with chunks
# data: {"chunk":"AI is...","done":false}
# data: {"chunk":" a field...","done":false}
# ...
# data: {"chunk":"","done":true}
```

#### 5. Test Personalization Endpoint (SSE Stream)

```bash
# Replace YOUR_TOKEN with the token from step 3
curl -N "http://localhost:8000/api/v1/personalize?pageId=test-page&token=YOUR_TOKEN&content=This%20is%20a%20tutorial%20about%20Python%20programming.%20It%20covers%20variables%2C%20functions%2C%20and%20object-oriented%20programming.%20The%20content%20includes%20examples%20of%20how%20to%20use%20AI%20agents%20in%20your%20applications.&programmingLevel=Beginner&aiLevel=Novice"

# Expected: Streaming personalized content tailored to Beginner programming + Novice AI
# data: {"chunk":"Let's explore...","done":false}
# ...
# data: {"chunk":"","done":true}
```

### Test Different Proficiency Levels

Try different combinations to see how content adapts:

```bash
# Expert Programming + Expert AI
curl -N "http://localhost:8000/api/v1/personalize?pageId=test&token=YOUR_TOKEN&content=...&programmingLevel=Expert&aiLevel=Expert"

# Novice Programming + Beginner AI
curl -N "http://localhost:8000/api/v1/personalize?pageId=test&token=YOUR_TOKEN&content=...&programmingLevel=Novice&aiLevel=Beginner"
```

**Expected differences:**
- **Novice**: Simple explanations, everyday analogies, no jargon
- **Beginner**: Clear explanations with basic examples
- **Intermediate**: Standard patterns, technical terminology
- **Expert**: Advanced techniques, trade-offs, performance optimizations

## Frontend Testing

### 1. Start Frontend Development Server

```bash
# From project root, navigate to book-source
cd book-source

# Install dependencies (if not done)
npm install

# Start development server
npm start

# Server should start at http://localhost:3000
```

### 2. Test Login Flow

1. Navigate to any page with ContentTabs component
2. Click on **"Summary"** tab
3. You should see "Login to See Summary" button
4. Click the button → redirected to `/login` page
5. Fill in the form:
   - **Name**: Test User
   - **Email**: test@example.com
   - **Programming Experience**: Select any level (e.g., Beginner)
   - **AI Proficiency**: Select any level (e.g., Novice)
6. Click "Login with Profile"
7. Should redirect back to original page

### 3. Test Summary Tab

1. After login, click **"Summary"** tab
2. Content should start streaming immediately
3. Verify:
   - ✅ Streaming indicator appears
   - ✅ Content appears progressively
   - ✅ Auto-scroll works during streaming
   - ✅ Summary completes and shows full content
   - ✅ Cache indicator shows "📋 Cached content" on second visit

### 4. Test Personalization Tab

1. Click **"Personalized"** tab
2. Verify profile badge shows: "Personalized for: Beginner Programming, Novice AI"
3. Click **"Generate Personalized Content"** button
4. Verify:
   - ✅ Button changes to "Generating..."
   - ✅ Streaming indicator appears: "✨ Generating personalized content..."
   - ✅ Content streams progressively
   - ✅ Auto-scroll works
   - ✅ Content complexity matches proficiency level (simpler for Novice)
   - ✅ After completion, "🔄 Regenerate" button appears
   - ✅ Navigate away and back → content loads from cache instantly
   - ✅ Cache indicator shows "📋 Cached content"

### 5. Test Cache with Different Profiles

1. Click profile name (if available) or logout
2. Login with **different proficiency levels**:
   - Example: Expert Programming + Expert AI
3. Navigate to same page
4. Click "Personalized" tab
5. Click "Generate Personalized Content"
6. Verify:
   - ✅ Content is different (more advanced for Expert)
   - ✅ New cache entry created (different cache key)
   - ✅ Previous profile's cache still works when you switch back

### 6. Test Error Handling

**Missing authentication:**
1. Clear sessionStorage (DevTools → Application → Session Storage → Clear)
2. Refresh page
3. Click "Summary" or "Personalized" tab
4. Verify: "Login to See..." button appears

**Network error simulation:**
1. Stop backend server
2. Try generating summary or personalized content
3. Verify:
   - ✅ Error message appears
   - ✅ Partial content preserved (if any was received)
   - ✅ Retry button available

## Browser DevTools Checks

### Console Logs

Open DevTools → Console, you should see:

```
✅ Cache hit for test-page - displaying cached summary
✅ Personalization completed for test-page
💾 Cached personalized content for test-page (Beginner-Novice)
```

### Network Tab

1. Open DevTools → Network
2. Generate personalized content
3. Look for request to `/api/v1/personalize`
4. Click on it → Preview tab
5. Should see SSE events streaming in

### Application Storage

Open DevTools → Application → Session Storage → http://localhost:3000

You should see keys:
- `auth_session` - Contains token and profile
- `summary_test-page` - Cached summary
- `personalized_test-page_Beginner-Novice` - Cached personalized content

## API Testing with Swagger UI

1. Open http://localhost:8000/docs
2. Click on **POST /api/v1/auth/dummy-login-with-profile**
3. Click "Try it out"
4. Fill in the request body:
   ```json
   {
     "name": "Test User",
     "email": "test@example.com",
     "programmingExperience": "Intermediate",
     "aiProficiency": "Beginner"
   }
   ```
5. Click "Execute"
6. Copy the token from response
7. Test **GET /api/v1/summarize** or **GET /api/v1/personalize**
8. Paste token in parameters
9. Add content and other required params
10. Click "Execute" → See streaming response

## Expected Behavior Summary

### Authentication (Phase 1-3) ✅
- Login form accepts 4 fields
- Token generated with `dummy_token_` prefix
- Profile stored in sessionStorage
- Redirect to returnTo page after login

### Summary Tab ✅
- Uses `/api/v1/summarize` endpoint
- Streams content progressively
- Caches result by pageId
- Shows login button if not authenticated

### Personalization Tab ✅
- Uses `/api/v1/personalize` endpoint
- Shows profile badge with proficiency levels
- Streams personalized content
- Caches by pageId + profile fingerprint (e.g., "Beginner-Novice")
- Content complexity matches proficiency
- Different profiles get different cached content

### Caching ✅
- First visit: generates content (slow)
- Second visit: loads from cache (< 500ms)
- Cache indicator shows source
- Different profiles → different cache keys

## Troubleshooting

**Backend won't start:**
- Check GOOGLE_API_KEY is set in `api/.env`
- Verify Python dependencies installed
- Check port 8000 is not already in use

**Frontend can't connect to backend:**
- Verify backend is running on port 8000
- Check CORS_ORIGINS in `api/.env` includes `http://localhost:3000`
- Open browser console for CORS errors

**Streaming doesn't work:**
- Check browser supports EventSource (all modern browsers do)
- Verify token is being sent correctly
- Check backend logs for errors

**Cache not working:**
- Check sessionStorage is enabled in browser
- Verify cache keys match (check Application tab in DevTools)
- Try clearing sessionStorage and testing again

## Success Criteria Checklist

- [ ] Backend starts without errors
- [ ] Health endpoint returns 200
- [ ] Login creates token and profile
- [ ] Summary endpoint streams content
- [ ] Personalize endpoint streams content with proficiency params
- [ ] Frontend login flow works
- [ ] Summary tab shows streaming content
- [ ] Personalization tab shows profile badge
- [ ] Personalized content complexity matches proficiency
- [ ] Cache works for both summary and personalization
- [ ] Different profiles get different cached content
- [ ] Error handling preserves partial content
- [ ] Login button appears when not authenticated
