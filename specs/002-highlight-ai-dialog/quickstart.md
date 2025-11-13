# Quickstart: Highlight Selection AI Dialog

This guide provides instructions to quickly set up and run the Highlight Selection AI Dialog feature.

## 1. Backend Setup (Python)

The backend is responsible for handling AI queries and managing the Gemini API key. It uses Python with FastAPI.

### Prerequisites
- Python 3.13+
- `pip` (Python package installer)

### Installation
1. Navigate to the `backend/` directory:
   ```bash
   cd backend/
   ```
2. Create a virtual environment and install dependencies:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
   pip install -r requirements.txt
   ```

### Configuration
1. Create a `.env` file in the `backend/` directory:
   ```bash
   touch .env
   ```
2. Add your Gemini API key to the `.env` file:
   ```
   GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
   ```
   Replace `YOUR_GEMINI_API_KEY` with your actual Google Gemini API key.

### Running the Backend
1. Ensure your virtual environment is active.
2. Run the FastAPI application:
   ```bash
   uvicorn main:app --reload
   ```
   The backend API will be available at `http://127.0.0.1:8000`.

## 2. Frontend Setup (TypeScript)

The frontend provides the user interface for highlighting text, the AI dialog, and configuration settings. It uses a web framework (e.g., React) with TypeScript.

### Prerequisites
- Node.js (LTS version recommended)
- npm or yarn

### Installation
1. Navigate to the `frontend/` directory:
   ```bash
   cd frontend/
   ```
2. Install dependencies:
   ```bash
   npm install
   # or yarn install
   ```

### Running the Frontend
1. Start the development server:
   ```bash
   npm start
   # or yarn start
   ```
   The frontend application will typically open in your browser at `http://localhost:3000` (or similar).

## 3. Interacting with the Feature

1. Ensure both the backend and frontend servers are running.
2. In the frontend application, highlight any text content.
3. Invoke the AI dialog using the designated mechanism (e.g., right-click context menu, keyboard shortcut - *implementation detail to be determined by frontend development*).
4. The highlighted text should appear in the AI dialog's input field.
5. Type your query and submit it. The AI's response from the `gemini-2.5-flash` model will be displayed in the dialog.

## 4. Configuring the Gemini API Key (Frontend UI)

1. Ensure both the backend and frontend servers are running.
2. In the frontend application, click the "Configure API Key" button.
3. In the Configuration UI, enter your Gemini API Key into the input field.
4. Click the "Save API Key" button.
5. The status will update to indicate if the key was saved successfully and if it's valid.
   - If the key is valid, the input field will show '********' to mask the key.
   - If the key is invalid, an error message will be displayed.
6. You can now close the Configuration UI and use the AI Dialog. The AI Dialog will use the newly configured API key.
