// Animation sequence data for the Hero IDE simulation
// Each sequence contains steps that will be executed in order

export interface AnimationStep {
    type: 'user-message' | 'agent-message' | 'system-message' | 'code';
    content: string;
    delay: number; // Duration in ms for this step to complete
}

export interface AnimationSequence {
    filename: string;
    language: 'python';
    steps: AnimationStep[];
}

export const pythonSequence: AnimationSequence = {
    filename: 'user_api.py',
    language: 'python',
    steps: [
        {
            type: 'user-message',
            content: 'Create a REST API for user management with authentication',
            delay: 1500,
        },
        {
            type: 'agent-message',
            content: "I'll create a spec-driven FastAPI with JWT authentication and user CRUD operations...",
            delay: 2000,
        },
        {
            type: 'system-message',
            content: '⏳ Generating code...',
            delay: 1000,
        },
        {
            type: 'code',
            content: `from fastapi import FastAPI, Depends
from pydantic import BaseModel
from typing import Optional

app = FastAPI()

class User(BaseModel):
    id: Optional[int] = None
    name: str
    email: str

@app.get("/users")
async def get_users():
    return {"users": []}

@app.post("/users")
async def create_user(user: User):
    return {"created": user}`,
            delay: 5000,
        },
        {
            type: 'agent-message',
            content: '✅ FastAPI endpoint ready with Pydantic validation!',
            delay: 2000,
        },
    ],
};

// Total duration for each sequence (for loop timing)
export const getSequenceDuration = (sequence: AnimationSequence): number => {
    return sequence.steps.reduce((total, step) => total + step.delay, 0);
};
