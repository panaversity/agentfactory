from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from backend.api import ai_generation

app = FastAPI()

origins = [
    "http://localhost:3000",  # Docusaurus development server
    # Add other origins as needed for production deployment
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(ai_generation.router)

@app.get("/")
async def read_root():
    return {"message": "FastAPI AI Service is running"}
