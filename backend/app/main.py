from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.chat import router as chat_router
from app.api.ingest import router as ingest_router
from app.core.config import settings
from app.api import health

app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG Chatbot for Physical AI & Humanoid Robotics book",
    version="1.0.0"
)

# Add CORS middleware
# Allow requests from GitHub Pages and local development
allowed_origins = [
    "https://muzaffar4011.github.io",
    "http://localhost:3000",
    "http://localhost:5173",
    "http://127.0.0.1:3000",
    "http://127.0.0.1:5173",
]

# Add environment variable for additional origins if needed
import os
additional_origins = os.getenv("ALLOWED_ORIGINS", "").split(",")
allowed_origins.extend([origin.strip() for origin in additional_origins if origin.strip()])

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
app.include_router(chat_router, prefix="/api", tags=["chat"])
app.include_router(ingest_router, prefix="/api", tags=["ingest"])
app.include_router(health.router, prefix="/api", tags=["health"])

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API for Physical AI & Humanoid Robotics"}