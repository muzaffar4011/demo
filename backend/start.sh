#!/bin/bash
# Startup script for Render deployment
# This script ensures the backend starts correctly on Render

# Change to backend directory
cd backend || exit 1

# Run the application
exec uvicorn app.main:app --host 0.0.0.0 --port ${PORT:-8000}

