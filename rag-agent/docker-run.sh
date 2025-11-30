#!/bin/bash
# Docker run script for local testing with .env variables

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Building Docker image...${NC}"
docker build -t robolearn-backend:latest .

echo -e "\n${GREEN}Starting container with .env variables...${NC}"
docker run -d \
  --name robolearn-backend \
  --rm \
  -p 8000:8000 \
  --env-file .env \
  robolearn-backend:latest

echo -e "\n${GREEN}Container started!${NC}"
echo -e "${BLUE}API available at: http://localhost:8000${NC}"
echo -e "${BLUE}Health check: http://localhost:8000/health${NC}"
echo -e "${BLUE}API docs: http://localhost:8000/docs${NC}"
echo -e "\n${BLUE}To view logs:${NC} docker logs -f robolearn-backend"
echo -e "${BLUE}To stop:${NC} docker stop robolearn-backend"

