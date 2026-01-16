#!/bin/bash
echo "=== Fixing Docker storage ==="

# Fix overlay error
echo "1. Cleaning build cache..."
docker builder prune -f

# Stop all containers
echo "2. Stopping containers..."
docker stop $(docker ps -q) 2>/dev/null

# Remove stopped containers
echo "3. Removing stopped containers..."
docker container prune -f

# Remove unused images
echo "4. Removing unused images..."
docker image prune -a -f

echo ""
echo "=== Current disk usage ==="
docker system df

echo ""
echo "Cleanup complete!"