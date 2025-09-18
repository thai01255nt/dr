#!/bin/bash

# Script to wait for drone takeoff completion
# Usage: wait_for_takeoff.sh [timeout_seconds]

TIMEOUT=${1:-120}  # Default 2 minutes timeout
FLAG_FILE="/tmp/drone_takeoff_complete"
COUNTER=0

echo "⏳ Waiting for drone takeoff completion..."
echo "👀 Monitoring flag file: $FLAG_FILE"
echo "⏰ Timeout: ${TIMEOUT} seconds"

# Remove any existing flag file
rm -f $FLAG_FILE

while [ $COUNTER -lt $TIMEOUT ]; do
    if [ -f "$FLAG_FILE" ]; then
        echo "✅ Drone takeoff completed!"
        echo "🚀 Ready to start navigation nodes..."
        exit 0
    fi
    
    # Print status every 10 seconds
    if [ $((COUNTER % 10)) -eq 0 ]; then
        echo "⏳ Still waiting... (${COUNTER}/${TIMEOUT}s)"
    fi
    
    sleep 1
    COUNTER=$((COUNTER + 1))
done

echo "❌ Timeout waiting for drone takeoff!"
echo "🔍 Check auto_takeoff_node logs for issues"
exit 1
