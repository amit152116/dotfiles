#!/bin/bash

# Get the directory of the script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Find the root of the Git repository
REPO_DIR=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)

# Navigate to the repository
cd "$REPO_DIR" || exit 1

# Check for changes
if [[ -n $(git status --porcelain) ]]; then
    echo "Changes detected. Committing and pushing..."

    # Get system details
    DATE=$(date "+%Y-%m-%d %H:%M:%S")
    USERNAME=$(whoami)
    HOSTNAME=$(hostname)

    # Construct commit message
    COMMIT_MSG="Auto-commit on $DATE by $USERNAME@$HOSTNAME"
    # Add, commit, and push changes
    git add .
    git commit -m "$COMMIT_MSG"
    git push origin "$(git rev-parse --abbrev-ref HEAD)"

    echo "Changes committed and pushed successfully!"
else
    echo "No changes detected. Skipping commit."
fi
