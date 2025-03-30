#!/bin/bash

source "./git_common.sh"

# Define inactivity threshold (30 days)
MONTH_SECONDS=$((30 * 24 * 60 * 60))
DESKTOP_DIR="$HOME"

# Get the directory of the script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Find the root of the Git repository
REPO_DIR=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)

USERNAME=$(git config --get user.name)
# Create a cron_logs folder
mkdir -p "$REPO_DIR"/cron_logs

# Function to process Git repositories
process_git_repo() {
    local remote_repo="$1"
    local repo_dir="$2"
    echo "($remote_repo) $repo_dir"
    cd "$repo_dir" || return

    # Check write access
    if ! check_write_access "$remote_repo"; then
        return
    fi

     # Check for uncommitted changes
    if ! check_uncommitted_changes ; then
        return
    fi

    # Check last modification time
    last_modified=$(find . -path ./.git -prune -o -type f -printf '%T@\n' 2>/dev/null | sort -n | tail -1 | cut -d. -f1)
    if [ -z "$last_modified" ]; then
        echo "  No files found. Skipping."
        return
    fi

    current_time=$(date +%s)
    time_diff=$((current_time - last_modified))

    if [ "$time_diff" -ge "$MONTH_SECONDS" ]; then
        echo "  Repo inactive for over a month. Auto-committing and pushing..."
        
        
        commit_msg="Auto-commit: Inactivity period exceeded 1 month"
        auto_commit_and_push "$commit_msg" "$remote_repo" 
        
    else
        echo "  Repo active within the last month. Skipping."
    fi
}

# Find all Git repositories and process them
find "$DESKTOP_DIR" -type d -name ".git" | while read -r git_dir; do
    repo_dir=$(dirname "$git_dir")
    cd "$repo_dir" || continue

    git remote | while read -r remote_repo; do
        # Do something with $remote_repo
        remote_url=$(git remote get-url "$remote_repo" 2>/dev/null)

        if  echo "$remote_url" | grep -qE "/$USERNAME/"; then
            process_git_repo "$remote_repo" "$repo_dir" 
        fi
    done

done

echo "Process completed."
