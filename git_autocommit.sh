#!/bin/bash
source "./git_common.sh"

# Define inactivity threshold (30 days)
MONTH_SECONDS=$((30 * 24 * 60 * 60))

# Get the directory of the script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

USERNAME=$(git config --get user.name)
if [ -z "$USERNAME" ]; then
    echo "Error: Git username not configured"
    exit 1
fi

# Create a cron_logs folder in the script directory
mkdir -p "$SCRIPT_DIR/cron_logs"
LOG_FILE="$SCRIPT_DIR/cron_logs/autocommit_$(date +%Y%m%d_%H%M%S).log"

# Redirect output to both console and log file
exec > >(tee -a "$LOG_FILE") 2>&1

echo "Starting auto-commit process at $(date)"
echo "Username: $USERNAME"
echo "Scanning home directory for git repositories..."

# Function to process Git repositories
process_git_repo() {
    local repo_dir="$1"

    cd "$repo_dir" || return

    echo "-----------------------------------------"
    echo "Processing repository: $repo_dir"

    # Check for remotes owned by the current user
    git remote | while read -r remote_repo; do
        remote_url=$(git remote get-url "$remote_repo" 2>/dev/null)

        if echo "$remote_url" | grep -qE "/$USERNAME/"; then
            echo "Found matching remote ($remote_repo): $remote_url"

            # Check write access
            if ! check_write_access "$remote_repo"; then
                echo "  No write access to $remote_repo. Skipping."
                continue
            fi

            # Check for uncommitted changes
            if ! check_uncommitted_changes; then
                echo "  Repository has no changes to commit. Skipping."
                continue
            fi

            # Check last modification time - excluding .git directory
            last_modified=$(find . -not -path "./.git*" -type f -printf '%T@\n' 2>/dev/null | sort -n | tail -1 | cut -d. -f1)
            if [ -z "$last_modified" ]; then
                echo "  No files found. Skipping."
                continue
            fi

            current_time=$(date +%s)
            time_diff=$((current_time - last_modified))

            if [ "$time_diff" -ge "$MONTH_SECONDS" ]; then
                echo "  Repo inactive for over a month. Auto-committing and pushing..."

                commit_msg="Auto-commit: Inactivity period exceeded 1 month"
                continue
                auto_commit_and_push "$commit_msg" "$remote_repo"

            else
                days_inactive=$((time_diff / 86400))
                echo "  Repo active within the last month ($days_inactive days ago). Skipping."
            fi
        fi
    done
}

echo "Finding all git repositories in $HOME..."
echo "(This may take a while depending on the number of files in your home directory)"

# Use find to locate all git repositories in the home directory
# The -prune option ensures we don't look inside .git directories themselves
REPOS_FOUND=0

# Find all .git directories, filter out submodules by checking for .git/config
find "$HOME" -type d -name ".git" -not -path "*/\.*/.git" -prune 2>/dev/null | while read -r git_dir; do
    # Extract the parent directory which is the actual git repository
    repo_dir=$(dirname "$git_dir")

    # Skip directories we don't have access to
    if [ ! -r "$repo_dir" ] || [ ! -x "$repo_dir" ]; then
        echo "Skipping $repo_dir - no read access"
        continue
    fi

    # Check if it's a valid git repository
    if [ -f "$git_dir/config" ]; then
        ((REPOS_FOUND++))
        process_git_repo "$repo_dir"
    fi
done
