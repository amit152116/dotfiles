#!/bin/bash

# Define required functions here to ensure they're available
check_write_access() {
    local remote="$1"
    # Try to fetch from remote to check if we have write access
    # Redirecting both stdout and stderr to /dev/null
    git fetch "$remote" &>/dev/null
    return $?
}

# Function to check if there are any uncommitted changes
check_uncommitted_changes() {
    if git status --porcelain | grep -q .; then
        return 0 # Changes exist
    else
        echo "No changes to commit."
        return 1 # No changes
    fi
}

# Function to pull changes from remote repository
git_pull_changes() {
    log_file="$REPO_DIR/cron_logs/auto_commit.log.$(date +%Y%m%d)"

    echo "$(date): Pulling changes from remote repository..." >>"$log_file"

    # Check for uncommitted changes
    local has_changes=0
    if git status --porcelain | grep -q .; then
        has_changes=1
        # Stash changes before pulling
        echo "$(date): Stashing uncommitted changes..." >>"$log_file"
        git stash push -m "Auto stash before pull" >>"$log_file" 2>&1
    fi

    # Get the current branch
    current_branch=$(git rev-parse --abbrev-ref HEAD)

    # Pull changes from the current branch
    pull_result=$(git pull origin "$current_branch" 2>&1)
    pull_status=$?

    if [ $pull_status -eq 0 ]; then
        echo "$(date): Successfully pulled changes from remote." >>"$log_file"
    else
        echo "$(date): Failed to pull changes: $pull_result" >>"$log_file"
    fi

    # Apply stashed changes if necessary
    if [ $has_changes -eq 1 ]; then
        echo "$(date): Restoring stashed changes..." >>"$log_file"
        git stash pop >>"$log_file" 2>&1
    fi

    return $pull_status
}

# Function to auto commit and push changes
auto_commit_and_push() {
    log_file="$REPO_DIR/cron_logs/auto_commit.log.$(date +%Y%m%d)"

    # Create a commit message with the current date or use the provided message
    if [ -n "$1" ]; then
        commit_msg="$1"
    else
        commit_msg="Auto commit $(date)"
    fi

    # Add all changes
    git add -A

    # Commit with the message
    git commit -m "$commit_msg"
    commit_status=$?

    if [ $commit_status -eq 0 ]; then
        echo "$(date): Committed changes with message: $commit_msg" >>"$log_file"

        # Push the changes
        push_result=$(git push 2>&1)
        push_status=$?

        if [ $push_status -eq 0 ]; then
            echo "$(date): Successfully pushed changes to remote." >>"$log_file"
        else
            echo "$(date): Failed to push changes: $push_result" >>"$log_file"
        fi
    else
        echo "$(date): Failed to commit changes." >>"$log_file"
    fi
}
