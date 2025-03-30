#!/bin/bash

# Function to check for uncommitted changes
check_uncommitted_changes() {
    if [[ -n $(git status --porcelain) ]]; then
        echo "  Uncommitted changes found."
        return 0  # Changes exist
    fi

    echo "  No uncommitted changes found."
    return 1  # No changes
}

check_write_access(){
    local remote_repo="$1"

    if ! git push --dry-run "$remote_repo" HEAD >/dev/null 2>&1; then
        echo "  No write access to remote. Skipping."
        return 1
    fi
    return 0
}

files_updates(){
   local changes
    changes=$(git status --porcelain)
    echo "$changes" 
}

file_count(){
    local count
    count=$(git status --porcelain | wc -l)
    echo "$count"
}
# Function to commit and push changes
auto_commit_and_push() {
    # Get system details
    DATE=$(date "+%I:%M %p")
    USERNAME=$(whoami)
    HOSTNAME=$(hostname)
    BRANCH=$(git rev-parse --abbrev-ref HEAD)

    local commit_msg="$1"
    local remote_repo="${2:-origin}"

    local updated_files
    updated_files=$(files_updates)

    local count
    count=$(file_count)

    # Construct commit message
    if [[ -z "$commit_msg" ]]; then
        commit_msg="Auto-commit: $HOSTNAME sync at $DATE by $USERNAME"
    fi


    commit_msg+=" (Updated $count files)"$'\n\n'"Affected files:"$'\n'"$updated_files"

    echo -e "COMMIT MSG -> \n$commit_msg"

    echo "  Committing and pushing changes..."
    git add --all >/dev/null 2>&1
    git commit -m "$commit_msg"
    git push "$remote_repo" "$BRANCH"
    echo "  Changes committed and pushed successfully!"
}
