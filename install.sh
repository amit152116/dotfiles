#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

# Define dotfiles directory
DOTFILES_DIR=$(git rev-parse --show-toplevel 2>/dev/null)
if [ -z "$DOTFILES_DIR" ]; then
    echo "Error: Could not determine dotfiles directory. Exiting..."
    exit 1
fi

# Define variables
GH_KEYRING="/etc/apt/keyrings/githubcli-archive-keyring.gpg"
GH_REPO="https://cli.github.com/packages"
GH_LIST="/etc/apt/sources.list.d/github-cli.list"

# Install GitHub CLI repository if missing
if ! grep -q "^deb .*github.com/packages" "$GH_LIST" 2>/dev/null; then
    echo "Setting up GitHub CLI repository..."
    sudo mkdir -p -m 755 /etc/apt/keyrings
    wget -qO- "$GH_REPO/githubcli-archive-keyring.gpg" | sudo tee "$GH_KEYRING" > /dev/null
    sudo chmod go+r "$GH_KEYRING"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=$GH_KEYRING] $GH_REPO stable main" | sudo tee "$GH_LIST" > /dev/null
fi

# Define list of required packages
PACKAGES=(
   snapd zsh tmux xclip fzf bat ripgrep fd-find curl git wget unzip python3-pip btop gh
)
# Update package lists and install missing packages
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

echo "Installing required packages..."
sudo apt install -y "${PACKAGES[@]}"
sudo apt autoremove -y

# Install Neovim via Snap
echo "Installing Neovim..."
sudo snap install nvim --classic


# Function to safely create symbolic links
createSymlink() {
    local src="$1"
    local dest="$2"

    # Check if destination already exists
    if [ -e "$dest" ] || [ -L "$dest" ]; then
        echo "Backup existing file: $dest"

        # Extract filename and extension safely
        filename=$(basename -- "$dest")
        extension="${filename##*.}"
        if [[ "$filename" == "$extension" ]]; then
            extension=""
        else
            filename="${filename%.*}"
        fi

        backup_name="${dest}.bak"
        if [ -n "$extension" ]; then
            backup_name="${dest%.*}_$(date +%Y%m%d_%H%M%S).bak.$extension"
        fi

        mv "$dest" "$backup_name"
    fi

    # Ensure parent directory exists
    mkdir -p "$(dirname "$dest")"

    # Create symbolic link
    echo "Creating symlink: $dest → $src"
    ln -sfn "$src" "$dest"
}

# Setup Git Configuration
echo "Setting up Git config..."
createSymlink "$DOTFILES_DIR/.gitconfig" "$HOME/.gitconfig"

# Setup btop config
echo "Setting up btop config..."
if [ ! -d "$HOME/.config/btop" ]; then
    mkdir -p "$HOME/.config/btop"
fi
createSymlink "$DOTFILES_DIR/btop" "$HOME/.config/btop"


# Setup Neovim
echo "Setting up Neovim..."
if [ ! -d "$HOME/.config/nvim" ]; then
    mkdir -p "$HOME/.config/nvim"
fi
createSymlink "$DOTFILES_DIR/nvim" "$HOME/.config/nvim"


# Setup Tmux
echo "Setting up Tmux..."
createSymlink "$DOTFILES_DIR/.tmux.conf" "$HOME/.tmux.conf"

# Install TPM (Tmux Plugin Manager)
if [ ! -d "$HOME/.tmux/plugins/tpm" ]; then
    echo "Installing TPM..."
    git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
fi

# Setup Zsh
echo "Setting up Zsh..."
createSymlink "$DOTFILES_DIR/.zshrc" "$HOME/.zshrc"


# Install Oh My Zsh if not installed
if [ ! -d "$HOME/.oh-my-zsh" ]; then
    echo "Installing Oh My Zsh..."
    sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended
fi


# Ensure `fzf` is installed (interactive fuzzy finder)
if command -v fzf &>/dev/null || [ -d "$HOME/.fzf" ]; then
    echo "fzf is already installed. Skipping installation."
else
    echo "Installing fzf..."
    git clone --depth 1 https://github.com/junegunn/fzf.git "$HOME/.fzf"
    "$HOME/.fzf/install" --all
fi

# Install zsh plugins
echo "Installing Zsh plugins..."
ZSH_CUSTOM="$HOME/.oh-my-zsh/custom"
declare -A plugins=(
    ["zsh-autosuggestions"]="https://github.com/zsh-users/zsh-autosuggestions"
    ["zsh-completions"]="https://github.com/zsh-users/zsh-completions"
    ["zsh-history-substring-search"]="https://github.com/zsh-users/zsh-history-substring-search"
    ["alias-tips"]="https://github.com/djui/alias-tips"
    ["zsh-syntax-highlighting"]="https://github.com/zsh-users/zsh-syntax-highlighting"
)
# Loop through plugins and install if missing
for plugin in "${!plugins[@]}"; do
    if [ ! -d "$ZSH_CUSTOM/plugins/$plugin" ]; then
        echo "Installing $plugin..."
        git clone --depth=1 "${plugins[$plugin]}" "$ZSH_CUSTOM/plugins/$plugin"
    else
        echo "$plugin is already installed, skipping..."
    fi
done

# Install Powerlevel10k theme
echo "Installing Powerlevel10k..."
if [ ! -d "$HOME/.oh-my-zsh/custom/themes/powerlevel10k" ]; then
    git clone --depth=1 https://github.com/romkatv/powerlevel10k.git "$ZSH_CUSTOM/themes/powerlevel10k"
fi

# Install IDEAVim Configuration
echo "Setting up IDEAVim..."
createSymlink "$DOTFILES_DIR/.ideavimrc" "$HOME/.ideavimrc"

# Change default shell to Zsh
if [[ "$SHELL" != "$(which zsh)" ]]; then
    echo "Changing default shell to Zsh..."
    if chsh -s "$(which zsh)"; then
        echo "Shell changed successfully! Restart your terminal."
    else
        echo "Failed to change shell. Try running: sudo chsh -s $(which zsh) $USER"
    fi
fi

# Make auto-commit script executable if it exists
SCRIPT_PATH="$DOTFILES_DIR/auto-commit.sh"
if [ -f "$SCRIPT_PATH" ]; then
    chmod +x "$SCRIPT_PATH"

    # Add the script to crontab if not already present
    if ! crontab -l 2>/dev/null | grep -q "$SCRIPT_PATH"; then
        (crontab -l 2>/dev/null; echo "0 22 * * * $SCRIPT_PATH") | crontab -
        echo "Auto-commit script scheduled in cron job successfully!"
    else
        echo "Auto-commit script is already scheduled. Skipping..."
    fi
fi

echo "Dotfiles setup complete! Restart your terminal for changes to take effect."
