#!/bin/bash

set -e # Exit immediately if a command exits with a non-zero status

# Store the actual user
ACTUAL_USER=${SUDO_USER:-$USER}
ACTUAL_HOME=$(eval echo ~$ACTUAL_USER)

# Check if running with sudo/root privileges
# if [ "$EUID" -ne 0 ]; then
#     echo "This script requires root privileges. Please run with sudo."
#     exit 1
# fi

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
    mkdir -p -m 755 /etc/apt/keyrings
    wget -qO- "$GH_REPO/githubcli-archive-keyring.gpg" | tee "$GH_KEYRING" >/dev/null
    chmod go+r "$GH_KEYRING"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=$GH_KEYRING] $GH_REPO stable main" | tee "$GH_LIST" >/dev/null
fi

# Define list of required packages
PACKAGES=(
    zsh tmux xclip fzf ripgrep fd-find curl git wget unzip python3-pip btop gh python3-pygments bat
)
# Update package lists and install missing packages
echo "Updating system packages..."
sudo apt update && apt upgrade -y

echo "Installing required packages..."
sudo apt install -y "${PACKAGES[@]}"
sudo apt autoremove -y

# Function to safely create symbolic links
createSymlink() {
    local src="$1"
    local dest="$2"

    # Check if the destination already exists
    if [ -e "$dest" ] || [ -L "$dest" ]; then
        # If the destination is already a symlink pointing to the correct source, do nothing
        if [ -L "$dest" ] && [ "$(readlink -- "$dest")" = "$src" ]; then
            echo "Symlink already exists: $dest → $src (No changes needed)"
            return 0
        fi

        echo "Backup existing file: $dest"

        # Handle files that start with a dot (e.g., .gitconfig)
        filename=$(basename -- "$dest")

        if [[ "$filename" == .* ]]; then
            backup_name="${dest}.bak"
        else
            extension="${filename##*.}"
            base="${filename%.*}"
            if [[ "$base" == "$filename" ]]; then
                backup_name="${dest}.bak"
            else
                backup_name="${dest%.*}.bak.$extension"
            fi
        fi

        mv "$dest" "$backup_name"
    fi

    # Ensure parent directory exists
    mkdir -p "$(dirname "$dest")"

    # Create symbolic link
    ln -s "$src" "$dest"
    echo "Symlink created: $dest → $src"
}

# Setup Git Configuration
echo "Setting up Git config..."
createSymlink "$DOTFILES_DIR/.gitconfig" "$HOME/.gitconfig"

# Setup btop config
echo "Setting up btop config..."
mkdir -p "$ACTUAL_HOME/.config/btop"
createSymlink "$DOTFILES_DIR/btop" "$ACTUAL_HOME/.config/btop"

# Setup Tmux
echo "Setting up Tmux..."
createSymlink "$DOTFILES_DIR/.tmux.conf" "$ACTUAL_HOME/.tmux.conf"

# Ask user whether to install Neovim
read -r -p "Do you want to install Neovim? (y/n): " choice
if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
    sudo apt install snapd -y
    # Install Neovim via Snap
    echo "Installing Neovim..."
    sudo snap install nvim --classic

    echo "Setting up Neovim..."
    mkdir -p "$ACTUAL_HOME/.config/nvim"
    createSymlink "$DOTFILES_DIR/nvim" "$ACTUAL_HOME/.config/nvim"
fi

# Ask user whether to install Docker
read -r -p "Do you want to install Docker? (y/n): " choice
if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
    # Install Docker
    curl -sSL https://get.docker.com | sh
    usermod -aG docker "$ACTUAL_USER"
fi

# Install TPM (Tmux Plugin Manager)
if [ ! -d "$ACTUAL_HOME/.tmux/plugins/tpm" ]; then
    echo "Installing TPM..."
    git clone https://github.com/tmux-plugins/tpm $ACTUAL_HOME/.tmux/plugins/tpm
fi

# Setup Zsh
echo "Setting up Zsh..."
createSymlink "$DOTFILES_DIR/.zshrc" "$ACTUAL_HOME/.zshrc"

# Ask user whether to install OMZ
read -r -p "Do you want to install OMZ? (y/n): " choice
if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
    # Install Oh My Zsh if not installed
    if [ ! -d "$ACTUAL_HOME/.oh-my-zsh" ]; then
        echo "Installing Oh My Zsh..."
        su -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh) --unattended" - "$ACTUAL_USER"
    fi
fi

# Ensure `fzf` is installed (interactive fuzzy finder)
if command -v fzf &>/dev/null || [ -d "$ACTUAL_HOME/.fzf" ]; then
    echo "fzf is already installed. Skipping installation."
else

    # Ask user whether to install OMZ
    read -r -p "Do you want to install interactive fzf? (y/n): " choice
    if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
        echo "Installing fzf..."
        git clone --depth 1 https://github.com/junegunn/fzf.git "$ACTUAL_HOME/.fzf"
        sudo chown -R "$ACTUAL_USER":"$ACTUAL_USER" "$ACTUAL_HOME/.fzf"
        su -c "$ACTUAL_HOME/.fzf/install --all" - "$ACTUAL_USER"
    fi
fi

# Install zsh plugins
echo "Installing Zsh plugins..."
ZSH_CUSTOM="$ACTUAL_HOME/.oh-my-zsh/custom"
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
        sudo chown -R "$ACTUAL_USER":"$ACTUAL_USER" "$ZSH_CUSTOM/plugins/$plugin"
    else
        echo "$plugin is already installed, skipping..."
    fi
done

# Install Powerlevel10k theme
echo "Installing Powerlevel10k..."
if [ ! -d "$ZSH_CUSTOM/themes/powerlevel10k" ]; then
    git clone --depth=1 https://github.com/romkatv/powerlevel10k.git "$ZSH_CUSTOM/themes/powerlevel10k"
    sudo chown -R "$ACTUAL_USER":"$ACTUAL_USER" "$ZSH_CUSTOM/themes/powerlevel10k"
fi

# Setup Powerlevel10k
echo "Setting up Powerlevel10k..."
createSymlink "$DOTFILES_DIR/.p10k.zsh" "$ACTUAL_HOME/.p10k.zsh"

# Ask user whether to install ideavimrc
read -r -p "Do you want to setup .ideavimrc ? (y/n): " choice
if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
    # Install IDEAVim Configuration
    echo "Setting up IDEAVim..."
    createSymlink "$DOTFILES_DIR/.ideavimrc" "$ACTUAL_HOME/.ideavimrc"
fi

# Ask user whether to install fonts
read -r -p "Do you want to install fonts ? (y/n): " choice
if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
    # Install fonts
    echo "Installing Nerd Fonts..."

    FONT_DIR="$ACTUAL_HOME/.local/share/fonts"
    mkdir -p "$FONT_DIR"

    # Read fonts from the file and download them
    FONTS_FILE="$DOTFILES_DIR/fonts.txt"
    while IFS= read -r font || [[ -n "$font" ]]; do
        # Ignore empty lines
        if [[ -z "$font" ]]; then
            continue
        fi

        # Check if font already exists - properly handle special characters
        font_pattern="${font// /_}*" # Convert spaces to _ for matching
        if find "$FONT_DIR" -name "$font_pattern" -o -name "${font}*.ttf" -o -name "${font}*.otf" 2>/dev/null | grep -q .; then
            echo "$font Nerd Font is already installed. Skipping..."
            continue
        fi

        # Ask user whether to install this font
        read -r -p "Do you want to install $font Nerd Font? (y/n): " choice
        if [[ "$choice" != "y" && "$choice" != "Y" ]]; then
            echo "Skipping $font Nerd Font."
            continue
        fi

        echo "Downloading $font Nerd Font..."
        temp_dir=$(mktemp -d)
        if wget -q --show-progress -O "$temp_dir/$font.zip" "https://github.com/ryanoasis/nerd-fonts/releases/latest/download/$font.zip"; then
            unzip -o "$temp_dir/$font.zip" -d "$FONT_DIR" 2>/dev/null
            rm "$temp_dir/$font.zip"
            rm -rf "$temp_dir"
            sudo chown -R "$ACTUAL_USER":"$ACTUAL_USER" "$FONT_DIR"
        else
            echo "Failed to download $font Nerd Font."
        fi
    done <"$FONTS_FILE"
fi

echo "Fonts installed successfully!"

# Change default shell to Zsh
if [[ "$(getent passwd "$ACTUAL_USER" | cut -d: -f7)" != "$(which zsh)" ]]; then
    echo "Changing default shell to Zsh for $ACTUAL_USER..."
    if sudo chsh -s "$(which zsh)" "$ACTUAL_USER"; then
        echo "Shell changed successfully! Restart your terminal."
    else
        echo "Failed to change shell. Try running: sudo chsh -s $(which zsh) $ACTUAL_USER"
    fi
fi

# Make auto-commit script executable if it exists
SCRIPT_PATH="$DOTFILES_DIR/auto-commit.sh"

read -r -p "Do you want to setup the auto commit script? (y/n): " choice
if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
    if [[ -f "$SCRIPT_PATH" ]]; then
        sudo chmod +x "$SCRIPT_PATH"

        # Check if the script is already in crontab for the actual user
        if ! su -c "crontab -l 2>/dev/null" - "$ACTUAL_USER" | grep -qF "$SCRIPT_PATH"; then
            (
                su -c "crontab -l 2>/dev/null" - "$ACTUAL_USER"
                echo "0 22 * * * $SCRIPT_PATH"
            ) | su -c "crontab -" - "$ACTUAL_USER"
            echo "Auto-commit script scheduled in cron job successfully!"
        else
            echo "Auto-commit script is already scheduled. Skipping..."
        fi
    fi
fi

# Refresh font cache
sudo fc-cache -fv
echo "Dotfiles setup complete! Restart your terminal for changes to take effect."
