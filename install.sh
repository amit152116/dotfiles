#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

# Define dotfiles directory
DOTFILES_DIR="$HOME/dotfiles"

# Install GitHub CLI (gh)
echo "Installing GitHub CLI..."
sudo mkdir -p -m 755 /etc/apt/keyrings
GH_KEYRING="/etc/apt/keyrings/githubcli-archive-keyring.gpg"
GH_REPO="https://cli.github.com/packages"

wget -qO- "$GH_REPO/githubcli-archive-keyring.gpg" | sudo tee "$GH_KEYRING" > /dev/null
sudo chmod go+r "$GH_KEYRING"

echo "deb [arch=$(dpkg --print-architecture) signed-by=$GH_KEYRING] $GH_REPO stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null

sudo apt update && sudo apt upgrade -y

# Define list of required packages
PACKAGES=(
   snapd zsh tmux xclip fzf bat ripgrep fd-find curl git wget unzip python3-pip btop gh
)

echo "Installing required packages..."
sudo apt install -y "${PACKAGES[@]}"

# Install Neovim via Snap
echo "Installing Neovim..."
sudo snap install nvim --classic

# Setup Git Configuration
echo "Setting up Git config..."
ln -sf "$DOTFILES_DIR/.gitconfig" "$HOME/.gitconfig"

# Setup btop config
echo "Setting up btop config..."
if [ ! -d "$HOME/.config/btop" ]; then
    mkdir -p "$HOME/.config/btop"
fi
ln -sfn "$DOTFILES_DIR/btop" "$HOME/.config/btop"

# Backup existing Neovim config if it exists
if [ -d "$HOME/.config/nvim" ]; then
    echo "Backing up existing Neovim config..."
    mv "$HOME/.config/nvim" "$HOME/.config/nvim_$(date +%Y%m%d_%H%M%S).bak"
fi

# Ensure `fzf` is installed (interactive fuzzy finder)
git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
~/.fzf/install --all

# Setup Neovim
echo "Setting up Neovim..."
mkdir -p "$HOME/.config/nvim"
ln -sfn "$DOTFILES_DIR/nvim" "$HOME/.config/nvim"

# Setup Tmux
echo "Setting up Tmux..."
ln -sf "$DOTFILES_DIR/.tmux.conf" "$HOME/.tmux.conf"

# Install TPM (Tmux Plugin Manager)
if [ ! -d "$HOME/.tmux/plugins/tpm" ]; then
    echo "Installing TPM..."
    git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
fi

# Setup Zsh
echo "Setting up Zsh..."
ln -sf "$DOTFILES_DIR/.zshrc" "$HOME/.zshrc"

# Install Oh My Zsh if not installed
if [ ! -d "$HOME/.oh-my-zsh" ]; then
    echo "Installing Oh My Zsh..."
    sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended
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
ln -sf "$DOTFILES_DIR/.ideavimrc" "$HOME/.ideavimrc"

# Change default shell to Zsh
if [[ "$SHELL" != "$(which zsh)" ]]; then
    echo "Changing default shell to Zsh..."
    chsh -s "$(which zsh)"
fi

echo "Dotfiles setup complete! Restart your terminal for changes to take effect."

# Make the script executable
SCRIPT_PATH="$DOTFILES_DIR/auto-commit.sh"
chmod +x "$SCRIPT_PATH"

# Add the script to the crontab (runs every day at midnight)
(crontab -l 2>/dev/null; echo "0 22 * * * $SCRIPT_PATH") | crontab -

echo "Auto-commit script created and scheduled in cron job successfully!"


