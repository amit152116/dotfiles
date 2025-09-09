# Dotfiles and Configuration Setup

This repository contains my personal dotfiles and system configurations for **Pop!_OS, Neovim, Tmux, Zsh, and more**. The goal is to have a seamless setup across different machines with minimal manual configuration.

## 📂 Repository Structure

.
├── alacritty/                   # Alacritty terminal configuration
├── auto_commit.sh               # Script to automate git commits
├── git_autocommit.sh            # Git auto-commit helper
├── git_common.sh                # Common git utility functions
├── btop/                        # Btop system monitor configs and themes
├── env/                         # Environment configuration
│   ├── .gitconfig               # Git configuration
│   ├── .gitignore               # Git ignore for environment
│   ├── .ideavimrc               # IdeaVim configuration
│   ├── .p10k.zsh                # Powerlevel10k prompt config
│   ├── .tmux.conf               # Tmux configuration
│   ├── .zsh/                    # Zsh shell scripts and plugins
│   └── .zshrc                   # Zsh main configuration
├── fonts.txt                    # List of installed/recommended fonts
├── git-aliases.md               # Documentation of custom git aliases
├── install                      # Installation scripts
├── installs/                    # Scripts to install various tools (Docker, Neovim, ROS2, tmux, etc.)
├── nvim/                        # Neovim configuration, plugins, and custom Lua scripts
├── scripts/                     # Utility scripts (network sharing, tmux management, system setup, etc.)
├── symlink.sh                   # Script to create symlinks for configuration files

## 🛠️ Installation & Setup

### 1️⃣ Clone the Repository

```bash
git clone --recursive https://github.com/amit152116/dotfiles.git ~/.dotfiles
cd ~/.dotfiles
```

### 2️⃣ Run the Install Script

```bash
chmod +x install
./install
```

This script will:

- Install required dependencies
- Symlink configuration files to the home directory
- Set up Neovim, Tmux, Zsh, and other essential tools

## ✨ Contributing

Suggestions, improvements, or issues? Feel free to open an issue or submit a pull request!

---
**Author**: Amit Kumar
📧 Contact: <amit152116@gmail.com>  
🔗 GitHub: [Amit152116](https://github.com/amit152116)
