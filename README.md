# Dotfiles and Configuration Setup

This repository contains my personal dotfiles and system configurations for **Pop!_OS, Neovim, Tmux, Zsh, and more**. The goal is to have a seamless setup across different machines with minimal manual configuration.

## 📂 Repository Structure

``` plaintext
.
├── .gitconfig                  # Git configuration
├── .ideavimrc                  # IdeaVim configuration for JetBrains IDEs
├── .tmux.conf                  # Tmux configuration with plugins
├── .zshrc                      # Zsh shell configuration
├── auto_commit.sh              # Script for automatic git commits
├── install.sh                  # Installation script for dependencies
├── btop/                       # Btop configuration
│   ├── btop.conf                  # Btop configuration file
├── nvim/                       # Neovim configuration (LazyVim-based)
│   ├── init.lua                # Main Neovim configuration
│   ├── lua/                    # Custom Lua configurations and plugins
│   ├── README.md               # Documentation for Neovim setup
│   ├── Session.vim             # Neovim session management
│   ├── .gitignore              # Ignore compiled files and backups
│   ├── .stylua.toml            # Lua formatting rules
│   ├── selene.toml             # Lua linting rules
│   ├── neovim.yml              # Additional Neovim configuration
├── share_internet_ethernet.sh  # Script to share internet over Ethernet
├── stop_share_internet_ethernet.sh # Script to stop sharing internet
├── windows_powershell/         # Windows PowerShell configuration
│   ├── oh-my-posh/myTheme.omp.json  # Custom Oh My Posh theme
│   ├── profile.ps1             # PowerShell profile script
└── README.md                   # This documentation
```

## 🚀 Features

- **Neovim Config**: Uses LazyVim with custom plugins and settings.
- **Tmux Config**: Enhanced Tmux setup with Dracula theme and useful keybindings.
- **Zsh & Oh-My-Zsh**: Configured with aliases, plugins, and a modern terminal experience.
- **Automatic Git Commit**: A cron job automatically commits and pushes changes every day.
- **System Setup Scripts**: Scripts for internet sharing, installation, and automation.

## 🛠️ Installation & Setup

### 1️⃣ Clone the Repository

```bash
git clone --recursive https://github.com/amit152116kumar/dotfiles.git ~/.dotfiles
cd ~/.dotfiles
```

### 2️⃣ Run the Install Script

```bash
chmod +x install.sh
./install.sh
```

This script will:

- Install required dependencies
- Symlink configuration files to the home directory
- Set up Neovim, Tmux, Zsh, and other essential tools

## ✨ Contributing

Suggestions, improvements, or issues? Feel free to open an issue or submit a pull request!

---
**Author**: Amit Kumar
📧 Contact: <amit170103004@alumini.iitg.ac.in>  
🔗 GitHub: [amit152116kumar](https://github.com/amit152116kumar)
