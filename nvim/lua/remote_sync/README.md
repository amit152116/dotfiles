# Remote Sync Plugin

A Neovim plugin for syncing files between two development remotes using rsync over SSH, with multi-address fallback support and smart SSH key management.

## Features

- **Per-project Configuration** - TOML-based config in `.nvim/remote_sync.toml`
- **Bidirectional Sync** - Send or receive files with simple keybindings
- **Multi-address Support** - Automatic fallback across hostname/IPs
- **Smart SSH Key Management** - Auto-detects keys from `~/.ssh/config` or explicit override
- **Diff Preview** - See changes before receiving files
- **Directory Sync** - Sync entire directories with .gitignore filtering
- **SSH ControlMaster** - Connection reuse for faster operations
- **Health Checks** - `:checkhealth remote_sync` support
- **SSH Setup Wizard** - Guided setup for new users

## Installation

The plugin is included in this AstroNvim configuration. It's automatically loaded via `lua/plugins/remote_sync.lua`.

### Prerequisites

1. **SSH Keys** configured for the remote host (password auth is NOT supported):

   ```bash
   # Generate a key if you don't have one
   ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_remote

   # Copy public key to remote
   ssh-copy-id -i ~/.ssh/id_ed25519_remote user@remote-hostname
   ```

2. **rsync** installed on both machines

3. **ssh-agent** running with your key loaded (required if key has passphrase):
   ```bash
   eval $(ssh-agent -s)
   ssh-add ~/.ssh/id_ed25519_remote
   ```

## Quick Start

1. Run `:RemoteSyncSetupSSH` to check your SSH configuration
2. Open a file in your project
3. Press `<Leader>sc` or run `:RemoteSyncConfigure`
4. Enter remote address in one of these formats:
   - **SSH config host**: `hp-elitebook` (must be defined in `~/.ssh/config` with `User`)
   - **user@hostname**: `amit152116@hp-elitebook.local`
   - **user@ip**: `amit152116@192.168.0.100`
5. Optionally add fallback addresses (e.g., `amit152116@10.70.66.233`)
6. Enter remote base path (e.g., `~/projects/myapp`)

This creates `.nvim/remote_sync.toml` in your project root.

## Keybindings

All mappings are under `<Leader>s` (Remote Sync):

| Key          | Command                | Description                        |
| ------------ | ---------------------- | ---------------------------------- |
| `<Leader>ss` | `:RemoteSyncSend`      | Send current file to remote        |
| `<Leader>sr` | `:RemoteSyncReceive`   | Receive current file from remote   |
| `<Leader>sd` | `:RemoteSyncDiff`      | Show diff between local and remote |
| `<Leader>sD` | `:RemoteSyncDir`       | Sync entire directory              |
| `<Leader>sc` | `:RemoteSyncConfigure` | Configure remote settings          |
| `<Leader>se` | `:RemoteSyncEdit`      | Edit configuration file            |
| `<Leader>si` | `:RemoteSyncStatus`    | Show status & test connection      |

## SSH Key Management

The plugin uses a "Smart Default, Manual Override" approach for SSH keys:

### Key Selection Priority

1. **Explicit `ssh_key`** in project TOML config (highest priority)
2. **`~/.ssh/config`** IdentityFile for matching host
3. **ssh-agent** loaded keys
4. **Default keys** (`~/.ssh/id_ed25519`, `~/.ssh/id_rsa`, etc.)

### Recommended Setup

Add your remote host to `~/.ssh/config`:

```ssh
Host hp-elitebook
    HostName hp-elitebook.local
    User yourusername
    IdentityFile ~/.ssh/id_ed25519_remote
    IdentitiesOnly yes
```

This way, the plugin automatically uses the correct key without any extra configuration.

### Explicit Key Override

If you need a specific key for a project, add to `.nvim/remote_sync.toml`:

```toml
remote_addresses = ["hp-elitebook"]
remote_base_path = "~/projects/myapp"

# Explicit SSH key for this project
ssh_key = "~/.ssh/id_ed25519_work"
```

### SSH Setup Wizard

Run `:RemoteSyncSetupSSH` to:

- Check if ssh-agent is running
- List available SSH keys
- Show which keys are loaded in agent
- Verify project configuration
- Get recommendations for fixing issues

### Test Connection

Run `:RemoteSyncTestConnection` to test SSH connectivity with detailed feedback.

## Configuration File

`.nvim/remote_sync.toml` in project root:

```toml
# Remote addresses to try (in order)
# Use either:
#   - SSH config hostname (must have User configured): "hp-elitebook"
#   - Full user@host format: "amit152116@192.168.0.100"
remote_addresses = ["hp-elitebook", "amit152116@10.70.66.233"]

# Remote path (local path is auto-detected from project root)
remote_base_path = "~/projects/myapp"

# Optional: explicit SSH key (overrides ~/.ssh/config)
# ssh_key = "~/.ssh/id_ed25519_remote"

# Optional: gitignore-style files for exclusions
# ignorefile_paths = ["~/.gitignore", ".gitignore"]

# Optional: files to sync even if matched by ignore patterns
# remote_includes = ["build.log", "build/generated.json"]

# Optional: auto-sync on save for this project
# sync_on_save = true

# Auto-updated by plugin
last_working_address = "hp-elitebook"
```

### Per-Project Options

- **remote_addresses**: List of addresses to try in order. Each must be either:
  - A hostname from `~/.ssh/config` with `User` directive (e.g., `hp-elitebook`)
  - A `user@host` format (e.g., `amit152116@192.168.0.100`)
- **remote_base_path**: Base directory on remote remote
- **ssh_key**: _(optional)_ Explicit SSH private key path (overrides ~/.ssh/config)
- **ignorefile_paths**: _(optional)_ Override gitignore files to use for exclusions
- **remote_includes**: _(optional)_ Files to sync even if matched by ignore patterns
- **sync_on_save**: _(optional)_ Override global sync-on-save setting

### Global Options (set in `setup()`)

- **ignorefile_paths**: Default gitignore files (default: `[".gitignore"]`)
- **remote_includes**: Default files to always include (default: `[]`)
- **auto_create_dirs**: Create remote directories if missing (default: `true`)
- **show_diff_on_receive**: Show diff before receiving (default: `true`)
- **sync_on_save**: Auto-sync on save (default: `false`)
- **connect_timeout**: SSH connection timeout in seconds (default: `5`)

## Multi-Address Support

### How It Works

1. **Tries last working address first** (cached from previous sync)
2. **Falls back to other addresses** if the first fails
3. **Remembers which worked** for next time
4. **Shows clear errors** if all addresses fail

### Example Setup

Your remote is accessible via:

- Hostname: `hp-elitebook` (same network)
- Institute IP: `10.70.66.233` (different subnet)
- Home IP: `192.168.1.100`

Configure with:

```
Primary address: hp-elitebook
Additional addresses: 10.70.66.233, 192.168.1.100
```

The plugin automatically picks the working address.

## Health Checks

Run `:checkhealth remote_sync` to verify:

- rsync is installed
- ssh is available
- ssh-agent status and loaded keys
- Available SSH keys in ~/.ssh/
- Project configuration and SSH key setup

## Troubleshooting

### "Failed to connect to remote"

The plugin provides detailed error messages. Common issues:

1. **SSH key not in agent**:

   ```bash
   eval $(ssh-agent -s)
   ssh-add ~/.ssh/id_ed25519
   ```

2. **Public key not on remote**:

   ```bash
   ssh-copy-id -i ~/.ssh/id_ed25519 user@hostname
   ```

3. **Wrong key permissions**:
   ```bash
   chmod 600 ~/.ssh/id_ed25519
   ```

Run `:RemoteSyncSetupSSH` for guided diagnostics.

### "File is outside project root"

The file isn't under `local_base_path`. Either:

1. Reconfigure with `<Leader>sc`
2. Move the file into the project

### "Remote file does not exist"

Use `<Leader>ss` (send) first to create the file on remote.

### All addresses fail

Check connectivity:

```bash
ping -c 1 hp-elitebook
ssh hp-elitebook 'echo connected'
```

### Reset last working address

Edit `.nvim/remote_sync.toml` and remove the `last_working_address` line.

## SSH Config Tips

Add to `~/.ssh/config` for optimal performance:

```ssh
Host hp-elitebook 10.70.66.233 192.168.1.100
    User yourusername
    IdentityFile ~/.ssh/id_ed25519_remote
    IdentitiesOnly yes
    # Connection multiplexing (optional - plugin handles this)
    ControlMaster auto
    ControlPath ~/.ssh/sockets/%r@%h-%p
    ControlPersist 600
```

## Plugin Architecture

```
lua/remote_sync/
├── init.lua       # Public API: setup(), send(), receive(), diff(), status()
├── config.lua     # Global configuration and defaults
├── project.lua    # Per-project TOML config handling
├── sync.lua       # Core rsync operations
├── ssh.lua        # SSH connection, key detection, and fallback
├── commands.lua   # User commands and autocmds
├── health.lua     # :checkhealth support
└── utils.lua      # Shell escaping, notifications
```

## API

```lua
local remote_sync = require("remote_sync")

-- Setup with custom global options
remote_sync.setup({
  sync_on_save = false,              -- Auto-sync on save (default: false)
  show_diff_on_receive = true,       -- Show diff before receiving (default: true)
  auto_create_dirs = true,           -- Create remote dirs if missing (default: true)
  connect_timeout = 5,               -- SSH timeout in seconds (default: 5)

  -- Gitignore-style files for exclusions
  ignorefile_paths = { ".gitignore" },

  -- Files to sync even if matched by ignore patterns
  remote_includes = { "build.log" },
})

-- Programmatic usage
remote_sync.send()           -- Send current file
remote_sync.receive()        -- Receive current file
remote_sync.diff()           -- Show diff
remote_sync.status()         -- Show configuration
```

## Commands

| Command                                       | Description                        |
| --------------------------------------------- | ---------------------------------- |
| `:RemoteSyncConfigure`                        | Configure project for syncing      |
| `:RemoteSyncSend`                             | Send current file to remote        |
| `:RemoteSyncReceive`                          | Receive current file from remote   |
| `:RemoteSyncDiff`                             | Show diff between local and remote |
| `:RemoteSyncDirUp`                            | Sync entire directory to remote    |
| `:RemoteSyncDirDown`                          | Sync entire directory from remote  |
| `:RemoteSyncCancel`                           | Cancel current sync operation      |
| `:RemoteSyncStatus`                           | Show sync status                   |
| `:RemoteSyncConfig [show\|reload\|global]`    | Show/reload configuration          |
| `:RemoteSyncOnSave [enable\|disable\|toggle]` | Toggle sync on save                |
| `:RemoteSyncSetupSSH`                         | SSH setup wizard                   |
| `:RemoteSyncTestConnection [host]`            | Test SSH connection                |

## License

Part of personal dotfiles configuration.
