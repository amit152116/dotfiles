#!/usr/bin/env bash

# --- Functions ---
createSymlink() {
	local src="$1"
	local dest="$2"

	if [ -e "$dest" ] || [ -L "$dest" ]; then
		if [ -L "$dest" ] && [ "$(readlink -- "$dest")" = "$src" ]; then
			echo "Symlink already exists: $dest → $src (No changes needed)"
			return 0
		fi

		echo "Backing up existing file: $dest"
		local filename backup_name
		filename=$(basename -- "$dest")

		if [[ "$filename" == .* ]]; then
			backup_name="${dest}.bak"
		else
			local extension="${filename##*.}"
			local base="${filename%.*}"
			if [[ "$base" == "$filename" ]]; then
				backup_name="${dest}.bak"
			else
				backup_name="${dest%.*}.bak.$extension"
			fi
		fi
		mv "$dest" "$backup_name"
	fi

	mkdir -p "$(dirname "$dest")"
	ln -s "$src" "$dest"
	echo "Symlink created: $dest → $src"
}
