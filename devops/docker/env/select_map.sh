#!/bin/bash

# Function to display help
function show_help() {
    echo "Usage: select_map [map name]"
    echo "Available maps:"
    for dir in ~/ws/src/race_stack/src/core/maps/*/; do
        if [ -d "$dir" ]; then
            echo "  $(basename "$dir")"
        fi
    done
    exit 1
}

# Function to select a map
function select_map() {
    local mapName=$1
    local mapPath=~/ws/src/race_stack/src/core/maps/$mapName

    # Check if the map exists
    if [ ! -d "$mapPath" ]; then
        echo "ERROR: Map '$mapName' not found."
        echo
        show_help
    fi

    # Update LARACE_CURRENT_MAP in ~/.bashrc
    if grep -q "export LARACE_CURRENT_MAP=" ~/.bashrc; then
        sed -i "s/export LARACE_CURRENT_MAP=.*/export LARACE_CURRENT_MAP=$mapName/" ~/.bashrc
    else
        echo "export LARACE_CURRENT_MAP=$mapName" >> ~/.bashrc
    fi

    # Source the updated ~/.bashrc
    source ~/.bashrc > /dev/null 2>&1
}

# Check if no arguments are provided or if help is requested
if [ $# -eq 0 ] || [[ "$1" =~ ^(-h|--help|help|-l|list)$ ]]; then
    show_help
fi

# Call select_map function with provided argument
select_map "$1"