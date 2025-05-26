#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <hostname>"
    exit 1
fi

KEY=".ssh/anchor_admin_id"
USER="anchor"
HOST="$1"

if [ ! -S "$SSH_AUTH_SOCK" ]; then
    if [ -f "$HOME/.ssh/agent_env" ]; then
        source "$HOME/.ssh/agent_env" > /dev/null
    fi
    if [ ! -S "$SSH_AUTH_SOCK" ]; then
        eval "$(ssh-agent -s)" > "$HOME/.ssh/agent_env"
        source "$HOME/.ssh/agent_env" > /dev/null
    fi
fi

if ! ssh-add -l | grep "$KEY" > /dev/null 2>&1; then
    ssh-add "$KEY"
fi

ssh "$USER@$HOST"
