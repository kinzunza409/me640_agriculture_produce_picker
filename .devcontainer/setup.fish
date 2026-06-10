#!/usr/bin/env fish

# ── Greeting ──
set -U fish_greeting (set_color blue)"
--------------------------------------------------------------------
█████▄  ▄████▄ ▄█████ ████▄   ██  ██ ▄▄ ▄▄ ▄▄   ▄▄ ▄▄▄▄  ▄▄    ▄▄▄▄▄ 
██▄▄██▄ ██  ██ ▀▀▀▄▄▄  ▄██▀   ██████ ██ ██ ██▀▄▀██ ██▄██ ██    ██▄▄  
██   ██ ▀████▀ █████▀ ███▄▄   ██  ██ ▀███▀ ██   ██ ██▄█▀ ██▄▄▄ ██▄▄▄ 
--------------------------------------------------------------------
"(set_color normal)

# ── Fisher ──
curl -sL https://raw.githubusercontent.com/jorgebucaran/fisher/main/functions/fisher.fish | source

# ── Plugins ──
fisher install \
    jorgebucaran/fisher \
    edc/bass \
    kpbaks/ros2.fish; or true
