#!/bin/bash
set -e

SHARED_FOLDER="/home/user/PyChronoSharedFolder"
PYCHRONO_DIR="${SHARED_FOLDER}/UAV_Sim_PyChrono_v9.0.1"

# ==============================================================================
# Dynamically fix container home directory ownership to match host UID/GID
# ==============================================================================
if [ "$(id -u)" != "0" ]; then
  # Check if the current user doesn't own their home directory
  if [ ! -w "/home/user" ]; then
    echo "🔧 Adjusting container home directory permissions for UID $(id -u)..."
    sudo chown -R $(id -u):$(id -g) /home/user 2>/dev/null || true
  fi
fi

#==========================================
# Check if simulator is already installed
#==========================================
cat >> /home/user/.bashrc << 'EOF'

PYCHRONO_DIR="/home/user/PyChronoSharedFolder/UAV_Sim_PyChrono_v9.0.1"

if [ -d "$PYCHRONO_DIR" ]; then
  echo -e "\n✓ PyChrono simulator already detected."
  echo -e "To start the simulator run:\n" 
  echo "cd UAV_Sim_PyChrono_v9.0.1"
  echo "python main.py"
else
  echo -e "\n⚠ PyChrono simulator not found."
  read -p "Do you want to download it now? (y/n): " answer
  case "$answer" in
    [Yy]* )
      echo -e "\nCloning repository..."
      git clone git@github.com:lucamarchiano/UAV_Sim_PyChrono_v9.0.1 "$PYCHRONO_DIR"
      ;;
    * )
      echo -e "\nDownload skipped."
      ;;
  esac
fi
EOF


#==========================================
# Add GitHub repo info in the prompt
#==========================================
if ! grep -q "parse_git_repo" /home/user/.bashrc 2>/dev/null; then
cat >> /home/user/.bashrc << 'EOF'
parse_git_repo() {
  local repo_url=$(git remote get-url origin 2>/dev/null)
  if [[ -z "$repo_url" ]]; then echo ''; return; fi
  # extract only repo name from ...user/repo(.git)
  local repo_name=$(echo "$repo_url" | sed -E 's|.*[:/]([^/:]+)/([^/]+)(\.git)?$|\2|' | sed 's/\.git$//')
  local branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
  echo "${repo_name} : ${branch}"
}

export PS1='\[\033[1;33m\]\u@\h\[\033[0m\]:\[\033[1;34m\]\w\[\033[0m\]$( [[ -n "$(git rev-parse --git-dir 2>/dev/null)" ]] && echo " \[\033[1;35m\][$(parse_git_repo)\[\033[1;35m\]]\[\033[0m\]" || echo "")\[\033[0m\]\$ \[\033[0m\]'

EOF
fi

exec "$@"