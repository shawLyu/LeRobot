# Source this file from bash or zsh after running bootstrap.sh.
if [ -n "${ZSH_VERSION:-}" ]; then
    project_activate_source="${(%):-%x}"
elif [ -n "${BASH_VERSION:-}" ]; then
    project_activate_source="${BASH_SOURCE[0]}"
else
    echo "Use bash or zsh to source scripts/activate.sh" >&2
    return 1
fi
project_activate_root="$(cd -- "$(dirname -- "$project_activate_source")/.." && pwd)"
export UV_CACHE_DIR="$project_activate_root/.cache/uv"
export UV_PYTHON_INSTALL_DIR="$project_activate_root/.tools/python"
export UV_PROJECT_ENVIRONMENT="$project_activate_root/.venv"
export HF_LEROBOT_HOME="$project_activate_root/data/lerobot"
export PATH="$project_activate_root/.tools/bin:$PATH"
source "$project_activate_root/.venv/bin/activate"
unset project_activate_source project_activate_root
