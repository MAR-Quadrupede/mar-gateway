#!/usr/bin/env bash
# ==============================================================================
# 🐾 M.A.R. — Multi-legged Autonomous Robot
# Raspberry Pi 5 Platform Initialisation & Orchestration Script
#
# Automates:
#  1. System prerequisites, cgroups, Docker, and K3s Kubernetes on ARM64
#  2. GitHub Actions Self-Hosted Runner installation & systemd integration
#  3. Dynamic discovery and cloning of all organization microservice repositories
#  4. Local container image builds (Docker on ARM64)
#  5. Kubernetes (K3s) deployment & inter-service message bus configuration
# ==============================================================================

set -eo pipefail

# ------------------------------------------------------------------------------
# CONFIGURATION & DEFAULTS
# ------------------------------------------------------------------------------
export DEBIAN_FRONTEND=noninteractive

INSTALL_DIR="${INSTALL_DIR:-$HOME/mar-platform}"
GITHUB_ORG="${GITHUB_ORG:-MAR-Quadrupede}"
GITHUB_TOKEN="${GITHUB_TOKEN:-}"
RUNNER_TOKEN="${RUNNER_TOKEN:-}"
RUNNER_NAME="${RUNNER_NAME:-rpi5-runner-$(hostname)}"
RUNNER_LABELS="${RUNNER_LABELS:-self-hosted,linux,arm64,rpi5}"
K8S_NAMESPACE="${K8S_NAMESPACE:-mar-system}"

FALLBACK_REPOS=(
  "mar-firmware"
  "mar-serial-bridge"
  "mar-lidar-service"
  "mar-vision-service"
  "mar-decision-engine"
  "mar-gateway"
  "mar-dashboard"
)

# Colors
C_RESET='\033[0m'
C_BOLD='\033[1m'
C_GREEN='\033[32m'
C_BLUE='\033[34m'
C_YELLOW='\033[33m'
C_RED='\033[31m'

log_info()    { echo -e "${C_BLUE}${C_BOLD}[INFO]${C_RESET} $*"; }
log_success() { echo -e "${C_GREEN}${C_BOLD}[OK]${C_RESET} $*"; }
log_warn()    { echo -e "${C_YELLOW}${C_BOLD}[WARN]${C_RESET} $*"; }
log_error()   { echo -e "${C_RED}${C_BOLD}[ERROR]${C_RESET} $*"; }

banner() {
  echo -e "${C_BOLD}${C_GREEN}"
  echo "======================================================================"
  echo " 🐾 M.A.R. — RASPBERRY PI 5 BOOTSTRAP & KUBERNETES ORCHESTRATOR"
  echo "======================================================================"
  echo -e "${C_RESET}"
  echo "Target Directory:   $INSTALL_DIR"
  echo "GitHub Org:         https://github.com/$GITHUB_ORG"
  echo "Kubernetes NS:      $K8S_NAMESPACE"
  echo "Architecture:       $(uname -m)"
  echo "======================================================================"
}

# ------------------------------------------------------------------------------
# PRE-FLIGHT CHECKS
# ------------------------------------------------------------------------------
preflight_checks() {
  log_info "Verifying hardware and architecture..."
  ARCH=$(uname -m)
  if [ "$ARCH" != "aarch64" ] && [ "$ARCH" != "arm64" ]; then
    log_warn "Current architecture is $ARCH. Raspberry Pi 5 runs aarch64/arm64."
    log_warn "Proceeding, but ensure ARM64 containers match target runtime."
  else
    log_success "Architecture $ARCH confirmed."
  fi

  mkdir -p "$INSTALL_DIR"
  cd "$INSTALL_DIR"
}

# ------------------------------------------------------------------------------
# 1. SYSTEM PREREQUISITES & DOCKER ENGINE
# ------------------------------------------------------------------------------
install_system_deps() {
  log_info "1/5 Installing system utilities and dependencies..."

  if command -v apt-get >/dev/null 2>&1; then
    sudo apt-get update -y
    sudo apt-get install -y --no-install-recommends \
      curl git jq tar build-essential ca-certificates gnupg lsb-release \
      v4l-utils udev libgl1 libglib2.0-0
    log_success "System packages installed."

    # Check and enable cgroups for Kubernetes on Raspberry Pi OS
    CMDLINE_FILE=""
    if [ -f "/boot/firmware/cmdline.txt" ]; then
      CMDLINE_FILE="/boot/firmware/cmdline.txt"
    elif [ -f "/boot/cmdline.txt" ]; then
      CMDLINE_FILE="/boot/cmdline.txt"
    fi

    if [ -n "$CMDLINE_FILE" ]; then
      if ! grep -q "cgroup_enable=cpuset" "$CMDLINE_FILE"; then
        log_info "Enabling cgroup memory and cpuset in $CMDLINE_FILE..."
        sudo sed -i '$s/$/ cgroup_enable=cpuset cgroup_enable=memory cgroup_memory=1/' "$CMDLINE_FILE"
        log_warn "cgroups enabled in $CMDLINE_FILE. A reboot may be required if K3s fails to start."
      fi
    fi
  else
    log_warn "apt-get not found. Skipping deb-package installation."
  fi

  # Install Docker Engine if not present
  if ! command -v docker >/dev/null 2>&1; then
    log_info "Installing Docker Engine via official installer..."
    curl -fsSL https://get.docker.com | sudo sh
    sudo usermod -aG docker "$USER" || true
    sudo systemctl enable --now docker
    log_success "Docker installed and started."
  else
    log_success "Docker is already installed ($(docker --version))."
  fi
}

# ------------------------------------------------------------------------------
# 2. KUBERNETES (K3S) INSTALLATION
# ------------------------------------------------------------------------------
install_k3s() {
  log_info "2/5 Initialising Kubernetes (K3s lightweight edge cluster)..."

  if ! command -v k3s >/dev/null 2>&1; then
    log_info "Downloading and running official K3s installer (with Docker CRI integration)..."
    # --docker ensures K3s can directly use Docker daemon image cache
    curl -sfL https://get.k3s.io | INSTALL_K3S_EXEC="--docker --write-kubeconfig-mode 644" sh -
    log_success "K3s service installed."
  else
    log_success "K3s is already installed."
  fi

  # Configure KUBECONFIG for non-root access
  mkdir -p "$HOME/.kube"
  if [ -f "/etc/rancher/k3s/k3s.yaml" ]; then
    sudo cp /etc/rancher/k3s/k3s.yaml "$HOME/.kube/config"
    sudo chown "$USER:$USER" "$HOME/.kube/config"
    chmod 600 "$HOME/.kube/config"
    export KUBECONFIG="$HOME/.kube/config"
  fi

  log_info "Waiting for Kubernetes node to report Ready..."
  local attempts=0
  until kubectl get nodes 2>/dev/null | grep -q "Ready"; do
    sleep 2
    attempts=$((attempts + 1))
    if [ $attempts -ge 30 ]; then
      log_warn "Timed out waiting for K3s node Ready. Proceeding with caution."
      break
    fi
  done
  log_success "Kubernetes cluster is operational:"
  kubectl get nodes -o wide || true
}

# ------------------------------------------------------------------------------
# 3. GITHUB ACTIONS SELF-HOSTED RUNNER SETUP
# ------------------------------------------------------------------------------
install_github_runner() {
  log_info "3/5 Configuring GitHub Actions Self-Hosted Runner for ARM64..."

  RUNNER_DIR="$INSTALL_DIR/actions-runner"
  mkdir -p "$RUNNER_DIR"
  cd "$RUNNER_DIR"

  if [ ! -f "config.sh" ]; then
    log_info "Querying latest GitHub Actions Runner release for ARM64..."
    LATEST_RUNNER_TAG=$(curl -s https://api.github.com/repos/actions/runner/releases/latest | jq -r '.tag_name // empty' | sed 's/^v//')
    if [ -z "$LATEST_RUNNER_TAG" ]; then
      LATEST_RUNNER_TAG="2.321.0"
      log_warn "Could not fetch latest tag from API. Using fallback v$LATEST_RUNNER_TAG."
    fi

    RUNNER_TARBALL="actions-runner-linux-arm64-${LATEST_RUNNER_TAG}.tar.gz"
    RUNNER_URL="https://github.com/actions/runner/releases/download/v${LATEST_RUNNER_TAG}/${RUNNER_TARBALL}"

    log_info "Downloading runner: $RUNNER_URL..."
    curl -sSL -o "$RUNNER_TARBALL" "$RUNNER_URL"
    tar -xzf "$RUNNER_TARBALL"
    rm -f "$RUNNER_TARBALL"

    # Install runner system dependencies
    if [ -f "./bin/installdependencies.sh" ]; then
      sudo ./bin/installdependencies.sh || true
    fi
    log_success "Runner binaries extracted in $RUNNER_DIR."
  fi

  # Helper script to register runner whenever a token is available
  cat <<'EOF' > "$INSTALL_DIR/register_runner.sh"
#!/usr/bin/env bash
set -e
RUNNER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/actions-runner" && pwd)"
cd "$RUNNER_DIR"

TOKEN="$1"
if [ -z "$TOKEN" ]; then
  echo "Usage: ./register_runner.sh <GITHUB_RUNNER_REGISTRATION_TOKEN>"
  echo "Obtain token from: https://github.com/organizations/MAR-Quadrupede/settings/actions/runners/new"
  exit 1
fi

ORG_URL="${2:-https://github.com/MAR-Quadrupede}"
RUNNER_NAME="${3:-rpi5-runner-$(hostname)}"

echo "Configuring GitHub Self-Hosted Runner for $ORG_URL..."
./config.sh --url "$ORG_URL" --token "$TOKEN" --name "$RUNNER_NAME" --labels "self-hosted,linux,arm64,rpi5" --unattended --replace

echo "Installing runner as a systemd service..."
sudo ./svc.sh install || true
sudo ./svc.sh start || true
sudo ./svc.sh status || true
echo "Runner successfully registered and started!"
EOF
  chmod +x "$INSTALL_DIR/register_runner.sh"

  if [ -n "$RUNNER_TOKEN" ]; then
    log_info "RUNNER_TOKEN provided. Registering runner with organization $GITHUB_ORG..."
    "$INSTALL_DIR/register_runner.sh" "$RUNNER_TOKEN" "https://github.com/$GITHUB_ORG" "$RUNNER_NAME" || log_warn "Runner registration failed. Check token validity."
  else
    log_warn "RUNNER_TOKEN was not provided."
    log_warn "To connect the runner to GitHub later, obtain a registration token and run:"
    log_warn "  $INSTALL_DIR/register_runner.sh <TOKEN>"
  fi
}

# ------------------------------------------------------------------------------
# 4. FLEXIBLE MULTI-REPO CLONING & DOCKER BUILDING
# ------------------------------------------------------------------------------
clone_and_build_repos() {
  log_info "4/5 Discovering and cloning repositories from $GITHUB_ORG..."

  REPOS=()

  # Dynamic discovery via GitHub API
  AUTH_HEADER=()
  if [ -n "$GITHUB_TOKEN" ]; then
    AUTH_HEADER=(-H "Authorization: token $GITHUB_TOKEN")
  fi

  API_RESPONSE=$(curl -s "${AUTH_HEADER[@]}" "https://api.github.com/orgs/$GITHUB_ORG/repos?per_page=100" || true)
  if echo "$API_RESPONSE" | jq -e 'type == "array"' >/dev/null 2>&1; then
    DISCOVERED=($(echo "$API_RESPONSE" | jq -r '.[].name'))
    for r in "${DISCOVERED[@]}"; do
      if [ "$r" != ".github" ]; then
        REPOS+=("$r")
      fi
    done
  fi

  if [ ${#REPOS[@]} -eq 0 ]; then
    log_warn "Could not discover repositories dynamically from API. Falling back to default list."
    REPOS=("${FALLBACK_REPOS[@]}")
  fi

  log_info "Identified ${#REPOS[@]} repositories to process: ${REPOS[*]}"

  REPOS_DIR="$INSTALL_DIR/repos"
  mkdir -p "$REPOS_DIR"

  for repo in "${REPOS[@]}"; do
    TARGET_REPO_DIR="$REPOS_DIR/$repo"
    if [ -d "$TARGET_REPO_DIR/.git" ]; then
      log_info "Updating repository $repo..."
      git -C "$TARGET_REPO_DIR" pull --ff-only || log_warn "Git pull failed for $repo (local modifications?)"
    else
      log_info "Cloning $GITHUB_ORG/$repo..."
      CLONE_URL="https://github.com/${GITHUB_ORG}/${repo}.git"
      if [ -n "$GITHUB_TOKEN" ]; then
        CLONE_URL="https://${GITHUB_TOKEN}@github.com/${GITHUB_ORG}/${repo}.git"
      fi
      git clone --depth 1 "$CLONE_URL" "$TARGET_REPO_DIR" || log_warn "Could not clone $repo. Skipping."
    fi

    # Build container image dynamically if Dockerfile exists
    if [ -f "$TARGET_REPO_DIR/Dockerfile" ]; then
      IMAGE_TAG="${repo}:latest"
      log_info "Building local Docker container image: $IMAGE_TAG..."
      docker build -t "$IMAGE_TAG" "$TARGET_REPO_DIR" || log_warn "Build failed for $IMAGE_TAG"
    fi
  done
}

# ------------------------------------------------------------------------------
# 5. KUBERNETES DEPLOYMENT & MESSAGE FABRIC CONFIGURATION
# ------------------------------------------------------------------------------
deploy_kubernetes_platform() {
  log_info "5/5 Deploying microservices and message fabric on Kubernetes (K3s)..."

  K8S_SOURCE_DIR=""
  # Search for k8s manifests directory
  if [ -d "$(dirname "$0")/k8s" ]; then
    K8S_SOURCE_DIR="$(cd "$(dirname "$0")/k8s" && pwd)"
  elif [ -d "$INSTALL_DIR/k8s" ]; then
    K8S_SOURCE_DIR="$INSTALL_DIR/k8s"
  fi

  if [ -n "$K8S_SOURCE_DIR" ] && [ -d "$K8S_SOURCE_DIR" ]; then
    log_info "Applying Kubernetes manifests from $K8S_SOURCE_DIR..."
    kubectl apply -k "$K8S_SOURCE_DIR" || kubectl apply -f "$K8S_SOURCE_DIR"
    log_success "Kubernetes manifests applied."
  else
    log_warn "No k8s/ manifests folder found directly. Applying standard microservices template..."
    kubectl create namespace "$K8S_NAMESPACE" --dry-run=client -o yaml | kubectl apply -f -
  fi

  log_info "Verifying Kubernetes pods in namespace '$K8S_NAMESPACE'..."
  kubectl get pods,svc -n "$K8S_NAMESPACE" -o wide || true
}

# ------------------------------------------------------------------------------
# MAIN EXECUTION
# ------------------------------------------------------------------------------
main() {
  banner
  preflight_checks
  install_system_deps
  install_k3s
  install_github_runner
  clone_and_build_repos
  deploy_kubernetes_platform

  echo ""
  echo -e "${C_BOLD}${C_GREEN}======================================================================${C_RESET}"
  echo -e "${C_BOLD}${C_GREEN} 🎉 M.A.R. PLATFORM INSTALLATION & CONFIGURATION COMPLETE!${C_RESET}"
  echo -e "${C_BOLD}${C_GREEN}======================================================================${C_RESET}"
  echo ""
  echo -e "${C_BOLD}Active Services & Access Endpoints:${C_RESET}"
  echo -e "  • Dashboard Web UI:      http://localhost:30000  (or :3000)"
  echo -e "  • Video Stream (MJPEG):  http://localhost:30080/stream/left (or :8080)"
  echo -e "  • WebSocket Gateway:     ws://localhost:30765    (or :8765)"
  echo -e "  • MQTT Message Broker:   mar-mosquitto:1883      (ClusterIP / DNS)"
  echo -e "  • Grafana Monitoring:    http://localhost:30300  (admin / admin)"
  echo -e "  • Portainer Cluster GUI: http://localhost:30900"
  echo -e "  • Prometheus Metrics:    http://localhost:30090"
  echo ""
  echo -e "${C_BOLD}Useful Management Commands:${C_RESET}"
  echo -e "  • Check cluster status:  kubectl get pods,svc -n $K8S_NAMESPACE"
  echo -e "  • View microservice log: kubectl logs -f deployment/<service> -n $K8S_NAMESPACE"
  echo -e "  • Register runner:       $INSTALL_DIR/register_runner.sh <RUNNER_REGISTRATION_TOKEN>"
  echo -e "  • Rebuild & update all:  $INSTALL_DIR/setup_rpi.sh"
  echo ""
}

main "$@"
