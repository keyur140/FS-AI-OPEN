#!/usr/bin/env bash
# fsai-open: one-time install - builds the pinned Docker image with everything inside
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
# Usage: ./install.sh                 (needs Docker and internet once)
#        ./install.sh --ca /path.crt  (only if your network uses its own TLS certificate authority)
set -euo pipefail
cd "$(dirname "$0")"
IMAGE=fsai-open:0.2.0
command -v docker >/dev/null || { echo "Docker is not installed. Get it from https://docs.docker.com/get-docker/"; exit 1; }
docker info >/dev/null 2>&1 || { echo "Docker is installed but not running. Start Docker and try again."; exit 1; }
SECRET=()
if [[ "${1:-}" == "--ca" && -n "${2:-}" ]]; then SECRET=(--secret "id=extra_ca,src=$2"); fi
echo "Building $IMAGE (first time takes a few minutes)..."
DOCKER_BUILDKIT=1 docker build "${SECRET[@]}" -f docker/Dockerfile -t "$IMAGE" .
echo
echo "Installed. Next:"
echo "  ./run.sh demo    watch the bridge drive the virtual car through a full Acceleration run"
echo "  ./run.sh test    run every test and write local_runs/reports/RESULTS.md"
