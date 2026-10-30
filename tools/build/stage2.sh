#!/usr/bin/env bash
# Build actual ROS contracts, two CPU services and the independent Policy SDK.
set -euo pipefail
cd "$(dirname "$0")/../.."
docker build -t sim-dual-arm/ros-base:stage2 -f deploy/docker/ros-base/Dockerfile .
docker build --target mock -t sim-dual-arm/backend-mock:stage2 -f deploy/docker/backend-sim/Dockerfile .
docker build --target runtime -t sim-dual-arm/runtime:stage2 -f deploy/docker/runtime/Dockerfile .
docker build --target protocol -t sim-dual-arm/learning:protocol -f deploy/docker/learning/Dockerfile .
