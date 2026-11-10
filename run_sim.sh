#!/bin/zsh
# PX4 SITL + Gazebo 시뮬레이터 실행 스크립트
# 사용법: zsh ~/drone/run_sim.sh [기체]   (기체 기본값: gz_x500)
# 예: zsh ~/drone/run_sim.sh gz_x500_mono_cam

cd "$HOME/drone/PX4-Autopilot" || exit 1
source .venv/bin/activate

# Gazebo와 PX4가 서로를 찾는 통신을 로컬로 고정 (macOS에서 "No route to host" 방지)
export GZ_IP=127.0.0.1

make px4_sitl "${1:-gz_x500}"
