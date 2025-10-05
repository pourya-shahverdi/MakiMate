# MakiMate

ROS 2 Jazzy codebase for the MAKI robot — Dockerized and ready for multi-arch (PC amd64 & Raspberry Pi 5 arm64) with CI/CD.

## Quick start (local)
```bash
docker build -t makimate:dev docker/robot
docker run --rm -it makimate:dev

