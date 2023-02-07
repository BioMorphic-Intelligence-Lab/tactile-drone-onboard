#!/bin/bash
docker buildx build --platform linux/arm64,linux/amd64 -t antbre/tactile-drone-rpi --push .