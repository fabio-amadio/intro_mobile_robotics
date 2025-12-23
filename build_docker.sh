#!/usr/bin/env bash

IMAGE_NAME=ghcr.io/fabio-amadio/turtlebot3-sim:2026
docker build -t ${IMAGE_NAME} .
