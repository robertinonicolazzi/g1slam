#!/usr/bin/env bash
set -e

IMAGE_NAME=g1_slam:humble

docker build -t "${IMAGE_NAME}" .