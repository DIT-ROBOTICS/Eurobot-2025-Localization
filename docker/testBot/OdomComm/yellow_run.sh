#!/bin/bash

## 0. clean container within same group
echo "=== [COMMUNICATION] Pull & Run ==="
echo "[COMMUNICATION] Remove Containers ..."
docker compose -p communication down --volumes --remove-orphans

## 2. environment setup  
export DISPLAY=:0
xhost +local:docker
cd docker

## 3. build image
echo "[COMMUNICATION] Pull Images ..."
docker pull pomelo925/ttennis-humble:communication

## 4. deployment
echo "[COMMUNICATION] Deploying..."
docker compose -p communication -f blue-compose.yml up
