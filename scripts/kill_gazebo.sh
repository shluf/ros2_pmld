#!/usr/bin/env bash

set -euo pipefail

PORT="${1:-}"
if [[ -z "$PORT" ]]; then
  if [[ -n "${GAZEBO_MASTER_URI:-}" ]]; then
    if [[ "$GAZEBO_MASTER_URI" =~ :([0-9]+)$ ]]; then
      PORT="${BASH_REMATCH[1]}"
    else
      PORT=11345
    fi
  else
    PORT=11345
  fi
fi

printf "[kill_gazebo] Target port: %s\n" "$PORT"

PIDS=$(pgrep -f "(^|/)(gzserver|gzclient|gazebo)( |$)" || true)
if [[ -n "$PIDS" ]]; then
  printf "[kill_gazebo] Sending SIGTERM to PIDs: %s\n" "$PIDS"
  kill $PIDS || true
  sleep 1
fi

PIDS=$(pgrep -f "(^|/)(gzserver|gzclient|gazebo)( |$)" || true)
if [[ -n "$PIDS" ]]; then
  printf "[kill_gazebo] Sending SIGKILL to PIDs: %s\n" "$PIDS"
  kill -9 $PIDS || true
  sleep 1
fi

if command -v lsof >/dev/null 2>&1; then
  LISTENERS=$(lsof -nP -iTCP:"$PORT" -sTCP:LISTEN -t || true)
  if [[ -n "$LISTENERS" ]]; then
    printf "[kill_gazebo] Killing listeners on port %s: %s\n" "$PORT" "$LISTENERS"
    kill $LISTENERS || true
    sleep 1
    LISTENERS=$(lsof -nP -iTCP:"$PORT" -sTCP:LISTEN -t || true)
    if [[ -n "$LISTENERS" ]]; then
      printf "[kill_gazebo] Forcing kill on remaining listeners: %s\n" "$LISTENERS"
      kill -9 $LISTENERS || true
      sleep 1
    fi
  fi
else
  if command -v ss >/dev/null 2>&1; then
    SS_LINE=$(ss -tlpn 2>/dev/null | awk -v p=":$PORT" '$4 ~ p {print $0}')
    if [[ -n "$SS_LINE" ]]; then
      PID=$(echo "$SS_LINE" | sed -n 's/.*pid=\([0-9]\+\).*/\1/p' | head -n1)
      if [[ -n "$PID" ]]; then
        printf "[kill_gazebo] Killing PID %s listening on port %s\n" "$PID" "$PORT"
        kill "$PID" || true
        sleep 1
        kill -9 "$PID" || true
      fi
    fi
  fi
fi

printf "[kill_gazebo] Done.\n" 
