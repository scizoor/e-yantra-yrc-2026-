"""
"""

import json
import time
from collections import deque

import paho.mqtt.client as mqtt

MAZE_ROWS = 13
MAZE_COLS = 13
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
POSE_TOPIC = "robot/pose"

# wall bit per side, OR'd together
WALL_N, WALL_E, WALL_S, WALL_W = 0x1, 0x2, 0x4, 0x8

WALLS = [
    [12, 6, 12, 6, 13, 4, 0, 4, 5, 6, 12, 5, 6],
    [10, 11, 10, 10, 12, 3, 8, 2, 12, 1, 1, 6, 10],
    [8, 5, 3, 8, 1, 6, 9, 2, 9, 6, 13, 2, 10],
    [10, 12, 4, 3, 12, 1, 4, 0, 6, 9, 4, 2, 10],
    [10, 10, 8, 5, 3, 13, 2, 10, 9, 6, 10, 11, 10],
    [8, 3, 9, 4, 5, 6, 8, 1, 6, 10, 9, 5, 2],
    [10, 12, 4, 1, 6, 10, 9, 6, 10, 8, 5, 5, 2],
    [8, 1, 2, 12, 3, 10, 12, 3, 9, 2, 12, 6, 10],
    [9, 6, 10, 10, 12, 1, 2, 12, 5, 1, 0, 1, 3],
    [14, 8, 1, 1, 3, 12, 1, 3, 12, 4, 2, 12, 6],
    [8, 0, 4, 7, 12, 3, 12, 6, 10, 9, 1, 2, 10],
    [10, 10, 9, 6, 10, 12, 2, 8, 1, 7, 12, 0, 2],
    [9, 1, 5, 1, 1, 3, 8, 1, 5, 5, 3, 9, 3],
]

EXIT_CELLS = [
    (0, 6, 'south'),
    (MAZE_ROWS - 1, 6, 'north'),
]

BOT_CMD_TOPIC = "bot/cmd"
PELLETS_TOPIC = "pellets/pose"
CMD_VEL_TOPIC = "robot/cmd_vel"

HEADING_DELTA = {
    0.0:   (0, 1, WALL_E),
    90.0:  (1, 0, WALL_N),
    180.0: (0, -1, WALL_W),
    270.0: (-1, 0, WALL_S),
}


# ============================================================================
# YOUR ALGORITHM GOES HERE. Everything above and below is plumbing.
# ============================================================================
DELTA_TO_YAW = {(dr, dc): yaw for yaw, (dr, dc, _wall) in HEADING_DELTA.items()}


def _in_bounds(r, c):
    return 0 <= r < MAZE_ROWS and 0 <= c < MAZE_COLS


def _neighbors(r, c):
    cell_walls = WALLS[r][c]
    for yaw, (dr, dc, wall_bit) in HEADING_DELTA.items():
        if cell_walls & wall_bit:
            continue
        nr, nc = r + dr, c + dc
        if _in_bounds(nr, nc):
            yield nr, nc, yaw


def _bfs(start, goal):
    if start == goal:
        return [start]
    visited = {start}
    parent = {}
    queue = deque([start])
    while queue:
        cur = queue.popleft()
        for nr, nc, yaw in _neighbors(*cur):
            nxt = (nr, nc)
            if nxt in visited:
                continue
            visited.add(nxt)
            parent[nxt] = (cur, yaw)
            if nxt == goal:
                path = [nxt]
                node = nxt
                while node != start:
                    node, _ = parent[node]
                    path.append(node)
                path.reverse()
                return path
            queue.append(nxt)
    return None


def _turn_command(current_yaw, target_yaw):
    diff = (target_yaw - current_yaw) % 360
    if diff == 0:
        return None
    if diff == 90:
        return "LEFT"
    if diff == 180:
        return "BACK"
    return "RIGHT"


def choose_command(pacbot_cell, pacbot_yaw, pellets_remaining):
    r, c = pacbot_cell

    if pellets_remaining:
        best_path = None
        for pellet in pellets_remaining:
            path = _bfs(pacbot_cell, pellet)
            if path and (best_path is None or len(path) < len(best_path)):
                best_path = path
        print(f"[debug-bfs] pellets={pellets_remaining} best_path={best_path}")
        if not best_path or len(best_path) < 2:
            return None
        next_cell = best_path[1]
        target_yaw = DELTA_TO_YAW[(next_cell[0] - r, next_cell[1] - c)]
    else:
        exit_facing = {"north": 90.0, "south": 270.0, "east": 0.0, "west": 180.0}
        best_path = None
        best_exit = None
        for (er, ec, _facing) in EXIT_CELLS:
            path = _bfs(pacbot_cell, (er, ec))
            if path and (best_path is None or len(path) < len(best_path)):
                best_path = path
                best_exit = (er, ec)

        if best_exit is not None and pacbot_cell == best_exit:
            for (er, ec, facing) in EXIT_CELLS:
                if (er, ec) == best_exit:
                    target_yaw = exit_facing[facing]
                    break
        elif best_path and len(best_path) >= 2:
            next_cell = best_path[1]
            target_yaw = DELTA_TO_YAW[(next_cell[0] - r, next_cell[1] - c)]
        else:
            return None

    turn = _turn_command(pacbot_yaw, target_yaw)
    return turn if turn is not None else "FRONT"
# ============================================================================


def parse_pellets(payload):
    return {tuple(cell) for cell in json.loads(payload)}


def main():
    state = {
        "running": False,
        "pellets": set(),
        "cell": (MAZE_ROWS // 2, MAZE_COLS // 2),
        "yaw": 0.0,
        "in_flight": False,
        "got_pose": False,
        "got_pellets": False,
    }

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="Controller")

    def decide_and_send():
        if (not state["running"] or state["in_flight"]
                or not state["got_pose"] or not state["got_pellets"]):
            return
        print(f"[debug] pose={state['cell']} yaw={state['yaw']} pellets={state['pellets']}")
        cmd = choose_command(state["cell"], state["yaw"], set(state["pellets"]))
        if cmd is not None:
            state["in_flight"] = True
            client.publish(CMD_VEL_TOPIC, cmd)
            print(f"[controller] {state['cell']} yaw={state['yaw']} -> {cmd}, "
                  f"pellets_left={len(state['pellets'])}")

    def on_message(client, userdata, msg):
        try:
            if msg.topic == BOT_CMD_TOPIC:
                running = msg.payload.decode().startswith("1")
                was_running = state["running"]
                state["running"] = running
                if running and not was_running:
                    decide_and_send()
            elif msg.topic == PELLETS_TOPIC:
                state["pellets"] = parse_pellets(msg.payload.decode())
                state["got_pellets"] = True
                decide_and_send()
            elif msg.topic == POSE_TOPIC:
                data = json.loads(msg.payload.decode())
                state["cell"] = (int(data["col"]), int(data["row"]))
                state["got_pose"] = True
                state["yaw"] = float(data.get("yaw", 0.0))
                state["in_flight"] = False
                decide_and_send()
        except Exception as e:
            print("[controller] mqtt parse error:", e)

    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe([(BOT_CMD_TOPIC, 0), (PELLETS_TOPIC, 0), (POSE_TOPIC, 0)])
    client.loop_start()

    print(f"[controller] ready; sending one '{CMD_VEL_TOPIC}' command at a time, "
          f"reacting to '{POSE_TOPIC}'/'{PELLETS_TOPIC}' feedback")

    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
