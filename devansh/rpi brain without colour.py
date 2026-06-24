#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════
 MAZE ROBOT — Raspberry Pi Brain  (MECANUM STRAFE, FULLY INSTRUMENTED)
═══════════════════════════════════════════════════════════════════════════
"""
from __future__ import annotations
import logging
import time
import struct
import threading
import queue
import heapq
import glob
import sys
from collections import deque, defaultdict
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional
import serial
import serial.tools.list_ports

# ═══════════════════════════════════════════════════════════════════════════
#  CONFIG
# ═══════════════════════════════════════════════════════════════════════════
SERIAL_BAUD    = 115200
SERIAL_TIMEOUT = 0.05

PKT_H1, PKT_H2 = 0xAA, 0x55

CMD_MOVE_FWD    = 0x01
CMD_TURN_LEFT   = 0x02
CMD_TURN_RIGHT  = 0x03
CMD_UTURN       = 0x04
CMD_STOP        = 0x05
CMD_GET_SENSORS = 0x06
CMD_SET_SPEED   = 0x07

RSP_SENSOR_DATA = 0x81
RSP_DONE        = 0x82
RSP_ACK         = 0x83
RSP_ERROR       = 0x84

STATUS_OK      = 0
STATUS_STALL   = 1
STATUS_TIMEOUT = 2

CMD_NAMES = {
    CMD_MOVE_FWD: "MOVE_FWD(North)", CMD_TURN_LEFT: "TURN_LEFT(West)",
    CMD_TURN_RIGHT: "TURN_RIGHT(East)", CMD_UTURN: "UTURN(South)",
    CMD_STOP: "STOP", CMD_GET_SENSORS: "GET_SENSORS", CMD_SET_SPEED: "SET_SPEED",
}
RSP_NAMES = {
    RSP_SENSOR_DATA: "SENSOR_DATA", RSP_DONE: "DONE",
    RSP_ACK: "ACK", RSP_ERROR: "ERROR",
}
STATUS_NAMES = {STATUS_OK: "OK", STATUS_STALL: "STALL", STATUS_TIMEOUT: "TIMEOUT"}

CELL_SIZE_MM   = 200
WALL_MM        = 160 

NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DIR_DX = [ 0, +1,  0, -1]
DIR_DY = [+1,  0, -1,  0]
OPPOSITE = [SOUTH, WEST, NORTH, EAST]

VERBOSE_PROTOCOL = True

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger('MazeRobot')

def find_arduino_port() -> Optional[str]:
    for p in serial.tools.list_ports.comports():
        desc = (p.description or '') + (p.manufacturer or '')
        if 'Arduino' in desc or 'ACM' in p.device or 'ttyUSB' in p.device:
            return p.device
    for pattern in ('/dev/ttyACM*', '/dev/ttyUSB*'):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return None

class SerialLink:
    def __init__(self, port: Optional[str] = None):
        chosen = port or find_arduino_port()
        if chosen is None:
            log.error("Could not find the Arduino Mega. Check USB connection.")
            sys.exit(1)
        try:
            self._ser = serial.Serial(chosen, SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
        except (serial.SerialException, OSError) as e:
            log.error(f"Could not open {chosen}: {e}")
            sys.exit(1)

        log.info(f"SUCCESS: Connected to Mega on {chosen}")
        self.port = chosen
        self._lock = threading.Lock()
        self._qs   = defaultdict(queue.Queue)
        self._running = True
        self.rx_ok_count  = defaultdict(int)
        self.rx_crc_fail  = 0
        self._thread  = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()
        time.sleep(2.0) 
        self._drain()

    def get_sensors(self, timeout: float = 3.0) -> Optional[tuple]:
        self._drain(RSP_SENSOR_DATA)
        self._send(CMD_GET_SENSORS)
        payload = self._wait(RSP_SENSOR_DATA, timeout)
        return self._parse_sensors(payload)

    def move_forward(self, cells: int = 1, timeout: float = 10.0):
        self._drain(RSP_SENSOR_DATA, RSP_DONE)
        self._send(CMD_MOVE_FWD, bytes([cells]))
        sensors = self._parse_sensors(self._wait(RSP_SENSOR_DATA, timeout))
        status  = self._parse_done(self._wait(RSP_DONE, timeout))
        return status, sensors

    def strafe_left(self, timeout: float = 8.0): return self._do_move(CMD_TURN_LEFT, timeout)
    def strafe_right(self, timeout: float = 8.0): return self._do_move(CMD_TURN_RIGHT, timeout)
    def strafe_backward(self, timeout: float = 8.0): return self._do_move(CMD_UTURN, timeout)
    def stop(self): self._send(CMD_STOP)
    def set_speed(self, speed: int): self._send(CMD_SET_SPEED, bytes([max(60, min(255, speed))]))

    def close(self):
        self._running = False
        self._thread.join(timeout=1)
        self._ser.close()

    def link_health(self) -> str:
        ok = {RSP_NAMES.get(k, hex(k)): v for k, v in self.rx_ok_count.items()}
        return f"valid_packets={ok} crc_failures={self.rx_crc_fail}"

    def _do_move(self, cmd_byte: int, timeout: float):
        self._drain(RSP_SENSOR_DATA, RSP_DONE)
        self._send(cmd_byte)
        sensors = self._parse_sensors(self._wait(RSP_SENSOR_DATA, timeout))
        status  = self._parse_done(self._wait(RSP_DONE, timeout))
        return status, sensors

    def _crc(self, cmd: int, payload: bytes) -> int:
        crc = cmd ^ len(payload)
        for b in payload: crc ^= b
        return crc & 0xFF

    def _send(self, cmd: int, payload: bytes = b''):
        pkt = bytes([PKT_H1, PKT_H2, cmd, len(payload)]) + payload
        pkt += bytes([self._crc(cmd, payload)])
        if VERBOSE_PROTOCOL:
            name = CMD_NAMES.get(cmd, hex(cmd))
            print(f"  TX -> cmd={name:<18} bytes={pkt.hex(' ')}")
        with self._lock: self._ser.write(pkt)

    def _wait(self, cmd_byte: int, timeout: float) -> Optional[bytes]:
        try:
            payload = self._qs[cmd_byte].get(timeout=timeout)
            if VERBOSE_PROTOCOL:
                name = RSP_NAMES.get(cmd_byte, hex(cmd_byte))
                print(f"  RX <- rsp={name:<18} payload={payload.hex(' ') if payload else '(empty)'}")
            return payload
        except queue.Empty:
            if VERBOSE_PROTOCOL:
                name = RSP_NAMES.get(cmd_byte, hex(cmd_byte))
                print(f"  RX <- rsp={name:<18} ***NO RESPONSE within {timeout}s***")
            return None

    def _drain(self, *cmd_bytes):
        targets = cmd_bytes if cmd_bytes else list(self._qs.keys())
        for c in targets:
            while not self._qs[c].empty():
                try: self._qs[c].get_nowait()
                except queue.Empty: break

    @staticmethod
    def _parse_sensors(payload) -> Optional[tuple]:
        if payload and len(payload) == 8: return struct.unpack('>HHHH', payload)
        return None

    @staticmethod
    def _parse_done(payload) -> Optional[int]:
        return payload[0] if payload else None

    def _reader(self):
        state = cmd = length = 0
        payload = bytearray()
        while self._running:
            try: chunk = self._ser.read(1)
            except Exception: time.sleep(0.01); continue
            if not chunk: continue
            b = chunk[0]
            if state == 0:
                if b == PKT_H1: state = 1
            elif state == 1: state = 2 if b == PKT_H2 else 0
            elif state == 2: cmd = b; state = 3
            elif state == 3:
                length = b; payload = bytearray()
                state = 4 if length > 0 else 5
            elif state == 4:
                payload.append(b)
                if len(payload) == length: state = 5
            elif state == 5:
                crc = cmd ^ length
                for x in payload: crc ^= x
                state = 0
                if b == (crc & 0xFF):
                    self.rx_ok_count[cmd] += 1
                    self._qs[cmd].put(bytes(payload))
                else:
                    self.rx_crc_fail += 1
                    if VERBOSE_PROTOCOL: print(f"  RX !! CRC MISMATCH discarding byte 0x{b:02X}")

class Localizer:
    def __init__(self):
        self.x: int = 0; self.y: int = 0; self.heading: int = NORTH 
    def on_move(self, cmd: str):
        if   cmd == 'F': self.y += 1
        elif cmd == 'R': self.x += 1
        elif cmd == 'U': self.y -= 1
        elif cmd == 'L': self.x -= 1
        else: log.warning(f"Localizer.on_move: unknown command {cmd!r}")
    @property
    def pose(self) -> tuple[int, int, int]: return self.x, self.y, self.heading
    def direction_sequence_to_commands(self, directions: list[int]) -> list[str]:
        table = {NORTH: 'F', EAST: 'R', SOUTH: 'U', WEST: 'L'}
        return [table[d] for d in directions]

W_N, W_E, W_S, W_W = 0x1, 0x2, 0x4, 0x8
WALL_BIT = [W_N, W_E, W_S, W_W]

@dataclass
class Cell: walls: int = 0; visited: bool = False; seen: int = 0

class MazeMap:
    def __init__(self): self._grid: dict[tuple[int,int], Cell] = {}
    def cell(self, x: int, y: int) -> Cell:
        key = (x, y)
        if key not in self._grid: self._grid[key] = Cell()
        return self._grid[key]
    def visited(self, x: int, y: int) -> bool: return self._grid.get((x, y), Cell()).visited
    def mark_visited(self, x: int, y: int): self.cell(x, y).visited = True
    def has_wall(self, x: int, y: int, d: int) -> bool: return bool(self.cell(x, y).walls & WALL_BIT[d])
    def is_open(self, x: int, y: int, d: int) -> bool:
        c = self.cell(x, y)
        return bool(c.seen & WALL_BIT[d]) and not bool(c.walls & WALL_BIT[d])
    def set_wall(self, x: int, y: int, d: int):
        self.cell(x, y).walls |= WALL_BIT[d]; self.cell(x, y).seen  |= WALL_BIT[d]
        nx, ny = x + DIR_DX[d], y + DIR_DY[d]; opp = OPPOSITE[d]
        self.cell(nx, ny).walls |= WALL_BIT[opp]; self.cell(nx, ny).seen  |= WALL_BIT[opp]
    def clear_wall(self, x: int, y: int, d: int):
        self.cell(x, y).walls &= ~WALL_BIT[d]; self.cell(x, y).seen  |= WALL_BIT[d]
        nx, ny = x + DIR_DX[d], y + DIR_DY[d]; opp = OPPOSITE[d]
        self.cell(nx, ny).walls &= ~WALL_BIT[opp]; self.cell(nx, ny).seen  |= WALL_BIT[opp]
    def update_from_sensors(self, x: int, y: int, heading: int, sensors: tuple[int, int, int, int]):
        abs_dirs = [NORTH, SOUTH, WEST, EAST]
        for mm, d in zip(sensors, abs_dirs):
            if mm is None: continue
            if mm < WALL_MM: self.set_wall(x, y, d)
            else: self.clear_wall(x, y, d)
    def open_neighbors(self, x: int, y: int) -> list[tuple[int,int,int]]:
        return [(x + DIR_DX[d], y + DIR_DY[d], d) for d in (NORTH, EAST, SOUTH, WEST) if self.is_open(x, y, d)]
    def passable_neighbors(self, x: int, y: int) -> list[tuple[int,int,int]]:
        return [(x + DIR_DX[d], y + DIR_DY[d], d) for d in (NORTH, EAST, SOUTH, WEST) if not self.has_wall(x, y, d)]
    def bfs_nearest_unvisited(self, sx: int, sy: int) -> Optional[list[tuple[int,int]]]:
        if not self.visited(sx, sy): return [(sx, sy)]
        seen = {(sx, sy)}; q: deque[tuple[int, int, list]] = deque([(sx, sy, [(sx, sy)])])
        while q:
            x, y, path = q.popleft()
            for nx, ny, _ in self.passable_neighbors(x, y):
                if (nx, ny) in seen: continue
                seen.add((nx, ny)); new_path = path + [(nx, ny)]
                if not self.visited(nx, ny): return new_path
                q.append((nx, ny, new_path))
        return None
    def astar(self, sx: int, sy: int, gx: int, gy: int) -> Optional[list[tuple[int,int]]]:
        h = lambda x, y: abs(x - gx) + abs(y - gy)
        open_set: list = []; heapq.heappush(open_set, (h(sx, sy), 0, sx, sy, [(sx, sy)]))
        g_cost: dict[tuple[int,int], int] = {(sx, sy): 0}
        while open_set:
            _, g, x, y, path = heapq.heappop(open_set)
            if (x, y) == (gx, gy): return path
            if g > g_cost.get((x, y), 10**9): continue
            for nx, ny, _ in self.open_neighbors(x, y):
                ng = g + 1
                if ng < g_cost.get((nx, ny), 10**9):
                    g_cost[(nx, ny)] = ng; f = ng + h(nx, ny)
                    heapq.heappush(open_set, (f, ng, nx, ny, path + [(nx, ny)]))
        return None
    def cell_path_to_directions(self, path: list[tuple[int,int]]) -> list[int]:
        dirs = []
        for i in range(len(path) - 1):
            x1, y1 = path[i]; x2, y2 = path[i + 1]
            dx, dy = x2 - x1, y2 - y1
            for d in (NORTH, EAST, SOUTH, WEST):
                if DIR_DX[d] == dx and DIR_DY[d] == dy:
                    dirs.append(d); break
        return dirs
    def ascii_map(self) -> str:
        if not self._grid: return "(empty)"
        xs, ys = [k[0] for k in self._grid], [k[1] for k in self._grid]
        lines = []
        for y in range(max(ys), min(ys) - 1, -1):
            row = "".join(" ?" if (c := self._grid.get((x, y))) is None else " ." if c.visited else " o" for x in range(min(xs), max(xs) + 1))
            lines.append(f"y={y:+3d} {row}")
        return "\n".join(lines)

class ExplorationComplete(Exception): pass

class Explorer:
    def __init__(self, maze: MazeMap, loc: Localizer): self.maze = maze; self.loc = loc
    def is_complete(self) -> bool: return self.maze.bfs_nearest_unvisited(*self.loc.pose[:2]) is None
    def next_command_sequence(self):
        x, y, _ = self.loc.pose
        bfs_path = self.maze.bfs_nearest_unvisited(x, y)
        if bfs_path is None: raise ExplorationComplete
        gx, gy = bfs_path[-1]
        path = self.maze.astar(x, y, gx, gy) or bfs_path
        return self.loc.direction_sequence_to_commands(self.maze.cell_path_to_directions(path)), path[-1]
    def return_to_start(self) -> list[str]:
        x, y, _ = self.loc.pose
        if x == 0 and y == 0: return []
        path = self.maze.astar(x, y, 0, 0)
        return self.loc.direction_sequence_to_commands(self.maze.cell_path_to_directions(path)) if path else []

def execute_cmd(cmd: str, link: SerialLink, loc: Localizer, maze: MazeMap, verbose_summary: bool = False):
    before = loc.pose
    if   cmd == 'F': status, sensors = link.move_forward(1)
    elif cmd == 'L': status, sensors = link.strafe_left()
    elif cmd == 'R': status, sensors = link.strafe_right()
    elif cmd == 'U': status, sensors = link.strafe_backward()
    else: return False, None, None

    ok = (status == STATUS_OK)
    if ok: loc.on_move(cmd)
    x, y, h = loc.pose
    if sensors:
        maze.mark_visited(x, y); maze.update_from_sensors(x, y, h, sensors)

    if verbose_summary:
        print(f"  >>> cmd={cmd}  status={STATUS_NAMES.get(status, f'UNKNOWN({status})'):<8}  {'MOVED' if ok else 'DID NOT MOVE'}")
        print(f"      pose before={before}  after={loc.pose}")
        if sensors: print(f"      sensors(mm): F={sensors[0]} B={sensors[1]} L={sensors[2]} R={sensors[3]}  (wall if < {WALL_MM}mm)")
        else: print(f"      sensors: NONE RECEIVED")
        print(f"      link health: {link.link_health()}")
    return ok, sensors, status

def run_interactive(link: SerialLink):
    maze, loc = MazeMap(), Localizer()
    print("\n" + "=" * 70 + "\n INTERACTIVE SINGLE-STEP MODE\n Commands: f=forward(N)  b=back(U)  l=left(W)  r=right(E)  s=sensors  x=stop  q=quit\n" + "=" * 70)
    while True:
        try: raw = input("\ncmd> ").strip().lower()
        except (EOFError, KeyboardInterrupt): break
        if not raw: continue
        c = raw[0]
        if c == 'q': break
        if c == 's':
            if (sensors := link.get_sensors()): print(f"  sensors(mm): F={sensors[0]} B={sensors[1]} L={sensors[2]} R={sensors[3]}")
            else: print("  NO sensor response.")
            continue
        if c == 'x': link.stop(); print("  stop sent."); continue
        cmd_map = {'f': 'F', 'b': 'U', 'l': 'L', 'r': 'R'}
        if c not in cmd_map: print(f"  unrecognised: {raw!r}"); continue
        execute_cmd(cmd_map[c], link, loc, maze, verbose_summary=True)
    print(f"\nFinal tracked position: {loc.pose[:2]}\nFinal link health: {link.link_health()}")

class State(Enum): INIT = auto(); SENSE = auto(); PLAN = auto(); EXECUTE = auto(); RETURN = auto(); DONE = auto()

def run_autonomous(link: SerialLink):
    log.info("=== Mecanum Maze Robot — Autonomous Explore ===")
    maze, loc, explorer = MazeMap(), Localizer(), Explorer(MazeMap(), Localizer())
    explorer.maze, explorer.loc = maze, loc
    state, cmd_queue = State.INIT, []

    while state != State.DONE:
        try:
            if state == State.INIT:
                log.info("Initialising…")
                sensors = link.get_sensors()
                if sensors is None: log.error("No sensor response. Check USB."); time.sleep(1); continue
                x, y, h = loc.pose
                maze.mark_visited(x, y); maze.update_from_sensors(x, y, h, sensors)
                log.info(f"Start cell (0,0) sensors(mm)={sensors}")
                state = State.PLAN

            elif state == State.SENSE:
                sensors = link.get_sensors()
                x, y, h = loc.pose
                maze.mark_visited(x, y)
                if sensors: maze.update_from_sensors(x, y, h, sensors)
                log.info(f"Sensed ({x},{y}) walls={maze.cell(x,y).walls:04b}")
                state = State.PLAN

            elif state == State.PLAN:
                try:
                    cmds, target_cell = explorer.next_command_sequence()
                    cmd_queue = cmds
                    log.info(f"Plan: {len(cmds)} cmds → target {target_cell}")
                    state = State.EXECUTE
                except ExplorationComplete:
                    log.info("All reachable cells explored.")
                    state = State.RETURN

            elif state == State.EXECUTE:
                if not cmd_queue: state = State.SENSE; continue
                cmd = cmd_queue.pop(0)
                ok, sensors, status = execute_cmd(cmd, link, loc, maze, verbose_summary=VERBOSE_PROTOCOL)
                if not ok:
                    log.warning(f"Motion failed: cmd={cmd} status={STATUS_NAMES.get(status, str(status))}. Replanning…")
                    cmd_queue.clear(); state = State.SENSE
                else:
                    if sensors:
                        x, y, h = loc.pose
                        old_walls = maze.cell(x, y).walls
                        maze.update_from_sensors(x, y, h, sensors)
                        if maze.cell(x, y).walls != old_walls:
                            log.info("New wall detected during travel — replanning")
                            cmd_queue.clear(); state = State.PLAN

            elif state == State.RETURN:
                x, y, _ = loc.pose
                if x == 0 and y == 0:
                    log.info("=== Robot at start. Exploration complete. ===")
                    print(maze.ascii_map())
                    link.stop(); state = State.DONE; continue
                log.info(f"Returning to start from ({x},{y})…")
                cmds = explorer.return_to_start()
                if not cmds: log.warning("No return path found."); state = State.DONE; continue
                while cmds:
                    cmd = cmds.pop(0)
                    ok, sensors, status = execute_cmd(cmd, link, loc, maze, verbose_summary=VERBOSE_PROTOCOL)
                    if not ok:
                        log.warning("Return motion failed — replanning return.")
                        cmds = explorer.return_to_start()
                        if not cmds: break
                state = State.DONE

        except KeyboardInterrupt: log.info("Interrupted by user."); break
        except Exception as exc: log.exception(f"Unhandled exception: {exc}"); time.sleep(0.5)

    link.stop()
    log.info("Robot halted.\n" + link.link_health())

def main():
    link = SerialLink()
    print("\nChoose mode:\n  [1] Interactive single-step\n  [2] Autonomous explore")
    choice = input("> ").strip()
    try: run_autonomous(link) if choice == '2' else run_interactive(link)
    finally: link.close()

if __name__ == '__main__': main()
