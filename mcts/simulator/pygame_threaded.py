from __future__ import annotations

from dataclasses import dataclass, field
import queue
import threading
import time
from typing import TypeAlias

import pygame


Color: TypeAlias = tuple[int, int, int]
Point: TypeAlias = tuple[int, int]


@dataclass(slots=True)
class MouseClick:
    button: int
    pos: Point


@dataclass(slots=True)
class InputSnapshot:
    quit_requested: bool = False
    tab_pressed: bool = False
    mouse_clicks: list[MouseClick] = field(default_factory=list)
    up_pressed: bool = False
    down_pressed: bool = False
    left_pressed: bool = False
    right_pressed: bool = False


@dataclass(slots=True)
class Fill:
    color: Color


@dataclass(slots=True)
class Rect:
    x: int
    y: int
    w: int
    h: int
    color: Color
    width: int = 0


@dataclass(slots=True)
class Circle:
    center: Point
    radius: int
    color: Color
    width: int = 0


@dataclass(slots=True)
class Line:
    start: Point
    end: Point
    color: Color
    width: int = 1


@dataclass(slots=True)
class Lines:
    points: list[Point]
    color: Color
    width: int = 1
    closed: bool = False


@dataclass(slots=True)
class Text:
    text: str
    pos: Point
    color: Color


@dataclass(slots=True)
class Present:
    pass


DrawCommand: TypeAlias = Fill | Rect | Circle | Line | Lines | Text | Present


class ThreadedPygameRuntime:
    def __init__(
        self, window_size: tuple[int, int], title: str, font_name: str, font_size: int
    ) -> None:
        self.window_size = window_size
        self.title = title
        self.font_name = font_name
        self.font_size = font_size

        self._input_lock = threading.Lock()
        self._quit_requested = False
        self._tab_pressed = False
        self._mouse_clicks: list[MouseClick] = []
        self._up_pressed = False
        self._down_pressed = False
        self._left_pressed = False
        self._right_pressed = False

        self._frame_queue: queue.Queue[list[DrawCommand]] = queue.Queue(maxsize=1)
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._started_event = threading.Event()
        self._init_error: Exception | None = None

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("ThreadedPygameRuntime has already been started")
        self._thread = threading.Thread(target=self._run_ui_loop, daemon=True)
        self._thread.start()
        self._started_event.wait(timeout=5.0)
        if not self._started_event.is_set():
            raise RuntimeError("Timed out starting PyGame UI thread")
        if self._init_error is not None:
            raise RuntimeError("Failed to initialize PyGame UI thread") from self._init_error

    def stop(self) -> None:
        if self._thread is None:
            return
        self._stop_event.set()
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            raise RuntimeError("PyGame UI thread did not stop cleanly")
        self._thread = None

    def poll_input(self) -> InputSnapshot:
        with self._input_lock:
            snapshot = InputSnapshot(
                quit_requested=self._quit_requested,
                tab_pressed=self._tab_pressed,
                mouse_clicks=list(self._mouse_clicks),
                up_pressed=self._up_pressed,
                down_pressed=self._down_pressed,
                left_pressed=self._left_pressed,
                right_pressed=self._right_pressed,
            )
            self._tab_pressed = False
            self._mouse_clicks.clear()
        return snapshot

    def submit_frame(self, commands: list[DrawCommand]) -> None:
        try:
            self._frame_queue.put_nowait(commands)
            return
        except queue.Full:
            pass
        try:
            self._frame_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            self._frame_queue.put_nowait(commands)
        except queue.Full:
            pass

    def _run_ui_loop(self) -> None:
        screen: pygame.Surface | None = None
        font: pygame.font.Font | None = None
        last_frame: list[DrawCommand] | None = None
        try:
            pygame.init()
            screen = pygame.display.set_mode(self.window_size)
            pygame.display.set_caption(self.title)
            font = pygame.font.SysFont(self.font_name, self.font_size)
            self._started_event.set()
            while not self._stop_event.is_set():
                self._pump_input()
                frame = self._drain_latest_frame()
                if frame is not None:
                    last_frame = frame
                if last_frame is not None and screen is not None and font is not None:
                    self._execute_frame(last_frame, screen, font)
                time.sleep(0.005)
        except Exception as exc:  # pragma: no cover - startup/runtime safety path
            self._init_error = exc
            self._started_event.set()
        finally:
            pygame.quit()

    def _pump_input(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                with self._input_lock:
                    self._quit_requested = True
            elif event.type == pygame.KEYDOWN:
                self._on_key_event(event.key, pressed=True)
            elif event.type == pygame.KEYUP:
                self._on_key_event(event.key, pressed=False)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                with self._input_lock:
                    self._mouse_clicks.append(MouseClick(button=int(event.button), pos=tuple(event.pos)))

    def _on_key_event(self, key: int, pressed: bool) -> None:
        with self._input_lock:
            if key == pygame.K_TAB and pressed:
                self._tab_pressed = True
            elif key == pygame.K_UP:
                self._up_pressed = pressed
            elif key == pygame.K_DOWN:
                self._down_pressed = pressed
            elif key == pygame.K_LEFT:
                self._left_pressed = pressed
            elif key == pygame.K_RIGHT:
                self._right_pressed = pressed

    def _drain_latest_frame(self) -> list[DrawCommand] | None:
        latest: list[DrawCommand] | None = None
        while True:
            try:
                latest = self._frame_queue.get_nowait()
            except queue.Empty:
                return latest

    def _execute_frame(
        self, commands: list[DrawCommand], screen: pygame.Surface, font: pygame.font.Font
    ) -> None:
        needs_flip = False
        for cmd in commands:
            if isinstance(cmd, Fill):
                screen.fill(cmd.color)
            elif isinstance(cmd, Rect):
                pygame.draw.rect(screen, cmd.color, pygame.Rect(cmd.x, cmd.y, cmd.w, cmd.h), width=cmd.width)
            elif isinstance(cmd, Circle):
                pygame.draw.circle(screen, cmd.color, cmd.center, cmd.radius, width=cmd.width)
            elif isinstance(cmd, Line):
                pygame.draw.line(screen, cmd.color, cmd.start, cmd.end, cmd.width)
            elif isinstance(cmd, Lines):
                if len(cmd.points) >= 2:
                    pygame.draw.lines(screen, cmd.color, cmd.closed, cmd.points, width=cmd.width)
            elif isinstance(cmd, Text):
                text_surface = font.render(cmd.text, True, cmd.color)
                screen.blit(text_surface, cmd.pos)
            elif isinstance(cmd, Present):
                needs_flip = True
        if needs_flip:
            pygame.display.flip()
