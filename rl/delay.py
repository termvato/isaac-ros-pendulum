"""Fixed transport delay on actions, to mimic the ROS loop inside Isaac Lab.

The action applied at step k is the one computed at step k - steps; before that, zeros (the
controller was holding upright, commanding ~0, when the harness teleported the body).
"""

from collections import deque

import torch


class ActionDelay:
    def __init__(self, steps: int):
        if steps < 0:
            raise ValueError(f"delay must be >= 0, got {steps}")
        self.steps = steps
        self._queue: deque[torch.Tensor] = deque()

    def __call__(self, action: torch.Tensor) -> torch.Tensor:
        if self.steps == 0:
            return action
        if not self._queue:
            self._queue.extend(torch.zeros_like(action) for _ in range(self.steps))
        self._queue.append(action)
        return self._queue.popleft()
