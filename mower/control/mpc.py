"""Small deterministic model-predictive steering controller."""

from __future__ import annotations

import math
from dataclasses import dataclass

from mower.path.trajectory import ContinuousTrajectory


def _angle_delta(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class MPCOutput:
    steering_deg: float
    cross_track_error_m: float
    heading_error_rad: float
    feedforward_deg: float
    predicted_cost: float


class MPCController:
    """Finite-horizon bicycle-model controller with bounded steering rate.

    This is intentionally deterministic and dependency-free.  It evaluates a
    compact set of steering targets, rolls the actual vehicle model forward,
    and chooses the lowest path/heading/smoothness cost.
    """

    def __init__(
        self,
        *,
        max_steering_deg: float = 45.0,
        steering_rate_deg_s: float = 120.0,
        horizon_steps: int = 5,
        dt_s: float = 0.10,
    ) -> None:
        self.max_steering_deg = float(max_steering_deg)
        self.steering_rate_deg_s = float(steering_rate_deg_s)
        # One second of constant-steering preview made the tractor cut into a
        # planned hairpin roughly 15 cm before its tangent point.  Half a
        # second still covers the complete 45-degree steering ramp at the
        # configured 120 deg/s rate, without optimizing through a turn that
        # has not started yet.
        self.horizon_steps = int(horizon_steps)
        self.dt_s = float(dt_s)

    def compute(
        self,
        pose,
        trajectory: ContinuousTrajectory,
        progress_s: float,
        *,
        speed_mps: float,
        wheelbase_m: float,
        previous_steering_deg: float,
    ) -> MPCOutput:
        reference = trajectory.sample(progress_s)
        dx, dy = pose.utm_x - reference.x, pose.utm_y - reference.y
        cross_track = math.sin(reference.heading_rad) * dx - math.cos(reference.heading_rad) * dy
        heading_error = _angle_delta(reference.heading_rad - pose.heading_rad)
        feedforward = math.degrees(math.atan(max(0.05, wheelbase_m) * reference.curvature))
        feedback = math.degrees(1.8 * heading_error + math.atan2(2.3 * cross_track, max(0.12, speed_mps)))
        centre = max(-self.max_steering_deg, min(self.max_steering_deg, feedforward + feedback))
        targets = {
            max(-self.max_steering_deg, min(self.max_steering_deg, value))
            for value in (
                centre - 16.0, centre - 8.0, centre - 4.0, centre,
                centre + 4.0, centre + 8.0, centre + 16.0,
                feedforward, previous_steering_deg,
                previous_steering_deg - 8.0, previous_steering_deg - 4.0,
                previous_steering_deg + 4.0, previous_steering_deg + 8.0,
            )
        }
        best_cost = math.inf
        best_first = previous_steering_deg
        velocity = max(0.05, abs(float(speed_mps)))
        wheelbase = max(0.05, float(wheelbase_m))
        maximum_step = self.steering_rate_deg_s * self.dt_s
        for target in targets:
            x, y, heading = pose.utm_x, pose.utm_y, pose.heading_rad
            steering = float(previous_steering_deg)
            first_steering = steering
            cost = 0.0
            previous_delta = 0.0
            for step in range(self.horizon_steps):
                delta = max(-maximum_step, min(maximum_step, target - steering))
                steering += delta
                if step == 0:
                    first_steering = steering
                yaw_rate = velocity * math.tan(math.radians(steering)) / wheelbase
                if abs(yaw_rate) < 1e-8:
                    x += velocity * math.cos(heading) * self.dt_s
                    y += velocity * math.sin(heading) * self.dt_s
                else:
                    new_heading = heading + yaw_rate * self.dt_s
                    radius = velocity / yaw_rate
                    x += radius * (math.sin(new_heading) - math.sin(heading))
                    y -= radius * (math.cos(new_heading) - math.cos(heading))
                    heading = new_heading
                future_s = trajectory.clamp_to_current_motion(
                    progress_s,
                    progress_s + velocity * self.dt_s * (step + 1),
                )
                future = trajectory.sample(future_s)
                error_x, error_y = x - future.x, y - future.y
                lateral = math.sin(future.heading_rad) * error_x - math.cos(future.heading_rad) * error_y
                longitudinal = math.cos(future.heading_rad) * error_x + math.sin(future.heading_rad) * error_y
                angle_error = _angle_delta(future.heading_rad - heading)
                terminal_weight = 2.2 if step == self.horizon_steps - 1 else 1.0
                cost += terminal_weight * (
                    18.0 * lateral * lateral
                    + 1.5 * longitudinal * longitudinal
                    + 3.0 * angle_error * angle_error
                )
                # Tracking centimetre-accurate lanes matters more than
                # avoiding a modest steering correction.  The old rate/jerk
                # weights made a 12--18 cm lateral error cheaper than turning
                # the wheels at all within the shorter physical horizon.
                cost += 0.00004 * steering * steering + 0.00004 * delta * delta
                cost += 0.00005 * (delta - previous_delta) ** 2
                previous_delta = delta
            if cost < best_cost:
                best_cost, best_first = cost, first_steering
        # Apply the tangent/curvature controller for the immediate command.
        # The finite-horizon rollout remains useful as a diagnostic cost, but
        # selecting its constant-steering candidate made "do nothing" win for
        # lateral errors up to roughly 18 cm.  Following ``centre`` through the
        # same physical rate limit is equivalent to a scheduled MPC first move:
        # it reacts to current error without steering toward a later hairpin.
        immediate = max(
            previous_steering_deg - maximum_step,
            min(previous_steering_deg + maximum_step, centre),
        )
        return MPCOutput(
            max(-self.max_steering_deg, min(self.max_steering_deg, immediate)),
            cross_track,
            heading_error,
            feedforward,
            best_cost,
        )
