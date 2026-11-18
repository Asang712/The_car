# -*- coding: utf-8 -*-
"""Runtime car capability selection.

The ``multi_brains`` node drives several hardware variants of the same
MicroROS car platform. Each variant exposes a different subset of sensors and
actuators, so the action layer needs a single source of truth for "does this
car have a camera? a pan/tilt head?" before dispatching an action.

Car type resolution order (see :func:`resolve_car_type`):

1. The ``CAR_TYPE`` environment variable (case-insensitive, whitespace trimmed).
2. The ``SYSTEM.hardware_variant`` config value.
3. The :data:`CAR_TYPE_PTZ` default.

Both sources are validated against :data:`VALID_CAR_TYPES`; an invalid value
falls through to the next source so a misconfigured environment never crashes
the node at startup.
"""

#: Pan/tilt car: camera + two controllable servos.
CAR_TYPE_PTZ = "ptz"
#: Fixed-camera car: camera but no controllable servos.
CAR_TYPE_NO_PTZ = "no_ptz"
#: Bare differential-drive base: no camera, no servos.
CAR_TYPE_BASIC = "basic_car"

#: All supported car type identifiers.
VALID_CAR_TYPES = {CAR_TYPE_PTZ, CAR_TYPE_NO_PTZ, CAR_TYPE_BASIC}

#: Human-readable labels, used in capability-rejection feedback to Dify.
CAR_TYPE_LABELS = {
    CAR_TYPE_PTZ: "PTZ 云台车",
    CAR_TYPE_NO_PTZ: "无云台车",
    CAR_TYPE_BASIC: "基础车",
}

#: Default car type when neither environment nor config yields a valid value.
_DEFAULT_CAR_TYPE = CAR_TYPE_PTZ


def _normalize(value):
    """Lower-case and trim a candidate car type string."""
    return str(value or "").strip().lower()


def resolve_car_type(env_value, config_value):
    """Resolve the active car type from environment and config sources.

    Args:
        env_value: Raw ``CAR_TYPE`` environment value (may be empty/invalid).
        config_value: ``SYSTEM.hardware_variant`` config value.

    Returns:
        One of the ``CAR_TYPE_*`` constants. Invalid sources are skipped, and
        when no source is valid :data:`CAR_TYPE_PTZ` is returned.
    """
    env_car_type = _normalize(env_value)
    if env_car_type in VALID_CAR_TYPES:
        return env_car_type

    config_car_type = _normalize(config_value)
    if config_car_type in VALID_CAR_TYPES:
        return config_car_type

    return _DEFAULT_CAR_TYPE


def has_camera(car_type):
    """Return whether the car type carries a camera.

    Both PTZ and No-PTZ cars have cameras; the basic car does not.
    """
    return car_type in (CAR_TYPE_PTZ, CAR_TYPE_NO_PTZ)


def has_ptz(car_type):
    """Return whether the car type has a controllable pan/tilt head.

    Only the PTZ car has controllable servos.
    """
    return car_type == CAR_TYPE_PTZ
