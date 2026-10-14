"""Stations: which robotN_* topics a Studio drives, and what it is called.

Robot stations 1..MAX_ROBOTS own the topics ``robotN_*``, one per robot PC.

The Master is the control seat for a PC with no arm of its own (a lab's control
PC, a developer's laptop): it sees and commands the whole fleet. Any number of
Masters can run at once. The setting is ``station_id: 0`` (MASTER); at start-up
each Master takes its own internal station, MASTER_BASE..MASTER_TOP, derived from
the PC's name (``master_station``), so its own simulations run on topics nobody
else uses and can never reach a real arm. It is always shown as "Master".
"""
import zlib

#: Robot stations are 1..MAX_ROBOTS.
MAX_ROBOTS = 4
#: The settings value meaning "this Studio is a Master".
MASTER = 0
#: Internal stations of running Masters.
MASTER_BASE = 100
MASTER_TOP = 999


def master_station(host: str) -> int:
    """Return the internal station of a Master on ``host`` (stable, 100..999)."""
    span = MASTER_TOP - MASTER_BASE + 1
    return MASTER_BASE + zlib.crc32(str(host).encode()) % span


def resolve_station(setting, host: str) -> int:
    """Turn a ``station_id`` setting into the station a Studio runs as."""
    n = int(setting)
    return master_station(host) if n == MASTER else n


def is_master(n) -> bool:
    """Return True for the Master setting or a Master's internal station."""
    return n == MASTER or (isinstance(n, int) and MASTER_BASE <= n <= MASTER_TOP)


def is_robot(n) -> bool:
    """Return True for a robot station (1..MAX_ROBOTS)."""
    return isinstance(n, int) and not isinstance(n, bool) and 1 <= n <= MAX_ROBOTS


def valid_station(n) -> bool:
    """Return True if ``n`` is a usable station (a robot, the Master setting, or a Master)."""
    return isinstance(n, int) and not isinstance(n, bool) and (is_robot(n) or is_master(n))


def all_stations():
    """Return the stations a toolbar picker offers (robots, then Master)."""
    return list(range(1, MAX_ROBOTS + 1)) + [MASTER]


def picker_value(n: int) -> int:
    """Return the toolbar picker entry for a running station (Masters map to MASTER)."""
    return MASTER if is_master(n) else n


def station_rid(n: int) -> str:
    """Return the topic prefix of station ``n`` ('robotN')."""
    return 'robot%d' % n


def station_label(n) -> str:
    """Return how a station is shown: 'Master' or 'Robot N'."""
    return 'Master' if is_master(n) else 'Robot %s' % n


def rid_station(rid: str):
    """Return the station of a 'robotN' prefix, or None."""
    if isinstance(rid, str) and rid.startswith('robot'):
        try:
            n = int(rid[len('robot'):])
        except ValueError:
            return None
        return n if valid_station(n) else None
    return None
