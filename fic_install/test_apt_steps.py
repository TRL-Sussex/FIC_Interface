"""The installer's apt steps (detect-system.sh fic_apt_steps): fresh lists first, the required packages, then the
optional ones best-effort, all in one root step (one password prompt)."""
import subprocess
from pathlib import Path

DETECT = Path(__file__).with_name('detect-system.sh')


def steps(required, optional):
    out = subprocess.run(['bash', '-c', '. "$0" >/dev/null 2>&1; fic_apt_steps "$1" "$2"', str(DETECT), required, optional],
                         capture_output=True, text=True, check=True).stdout
    return out.splitlines()


def test_only_pinocchio_missing_still_refreshes_the_lists_first():
    lines = steps('', 'ros-jazzy-pinocchio')
    assert 'update -y' in lines[0]
    assert len(lines) == 2 and 'install -y ros-jazzy-pinocchio ||' in lines[1]
    assert 'exit 1' not in lines[1]           # optional: never fatal


def test_required_then_optional_in_one_script():
    lines = steps('libssl-dev zlib1g-dev', 'ros-jazzy-pinocchio')
    assert 'update -y' in lines[0]
    assert 'install -y libssl-dev zlib1g-dev || exit 1' in lines[1]
    assert 'install -y ros-jazzy-pinocchio' in lines[2]


def test_nothing_missing_is_no_step():
    assert steps('', '') == []
