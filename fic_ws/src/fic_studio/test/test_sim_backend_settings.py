"""Simulation-backend setting: v4.5.0 retired the C++ 'mujoco' backend.

A settings file saved by an older Studio may still say `backend: mujoco`; it
must load as the CPU simulator (`mujoco_native`) instead of selecting a
package that no longer exists, and nothing may crash on odd values.
"""
import pytest
import yaml

pytest.importorskip('PyQt5')

from fic_studio.mixins.settings_mixin import (  # noqa: E402
    SIM_BACKEND_CPU, SIM_BACKEND_GPU, SettingsMixin, normalize_sim_backend)


@pytest.mark.parametrize('saved, expected', [
    ('mujoco', 'mujoco_native'),
    ('mujoco_native', 'mujoco_native'),
    ('newton', 'newton'),
    (None, 'mujoco_native'),
    ('', 'mujoco_native'),
    (42, 'mujoco_native'),
])
def test_normalize(saved, expected):
    assert normalize_sim_backend(saved) == expected


def _load(tmp_path, content):
    path = tmp_path / 'settings.yaml'
    path.write_text(content)

    class Probe(SettingsMixin):
        SETTINGS_FILE = str(path)
    return Probe()._load_settings()


def test_old_settings_file_with_cpp_backend_loads_as_cpu(tmp_path):
    loaded = _load(tmp_path, yaml.safe_dump({'simulation': {'backend': 'mujoco'}}))
    assert loaded['simulation']['backend'] == SIM_BACKEND_CPU


def test_newton_choice_survives(tmp_path):
    loaded = _load(tmp_path, yaml.safe_dump({'simulation': {'backend': 'newton'}}))
    assert loaded['simulation']['backend'] == SIM_BACKEND_GPU


def test_malformed_simulation_section_does_not_crash(tmp_path):
    loaded = _load(tmp_path, yaml.safe_dump({'simulation': 'mujoco'}))
    assert loaded['simulation']['backend'] == SIM_BACKEND_CPU
