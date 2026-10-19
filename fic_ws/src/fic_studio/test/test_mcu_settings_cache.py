"""controller_mcu_settings is asked twice a second by the status bar: parse the YAML only when it changes."""
import os

import yaml

from fic_studio import mcu_probe


def _write(path, port):
    path.write_text(yaml.safe_dump({'robots': {'Kinova': {'mcu': {'port': port}}}}))


def test_unchanged_file_is_parsed_once(tmp_path, monkeypatch):
    cfg = tmp_path / 'robots_config.yaml'
    _write(cfg, 7350)
    monkeypatch.setattr(mcu_probe, '_robots_config_path', lambda: str(cfg))
    parsed = []
    real = yaml.safe_load
    monkeypatch.setattr(mcu_probe.yaml, 'safe_load', lambda f: parsed.append(1) or real(f))
    mcu_probe._MCU_CACHE.clear()
    for _ in range(5):
        assert mcu_probe.controller_mcu_settings('Kinova') == {'port': 7350}
    assert len(parsed) == 1


def test_an_edited_file_is_read_again(tmp_path, monkeypatch):
    cfg = tmp_path / 'robots_config.yaml'
    _write(cfg, 7350)
    monkeypatch.setattr(mcu_probe, '_robots_config_path', lambda: str(cfg))
    mcu_probe._MCU_CACHE.clear()
    assert mcu_probe.controller_mcu_settings('Kinova') == {'port': 7350}
    _write(cfg, 7351)
    st = os.stat(cfg)
    os.utime(cfg, ns=(st.st_atime_ns, st.st_mtime_ns + 1_000_000))
    assert mcu_probe.controller_mcu_settings('Kinova') == {'port': 7351}


def test_a_missing_file_gives_no_settings(tmp_path, monkeypatch):
    monkeypatch.setattr(mcu_probe, '_robots_config_path', lambda: str(tmp_path / 'none.yaml'))
    mcu_probe._MCU_CACHE.clear()
    assert mcu_probe.controller_mcu_settings('Kinova') == {}
