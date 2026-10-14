"""config.xml is replaced in one step, never rewritten in place.

The controllers read that very file (macOS links the installed config folder to it, Linux
--symlink-install links its files), and a reload that caught a half-written file would run on
the loader's defaults.
"""
import os
import pathlib
import shutil
import stat

from fic_studio.xml_manager import XMLManager, find_config_path, write_atomic

CMAKE = pathlib.Path(__file__).resolve().parents[2] / 'fic_simulation' / 'CMakeLists.txt'


def test_write_atomic_replaces_the_file_keeps_its_mode_and_leaves_nothing_behind(tmp_path):
    path = tmp_path / 'config.xml'
    path.write_text('old')
    os.chmod(path, 0o640)
    before = path.stat().st_ino
    write_atomic(str(path), '<robots/>')
    assert path.read_text() == '<robots/>' and stat.S_IMODE(path.stat().st_mode) == 0o640
    assert path.stat().st_ino != before                                  # renamed over, not rewritten
    assert sorted(p.name for p in tmp_path.iterdir()) == ['config.xml']


def test_write_atomic_through_a_link_replaces_what_it_points_to(tmp_path):
    real = tmp_path / 'repo' / 'config.xml'
    real.parent.mkdir()
    real.write_text('old')
    installed = tmp_path / 'install'
    installed.symlink_to(real.parent, target_is_directory=True)        # as the macOS install links it
    write_atomic(str(installed / 'config.xml'), 'new')
    assert installed.is_symlink() and real.read_text() == 'new' and (installed / 'config.xml').read_text() == 'new'


def test_xml_manager_saves_through_it(tmp_path):
    path = tmp_path / 'config.xml'
    shutil.copy(find_config_path(), path)
    before = path.stat().st_ino
    mgr = XMLManager(str(path))
    mgr.save()
    assert path.stat().st_ino != before and XMLManager(str(path)).get_available_robots()
    assert [p.name for p in tmp_path.iterdir()] == ['config.xml']


def test_the_macos_build_links_the_config_folder_instead_of_copying_it():
    text = CMAKE.read_text()
    apple = text[text.index('if(APPLE)'):text.index('\nelse()', text.index('if(APPLE)'))]   # up to the Linux branch
    assert 'CREATE_LINK' in apple and 'SYMBOLIC' in apple
    assert 'IS_SYMLINK' in apple.split('REMOVE_RECURSE')[0]              # never recurse into the link
