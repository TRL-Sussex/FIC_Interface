"""
Tests for the Script Control API in the docs and the Help page.

Verifies:
- ``api_reference(api)`` covers every public method on ``RobotAPI`` and
  every sub-API (state, motion, mode, process, target, tune, safety, topics).
- The Help Scripting section is present in ``HELP_SECTIONS`` and renders
  non-empty HTML containing expected content (robot facade description,
  flat-alias table, examples).
- The Scripting section renders correctly in both warmth_dark and warmth_light
  themes via the mixin.
- ``docs/scripting/API.md`` exists and is non-empty.
"""
import os
import pathlib
import sys as _sys
import threading

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

# Ensure the Extensions package is importable from the repo root
_REPO_ROOT = str(
    pathlib.Path(__file__).resolve().parent.parent.parent.parent.parent
)
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)

from PyQt5.QtWidgets import QApplication, QMainWindow  # noqa: E402

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# Stubs for building a RobotAPI without a live ROS node
# ---------------------------------------------------------------------------

class _StubNode:
    """Minimal node stub for RobotAPI / api_reference generation."""

    _MODE_NAMES = [
        'Joint Impedance', 'Joint Planner', 'Joint Admittance',
        'Joint Admittance Haptic', 'Cartesian Impedance',
        'Cartesian Planner', 'Cartesian Admittance',
        'Cartesian Admittance Haptic',
    ]

    def publish_mode(self, *_a, **_kw):
        """No-op publish stub."""

    def publish_postural_enable(self, *_a, **_kw):
        """No-op publish stub."""

    def publish_elbow_enable(self, *_a, **_kw):
        """No-op publish stub."""

    def publish_hpmp_enable(self, *_a, **_kw):
        """No-op publish stub."""

    def publish_admittance_planner(self, *_a, **_kw):
        """No-op publish stub."""

    def publish_param_update(self, *_a, **_kw):
        """No-op publish stub."""

    def publish_joint_angles(self, *_a, **_kw):
        """No-op publish stub."""

    def publish_real_joint_angles(self, *_a, **_kw):
        """No-op publish stub."""

    def publish_position(self, *_a, **_kw):
        """No-op publish stub."""

    def publish_gripper_width(self, *_a, **_kw):
        """No-op publish stub."""

    def get_topic_names_and_types(self):
        """Return empty topic list stub."""
        return []

    active_profile_publisher_ = None


class _StubHost:
    """Minimal host stub for RobotAPI construction."""

    _mw = None

    def joint_positions_for(self, _rid):
        """Return empty joint list stub."""
        return []

    def joint_velocity_for(self, _rid):
        """Return empty velocity list stub."""
        return []

    def joint_effort_for(self, _rid):
        """Return empty effort list stub."""
        return []

    def joint_commanded_for(self, _rid):
        """Return empty commanded list stub."""
        return []

    def ee_pose_for(self, _rid):
        """Return None EE pose stub."""
        return None

    def wrench_for(self, _rid):
        """Return None wrench stub."""
        return None

    def robot_target(self, _rid):
        """Return simulation target string stub."""
        return 'simulation'

    def broadcast_stop(self):
        """No-op broadcast stop stub."""


def _make_stub_api():
    """Return a ``RobotAPI`` built from stubs (no live ROS required)."""
    from Extensions.script_editor.robot_api import RobotAPI
    stop_ev = threading.Event()
    return RobotAPI(
        host=_StubHost(),
        node=_StubNode(),
        rid='robot1',
        target='simulation',
        marshal=lambda fn, **_kw: None,
        is_stopped=stop_ev.is_set,
    )


def _make_mixin_window(theme_name='warmth_dark'):
    """Return a minimal QMainWindow that mixes in HelpMixin."""
    from fic_studio.theme import apply_theme
    from fic_studio.mixins.help_mixin import HelpMixin

    class _StubWindow(QMainWindow, HelpMixin):
        """Minimal stub that satisfies HelpMixin dependencies."""

        def log_message(self, *_args):
            """Stub logger."""

    win = _StubWindow()
    apply_theme(win, theme_name)
    return win


# ---------------------------------------------------------------------------
# api_reference coverage
# ---------------------------------------------------------------------------

class TestApiReferenceCoverage:
    """Verify api_reference covers every public method on RobotAPI."""

    def _get_entries(self):
        """Return list of api_reference entry dicts."""
        from Extensions.script_editor.robot_api import api_reference
        api = _make_stub_api()
        return api_reference(api)

    def test_api_reference_returns_list(self):
        """api_reference must return a non-empty list."""
        entries = self._get_entries()
        assert isinstance(entries, list)
        assert len(entries) > 0

    def test_entries_have_required_keys(self):
        """Every entry must have 'name', 'signature', and 'doc' keys."""
        entries = self._get_entries()
        for entry in entries:
            assert 'name' in entry, f'Entry missing "name": {entry}'
            assert 'signature' in entry, f'Entry missing "signature": {entry}'
            assert 'doc' in entry, f'Entry missing "doc": {entry}'

    def test_state_api_joints_covered(self):
        """robot.state.joints must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.state.joints' in names, (
            f'"robot.state.joints" not found; got: {sorted(names)[:10]}'
        )

    def test_state_api_ee_pose_covered(self):
        """robot.state.ee_pose must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.state.ee_pose' in names

    def test_state_api_wrench_covered(self):
        """robot.state.wrench must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.state.wrench' in names

    def test_motion_move_joints_covered(self):
        """robot.motion.move_joints must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.motion.move_joints' in names

    def test_motion_move_cartesian_covered(self):
        """robot.motion.move_cartesian must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.motion.move_cartesian' in names

    def test_motion_set_gripper_covered(self):
        """robot.motion.set_gripper must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.motion.set_gripper' in names

    def test_mode_set_covered(self):
        """robot.mode.set must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.mode.set' in names

    def test_mode_postural_covered(self):
        """robot.mode.postural must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.mode.postural' in names

    def test_process_start_sim_covered(self):
        """robot.process.start_sim must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.process.start_sim' in names

    def test_process_stop_bags_covered(self):
        """robot.process.stop_bags must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.process.stop_bags' in names

    def test_safety_estop_covered(self):
        """robot.safety.estop must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.safety.estop' in names

    def test_topics_subscribe_covered(self):
        """robot.topics.subscribe must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.topics.subscribe' in names

    def test_tune_param_update_covered(self):
        """robot.tune.param_update must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.tune.param_update' in names

    def test_target_current_covered(self):
        """robot.target.current must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.target.current' in names

    def test_robot_wait_covered(self):
        """robot.wait must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.wait' in names

    def test_robot_is_stopped_covered(self):
        """robot.is_stopped must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'robot.is_stopped' in names

    def test_flat_alias_move_joints_present(self):
        """Flat alias 'move_joints' must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'move_joints' in names

    def test_flat_alias_get_joint_positions_present(self):
        """Flat alias 'get_joint_positions' must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'get_joint_positions' in names

    def test_flat_alias_set_mode_present(self):
        """Flat alias 'set_mode' must appear in api_reference."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        assert 'set_mode' in names

    def test_all_sub_apis_represented(self):
        """api_reference must include entries from all eight sub-namespaces."""
        entries = self._get_entries()
        names = {e['name'] for e in entries}
        sub_apis = [
            'robot.state', 'robot.motion', 'robot.mode', 'robot.process',
            'robot.target', 'robot.tune', 'robot.safety', 'robot.topics',
        ]
        for ns in sub_apis:
            found = any(n.startswith(ns + '.') for n in names)
            assert found, f'No entries found for sub-namespace {ns!r}'

    def test_docs_covered_signatures_non_empty(self):
        """Every entry in api_reference must have a non-None signature."""
        entries = self._get_entries()
        for entry in entries:
            assert entry['signature'] is not None, (
                f'{entry["name"]}: signature is None'
            )


# ---------------------------------------------------------------------------
# Help Scripting section in HELP_SECTIONS
# ---------------------------------------------------------------------------

class TestHelpScriptingSection:
    """Verify the Scripting section is present and renders correctly."""

    def test_scripting_key_present_in_help_sections(self):
        """'scripting' must be a key in HELP_SECTIONS."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        keys = {k for k, _, __ in HELP_SECTIONS}
        assert 'scripting' in keys, (
            f'"scripting" not found in HELP_SECTIONS; keys={sorted(keys)}'
        )

    def test_scripting_section_has_callable_content(self):
        """The scripting section content must be a callable (lazy generator)."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        for key, _title, content in HELP_SECTIONS:
            if key == 'scripting':
                assert callable(content), (
                    f'scripting content should be callable, got {type(content)}'
                )
                return
        raise AssertionError('"scripting" section not found in HELP_SECTIONS')

    def test_scripting_section_title(self):
        """The scripting section title must be 'Scripting'."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        for key, title, _ in HELP_SECTIONS:
            if key == 'scripting':
                assert title == 'Scripting', (
                    f'Expected title "Scripting", got {title!r}'
                )
                return
        raise AssertionError('"scripting" section not found in HELP_SECTIONS')

    def test_scripting_html_returns_string(self):
        """Calling the scripting content callable must return a str."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        for key, _, content in HELP_SECTIONS:
            if key == 'scripting':
                result = content()
                assert isinstance(result, str), (
                    f'Expected str from scripting content(), got {type(result)}'
                )
                return
        raise AssertionError('"scripting" section not found')

    def test_scripting_html_non_empty(self):
        """The scripting section HTML must be non-empty."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        for key, _, content in HELP_SECTIONS:
            if key == 'scripting':
                html = content()
                assert len(html) > 100, (
                    f'Scripting HTML appears empty (length={len(html)})'
                )
                return
        raise AssertionError('"scripting" section not found')

    def test_scripting_html_contains_robot_facade(self):
        """Scripting HTML must mention the robot facade."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        for key, _, content in HELP_SECTIONS:
            if key == 'scripting':
                html = content()
                assert 'robot' in html.lower(), (
                    'Scripting HTML must describe the robot facade'
                )
                return
        raise AssertionError('"scripting" section not found')

    def test_scripting_html_contains_flat_aliases(self):
        """Scripting HTML must mention flat alias functions."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        for key, _, content in HELP_SECTIONS:
            if key == 'scripting':
                html = content()
                assert 'move_joints' in html, (
                    'Scripting HTML must contain move_joints alias description'
                )
                return
        raise AssertionError('"scripting" section not found')

    def test_scripting_html_contains_example(self):
        """Scripting HTML must include at least one code example."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        for key, _, content in HELP_SECTIONS:
            if key == 'scripting':
                html = content()
                assert '<pre>' in html or 'robot.state' in html, (
                    'Scripting HTML must contain a code example'
                )
                return
        raise AssertionError('"scripting" section not found')

    def test_scripting_section_renders_via_mixin_dark(self):
        """warmth_dark: scripting section must render non-empty HTML via mixin."""
        win = _make_mixin_window('warmth_dark')
        page = win._build_help_page()
        win.setCentralWidget(page)
        win._help_render_section('scripting')
        html = win._help_browser.toHtml()
        assert 'robot' in html.lower(), (
            'warmth_dark scripting render must mention "robot"'
        )
        assert len(html) > 200, (
            f'warmth_dark scripting HTML too short (length={len(html)})'
        )

    def test_scripting_section_renders_via_mixin_light(self):
        """warmth_light: scripting section must render non-empty HTML via mixin."""
        win = _make_mixin_window('warmth_light')
        page = win._build_help_page()
        win.setCentralWidget(page)
        win._help_render_section('scripting')
        html = win._help_browser.toHtml()
        assert 'robot' in html.lower(), (
            'warmth_light scripting render must mention "robot"'
        )

    def test_scripting_nav_item_visible(self):
        """The 'Scripting' nav item must appear in the help nav list."""
        win = _make_mixin_window()
        page = win._build_help_page()
        win.setCentralWidget(page)
        titles = [
            win._help_nav.item(i).text()
            for i in range(win._help_nav.count())
        ]
        assert 'Scripting' in titles, (
            f'"Scripting" not found in nav list; titles={titles}'
        )

    def test_help_sections_count_is_thirteen(self):
        """HELP_SECTIONS must have exactly 13 sections."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        assert len(HELP_SECTIONS) == 13, (
            f'Expected 13 sections, got {len(HELP_SECTIONS)}'
        )


# ---------------------------------------------------------------------------
# API.md doc file
# ---------------------------------------------------------------------------

class TestApiMarkdownFile:
    """Verify that docs/scripting/API.md exists and is well-formed."""

    def _api_md_path(self):
        """Return the absolute path to docs/scripting/API.md."""
        return pathlib.Path(_REPO_ROOT) / 'docs' / 'scripting' / 'API.md'

    def test_api_md_exists(self):
        """docs/scripting/API.md must exist."""
        p = self._api_md_path()
        assert p.exists(), f'docs/scripting/API.md not found at {p}'

    def test_api_md_non_empty(self):
        """docs/scripting/API.md must be non-empty."""
        p = self._api_md_path()
        text = p.read_text(encoding='utf-8')
        assert len(text) > 500, (
            f'docs/scripting/API.md appears too short ({len(text)} chars)'
        )

    def test_api_md_contains_getting_started(self):
        """docs/scripting/API.md must contain a Getting Started section."""
        p = self._api_md_path()
        text = p.read_text(encoding='utf-8')
        assert 'Getting Started' in text

    def test_api_md_contains_robot_state(self):
        """docs/scripting/API.md must document robot.state."""
        p = self._api_md_path()
        text = p.read_text(encoding='utf-8')
        assert 'robot.state' in text

    def test_api_md_contains_move_joints(self):
        """docs/scripting/API.md must document move_joints."""
        p = self._api_md_path()
        text = p.read_text(encoding='utf-8')
        assert 'move_joints' in text

    def test_api_md_contains_examples(self):
        """docs/scripting/API.md must include example scripts."""
        p = self._api_md_path()
        text = p.read_text(encoding='utf-8')
        assert 'Example' in text

    def test_api_md_contains_all_namespaces(self):
        """docs/scripting/API.md must document all 8 sub-namespaces."""
        p = self._api_md_path()
        text = p.read_text(encoding='utf-8')
        namespaces = [
            'robot.state', 'robot.motion', 'robot.mode', 'robot.process',
            'robot.target', 'robot.tune', 'robot.safety', 'robot.topics',
        ]
        for ns in namespaces:
            assert ns in text, f'Namespace {ns!r} not found in API.md'

    def test_api_md_contains_flat_aliases_section(self):
        """docs/scripting/API.md must contain a Flat Aliases section."""
        p = self._api_md_path()
        text = p.read_text(encoding='utf-8')
        assert 'Flat Aliases' in text or 'flat alias' in text.lower()

    def test_api_md_contains_safety_notes(self):
        """docs/scripting/API.md must contain a safety notes section."""
        p = self._api_md_path()
        text = p.read_text(encoding='utf-8')
        assert 'Safety' in text or 'E-STOP' in text or 'estop' in text.lower()
