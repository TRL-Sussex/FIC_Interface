# My Extension (template)

A FIC Studio extension with a web page. New extensions start as a copy of
`Extensions/_template`, made from the repository root with:

```bash
python3 fic_install/new_extension.py my_tool "My Tool"
```

Then enable the new extension in **Settings → Extensions** and restart the
Studio. (The Studio does not load folders whose name starts with `_`, so the
template itself never appears.)

| File | What it is |
|---|---|
| `manifest.json` | id, name, sidebar section, and the `web` block (page, poll rate, local-only actions) |
| `extension.py` | the Python side: `web_state()` (what the page shows), `web_action()` (what it does), lifecycle hooks |
| `web/page.js` | the page: `build(root, api)` draws it with the UI kit (`api.ui`), `update(state)` refreshes it |

The full guide: `fic_docs/pages/extensions.html` (Help → Writing Extensions in the Studio).
