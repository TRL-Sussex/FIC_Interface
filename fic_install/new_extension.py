#!/usr/bin/env python3
"""Make a new FIC Studio extension from the template.

    python3 fic_install/new_extension.py my_tool "My Tool" [--section "Extensions"]

Copies Extensions/_template to Extensions/<id>, renames the id, name, class
constants and topic, and says how to enable it. The guide is
fic_docs/pages/extensions.html ("Writing Extensions").
"""
import argparse
import json
import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
EXTENSIONS = ROOT / 'Extensions'
TEMPLATE = EXTENSIONS / '_template'
TEMPLATE_ID, TEMPLATE_NAME = 'my_extension', 'My Extension'


def main(argv=None):
    ap = argparse.ArgumentParser(description='Make a new FIC Studio extension from the template.')
    ap.add_argument('id', help='folder and id: lower case, digits and _ (e.g. my_tool)')
    ap.add_argument('name', help='the name in the sidebar (e.g. "My Tool")')
    ap.add_argument('--section', default='Extensions', help='sidebar section (default Extensions)')
    ap.add_argument('--description', default='', help='one line for the sidebar tooltip and Settings')
    args = ap.parse_args(argv)

    if not re.fullmatch(r'[a-z][a-z0-9_]{1,40}', args.id):
        sys.exit('The id must start with a letter and use only a-z, 0-9 and _ (2-41 characters).')
    target = EXTENSIONS / args.id
    if target.exists():
        sys.exit('%s already exists.' % target)
    taken = {json.loads(p.read_text()).get('id') for p in EXTENSIONS.glob('*/manifest.json')}
    if args.id in taken:
        sys.exit('Another extension already uses the id %r.' % args.id)

    shutil.copytree(TEMPLATE, target, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    description = args.description or 'Describe %s here.' % args.name

    manifest = json.loads((target / 'manifest.json').read_text())
    manifest.update(id=args.id, name=args.name, description=description, sidebar_section=args.section)
    (target / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')

    for rel in ('extension.py', 'web/page.js', 'README.md'):
        path = target / rel
        text = path.read_text()
        text = text.replace('ID = "%s"' % TEMPLATE_ID, 'ID = "%s"' % args.id)
        text = text.replace('NAME = "%s"' % TEMPLATE_NAME, 'NAME = %s' % json.dumps(args.name))
        text = re.sub(r'DESCRIPTION = ".*"', 'DESCRIPTION = %s' % json.dumps(description), text)
        text = text.replace('SIDEBAR_SECTION = "Extensions"', 'SIDEBAR_SECTION = %s' % json.dumps(args.section))
        text = text.replace(TEMPLATE_NAME, args.name).replace(TEMPLATE_ID, args.id)
        text = text.replace('# %s (template)' % args.name, '# %s' % args.name)
        path.write_text(text)

    print('Made %s' % target.relative_to(ROOT))
    print('  manifest.json   id %r, name %r, section %r' % (args.id, args.name, args.section))
    print('  extension.py    web_state() and web_action(): the Studio side')
    print('  web/page.js     the page, drawn with the UI kit')
    print('Next: enable it in the Studio (Settings > Extensions) and restart the Studio.')
    print('Guide: fic_docs/pages/extensions.html')
    return 0


if __name__ == '__main__':
    sys.exit(main())
