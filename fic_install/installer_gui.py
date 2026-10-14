#!/usr/bin/env python3
"""
installer_gui.py — the installer's dialogs for FIC Control Studio.

tkinter, in the web interface's dark theme (installer_theme.py). The installer
falls back to zenity when tkinter is missing.

Subcommands:
    info      TITLE TEXT              Info dialog with OK button
    error     TITLE TEXT              Error dialog with OK button
    warning   TITLE TEXT              Warning dialog with OK button
    question  TITLE TEXT              Yes/No dialog (exit 0=Yes, 1=No)
    choice    TITLE TEXT OPT1 OPT2..  Radio-button selection (prints chosen option)
    progress  TITLE TEXT CMD          Run CMD, show live output + progress bar
    splash    TITLE TEXT SECONDS      Auto-closing info (splash screen)
"""

import sys
import os
import subprocess
import threading
import re

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from installer_theme import COLORS, MONO, SANS, pick_family  # noqa: E402

WIDTH = 500          # dialogs; the progress window is wider
PAD = 24
_FONTS = {}


def _fonts():
    """The web interface's fonts, from what is installed (needs a Tk root)."""
    if not _FONTS:
        import tkinter.font as tkfont
        families = tkfont.families()
        sans = pick_family(families, SANS, 'TkDefaultFont')
        mono = pick_family(families, MONO, 'TkFixedFont')
        _FONTS.update(body=(sans, 11), small=(sans, 9), bold=(sans, 11, 'bold'),
                      title=(sans, 14, 'bold'), icon=(sans, 16, 'bold'), mono=(mono, 10))
    return _FONTS


# ══════════════════════════════════════════════════════════════════════════════
#  HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def _fit(win, w, max_h=720):
    """Size the window to its content at width ``w`` and centre it."""
    win.update_idletasks()
    h = min(max(win.winfo_reqheight(), 160), max_h)
    x = (win.winfo_screenwidth() - w) // 2
    y = (win.winfo_screenheight() - h) // 2
    win.geometry(f'{w}x{h}+{x}+{y}')


def _make_root(title):
    import tkinter as tk
    root = tk.Tk()
    root.title(title)
    root.configure(bg=COLORS['canvas'])
    root.resizable(True, True)
    try:
        root.tk.call('tk', 'scaling', 1.0)
    except Exception:
        pass
    # Bring the dialog to the front: the installer pops dialogs in sequence, and a
    # buried one looks like a stuck installer.
    try:
        root.attributes('-topmost', True)
        root.after(400, lambda: root.attributes('-topmost', False))
        root.lift()
        root.focus_force()
    except Exception:
        pass
    _fonts()
    return root


def _styled_button(parent, text, command, primary=False, **kw):
    """A button as in the web interface: solid accent for the main action, quiet otherwise."""
    import tkinter as tk
    f = _fonts()
    if primary:
        bg, fg, hover = COLORS['accent'], COLORS['on_accent'], COLORS['accent_hover']
    else:
        bg, fg, hover = COLORS['canvas'], COLORS['ink'], COLORS['hover']
    btn = tk.Button(parent, text=text, command=command, bg=bg, fg=fg,
                    activebackground=hover, activeforeground=fg,
                    disabledforeground=COLORS['ink_faint'],
                    relief='flat', bd=0, highlightthickness=0,
                    padx=16, pady=7, font=f['bold'] if primary else f['body'],
                    cursor='hand2', **kw)
    btn.bind('<Enter>', lambda e: btn['state'] != 'disabled' and btn.configure(bg=hover))
    btn.bind('<Leave>', lambda e: btn['state'] != 'disabled' and btn.configure(bg=bg))
    btn.rest_bg = bg
    return btn


def _set_enabled(btn, on):
    """Enable or disable a button; a disabled one goes quiet, whatever its kind."""
    btn.configure(state='normal' if on else 'disabled',
                  bg=btn.rest_bg if on else COLORS['canvas'])


def _header_frame(root, title_text, icon_type='info'):
    """The title row: a coloured status glyph and the title."""
    import tkinter as tk
    f = _fonts()
    frame = tk.Frame(root, bg=COLORS['canvas'])
    frame.pack(fill='x', padx=PAD, pady=(PAD - 4, 0))
    icons = {
        'info':     ('i', COLORS['accent']),
        'error':    ('\u2715', COLORS['bad']),      # ✕
        'warning':  ('!', COLORS['warn']),
        'question': ('?', COLORS['accent']),
        'success':  ('\u2713', COLORS['ok']),       # ✓
        'build':    ('\u2022', COLORS['accent']),   # •
    }
    char, color = icons.get(icon_type, icons['info'])
    tk.Label(frame, text=char, font=f['icon'], fg=color, bg=COLORS['canvas'],
             width=2, anchor='w').pack(side='left')
    tk.Label(frame, text=title_text, font=f['title'], fg=COLORS['ink_strong'],
             bg=COLORS['canvas'], anchor='w', justify='left',
             wraplength=WIDTH - 2 * PAD - 40).pack(side='left', fill='x', expand=True)
    return frame


def _body_label(root, text, width=WIDTH):
    import tkinter as tk
    clean = text.replace('<b>', '').replace('</b>', '').replace('\\n', '\n')
    lbl = tk.Label(root, text=clean, font=_fonts()['body'], fg=COLORS['ink'],
                   bg=COLORS['canvas'], justify='left', wraplength=width - 2 * PAD,
                   anchor='w')
    lbl.pack(fill='x', padx=PAD, pady=(12, 0))
    return lbl


def _button_row(root):
    """A hairline, then a right-aligned row for the buttons (pack them side='right')."""
    import tkinter as tk
    tk.Frame(root, bg=COLORS['rule'], height=1).pack(fill='x', padx=PAD, pady=(PAD, 0))
    row = tk.Frame(root, bg=COLORS['canvas'])
    row.pack(fill='x', padx=PAD, pady=(12, 16))
    return row


# ══════════════════════════════════════════════════════════════════════════════
#  DIALOG: info / error / warning
# ══════════════════════════════════════════════════════════════════════════════

def dialog_message(title, text, icon_type='info', autoclose_seconds=0):
    """Show an info/error/warning/success dialog.

    autoclose_seconds > 0 makes the dialog dismiss itself after that many
    seconds with a visible countdown, while still showing OK so users can
    skip the wait. Used for purely-informational steps (e.g. "Step complete")
    so the installer doesn't strand users on a screen that needs no input.
    Errors and warnings always stay open — those need to be read.
    """
    import tkinter as tk
    root = _make_root(title)

    _header_frame(root, title, icon_type)
    _body_label(root, text)

    btn_frame = _button_row(root)
    ok_btn = _styled_button(btn_frame, 'OK', root.destroy, primary=True)
    ok_btn.pack(side='right')

    if autoclose_seconds > 0:
        remaining = tk.StringVar(value=f'Continuing in {autoclose_seconds}s…')
        tk.Label(btn_frame, textvariable=remaining, font=_fonts()['small'],
                 fg=COLORS['ink_muted'], bg=COLORS['canvas']).pack(side='left')

        def tick(n):
            if not root.winfo_exists():
                return
            if n <= 0:
                root.destroy()
                return
            remaining.set(f'Continuing in {n}s…')
            root.after(1000, tick, n - 1)

        root.after(0, tick, autoclose_seconds)

    _fit(root, WIDTH)
    root.bind('<Return>', lambda e: root.destroy())
    root.bind('<Escape>', lambda e: root.destroy())
    root.mainloop()


# ══════════════════════════════════════════════════════════════════════════════
#  DIALOG: question (Yes / No)
# ══════════════════════════════════════════════════════════════════════════════

def dialog_question(title, text, yes_label='Yes', no_label='No'):
    import tkinter as tk
    result = {'answer': False}
    root = _make_root(title)

    _header_frame(root, title, 'question')
    _body_label(root, text)
    btn_frame = _button_row(root)

    def on_yes():
        result['answer'] = True
        root.destroy()

    _styled_button(btn_frame, yes_label, on_yes, primary=True).pack(side='right')
    _styled_button(btn_frame, no_label, root.destroy).pack(side='right', padx=(0, 8))
    _fit(root, WIDTH)

    root.bind('<Return>', lambda e: on_yes())
    root.bind('<Escape>', lambda e: root.destroy())
    root.mainloop()
    return result['answer']


# ══════════════════════════════════════════════════════════════════════════════
#  DIALOG: splash (auto-close after N seconds)
# ══════════════════════════════════════════════════════════════════════════════

def dialog_splash(title, text, seconds=3):
    import tkinter as tk
    root = _make_root(title)

    _header_frame(root, title, 'info')
    _body_label(root, text)

    remaining = tk.StringVar(value=f'Closing in {seconds}s…')
    tk.Label(root, textvariable=remaining, font=_fonts()['small'],
             fg=COLORS['ink_muted'], bg=COLORS['canvas'], anchor='w').pack(
                 fill='x', padx=PAD, pady=(PAD, 16))
    _fit(root, WIDTH)

    def tick(n):
        if n <= 0:
            root.destroy()
            return
        remaining.set(f'Closing in {n}s…')
        root.after(1000, tick, n - 1)

    root.after(0, tick, seconds)
    root.bind('<Return>', lambda e: root.destroy())
    root.bind('<Escape>', lambda e: root.destroy())
    root.mainloop()


# ══════════════════════════════════════════════════════════════════════════════
#  DIALOG: choice (radio-button selection)
# ══════════════════════════════════════════════════════════════════════════════

def dialog_choice(title, text, options):
    """Show a radio-button selection dialog. Returns the chosen option string or None."""
    import tkinter as tk
    result = {'choice': None}
    # Height must account for BOTH the wrapped body text and the radio options,
    # otherwise long prompts (e.g. the Newton backend choice) push the
    # Cancel/Install buttons off the bottom of the window. Estimate the wrapped
    # body height from its line count at the ~420px wraplength (~55 chars/line).
    root = _make_root(title)

    _header_frame(root, title, 'question')
    _body_label(root, text)

    # The options, one per row, as in a web page's list
    selected = tk.StringVar(value=options[0])
    radio_frame = tk.Frame(root, bg=COLORS['canvas'])
    radio_frame.pack(fill='x', padx=PAD, pady=(14, 0))
    for opt in options:
        tk.Radiobutton(
            radio_frame, text=opt, variable=selected, value=opt,
            font=_fonts()['body'], fg=COLORS['ink'], bg=COLORS['surface'],
            selectcolor=COLORS['sunk'], activebackground=COLORS['hover'],
            activeforeground=COLORS['ink_strong'], highlightthickness=1,
            highlightbackground=COLORS['rule'], highlightcolor=COLORS['accent'],
            bd=0, anchor='w', justify='left', wraplength=WIDTH - 2 * PAD - 48,
            padx=10, pady=7, cursor='hand2').pack(fill='x', pady=(0, 6))

    btn_frame = _button_row(root)

    def on_ok():
        result['choice'] = selected.get()
        root.destroy()

    _styled_button(btn_frame, 'Continue', on_ok, primary=True).pack(side='right')
    _styled_button(btn_frame, 'Cancel', root.destroy).pack(side='right', padx=(0, 8))
    _fit(root, WIDTH)

    root.bind('<Return>', lambda e: on_ok())
    root.bind('<Escape>', lambda e: root.destroy())
    root.mainloop()
    return result['choice']


# ══════════════════════════════════════════════════════════════════════════════
#  DIALOG: progress (run a command with live output)
# ══════════════════════════════════════════════════════════════════════════════

def dialog_progress(title, text, command, password_cmd=False):
    """Run a shell command showing live output and a progress bar.

    Args:
        password_cmd: If True, the command uses pkexec and may prompt for password.
    """
    import tkinter as tk

    root = _make_root(title)
    exit_code = {'value': 1}
    f = _fonts()

    _header_frame(root, title, 'build')

    status_var = tk.StringVar(value=text)
    status_lbl = tk.Label(root, textvariable=status_var, font=f['body'],
                          fg=COLORS['ink_muted'], bg=COLORS['canvas'], anchor='w')
    status_lbl.pack(fill='x', padx=PAD, pady=(10, 8))

    # A thin bar on a hairline track
    bar_canvas = tk.Canvas(root, height=4, bg=COLORS['rule'], highlightthickness=0)
    bar_canvas.pack(fill='x', padx=PAD, pady=(0, 12))
    bar_rect = bar_canvas.create_rectangle(0, 0, 0, 4, fill=COLORS['accent'], width=0)

    # Pulse animation state
    pulse_state = {'pos': 0, 'dir': 1, 'active': True}

    def pulse_bar():
        if not pulse_state['active']:
            return
        w = bar_canvas.winfo_width()
        chunk = max(w // 4, 40)
        pulse_state['pos'] += pulse_state['dir'] * 4
        if pulse_state['pos'] + chunk >= w:
            pulse_state['dir'] = -1
        elif pulse_state['pos'] <= 0:
            pulse_state['dir'] = 1
        bar_canvas.coords(bar_rect, pulse_state['pos'], 0,
                          pulse_state['pos'] + chunk, 4)
        root.after(30, pulse_bar)

    root.after(100, pulse_bar)

    # Output console
    console = tk.Text(root, font=f['mono'], bg=COLORS['sunk'], fg=COLORS['ink'],
                      insertbackground=COLORS['ink'], relief='flat', bd=0,
                      padx=12, pady=10, wrap='word', state='disabled',
                      selectbackground=COLORS['accent'], height=16,
                      highlightthickness=1, highlightbackground=COLORS['rule'],
                      highlightcolor=COLORS['rule'])
    console.pack(fill='both', expand=True, padx=PAD)
    console.tag_configure('dim', foreground=COLORS['ink_muted'])
    console.tag_configure('error', foreground=COLORS['bad'])
    console.tag_configure('success', foreground=COLORS['ok'])
    console.tag_configure('warn', foreground=COLORS['warn'])

    btn_frame = _button_row(root)
    close_btn = _styled_button(btn_frame, 'Close', root.destroy, primary=True)
    close_btn.pack(side='right')
    _set_enabled(close_btn, False)

    proc_handle = {'proc': None, 'killed': False}

    def on_cancel():
        if proc_handle['proc'] and proc_handle['proc'].poll() is None:
            proc_handle['killed'] = True
            proc_handle['proc'].kill()

    cancel_btn = _styled_button(btn_frame, 'Cancel', on_cancel)
    cancel_btn.pack(side='right', padx=(0, 8))
    _fit(root, 720, max_h=560)

    def append_text(txt, tag=None):
        console.configure(state='normal')
        if tag:
            console.insert('end', txt, tag)
        else:
            console.insert('end', txt)
        console.see('end')
        console.configure(state='disabled')

    # Elapsed timer
    elapsed = {'seconds': 0}

    def tick_elapsed():
        if not pulse_state['active']:
            return                       # finished: the final status stays
        elapsed['seconds'] += 1
        m, s = divmod(elapsed['seconds'], 60)
        status_var.set(f'{text} ({m}:{s:02d})')
        root.after(1000, tick_elapsed)

    def run_command():
        append_text(f'$ {command}\n', 'dim')
        try:
            proc = subprocess.Popen(
                command, shell=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1)
            proc_handle['proc'] = proc

            for line in proc.stdout:
                clean = re.sub(r'\x1b\[[0-9;]*m', '', line)
                root.after(0, append_text, clean)
            proc.wait()

            code = proc.returncode
            exit_code['value'] = 2 if proc_handle['killed'] else code

            def finish():
                pulse_state['active'] = False
                m, s = divmod(elapsed['seconds'], 60)
                w = bar_canvas.winfo_width()

                if proc_handle['killed']:
                    bar_canvas.itemconfig(bar_rect, fill=COLORS['warn'])
                    bar_canvas.coords(bar_rect, 0, 0, w, 4)
                    status_var.set('Cancelled')
                    status_lbl.configure(fg=COLORS['warn'])
                    append_text('\n--- Cancelled by user ---\n', 'warn')
                elif code == 0:
                    bar_canvas.itemconfig(bar_rect, fill=COLORS['ok'])
                    bar_canvas.coords(bar_rect, 0, 0, w, 4)
                    status_var.set(f'Completed successfully ({m}:{s:02d})')
                    status_lbl.configure(fg=COLORS['ok'])
                    append_text(f'\n--- Completed successfully ({m}:{s:02d}) ---\n',
                                'success')
                    _set_enabled(cancel_btn, False)
                    _set_enabled(close_btn, True)
                    root.after(2000, root.destroy)       # closes itself on success
                    return
                else:
                    bar_canvas.itemconfig(bar_rect, fill=COLORS['bad'])
                    bar_canvas.coords(bar_rect, 0, 0, w, 4)
                    status_var.set(f'Failed (exit code {code})')
                    status_lbl.configure(fg=COLORS['bad'])
                    append_text(f'\n--- Failed with exit code {code} ({m}:{s:02d}) ---\n',
                                'error')

                _set_enabled(cancel_btn, False)
                _set_enabled(close_btn, True)

            root.after(0, finish)
        except Exception as e:
            root.after(0, append_text, f'Error: {e}\n', 'error')
            root.after(0, lambda: _set_enabled(close_btn, True))
            root.after(0, lambda: _set_enabled(cancel_btn, False))
            exit_code['value'] = 1

    root.after(200, tick_elapsed)
    threading.Thread(target=run_command, daemon=True).start()

    root.mainloop()
    return exit_code['value']


# ══════════════════════════════════════════════════════════════════════════════
#  CLI
# ═���════════════════════════════════════════════════════════════════════════════

def main():
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        sys.exit(1)

    cmd = sys.argv[1]

    # Optional 5th arg overrides the auto-close timer (in seconds; 0 = no
    # auto-close, user must click OK). Default behaviour:
    #   info    → 4s autoclose (purely informational steps)
    #   success → 5s autoclose (final "done" message)
    #   error   → 0 (must be read)
    #   warning → 0 (must be read)
    def _autoclose(default_secs):
        if len(sys.argv) >= 5:
            try:
                return max(0, int(sys.argv[4]))
            except ValueError:
                return default_secs
        return default_secs

    if cmd == 'info' and len(sys.argv) >= 4:
        dialog_message(sys.argv[2], sys.argv[3], 'info',
                       autoclose_seconds=_autoclose(4))
    elif cmd == 'error' and len(sys.argv) >= 4:
        dialog_message(sys.argv[2], sys.argv[3], 'error',
                       autoclose_seconds=_autoclose(0))
    elif cmd == 'warning' and len(sys.argv) >= 4:
        dialog_message(sys.argv[2], sys.argv[3], 'warning',
                       autoclose_seconds=_autoclose(0))
    elif cmd == 'success' and len(sys.argv) >= 4:
        dialog_message(sys.argv[2], sys.argv[3], 'success',
                       autoclose_seconds=_autoclose(5))
    elif cmd == 'question' and len(sys.argv) >= 4:
        yes_label = sys.argv[4] if len(sys.argv) > 4 else 'Yes'
        no_label = sys.argv[5] if len(sys.argv) > 5 else 'No'
        ok = dialog_question(sys.argv[2], sys.argv[3], yes_label, no_label)
        sys.exit(0 if ok else 1)
    elif cmd == 'choice' and len(sys.argv) >= 5:
        options = sys.argv[4:]
        chosen = dialog_choice(sys.argv[2], sys.argv[3], options)
        if chosen is None:
            sys.exit(1)
        print(chosen)
        sys.exit(0)
    elif cmd == 'splash' and len(sys.argv) >= 4:
        seconds = int(sys.argv[4]) if len(sys.argv) > 4 else 3
        dialog_splash(sys.argv[2], sys.argv[3], seconds)
    elif cmd == 'progress' and len(sys.argv) >= 5:
        code = dialog_progress(sys.argv[2], sys.argv[3], sys.argv[4])
        sys.exit(code)
    else:
        print(f"Unknown command or missing args: {cmd}", file=sys.stderr)
        print(__doc__, file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
