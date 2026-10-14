/* ═══════════════════════════════════════════════════════════
   FIC Interface Documentation — Navigation + Full-text Search
   ═══════════════════════════════════════════════════════════ */

const pageCache = {};
const searchIndex = [];
let searchReady = false;
let activeResultIdx = -1;

// Pages this copy of the docs does not ship (their extension is not in the
// tree). Filled while the search index is built; links to them are unlinked.
const missingPages = new Set();

function unlinkMissing(root) {
  if (!missingPages.size || !root) return;
  root.querySelectorAll('a[href^="#"]').forEach(a => {
    if (missingPages.has(a.getAttribute('href').slice(1))) a.replaceWith(...a.childNodes);
  });
}

// ── Page loading ──────────────────────────────────────────

let currentPageId = null;

async function showPage(id, opts) {
  const container = document.getElementById('content');
  const replace = opts && opts.replace;

  // Anchors now carry a real href AND an inline handler, so a plain click can
  // reach here twice. Re-rendering the same page would throw away scroll
  // position and re-run the reveal observer for nothing.
  if (id === currentPageId) { closeSearch(); return; }

  if (pageCache[id]) {
    container.innerHTML = pageCache[id];
  } else {
    try {
      const resp = await fetch('pages/' + id + '.html');
      if (!resp.ok) throw new Error('Page not found: ' + id);
      const html = await resp.text();
      pageCache[id] = html;
      container.innerHTML = html;
    } catch (e) {
      container.innerHTML = '<h1>Page not found</h1><p>pages/' + id +
                            '.html is not part of this copy of the documentation.</p>';
    }
  }
  currentPageId = id;
  unlinkMissing(container);

  document.querySelectorAll('.sidebar a[data-page]').forEach(a => {
    const on = a.dataset.page === id;
    a.classList.toggle('active', on);
    if (on) { a.setAttribute('aria-current', 'page'); }
    else    { a.removeAttribute('aria-current'); }
  });

  if (location.hash.slice(1) !== id) {
    if (replace) history.replaceState(null, '', '#' + id);
    else         history.pushState(null, '', '#' + id);
  }

  window.scrollTo(0, 0);
  closeSearch();
  observeReveals(container);

  // Move the reading position for assistive tech, without a visible ring.
  container.focus({ preventScroll: true });
}

// ── Scroll entry ─────────────────────────────────────────
// IntersectionObserver, never a scroll listener. Skipped entirely when the
// reader has asked for reduced motion — the content is simply already there.

let revealObserver = null;

function observeReveals(root) {
  const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  if (reduced) return;

  if (!revealObserver) {
    revealObserver = new IntersectionObserver((entries, obs) => {
      entries.forEach(entry => {
        if (!entry.isIntersecting) return;
        entry.target.classList.add('shown');
        obs.unobserve(entry.target);
      });
    }, { rootMargin: '0px 0px -8% 0px', threshold: 0.01 });
  }

  const blocks = root.querySelectorAll(
    ':scope > h2, :scope > h3, :scope > p, :scope > ul, :scope > ol, ' +
    ':scope > table, :scope > pre, :scope > .admonition, :scope > .card-grid, ' +
    ':scope > svg, :scope > .doc-footer'
  );
  blocks.forEach((el, i) => {
    el.classList.add('reveal');
    el.style.setProperty('--index', String(Math.min(i, 6)));
    revealObserver.observe(el);
  });
}

// ── Search index builder ─────────────────────────────────

function stripTags(html) {
  const div = document.createElement('div');
  div.innerHTML = html;
  return div.textContent || '';
}

async function buildSearchIndex() {
  const links = document.querySelectorAll('.sidebar a[data-page]');
  const fetches = [];

  links.forEach(a => {
    const id = a.dataset.page;
    const title = a.textContent.trim();
    fetches.push(
      fetch('pages/' + id + '.html')
        .then(r => {
          // A page this copy of the docs does not ship: drop it from the
          // sidebar rather than offer a dead link.
          if (r.status === 404) { missingPages.add(id); a.remove(); return null; }
          return r.ok ? r.text() : '';
        })
        .then(html => {
          if (html === null) return;
          pageCache[id] = html;
          const text = stripTags(html);
          searchIndex.push({ id, title, text: text.toLowerCase(), rawText: text });
        })
        .catch(() => {})
    );
  });

  await Promise.all(fetches);
  searchReady = true;
  unlinkMissing(document.getElementById('content'));
}

// ── Search UI ────────────────────────────────────────────

function onSearchInput(query) {
  filterSidebar(query);
  if (!query || query.length < 2) {
    closeSearch();
    return;
  }

  const q = query.toLowerCase();
  const results = [];

  if (searchReady) {
    for (const entry of searchIndex) {
      const idx = entry.text.indexOf(q);
      if (idx === -1 && !entry.title.toLowerCase().includes(q)) continue;

      let snippet = '';
      if (idx !== -1) {
        const start = Math.max(0, idx - 60);
        const end = Math.min(entry.rawText.length, idx + q.length + 100);
        const raw = (start > 0 ? '\u2026' : '') +
                    entry.rawText.slice(start, end) +
                    (end < entry.rawText.length ? '\u2026' : '');
        snippet = highlightSnippet(raw, query);
      } else {
        snippet = entry.rawText.slice(0, 120) + '\u2026';
      }

      const titleMatch = entry.title.toLowerCase().includes(q);
      results.push({ ...entry, snippet, titleMatch, matchIdx: idx === -1 ? 99999 : idx });
    }
  } else {
    document.querySelectorAll('.sidebar a[data-page]').forEach(a => {
      const title = a.textContent.trim();
      if (title.toLowerCase().includes(q)) {
        results.push({ id: a.dataset.page, title, snippet: '', titleMatch: true, matchIdx: 0 });
      }
    });
  }

  results.sort((a, b) => {
    if (a.titleMatch !== b.titleMatch) return a.titleMatch ? -1 : 1;
    return a.matchIdx - b.matchIdx;
  });

  renderResults(results.slice(0, 15), query);
}

function highlightSnippet(text, query) {
  const escaped = query.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  return text.replace(new RegExp('(' + escaped + ')', 'gi'), '<mark>$1</mark>');
}

function renderResults(results, query) {
  const box = document.getElementById('search-results');
  activeResultIdx = -1;

  if (results.length === 0) {
    box.innerHTML = '<div class="search-no-results">No results for "' +
                    query.replace(/</g, '&lt;') + '"</div>';
    box.classList.add('open');
    setSearchExpanded(true);
    announce('No results');
    return;
  }

  box.innerHTML = results.map((r, i) =>
    '<div class="search-result" id="search-opt-' + i + '" role="option" ' +
    'aria-selected="false" data-page="' + r.id + '" data-idx="' + i + '" ' +
    'onmousedown="showPage(\'' + r.id + '\')" ' +
    'onmouseenter="setActiveResult(' + i + ')">' +
    '<div class="search-result-title">' + highlightSnippet(r.title, query) + '</div>' +
    '<div class="search-result-snippet">' + r.snippet + '</div>' +
    '</div>'
  ).join('');

  box.classList.add('open');
  setSearchExpanded(true);
  announce(results.length + (results.length === 1 ? ' result' : ' results'));
}

// ── Assistive-tech state for the combobox ────────────────

function setSearchExpanded(open) {
  const input = document.getElementById('search-input');
  if (input) input.setAttribute('aria-expanded', open ? 'true' : 'false');
}

function announce(message) {
  const el = document.getElementById('search-status');
  if (el) el.textContent = message;
}

function setActiveResult(idx) {
  const items = document.querySelectorAll('.search-result');
  const input = document.getElementById('search-input');
  items.forEach(el => {
    el.classList.remove('active');
    el.setAttribute('aria-selected', 'false');
  });
  if (idx >= 0 && idx < items.length) {
    items[idx].classList.add('active');
    items[idx].setAttribute('aria-selected', 'true');
    items[idx].scrollIntoView({ block: 'nearest' });
    if (input) input.setAttribute('aria-activedescendant', items[idx].id);
  } else if (input) {
    input.removeAttribute('aria-activedescendant');
  }
  activeResultIdx = idx;
}

function closeSearch() {
  const box = document.getElementById('search-results');
  if (box) {
    box.classList.remove('open');
    box.innerHTML = '';
    setSearchExpanded(false);
    const input = document.getElementById('search-input');
    if (input) input.removeAttribute('aria-activedescendant');
  }
  activeResultIdx = -1;
}

function onSearchFocus() {
  const input = document.getElementById('search-input');
  if (input.value.length >= 2) {
    onSearchInput(input.value);
  }
}

function onSearchKey(e) {
  const items = document.querySelectorAll('.search-result');
  if (!items.length) return;

  if (e.key === 'ArrowDown') {
    e.preventDefault();
    setActiveResult(Math.min(activeResultIdx + 1, items.length - 1));
  } else if (e.key === 'ArrowUp') {
    e.preventDefault();
    setActiveResult(Math.max(activeResultIdx - 1, 0));
  } else if (e.key === 'Enter') {
    e.preventDefault();
    if (activeResultIdx >= 0 && activeResultIdx < items.length) {
      showPage(items[activeResultIdx].dataset.page);
    }
  } else if (e.key === 'Escape') {
    closeSearch();
    document.getElementById('search-input').blur();
  }
}

// ── Sidebar filtering (still works alongside search) ─────

function filterSidebar(query) {
  const q = query.toLowerCase();
  document.querySelectorAll('.sidebar a[data-page]').forEach(a => {
    const text = a.textContent.toLowerCase();
    a.style.display = (!q || text.includes(q)) ? '' : 'none';
  });
  document.querySelectorAll('.sidebar .section-title').forEach(t => {
    if (!q) { t.style.display = ''; return; }
    let next = t.nextElementSibling;
    let hasVisible = false;
    while (next && !next.classList.contains('section-title')) {
      if (next.style.display !== 'none') hasVisible = true;
      next = next.nextElementSibling;
    }
    t.style.display = hasVisible ? '' : 'none';
  });
}

// ── Close search on outside click ────────────────────────

document.addEventListener('click', e => {
  const box = document.getElementById('search-box');
  if (box && !box.contains(e.target)) {
    closeSearch();
  }
});

// ── Init ─────────────────────────────────────────────────

// Cmd/Ctrl/Shift/middle-click on an in-app link must open a tab or window,
// not navigate in place. Capture phase, so the anchor's own inline handler
// never runs for a modified click; the browser's default then does the
// right thing with the href.
document.addEventListener('click', e => {
  if (!(e.metaKey || e.ctrlKey || e.shiftKey || e.altKey || e.button !== 0)) return;
  const link = e.target.closest && e.target.closest('a[href^="#"]');
  if (link) e.stopImmediatePropagation();
}, true);

document.addEventListener('auxclick', e => {
  const link = e.target.closest && e.target.closest('a[href^="#"]');
  if (link && e.button === 1) e.stopImmediatePropagation();
}, true);

window.addEventListener('popstate', () => {
  showPage(location.hash.slice(1) || 'home', { replace: true });
});

document.addEventListener('DOMContentLoaded', () => {
  showPage(location.hash.slice(1) || 'home', { replace: true });
  buildSearchIndex();
});
