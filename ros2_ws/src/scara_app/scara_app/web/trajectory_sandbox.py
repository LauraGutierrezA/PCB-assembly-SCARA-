"""
trajectory_sandbox.py - Sandbox de bloques de la pestana Trayectoria (JavaScript + CSS).

Todo el arrastre ocurre en el NAVEGADOR (iPad o PC) con pointer events (dedo
y mouse por igual); el servidor solo pide el programa al presionar "Iniciar"
(window.scaraTraj.get()) y marca el bloque activo (setActive / mark).

Uso en el sandbox:
  - Tocar un bloque de la barra de arriba  -> se agrega al FINAL de la cadena
    y queda conectado solo.
  - Arrastrar un bloque de la barra        -> se suelta donde quieras (sin conectar).
  - Arrastrar desde el punto de SALIDA (derecha) hasta otro bloque -> flecha.
  - Tocar la (x) de una flecha             -> se borra la flecha.
  - Arrastrar la cabecera de un bloque     -> moverlo.  (x) de la cabecera -> borrarlo.
  - El numero #n de cada bloque es su orden de ejecucion; INICIO = primer bloque.
El diagrama se guarda solo en el navegador (localStorage) y vuelve al
recargar la pagina. Cada pantalla (iPad, PC) tiene su propio diagrama.

Colores: tokens de gui/theme.py (no se definen colores nuevos).
"""
import json

from scara_app.gui import theme as t
from scara_app.trajectory import DEFAULT_PARAMS, KIND_LABEL

from . import theme as wt

KIND_COLOR = {'goto': t.PRIMARY, 'pick': t.ACCENT, 'place': t.CONNECTED, 'rotate': t.TEXT}

# Ejemplo inicial (el de la imagen): GO TO -> PICK -> GO TO -> PLACE -> GO TO -> GO TO
SEED = {
    'next': 7,
    'blocks': [
        {'id': 'b1', 'kind': 'goto', 'x': 30, 'y': 30,
         'params': {'x': 160, 'y': 80, 'z': 30, 'codo': 1, 'safe': True}},
        {'id': 'b2', 'kind': 'pick', 'x': 300, 'y': 60, 'params': {'wait': 0.5}},
        {'id': 'b3', 'kind': 'goto', 'x': 530, 'y': 30,
         'params': {'x': 160, 'y': -80, 'z': 30, 'codo': 1, 'safe': True}},
        {'id': 'b4', 'kind': 'place', 'x': 800, 'y': 60, 'params': {'wait': 0.8}},
        {'id': 'b5', 'kind': 'goto', 'x': 800, 'y': 330,
         'params': {'x': 180, 'y': 0, 'z': 50, 'codo': 1, 'safe': True}},
        {'id': 'b6', 'kind': 'goto', 'x': 530, 'y': 330,
         'params': {'x': 180, 'y': 0, 'z': 72.6, 'codo': 1, 'safe': False}},
    ],
    'links': [{'from': 'b1', 'to': 'b2'}, {'from': 'b2', 'to': 'b3'},
              {'from': 'b3', 'to': 'b4'}, {'from': 'b4', 'to': 'b5'},
              {'from': 'b5', 'to': 'b6'}],
}

_JS = r"""
<script>
window.scaraTraj = (function () {
  const KEY = 'scara.traj.v1';
  const DEF = __DEFAULTS__;
  const LABEL = __LABELS__;
  const SEED = __SEED__;
  const W = {goto: 236, pick: 188, place: 188, rotate: 200};
  const PORT_Y = 23;                 // altura de los puntos de entrada/salida
  const WORLD_W = 2400, WORLD_H = 1600;

  let st = null, root = null, world = null, svg = null, dels = null, locked = false;
  let tmpPath = null;

  const clone = (o) => JSON.parse(JSON.stringify(o));
  const byId = (id) => st.blocks.find(b => b.id === id);
  const emit = (name, args) => { try { emitEvent(name, args); } catch (e) {} };

  function load() {
    try {
      const s = localStorage.getItem(KEY);
      if (s) { const v = JSON.parse(s); if (v && Array.isArray(v.blocks)) st = v; }
    } catch (e) {}
    if (!st) st = clone(SEED);
  }
  function save() { try { localStorage.setItem(KEY, JSON.stringify(st)); } catch (e) {} }

  // ---------------- Orden de ejecucion (igual que trajectory.order_blocks) ----------------
  function chain() {
    const nxt = {}, hasIn = new Set();
    st.links.forEach(l => { nxt[l.from] = l.to; hasIn.add(l.to); });
    let heads = st.blocks.filter(b => nxt[b.id] && !hasIn.has(b.id));
    if (!st.links.length && st.blocks.length === 1) heads = [st.blocks[0]];
    if (heads.length !== 1) return [];
    const out = [], seen = new Set();
    let cur = heads[0].id;
    while (cur && !seen.has(cur)) { seen.add(cur); out.push(cur); cur = nxt[cur]; }
    return out;
  }
  function tail() { const c = chain(); return c.length ? c[c.length - 1] : null; }

  // ---------------- Dibujo ----------------
  function fmt(v) { return (v === null || v === undefined) ? '' : String(v); }
  function row(label, key, val, unit) {
    return `<label class="tb-row"><span class="tb-k">${label}</span>` +
      `<input class="tb-in-num" data-k="${key}" inputmode="decimal" enterkeyhint="done" ` +
      `autocomplete="off" value="${fmt(val)}"><span class="tb-u">${unit}</span></label>`;
  }
  function seg(key, val, opts) {
    return `<div class="tb-seg" data-k="${key}">` + opts.map(([txt, v]) =>
      `<button type="button" data-v="${v}" class="${String(val) === String(v) ? 'on' : ''}">${txt}</button>`
    ).join('') + `</div>`;
  }
  function bodyHtml(b) {
    const p = b.params;
    if (b.kind === 'goto') {
      const cell = (k) => `<label class="tb-cell"><span class="tb-k">${k.toUpperCase()} mm</span>` +
        `<input class="tb-in-num" data-k="${k}" inputmode="decimal" enterkeyhint="done" ` +
        `autocomplete="off" value="${fmt(p[k])}"></label>`;
      return `<div class="tb-xyz">${cell('x')}${cell('y')}${cell('z')}</div>` +
        `<div class="tb-line"><span class="tb-k">Codo</span>` +
        seg('codo', p.codo, [['+1', 1], ['−1', -1]]) + `</div>` +
        `<button type="button" class="tb-check ${p.safe ? 'on' : ''}" data-k="safe">` +
        `<i></i>Pasar por Z segura</button>`;
    }
    if (b.kind === 'rotate') {
      return `<div class="tb-line"><span class="tb-k">Giro</span>` +
        seg('dir', p.dir, [['+90°', 90], ['−90°', -90]]) + `</div>` +
        row('Espera', 'wait', p.wait, 's');
    }
    const what = b.kind === 'pick' ? 'Bomba ON' : 'Bomba OFF';
    return `<div class="tb-note">${what}</div>` + row('Espera', 'wait', p.wait, 's');
  }
  function blockEl(b) {
    const el = document.createElement('div');
    el.className = `tb tb-${b.kind}`;
    el.dataset.id = b.id;
    el.style.left = b.x + 'px'; el.style.top = b.y + 'px'; el.style.width = W[b.kind] + 'px';
    el.innerHTML =
      `<div class="tb-port tb-port-in" title="entrada"></div>` +
      `<div class="tb-head"><span class="tb-badge">INICIO</span>` +
      `<span class="tb-title">${LABEL[b.kind]}</span><span class="tb-num"></span>` +
      `<button type="button" class="tb-del" title="borrar bloque">×</button></div>` +
      `<div class="tb-body">${bodyHtml(b)}</div>` +
      `<div class="tb-port tb-port-out" title="arrastra hasta el siguiente bloque"></div>`;
    return el;
  }
  function render() {
    if (!world) return;
    world.querySelectorAll('.tb').forEach(e => e.remove());
    st.blocks.forEach(b => world.appendChild(blockEl(b)));
    refresh();
  }
  function portPos(b, side) {
    return side === 'out' ? [b.x + W[b.kind], b.y + PORT_Y] : [b.x, b.y + PORT_Y];
  }
  function curve(x1, y1, x2, y2) {
    const dx = Math.max(40, Math.abs(x2 - x1) / 2);
    return [[x1, y1], [x1 + dx, y1], [x2 - dx, y2], [x2, y2]];
  }
  const d = (c) => `M${c[0][0]},${c[0][1]} C${c[1][0]},${c[1][1]} ${c[2][0]},${c[2][1]} ${c[3][0]},${c[3][1]}`;
  function refresh() {
    if (!svg) return;
    // flechas
    svg.querySelectorAll('.tl').forEach(e => e.remove());
    dels.innerHTML = '';
    st.links.forEach((l, i) => {
      const a = byId(l.from), b = byId(l.to);
      if (!a || !b) return;
      const [x1, y1] = portPos(a, 'out'), [x2, y2] = portPos(b, 'in');
      const c = curve(x1, y1, x2 - 6, y2);
      const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
      path.setAttribute('d', d(c)); path.setAttribute('class', 'tl');
      path.setAttribute('marker-end', 'url(#tl-arrow)');
      svg.appendChild(path);
      const mx = (c[0][0] + 3 * c[1][0] + 3 * c[2][0] + c[3][0]) / 8;
      const my = (c[0][1] + 3 * c[1][1] + 3 * c[2][1] + c[3][1]) / 8;
      const x = document.createElement('button');
      x.type = 'button'; x.className = 'tl-del'; x.textContent = '×'; x.dataset.link = i;
      x.title = 'borrar flecha';
      x.style.left = mx + 'px'; x.style.top = my + 'px';
      dels.appendChild(x);
    });
    // orden de ejecucion
    const order = chain();
    world.querySelectorAll('.tb').forEach(el => {
      const n = order.indexOf(el.dataset.id);
      el.querySelector('.tb-num').textContent = n >= 0 ? '#' + (n + 1) : '';
      el.classList.toggle('start', n === 0);
      el.classList.toggle('loose', n < 0);
    });
    root.classList.toggle('empty', st.blocks.length === 0);
  }
  function changed() { refresh(); save(); }

  // ---------------- Edicion ----------------
  function addBlock(kind, x, y, connect) {
    const id = 'b' + (st.next++);
    const t = tail();
    if (x === undefined) {
      const last = t ? byId(t) : st.blocks[st.blocks.length - 1];
      if (last) {
        x = last.x + W[last.kind] + 50; y = last.y;
        if (x + W[kind] > Math.max(root.clientWidth, 600) + root.scrollLeft - 10) {
          x = 30; y = Math.max(...st.blocks.map(b => b.y)) + 290;
        }
      } else { x = 30; y = 30; }
    }
    st.blocks.push({id, kind, x: Math.max(0, x), y: Math.max(0, y), params: clone(DEF[kind])});
    if (connect && t) st.links.push({from: t, to: id});
    render(); save();
    const el = world.querySelector(`.tb[data-id="${id}"]`);
    if (el) el.scrollIntoView({block: 'nearest', inline: 'nearest', behavior: 'smooth'});
    select(id);
  }
  function removeBlock(id) {
    st.blocks = st.blocks.filter(b => b.id !== id);
    st.links = st.links.filter(l => l.from !== id && l.to !== id);
    render(); save();
  }
  function connect(a, b) {
    if (a === b) return;
    st.links = st.links.filter(l => l.from !== a && l.to !== b && !(l.from === b && l.to === a));
    st.links.push({from: a, to: b});
    changed();
  }
  function select(id) {
    const b = byId(id);
    if (b) emit('traj_select', {id: b.id, kind: b.kind, params: b.params});
  }
  function setParam(id, key, value) {
    const b = byId(id);
    if (!b) return;
    b.params[key] = value;
    save(); select(id);
  }

  // ---------------- Coordenadas ----------------
  function toWorld(cx, cy) {
    const r = world.getBoundingClientRect();
    return [cx - r.left, cy - r.top];
  }
  function overSandbox(cx, cy) {
    const r = root.getBoundingClientRect();
    return cx >= r.left && cx <= r.right && cy >= r.top && cy <= r.bottom;
  }

  // ---------------- Arrastres (dedo y mouse) ----------------
  let drag = null;
  function onDown(e) {
    if (e.button !== undefined && e.button > 0) return;
    const item = e.target.closest('.tp-item');
    if (item) {
      if (locked) return;
      e.preventDefault();
      const ghost = item.cloneNode(true);
      ghost.classList.add('tp-ghost');
      document.body.appendChild(ghost);
      drag = {type: 'palette', kind: item.dataset.kind, ghost, sx: e.clientX, sy: e.clientY,
              moved: false, pid: e.pointerId};
      moveGhost(e);
      return;
    }
    if (!root || !root.contains(e.target)) return;
    const blk = e.target.closest('.tb');
    if (blk && !e.target.closest('input, .tb-seg, .tb-check, .tb-del')) select(blk.dataset.id);
    if (locked) return;
    if (e.target.closest('.tb-port-out')) {
      e.preventDefault();
      const b = byId(blk.dataset.id);
      tmpPath = document.createElementNS('http://www.w3.org/2000/svg', 'path');
      tmpPath.setAttribute('class', 'tl tl-tmp');
      svg.appendChild(tmpPath);
      drag = {type: 'link', from: b.id, pid: e.pointerId};
      onMove(e);
      return;
    }
    const head = e.target.closest('.tb-head');
    if (head && !e.target.closest('.tb-del')) {
      e.preventDefault();
      const b = byId(blk.dataset.id);
      const [wx, wy] = toWorld(e.clientX, e.clientY);
      drag = {type: 'block', id: b.id, el: blk, dx: wx - b.x, dy: wy - b.y, pid: e.pointerId};
      blk.classList.add('dragging');
    }
  }
  function moveGhost(e) {
    drag.ghost.style.left = (e.clientX - 40) + 'px';
    drag.ghost.style.top = (e.clientY - 20) + 'px';
  }
  function onMove(e) {
    if (!drag || e.pointerId !== drag.pid) return;
    e.preventDefault();
    if (drag.type === 'palette') {
      if (Math.hypot(e.clientX - drag.sx, e.clientY - drag.sy) > 8) drag.moved = true;
      moveGhost(e);
      drag.ghost.classList.toggle('over', overSandbox(e.clientX, e.clientY));
    } else if (drag.type === 'block') {
      const [wx, wy] = toWorld(e.clientX, e.clientY);
      const b = byId(drag.id);
      b.x = Math.max(0, Math.min(WORLD_W - W[b.kind], Math.round(wx - drag.dx)));
      b.y = Math.max(0, Math.min(WORLD_H - 80, Math.round(wy - drag.dy)));
      drag.el.style.left = b.x + 'px'; drag.el.style.top = b.y + 'px';
      refresh();
    } else if (drag.type === 'link') {
      const a = byId(drag.from);
      const [x1, y1] = portPos(a, 'out');
      const [x2, y2] = toWorld(e.clientX, e.clientY);
      tmpPath.setAttribute('d', d(curve(x1, y1, x2, y2)));
      world.querySelectorAll('.tb.target').forEach(el => el.classList.remove('target'));
      const over = document.elementFromPoint(e.clientX, e.clientY);
      const tb = over && over.closest('.tb');
      if (tb && tb.dataset.id !== drag.from) tb.classList.add('target');
    }
  }
  function onUp(e) {
    if (!drag || e.pointerId !== drag.pid) return;
    const dr = drag; drag = null;
    if (dr.type === 'palette') {
      dr.ghost.remove();
      if (!dr.moved) addBlock(dr.kind, undefined, undefined, true);          // toque
      else if (overSandbox(e.clientX, e.clientY)) {
        const [wx, wy] = toWorld(e.clientX, e.clientY);
        addBlock(dr.kind, Math.round(wx - 40), Math.round(wy - 20), false);
      }
    } else if (dr.type === 'block') {
      dr.el.classList.remove('dragging');
      save();
    } else if (dr.type === 'link') {
      if (tmpPath) { tmpPath.remove(); tmpPath = null; }
      world.querySelectorAll('.tb.target').forEach(el => el.classList.remove('target'));
      const over = document.elementFromPoint(e.clientX, e.clientY);
      const tb = over && over.closest('.tb');
      if (tb && root.contains(tb)) connect(dr.from, tb.dataset.id);
    }
  }
  function onClick(e) {
    if (!root || !root.contains(e.target) || locked) return;
    const del = e.target.closest('.tb-del');
    if (del) { removeBlock(del.closest('.tb').dataset.id); return; }
    const ldel = e.target.closest('.tl-del');
    if (ldel) { st.links.splice(+ldel.dataset.link, 1); changed(); return; }
    const sb = e.target.closest('.tb-seg button');
    if (sb) {
      const segEl = sb.parentElement, id = sb.closest('.tb').dataset.id;
      segEl.querySelectorAll('button').forEach(x => x.classList.toggle('on', x === sb));
      setParam(id, segEl.dataset.k, Number(sb.dataset.v));
      return;
    }
    const chk = e.target.closest('.tb-check');
    if (chk) {
      const id = chk.closest('.tb').dataset.id;
      const on = !chk.classList.contains('on');
      chk.classList.toggle('on', on);
      setParam(id, chk.dataset.k, on);
    }
  }
  function onInput(e) {
    const inp = e.target.closest('.tb-in-num');
    if (!inp || !root.contains(inp)) return;
    const raw = inp.value.replace(',', '.').trim();
    const v = parseFloat(raw);
    inp.classList.toggle('bad', !(raw !== '' && isFinite(v) && /^[-+]?\d*\.?\d*$/.test(raw)));
    setParam(inp.closest('.tb').dataset.id, inp.dataset.k, (raw !== '' && isFinite(v)) ? v : raw);
  }

  // ---------------- Montaje ----------------
  function mount(el) {
    root = el;
    root.dataset.mounted = '1';
    root.innerHTML =
      `<div class="tw" style="width:${WORLD_W}px;height:${WORLD_H}px">` +
      `<svg class="tl-svg" width="${WORLD_W}" height="${WORLD_H}"><defs>` +
      `<marker id="tl-arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" ` +
      `markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" class="tl-head"/>` +
      `</marker></defs></svg><div class="tl-dels"></div></div>` +
      `<div class="tw-empty">Toca o arrastra un bloque de la barra de arriba</div>`;
    world = root.querySelector('.tw');
    svg = root.querySelector('.tl-svg');
    dels = root.querySelector('.tl-dels');
    root.addEventListener('input', onInput);
    root.addEventListener('keydown', e => {
      if (e.key === 'Enter' && e.target.closest('.tb-in-num')) e.target.blur();
    });
    render();
    setLocked(locked);
  }
  function tryMount() {
    const el = document.getElementById('traj-sb');
    if (el && el.dataset.mounted !== '1') mount(el);
    // Las pestanas ocultas quedan fuera del documento: al volver, re-aplicar el bloqueo
    document.querySelectorAll('.traj-palette').forEach(p => {
      if (p.classList.contains('locked') !== locked) p.classList.toggle('locked', locked);
    });
  }

  // ---------------- API para el servidor ----------------
  function get() {
    return {blocks: st.blocks.map(b => ({id: b.id, kind: b.kind, params: b.params})),
            links: st.links.map(l => ({from: l.from, to: l.to}))};
  }
  function setLocked(on) {
    locked = !!on;
    if (root) {
      root.classList.toggle('locked', locked);
      root.querySelectorAll('input').forEach(i => { i.readOnly = locked; });
    }
    document.querySelectorAll('.traj-palette').forEach(p => p.classList.toggle('locked', locked));
  }
  function setActive(id) {
    if (!world) return;
    world.querySelectorAll('.tb').forEach(el => {
      const on = el.dataset.id === id;
      if (on && !el.classList.contains('active')) {
        el.scrollIntoView({block: 'nearest', inline: 'nearest', behavior: 'smooth'});
      }
      el.classList.toggle('active', on);
      if (on) el.classList.remove('done');
    });
  }
  function mark(id, cls) {
    const el = world && world.querySelector(`.tb[data-id="${id}"]`);
    if (el) { el.classList.add(cls); if (cls === 'error') el.scrollIntoView({block: 'nearest', inline: 'nearest'}); }
  }
  function clearMarks() {
    if (world) world.querySelectorAll('.tb').forEach(el => el.classList.remove('active', 'done', 'error'));
  }
  function clearAll() {
    if (locked) return;
    st = {next: st.next, blocks: [], links: []};
    render(); save();
  }
  function example() {
    if (locked) return;
    st = clone(SEED);
    render(); save();
  }

  load();
  document.addEventListener('pointerdown', onDown);
  document.addEventListener('pointermove', onMove, {passive: false});
  document.addEventListener('pointerup', onUp);
  document.addEventListener('pointercancel', onUp);
  document.addEventListener('click', onClick);
  new MutationObserver(tryMount).observe(document.documentElement, {childList: true, subtree: true});
  document.addEventListener('DOMContentLoaded', tryMount);

  return {get, setLocked, setActive, mark, clearMarks, clearAll, example, tryMount};
})();
</script>
"""

_CSS = """
<style>
/* ---------------- Trayectoria ---------------- */
.traj-toolbar {
  display: flex; align-items: flex-end; gap: 14px; flex-wrap: wrap;
  padding-bottom: 14px; border-bottom: 1px solid %DIVIDER%; margin-bottom: 14px;
}
.traj-palette { display: flex; flex-direction: column; gap: 6px; }
.traj-palette-row { display: flex; gap: 10px; flex-wrap: wrap; }
.traj-palette.locked { opacity: .4; pointer-events: none; }
.tp-item {
  display: flex; align-items: center; gap: 10px;
  min-height: 52px; padding: 0 16px 0 0; overflow: hidden;
  background: %BG_PAGE%; border: 1px solid %BORDER_CARD%; border-radius: 6px;
  font-family: %MONO%; font-size: 14px; font-weight: 700; color: %TEXT_STRONG%;
  letter-spacing: .5px; cursor: grab;
  user-select: none; -webkit-user-select: none; -webkit-touch-callout: none; touch-action: none;
}
.tp-item::before { content: ''; align-self: stretch; width: 6px; background: var(--k); }
.tp-item .tp-plus { color: %TEXT_MUTED%; font-weight: 400; margin-left: 2px; }
.tp-ghost {
  position: fixed; z-index: 9999; pointer-events: none; opacity: .85;
  box-shadow: 0 8px 24px rgba(0,0,0,.5); transform: rotate(-2deg);
}
.tp-ghost.over { border-color: %ACCENT%; }
.traj-field .num-param { width: 104px; }

.traj-split { display: flex; gap: 18px; flex: 1 1 auto; min-height: 0; }
.traj-left { flex: 1 1 auto; min-width: 0; display: flex; flex-direction: column; min-height: 0; gap: 10px; }
.traj-right { flex: 0 0 380px; display: flex; flex-direction: column; min-height: 0; gap: 10px; }
.traj-left .log-wrap { flex: 0 0 120px; min-height: 0; }
.traj-sb-head { display: flex; align-items: center; justify-content: space-between; gap: 10px; }
.traj-sb-head .hint { font-size: 13px; }

/* Sandbox */
.traj-sb {
  position: relative; flex: 1 1 auto; min-height: 220px;
  overflow: auto; -webkit-overflow-scrolling: touch;
  background-color: %BG_PAGE%;
  background-image: radial-gradient(%DIVIDER% 1.3px, transparent 1.3px);
  background-size: 22px 22px;
  border: 1px solid %DIVIDER%; border-radius: 6px;
}
.tw { position: relative; }
.tw-empty {
  position: absolute; inset: 0; display: none; align-items: center; justify-content: center;
  color: %TEXT_MUTED%; font-size: 15px; pointer-events: none;
}
.traj-sb.empty .tw-empty { display: flex; }
.tl-svg { position: absolute; left: 0; top: 0; pointer-events: none; overflow: visible; }
.tl { fill: none; stroke: %TEXT_MUTED%; stroke-width: 2.5; }
.tl-tmp { stroke: %ACCENT%; stroke-dasharray: 6 5; }
.tl-head { fill: %TEXT_MUTED%; }
.tl-dels { position: absolute; left: 0; top: 0; }
.tl-del {
  position: absolute; width: 26px; height: 26px; margin: -13px 0 0 -13px; padding: 0;
  border-radius: 50%; border: 1px solid %BORDER_CARD%; background: %BG_CARD%;
  color: %TEXT_MUTED%; font-size: 16px; line-height: 22px; cursor: pointer; z-index: 3;
}
.tl-del:hover { color: %DANGER%; border-color: %DANGER%; }
.traj-sb.locked .tl-del, .traj-sb.locked .tb-del, .traj-sb.locked .tb-port-out { display: none; }

/* Bloques */
.tb {
  position: absolute; z-index: 2;
  background: %BG_CARD%; border: 1px solid %BORDER_CARD%; border-radius: 8px;
  box-shadow: 0 2px 10px rgba(0,0,0,.35);
  border-left: 6px solid var(--k);
}
.tb-goto { --k: %K_GOTO%; } .tb-pick { --k: %K_PICK%; }
.tb-place { --k: %K_PLACE%; } .tb-rotate { --k: %K_ROTATE%; }
.tb.loose { opacity: .55; }
.tb.dragging { z-index: 5; box-shadow: 0 10px 28px rgba(0,0,0,.55); }
.tb.target { border-color: %ACCENT%; }
.tb.active { border-color: %ACCENT%; box-shadow: 0 0 0 2px %ACCENT%, 0 0 22px rgba(63,224,232,.35); opacity: 1; }
.tb.done { opacity: .6; }
.tb.error { border-color: %DANGER%; box-shadow: 0 0 0 2px %DANGER%; }
.tb-head {
  display: flex; align-items: center; gap: 8px; height: 46px; padding: 0 6px 0 12px;
  border-bottom: 1px solid %DIVIDER%; cursor: grab;
  user-select: none; -webkit-user-select: none; touch-action: none;
}
.traj-sb.locked .tb-head { cursor: default; }
.tb-title { white-space: nowrap; font-family: %MONO%; font-size: 14px; font-weight: 700; color: %TEXT_STRONG%; letter-spacing: .5px; }
.tb-num { font-family: %MONO%; font-size: 13px; color: %ACCENT%; margin-left: auto; }
.tb-badge {
  display: none; position: absolute; left: -6px; top: -22px; font-size: 10px; font-weight: 700; letter-spacing: 1px;
  color: %BG_PAGE%; background: %ACCENT%; border-radius: 3px; padding: 2px 5px;
}
.tb.start .tb-badge { display: inline-block; }
.tb-del {
  width: 36px; height: 36px; border: none; background: transparent; color: %TEXT_MUTED%;
  font-size: 22px; line-height: 1; cursor: pointer; border-radius: 4px;
}
.tb-del:hover { color: %DANGER%; }
.tb-body { padding: 10px 12px 12px; display: flex; flex-direction: column; gap: 8px; }
.tb-row, .tb-line { display: flex; align-items: center; gap: 8px; }
.tb-xyz { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; }
.tb-cell { display: flex; flex-direction: column; gap: 3px; min-width: 0; }
.tb-cell .tb-k { width: auto; font-size: 11px; }
.tb-cell .tb-in-num { width: 100%; padding: 0 6px; }
.tb-k { width: 52px; font-size: 12px; font-weight: 700; letter-spacing: 1px; color: %TEXT_MUTED%; text-transform: uppercase; }
.tb-u { width: 22px; font-family: %MONO%; font-size: 13px; color: %TEXT_MUTED%; }
.tb-in-num {
  flex: 1 1 auto; min-width: 0; height: 40px; padding: 0 10px;
  background: %BG_PAGE%; color: %TEXT%; border: 1px solid %BORDER_CARD%; border-radius: 4px;
  font-family: %MONO%; font-size: 16px; text-align: right; outline: none;
}
.tb-in-num:focus { border-color: %ACCENT%; }
.tb-in-num.bad { border-color: %DANGER%; color: %DANGER%; }
.traj-sb.locked .tb-in-num { border-color: %DIVIDER%; color: %TEXT_MUTED%; }
.tb-seg { display: inline-flex; }
.tb-seg button {
  min-width: 56px; height: 40px; padding: 0 10px; cursor: pointer;
  background: transparent; color: %TEXT_MUTED%; border: 1px solid %BORDER_CARD%;
  font-family: %MONO%; font-size: 15px; font-weight: 700;
}
.tb-seg button:first-child { border-radius: 4px 0 0 4px; border-right: none; }
.tb-seg button:last-child { border-radius: 0 4px 4px 0; }
.tb-seg button.on { background: %DIVIDER%; color: %ACCENT%; border-color: %ACCENT%; }
.tb-seg button.on:first-child { border-right: 1px solid %ACCENT%; }
.tb-seg button.on + button { border-left: none; }
.tb-check {
  display: flex; align-items: center; gap: 10px; height: 36px; padding: 0 4px;
  background: transparent; border: none; color: %TEXT%; font-size: 14px; cursor: pointer;
}
.tb-check i {
  width: 22px; height: 22px; border: 1px solid %BORDER_CARD%; border-radius: 4px; position: relative;
}
.tb-check.on i { background: %ACCENT%; border-color: %ACCENT%; }
.tb-check.on i::after {
  content: ''; position: absolute; left: 7px; top: 3px; width: 6px; height: 11px;
  border: solid %BG_PAGE%; border-width: 0 2px 2px 0; transform: rotate(45deg);
}
.tb-note { font-family: %MONO%; font-size: 13px; color: %TEXT_MUTED%; }
.traj-sb.locked .tb-seg, .traj-sb.locked .tb-check { pointer-events: none; opacity: .7; }

/* Puertos */
.tb-port {
  position: absolute; top: 12px; width: 22px; height: 22px; border-radius: 50%;
  background: %BG_PAGE%; border: 2px solid var(--k); touch-action: none;
}
.tb-port-in { left: -15px; }
.tb-port-out { right: -12px; cursor: crosshair; }
.tb-port-out::after {
  content: ''; position: absolute; inset: -12px; border-radius: 50%;   /* zona de toque mas grande */
}
.tb-port-out:hover { background: var(--k); }

/* Estado del ciclo */
.traj-state {
  display: flex; align-items: center; gap: 12px; padding: 10px 14px;
  background: %BG_PAGE%; border: 1px solid %DIVIDER%; border-radius: 6px;
}
.traj-chip {
  font-family: %MONO%; font-size: 13px; font-weight: 700; letter-spacing: 1px;
  padding: 4px 8px; border-radius: 4px; border: 1px solid currentColor; white-space: nowrap;
}
.traj-progress { font-family: %MONO%; font-size: 13.5px; color: %TEXT%; line-height: 1.45; min-width: 0; }
.traj-progress .muted { color: %TEXT_MUTED%; }
.traj-right .twin { min-height: 220px; }
.traj-right .deadman { min-height: 64px; flex: none; }

@media (max-width: 900px) {
  .tab-body.traj { overflow-y: auto; -webkit-overflow-scrolling: touch; }
  .traj-split { flex-direction: column; flex: 0 0 auto; }
  .traj-left { flex: 0 0 auto; }
  .traj-sb { flex: 0 0 460px; }
  .traj-right { flex: 0 0 auto; }
  .traj-right .twin { height: 460px; flex: none; }
}
</style>
"""


def head_html():
    js = (_JS.replace('__DEFAULTS__', json.dumps(DEFAULT_PARAMS))
             .replace('__LABELS__', json.dumps(KIND_LABEL))
             .replace('__SEED__', json.dumps(SEED)))
    css = _CSS
    tokens = {
        'BG_PAGE': t.BG_PAGE, 'BG_CARD': t.BG_CARD, 'BORDER_CARD': t.BORDER_CARD,
        'DIVIDER': t.DIVIDER, 'TEXT': t.TEXT, 'TEXT_STRONG': t.TEXT_STRONG,
        'TEXT_MUTED': t.TEXT_MUTED, 'ACCENT': t.ACCENT, 'DANGER': t.DANGER,
        'MONO': wt.MONO,
        'K_GOTO': KIND_COLOR['goto'], 'K_PICK': KIND_COLOR['pick'],
        'K_PLACE': KIND_COLOR['place'], 'K_ROTATE': KIND_COLOR['rotate'],
    }
    for k, v in tokens.items():
        css = css.replace(f'%{k}%', v)
    return css + js
