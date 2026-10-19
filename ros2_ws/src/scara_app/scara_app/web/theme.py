"""
theme.py - Design system de la interfaz web (CSS).

Los colores NO se definen aqui: salen de gui/theme.py, asi la version de
escritorio (PyQt5) y la web (NiceGUI) comparten los mismos tokens. Si
cambias un color alla, cambia en las dos.

Pensado para iPad (horizontal 1180 x ~760, vertical 820 x ~1100):
  - Botones y campos de minimo 48-52 px de alto (dedo, no mouse).
  - Campos con letra >= 16 px: Safari hace zoom si la letra es menor.
  - Sin fuentes de internet: en la demo la red puede no tener salida.
"""
from scara_app.gui import theme as t

MONO = ('"JetBrains Mono", "IBM Plex Mono", "Fira Code", "SF Mono", ui-monospace, '
        'Menlo, "DejaVu Sans Mono", monospace')
SANS = ('Inter, "IBM Plex Sans", -apple-system, BlinkMacSystemFont, system-ui, '
        'Ubuntu, "Noto Sans", sans-serif')

# Etiquetas para "Agregar a pantalla de inicio" en el iPad (se abre sin barra de Safari)
HEAD_HTML = f"""
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black">
<meta name="apple-mobile-web-app-title" content="SCARA">
<meta name="theme-color" content="{t.BG_PAGE}">
""" + """
<script>
/* Dead-man switch en el NAVEGADOR (iPad o PC).
 * Mientras un dedo este sobre un boton .deadman, o la barra espaciadora
 * presionada con un boton .deadman visible, se le manda al servidor un
 * evento 'deadman' cada 100 ms; el servidor lo pasa al ESP32 como "dm1".
 * Los latidos salen de aqui (no del servidor): si el iPad pierde el WiFi o
 * se bloquea, dejan de llegar y el firmware detiene los motores (300 ms).
 */
window.scaraDeadman = (function () {
  const PERIOD_MS = 100;
  const held = new Set();
  let timer = null, source = null;
  const send = (args) => { try { emitEvent('deadman', args); } catch (e) {} };
  function render(on) {
    document.querySelectorAll('.deadman').forEach(b => {
      b.classList.toggle('held', on);
      const t = b.querySelector('.deadman-text');
      if (t) t.textContent = on ? b.dataset.held : b.dataset.idle;
    });
  }
  function update() {
    const on = held.size > 0;
    if (on && !timer) {
      send({held: true, start: true, source: source});
      timer = setInterval(() => send({held: true}), PERIOD_MS);
    } else if (!on && timer) {
      clearInterval(timer); timer = null;
      send({held: false});
    }
    render(on);
  }
  function press(key, src) { if (!held.size) source = src; held.add(key); update(); }
  function release(key) { held.delete(key); update(); }
  function releaseAll() { held.clear(); update(); }
  const visibleButton = () => [...document.querySelectorAll('.deadman')]
      .find(b => b.offsetParent !== null);

  document.addEventListener('pointerdown', e => {
    const b = e.target.closest('.deadman');
    if (!b) return;
    e.preventDefault();
    try { b.setPointerCapture(e.pointerId); } catch (err) {}
    press('p' + e.pointerId, b.dataset.dm);
  });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(type =>
    document.addEventListener(type, e => release('p' + e.pointerId)));
  document.addEventListener('keydown', e => {
    if (e.code !== 'Space') return;
    const b = visibleButton();
    if (!b) return;
    e.preventDefault();
    if (!e.repeat) press('space', b.dataset.dm);
  }, true);
  document.addEventListener('keyup', e => {
    if (e.code !== 'Space' || !held.has('space')) return;
    e.preventDefault();
    release('space');
  }, true);
  window.addEventListener('blur', releaseAll);
  document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });
  document.addEventListener('contextmenu', e => {
    if (e.target.closest('.deadman')) e.preventDefault();
  });
  return {releaseAll};
})();
</script>
"""


def css() -> str:
    return f"""
<style>
/* ---------------- Pagina ---------------- */
:root {{ --scara-accent: {t.ACCENT}; }}
html, body {{
  height: 100%;
  background: {t.BG_PAGE} !important;
  overscroll-behavior: none;
}}
body {{
  color: {t.TEXT};
  font-family: {SANS};
  -webkit-tap-highlight-color: transparent;
  -webkit-text-size-adjust: 100%;
}}
button, .q-btn, .segment, input {{ touch-action: manipulation; }}
.q-page {{ min-height: 0 !important; }}
.nicegui-content {{
  height: 100dvh;
  padding: 6px !important;
  gap: 0 !important;
}}
.mono {{ font-family: {MONO}; }}
.grow {{ flex: 1 1 auto; }}

/* ---------------- Tarjeta principal ---------------- */
.card {{
  flex: 1 1 auto; min-height: 0; width: 100%;
  display: flex; flex-direction: column;
  background: {t.BG_CARD};
  border: 1px solid {t.BORDER_CARD};
  border-radius: 10px;
  overflow: hidden;
}}

/* ---------------- Encabezado ---------------- */
.header {{
  display: flex; align-items: center; gap: 20px;
  padding: 12px 16px 12px 22px;
  border-bottom: 1px solid {t.DIVIDER};
}}
.conn {{ display: flex; align-items: center; gap: 8px; font-family: {MONO}; font-size: 15px; }}
.conn-dot {{ font-size: 14px; line-height: 1; }}
.status-box {{ text-align: right; min-width: 0; }}
.status-value {{
  font-family: {MONO}; font-size: 22px; color: {t.ACCENT};
  white-space: nowrap; overflow: hidden; text-overflow: ellipsis;
}}
.caps {{
  font-size: 11px; font-weight: 700; letter-spacing: 1.2px;
  color: {t.TEXT_MUTED}; text-transform: uppercase;
}}

/* ---------------- Pestanas ---------------- */
.tabs {{ border-bottom: 1px solid {t.DIVIDER}; padding-left: 8px; }}
.tabs .q-tab {{ min-height: 52px; padding: 0 18px; color: {t.TEXT_MUTED}; }}
.tabs .q-tab__label {{ font-size: 16px; font-weight: 700; }}
.tabs .q-tab--active {{ color: {t.TEXT_STRONG}; }}
.tabs .q-tab__indicator {{ background: {t.ACCENT}; height: 2px; }}
.panels {{ flex: 1 1 auto; min-height: 0; background: transparent !important; }}
.panels > .q-panel {{ height: 100%; }}
.panels .q-tab-panel {{
  height: 100%; padding: 0 !important; gap: 0 !important;
  display: flex; flex-direction: column; align-items: stretch !important;
}}
.tab-body {{
  flex: 1 1 auto; min-height: 0; width: 100%; align-self: stretch;
  display: flex; flex-direction: column;
  padding: 20px 22px 20px;
}}

/* ---------------- Botones ---------------- */
.btn {{
  min-height: 52px; padding: 0 26px;
  font-size: 16px; font-weight: 700; letter-spacing: 0;
  border-radius: 4px;
}}
.btn.disabled, .btn[disabled] {{ opacity: 1 !important; }}
.btn-primary {{ background: {t.PRIMARY} !important; color: {t.TEXT_STRONG} !important; }}
.btn-primary.disabled {{
  background: {t.DIVIDER} !important; color: {t.TEXT_DISABLED} !important;
}}
.btn-secondary {{
  background: transparent !important; color: {t.TEXT_STRONG} !important;
  border: 1px solid {t.BORDER_CARD};
}}
.btn-secondary.disabled {{ border-color: {t.DIVIDER}; color: {t.TEXT_DISABLED} !important; }}
.btn-danger {{ background: {t.DANGER_BG} !important; color: {t.TEXT_STRONG} !important; }}
.btn-danger.disabled {{
  background: {t.DANGER_BG_OFF} !important; color: {t.TEXT_DISABLED} !important;
}}
.btn-text {{
  min-height: 40px; padding: 0 4px;
  background: transparent !important; color: {t.TEXT} !important;
  font-size: 14px; font-weight: 600;
}}
.btn-estop {{ min-height: 56px; min-width: 128px; font-size: 18px; letter-spacing: 1px; }}
.btn-row {{ display: flex; gap: 12px; flex-wrap: wrap; }}

/* Selector segmentado (Codo +1/-1, sentido del motor) */
.segmented {{ display: inline-flex; }}
.segmented .segment {{
  min-height: 52px; min-width: 64px; padding: 0 16px;
  background: transparent !important; color: {t.TEXT_MUTED} !important;
  border: 1px solid {t.BORDER_CARD}; border-radius: 0;
  font-family: {MONO}; font-size: 16px; font-weight: 700;
}}
.segmented .segment:first-child {{ border-radius: 4px 0 0 4px; border-right: none; }}
.segmented .segment:last-child {{ border-radius: 0 4px 4px 0; }}
.segmented .segment.on {{
  background: {t.DIVIDER} !important; color: {t.ACCENT} !important; border-color: {t.ACCENT};
}}
.segmented .segment.on:first-child {{ border-right: 1px solid {t.ACCENT}; }}
.segmented .segment.on + .segment {{ border-left: none; }}

/* ---------------- Campos numericos ---------------- */
.num .q-field__control {{
  background: {t.BG_PAGE}; border-radius: 4px; padding: 0 12px;
}}
.num.q-field--outlined .q-field__control:before {{ border: 1px solid {t.BORDER_CARD}; }}
.num.q-field--outlined.q-field--focused .q-field__control:after {{
  border: 1px solid {t.ACCENT}; border-width: 1px !important;
}}
.num input {{
  font-family: {MONO}; color: {t.TEXT} !important; text-align: right;
  -moz-appearance: textfield;
}}
.num input::-webkit-outer-spin-button,
.num input::-webkit-inner-spin-button {{ -webkit-appearance: none; margin: 0; }}
.num .q-field__suffix {{ font-family: {MONO}; color: {t.TEXT_MUTED}; padding-left: 4px; }}
.num-coord {{ width: 150px; }}
.num-coord .q-field__control {{ height: 56px; }}
.num-coord input {{ font-size: 22px; }}
.num-param {{ width: 96px; }}
.num-param.q-field--outlined .q-field__control:before {{ border-color: {t.DIVIDER}; }}
.num-param .q-field__control {{ height: 48px; padding: 0 10px; }}
.num-param input {{ font-size: 17px; }}
.field {{ display: flex; flex-direction: column; gap: 6px; }}

/* ---------------- Historial / consola ---------------- */
.log-wrap {{ flex: 1 1 auto; min-height: 140px; display: flex; flex-direction: column; gap: 6px; }}
.log-header {{ display: flex; align-items: center; justify-content: space-between; }}
.log {{
  flex: 1 1 auto; min-height: 0;
  display: flex; flex-direction: column-reverse;   /* se queda pegado abajo */
  overflow-y: auto; -webkit-overflow-scrolling: touch;
  background: {t.BG_PAGE};
  border: 1px solid {t.DIVIDER}; border-radius: 4px;
  padding: 10px 14px;
  font-family: {MONO}; font-size: 15px; line-height: 1.6;
}}
.log-line {{ display: flex; gap: 14px; }}
.log-stamp {{ color: {t.TEXT_MUTED}; flex: none; }}
.log-text {{ white-space: pre-wrap; word-break: break-word; }}

/* ---------------- Punto a punto ---------------- */
.p2p-params {{ display: flex; flex-wrap: wrap; gap: 10px; margin-top: 8px; }}
.p2p-target {{ display: flex; flex-wrap: wrap; align-items: flex-end; gap: 14px; margin: 22px 0; }}
.p2p-header {{ display: flex; align-items: center; justify-content: space-between; }}

/* ---------------- Prueba de circuito ---------------- */
.motors {{
  display: grid; gap: 14px;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  overflow-y: auto; min-height: 0; flex: 1 1 auto; align-content: start;
}}
.panel {{
  background: {t.BG_PAGE}; border: 1px solid {t.DIVIDER}; border-radius: 6px;
  padding: 14px 16px 16px;
  display: flex; flex-direction: column; gap: 12px;
}}
.panel-head {{ display: flex; align-items: baseline; justify-content: space-between; }}
.panel-title {{ font-family: {MONO}; font-size: 17px; font-weight: 700; color: {t.TEXT_STRONG}; }}
.panel-divider {{ height: 1px; background: {t.DIVIDER}; }}
.panel-row {{ display: flex; align-items: flex-end; justify-content: space-between; gap: 16px; }}
.small {{ font-size: 13px; color: {t.TEXT_MUTED}; }}
.leds {{ display: flex; gap: 12px; }}
.led {{
  width: 44px; height: 44px; border-radius: 50%;
  display: flex; align-items: center; justify-content: center;
  font-family: {MONO}; font-size: 16px; font-weight: 700;
  border: 2px solid {t.DIVIDER}; color: {t.TEXT_DISABLED}; background: transparent;
}}
.led.off {{ border-color: {t.BORDER_CARD}; color: {t.TEXT_MUTED}; background: {t.BG_PAGE}; }}
.led.on  {{ border-color: {t.ACCENT}; color: {t.BG_PAGE}; background: {t.ACCENT}; }}
.enc-counts {{
  font-family: {MONO}; font-size: 22px; font-weight: 700;
  color: {t.TEXT_STRONG}; text-align: right;
}}
.enc-pos {{ font-family: {MONO}; font-size: 15px; color: {t.TEXT_MUTED}; text-align: right; }}
.speed-row {{ display: flex; align-items: center; gap: 14px; }}
.speed-value {{ font-family: {MONO}; font-size: 16px; width: 56px; text-align: right; }}
.slider .q-slider__track {{ background: {t.DIVIDER} !important; }}
.slider .q-slider__selection {{ background: {t.PRIMARY} !important; }}
.slider .q-slider__thumb {{ color: {t.ACCENT} !important; }}
.panel .btn-row .btn {{ flex: 1 1 0; }}
.circuit-bottom {{
  display: flex; align-items: center; justify-content: space-between;
  gap: 12px; margin-top: 14px;
}}
.hint {{ font-size: 14px; color: {t.TEXT_MUTED}; }}

/* ---------------- Dead-man ---------------- */
.deadman {{
  display: flex; align-items: center; justify-content: center;
  min-height: 72px; margin-top: 4px;
  border: 2px dashed {t.ACCENT}; border-radius: 6px;
  color: {t.ACCENT}; font-size: 18px; font-weight: 700;
  user-select: none; -webkit-user-select: none; -webkit-touch-callout: none;
  touch-action: none; cursor: pointer;
}}
.deadman.held {{ background: {t.ACCENT}; color: {t.BG_PAGE}; border-style: solid; }}
.deadman .deadman-text {{ pointer-events: none; }}

/* ---------------- DK ---------------- */
.dk-grid {{
  display: grid; grid-template-columns: 44px 1fr 120px 110px;
  align-items: center; gap: 14px 16px;
}}
.dk-name {{ font-family: {MONO}; font-size: 22px; font-weight: 700; color: {t.TEXT_STRONG}; }}
.dk-limits {{ font-family: {MONO}; font-size: 13px; color: {t.TEXT_MUTED}; }}
.dk-readout {{ display: flex; flex-wrap: wrap; gap: 8px 40px; margin: 18px 0 14px; }}
.dk-value {{ font-family: {MONO}; font-size: 16px; }}
.dk-value.muted {{ color: {t.TEXT_MUTED}; }}
.num-dk {{ width: 120px; }}
.num-dk .q-field__control {{ height: 48px; }}
.num-dk input {{ font-size: 18px; }}

/* ---------------- Gemelo ---------------- */
.split {{ display: flex; gap: 18px; flex: 1 1 auto; min-height: 0; }}
.split > .left {{ flex: 0 0 470px; display: flex; flex-direction: column; min-height: 0; }}
.split > .right {{ flex: 1 1 auto; min-width: 0; display: flex; flex-direction: column; min-height: 0; }}
.twin {{
  flex: 1 1 auto; min-height: 0; display: flex; flex-direction: column;
  background: {t.BG_PAGE}; border: 1px solid {t.DIVIDER}; border-radius: 6px; overflow: hidden;
}}
.twin-head {{
  display: flex; align-items: center; justify-content: space-between;
  padding: 8px 10px 8px 14px; border-bottom: 1px solid {t.DIVIDER};
}}
.twin-body {{ flex: 1 1 auto; min-height: 0; display: flex; }}
.plan-wrap, .scene-wrap {{ flex: 1 1 auto; min-width: 0; position: relative; }}
.plan-wrap {{ cursor: crosshair; }}
.plan-layer {{ position: absolute; inset: 0; }}
.plan-layer > svg, .zbar > svg {{ width: 100%; height: 100%; display: block; }}
.plan-svg {{ width: 100%; height: 100%; display: block; }}
.scene3d, .scene-wrap > div {{ width: 100% !important; height: 100% !important; }}
.zbar {{ flex: 0 0 66px; border-left: 1px solid {t.DIVIDER}; }}
.legend {{ padding: 8px 14px; border-top: 1px solid {t.DIVIDER}; }}
.legend-row {{ display: flex; gap: 16px; flex-wrap: wrap; font-size: 12.5px; color: {t.TEXT_MUTED}; }}
.sw {{
  display: inline-block; width: 18px; height: 4px; border-radius: 2px;
  vertical-align: middle; margin-right: 6px;
}}
.sw-zone {{ background: {t.ACCENT}; opacity: .35; height: 10px; }}
.segmented.small .segment {{ min-height: 40px; min-width: 64px; font-size: 14px; font-family: {SANS}; }}
.compact .dk-grid {{ grid-template-columns: 34px 1fr 104px; gap: 10px 12px; }}
.compact .dk-limits {{ display: none; }}
.compact .num-dk {{ width: 104px; }}
.compact .log-wrap {{ min-height: 90px; }}
.compact .deadman {{ min-height: 64px; }}
.coords {{ display: flex; gap: 10px; align-items: flex-end; flex-wrap: wrap; }}
.coords .num-coord {{ width: 112px; }}
.coords .num-coord input {{ font-size: 19px; }}
.mini {{ font-family: {MONO}; font-size: 13.5px; line-height: 1.55; margin: 14px 0 12px; }}
.mini-ref {{ color: {t.TEXT}; }}
.mini-ref b {{ color: {t.ACCENT}; font-weight: 600; }}
.mini-real {{ color: {t.TEXT_MUTED}; }}
.p2p-row {{ display: flex; gap: 12px; align-items: flex-end; margin: 14px 0; flex-wrap: wrap; }}

/* ---------------- Vertical (iPad parado) ---------------- */
@media (max-width: 900px) {{
  .header {{ gap: 12px; padding-left: 16px; }}
  .status-value {{ font-size: 18px; }}
  .tab-body {{ padding: 16px; }}
  .dk-grid {{ grid-template-columns: 36px 1fr 110px; }}
  .dk-limits {{ display: none; }}
  .tab-body.compact {{ overflow-y: auto; -webkit-overflow-scrolling: touch; }}
  .split {{ flex-direction: column; flex: 0 0 auto; }}
  .split > .left {{ flex: 0 0 auto; }}
  .split > .right {{ flex: 0 0 auto; height: 560px; }}
  .compact .log-wrap {{ flex: 0 0 160px; }}
}}
</style>
"""
