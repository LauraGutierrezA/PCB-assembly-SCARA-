#!/usr/bin/env bash
# =====================================================================
#  scara_prep.sh - Lo que antes se hacia a mano en la "Terminal 1" y al
#  inicio de la "Terminal 3". Lo llama tools/scara.launch.py ANTES de abrir
#  el agente y la interfaz web (el agente tiene que estar cerrado para
#  flashear: los dos usan el mismo puerto).
#
#  Pasos (cada uno se puede saltar con su argumento):
#    1. Descomprimir un zip en ~/scara_ws y "tocar" sus archivos (el zip trae
#       fechas viejas y, sin touch, idf.py / colcon creen que no cambiaron).
#       Si el zip trae un componente NUEVO de firmware -> idf.py reconfigure.
#    2. idf.py build  (salida filtrada; el log completo queda en
#       firmware/build_last.log)
#    3. idf.py -p <puerto> flash
#    4. colcon build --packages-select scara_app
#       (+ scara_description scara_twin si SCARA_TWIN = rviz o gazebo: GEMELO)
#
#  ESP-IDF se ejecuta en un entorno LIMPIO (sin ROS cargado): el componente
#  de micro-ROS para ESP-IDF falla si ROS 2 esta "sourceado" en la misma
#  terminal.
#
#  Uso directo (tambien sirve sin el launch):
#    tools/scara_prep.sh [zip] [flash:true|false] [web:true|false] [puerto] [export.sh]
#  (el launch usa las variables SCARA_WS, SCARA_ZIP, SCARA_FLASH, SCARA_WEB,
#   SCARA_PORT y SCARA_IDF en vez de los argumentos)
# =====================================================================
set -o pipefail

# El launch pasa todo por variables de entorno (un argumento vacio se perderia)
WS="${SCARA_WS:-$HOME/scara_ws}"
ZIP="${SCARA_ZIP-${1:-}}"
DO_FLASH="${SCARA_FLASH:-${2:-true}}"
DO_WEB="${SCARA_WEB:-${3:-true}}"
PORT="${SCARA_PORT:-${4:-/dev/ttyUSB0}}"
IDF_EXPORT="${SCARA_IDF-${5:-}}"
TWIN="${SCARA_TWIN:-${6:-none}}"          # GEMELO: none | rviz | gazebo
WS="${WS/#\~/$HOME}"

ZIP="${ZIP/#\~/$HOME}"
IDF_EXPORT="${IDF_EXPORT/#\~/$HOME}"

step() { printf '\n\033[1;36m==== %s ====\033[0m\n' "$*"; }
fail() { printf '\n\033[1;31mERROR: %s\033[0m\n' "$*"; exit 1; }

cd "$WS" || fail "no existe $WS"

# ---------------------------------------------------------------- 1. zip
RECONFIGURE=false
if [ -n "$ZIP" ]; then
    step "1/4  Descomprimiendo $(basename "$ZIP")"
    [ -f "$ZIP" ] || fail "no encuentro el zip: $ZIP"
    mapfile -t FILES < <(unzip -Z1 "$ZIP" | grep -v '/$')
    # Componentes de firmware que el zip trae y que todavia no existen
    for f in "${FILES[@]}"; do
        if [[ "$f" =~ ^firmware/components/([^/]+)/ ]]; then
            [ -d "firmware/components/${BASH_REMATCH[1]}" ] || RECONFIGURE=true
        fi
    done
    unzip -o -q "$ZIP" -d "$WS" || fail "no se pudo descomprimir $ZIP"
    for f in "${FILES[@]}"; do [ -e "$f" ] && touch "$f"; done
    echo "  ${#FILES[@]} archivos actualizados"
    $RECONFIGURE && echo "  el zip trae un componente nuevo -> se hara idf.py reconfigure"
else
    step "1/4  Sin zip (se compila lo que ya esta en $WS)"
fi

# ---------------------------------------------------------------- 2-3. firmware
if [ "$DO_FLASH" = "true" ]; then
    # Buscar export.sh de ESP-IDF
    if [ -z "$IDF_EXPORT" ]; then
        for c in "${IDF_PATH:+$IDF_PATH/export.sh}" "$HOME/esp/esp-idf/export.sh" \
                 "$HOME"/esp/v*/esp-idf/export.sh "$HOME"/esp/esp-idf*/export.sh; do
            [ -n "$c" ] && [ -f "$c" ] && { IDF_EXPORT="$c"; break; }
        done
    fi
    [ -f "$IDF_EXPORT" ] || fail "no encuentro export.sh de ESP-IDF. Pasalo con idf:=/ruta/a/esp-idf/export.sh"

    [ -e "$PORT" ] || fail "no existe $PORT (esta conectado el ESP32?)"
    if command -v fuser >/dev/null && fuser "$PORT" >/dev/null 2>&1; then
        fail "$PORT esta ocupado (un agente micro-ROS abierto en otra terminal?). Cierralo y vuelve a lanzar."
    fi

    # Entorno limpio para ESP-IDF (sin ROS)
    run_idf() {
        env -i HOME="$HOME" USER="$USER" TERM="${TERM:-xterm}" LANG="${LANG:-C.UTF-8}" \
            PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
            bash --noprofile --norc -c ". '$IDF_EXPORT' >/dev/null 2>&1 && cd '$WS/firmware' && $1"
    }

    if $RECONFIGURE; then
        step "2/4  idf.py reconfigure (componente nuevo)"
        run_idf "idf.py reconfigure" > "$WS/firmware/reconfigure_last.log" 2>&1 \
            || { tail -30 "$WS/firmware/reconfigure_last.log"; fail "idf.py reconfigure"; }
    fi

    step "2/4  idf.py build"
    LOG="$WS/firmware/build_last.log"
    run_idf "idf.py build" > "$LOG" 2>&1
    BUILD_RC=$?
    grep -E "\.c\.obj|SCARA: controlador|error" "$LOG" | grep -v "^--" | tail -40
    if [ $BUILD_RC -ne 0 ]; then
        echo; grep -n -E "error|Error" "$LOG" | head -20
        fail "fallo idf.py build (log completo: $LOG)"
    fi
    echo "  build OK"

    step "3/4  idf.py -p $PORT flash"
    run_idf "idf.py -p '$PORT' flash" 2>&1 | grep -E "Wrote|Hard resetting|error|Error|A fatal"; FLASH_RC=${PIPESTATUS[0]}
    [ "$FLASH_RC" -eq 0 ] || fail "fallo el flasheo"
    echo "  flash OK"
    sleep 1   # que el ESP32 termine de reiniciar antes de abrir el agente
else
    step "2-3/4  Sin compilar ni flashear el ESP32 (flash:=false)"
fi

# ---------------------------------------------------------------- 4. web
if [ "$DO_WEB" = "true" ]; then
    PKGS="scara_app"
    if [ "$TWIN" != "none" ]; then        # GEMELO
        PKGS="scara_app scara_description scara_twin"
    fi
    step "4/4  colcon build $PKGS"
    [ -n "$ROS_DISTRO" ] || fail "ROS 2 no esta cargado: source /opt/ros/jazzy/setup.bash"
    cd "$WS/ros2_ws" || fail "no existe $WS/ros2_ws"
    for p in $PKGS; do
        [ -d "src/$p" ] || fail "no existe ros2_ws/src/$p (falta descomprimir el gemelo?)"
        find "src/$p" -type f -exec touch {} +
    done
    colcon build --packages-select $PKGS 2>&1 | tail -5
    [ "${PIPESTATUS[0]}" -eq 0 ] || fail "fallo colcon build"
else
    step "4/4  Sin compilar la interfaz (web:=false)"
    if [ "$TWIN" != "none" ] && [ ! -d "$WS/ros2_ws/install/scara_twin" ]; then
        fail "twin:=$TWIN pero scara_twin no esta compilado: lanza una vez con web:=true"
    fi
fi

printf '\n\033[1;32mListo: abriendo agente micro-ROS e interfaz web\033[0m\n'
