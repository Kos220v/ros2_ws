#!/usr/bin/env bash
#
# setup_dds.sh — подключает настройку CycloneDDS для большого стека узлов.
#
# ЗАЧЕМ
# =====
# Полный стек робота поднимает больше двух десятков процессов. Каждый из
# них — отдельный участник сети DDS. При обнаружении через unicast (а так
# бывает на Wi-Fi, через VPN и на loopback) количество участников
# ограничено, и лишние узлы падают с сообщением:
#
#     Failed to find a free participant index for domain 0
#
# Падает при этом не «виноватый» узел, а тот, кому не хватило места, —
# то есть случайный. В логе это выглядит как массовое падение половины
# стека без внятной причины.
#
# Скрипт прописывает путь к файлу настройки в ~/.bashrc. Выполняется один
# раз, повторные запуски безопасны.
#
#     ./setup_dds.sh
#
# После этого ОБЯЗАТЕЛЬНО откройте новый терминал либо выполните
# `source ~/.bashrc` — переменная должна быть установлена ДО ros2 launch.

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="$WS_DIR/src/robot_navigation/config/cyclonedds.xml"
BASHRC="$HOME/.bashrc"
MARKER="# CycloneDDS для стека робота (добавлено setup_dds.sh)"

if [ ! -f "$CONFIG" ]; then
    echo "ОШИБКА: не найден $CONFIG" >&2
    echo "Запускайте скрипт из корня рабочего пространства." >&2
    exit 1
fi

echo "Файл настройки: $CONFIG"
echo

# --- Текущее состояние -----------------------------------------------------
CURRENT="${CYCLONEDDS_URI:-}"
if [ -n "$CURRENT" ]; then
    echo "Сейчас CYCLONEDDS_URI = $CURRENT"
    if [ "$CURRENT" = "file://$CONFIG" ]; then
        echo "Настройка уже подключена в этом терминале."
    else
        echo "ВНИМАНИЕ: указан другой файл. Проверьте, что в нём тоже поднят"
        echo "параметр MaxAutoParticipantIndex, иначе проблема останется."
    fi
    echo
fi

# --- Прописываем в ~/.bashrc ------------------------------------------------
if grep -qF "$MARKER" "$BASHRC" 2>/dev/null; then
    echo "Запись в ~/.bashrc уже есть — обновляю путь."
    # Убираем прежний блок целиком, чтобы не плодить дубликаты
    tmp="$(mktemp)"
    grep -vF "$MARKER" "$BASHRC" | grep -v '^export CYCLONEDDS_URI=' > "$tmp"
    mv "$tmp" "$BASHRC"
fi

{
    echo ""
    echo "$MARKER"
    echo "export CYCLONEDDS_URI=file://$CONFIG"
} >> "$BASHRC"

echo "В ~/.bashrc добавлено:"
echo "    export CYCLONEDDS_URI=file://$CONFIG"
echo

# --- Проверка на зависшие процессы -----------------------------------------
# Лишние процессы от прошлых запусков тоже занимают места участников,
# поэтому лимит можно исчерпать даже с правильной настройкой.
LEFTOVERS="$(pgrep -c -f 'ros2 launch|_node|_server|_control' 2>/dev/null || true)"
if [ "${LEFTOVERS:-0}" -gt 0 ]; then
    echo "Обнаружено запущенных процессов ROS: $LEFTOVERS"
    echo "Перед новым запуском их стоит закрыть, иначе они продолжат"
    echo "занимать места участников:"
    echo
    echo "    pgrep -a -f 'ros2 launch'      # посмотреть, что живо"
    echo "    pkill -f 'ros2 launch'         # закрыть launch со всем деревом"
    echo
fi

echo "======================================================================"
echo "  Готово. Откройте НОВЫЙ терминал или выполните:"
echo
echo "      source ~/.bashrc"
echo
echo "  Проверить, что переменная видна:"
echo
echo "      echo \$CYCLONEDDS_URI"
echo "======================================================================"
