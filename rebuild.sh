#!/usr/bin/env bash
#
# rebuild.sh — чистая пересборка рабочего пространства.
#
# ЗАЧЕМ ЭТО НУЖНО
# ---------------
# У colcon есть два режима сборки Python-пакетов:
#
#   colcon build                    — копирует файлы в install/
#   colcon build --symlink-install  — ставит ссылки на исходники
#
# Режимы несовместимы между собой. Если собрать пакет сначала одним
# способом, а потом другим, в install/ остаётся мешанина: исполняемый файл
# от одного режима, а метаданные Python — от другого. Узел падает при
# запуске с сообщением:
#
#   importlib.metadata.PackageNotFoundError:
#       No package metadata was found for relay-reliable
#
# Коварство в том, что СБОРКА при этом проходит успешно и без единого
# предупреждения — ошибка вылезает только при запуске. И ломаются обычно
# не все пакеты сразу, а лишь часть, что сбивает с толку ещё сильнее.
#
# Лечится единственным надёжным способом: удалить результаты сборки
# полностью и собрать заново одним режимом. Этим скрипт и занимается.
#
# ИСПОЛЬЗОВАНИЕ
# -------------
#   ./rebuild.sh              обычная сборка (надёжнее всего)
#   ./rebuild.sh --symlink    сборка ссылками (удобно править YAML на лету)
#   ./rebuild.sh --keep       без очистки, только досборка изменённого
#
# Сборка занимает 2-5 минут на Raspberry Pi 4: дольше всего компилируется
# драйвер лидара на C++.

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

SYMLINK=0
CLEAN=1

for arg in "$@"; do
    case "$arg" in
        --symlink) SYMLINK=1 ;;
        --keep)    CLEAN=0 ;;
        -h|--help)
            sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "Неизвестный аргумент: $arg" >&2
            echo "Допустимо: --symlink, --keep, --help" >&2
            exit 1
            ;;
    esac
done

# --- Проверка, что мы там, где думаем -------------------------------------
if [ ! -d "$WS_DIR/src" ]; then
    echo "ОШИБКА: в $WS_DIR нет папки src." >&2
    echo "Скрипт должен лежать в корне рабочего пространства (~/ros2_ws)." >&2
    exit 1
fi

# --- Подключаем ROS --------------------------------------------------------
# Без этого colcon не найдёт ни одной зависимости.
if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        # shellcheck disable=SC1091
        source /opt/ros/jazzy/setup.bash
        echo "Подключён ROS 2 Jazzy"
    else
        echo "ОШИБКА: не найден /opt/ros/jazzy/setup.bash" >&2
        echo "Проверьте, что ROS 2 Jazzy установлен." >&2
        exit 1
    fi
fi

# ВАЖНО: собираем в чистом окружении. Если сейчас подключён install/ этого же
# рабочего пространства, colcon может подхватить старые версии пакетов
# и результат окажется непредсказуемым.
if [[ ":${AMENT_PREFIX_PATH:-}:" == *":$WS_DIR/install"* ]]; then
    echo
    echo "ВНИМАНИЕ: в этом терминале уже подключён install/ из $WS_DIR."
    echo "Для чистой сборки откройте НОВЫЙ терминал, в котором выполнен"
    echo "только 'source /opt/ros/jazzy/setup.bash', и запустите скрипт там."
    echo
    read -r -p "Всё равно продолжить? [y/N] " answer
    [[ "$answer" =~ ^[Yy]$ ]] || exit 1
fi

# --- Какая версия кода собирается ------------------------------------------
# Частая и обидная ситуация: правка выложена в репозиторий, но на роботе
# не выполнен git pull, и человек продолжает ловить уже исправленную
# ошибку. Печатаем коммит явно, чтобы это было видно сразу.
if git -C "$WS_DIR" rev-parse --git-dir >/dev/null 2>&1; then
    BRANCH="$(git -C "$WS_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null)"
    COMMIT="$(git -C "$WS_DIR" log -1 --format='%h %s' 2>/dev/null)"
    echo "Ветка:  $BRANCH"
    echo "Коммит: $COMMIT"

    # Есть ли на сервере что-то новее?
    if git -C "$WS_DIR" fetch --quiet origin "$BRANCH" 2>/dev/null; then
        BEHIND="$(git -C "$WS_DIR" rev-list --count HEAD..FETCH_HEAD 2>/dev/null || echo 0)"
        if [ "${BEHIND:-0}" -gt 0 ]; then
            echo
            echo "ВНИМАНИЕ: на GitHub есть $BEHIND новых коммит(ов),"
            echo "которых нет у вас локально. Сначала выполните:"
            echo
            echo "    git pull"
            echo
            read -r -p "Собрать всё равно, старую версию? [y/N] " answer
            [[ "$answer" =~ ^[Yy]$ ]] || exit 1
        fi
    fi
    echo
fi

# --- Очистка ---------------------------------------------------------------
if [ "$CLEAN" -eq 1 ]; then
    echo "Удаляю build/ install/ log/ ..."
    rm -rf build install log

    # Побочный продукт режима --symlink-install: метаданные Python остаются
    # прямо в исходниках и переживают удаление install/. Если их не убрать,
    # старая поломка воспроизведётся снова.
    echo "Удаляю следы предыдущих сборок из src/ ..."
    find src -maxdepth 3 -name "*.egg-info" -type d -exec rm -rf {} + 2>/dev/null || true
    find src -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
fi

# --- Сборка ----------------------------------------------------------------
if [ "$SYMLINK" -eq 1 ]; then
    echo "Собираю в режиме ССЫЛОК (--symlink-install) ..."
    colcon build --symlink-install
else
    echo "Собираю в ОБЫЧНОМ режиме ..."
    colcon build
fi

echo
echo "======================================================================"
echo "  Сборка завершена."
echo
echo "  Теперь ОБЯЗАТЕЛЬНО выполните в каждом открытом терминале:"
echo
echo "      source $WS_DIR/install/setup.bash"
echo
echo "  Проверить, что узлы на месте:"
echo
echo "      ros2 pkg executables robot_navigation"
echo "======================================================================"
