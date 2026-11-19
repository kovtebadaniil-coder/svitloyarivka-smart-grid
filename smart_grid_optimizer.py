import copy
import heapq
import math
from pathlib import Path

# ==============================================================================
# ПРОСТОРОВИЙ ШІ-АРХІТЕКТОР МІКРОМЕРЕЖІ "СВІТЛОЯРІВКА" (v0.4)
# Алгоритм: пріоритетний розподіл + A* маршрутизація з обходом перешкод +
# розрахунок перерізу кабелю (АВБбШв 0.4 кВ), втрат напруги (ПУЕ), CAPEX та SVG
# Без сторонніх бібліотек (чистий Python 3)
# ==============================================================================

GRID_SIZE = 100            # Розмір координатної сітки (100 x 100 клітинок)
CELL_SIZE_M = 10.0         # Масштаб: 1 клітинка сітки = 10 метрів (село 1 x 1 км)
VOLTAGE_V = 400.0          # Лінійна напруга трифазної мережі 0.4 кВ (ДСТУ EN 50160)
COS_PHI = 0.90             # Коефіцієнт потужності навантаження
GAMMA_AL = 34.4            # Питома провідність алюмінію, м/(Ом*мм^2)
X0_OHM_PER_M = 0.00008     # Питомий індуктивний опір кабелю 0.4 кВ, Ом/м
TRENCH_COST_UAH_M = 850.0  # Орієнтовна вартість земляних робіт і монтажу, грн/м

# Стандартні перерізи 4-жильного броньованого кабелю АВБбШв-1 (0.4 кВ):
# (переріз мм^2, допустимий струм у землі за ПУЕ в А, орієнтовна ціна кабелю грн/м)
CABLE_CATALOG = [
    (16, 75, 115.0),
    (25, 95, 165.0),
    (35, 115, 220.0),
    (50, 140, 295.0),
    (70, 170, 410.0),
    (95, 205, 550.0),
    (120, 235, 690.0),
    (150, 265, 860.0),
    (185, 300, 1080.0),
    (240, 345, 1420.0),
]

# Вхідні дані: координати збережено для наступності з версією v0.3
CONSUMERS_BASE = [
    {"name": "Лікарня та Водоканал", "coords": (15, 20), "priority": 1, "demand_kw": 50},
    {"name": "Фермерський комплекс", "coords": (85, 90), "priority": 2, "demand_kw": 120},
    {"name": "Житловий сектор", "coords": (40, 50), "priority": 3, "demand_kw": 200},
]

GENERATORS_BASE = [
    {"name": "Центральна СЕС + Акумулятори", "coords": (18, 22), "capacity_kw": 100},
    {"name": "Біогазова станція", "coords": (82, 88), "capacity_kw": 150},
    {"name": "Розподілені СЕС на дахах", "coords": (45, 45), "capacity_kw": 250},
]

# Перешкоди на карті "Світлоярівки" (прямокутники: x1, y1, x2, y2)
# Алгоритм A* прокладає підземні кабелі в обхід водойм, щільної забудови та ярів
OBSTACLES = [
    {"name": "Сільський ставок", "rect": (42, 46, 44, 50), "kind": "water"},
    {"name": "Господарські будівлі", "rect": (16, 20, 17, 22), "kind": "building"},
    {"name": "Яр і заболочена ділянка", "rect": (83, 88, 84, 90), "kind": "terrain"},
    {"name": "Центральна забудова", "rect": (55, 60, 72, 74), "kind": "building"},
]


def build_blocked_cells(obstacles):
    blocked = set()
    for obs in obstacles:
        x1, y1, x2, y2 = obs["rect"]
        for x in range(x1, x2 + 1):
            for y in range(y1, y2 + 1):
                blocked.add((x, y))
    return blocked


def euclidean_distance_cells(c1, c2):
    return math.hypot(c1[0] - c2[0], c1[1] - c2[1])


def astar_route(start, goal, blocked):
    """
    Пошук найкоротшого маршруту прокладання підземного кабелю алгоритмом A*
    по 4-направленій вуличній сітці (без діагональних зрізів через будівлі).
    Повертає (список_координат_маршруту, довжина_у_клітинках).
    """
    if start == goal:
        return [start], 0.0

    effective_blocked = blocked - {start, goal}

    open_heap = []
    heapq.heappush(open_heap, (0, start))
    came_from = {}
    g_score = {start: 0}

    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path, float(len(path) - 1)

        cx, cy = current
        for dx, dy in directions:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx <= GRID_SIZE and 0 <= ny <= GRID_SIZE):
                continue
            neighbor = (nx, ny)
            if neighbor in effective_blocked:
                continue

            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                h = abs(nx - goal[0]) + abs(ny - goal[1])
                heapq.heappush(open_heap, (tentative_g + h, neighbor))

    return None, float("inf")


def select_cable_and_calc(power_kw, length_m, max_drop_pct=5.0):
    """
    Підбирає стандартний підземний кабель АВБбШв 0.4 кВ за допустимим струмом
    та перевіряє втрати напруги. Якщо переріз не проходить по втратах,
    автоматично підвищує переріз або закладає 2 паралельні лінії.
    """
    if power_kw <= 0:
        return {
            "cable_type": "-",
            "current_a": 0.0,
            "voltage_drop_pct": 0.0,
            "capex_uah": 0.0,
        }

    tan_phi = math.tan(math.acos(COS_PHI))
    current_a = (power_kw * 1000.0) / (math.sqrt(3.0) * VOLTAGE_V * COS_PHI)

    for parallel_lines in (1, 2):
        line_current = current_a / parallel_lines
        for section_mm2, max_amp, cable_price in CABLE_CATALOG:
            if max_amp < line_current:
                continue

            r_total = (length_m / (GAMMA_AL * section_mm2)) / parallel_lines
            x_total = (X0_OHM_PER_M * length_m) / parallel_lines

            p_w = power_kw * 1000.0
            q_var = p_w * tan_phi
            delta_u_v = (p_w * r_total + q_var * x_total) / VOLTAGE_V
            delta_u_pct = (delta_u_v / VOLTAGE_V) * 100.0

            if delta_u_pct <= max_drop_pct:
                capex = length_m * (cable_price * parallel_lines + TRENCH_COST_UAH_M)
                label = (
                    f"АВБбШв 4x{section_mm2}"
                    if parallel_lines == 1
                    else f"2x(АВБбШв 4x{section_mm2})"
                )
                return {
                    "cable_type": label,
                    "current_a": current_a,
                    "voltage_drop_pct": delta_u_pct,
                    "capex_uah": capex,
                }

    return {
        "cable_type": "Потрібна лінія 10 кВ (перевищено ліміт 0.4 кВ)",
        "current_a": current_a,
        "voltage_drop_pct": 99.9,
        "capex_uah": 0.0,
    }


def optimize_microgrid(
    consumers_data,
    generators_data,
    obstacles,
    capacity_factor=1.0,
    scenario_title="Штатний режим",
):
    """
    Головна функція просторової оптимізації (v0.4):
    - сортує споживачів за пріоритетом (1 -> 2 -> 3);
    - застосовує коефіцієнт доступності генерації (наприклад, 0.5 для Тесту 4);
    - будує маршрути кабелів алгоритмом A* з обходом перешкод;
    - коректно обробляє дефіцит потужності (автоматичне розвантаження).
    """
    consumers = copy.deepcopy(consumers_data)
    generators = copy.deepcopy(generators_data)
    blocked = build_blocked_cells(obstacles)

    # У штатному режимі допустимі втрати до 5%, у післяаварійному - до 10% (ДСТУ EN 50160)
    allowed_drop_pct = 5.0 if capacity_factor >= 0.99 else 10.0

    for gen in generators:
        gen["initial_kw"] = round(gen["capacity_kw"] * capacity_factor, 1)
        gen["remaining_kw"] = gen["initial_kw"]

    sorted_consumers = sorted(consumers, key=lambda c: c["priority"])

    connections = []
    warnings = []
    total_length_m = 0.0
    total_euclid_m = 0.0
    total_capex_uah = 0.0
    total_demand_kw = sum(c["demand_kw"] for c in sorted_consumers)
    total_supplied_kw = 0.0

    for consumer in sorted_consumers:
        best_gen = None
        best_path = None
        best_len_cells = float("inf")

        # 1. Спроба повного заживлення від найближчого генератора по графу A*
        for gen in generators:
            if gen["remaining_kw"] >= consumer["demand_kw"]:
                path, dist_cells = astar_route(consumer["coords"], gen["coords"], blocked)
                if path is not None and dist_cells < best_len_cells:
                    best_len_cells = dist_cells
                    best_path = path
                    best_gen = gen

        if best_gen is not None:
            length_m = best_len_cells * CELL_SIZE_M
            euclid_m = euclidean_distance_cells(consumer["coords"], best_gen["coords"]) * CELL_SIZE_M
            elec = select_cable_and_calc(consumer["demand_kw"], length_m, max_drop_pct=allowed_drop_pct)

            best_gen["remaining_kw"] = round(best_gen["remaining_kw"] - consumer["demand_kw"], 1)
            total_length_m += length_m
            total_euclid_m += euclid_m
            total_capex_uah += elec["capex_uah"]
            total_supplied_kw += consumer["demand_kw"]

            if elec["voltage_drop_pct"] > 5.0:
                warnings.append(
                    f"УВАГА ПО НАПРУЗІ: резервна траса [{consumer['name']}] -> [{best_gen['name']}] "
                    f"({length_m:.0f} м) має втрати {elec['voltage_drop_pct']:.2f}% (у межах аварійного ліміту 10%)."
                )

            connections.append({
                "consumer": consumer["name"],
                "priority": consumer["priority"],
                "demand_kw": consumer["demand_kw"],
                "supplied_kw": consumer["demand_kw"],
                "status": "Заживлено (100%)",
                "generator": best_gen["name"],
                "gen_remaining_kw": best_gen["remaining_kw"],
                "path": best_path,
                "length_m": length_m,
                "euclid_m": euclid_m,
                "cable_type": elec["cable_type"],
                "current_a": elec["current_a"],
                "voltage_drop_pct": elec["voltage_drop_pct"],
                "capex_uah": elec["capex_uah"],
            })
        else:
            # 2. Аварійний режим дефіциту: підключаємо часткове живлення (графік розвантаження)
            available_gens = [g for g in generators if g["remaining_kw"] > 0]
            if available_gens:
                best_partial = None
                best_partial_path = None
                best_partial_len = float("inf")

                for gen in available_gens:
                    path, dist_cells = astar_route(consumer["coords"], gen["coords"], blocked)
                    if path is None:
                        continue
                    if (
                        best_partial is None
                        or gen["remaining_kw"] > best_partial["remaining_kw"]
                        or (gen["remaining_kw"] == best_partial["remaining_kw"] and dist_cells < best_partial_len)
                    ):
                        best_partial = gen
                        best_partial_path = path
                        best_partial_len = dist_cells

                supplied = best_partial["remaining_kw"]
                deficit = round(consumer["demand_kw"] - supplied, 1)
                pct = (supplied / consumer["demand_kw"]) * 100.0
                length_m = best_partial_len * CELL_SIZE_M
                euclid_m = euclidean_distance_cells(consumer["coords"], best_partial["coords"]) * CELL_SIZE_M
                elec = select_cable_and_calc(supplied, length_m, max_drop_pct=allowed_drop_pct)

                best_partial["remaining_kw"] = 0.0
                total_length_m += length_m
                total_euclid_m += euclid_m
                total_capex_uah += elec["capex_uah"]
                total_supplied_kw += supplied

                warnings.append(
                    f"ДЕФІЦИТ ПОТУЖНОСТІ: вузол [{consumer['name']}] (пріоритет {consumer['priority']}) "
                    f"отримав лише {supplied:.1f} із {consumer['demand_kw']} кВт ({pct:.1f}%) від "
                    f"[{best_partial['name']}]. Необхідне аварійне розвантаження на {deficit:.1f} кВт."
                )

                connections.append({
                    "consumer": consumer["name"],
                    "priority": consumer["priority"],
                    "demand_kw": consumer["demand_kw"],
                    "supplied_kw": supplied,
                    "status": f"Часткове живлення ({pct:.1f}%)",
                    "generator": best_partial["name"],
                    "gen_remaining_kw": 0.0,
                    "path": best_partial_path,
                    "length_m": length_m,
                    "euclid_m": euclid_m,
                    "cable_type": elec["cable_type"],
                    "current_a": elec["current_a"],
                    "voltage_drop_pct": elec["voltage_drop_pct"],
                    "capex_uah": elec["capex_uah"],
                })
            else:
                warnings.append(
                    f"ПОВНИЙ ДЕФІЦИТ: вузол [{consumer['name']}] (пріоритет {consumer['priority']}, "
                    f"{consumer['demand_kw']} кВт) тимчасово відключено для збереження балансу мережі."
                )
                connections.append({
                    "consumer": consumer["name"],
                    "priority": consumer["priority"],
                    "demand_kw": consumer["demand_kw"],
                    "supplied_kw": 0.0,
                    "status": "Відключено (дефіцит)",
                    "generator": "Немає доступного джерела",
                    "gen_remaining_kw": 0.0,
                    "path": [],
                    "length_m": 0.0,
                    "euclid_m": 0.0,
                    "cable_type": "-",
                    "current_a": 0.0,
                    "voltage_drop_pct": 0.0,
                    "capex_uah": 0.0,
                })

    total_capacity_kw = sum(g["initial_kw"] for g in generators)
    total_reserve_kw = round(sum(g["remaining_kw"] for g in generators), 1)
    p1_demand = sum(c["demand_kw"] for c in sorted_consumers if c["priority"] == 1)
    p1_supplied = sum(c["supplied_kw"] for c in connections if c["priority"] == 1)

    return {
        "title": scenario_title,
        "capacity_factor": capacity_factor,
        "connections": connections,
        "warnings": warnings,
        "total_length_m": total_length_m,
        "total_euclid_m": total_euclid_m,
        "total_capex_uah": total_capex_uah,
        "total_demand_kw": total_demand_kw,
        "total_supplied_kw": total_supplied_kw,
        "total_capacity_kw": total_capacity_kw,
        "total_reserve_kw": total_reserve_kw,
        "p1_coverage_pct": (p1_supplied / p1_demand * 100.0) if p1_demand else 100.0,
        "overall_coverage_pct": (total_supplied_kw / total_demand_kw * 100.0) if total_demand_kw else 100.0,
        "consumers": consumers,
        "generators": generators,
    }


def export_svg_map(result, obstacles, filename):
    """
    Генерує чисту векторну схему підключень (.svg) без сторонніх бібліотек.
    Файл відкривається в будь-якому браузері та підтримується на GitHub.
    """
    width, height = 890, 680
    pad_left, pad_top = 60, 70
    map_size = 560
    scale = map_size / GRID_SIZE

    def to_px(x, y):
        px = pad_left + x * scale
        py = pad_top + (GRID_SIZE - y) * scale
        return px, py

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#0f172a"/>',
        f'<text x="60" y="34" fill="#f8fafc" font-family="Segoe UI, Arial, sans-serif" font-size="18" font-weight="bold">Світлоярівка Smart Grid (v0.4) - {result["title"]}</text>',
        f'<text x="60" y="54" fill="#94a3b8" font-family="Segoe UI, Arial, sans-serif" font-size="12">Маршрутизація A* (1 клітинка = {int(CELL_SIZE_M)} м) | Довжина трас: {result["total_length_m"]:.0f} м | Покриття 1-го пріоритету: {result["p1_coverage_pct"]:.0f}%</text>',
        f'<rect x="{pad_left}" y="{pad_top}" width="{map_size}" height="{map_size}" fill="#1e293b" stroke="#334155" stroke-width="1.5"/>',
    ]

    # Координатна сітка з кроком 10 клітинок (100 м)
    for step in range(0, GRID_SIZE + 1, 10):
        x_px, _ = to_px(step, 0)
        _, y_px = to_px(0, step)
        lines.append(f'<line x1="{x_px:.1f}" y1="{pad_top}" x2="{x_px:.1f}" y2="{pad_top + map_size}" stroke="#334155" stroke-width="0.6" stroke-dasharray="3,3"/>')
        lines.append(f'<line x1="{pad_left}" y1="{y_px:.1f}" x2="{pad_left + map_size}" y2="{y_px:.1f}" stroke="#334155" stroke-width="0.6" stroke-dasharray="3,3"/>')
        lines.append(f'<text x="{x_px:.1f}" y="{pad_top + map_size + 18}" fill="#94a3b8" font-family="Arial" font-size="10" text-anchor="middle">{step * int(CELL_SIZE_M)} м</text>')
        lines.append(f'<text x="{pad_left - 8}" y="{y_px + 3:.1f}" fill="#94a3b8" font-family="Arial" font-size="10" text-anchor="end">{step * int(CELL_SIZE_M)}</text>')

    # Перешкоди (водойми, забудова, яри)
    obs_colors = {"water": "#0284c7", "building": "#475569", "terrain": "#78350f"}
    for obs in obstacles:
        x1, y1, x2, y2 = obs["rect"]
        px1, py_bottom = to_px(x1, y1)
        px2, py_top = to_px(x2 + 1, y2 + 1)
        w = max(px2 - px1, 4)
        h = max(py_bottom - py_top, 4)
        color = obs_colors.get(obs["kind"], "#64748b")
        lines.append(f'<rect x="{px1:.1f}" y="{py_top:.1f}" width="{w:.1f}" height="{h:.1f}" fill="{color}" fill-opacity="0.78" stroke="#cbd5e1" stroke-width="0.8"/>')

    # Кабельні траси (A*)
    route_colors = {1: "#ef4444", 2: "#f59e0b", 3: "#38bdf8"}
    for conn in result["connections"]:
        if not conn["path"]:
            continue
        pts = " ".join(f"{to_px(x, y)[0]:.1f},{to_px(x, y)[1]:.1f}" for x, y in conn["path"])
        color = route_colors.get(conn["priority"], "#a855f7")
        dash = ' stroke-dasharray="6,4"' if "Часткове" in conn["status"] else ""
        lines.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="3.2"{dash} stroke-linecap="round" stroke-linejoin="round"/>')

    # Вузли генерації (зелені квадрати)
    for gen in result["generators"]:
        gx, gy = to_px(*gen["coords"])
        anchor = "end" if gen["coords"][0] > 55 else "start"
        tx = gx - 10 if anchor == "end" else gx + 10
        lines.append(f'<rect x="{gx - 6:.1f}" y="{gy - 6:.1f}" width="12" height="12" fill="#22c55e" stroke="#f8fafc" stroke-width="1.5"/>')
        lines.append(f'<text x="{tx:.1f}" y="{gy + 18:.1f}" fill="#86efac" font-family="Segoe UI, Arial" font-size="11" text-anchor="{anchor}">{gen["name"]} ({gen["initial_kw"]:.0f} кВт)</text>')

    # Вузли споживання (кола за пріоритетом)
    for cons in result["consumers"]:
        cx, cy = to_px(*cons["coords"])
        color = route_colors.get(cons["priority"], "#e2e8f0")
        anchor = "end" if cons["coords"][0] > 55 else "start"
        tx = cx - 10 if anchor == "end" else cx + 10
        lines.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="6.5" fill="{color}" stroke="#f8fafc" stroke-width="1.5"/>')
        lines.append(f'<text x="{tx:.1f}" y="{cy - 12:.1f}" fill="#f8fafc" font-family="Segoe UI, Arial" font-size="11" font-weight="bold" text-anchor="{anchor}">P{cons["priority"]}: {cons["name"]} ({cons["demand_kw"]} кВт)</text>')

    # Права панель легенди та метрик
    lx = 640
    lines.append(f'<rect x="{lx}" y="{pad_top}" width="230" height="350" rx="8" fill="#1e293b" stroke="#334155" stroke-width="1"/>')
    lines.append(f'<text x="{lx + 14}" y="{pad_top + 26}" fill="#f8fafc" font-family="Segoe UI, Arial" font-size="13" font-weight="bold">Легенда та підсумки</text>')

    legend_items = [
        ("#ef4444", "Пріоритет 1 (Критична інфр.)"),
        ("#f59e0b", "Пріоритет 2 (Агросектор)"),
        ("#38bdf8", "Пріоритет 3 (Житловий фонд)"),
        ("#22c55e", "Локальна генерація (ВДЕ)"),
        ("#0284c7", "Перешкода: водойма"),
        ("#475569", "Перешкода: забудова"),
        ("#78350f", "Перешкода: яр / рельєф"),
    ]
    for idx, (col, text) in enumerate(legend_items):
        yy = pad_top + 52 + idx * 24
        lines.append(f'<rect x="{lx + 14}" y="{yy - 10}" width="12" height="12" rx="2" fill="{col}"/>')
        lines.append(f'<text x="{lx + 34}" y="{yy}" fill="#cbd5e1" font-family="Segoe UI, Arial" font-size="11">{text}</text>')

    stats_y = pad_top + 235
    stats = [
        f'Покриття всього: {result["overall_coverage_pct"]:.1f}%',
        f'Покриття P1: {result["p1_coverage_pct"]:.1f}%',
        f'Траси A*: {result["total_length_m"]:.0f} м',
        f'Пряма (v0.3): {result["total_euclid_m"]:.1f} м',
        f'Резерв: {result["total_reserve_kw"]:.1f} кВт',
        f'CAPEX: {result["total_capex_uah"] / 1000:.1f} тис. грн',
    ]
    for idx, st in enumerate(stats):
        lines.append(f'<text x="{lx + 14}" y="{stats_y + idx * 18}" fill="#e2e8f0" font-family="Segoe UI, Arial" font-size="11">{st}</text>')

    lines.append("</svg>")
    Path(filename).write_text("\n".join(lines), encoding="utf-8")


def print_report(result):
    print("=" * 78)
    print(f"СЦЕНАРІЙ: {result['title']} (коефіцієнт генерації: {int(result['capacity_factor'] * 100)}%)")
    print("=" * 78)
    for c in result["connections"]:
        print(
            f" -> [P{c['priority']}] {c['consumer']} ({c['demand_kw']} кВт) -> {c['generator']}\n"
            f"    Статус: {c['status']} | Довжина A*: {c['length_m']:.0f} м (пряма: {c['euclid_m']:.1f} м)\n"
            f"    Кабель: {c['cable_type']} | Струм: {c['current_a']:.1f} А | "
            f"Втрати dU: {c['voltage_drop_pct']:.2f}% | Оцінка CAPEX: {c['capex_uah']:,.0f} грн"
        )

    if result["warnings"]:
        print("\n[ПОПЕРЕДЖЕННЯ ПРО ДЕФІЦИТ ТА РЕЖИМ МЕРЕЖІ]")
        for w in result["warnings"]:
            print(" !", w)

    print("\nПІДСУМКИ СЦЕНАРІЮ:")
    print(f" - Навантаження / покрито: {result['total_demand_kw']:.0f} кВт / {result['total_supplied_kw']:.1f} кВт ({result['overall_coverage_pct']:.1f}%)")
    print(f" - Покриття критичних об'єктів (1-й пріоритет): {result['p1_coverage_pct']:.1f}%")
    print(f" - Доступна генерація / залишковий резерв: {result['total_capacity_kw']:.1f} кВт / {result['total_reserve_kw']:.1f} кВт")
    print(f" - Сумарна довжина підземних трас (A*): {result['total_length_m']:.0f} м (проти {result['total_euclid_m']:.1f} м по прямій)")
    print(f" - Орієнтовний CAPEX кабельних ліній: {result['total_capex_uah']:,.0f} грн")
    print()


if __name__ == "__main__":
    # Сценарій 1: Штатний "острівний режим" (100% встановленої потужності)
    normal_res = optimize_microgrid(
        CONSUMERS_BASE,
        GENERATORS_BASE,
        OBSTACLES,
        capacity_factor=1.0,
        scenario_title="Штатний острівний режим (100% генерації)",
    )
    print_report(normal_res)
    export_svg_map(normal_res, OBSTACLES, "svitloyarivka_grid_normal.svg")

    # Сценарій 2 (Тест 4): Аварійний дефіцит генерації (-50% потужності станцій)
    deficit_res = optimize_microgrid(
        CONSUMERS_BASE,
        GENERATORS_BASE,
        OBSTACLES,
        capacity_factor=0.5,
        scenario_title="Тест 4: Аварійний дефіцит (-50% генерації)",
    )
    print_report(deficit_res)
    export_svg_map(deficit_res, OBSTACLES, "svitloyarivka_grid_deficit.svg")

    print("Векторні схеми збережено: svitloyarivka_grid_normal.svg та svitloyarivka_grid_deficit.svg")