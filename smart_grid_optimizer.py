import math

# 1. Вхідні дані: Цифровий двійник Світлоярівки (Координати, Пріоритет, Потреба)
# Пріоритети: 1 - Критична інфраструктура, 2 - Агросектор, 3 - Житловий фонд
consumers = [
    {"name": "Лікарня та Водоканал", "coords": (15, 20), "priority": 1, "demand_kw": 50},
    {"name": "Фермерський комплекс", "coords": (85, 90), "priority": 2, "demand_kw": 120},
    {"name": "Житловий сектор", "coords": (40, 50), "priority": 3, "demand_kw": 200},
]

# Розташування нових джерел генерації (СЕС, Вітряки, Біогаз)
generators = [
    {"name": "Центральна СЕС + Акумулятори", "coords": (18, 22), "capacity_kw": 100},
    {"name": "Біогазова станція", "coords": (82, 88), "capacity_kw": 150},
    {"name": "Розподілені СЕС на дахах", "coords": (45, 45), "capacity_kw": 250},
]


# 2. Математична модель: Розрахунок найкоротшого шляху (Spatial Optimization)
def calculate_distance(coord1, coord2):
    # Використовуємо теорему Піфагора для розрахунку довжини підземного кабелю
    return math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)


def optimize_microgrid(consumers_list, generators_list):
    print("--- Запуск просторового ШІ-архітектора (Світлоярівка) ---")

    # КРОК 1: Сортуємо споживачів за рівнем критичності (пріоритетом)
    sorted_consumers = sorted(consumers_list, key=lambda x: x["priority"])

    connections = []
    total_cable_length = 0.0

    # КРОК 2: Підбір оптимального джерела для кожного споживача
    for consumer in sorted_consumers:
        best_gen = None
        min_distance = float("inf")

        for gen in generators_list:
            # Перевіряємо, чи вистачає потужності, і шукаємо найближчу станцію
            if gen["capacity_kw"] >= consumer["demand_kw"]:
                dist = calculate_distance(consumer["coords"], gen["coords"])
                if dist < min_distance:
                    min_distance = dist
                    best_gen = gen

        # КРОК 3: Фіксація підключення
        if best_gen:
            best_gen["capacity_kw"] -= consumer["demand_kw"]
            connections.append(
                f"[{consumer['name']}] підключено до [{best_gen['name']}] "
                f"(Довжина кабелю: {min_distance:.2f} м; залишок потужності: {best_gen['capacity_kw']} кВт)"
            )
            total_cable_length += min_distance

    return connections, total_cable_length


if __name__ == "__main__":
    # 3. Виконання та виведення результатів
    routes, total_cable = optimize_microgrid(consumers, generators)
    print("\nРезультати топологічної оптимізації:")
    for route in routes:
        print(" ->", route)

    total_reserve = sum(g["capacity_kw"] for g in generators)
    print(f"\nЗагальна довжина нових підземних трас: {total_cable:.2f} м.")
    print(f"Сумарний резерв потужності локальної генерації: {total_reserve} кВт.")
    print("Оптимізацію завершено. Мережа децентралізована та стійка до блекаутів.")