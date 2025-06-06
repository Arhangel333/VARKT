import krpc
import time
import math

# --- Константы ---
R = 600000  # Радиус Кербина (м)
TARGET_ALT = 1000  # Высота для завершения коррекции (м)
MISS = 0.2

# --- Функции ---

def get_current_thrust(vessel):
    """Возвращает текущую доступную тягу всех активных двигателей (в Н)"""
    return sum(engine.available_thrust 
              for engine in vessel.parts.engines 
              if engine.active and engine.has_fuel)

def get_angular_velocities(vessel, dt=1.0):
    """Возвращает угловые скорости по широте/долготе (град/сек) и вертикальную скорость (м/с)."""
    lat1, lon1 = vessel.flight().latitude, vessel.flight().longitude
    alt1 = vessel.flight().surface_altitude
    
    time.sleep(dt)
    
    lat2, lon2 = vessel.flight().latitude, vessel.flight().longitude
    alt2 = vessel.flight().surface_altitude
    
    ω_lat = (lat2 - lat1) / dt
    ω_lon = (lon2 - lon1) / dt
    V_alt = (alt2 - alt1) / dt
    
    return ω_lat, ω_lon, V_alt

def predict_landing_coords(vessel, ω_lat, ω_lon, V_alt):
    """Прогнозирует координаты приземления."""
    if V_alt >= 0:
        return None, None # Корабль не падает
    
    t = (vessel.flight().surface_altitude - TARGET_ALT) / abs(V_alt)
    pred_lat = vessel.flight().latitude + ω_lat * t
    pred_lon = vessel.flight().longitude + ω_lon * t
    
    return pred_lat, pred_lon

def calculate_required_dV(dcoord, t, lat=0):
    """Переводит изменение координат (град) в dV (м/с)."""
    if abs(dcoord) < 0.01:
        return 0
    
    # Для широты: dV = (dlat * R * π) / (180 * t)
    # Для долготы: dV = (dlon * R * π * cos(lat)) / (180 * t)
    multiplier = (math.pi * R) / (180 * t)
    if abs(lat) > 0.1:  # Для долготы
        multiplier *= math.cos(math.radians(lat))
    
    return dcoord * multiplier

def apply_correction(vessel, dV_lat, dV_lon, thr_pow = 0.5):
    """Применяет корректирующий импульс."""
    ap = vessel.auto_pilot
    control = vessel.control
    
    ap.reference_frame = vessel.surface_reference_frame
    ap.engage()
    heading = 0
    roll_angle = 0
    # Коррекция по широте (крен)
    if abs(dV_lat) > 0.1:
        roll_angle = 1 if dV_lat > 0 else -1  # Крен для смещения
        #ap.target_pitch = roll_angle#ap.target_roll = roll_angle
        #ap.target_direction = (0, roll_angle, 0)
        print(f"\n dV_lat {dV_lat} Попытка коррекции roll_angle {roll_angle}")
        """ start_time = time.time()
        while ap.error > 5:
            if time.time() - start_time > 10:
                raise RuntimeError("Корабль не стабилизировался")
            time.sleep(0.1)
        print("Ориентация достигнута!")  """ # Ждём установки крена
    
    # Коррекция по долготе (рыскание)
    if abs(dV_lon) > 0.1:
        heading = 1 if dV_lon > 0 else -1  # Рыскание для смещения
        print(f"\n dV_lon {dV_lon} Попытка коррекции heading {heading}")
        #ap.target_heading = heading
        #ap.target_direction = (0, 0, heading)
        
    if abs(dV_lon) < 10 or abs(dV_lat) < 10:
        ap.target_direction = (heading, roll_angle, 0)
        print("(heading, roll_angle, 0)\n")
    else:
        print("(heading, roll_angle, heading)\n")
        ap.target_direction = (heading, roll_angle, heading)
    start_time = time.time()
    while ap.error > 5:
        if time.time() - start_time > 20:
            ap.target_direction = (0, 0, 0)
            ap.target_direction = (heading, roll_angle, 0)
        time.sleep(0.5)
    time.sleep(1)
    print("Ориентация достигнута!")
    # Расчёт времени импульса
    mass = vessel.mass
    dV = math.sqrt(dV_lat**2 + dV_lon**2)
    thrust = get_current_thrust(vessel)
    if thrust > 0:
        impulse_time = (mass * dV) / thrust
    else:
        print("Критическая ошибка: thrust = 0")
        return False
    #impulse_time = (mass * dV) / thrust
    

    lat1, lon1, _ = get_angular_velocities(vessel) 
    print(f"Velocitys: lat={lat1:.3f}, lon={lon1:.3f}")
    # Даём импульс
    
    control.throttle = thr_pow
    time.sleep(max(0.05, min(impulse_time, 5)))  # Ограничиваем 5 сек #(impulse_time)
    control.throttle = 0
    lat1, lon1, _ = get_angular_velocities(vessel) 
    print(f"Velocitys: lat={lat1:.3f}, lon={lon1:.3f}")

    # Сброс ориентации
    roll_angle = 0
    heading = 0
    """ap.target_roll = 0
    ap.target_heading = 0
    ap.disengage() """

def check_and_correct(vessel, target_lat, target_lon, max_attempts=20):
    """Проверяет точность и корректирует траекторию."""
    for attempt in range(1, max_attempts + 1):
        print(f"\nПопытка коррекции #{attempt}")
        
        # Получаем текущие скорости
        ω_lat, ω_lon, V_alt = get_angular_velocities(vessel)
        pred_lat, pred_lon = predict_landing_coords(vessel, ω_lat, ω_lon, V_alt)
        
        while pred_lat is None:
            print(f"Ошибка: корабль не падает! {pred_lat}")
            ap = vessel.auto_pilot
            control = vessel.control
            ap.reference_frame = vessel.surface_reference_frame
            ap.engage()
            ap.target_direction = (0, 0, -1)
            start_time = time.time()
            while ap.error > 5:
                if time.time() - start_time > 20:
                    raise RuntimeError("Корабль не стабилизировался")
                time.sleep(0.1)
            print(f"Ориентация достигнута!")
            control.throttle = 0.5
            time.sleep(0.1)
            control.throttle = 0
            ω_lat, ω_lon, V_alt = get_angular_velocities(vessel)
            pred_lat, pred_lon = predict_landing_coords(vessel, ω_lat, ω_lon, V_alt)
        


        # Рассчёт ошибки
        dlat = target_lat - pred_lat
        dlon = target_lon - pred_lon
        t = (vessel.flight().surface_altitude - TARGET_ALT) / abs(V_alt)
        
        print(f"Промах: lat={dlat:.3f}°, lon={dlon:.3f}° --------------------")
        
        if abs(dlat) < MISS and abs(dlon) < MISS:
            print("Точность достигнута!")
            return True
        
        # Рассчёт требуемых dV
        dV_lat = calculate_required_dV(dlat, t)
        dV_lon = calculate_required_dV(dlon, t, vessel.flight().latitude)
        
        # Применяем коррекцию
        
        apply_correction(vessel, dV_lat, dV_lon, 0.5)
    
    print("Достигнут лимит попыток.")
    return False

# --- Главная функция ---
def main():
    conn = krpc.connect(name="Precision Landing")
    vessel = conn.space_center.active_vessel
    
    # Целевые координаты (KSC: -0.096944, -74.5575)
    target_lat = -0.096944 #float(input("Введите целевую широту: "))
    target_lon = -74.5575 #float(input("Введите целевую долготу: "))

    target_mis1 = 0 #погрешность из-за атмосферы и гравитации
    target_mis2 = -0.4977200650055238

    target_lat = target_lat + target_mis1
    target_lon = target_lon + target_mis2

    lat1, lon1 = vessel.flight().latitude, vessel.flight().longitude
    dlat = lat1 - target_lat
    dlon = lon1 - target_lon
    """ if abs(dlat) < 60 and abs(dlon) < 60:
        print(f"\nПерелёт ждём точки входа")
        while abs(dlat) < 60 and abs(dlon) < 60:
            lat1, lon1 = vessel.flight().latitude, vessel.flight().longitude
            dlat = lat1 - target_lat
            dlon = lon1 - target_lon
            print(f"Коорд: lat={lat1:.3f}, lon={lon1:.3f} , dlat {dlat}, dlon {dlon}")
 """
    while abs(dlat) > 60 or abs(dlon) > 60:
        lat1, lon1 = vessel.flight().latitude, vessel.flight().longitude
        dlat = lat1 - target_lat
        dlon = lon1 - target_lon
        time.sleep(0.1)
        print(f"Coords: lat={lat1:.3f}, lon={lon1:.3f} , dlat {dlat}, dlon {dlon}")
    print(f"Coords: lat={lat1:.3f}, lon={lon1:.3f} , dlat {dlat}, dlon {dlon}")
    print("\nНачинаем коррекцию траектории...")
    vessel.control.sas = False
    time.sleep(1)
    vessel.control.sas = True
    success = check_and_correct(vessel, target_lat, target_lon)
    vessel.control.sas = False

    if success:
        print("Корабль приземлится в заданной точке!")
    else:
        print("Требуется ручная коррекция.")
    ap = vessel.auto_pilot
    control = vessel.control
    ap.reference_frame = vessel.surface_reference_frame
    ap.engage()
    vessel.control.sas = False
    time.sleep(1)
    vessel.control.sas = True
    ap.target_direction = (0, 0, -1)
    start_time = time.time()
    while ap.error > 5:
        if time.time() - start_time > 10:
            raise RuntimeError("Корабль не стабилизировался")
        time.sleep(0.1)
    print(f"Ориентация для входа в атмосферу и приземления достигнута!")
    while vessel.flight().surface_altitude > 1500:
        ap.target_direction = (0, 0, -1)
        time.sleep(1)
    vessel.control.activate_next_stage()
    vessel.control.sas = False
    ap.disengage()
    h = vessel.flight().surface_altitude
    time.sleep(5)
    while vessel.flight().surface_altitude - h > 1:
        print(f"Снижение... {h} {vessel.flight().surface_altitude}")
        h = vessel.flight().surface_altitude
        time.sleep(2)
    print(f"\rСнижение... {h}")
    lat1, lon1 = vessel.flight().latitude, vessel.flight().longitude
    alt1 = vessel.flight().surface_altitude
    print(f"Снижение завершено координаты приземления {lat1:3f}° {lon1:3f}°")

if __name__ == "__main__":
    main()
