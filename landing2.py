import krpc
import time
import math

# --- Константы ---
R = 600000  # Радиус Кербина (м)
TARGET_ALT = 1000  # Высота для завершения коррекции (м)
THRUST = 50000  # Тяга двигателя (Н)
EFFICIENCY = 0.8  # КПД коррекции

# --- Функции ---
def get_angular_velocities(vessel, dt=1.0):
    """Возвращает угловые скорости по широте/долготе (град/сек) и вертикальную скорость (м/с)."""
    lat1, lon1 = vessel.latitude, vessel.longitude
    alt1 = vessel.flight().surface_altitude
    
    time.sleep(dt)
    
    lat2, lon2 = vessel.latitude, vessel.longitude
    alt2 = vessel.flight().surface_altitude
    
    ω_lat = (lat2 - lat1) / dt
    ω_lon = (lon2 - lon1) / dt
    V_alt = (alt2 - alt1) / dt
    
    return ω_lat, ω_lon, V_alt

def predict_landing_coords(vessel, ω_lat, ω_lon, V_alt):
    """Прогнозирует координаты приземления."""
    if V_alt >= 0:
        return None  # Корабль не падает
    
    t = (vessel.flight().surface_altitude - TARGET_ALT) / abs(V_alt)
    pred_lat = vessel.latitude + ω_lat * t
    pred_lon = vessel.longitude + ω_lon * t
    
    return pred_lat, pred_lon

def calculate_required_ΔV(Δcoord, t, lat=0):
    """Переводит изменение координат (град) в ΔV (м/с)."""
    if abs(Δcoord) < 0.01:
        return 0
    
    # Для широты: ΔV = (Δlat * R * π) / (180 * t)
    # Для долготы: ΔV = (Δlon * R * π * cos(lat)) / (180 * t)
    multiplier = (math.pi * R) / (180 * t)
    if abs(lat) > 0.1:  # Для долготы
        multiplier *= math.cos(math.radians(lat))
    
    return Δcoord * multiplier

def apply_correction(vessel, ΔV_lat, ΔV_lon):
    """Применяет корректирующий импульс."""
    ap = vessel.auto_pilot
    control = vessel.control
    
    ap.reference_frame = vessel.surface_reference_frame
    ap.engage()
    
    # Коррекция по широте (крен)
    if abs(ΔV_lat) > 0.1:
        roll_angle = -30 if ΔV_lat > 0 else 30  # Крен для смещения
        ap.target_roll = roll_angle
        time.sleep(1)  # Ждём установки крена
    
    # Коррекция по долготе (рыскание)
    if abs(ΔV_lon) > 0.1:
        heading = 90 if ΔV_lon > 0 else 270  # Рыскание для смещения
        ap.target_heading = heading
        time.sleep(1)
    
    # Расчёт времени импульса
    mass = vessel.mass
    ΔV = math.sqrt(ΔV_lat**2 + ΔV_lon**2)
    impulse_time = (mass * ΔV) / (THRUST * EFFICIENCY)
    
    # Даём импульс
    control.throttle = 0.5
    time.sleep(max(0.1, min(impulse_time, 5)))  # Ограничиваем 5 сек
    control.throttle = 0
    
    # Сброс ориентации
    ap.target_roll = 0
    ap.target_heading = 0
    ap.disengage()

def check_and_correct(vessel, target_lat, target_lon, max_attempts=2):
    """Проверяет точность и корректирует траекторию."""
    for attempt in range(1, max_attempts + 1):
        print(f"\nПопытка коррекции #{attempt}")
        
        # Получаем текущие скорости
        ω_lat, ω_lon, V_alt = get_angular_velocities(vessel)
        pred_lat, pred_lon = predict_landing_coords(vessel, ω_lat, ω_lon, V_alt)
        
        if pred_lat is None:
            print("Ошибка: корабль не падает!")
            return False
        
        # Рассчёт ошибки
        Δlat = target_lat - pred_lat
        Δlon = target_lon - pred_lon
        t = (vessel.flight().surface_altitude - TARGET_ALT) / abs(V_alt)
        
        print(f"Ошибка: lat={Δlat:.3f}°, lon={Δlon:.3f}°")
        
        if abs(Δlat) < 0.02 and abs(Δlon) < 0.02:
            print("Точность достигнута!")
            return True
        
        # Рассчёт требуемых ΔV
        ΔV_lat = calculate_required_ΔV(Δlat, t)
        ΔV_lon = calculate_required_ΔV(Δlon, t, vessel.latitude)
        
        # Применяем коррекцию
        print(f"Коррекция: ΔV_lat={ΔV_lat:.1f} м/с, ΔV_lon={ΔV_lon:.1f} м/с")
        apply_correction(vessel, ΔV_lat, ΔV_lon)
    
    print("Достигнут лимит попыток.")
    return False

# --- Главная функция ---
def main():
    conn = krpc.connect(name="Precision Landing")
    vessel = conn.space_center.active_vessel
    
    # Целевые координаты (KSC: -0.096944, -74.5575)
    target_lat = float(input("Введите целевую широту: "))
    target_lon = float(input("Введите целевую долготу: "))
    
    print("\nНачинаем коррекцию траектории...")
    success = check_and_correct(vessel, target_lat, target_lon)
    
    if success:
        print("Корабль приземлится в заданной точке!")
    else:
        print("Требуется ручная коррекция.")

if __name__ == "__main__":
    main()
