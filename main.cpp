#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <string>
#include <memory>
#include <map>

using namespace std;

// ============================ КОНСТАНТЫ И ПАРАМЕТРЫ ============================
struct AircraftParams {
    double mass = 13900.0;           // Масса самолета, кг
    double wing_area = 70.0;         // Площадь крыла, м²
    int engine_count = 3;            // Количество двигателей
    double thrust_percent = 100.0;   // Режим работы двигателей, %
    
    // Коэффициенты аэродинамики
    double cy0 = -0.08;
    double cy1 = 0.075;
    double cx0 = 0.022;
    double k = 0.035;
    
    // Ограничения
    double max_vertical_speed = 10.0;     // м/с
    double max_climb_angle = 15.0;        // градусы
    double min_climb_speed_kmh = 320.0;   // км/ч
};

struct FlightConditions {
    double start_altitude = 400.0;        // Начальная высота, м
    double end_altitude = 6100.0;         // Конечная высота, м
    double start_speed_kmh = 260.0;       // Начальная скорость, км/ч
    double end_speed_kmh = 860.0;         // Конечная скорость, км/ч
    
    int grid_points = 10;                 // Количество точек разбиения
};

// ============================ СИСТЕМА АТМОСФЕРЫ ============================
class Atmosphere {
private:
    struct AtmospherePoint {
        double altitude;   // м
        double density;    // кг/м³
        double sound_speed;// м/с
        double temperature;// K
    };
    
    vector<AtmospherePoint> table;
    
public:
    Atmosphere() {
        // Таблица атмосферы (ИСА)
        table = {
            {0.0,     1.22500, 340.294, 288.150},
            {500.0,   1.16727, 338.370, 284.900},
            {1000.0,  1.11166, 336.435, 281.651},
            {2000.0,  1.00655, 332.532, 275.154},
            {3000.0,  0.90254, 328.584, 268.659},
            {4000.0,  0.81935, 324.589, 262.166},
            {5000.0,  0.73643, 320.545, 255.676},
            {6000.0,  0.66011, 316.452, 249.187},
            {7000.0,  0.59002, 312.306, 242.700},
            {8000.0,  0.52678, 308.105, 236.215},
            {9000.0,  0.46706, 303.848, 229.733},
            {10000.0, 0.41351, 299.532, 223.252},
            {11000.0, 0.36480, 295.154, 216.774}
        };
    }
    
    void getConditions(double altitude, double& density, double& sound_speed) const {
        if (altitude <= table.front().altitude) {
            density = table.front().density;
            sound_speed = table.front().sound_speed;
            return;
        }
        
        if (altitude >= table.back().altitude) {
            density = table.back().density;
            sound_speed = table.back().sound_speed;
            return;
        }
        
        for (size_t i = 0; i < table.size() - 1; i++) {
            if (altitude >= table[i].altitude && altitude < table[i+1].altitude) {
                double t = (altitude - table[i].altitude) / 
                          (table[i+1].altitude - table[i].altitude);
                density = table[i].density + t * (table[i+1].density - table[i].density);
                sound_speed = table[i].sound_speed + t * (table[i+1].sound_speed - table[i].sound_speed);
                return;
            }
        }
    }
    
    double getMachNumber(double altitude, double speed_ms) const {
        double density, sound_speed;
        getConditions(altitude, density, sound_speed);
        return speed_ms / sound_speed;
    }
};

// ============================ АЭРОДИНАМИЧЕСКАЯ МОДЕЛЬ ============================
class AerodynamicModel {
private:
    AircraftParams params;
    
public:
    AerodynamicModel(const AircraftParams& p) : params(p) {}
    
    double calculateLiftCoefficient(double angle_of_attack_deg) const {
        // Упрощенная поляра крыла
        angle_of_attack_deg = max(0.0, min(10.0, angle_of_attack_deg));
        return params.cy0 + params.cy1 * angle_of_attack_deg;
    }
    
    double calculateDragCoefficient(double angle_of_attack_deg) const {
        // Квадратичная зависимость Cx от Cy
        double cy = calculateLiftCoefficient(angle_of_attack_deg);
        return params.cx0 + params.k * cy * cy;
    }
    
    double calculateRequiredAngleOfAttack(double altitude, double speed_ms, 
                                         double mass) const {
        const double g = 9.81;
        const double deg_to_rad = 57.3;
        
        Atmosphere atm;
        double density, sound_speed;
        atm.getConditions(altitude, density, sound_speed);
        
        double dynamic_pressure = 0.5 * density * speed_ms * speed_ms;
        
        if (dynamic_pressure < 100.0) return 8.0;
        
        // Уравнение баланса сил по вертикали
        double required_cy = (mass * g) / (dynamic_pressure * params.wing_area);
        
        // Вычисляем угол атаки из линейной зависимости Cy(alpha)
        double alpha_deg = (required_cy - params.cy0) / params.cy1;
        
        return max(0.0, min(10.0, alpha_deg));
    }
};

// ============================ МОДЕЛЬ ДВИГАТЕЛЯ ============================
class EngineModel {
private:
    AircraftParams params;
    
public:
    EngineModel(const AircraftParams& p) : params(p) {}
    
    double calculateThrustPerEngine(double altitude, double mach) const {
        // Номинальная тяга на уровне моря
        double sea_level_thrust = 58860.0; // Н
        
        // Коррекция по высоте
        double altitude_km = altitude / 1000.0;
        double altitude_factor;
        
        if (altitude_km <= 0) {
            altitude_factor = 1.0;
        } else if (altitude_km >= 11.0) {
            altitude_factor = 0.50;
        } else {
            altitude_factor = 1.0 - 0.50 * pow(altitude_km / 11.0, 0.7);
        }
        
        // Коррекция по числу Маха
        double mach_factor = 0.88 + 0.24 * mach;
        mach_factor = min(mach_factor, 1.08);
        
        return sea_level_thrust * altitude_factor * mach_factor;
    }
    
    double calculateTotalThrust(double altitude, double speed_ms) const {
        Atmosphere atm;
        double mach = atm.getMachNumber(altitude, speed_ms);
        
        double thrust_per_engine = calculateThrustPerEngine(altitude, mach);
        return thrust_per_engine * params.engine_count * (params.thrust_percent / 100.0);
    }
    
    double calculateFuelConsumption(double altitude, double speed_ms, 
                                   double thrust_setting) const {
        // Удельный расход топлива, кг/(Н·ч)
        double base_consumption = 0.72 / 9.81; // пересчет в правильные единицы
        
        Atmosphere atm;
        double mach = atm.getMachNumber(altitude, speed_ms);
        double altitude_km = altitude / 1000.0;
        
        // Коррекция по режиму работы
        double regime_factor = 1.0;
        if (thrust_setting >= 1.0) {
            regime_factor = 1.0 + 0.45 * pow(thrust_setting - 1.0, 1.2);
        } else if (thrust_setting >= 0.88) {
            regime_factor = 0.92 - 0.02 * (thrust_setting - 0.88) / 0.12;
        } else if (thrust_setting >= 0.70) {
            regime_factor = 0.92 + 0.12 * pow((0.88 - thrust_setting) / 0.18, 1.1);
        } else {
            regime_factor = 1.18;
        }
        
        // Коррекция по высоте
        double altitude_factor = 1.0 - 0.07 * min(1.0, altitude_km / 11.0);
        
        // Коррекция по числу Маха
        double mach_factor = 1.0 + 0.14 * max(0.0, mach - 0.5);
        
        return base_consumption * regime_factor * altitude_factor * mach_factor;
    }
};

// ============================ МАНЕВРЫ ПОЛЕТА ============================
enum class ManeuverType {
    ACCELERATION,      // Разгон
    CLIMB,             // Подъем
    COMBINED           // Комбинированный
};

struct ManeuverResult {
    bool valid;
    double time;        // с
    double fuel;        // кг
    double distance;    // м (опционально)
    
    ManeuverResult() : valid(false), time(0), fuel(0), distance(0) {}
};

class FlightManeuver {
protected:
    AircraftParams aircraft_params;
    AerodynamicModel aero_model;
    EngineModel engine_model;
    Atmosphere atmosphere;
    
public:
    FlightManeuver(const AircraftParams& params) 
        : aircraft_params(params), 
          aero_model(params), 
          engine_model(params) {}
    
    virtual ManeuverResult calculate(double start_alt, double end_alt,
                                    double start_speed_ms, double end_speed_ms,
                                    double mass, double thrust_setting) = 0;
    
    virtual ~FlightManeuver() {}
};

class AccelerationManeuver : public FlightManeuver {
public:
    AccelerationManeuver(const AircraftParams& params) : FlightManeuver(params) {}
    
    ManeuverResult calculate(double start_alt, double end_alt,
                            double start_speed_ms, double end_speed_ms,
                            double mass, double thrust_setting) override {
        ManeuverResult result;
        
        // Для разгона высота постоянна
        if (fabs(start_alt - end_alt) > 1.0) {
            return result; // Невалидный маневр
        }
        
        double altitude = start_alt;
        double avg_speed_ms = 0.5 * (start_speed_ms + end_speed_ms);
        
        // Проверка на валидность условий
        if (avg_speed_ms * 3.6 < aircraft_params.min_climb_speed_kmh * 0.8) {
            return result;
        }
        
        // Угол атаки для горизонтального полета
        double angle_of_attack = aero_model.calculateRequiredAngleOfAttack(
            altitude, avg_speed_ms, mass);
        
        // Полная тяга
        double total_thrust = engine_model.calculateTotalThrust(altitude, avg_speed_ms);
        double used_thrust = total_thrust * thrust_setting;
        
        // Аэродинамические силы
        double density, sound_speed;
        atmosphere.getConditions(altitude, density, sound_speed);
        double dynamic_pressure = 0.5 * density * avg_speed_ms * avg_speed_ms;
        
        double drag_coef = aero_model.calculateDragCoefficient(angle_of_attack);
        double drag_force = drag_coef * dynamic_pressure * aircraft_params.wing_area;
        
        // Ускорение
        double acceleration = (used_thrust - drag_force) / mass;
        
        if (acceleration <= 0.01) {
            return result; // Недостаточная тяга
        }
        
        // Время разгона
        result.time = (end_speed_ms - start_speed_ms) / acceleration;
        
        if (result.time <= 0 || result.time > 1000.0) {
            return result;
        }
        
        // Расход топлива
        double fuel_consumption = engine_model.calculateFuelConsumption(
            altitude, avg_speed_ms, thrust_setting);
        result.fuel = fuel_consumption * used_thrust * result.time / 3600.0;
        
        // Расстояние
        result.distance = avg_speed_ms * result.time;
        
        result.valid = true;
        return result;
    }
};

class ClimbManeuver : public FlightManeuver {
private:
    double max_vertical_speed_factor;
    
public:
    ClimbManeuver(const AircraftParams& params, double factor = 1.0) 
        : FlightManeuver(params), max_vertical_speed_factor(factor) {}
    
    ManeuverResult calculate(double start_alt, double end_alt,
                            double start_speed_ms, double end_speed_ms,
                            double mass, double thrust_setting) override {
        ManeuverResult result;
        
        // Для подъема скорость постоянна
        if (fabs(start_speed_ms - end_speed_ms) > 0.1) {
            return result;
        }
        
        double speed_ms = start_speed_ms;
        double avg_altitude = 0.5 * (start_alt + end_alt);
        
        if (speed_ms * 3.6 < aircraft_params.min_climb_speed_kmh) {
            return result;
        }
        
        // Угол атаки
        double angle_of_attack = aero_model.calculateRequiredAngleOfAttack(
            avg_altitude, speed_ms, mass);
        
        // Тяга
        double total_thrust = engine_model.calculateTotalThrust(avg_altitude, speed_ms);
        double used_thrust = total_thrust * thrust_setting;
        
        // Аэродинамические силы
        double density, sound_speed;
        atmosphere.getConditions(avg_altitude, density, sound_speed);
        double dynamic_pressure = 0.5 * density * speed_ms * speed_ms;
        
        double drag_coef = aero_model.calculateDragCoefficient(angle_of_attack);
        double drag_force = drag_coef * dynamic_pressure * aircraft_params.wing_area;
        
        // Избыточная тяга для набора высоты
        double excess_thrust = used_thrust - drag_force;
        
        if (excess_thrust <= 0) {
            return result;
        }
        
        // Угол набора высоты
        const double deg_to_rad = M_PI / 180.0;
        double max_climb_angle_rad = aircraft_params.max_climb_angle * deg_to_rad;
        double sin_theta = min(excess_thrust / (mass * 9.81), sin(max_climb_angle_rad));
        
        if (sin_theta <= 0.005) {
            return result;
        }
        
        // Вертикальная скорость
        double vertical_speed = speed_ms * sin_theta;
        double max_vertical_limit = aircraft_params.max_vertical_speed * max_vertical_speed_factor;
        
        if (vertical_speed > max_vertical_limit) {
            vertical_speed = max_vertical_limit;
            sin_theta = vertical_speed / speed_ms;
        }
        
        // Время набора высоты
        result.time = (end_alt - start_alt) / vertical_speed;
        
        if (result.time <= 0 || result.time > 2000.0) {
            return result;
        }
        
        // Расход топлива
        double fuel_consumption = engine_model.calculateFuelConsumption(
            avg_altitude, speed_ms, thrust_setting);
        result.fuel = fuel_consumption * used_thrust * result.time / 3600.0;
        
        // Горизонтальная проекция пути
        double cos_theta = sqrt(1.0 - sin_theta * sin_theta);
        result.distance = speed_ms * cos_theta * result.time;
        
        result.valid = true;
        return result;
    }
};

// ============================ СИСТЕМА ОПТИМИЗАЦИИ ============================
class TrajectoryOptimizer {
private:
    AircraftParams aircraft;
    FlightConditions conditions;
    
    vector<double> altitude_grid;
    vector<double> speed_grid_kmh;
    vector<double> speed_grid_ms;
    
    map<pair<int, int>, double> best_cost;
    map<pair<int, int>, pair<int, int>> predecessor;
    map<pair<int, int>, ManeuverType> maneuver_used;
    map<pair<int, int>, double> segment_time;
    map<pair<int, int>, double> segment_fuel;
    
    void createGrid() {
        int n = conditions.grid_points;
        
        double dh = (conditions.end_altitude - conditions.start_altitude) / n;
        double dv_kmh = (conditions.end_speed_kmh - conditions.start_speed_kmh) / n;
        
        altitude_grid.resize(n + 1);
        speed_grid_kmh.resize(n + 1);
        speed_grid_ms.resize(n + 1);
        
        for (int i = 0; i <= n; i++) {
            altitude_grid[i] = conditions.start_altitude + i * dh;
            speed_grid_kmh[i] = conditions.start_speed_kmh + i * dv_kmh;
            speed_grid_ms[i] = speed_grid_kmh[i] / 3.6;
        }
    }
    
public:
    TrajectoryOptimizer(const AircraftParams& ac, const FlightConditions& fc) 
        : aircraft(ac), conditions(fc) {
        createGrid();
    }
    
    struct TrajectorySolution {
        vector<pair<double, double>> path;  // (высота, скорость)
        vector<ManeuverType> maneuvers;
        vector<double> times;
        vector<double> fuels;
        vector<double> distances;
        double total_time;
        double total_fuel;
        double total_distance;
        
        TrajectorySolution() : total_time(0), total_fuel(0), total_distance(0) {}
    };
    
    TrajectorySolution findOptimalPath(bool minimize_time, 
                                      const vector<double>& thrust_settings,
                                      double vertical_speed_factor) {
        int n = conditions.grid_points;
        
        // Инициализация
        best_cost.clear();
        predecessor.clear();
        maneuver_used.clear();
        segment_time.clear();
        segment_fuel.clear();
        
        best_cost[{0, 0}] = 0.0;
        
        AccelerationManeuver accel_maneuver(aircraft);
        ClimbManeuver climb_maneuver(aircraft, vertical_speed_factor);
        
        // Динамическое программирование
        for (int i = 0; i <= n; i++) {
            for (int j = 0; j <= n; j++) {
                if (best_cost.find({i, j}) == best_cost.end()) continue;
                
                double current_cost = best_cost[{i, j}];
                double current_alt = altitude_grid[i];
                double current_speed_ms = speed_grid_ms[j];
                
                // Перебор режимов тяги
                for (double thrust_setting : thrust_settings) {
                    // Разгон (увеличение скорости)
                    if (j < n) {
                        double next_speed_ms = speed_grid_ms[j + 1];
                        ManeuverResult result = accel_maneuver.calculate(
                            current_alt, current_alt,
                            current_speed_ms, next_speed_ms,
                            aircraft.mass, thrust_setting);
                        
                        if (result.valid) {
                            double incremental_cost = minimize_time ? result.time : result.fuel;
                            double new_cost = current_cost + incremental_cost;
                            
                            pair<int, int> next_state = {i, j + 1};
                            
                            if (best_cost.find(next_state) == best_cost.end() || 
                                new_cost < best_cost[next_state]) {
                                best_cost[next_state] = new_cost;
                                predecessor[next_state] = {i, j};
                                maneuver_used[next_state] = ManeuverType::ACCELERATION;
                                segment_time[next_state] = result.time;
                                segment_fuel[next_state] = result.fuel;
                            }
                        }
                    }
                    
                    // Подъем (увеличение высоты)
                    if (i < n) {
                        double next_alt = altitude_grid[i + 1];
                        ManeuverResult result = climb_maneuver.calculate(
                            current_alt, next_alt,
                            current_speed_ms, current_speed_ms,
                            aircraft.mass, thrust_setting);
                        
                        if (result.valid) {
                            double incremental_cost = minimize_time ? result.time : result.fuel;
                            double new_cost = current_cost + incremental_cost;
                            
                            pair<int, int> next_state = {i + 1, j};
                            
                            if (best_cost.find(next_state) == best_cost.end() || 
                                new_cost < best_cost[next_state]) {
                                best_cost[next_state] = new_cost;
                                predecessor[next_state] = {i, j};
                                maneuver_used[next_state] = ManeuverType::CLIMB;
                                segment_time[next_state] = result.time;
                                segment_fuel[next_state] = result.fuel;
                            }
                        }
                    }
                }
            }
        }
        
        // Восстановление пути
        TrajectorySolution solution;
        
        if (best_cost.find({n, n}) == best_cost.end()) {
            cerr << "Не удалось найти путь!" << endl;
            return solution;
        }
        
        // Восстанавливаем путь от конечной точки
        pair<int, int> current = {n, n};
        
        while (true) {
            solution.path.push_back({altitude_grid[current.first], 
                                    speed_grid_kmh[current.second]});
            
            if (current == make_pair(0, 0)) break;
            
            pair<int, int> prev = predecessor[current];
            solution.maneuvers.push_back(maneuver_used[current]);
            solution.times.push_back(segment_time[current]);
            solution.fuels.push_back(segment_fuel[current]);
            
            current = prev;
        }
        
        // Реверсируем, так как восстанавливали с конца
        reverse(solution.path.begin(), solution.path.end());
        reverse(solution.maneuvers.begin(), solution.maneuvers.end());
        reverse(solution.times.begin(), solution.times.end());
        reverse(solution.fuels.begin(), solution.fuels.end());
        
        // Суммируем
        for (double t : solution.times) solution.total_time += t;
        for (double f : solution.fuels) solution.total_fuel += f;
        
        return solution;
    }
    
    void printMatrices() const {
        int n = conditions.grid_points;
        
        cout << "\n=== МАТРИЦА ВРЕМЕНИ (с) ===\n";
        cout << "      V->";
        for (int j = 0; j <= n; j++) {
            cout << setw(8) << fixed << setprecision(0) << speed_grid_kmh[j];
        }
        cout << "\nH\n";
        
        for (int i = 0; i <= n; i++) {
            cout << setw(6) << fixed << setprecision(0) << altitude_grid[i];
            for (int j = 0; j <= n; j++) {
                pair<int, int> state = {i, j};
                if (best_cost.find(state) != best_cost.end()) {
                    // Для матрицы времени нам нужны накопленные времена
                    // Упрощенный вывод - можно доработать
                    cout << setw(8) << fixed << setprecision(0) << "X";
                } else {
                    cout << setw(8) << "---";
                }
            }
            cout << endl;
        }
    }
};

// ============================ ИНТЕРФЕЙС ПОЛЬЗОВАТЕЛЯ ============================
class FlightPlanner {
private:
    AircraftParams aircraft;
    FlightConditions conditions;
    
public:
    FlightPlanner() {
        // Параметры по умолчанию
    }
    
    void run() {
        cout << "=================================================\n";
        cout << "   ПЛАНИРОВЩИК ПОЛЕТА ЯК-40 (МОДИФИЦИРОВАННЫЙ)\n";
        cout << "=================================================\n";
        cout << "Самолет: Як-40 (масса " << aircraft.mass / 1000.0 << " т)\n";
        cout << "Двигатели: 3 х АИ-95 (" << aircraft.thrust_percent << "% номинала)\n";
        cout << "Старт: H = " << conditions.start_altitude << " м, V = " 
             << conditions.start_speed_kmh << " км/ч\n";
        cout << "Финиш: H = " << conditions.end_altitude << " м, V = " 
             << conditions.end_speed_kmh << " км/ч\n";
        cout << "=================================================\n\n";
        
        int choice;
        cout << "Выберите критерий оптимизации:\n";
        cout << "1 - Минимизация времени полета\n";
        cout << "2 - Минимизация расхода топлива\n";
        cout << "3 - Сравнение обоих вариантов\n";
        cout << "Ваш выбор: ";
        cin >> choice;
        
        switch (choice) {
            case 1:
                optimizeForTime();
                break;
            case 2:
                optimizeForFuel();
                break;
            case 3:
                compareBoth();
                break;
            default:
                cout << "Неверный выбор!\n";
                break;
        }
    }
    
private:
    void optimizeForTime() {
        cout << "\n=== ОПТИМИЗАЦИЯ ПО ВРЕМЕНИ ===\n";
        
        vector<double> thrust_settings = {1.08, 1.05, 1.00}; // Максимальные режимы
        double vertical_factor = 1.0;
        
        TrajectoryOptimizer optimizer(aircraft, conditions);
        auto solution = optimizer.findOptimalPath(true, thrust_settings, vertical_factor);
        
        printSolution(solution, "Минимизация времени");
        saveSolutionToFile(solution, "time_optimal");
        optimizer.printMatrices();
    }
    
    void optimizeForFuel() {
        cout << "\n=== ОПТИМИЗАЦИЯ ПО ТОПЛИВУ ===\n";
        
        vector<double> thrust_settings = {0.88, 0.82, 0.75}; // Экономичные режимы
        double vertical_factor = 0.65;
        
        TrajectoryOptimizer optimizer(aircraft, conditions);
        auto solution = optimizer.findOptimalPath(false, thrust_settings, vertical_factor);
        
        printSolution(solution, "Минимизация топлива");
        saveSolutionToFile(solution, "fuel_optimal");
        optimizer.printMatrices();
    }
    
    void compareBoth() {
        cout << "\n=== СРАВНЕНИЕ ВАРИАНТОВ ===\n";
        
        // Оптимизация по времени
        vector<double> time_settings = {1.08, 1.05, 1.00};
        TrajectoryOptimizer optimizer_time(aircraft, conditions);
        auto solution_time = optimizer_time.findOptimalPath(true, time_settings, 1.0);
        
        // Оптимизация по топливу
        vector<double> fuel_settings = {0.88, 0.82, 0.75};
        TrajectoryOptimizer optimizer_fuel(aircraft, conditions);
        auto solution_fuel = optimizer_fuel.findOptimalPath(false, fuel_settings, 0.65);
        
        cout << "\n--- РЕЗУЛЬТАТЫ СРАВНЕНИЯ ---\n";
        cout << "Критерий          Время (мин)   Топливо (кг)   Vy ср (м/с)\n";
        cout << "----------------------------------------------------------\n";
        
        double avg_vy_time = (conditions.end_altitude - conditions.start_altitude) 
                           / solution_time.total_time;
        double avg_vy_fuel = (conditions.end_altitude - conditions.start_altitude) 
                           / solution_fuel.total_time;
        
        cout << fixed << setprecision(1);
        cout << "Минимум времени:  " << setw(10) << solution_time.total_time / 60.0
             << setw(15) << solution_time.total_fuel
             << setw(15) << avg_vy_time << endl;
        
        cout << "Минимум топлива:  " << setw(10) << solution_fuel.total_time / 60.0
             << setw(15) << solution_fuel.total_fuel
             << setw(15) << avg_vy_fuel << endl;
        
        // Сохраняем оба решения
        saveSolutionToFile(solution_time, "time_comparison");
        saveSolutionToFile(solution_fuel, "fuel_comparison");
        
        // Создаем скрипт для GNUPLOT
        createComparisonPlot(solution_time, solution_fuel);
    }
    
    void printSolution(const TrajectoryOptimizer::TrajectorySolution& solution, 
                      const string& title) {
        cout << "\n=== " << title << " ===\n";
        cout << "Всего точек: " << solution.path.size() << endl;
        cout << "Общее время: " << solution.total_time << " с (" 
             << solution.total_time / 60.0 << " мин)\n";
        cout << "Расход топлива: " << solution.total_fuel << " кг\n";
        
        int accel_count = 0, climb_count = 0;
        for (auto m : solution.maneuvers) {
            if (m == ManeuverType::ACCELERATION) accel_count++;
            else if (m == ManeuverType::CLIMB) climb_count++;
        }
        
        cout << "Маневры: Разгон - " << accel_count 
             << ", Подъем - " << climb_count << endl;
        
        cout << "\nТраектория:\n";
        cout << "№   Высота (м)   Скорость (км/ч)   Маневр\n";
        cout << "------------------------------------------\n";
        
        for (size_t i = 0; i < solution.path.size(); i++) {
            cout << setw(2) << i + 1 << "  "
                 << setw(10) << fixed << setprecision(0) << solution.path[i].first
                 << setw(15) << fixed << setprecision(0) << solution.path[i].second;
            
            if (i > 0) {
                string maneuver;
                switch (solution.maneuvers[i-1]) {
                    case ManeuverType::ACCELERATION: maneuver = "Разгон"; break;
                    case ManeuverType::CLIMB: maneuver = "Подъем"; break;
                    case ManeuverType::COMBINED: maneuver = "Комбинир."; break;
                }
                cout << setw(15) << maneuver;
            } else {
                cout << setw(15) << "Старт";
            }
            cout << endl;
        }
    }
    
    void saveSolutionToFile(const TrajectoryOptimizer::TrajectorySolution& solution,
                           const string& filename) {
        ofstream file(filename + ".csv");
        file << "Point,Altitude_m,Speed_kmh,Maneuver,Time_s,Fuel_kg\n";
        
        for (size_t i = 0; i < solution.path.size(); i++) {
            file << i + 1 << ","
                 << solution.path[i].first << ","
                 << solution.path[i].second << ",";
            
            if (i == 0) {
                file << "START,0,0";
            } else {
                string maneuver;
                switch (solution.maneuvers[i-1]) {
                    case ManeuverType::ACCELERATION: maneuver = "ACCELERATION"; break;
                    case ManeuverType::CLIMB: maneuver = "CLIMB"; break;
                    case ManeuverType::COMBINED: maneuver = "COMBINED"; break;
                }
                file << maneuver << ","
                     << solution.times[i-1] << ","
                     << solution.fuels[i-1];
            }
            file << "\n";
        }
        file.close();
        
        cout << "\nДанные сохранены в файл: " << filename << ".csv\n";
    }
    
    void createComparisonPlot(const TrajectoryOptimizer::TrajectorySolution& time_sol,
                             const TrajectoryOptimizer::TrajectorySolution& fuel_sol) {
        ofstream script("comparison_plot.gp");
        script << "# GNUPLOT script for trajectory comparison\n";
        script << "set terminal pngcairo size 1200,800 enhanced\n";
        script << "set output 'trajectory_comparison.png'\n\n";
        script << "set title 'Сравнение траекторий Як-40'\n";
        script << "set xlabel 'Скорость (км/ч)'\n";
        script << "set ylabel 'Высота (м)'\n";
        script << "set grid\n";
        script << "set key top left\n";
        script << "set datafile separator ','\n\n";
        script << "plot 'time_comparison.csv' every ::1 using 3:2 \\\n";
        script << "     with linespoints lw 2 pt 7 ps 1 title 'Минимум времени', \\\n";
        script << "     'fuel_comparison.csv' every ::1 using 3:2 \\\n";
        script << "     with linespoints lw 2 pt 7 ps 1 title 'Минимум топлива'\n";
        script.close();
        
        cout << "\nСоздан скрипт для GNUPLOT: comparison_plot.gp\n";
        cout << "Для построения графика выполните: gnuplot comparison_plot.gp\n";
    }
};

// ============================ ГЛАВНАЯ ФУНКЦИЯ ============================
int main() {
    // Установка параметров
    AircraftParams aircraft;
    FlightConditions conditions;
    
    // Создание и запуск планировщика
    FlightPlanner planner;
    planner.run();
    
    cout << "\nПрограмма завершена.\n";
    return 0;
}