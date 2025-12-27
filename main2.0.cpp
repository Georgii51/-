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

using namespace std;

// ==================== КОНСТАНТЫ ====================
const double MASS = 13900.0;          // Масса самолета, кг
const double WING_AREA = 70.0;        // Площадь крыла, м²
const int ENGINE_COUNT = 3;           // Количество двигателей
const double THRUST_PERCENT = 100.0;  // Режим работы двигателей, %

// Начальные и конечные параметры
const double H_START = 400.0;         // Начальная высота, м
const double H_FINISH = 6100.0;       // Конечная высота, м
const double V_START_KMH = 260.0;     // Начальная скорость, км/ч
const double V_FINISH_KMH = 860.0;    // Конечная скорость, км/ч

const int N = 10;                     // Количество точек разбиения
const double G = 9.81;
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

// Ограничения
const double MAX_VERTICAL_SPEED = 10.0;      // м/с
const double MAX_CLIMB_ANGLE = 15.0;         // градусы
const double MIN_CLIMB_SPEED_KMH = 320.0;    // км/ч

// Аэродинамические коэффициенты
const double CY0 = -0.08;
const double CY1 = 0.075;
const double CX0 = 0.022;
const double K = 0.035;

// ==================== АТМОСФЕРА ====================
struct AtmosData {
    double H, rho, a, T;
};

static const AtmosData ATMOS_TABLE[] = {
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
const int ATMOS_SIZE = 13;

void getAtmosphere(double H, double& rho, double& a) {
    if (H <= ATMOS_TABLE[0].H) {
        rho = ATMOS_TABLE[0].rho;
        a = ATMOS_TABLE[0].a;
        return;
    }
    if (H >= ATMOS_TABLE[ATMOS_SIZE-1].H) {
        rho = ATMOS_TABLE[ATMOS_SIZE-1].rho;
        a = ATMOS_TABLE[ATMOS_SIZE-1].a;
        return;
    }
    
    for (int i = 0; i < ATMOS_SIZE-1; i++) {
        if (H >= ATMOS_TABLE[i].H && H < ATMOS_TABLE[i+1].H) {
            double t = (H - ATMOS_TABLE[i].H) / (ATMOS_TABLE[i+1].H - ATMOS_TABLE[i].H);
            rho = ATMOS_TABLE[i].rho + t * (ATMOS_TABLE[i+1].rho - ATMOS_TABLE[i].rho);
            a = ATMOS_TABLE[i].a + t * (ATMOS_TABLE[i+1].a - ATMOS_TABLE[i].a);
            return;
        }
    }
}

double getMach(double H, double V_ms) {
    double rho, a;
    getAtmosphere(H, rho, a);
    return V_ms / a;
}

// ==================== АЭРОДИНАМИКА ====================
double calculateCy(double alpha_deg) {
    alpha_deg = max(0.0, min(10.0, alpha_deg));
    return CY0 + CY1 * alpha_deg;
}

double calculateCx(double alpha_deg) {
    double Cy = calculateCy(alpha_deg);
    return CX0 + K * Cy * Cy;
}

double calculateAlpha(double H, double V_ms, double mass) {
    double rho, a;
    getAtmosphere(H, rho, a);
    
    double q = 0.5 * rho * V_ms * V_ms;
    if (q < 1.0) return 5.0;
    
    // Баланс сил по вертикали: Cy*q*S = m*g
    double required_Cy = (mass * G) / (q * WING_AREA);
    double alpha_deg = (required_Cy - CY0) / CY1;
    
    return max(0.0, min(10.0, alpha_deg));
}

// ==================== ДВИГАТЕЛЬ ====================
double calculateThrust(double H, double V_ms) {
    double M = getMach(H, V_ms);
    double H_km = H / 1000.0;
    
    // Номинальная тяга на уровне моря
    double P0 = 58860.0; // Н
    
    // Коррекция по высоте
    double altitude_factor;
    if (H_km <= 0) {
        altitude_factor = 1.0;
    } else if (H_km >= 11.0) {
        altitude_factor = 0.50;
    } else {
        altitude_factor = 1.0 - 0.50 * pow(H_km / 11.0, 0.7);
    }
    
    // Коррекция по числу Маха
    double mach_factor = 0.88 + 0.24 * M;
    if (mach_factor > 1.08) mach_factor = 1.08;
    
    double single_engine_thrust = P0 * altitude_factor * mach_factor;
    return single_engine_thrust * ENGINE_COUNT * (THRUST_PERCENT / 100.0);
}

double calculateFuelConsumption(double H, double V_ms, double thrust_setting) {
    double M = getMach(H, V_ms);
    double H_km = H / 1000.0;
    
    double Cp_base = 0.72 / 9.81; // кг/(Н·ч)
    
    // Коррекция по режиму работы
    double regime_factor;
    if (thrust_setting >= 1.0) {
        regime_factor = 1.0 + 0.3 * (thrust_setting - 1.0);
    } else if (thrust_setting >= 0.88) {
        regime_factor = 0.92 - 0.02 * (thrust_setting - 0.88) / 0.12;
    } else if (thrust_setting >= 0.70) {
        regime_factor = 0.92 + 0.10 * (0.88 - thrust_setting) / 0.18;
    } else {
        regime_factor = 1.05;
    }
    
    // Упрощенные коррекции
    double altitude_factor = 1.0 - 0.05 * min(1.0, H_km / 11.0);
    double mach_factor = 1.0 + 0.1 * max(0.0, M - 0.5);
    
    return Cp_base * regime_factor * altitude_factor * mach_factor;
}

// ==================== МАНЕВРЫ ====================
enum Maneuver {
    RAZGON = 1,
    PODIEM = 2
};

struct Segment {
    bool valid;
    double time;
    double fuel;
    
    Segment() : valid(false), time(0), fuel(0) {}
};

Segment calculateAcceleration(double H, double V1_ms, double V2_ms, double thrust_setting) {
    Segment seg;
    
    double V_avg = 0.5 * (V1_ms + V2_ms);
    if (V_avg * 3.6 < 200.0) return seg;
    
    // Угол атаки
    double alpha_deg = calculateAlpha(H, V_avg, MASS);
    
    // Тяга
    double P_max = calculateThrust(H, V_avg);
    double P_used = P_max * thrust_setting;
    
    // Аэродинамическое сопротивление
    double rho, a;
    getAtmosphere(H, rho, a);
    double q = 0.5 * rho * V_avg * V_avg;
    
    double Cx = calculateCx(alpha_deg);
    double X = Cx * q * WING_AREA;
    
    // Ускорение
    double dV_dt = (P_used - X) / MASS;
    
    if (dV_dt <= 0.005) {  // Ослабленное условие
        // cout << "Разгон: малое ускорение " << dV_dt << endl;
        return seg;
    }
    
    // Время разгона
    seg.time = (V2_ms - V1_ms) / dV_dt;
    
    if (seg.time <= 0 || seg.time > 2000.0) {
        // cout << "Разгон: неверное время " << seg.time << endl;
        return seg;
    }
    
    // Расход топлива
    double Cp = calculateFuelConsumption(H, V_avg, thrust_setting);
    seg.fuel = Cp * P_used * seg.time / 3600.0;
    
    seg.valid = true;
    return seg;
}

Segment calculateClimb(double H1, double H2, double V_ms, double thrust_setting, double vy_factor) {
    Segment seg;
    
    if (V_ms * 3.6 < MIN_CLIMB_SPEED_KMH) {
        // cout << "Подъем: скорость мала " << V_ms*3.6 << endl;
        return seg;
    }
    
    double H_avg = 0.5 * (H1 + H2);
    
    // Угол атаки
    double alpha_deg = calculateAlpha(H_avg, V_ms, MASS);
    
    // Тяга
    double P_max = calculateThrust(H_avg, V_ms);
    double P_used = P_max * thrust_setting;
    
    // Сопротивление
    double rho, a;
    getAtmosphere(H_avg, rho, a);
    double q = 0.5 * rho * V_ms * V_ms;
    
    double Cx = calculateCx(alpha_deg);
    double X = Cx * q * WING_AREA;
    
    // Избыточная тяга
    double P_excess = P_used - X;
    
    if (P_excess <= 100.0) {  // Ослабленное условие
        // cout << "Подъем: малая избыточная тяга " << P_excess << endl;
        return seg;
    }
    
    // Угол набора
    double sin_theta = min(P_excess / (MASS * G), sin(MAX_CLIMB_ANGLE * DEG_TO_RAD));
    
    if (sin_theta <= 0.002) {  // Ослабленное условие
        // cout << "Подъем: малый sin_theta " << sin_theta << endl;
        return seg;
    }
    
    // Вертикальная скорость
    double Vy = V_ms * sin_theta;
    double max_vy = MAX_VERTICAL_SPEED * vy_factor;
    if (Vy > max_vy) Vy = max_vy;
    
    // Время набора
    seg.time = (H2 - H1) / Vy;
    
    if (seg.time <= 0 || seg.time > 3000.0) {
        // cout << "Подъем: неверное время " << seg.time << endl;
        return seg;
    }
    
    // Расход топлива
    double Cp = calculateFuelConsumption(H_avg, V_ms, thrust_setting);
    seg.fuel = Cp * P_used * seg.time / 3600.0;
    
    seg.valid = true;
    return seg;
}

// ==================== ОПТИМИЗАЦИЯ ====================
struct Result {
    vector<pair<double, double>> path;  // (H, V)
    vector<Maneuver> maneuvers;
    vector<double> times;
    vector<double> fuels;
    double total_time;
    double total_fuel;
    
    Result() : total_time(0), total_fuel(0) {}
};

Result findOptimalPath(bool minimize_time, const string& name) {
    Result result;
    
    cout << "\n========================================\n";
    cout << " " << name << "\n";
    cout << "========================================\n";
    
    // Создаем сетку
    vector<double> H_grid(N+1), V_kmh_grid(N+1), V_ms_grid(N+1);
    
    double dH = (H_FINISH - H_START) / N;
    double dV_kmh = (V_FINISH_KMH - V_START_KMH) / N;
    
    for (int i = 0; i <= N; i++) {
        H_grid[i] = H_START + i * dH;
        V_kmh_grid[i] = V_START_KMH + i * dV_kmh;
        V_ms_grid[i] = V_kmh_grid[i] / 3.6;
    }
    
    // Настройки для разных критериев
    vector<double> thrust_settings;
    double vy_factor;
    
    if (minimize_time) {
        thrust_settings = {1.05, 1.00, 0.95};  // Высокие режимы
        vy_factor = 1.0;
        cout << "Режимы тяги: 1.05, 1.00, 0.95\n";
    } else {
        thrust_settings = {0.88, 0.82, 0.75};  // Экономичные режимы
        vy_factor = 0.7;
        cout << "Режимы тяги: 0.88, 0.82, 0.75\n";
    }
    
    // Матрицы для ДП
    vector<vector<double>> time_mat(N+1, vector<double>(N+1, 1e9));
    vector<vector<double>> fuel_mat(N+1, vector<double>(N+1, 1e9));
    vector<vector<double>> cost_mat(N+1, vector<double>(N+1, 1e9));
    vector<vector<bool>> reachable(N+1, vector<bool>(N+1, false));
    vector<vector<pair<int, int>>> prev(N+1, vector<pair<int, int>>(N+1, {-1, -1}));
    vector<vector<Maneuver>> man_mat(N+1, vector<Maneuver>(N+1, RAZGON));
    
    // Начальное состояние
    time_mat[0][0] = 0;
    fuel_mat[0][0] = 0;
    cost_mat[0][0] = 0;
    reachable[0][0] = true;
    
    int valid_segments = 0;
    
    // Динамическое программирование
    for (int i = 0; i <= N; i++) {
        for (int j = 0; j <= N; j++) {
            if (!reachable[i][j]) continue;
            
            double current_H = H_grid[i];
            double current_V_ms = V_ms_grid[j];
            
            for (double thrust_setting : thrust_settings) {
                // Разгон
                if (j < N) {
                    double next_V_ms = V_ms_grid[j + 1];
                    Segment seg = calculateAcceleration(current_H, current_V_ms, next_V_ms, thrust_setting);
                    
                    if (seg.valid) {
                        valid_segments++;
                        double new_time = time_mat[i][j] + seg.time;
                        double new_fuel = fuel_mat[i][j] + seg.fuel;
                        double new_cost = minimize_time ? new_time : new_fuel;
                        
                        if (new_cost < cost_mat[i][j+1]) {
                            time_mat[i][j+1] = new_time;
                            fuel_mat[i][j+1] = new_fuel;
                            cost_mat[i][j+1] = new_cost;
                            reachable[i][j+1] = true;
                            prev[i][j+1] = {i, j};
                            man_mat[i][j+1] = RAZGON;
                        }
                    }
                }
                
                // Подъем
                if (i < N) {
                    double next_H = H_grid[i + 1];
                    Segment seg = calculateClimb(current_H, next_H, current_V_ms, thrust_setting, vy_factor);
                    
                    if (seg.valid) {
                        valid_segments++;
                        double new_time = time_mat[i][j] + seg.time;
                        double new_fuel = fuel_mat[i][j] + seg.fuel;
                        double new_cost = minimize_time ? new_time : new_fuel;
                        
                        if (new_cost < cost_mat[i+1][j]) {
                            time_mat[i+1][j] = new_time;
                            fuel_mat[i+1][j] = new_fuel;
                            cost_mat[i+1][j] = new_cost;
                            reachable[i+1][j] = true;
                            prev[i+1][j] = {i, j};
                            man_mat[i+1][j] = PODIEM;
                        }
                    }
                }
            }
        }
    }
    
    cout << "Валидных сегментов: " << valid_segments << endl;
    cout << "Конечная точка достижима: " << (reachable[N][N] ? "ДА" : "НЕТ") << endl;
    
    // Проверяем достижимость
    if (!reachable[N][N]) {
        cout << "\nВНИМАНИЕ: Не удалось достичь конечной точки!\n";
        
        // Ищем ближайшую достижимую
        double min_dist = 1e9;
        int best_i = -1, best_j = -1;
        
        for (int i = 0; i <= N; i++) {
            for (int j = 0; j <= N; j++) {
                if (reachable[i][j]) {
                    double dist = sqrt(pow(i-N, 2) + pow(j-N, 2));
                    if (dist < min_dist) {
                        min_dist = dist;
                        best_i = i;
                        best_j = j;
                    }
                }
            }
        }
        
        if (best_i != -1) {
            cout << "Ближайшая достижимая точка: [" << best_i << "," << best_j << "]\n";
            cout << "H = " << H_grid[best_i] << " м (цель: " << H_grid[N] << " м)\n";
            cout << "V = " << V_kmh_grid[best_j] << " км/ч (цель: " << V_kmh_grid[N] << " км/ч)\n";
            
            // Восстанавливаем путь до этой точки
            int ci = best_i, cj = best_j;
            while (ci >= 0 && cj >= 0) {
                result.path.push_back({H_grid[ci], V_kmh_grid[cj]});
                
                int pi = prev[ci][cj].first;
                int pj = prev[ci][cj].second;
                
                if (pi == -1) break;
                
                result.maneuvers.push_back(man_mat[ci][cj]);
                result.times.push_back(time_mat[ci][cj] - time_mat[pi][pj]);
                result.fuels.push_back(fuel_mat[ci][cj] - fuel_mat[pi][pj]);
                
                ci = pi;
                cj = pj;
            }
            
            reverse(result.path.begin(), result.path.end());
            reverse(result.maneuvers.begin(), result.maneuvers.end());
            reverse(result.times.begin(), result.times.end());
            reverse(result.fuels.begin(), result.fuels.end());
            
            result.total_time = time_mat[best_i][best_j];
            result.total_fuel = fuel_mat[best_i][best_j];
            
            cout << "Восстановлен частичный путь (" << result.path.size() << " точек)\n";
        }
        
        return result;
    }
    
    // Восстанавливаем полный путь
    int ci = N, cj = N;
    
    while (ci >= 0 && cj >= 0) {
        result.path.push_back({H_grid[ci], V_kmh_grid[cj]});
        
        int pi = prev[ci][cj].first;
        int pj = prev[ci][cj].second;
        
        if (pi == -1) break;
        
        result.maneuvers.push_back(man_mat[ci][cj]);
        result.times.push_back(time_mat[ci][cj] - time_mat[pi][pj]);
        result.fuels.push_back(fuel_mat[ci][cj] - fuel_mat[pi][pj]);
        
        ci = pi;
        cj = pj;
    }
    
    reverse(result.path.begin(), result.path.end());
    reverse(result.maneuvers.begin(), result.maneuvers.end());
    reverse(result.times.begin(), result.times.end());
    reverse(result.fuels.begin(), result.fuels.end());
    
    result.total_time = time_mat[N][N];
    result.total_fuel = fuel_mat[N][N];
    
    // Вывод матриц
    cout << "\n=== МАТРИЦА ВРЕМЕНИ (с) ===\n";
    cout << "     V->";
    for (int j = 0; j <= N; j++) {
        cout << setw(7) << (int)V_kmh_grid[j];
    }
    cout << "\nH\n";
    
    for (int i = 0; i <= N; i++) {
        cout << setw(5) << (int)H_grid[i];
        for (int j = 0; j <= N; j++) {
            if (reachable[i][j] && time_mat[i][j] < 1e8) {
                cout << setw(7) << (int)time_mat[i][j];
            } else {
                cout << setw(7) << "---";
            }
        }
        cout << endl;
    }
    
    cout << "\n=== МАТРИЦА ТОПЛИВА (кг) ===\n";
    cout << "     V->";
    for (int j = 0; j <= N; j++) {
        cout << setw(7) << (int)V_kmh_grid[j];
    }
    cout << "\nH\n";
    
    for (int i = 0; i <= N; i++) {
        cout << setw(5) << (int)H_grid[i];
        for (int j = 0; j <= N; j++) {
            if (reachable[i][j] && fuel_mat[i][j] < 1e8) {
                cout << setw(7) << (int)fuel_mat[i][j];
            } else {
                cout << setw(7) << "---";
            }
        }
        cout << endl;
    }
    
    // Сохраняем матрицы в файлы
    string suffix = minimize_time ? "min_time" : "min_fuel";
    
    ofstream time_csv("time_matrix_" + suffix + ".csv");
    time_csv << "H/V";
    for (int j = 0; j <= N; j++) time_csv << "," << V_kmh_grid[j];
    time_csv << "\n";
    
    for (int i = 0; i <= N; i++) {
        time_csv << H_grid[i];
        for (int j = 0; j <= N; j++) {
            time_csv << ",";
            if (reachable[i][j] && time_mat[i][j] < 1e8) {
                time_csv << time_mat[i][j];
            }
        }
        time_csv << "\n";
    }
    time_csv.close();
    
    ofstream fuel_csv("fuel_matrix_" + suffix + ".csv");
    fuel_csv << "H/V";
    for (int j = 0; j <= N; j++) fuel_csv << "," << V_kmh_grid[j];
    fuel_csv << "\n";
    
    for (int i = 0; i <= N; i++) {
        fuel_csv << H_grid[i];
        for (int j = 0; j <= N; j++) {
            fuel_csv << ",";
            if (reachable[i][j] && fuel_mat[i][j] < 1e8) {
                fuel_csv << fuel_mat[i][j];
            }
        }
        fuel_csv << "\n";
    }
    fuel_csv.close();
    
    // Сохраняем траекторию
    ofstream traj_csv("trajectory_" + suffix + ".csv");
    traj_csv << "Point,H_m,V_kmh,Maneuver,Time_s,Fuel_kg\n";
    
    for (size_t k = 0; k < result.path.size(); k++) {
        traj_csv << k+1 << "," << result.path[k].first << "," << result.path[k].second << ",";
        
        if (k == 0) {
            traj_csv << "START,0,0";
        } else {
            string man_str = (result.maneuvers[k-1] == RAZGON) ? "RAZGON" : "PODIEM";
            traj_csv << man_str << "," << result.times[k-1] << "," << result.fuels[k-1];
        }
        traj_csv << "\n";
    }
    traj_csv.close();
    
    // Вывод результатов
    cout << "\n=== РЕЗУЛЬТАТЫ ===\n";
    cout << "Путь состоит из " << result.path.size() << " точек\n";
    
    int razgon_count = 0, podiem_count = 0;
    for (auto m : result.maneuvers) {
        if (m == RAZGON) razgon_count++;
        else podiem_count++;
    }
    
    cout << "Маневры: Разгон - " << razgon_count << ", Подъем - " << podiem_count << endl;
    cout << fixed << setprecision(1);
    cout << "Общее время: " << result.total_time << " с (" << result.total_time/60.0 << " мин)\n";
    cout << "Общий расход топлива: " << result.total_fuel << " кг\n";
    
    double avg_vy = (H_FINISH - H_START) / result.total_time;
    cout << "Средняя вертикальная скорость: " << avg_vy << " м/с\n";
    
    cout << "\nДетальная траектория:\n";
    cout << "№  Высота(м)  Скорость(км/ч)  Маневр  Время(с)  Топливо(кг)\n";
    cout << "----------------------------------------------------------\n";
    
    for (size_t k = 0; k < result.path.size(); k++) {
        cout << setw(2) << k+1 << "  "
             << setw(9) << result.path[k].first << "  "
             << setw(12) << result.path[k].second << "  ";
        
        if (k == 0) {
            cout << "СТАРТ      0        0";
        } else {
            string man_str = (result.maneuvers[k-1] == RAZGON) ? "РАЗГОН" : "ПОДЪЕМ";
            cout << setw(6) << man_str << "  "
                 << setw(8) << result.times[k-1] << "  "
                 << setw(10) << result.fuels[k-1];
        }
        cout << endl;
    }
    
    cout << "\nФайлы сохранены:\n";
    cout << "- trajectory_" << suffix << ".csv\n";
    cout << "- time_matrix_" << suffix << ".csv\n";
    cout << "- fuel_matrix_" << suffix << ".csv\n";
    
    return result;
}

// ==================== ГЛАВНАЯ ФУНКЦИЯ ====================
int main() {
    cout << fixed << setprecision(2);
    
    cout << "\n=================================================\n";
    cout << "   ОПТИМИЗАЦИЯ ТРАЕКТОРИИ ЯК-40 (УПРОЩЕННЫЙ)\n";
    cout << "=================================================\n";
    cout << "Самолет: Як-40 (масса " << MASS/1000.0 << " т)\n";
    cout << "Двигатели: 3 х Д-30КП (" << THRUST_PERCENT << "% номинала)\n";
    cout << "Старт: H = " << H_START << " м, V = " << V_START_KMH << " км/ч\n";
    cout << "Финиш: H = " << H_FINISH << " м, V = " << V_FINISH_KMH << " км/ч\n";
    cout << "Сетка: " << N << " сегментов\n";
    cout << "=================================================\n\n";
    
    int choice;
    cout << "Выберите вариант:\n";
    cout << "1 - Минимизация времени\n";
    cout << "2 - Минимизация расхода топлива\n";
    cout << "3 - Сравнение обоих\n";
    cout << "Ваш выбор: ";
    cin >> choice;
    
    if (choice == 1) {
        Result res = findOptimalPath(true, "МИНИМИЗАЦИЯ ВРЕМЕНИ");
        
        // Создаем простой скрипт для GNUPLOT
        ofstream gp("plot_time.gp");
        gp << "set terminal pngcairo size 800,600\n";
        gp << "set output 'trajectory_time.png'\n";
        gp << "set title 'Як-40: Минимизация времени'\n";
        gp << "set xlabel 'Скорость (км/ч)'\n";
        gp << "set ylabel 'Высота (м)'\n";
        gp << "set grid\n";
        gp << "plot 'trajectory_min_time.csv' every ::1 using 3:2 with linespoints\n";
        gp.close();
        
        cout << "\nДля построения графика: gnuplot plot_time.gp\n";
        
    } else if (choice == 2) {
        Result res = findOptimalPath(false, "МИНИМИЗАЦИЯ ТОПЛИВА");
        
        ofstream gp("plot_fuel.gp");
        gp << "set terminal pngcairo size 800,600\n";
        gp << "set output 'trajectory_fuel.png'\n";
        gp << "set title 'Як-40: Минимизация топлива'\n";
        gp << "set xlabel 'Скорость (км/ч)'\n";
        gp << "set ylabel 'Высота (м)'\n";
        gp << "set grid\n";
        gp << "plot 'trajectory_min_fuel.csv' every ::1 using 3:2 with linespoints\n";
        gp.close();
        
        cout << "\nДля построения графика: gnuplot plot_fuel.gp\n";
        
    } else if (choice == 3) {
        cout << "\n=== СРАВНЕНИЕ ВАРИАНТОВ ===\n\n";
        
        Result time_res = findOptimalPath(true, "МИНИМИЗАЦИЯ ВРЕМЕНИ");
        cout << "\n\n";
        Result fuel_res = findOptimalPath(false, "МИНИМИЗАЦИЯ ТОПЛИВА");
        
        cout << "\n=== ИТОГИ СРАВНЕНИЯ ===\n";
        cout << "Критерий       Время(мин)  Топливо(кг)  Vy ср(м/с)\n";
        cout << "--------------------------------------------------\n";
        
        double avg_vy_time = (H_FINISH - H_START) / time_res.total_time;
        double avg_vy_fuel = (H_FINISH - H_START) / fuel_res.total_time;
        
        cout << fixed << setprecision(1);
        cout << "Минимум времени  " << setw(10) << time_res.total_time/60.0
             << setw(12) << time_res.total_fuel
             << setw(12) << avg_vy_time << endl;
        
        cout << "Минимум топлива  " << setw(10) << fuel_res.total_time/60.0
             << setw(12) << fuel_res.total_fuel
             << setw(12) << avg_vy_fuel << endl;
        
        // Создаем скрипт для сравнения
        ofstream gp("plot_compare.gp");
        gp << "set terminal pngcairo size 1000,800\n";
        gp << "set output 'trajectory_compare.png'\n";
        gp << "set title 'Сравнение траекторий Як-40'\n";
        gp << "set xlabel 'Скорость (км/ч)'\n";
        gp << "set ylabel 'Высота (м)'\n";
        gp << "set grid\n";
        gp << "set key top left\n";
        gp << "plot 'trajectory_min_time.csv' every ::1 using 3:2 with linespoints title 'Минимум времени', \\\n";
        gp << "     'trajectory_min_fuel.csv' every ::1 using 3:2 with linespoints title 'Минимум топлива'\n";
        gp.close();
        
        cout << "\nДля построения графика сравнения: gnuplot plot_compare.gp\n";
        
    } else {
        cout << "Неверный выбор!\n";
    }
    
    cout << "\nПрограмма завершена.\n";
    return 0;
}