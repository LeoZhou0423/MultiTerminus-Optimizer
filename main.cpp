#include <iostream>     // 标准输入输出
#include <vector>       // 动态数组
#include <string>       // 字符串处理
#include <fstream>      // 文件读写（预留接口）
#include <sstream>      // 字符串流
#include <cmath>        // 数学函数（sin, cos, exp 等）
#include <algorithm>    // 算法（sort, next_permutation, iota）
#include <random>       // 随机数生成
#include <chrono>       // 高精度计时
#include <iomanip>      // 浮点数格式化输出
#include <limits>       // numeric_limits
#include <cstdint>      // uint64_t 定义
#include <unordered_map> // 哈希表用于距离缓存

// --- 地理坐标结构 ---
struct Coordinate {
    double longitude;
    double latitude;
    Coordinate(double lon = 0, double lat = 0) : longitude(lon), latitude(lat) {}
};

// --- 距离计算器（支持缓存可选）---
class DistanceCalculator {
private:
    mutable std::unordered_map<uint64_t, double> distance_cache; // 缓存距离计算结果
    bool use_cache;

    // 哈希键：(i << 32) | j，适用于 n <= 60
    uint64_t make_key(int i, int j) const {
        return (static_cast<uint64_t>(i) << 32) | static_cast<uint64_t>(j);
    }

public:
    explicit DistanceCalculator(bool enable_cache = true) : use_cache(enable_cache) {}

    // 清理缓存：在大规模计算后调用，释放内存
    void clear_distance_cache() {
        distance_cache.clear();
    }

    double calculate(const Coordinate& a, const Coordinate& b) const {
        const double R = 6371000; // 地球半径（米）
        double lat1 = a.latitude * M_PI / 180.0;
        double lat2 = b.latitude * M_PI / 180.0;
        double delta_lat = (b.latitude - a.latitude) * M_PI / 180.0;
        double delta_lon = (b.longitude - a.longitude) * M_PI / 180.0;

        double aa = std::sin(delta_lat / 2) * std::sin(delta_lat / 2) +
                   std::cos(lat1) * std::cos(lat2) *
                   std::sin(delta_lon / 2) * std::sin(delta_lon / 2);
        double c = 2 * std::atan2(std::sqrt(aa), std::sqrt(1 - aa));
        return R * c;
    }

    // 带缓存的距离查询（仅用于精确算法）
    double get_distance(const std::vector<Coordinate>& points, int i, int j) const {
        if (!use_cache) return calculate(points[i], points[j]);

        uint64_t key1 = make_key(i, j);
        uint64_t key2 = make_key(j, i);
        auto it = distance_cache.find(key1);
        if (it != distance_cache.end()) return it->second;
        it = distance_cache.find(key2);
        if (it != distance_cache.end()) return it->second;

        double dist = calculate(points[i], points[j]);
        distance_cache[key1] = dist;
        return dist;
    }
};

// --- 算法参数配置类 ---
struct AlgorithmConfig {
    int population_size = 100;           // 遗传算法种群大小（预留）
    double cooling_rate = 0.995;         // 模拟退火冷却率
    double initial_temperature = 10000.0; // 初始温度
    int max_iterations = 100000;         // 最大迭代次数
    bool use_cache = true;               // 是否启用距离缓存
    bool enable_local_search = true;     // 是否启用 2-opt 后优化

    // 构造函数支持自定义参数
    AlgorithmConfig() = default;
    AlgorithmConfig(double cool_rate, double init_temp, int max_iter)
        : cooling_rate(cool_rate), initial_temperature(init_temp), max_iterations(max_iter) {}
};

// --- 路径规划结果 ---
struct RouteResult {
    std::vector<Coordinate> path;
    double total_distance;
    double execution_time_ms;
    int start_index;
    int end_index;
};

// --- 路径规划器 ---
class RoutePlanner {
private:
    DistanceCalculator dist_calc;
    std::mt19937 gen;
    AlgorithmConfig config; // 算法配置参数

    double calculate_path_distance(const std::vector<Coordinate>& path) const {
        if (path.size() < 2) return 0.0;
        double total = 0.0;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            total += dist_calc.calculate(path[i], path[i+1]);
        }
        return total;
    }

    // 构造贪婪路径：从随机起点开始，每次选择最近的未访问点
    std::vector<size_t> construct_greedy_path(const std::vector<Coordinate>& waypoints) {
        size_t n = waypoints.size();
        if (n == 0) return {};
        
        std::vector<size_t> path(n);
        std::vector<bool> visited(n, false);
        
        // 随机选择起始点
        std::uniform_int_distribution<size_t> dist(0, n-1);
        size_t current = dist(gen);
        path[0] = current;
        visited[current] = true;

        // 逐步构建路径
        for (size_t i = 1; i < n; ++i) {
            double best_dist = std::numeric_limits<double>::max();
            size_t next_point = current;
            
            // 寻找最近的未访问点
            for (size_t j = 0; j < n; ++j) {
                if (!visited[j]) {
                    double d = dist_calc.calculate(waypoints[current], waypoints[j]);
                    if (d < best_dist) {
                        best_dist = d;
                        next_point = j;
                    }
                }
            }
            
            if (next_point != current) {
                path[i] = next_point;
                visited[next_point] = true;
                current = next_point;
            }
        }
        return path;
    }

    // 2-opt 反转
    void two_opt_swap(std::vector<size_t>& path, int i, int j) {
        while (i < j) {
            std::swap(path[i], path[j]);
            ++i;
            --j;
        }
    }

    // 2-opt 局部搜索优化
    std::vector<size_t> two_opt_optimize(const std::vector<Coordinate>& waypoints,
                                        std::vector<size_t> path) {
        if (!config.enable_local_search || path.size() < 4) return path;

        bool improved = true;
        double best_distance = calculate_path_distance(extract_path(waypoints, path));
        int n = path.size();
        std::vector<size_t> current_path = path;

        while (improved) {
            improved = false;
            for (int i = 1; i < n - 1; ++i) {
                for (int j = i + 1; j < n; ++j) {
                    two_opt_swap(current_path, i, j);
                    double new_distance = calculate_path_distance(extract_path(waypoints, current_path));
                    if (new_distance < best_distance) {
                        best_distance = new_distance;
                        improved = true;
                    } else {
                        two_opt_swap(current_path, i, j);
                    }
                }
            }
        }
        return current_path;
    }

    std::vector<Coordinate> extract_path(const std::vector<Coordinate>& waypoints,
                                        const std::vector<size_t>& indices) const {
        std::vector<Coordinate> result;
        result.reserve(indices.size());
        for (size_t idx : indices) {
            result.push_back(waypoints[idx]);
        }
        return result;
    }

    // 模拟退火算法
    std::vector<size_t> simulated_annealing(const std::vector<Coordinate>& waypoints) {
        size_t n = waypoints.size();
        if (n <= 1) {
            std::vector<size_t> path(n);
            std::iota(path.begin(), path.end(), 0);
            return path;
        }
        
        // 用贪婪算法生成初始解
        std::vector<size_t> current_path = construct_greedy_path(waypoints);
        std::vector<size_t> best_path = current_path;
        double current_dist = calculate_path_distance(extract_path(waypoints, current_path));
        double best_dist = current_dist;

        double temp = config.initial_temperature;
        double cooling_rate = config.cooling_rate;
        int max_iter = std::min(config.max_iterations, (int)(n * n * 50));

        std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
        std::uniform_int_distribution<size_t> index_dist(1, n-1);

        for (int iter = 0; iter < max_iter; ++iter) {
            // 随机选择两个点进行2-opt交换
            size_t i = index_dist(gen);
            size_t j = index_dist(gen);
            if (i == j) continue;
            if (i > j) std::swap(i, j);

            two_opt_swap(current_path, i, j);
            double new_dist = calculate_path_distance(extract_path(waypoints, current_path));

            // 计算接受概率
            double delta = new_dist - current_dist;
            if (delta < 0 || (temp > 1e-9 && prob_dist(gen) < std::exp(-delta / temp))) {
                current_dist = new_dist;
                if (new_dist < best_dist) {
                    best_dist = new_dist;
                    best_path = current_path;
                }
            } else {
                two_opt_swap(current_path, i, j);
            }

            temp *= cooling_rate;
            if (temp < 1.0) break;
        }

        return best_path;
    }

    // 精确解法（仅适用于小规模问题）
    std::vector<size_t> solve_exact_internal(const std::vector<Coordinate>& waypoints) {
        size_t n = waypoints.size();
        if (n > 12) {
            return simulated_annealing(waypoints);
        }

        std::vector<size_t> indices(n);
        std::iota(indices.begin(), indices.end(), 0);
        std::vector<size_t> best_perm = indices;
        double best_dist = calculate_path_distance(extract_path(waypoints, indices));

        // 枚举所有排列寻找最优解
        do {
            double dist = calculate_path_distance(extract_path(waypoints, indices));
            if (dist < best_dist) {
                best_dist = dist;
                best_perm = indices;
            }
        } while (std::next_permutation(indices.begin(), indices.end()));

        return best_perm;
    }

public:
    // 支持传入配置参数
    explicit RoutePlanner(const AlgorithmConfig& cfg = AlgorithmConfig())
        : gen(std::random_device{}()), config(cfg), dist_calc(cfg.use_cache) {}

    RouteResult plan_route(const std::vector<Coordinate>& starts,
                          const std::vector<Coordinate>& waypoints,
                          const std::vector<Coordinate>& ends) {
        auto start_time = std::chrono::high_resolution_clock::now();

        std::vector<Coordinate> best_full_path;
        double best_total_distance = std::numeric_limits<double>::max();
        int best_start_idx = 0;
        int best_end_idx = 0;

        // 遍历所有可能的起点和终点组合
        for (size_t si = 0; si < starts.size(); ++si) {
            for (size_t ei = 0; ei < ends.size(); ++ei) {
                std::vector<size_t> internal_order;
                if (waypoints.size() <= 12) {
                    internal_order = solve_exact_internal(waypoints);
                } else {
                    internal_order = simulated_annealing(waypoints);
                    if (config.enable_local_search) {
                        internal_order = two_opt_optimize(waypoints, internal_order);
                    }
                }

                // 构建完整路径
                std::vector<Coordinate> full_path;
                full_path.push_back(starts[si]);
                for (size_t idx : internal_order) {
                    full_path.push_back(waypoints[idx]);
                }
                full_path.push_back(ends[ei]);

                // 更新最优路径
                double total_dist = calculate_path_distance(full_path);
                if (total_dist < best_total_distance) {
                    best_total_distance = total_dist;
                    best_full_path = full_path;
                    best_start_idx = static_cast<int>(si);
                    best_end_idx = static_cast<int>(ei);
                }
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double exec_time = duration.count() / 1000.0;

        RouteResult result;
        result.path = best_full_path;
        result.total_distance = best_total_distance;
        result.execution_time_ms = exec_time;
        result.start_index = best_start_idx;
        result.end_index = best_end_idx;

        return result;
    }

    // 清理距离缓存
    void clear_distance_cache() {
        dist_calc.clear_distance_cache();
    }
};

// ===== 随机生成坐标 =====
std::vector<Coordinate> generate_random_coordinates(int count, double lon_min, double lon_max,
                                                    double lat_min, double lat_max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> lon_dist(lon_min, lon_max);
    std::uniform_real_distribution<double> lat_dist(lat_min, lat_max);

    std::vector<Coordinate> coords;
    for (int i = 0; i < count; ++i) {
        coords.emplace_back(lon_dist(gen), lat_dist(gen));
    }
    return coords;
}

void print_coordinates(const std::vector<Coordinate>& coords, const std::string& title) {
    std::cout << "===== " << title << " =====" << std::endl;
    for (size_t i = 0; i < coords.size(); ++i) {
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "  点" << (i+1) << ": 经度=" << coords[i].longitude
                  << ", 纬度=" << coords[i].latitude << std::endl;
    }
}

void print_result(const RouteResult& result) {
    std::cout << "\n===== 路线规划结果 =====" << std::endl;
    std::cout << "总距离: " << std::fixed << std::setprecision(2) << result.total_distance << " 米" << std::endl;
    std::cout << "计算耗时: " << std::fixed << std::setprecision(0) << result.execution_time_ms << " 毫秒" << std::endl;
    std::cout << "路线顺序 (" << result.path.size() << "个点):" << std::endl;

    for (size_t i = 0; i < result.path.size(); ++i) {
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "  " << (i+1) << ". 经度=" << result.path[i].longitude
                  << ", 纬度=" << result.path[i].latitude << std::endl;
    }

    DistanceCalculator dist_calc;
    double verify_dist = 0.0;
    }

int main() {
    // 可自定义配置
    AlgorithmConfig config(0.995, 10000.0, 100000);
    config.use_cache = true;
    config.enable_local_search = true;

    // 让用户输入各点数量
    int num_starts, num_waypoints, num_ends;
    std::cout << "请输入起点数量: ";
    std::cin >> num_starts;
    std::cout << "请输入途经点数量: ";
    std::cin >> num_waypoints;
    std::cout << "请输入终点数量: ";
    std::cin >> num_ends;

    // 验证输入有效性
    if (num_starts <= 0 || num_waypoints < 0 || num_ends <= 0) {
        std::cout << "输入数量必须为正整数！" << std::endl;
        return 1;
    }

    // 生成随机坐标（范围：中国东部某区域）
    auto starts = generate_random_coordinates(num_starts, 120.05, 120.25, 30.18, 30.38);
    auto waypoints = generate_random_coordinates(num_waypoints, 120.05, 120.25, 30.18, 30.38);
    auto ends = generate_random_coordinates(num_ends, 120.05, 120.25, 30.18, 30.38);

    // 打印所有点信息
    print_coordinates(starts, "待选起点");
    print_coordinates(waypoints, "途经点");
    print_coordinates(ends, "待选终点");

    // 执行路径规划
    RoutePlanner planner(config);
    RouteResult result = planner.plan_route(starts, waypoints, ends);
    print_result(result);

    // 大规模计算后，释放缓存
    planner.clear_distance_cache();

    return 0;
}
