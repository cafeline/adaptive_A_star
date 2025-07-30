#include "adaptive_A_star/astar_planner.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <unordered_set>
#include <queue>
#include <limits>

namespace tvvf_vo_c {

AStarPathPlanner::AStarPathPlanner(const nav_msgs::msg::OccupancyGrid& grid,
                                   double resolution, const Position& origin,
                                   double wall_clearance_distance,
                                   double min_passage_width,
                                   bool enable_narrow_passage_mode)
    : width_(grid.info.width), height_(grid.info.height), 
      resolution_(resolution), origin_(origin),
      wall_clearance_distance_(wall_clearance_distance),
      min_passage_width_(min_passage_width),
      enable_narrow_passage_mode_(enable_narrow_passage_mode) {
    
    // 占有格子データを2D配列に変換（Python版と同じ形式）
    grid_.resize(height_, std::vector<int>(width_));
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int index = y * width_ + x;
            if (index < static_cast<int>(grid.data.size())) {
                grid_[y][x] = grid.data[index];
            } else {
                grid_[y][x] = -1;  // 不明領域として扱う
            }
        }
    }
    
    // 壁からの距離を考慮した膨張マップを作成
    create_inflated_grid();
}

std::optional<Path> AStarPathPlanner::plan_path(const Position& start_pos, const Position& goal_pos) {
    // 世界座標をグリッド座標に変換
    auto start_grid = world_to_grid(start_pos);
    auto goal_grid = world_to_grid(goal_pos);
    
    // 基本的な有効性チェック
    if (start_grid.first < 0 || start_grid.first >= width_ || 
        start_grid.second < 0 || start_grid.second >= height_ ||
        goal_grid.first < 0 || goal_grid.first >= width_ || 
        goal_grid.second < 0 || goal_grid.second >= height_) {
        return std::nullopt;
    }
    
    // 動的制約選択方式：各位置の通路幅に応じて適切な制約を動的に選択
    std::cout << "Starting path planning with dynamic clearance selection" << std::endl;
    std::cout << "Wall clearance distance: " << wall_clearance_distance_ << "m" << std::endl;
    std::cout << "Min passage width: " << min_passage_width_ << "m" << std::endl;
    
    // 動的制約でA*を実行
    auto result = plan_path_with_dynamic_clearance(start_grid, goal_grid);
    
    if (result.has_value()) {
        std::cout << "Path found with dynamic clearance selection" << std::endl;
        return result;
    }
    
    std::cout << "Path planning failed" << std::endl;
    return std::nullopt;
}

std::pair<int, int> AStarPathPlanner::world_to_grid(const Position& world_pos) const {
    int grid_x = static_cast<int>((world_pos.x - origin_.x) / resolution_);
    int grid_y = static_cast<int>((world_pos.y - origin_.y) / resolution_);
    return {grid_x, grid_y};
}

Position AStarPathPlanner::grid_to_world(const std::pair<int, int>& grid_pos) const {
    double world_x = grid_pos.first * resolution_ + origin_.x;
    double world_y = grid_pos.second * resolution_ + origin_.y;
    return Position(world_x, world_y);
}

bool AStarPathPlanner::is_valid_position(const std::pair<int, int>& grid_pos) const {
    int x = grid_pos.first;
    int y = grid_pos.second;
    
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return false;
    }
    
    // 膨張マップで占有状態をチェック
    int cell_value = inflated_grid_[y][x];
    return cell_value >= 0 && cell_value < OCCUPIED_THRESHOLD;
}

std::vector<std::pair<int, int>> AStarPathPlanner::get_neighbors(const std::pair<int, int>& position) const {
    int x = position.first;
    int y = position.second;
    std::vector<std::pair<int, int>> neighbors;
    
    // 8方向の移動（対角線も含む）
    static const std::vector<std::pair<int, int>> directions = {
        {-1, -1}, {-1, 0}, {-1, 1},
        {0, -1},           {0, 1},
        {1, -1},  {1, 0},  {1, 1}
    };
    
    for (const auto& dir : directions) {
        int new_x = x + dir.first;
        int new_y = y + dir.second;
        std::pair<int, int> new_pos = {new_x, new_y};
        
        if (is_valid_position(new_pos)) {
            neighbors.push_back(new_pos);
        }
    }
    
    return neighbors;
}

void AStarPathPlanner::create_inflated_grid() {
    inflated_grid_ = grid_;  // コピー作成
    
    // 膨張半径をグリッドセルで計算
    int inflation_cells = static_cast<int>(wall_clearance_distance_ / resolution_) + 1;
    
    // 元の占有セルを特定
    std::vector<std::pair<int, int>> occupied_cells;
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (grid_[y][x] >= OCCUPIED_THRESHOLD) {
                occupied_cells.emplace_back(x, y);
            }
        }
    }
    
    // 各占有セルの周囲を膨張
    for (const auto& cell : occupied_cells) {
        int x = cell.first;
        int y = cell.second;
        
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                int new_x = x + dx;
                int new_y = y + dy;
                
                // 境界チェック
                if (new_x >= 0 && new_x < width_ && new_y >= 0 && new_y < height_) {
                    // ユークリッド距離で膨張範囲を決定
                    double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                    if (distance <= wall_clearance_distance_) {
                        // 元が自由空間の場合のみ膨張値を設定
                        if (inflated_grid_[new_y][new_x] < OCCUPIED_THRESHOLD) {
                            inflated_grid_[new_y][new_x] = OCCUPIED_THRESHOLD;
                        }
                    }
                }
            }
        }
    }
}

double AStarPathPlanner::heuristic(const std::pair<int, int>& pos1, const std::pair<int, int>& pos2) const {
    double dx = pos1.first - pos2.first;
    double dy = pos1.second - pos2.second;
    return std::sqrt(dx * dx + dy * dy);
}

double AStarPathPlanner::get_movement_cost(const std::pair<int, int>& from_pos, 
                                          const std::pair<int, int>& to_pos) const {
    int dx = std::abs(to_pos.first - from_pos.first);
    int dy = std::abs(to_pos.second - from_pos.second);
    
    if (dx == 1 && dy == 1) {
        return std::sqrt(2.0);  // 対角線移動
    } else {
        return 1.0;  // 直線移動
    }
}

Path AStarPathPlanner::reconstruct_path(std::shared_ptr<AStarNode> goal_node) const {
    Path path;
    auto current_node = goal_node;
    std::vector<std::pair<int, int>> grid_path;
    
    // 逆順にたどって経路を構築
    while (current_node != nullptr) {
        grid_path.push_back(current_node->position);
        current_node = current_node->parent;
    }
    
    // 正順に変換して世界座標に変換
    std::reverse(grid_path.begin(), grid_path.end());
    
    for (size_t i = 0; i < grid_path.size(); ++i) {
        Position world_pos = grid_to_world(grid_path[i]);
        
        // 移動コストを計算（簡易版）
        double cost = 0.0;
        if (i > 0) {
            Position prev_world_pos = grid_to_world(grid_path[i-1]);
            cost = world_pos.distance_to(prev_world_pos);
        }
        
        path.add_point(world_pos, cost);
    }
    
    return path;
}

Path AStarPathPlanner::smooth_path(const Path& path) const {
    if (path.points.size() <= 2) {
        return path;  // 短い経路はそのまま返す
    }
    
    Path smoothed_path;
    std::vector<Position> positions;
    
    // 位置データを抽出
    for (const auto& point : path.points) {
        positions.push_back(point.position);
    }
    
    // 最初の点を追加
    smoothed_path.add_point(positions[0], 0.0);
    
    // Line of Sight（視線）アルゴリズムでスムージング
    size_t current_index = 0;
    
    while (current_index < positions.size() - 1) {
        size_t farthest_visible_index = current_index + 1;
        
        // 現在の点から見える最も遠い点を探す
        for (size_t test_index = current_index + 2; test_index < positions.size(); ++test_index) {
            if (has_line_of_sight(positions[current_index], positions[test_index])) {
                farthest_visible_index = test_index;
            } else {
                break;  // 障害物があるので探索終了
            }
        }
        
        // 次の経由点として追加
        if (farthest_visible_index < positions.size() - 1) {
            double cost = positions[current_index].distance_to(positions[farthest_visible_index]);
            smoothed_path.add_point(positions[farthest_visible_index], cost);
        }
        
        current_index = farthest_visible_index;
    }
    
    // 最後の点を追加
    if (smoothed_path.points.empty() || 
        smoothed_path.points.back().position.distance_to(positions.back()) > 1e-6) {
        double cost = smoothed_path.points.empty() ? 0.0 : 
                     smoothed_path.points.back().position.distance_to(positions.back());
        smoothed_path.add_point(positions.back(), cost);
    }
    
    return smoothed_path;
}

bool AStarPathPlanner::has_line_of_sight(const Position& start, const Position& end) const {
    // グリッド座標に変換
    auto start_grid = world_to_grid(start);
    auto end_grid = world_to_grid(end);
    
    // Bresenhamアルゴリズムで直線上の全セルをチェック
    int x0 = start_grid.first;
    int y0 = start_grid.second;
    int x1 = end_grid.first;
    int y1 = end_grid.second;
    
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int x_inc = (x0 < x1) ? 1 : -1;
    int y_inc = (y0 < y1) ? 1 : -1;
    int error = dx - dy;
    
    int x = x0;
    int y = y0;
    
    while (true) {
        // 現在のセルが占有されているかチェック
        if (!is_valid_position({x, y})) {
            return false;  // 障害物がある
        }
        
        if (x == x1 && y == y1) {
            break;  // 終点に到達
        }
        
        int error2 = 2 * error;
        if (error2 > -dy) {
            error -= dy;
            x += x_inc;
        }
        if (error2 < dx) {
            error += dx;
            y += y_inc;
        }
    }
    
    return true;  // 直線経路に障害物なし
}

// 新しいシンプルなメソッドの実装

bool AStarPathPlanner::is_valid_position(const std::pair<int, int>& grid_pos, 
                                        const std::vector<std::vector<int>>& inflated_grid) const {
    int x = grid_pos.first;
    int y = grid_pos.second;
    
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return false;
    }
    
    // 指定された膨張マップで占有状態をチェック
    int cell_value = inflated_grid[y][x];
    return cell_value >= 0 && cell_value < OCCUPIED_THRESHOLD;
}

std::vector<std::pair<int, int>> AStarPathPlanner::get_neighbors(const std::pair<int, int>& position, 
                                                                const std::vector<std::vector<int>>& inflated_grid) const {
    int x = position.first;
    int y = position.second;
    std::vector<std::pair<int, int>> neighbors;
    
    // 8方向の移動（対角線も含む）
    static const std::vector<std::pair<int, int>> directions = {
        {-1, -1}, {-1, 0}, {-1, 1},
        {0, -1},           {0, 1},
        {1, -1},  {1, 0},  {1, 1}
    };
    
    for (const auto& dir : directions) {
        int new_x = x + dir.first;
        int new_y = y + dir.second;
        std::pair<int, int> new_pos = {new_x, new_y};
        
        if (is_valid_position(new_pos, inflated_grid)) {
            neighbors.push_back(new_pos);
        }
    }
    
    return neighbors;
}

void AStarPathPlanner::create_custom_inflated_grid(double clearance_distance) {
    custom_inflated_grid_ = grid_;  // コピー作成
    
    // 膨張半径をグリッドセルで計算
    int inflation_cells = static_cast<int>(clearance_distance / resolution_) + 1;
    
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (grid_[y][x] >= OCCUPIED_THRESHOLD || grid_[y][x] == -1) {
                // 占有セルの周囲を膨張
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                            if (distance <= clearance_distance) {
                                custom_inflated_grid_[ny][nx] = OCCUPIED_THRESHOLD;
                            }
                        }
                    }
                }
            }
        }
    }
}

double AStarPathPlanner::calculate_passage_width(const std::pair<int, int>& grid_pos) const {
    int x = grid_pos.first;
    int y = grid_pos.second;
    
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return 0.0;
    }
    
    double min_distance_to_wall = std::numeric_limits<double>::max();
    
    // 周囲の占有セルまでの最短距離を計算（簡易版）
    for (int dy = -50; dy <= 50; ++dy) {  // 適度な範囲で探索
        for (int dx = -50; dx <= 50; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                if (grid_[ny][nx] >= OCCUPIED_THRESHOLD || grid_[ny][nx] == -1) {
                    double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                    min_distance_to_wall = std::min(min_distance_to_wall, distance);
                }
            }
        }
    }
    
    return min_distance_to_wall * 2.0;  // 通路幅 = 壁までの最短距離 × 2
}

double AStarPathPlanner::calculate_distance_from_passage_center(const std::pair<int, int>& grid_pos) const {
    int x = grid_pos.first;
    int y = grid_pos.second;
    
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return std::numeric_limits<double>::max();
    }
    
    double min_distance_to_wall = std::numeric_limits<double>::max();
    
    // 周囲の占有セルまでの最短距離を計算
    for (int dy = -50; dy <= 50; ++dy) {
        for (int dx = -50; dx <= 50; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                if (grid_[ny][nx] >= OCCUPIED_THRESHOLD || grid_[ny][nx] == -1) {
                    double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                    min_distance_to_wall = std::min(min_distance_to_wall, distance);
                }
            }
        }
    }
    
    // 通路中央からの距離 = 壁までの最短距離が最大になる位置が中央
    // 値が大きいほど中央に近いので、コスト用に反転
    return 1.0 / (min_distance_to_wall + 1e-6);  // 中央に近いほど小さいコスト
}

std::optional<Path> AStarPathPlanner::plan_path_with_clearance(const std::pair<int, int>& start_grid,
                                                             const std::pair<int, int>& goal_grid,
                                                             double clearance_distance,
                                                             const std::vector<std::vector<int>>& inflated_grid) {
    // A*アルゴリズム実装
    std::vector<std::shared_ptr<AStarNode>> open_set;
    std::unordered_set<std::pair<int, int>, PairHash> closed_set;
    std::unordered_map<std::pair<int, int>, std::shared_ptr<AStarNode>, PairHash> open_dict;
    
    // 開始ノード
    auto start_node = std::make_shared<AStarNode>(
        start_grid, 0.0, heuristic(start_grid, goal_grid)
    );
    open_set.push_back(start_node);
    std::push_heap(open_set.begin(), open_set.end(), [](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
        return *a > *b;  // 最小ヒープ
    });
    open_dict[start_grid] = start_node;
    
    while (!open_set.empty()) {
        // 最小コストノードを取得
        std::pop_heap(open_set.begin(), open_set.end(), [](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
            return *a > *b;
        });
        auto current_node = open_set.back();
        open_set.pop_back();
        auto current_pos = current_node->position;
        
        // 辞書からも削除
        auto dict_it = open_dict.find(current_pos);
        if (dict_it != open_dict.end()) {
            open_dict.erase(dict_it);
        }
        
        // ゴール到達チェック
        if (current_pos == goal_grid) {
            auto path = reconstruct_path(current_node);
            return smooth_path(path);
        }
        
        // クローズドセットに追加
        closed_set.insert(current_pos);
        
        // 隣接ノードを探索
        for (const auto& neighbor_pos : get_neighbors(current_pos, inflated_grid)) {
            if (closed_set.find(neighbor_pos) != closed_set.end()) {
                continue;
            }
            
            // 移動コスト計算
            double movement_cost = get_movement_cost(current_pos, neighbor_pos);
            double tentative_g_cost = current_node->g_cost + movement_cost;
            
            // 既存ノードのチェック
            auto open_it = open_dict.find(neighbor_pos);
            if (open_it != open_dict.end()) {
                auto neighbor_node = open_it->second;
                if (tentative_g_cost < neighbor_node->g_cost) {
                    // より良いパスを発見
                    neighbor_node->g_cost = tentative_g_cost;
                    neighbor_node->f_cost = tentative_g_cost + neighbor_node->h_cost;
                    neighbor_node->parent = current_node;
                    // ヒープを再構築
                    std::make_heap(open_set.begin(), open_set.end(), [](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
                        return *a > *b;
                    });
                }
            } else {
                // 新しいノードを作成
                double h_cost = heuristic(neighbor_pos, goal_grid);
                auto neighbor_node = std::make_shared<AStarNode>(
                    neighbor_pos, tentative_g_cost, h_cost, current_node
                );
                open_set.push_back(neighbor_node);
                std::push_heap(open_set.begin(), open_set.end(), [](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
                    return *a > *b;
                });
                open_dict[neighbor_pos] = neighbor_node;
            }
        }
    }
    
    // 経路が見つからない
    return std::nullopt;
}

std::optional<Path> AStarPathPlanner::plan_path_with_dynamic_clearance(const std::pair<int, int>& start_grid,
                                                                      const std::pair<int, int>& goal_grid) {
    // A*アルゴリズム実装（動的制約版）
    std::vector<std::shared_ptr<AStarNode>> open_set;
    std::unordered_set<std::pair<int, int>, PairHash> closed_set;
    std::unordered_map<std::pair<int, int>, std::shared_ptr<AStarNode>, PairHash> open_dict;
    
    // 開始ノード
    auto start_node = std::make_shared<AStarNode>(
        start_grid, 0.0, heuristic(start_grid, goal_grid)
    );
    open_set.push_back(start_node);
    std::push_heap(open_set.begin(), open_set.end(), [](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
        return *a > *b;  // 最小ヒープ
    });
    open_dict[start_grid] = start_node;
    
    while (!open_set.empty()) {
        // 最小コストノードを取得
        std::pop_heap(open_set.begin(), open_set.end(), [](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
            return *a > *b;
        });
        auto current_node = open_set.back();
        open_set.pop_back();
        auto current_pos = current_node->position;
        
        // 辞書からも削除
        auto dict_it = open_dict.find(current_pos);
        if (dict_it != open_dict.end()) {
            open_dict.erase(dict_it);
        }
        
        // ゴール到達チェック
        if (current_pos == goal_grid) {
            auto path = reconstruct_path(current_node);
            return smooth_path(path);
        }
        
        // クローズドセットに追加
        closed_set.insert(current_pos);
        
        // 隣接ノードを探索（動的制約使用）
        for (const auto& neighbor_pos : get_neighbors_dynamic(current_pos)) {
            if (closed_set.find(neighbor_pos) != closed_set.end()) {
                continue;
            }
            
            // 移動コスト計算
            double movement_cost = get_movement_cost(current_pos, neighbor_pos);
            
            // 狭い通路では中央維持コストを追加
            double passage_width = calculate_passage_width(neighbor_pos);
            double center_cost = 0.0;
            
            if (passage_width < wall_clearance_distance_ * 2.0 && passage_width >= min_passage_width_) {
                // 狭い通路の場合、中央からの距離をコストに加算
                // 通路が狭いほど中央維持の重要度を上げる
                double narrowness_factor = (wall_clearance_distance_ * 2.0 - passage_width) / (wall_clearance_distance_ * 2.0);
                center_cost = calculate_distance_from_passage_center(neighbor_pos) * narrowness_factor * 1.0;  // 重み調整
            }
            
            double tentative_g_cost = current_node->g_cost + movement_cost + center_cost;
            
            // 既存ノードのチェック
            auto open_it = open_dict.find(neighbor_pos);
            if (open_it != open_dict.end()) {
                auto neighbor_node = open_it->second;
                if (tentative_g_cost < neighbor_node->g_cost) {
                    // より良いパスを発見
                    neighbor_node->g_cost = tentative_g_cost;
                    neighbor_node->f_cost = tentative_g_cost + neighbor_node->h_cost;
                    neighbor_node->parent = current_node;
                    // ヒープを再構築
                    std::make_heap(open_set.begin(), open_set.end(), [](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
                        return *a > *b;
                    });
                }
            } else {
                // 新しいノードを作成
                double h_cost = heuristic(neighbor_pos, goal_grid);
                auto neighbor_node = std::make_shared<AStarNode>(
                    neighbor_pos, tentative_g_cost, h_cost, current_node
                );
                open_set.push_back(neighbor_node);
                std::push_heap(open_set.begin(), open_set.end(), [](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
                    return *a > *b;
                });
                open_dict[neighbor_pos] = neighbor_node;
            }
        }
    }
    
    // 経路が見つからない
    return std::nullopt;
}

bool AStarPathPlanner::is_valid_position_dynamic(const std::pair<int, int>& grid_pos) const {
    int x = grid_pos.first;
    int y = grid_pos.second;
    
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return false;
    }
    
    // 元のグリッドで占有状態をチェック
    if (grid_[y][x] >= OCCUPIED_THRESHOLD || grid_[y][x] == -1) {
        return false;
    }
    
    // 通路幅を計算
    double passage_width = calculate_passage_width(grid_pos);
    
    // 通路幅に応じた制約を動的に適用
    if (passage_width >= wall_clearance_distance_ * 2.0) {
        // 広い通路：通常制約を使用
        return inflated_grid_[y][x] < OCCUPIED_THRESHOLD;
    } else if (passage_width >= min_passage_width_) {
        // 狭い通路（但し最小幅以上）：縮小制約を使用
        return true;  // 元のグリッドでOKなら通行可能
    } else {
        // 通路が狭すぎる
        return false;
    }
}

std::vector<std::pair<int, int>> AStarPathPlanner::get_neighbors_dynamic(const std::pair<int, int>& position) const {
    int x = position.first;
    int y = position.second;
    std::vector<std::pair<int, int>> neighbors;
    
    // 8方向の移動（対角線も含む）
    static const std::vector<std::pair<int, int>> directions = {
        {-1, -1}, {-1, 0}, {-1, 1},
        {0, -1},           {0, 1},
        {1, -1},  {1, 0},  {1, 1}
    };
    
    for (const auto& dir : directions) {
        int new_x = x + dir.first;
        int new_y = y + dir.second;
        std::pair<int, int> new_pos = {new_x, new_y};
        
        if (is_valid_position_dynamic(new_pos)) {
            neighbors.push_back(new_pos);
        }
    }
    
    return neighbors;
}

} // namespace tvvf_vo_c