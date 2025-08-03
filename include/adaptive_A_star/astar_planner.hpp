#ifndef TVVF_VO_C_CORE_ASTAR_PLANNER_HPP_
#define TVVF_VO_C_CORE_ASTAR_PLANNER_HPP_

#include "adaptive_A_star/types.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace tvvf_vo_c {

/**
 * @brief A*アルゴリズムによる経路計画クラス
 */
class AStarPathPlanner {
public:
    /**
     * @brief コンストラクタ
     * @param grid 占有格子地図
     * @param resolution グリッドの解像度 [m/cell]
     * @param origin グリッドの原点座標
     * @param wall_clearance_distance 壁からの最小距離 [m]
     * @param min_passage_width 最小通行可能幅 [m]
     * @param enable_narrow_passage_mode 狭い通路モード有効化
     */
    AStarPathPlanner(const nav_msgs::msg::OccupancyGrid& grid,
                     double resolution, const Position& origin,
                     double wall_clearance_distance,
                     double min_passage_width = 0.15,
                     bool enable_narrow_passage_mode = true);

    /**
     * @brief A*アルゴリズムによる経路計画
     * @param start_pos 開始位置（世界座標）
     * @param goal_pos 目標位置（世界座標）
     * @return 計画された経路（失敗時はstd::nullopt）
     */
    std::optional<Path> plan_path(const Position& start_pos, const Position& goal_pos);

    /**
     * @brief 動的障害物を考慮したA*経路計画
     * @param start_pos 開始位置（世界座標）
     * @param goal_pos 目標位置（世界座標）
     * @param dynamic_obstacles 動的障害物情報
     * @return 計画された経路（失敗時はstd::nullopt）
     */
    std::optional<Path> plan_path_with_dynamic_obstacles(
        const Position& start_pos, 
        const Position& goal_pos,
        const std::vector<std::pair<Position, double>>& dynamic_obstacles);


    /**
     * @brief 世界座標をグリッド座標に変換
     * @param world_pos 世界座標
     * @return グリッド座標
     */
    std::pair<int, int> world_to_grid(const Position& world_pos) const;

    /**
     * @brief グリッド座標を世界座標に変換
     * @param grid_pos グリッド座標
     * @return 世界座標
     */
    Position grid_to_world(const std::pair<int, int>& grid_pos) const;

    /**
     * @brief グリッド位置が有効かチェック（膨張マップ使用）
     * @param grid_pos グリッド座標
     * @return 有効性
     */
    bool is_valid_position(const std::pair<int, int>& grid_pos) const;

    /**
     * @brief グリッド位置が有効かチェック（指定膨張マップ使用）
     * @param grid_pos グリッド座標
     * @param inflated_grid 使用する膨張マップ
     * @return 有効性
     */
    bool is_valid_position(const std::pair<int, int>& grid_pos, 
                          const std::vector<std::vector<int>>& inflated_grid) const;

    /**
     * @brief 膨張マップを取得
     * @return 膨張マップの参照
     */
    const std::vector<std::vector<int>>& get_inflated_grid() const;


    /**
     * @brief 8近傍の有効な隣接セルを取得
     * @param position グリッド座標
     * @return 有効な隣接座標のリスト
     */
    std::vector<std::pair<int, int>> get_neighbors(const std::pair<int, int>& position) const;

    /**
     * @brief 8近傍の有効な隣接セルを取得（指定膨張マップ使用）
     * @param position グリッド座標
     * @param inflated_grid 使用する膨張マップ
     * @return 有効な隣接座標のリスト
     */
    std::vector<std::pair<int, int>> get_neighbors(const std::pair<int, int>& position, 
                                                  const std::vector<std::vector<int>>& inflated_grid) const;

private:
    // グリッドデータ
    int width_;
    int height_;
    double resolution_;
    Position origin_;
    double wall_clearance_distance_;
    double min_passage_width_;               // 最小通行可能幅 [m]
    bool enable_narrow_passage_mode_;        // 狭い通路モード有効化
    
    // 占有格子データ
    std::vector<std::vector<int>> grid_;
    std::vector<std::vector<int>> inflated_grid_;
    std::vector<std::vector<int>> custom_inflated_grid_;  // カスタム膨張マップ
    std::vector<std::vector<double>> distance_field_;     // 距離場（障害物からの最短距離）
    
    // 占有閾値
    static constexpr int OCCUPIED_THRESHOLD = 65;
    static constexpr int FREE_THRESHOLD = 25;

    /**
     * @brief 壁からの距離を考慮した膨張マップを作成
     */
    void create_inflated_grid();

    /**
     * @brief カスタム膨張マップを作成（指定距離で）
     * @param clearance_distance 膨張距離
     */
    void create_custom_inflated_grid(double clearance_distance);

    /**
     * @brief 動的障害物を考慮した一時的な占有グリッドを作成
     * @param dynamic_obstacles 動的障害物情報（位置と半径のペア）
     * @return 動的障害物を反映した占有グリッド
     */
    std::vector<std::vector<int>> create_temporary_grid_with_obstacles(
        const std::vector<std::pair<Position, double>>& dynamic_obstacles);

    /**
     * @brief 指定された制約でA*経路計画を実行
     * @param start_grid 開始位置（グリッド座標）
     * @param goal_grid 目標位置（グリッド座標）
     * @param clearance_distance 使用する膨張距離
     * @param inflated_grid 使用する膨張マップ
     * @return 計画された経路（失敗時はstd::nullopt）
     */
    std::optional<Path> plan_path_with_clearance(const std::pair<int, int>& start_grid,
                                               const std::pair<int, int>& goal_grid,
                                               double clearance_distance,
                                               const std::vector<std::vector<int>>& inflated_grid);

    /**
     * @brief 動的制約選択でA*経路計画を実行
     * @param start_grid 開始位置（グリッド座標）
     * @param goal_grid 目標位置（グリッド座標）
     * @return 計画された経路（失敗時はstd::nullopt）
     */
    std::optional<Path> plan_path_with_dynamic_clearance(const std::pair<int, int>& start_grid,
                                                        const std::pair<int, int>& goal_grid);

    /**
     * @brief 通路幅を計算（指定位置での最小幅）
     * @param grid_pos グリッド座標
     * @return 通路幅 [m]
     */
    double calculate_passage_width(const std::pair<int, int>& grid_pos) const;

    /**
     * @brief 通路中央からの距離を計算
     * @param grid_pos グリッド座標
     * @return 通路中央からの距離 [m]（値が小さいほど中央に近い）
     */
    double calculate_distance_from_passage_center(const std::pair<int, int>& grid_pos) const;

    /**
     * @brief 動的制約で位置の有効性をチェック
     * @param grid_pos グリッド座標
     * @return 有効性
     */
    bool is_valid_position_dynamic(const std::pair<int, int>& grid_pos) const;

    /**
     * @brief 動的制約で隣接セルを取得
     * @param position グリッド座標
     * @return 有効な隣接座標のリスト
     */
    std::vector<std::pair<int, int>> get_neighbors_dynamic(const std::pair<int, int>& position) const;

    /**
     * @brief ユークリッド距離ヒューリスティック
     * @param pos1 位置1
     * @param pos2 位置2
     * @return ヒューリスティック距離
     */
    double heuristic(const std::pair<int, int>& pos1, const std::pair<int, int>& pos2) const;

    /**
     * @brief 移動コスト計算（対角線移動は√2倍）
     * @param from_pos 移動元
     * @param to_pos 移動先
     * @return 移動コスト
     */
    double get_movement_cost(const std::pair<int, int>& from_pos, 
                            const std::pair<int, int>& to_pos) const;

    /**
     * @brief ゴールノードから経路を再構築
     * @param goal_node ゴールノード
     * @return 再構築された経路
     */
    Path reconstruct_path(std::shared_ptr<AStarNode> goal_node) const;

    /**
     * @brief 経路をスムージング
     * @param path 元の経路
     * @return スムージングされた経路
     */
    Path smooth_path(const Path& path) const;

    /**
     * @brief 2点間の視線チェック（障害物なしの直線経路）
     * @param start 開始位置
     * @param end 終了位置
     * @return 障害物がない場合true
     */
    bool has_line_of_sight(const Position& start, const Position& end) const;

    /**
     * @brief 距離場を事前計算（BFSベース）
     */
    void compute_distance_field();

    /**
     * @brief ペアのハッシュ関数
     */
    struct PairHash {
        std::size_t operator()(const std::pair<int, int>& p) const {
            return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
        }
    };
};

} // namespace tvvf_vo_c

#endif // TVVF_VO_C_CORE_ASTAR_PLANNER_HPP_