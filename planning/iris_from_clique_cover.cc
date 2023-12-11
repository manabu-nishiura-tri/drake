#include "drake/planning/iris_from_clique_cover.h"

#include <algorithm>
#include <memory>
#include <thread>
#include <utility>
#include <vector>
#include <iostream>

#include "drake/common/random.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/adjacency_matrix_builder_base.h"
#include "drake/planning/approximate_convex_cover_builder_base.h"
#include "drake/planning/coverage_checker_via_bernoulli_test.h"
#include "drake/planning/iris_region_from_clique_builder.h"
#include "drake/planning/rejection_sampler.h"
#include "drake/planning/uniform_set_sampler.h"

namespace drake {
namespace planning {
using geometry::optimization::ConvexSets;
using geometry::optimization::HPolyhedron;

namespace {
using Eigen::Triplet;

class ConvexObstaclesVisibilityMaker final : public AdjacencyMatrixBuilderBase {
 public:
  explicit ConvexObstaclesVisibilityMaker(const ConvexSets& obstacles)
      : obstacles_{obstacles} {}

  // TODO(Alexandre.Amice) make this more robust and parallel
  Eigen::SparseMatrix<bool> DoBuildAdjacencyMatrix(
      const Eigen::Ref<const Eigen::MatrixXd>& points) const {
    const int num_points = points.cols();

    auto IsVisible = [this](const Eigen::Ref<const Eigen::VectorXd>& p1,
                            const Eigen::Ref<const Eigen::VectorXd>& p2) {
      const int num_sample_points = 100;
      const double delta = 1.0 / (num_sample_points - 1);
      double t = 0;
      while (t <= 1) {
        for (const auto& obstacle : obstacles_) {
          if (obstacle->PointInSet(t * p1 + (1 - t) * p2)) {
            return false;
          }
        }
        t += delta;
      }
      return true;
    };

    std::vector<Triplet<bool>> visibility_edges;
    for (int i = 0; i < num_points; ++i) {
      for (int j = i + 1; j < num_points; ++j) {
        if (IsVisible(points.col(i), points.col(j))) {
          visibility_edges.emplace_back(i, j, true);
          visibility_edges.emplace_back(j, i, true);
        }
      }
    }
    Eigen::SparseMatrix<bool> adjacency_matrix(num_points, num_points);
    adjacency_matrix.setFromTriplets(visibility_edges.begin(),
                                     visibility_edges.end());
    return adjacency_matrix;
  }

 private:
  const ConvexSets obstacles_;
};
}  // namespace

void IrisFromCliqueCover(const ConvexSets& obstacles, const HPolyhedron& domain,
                         const IrisFromCliqueCoverOptions& options,
                         std::vector<copyable_unique_ptr<HPolyhedron>>* sets) {
  // Convert the concrete HPolyhedrons to ConvexSets
  ConvexSets abstract_sets;
  abstract_sets.reserve(sets->size());
  for (const auto& set : *sets) {
    abstract_sets.emplace_back(std::move(set));
  }
  // The pointers in sets are now owned by abstract sets
  sets->clear();

  // The sampling distribution for the domain of the configuration space.
  std::shared_ptr<PointSamplerBase> sampler =
      options.point_sampler.has_value()
          ? options.point_sampler.value()
          : std::make_shared<UniformSetSampler<HPolyhedron>>(domain);

  // Define the coverage checker.
  const std::function<bool(const Eigen::Ref<const Eigen::VectorXd>&)>&
      reject_in_collision =
          [&obstacles](const Eigen::Ref<const Eigen::VectorXd>& sample) {
            for (const auto& obstacle : obstacles) {
              if (obstacle->PointInSet(sample)) {
                return true;
              }
            }
            return false;
          };
  std::unique_ptr<PointSamplerBase> rejection_collision_sampler =
      std::make_unique<RejectionSampler>(sampler, reject_in_collision);
  std::unique_ptr<CoverageCheckerBase> coverage_checker =
      std::make_unique<CoverageCheckerViaBernoulliTest>(
          options.coverage_termination_threshold,
          options.num_points_per_coverage_check,
          std::move(rejection_collision_sampler));

  // Define the PointSampler for points in the cliques.
  const std::function<bool(const Eigen::Ref<const Eigen::VectorXd>&)>&
      reject_in_collision_and_sets =
          [&obstacles,
           &abstract_sets](const Eigen::Ref<const Eigen::VectorXd>& sample) {
            for (const auto& obstacle : obstacles) {
              if (obstacle->PointInSet(sample)) {
                return true;
              }
            }
            for (const auto& set : abstract_sets) {
              if (set->PointInSet(sample)) {
                return true;
              }
            }
            return false;
          };
  std::unique_ptr<PointSamplerBase> point_sampler =
      std::make_unique<RejectionSampler>(sampler, reject_in_collision_and_sets);

  // Define the adjacency matrix builder.
  std::unique_ptr<AdjacencyMatrixBuilderBase> adjacency_matrix_builder =
      std::make_unique<ConvexObstaclesVisibilityMaker>(obstacles);

  // Define the set builders.
  const int max_concurrency{
      std::max(static_cast<int>(std::thread::hardware_concurrency()) - 1, 1)};
  const int num_builders =
      options.num_builders < 1 ? max_concurrency : options.num_builders;
  std::vector<std::unique_ptr<ConvexSetFromCliqueBuilderBase>> set_builders;
  set_builders.reserve(num_builders);
  for (int i = 0; i < num_builders; ++i) {
    set_builders.emplace_back(std::make_unique<IrisRegionFromCliqueBuilder>(
        obstacles, domain, options.iris_options,
        options.rank_tol_for_lowner_john_ellipse));
  }

  ApproximateConvexCoverFromCliqueCoverOptions
      approximate_convex_cover_from_clique_cover_options;
  // Cliques need to generate full dimensional Lowner John ellipsoids. So we
  // ensure that we always retrieve cliques of size at least 1 + the ambient
  // dimension.
  const int minimum_clique_size =
      std::max(options.minimum_clique_size,
               static_cast<int>(domain.ambient_dimension() + 1));
  approximate_convex_cover_from_clique_cover_options.minimum_clique_size =
      minimum_clique_size;
  approximate_convex_cover_from_clique_cover_options.num_sampled_points =
      options.num_points_per_visibility_round;
  ApproximateConvexCoverFromCliqueCover(
      coverage_checker.get(), point_sampler.get(),
      adjacency_matrix_builder.get(), options.max_clique_solver.get(),
      set_builders, approximate_convex_cover_from_clique_cover_options,
      &abstract_sets);

  // Now put the HPolyhedra back into sets.
  sets->reserve(abstract_sets.size());
  int set_index = 0;
  for (auto& abstract_set : abstract_sets) {
    std::unique_ptr<HPolyhedron> set{
        dynamic_cast<HPolyhedron*>(abstract_set.release())};
    auto A = set->A();
    auto b = set->b();
    std::cout<<"set "<<set_index<<std::endl;
    std::cout<<"  A :"<<", rows:"<<A.rows()<<", cols:"<<A.cols()<<std::endl;
    std::cout<<"  b :"<<", rows:"<<b.rows()<<", cols:"<<b.cols()<<std::endl;
    /*
    for (int i=0;i<A.rows();i++) {
      for (int j=0;j<A.cols();i++) {
        std::cout<<A(i,j)<<",";
      }
      std::cout<<"\n";
    }
    std::cout<<"\n";
    std::cout<<"  b :"<<", rows:"<<b.rows()<<", cols:"<<b.cols()<<std::endl;
    for (int i=0;i<b.rows();i++) {
      for (int j=0;j<b.cols();i++) {
        std::cout<<b(i,j)<<",";
      }
      std::cout<<"\n";
    }
    std::cout<<"\n";
    */
    sets->emplace_back(std::move(set));
    set_index += 1;
  }
}

}  // namespace planning
}  // namespace drake
