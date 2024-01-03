#include <iostream>
#include "drake/planning/iris_region_from_clique_builder.h"

#include "drake/common/find_resource.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "planning/robot_diagram_builder.h"

namespace drake {
namespace planning {
using Eigen::Vector3d;
using geometry::optimization::ConvexSets;
using geometry::optimization::Hyperellipsoid;
using geometry::optimization::IrisOptions;
using planning::CollisionCheckerParams;
using math::RigidTransform;
using math::RollPitchYawd;
using multibody::Parser;
using systems::Context;
using planning::RobotDiagramBuilder;

/**
 * The default configurations for running IRIS when building a convex set from a
 * clique. Currently, it is recommended to only run IRIS for one iteration when
 * building from a clique so as to avoid discarding the information gained from
 * the clique.
 */
DefaultIrisOptionsForIrisRegionFromCliqueBuilder::
    DefaultIrisOptionsForIrisRegionFromCliqueBuilder()
    : IrisOptions() {
  iteration_limit = 1;
}

IrisRegionFromCliqueBuilder::IrisRegionFromCliqueBuilder(
    const ConvexSets& obstacles, const HPolyhedron& domain,
    const IrisOptions& options, const double rank_tol_for_lowner_john_ellipse)
    : ConvexSetFromCliqueBuilderBase(),
      obstacles_{obstacles},
      domain_{domain},
      options_{options},
      rank_tol_for_lowner_john_ellipse_{rank_tol_for_lowner_john_ellipse} {}

std::unique_ptr<ConvexSet> IrisRegionFromCliqueBuilder::DoBuildConvexSet(
    const Eigen::Ref<const Eigen::MatrixXd>& clique_points) {
  const Hyperellipsoid starting_ellipse =
      Hyperellipsoid::MinimumVolumeCircumscribedEllipsoid(
          clique_points, rank_tol_for_lowner_john_ellipse_);
  options_.starting_ellipse = starting_ellipse;
  return std::move(geometry::optimization::Iris(obstacles_,
                                                starting_ellipse.center(),
                                                domain_, options_))
      .Clone();
}

IrisInConfigurationSpaceRegionFromCliqueBuilder::
    IrisInConfigurationSpaceRegionFromCliqueBuilder(
        const multibody::MultibodyPlant<double>& plant,
        const systems::Context<double>& context, const IrisOptions& options,
        const double rank_tol_for_lowner_john_ellipse)
    : plant_(plant),
      context_(context),
      options_(options),
      rank_tol_for_lowner_john_ellipse_(rank_tol_for_lowner_john_ellipse) {}

std::unique_ptr<ConvexSet>
IrisInConfigurationSpaceRegionFromCliqueBuilder::DoBuildConvexSet(
    const Eigen::Ref<const Eigen::MatrixXd>& clique_points) {
  const Hyperellipsoid starting_ellipse =
      Hyperellipsoid::MinimumVolumeCircumscribedEllipsoid(
          clique_points, rank_tol_for_lowner_john_ellipse_);
  options_.starting_ellipse = starting_ellipse;
  auto center = starting_ellipse.center();
  std::cout<<"printing center fo starting ellipse"<<std::endl;
  size_t n_col = center.cols();
  size_t n_row = center.rows();
  std::cout<<"n_col: "<<n_col<<", n_row: "<<n_row<<std::endl;
  int col_index = 0;
  int row_index = 0;
  for (size_t i = 0; i<n_row; i++) {
    col_index = 0;
    for (size_t j = 0; j<n_col; j++) {
      if (j < n_col-1) {
        std::cout<<center(row_index, col_index)<<",";
      }
      else {
        std::cout<<center(row_index, col_index)<<"."<<std::endl;
      }
      col_index += 1;
    }
    row_index += 1;
  } 
  VectorX<double> center_q(n_row);
  for (int i=0;i<row_index;i++) {
    center_q(i) = center(i,0);
    std::cout<<"center_q("<<i<<")"<<center_q(i)<<std::endl;
  }
  /*
  drake::multibody::MultibodyPlant<double> *plant_tmp;
  drake::systems::DiagramBuilder<double> builder;
  drake::geometry::SceneGraph<double>* scene_graph;
  std::tie(plant_tmp, scene_graph) = drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  const char boxes_in_corners[] = R"""(
  <robot name="boxes">
    <link name="fixed">
      <collision name="top_left">
        <origin rpy="0 0 0" xyz="-1 1 0"/>
        <geometry><box size="1 1 1"/></geometry>
      </collision>
      <collision name="top_right">
        <origin rpy="0 0 0" xyz="1 1 0"/>
        <geometry><box size="1 1 1"/></geometry>
      </collision>
      <collision name="bottom_left">
        <origin rpy="0 0 0" xyz="-1 -1 0"/>
        <geometry><box size="1 1 1"/></geometry>
      </collision>
      <collision name="bottom_right">
        <origin rpy="0 0 0" xyz="1 -1 0"/>
        <geometry><box size="1 1 1"/></geometry>
      </collision>
    </link>
    <joint name="fixed_link_weld" type="fixed">
      <parent link="world"/>
      <child link="fixed"/>
    </joint>
    <link name="movable">
      <collision name="sphere">
        <geometry><sphere radius="0.1"/></geometry>
      </collision>
    </link>
    <link name="for_joint"/>
    <joint name="x" type="prismatic">
      <axis xyz="1 0 0"/>
      <limit lower="-2" upper="2"/>
      <parent link="world"/>
      <child link="for_joint"/>
    </joint>
    <joint name="y" type="prismatic">
      <axis xyz="0 1 0"/>
      <limit lower="-2" upper="2"/>
      <parent link="for_joint"/>
      <child link="movable"/>
    </joint>
  </robot>
  )""";
  Parser parser(plant_tmp);
  parser.AddModelsFromString(boxes_in_corners, "urdf");
  plant_tmp->Finalize();
  auto diagram = builder.Build();
  auto context_tmp = plant_tmp->CreateDefaultContext();
  auto diagram_context_tmp = diagram->CreateDefaultContext();
  auto& plant_context_tmp = plant_tmp->GetMyContextFromRoot(*diagram_context_tmp);
  std::cout<<"Just Before SetPositions"<<std::endl;
  std::cout<<"Num positions: "<<plant_tmp->num_positions()<<std::endl;
  //plant_tmp->SetPositions(&plant_context_tmp.mutable_plant_context(), center_q);
  //plant_tmp->SetPositions(context_tmp.get(), center_q);
  // TODO: cast context_tmp to const Context<double>& context
  return std::move(geometry::optimization::IrisInConfigurationSpace(
                       *plant_tmp, *context_tmp, options_))
      .Clone();
  */

  /*
  // Followings are only for boxes_in_corners file.
  CollisionCheckerParams params;
  RobotDiagramBuilder<double> robod_builder(0.0);
  params.robot_model_instances = robod_builder.parser().AddModelsFromString(
      boxes_in_corners, "urdf");
  params.edge_step_size = 0.01;
  params.model = robod_builder.Build();

  //auto checker = std::make_unique<SceneGraphCollisionChecker>(std::move(params));
  //*checker.UpdateContextPositions(*checker.plant, center_q);
  SceneGraphCollisionChecker dut(std::move(params));
  //dut.UpdateContextPositions(dut.plant_context(), center_q);
  std::shared_ptr<CollisionCheckerContext> collision_context =
      dut.MakeStandaloneModelContext();
  const Context<double>& standalone_plant_context =
      dut.UpdateContextPositions(collision_context.get(), center_q);
  */

  // Load 2d arms assets.
  auto oneDoF_iiwa_asset = FindResourceOrThrow(
      "drake/planning/2d_arms/oneDOF_iiwa7_with_box_collision.sdf");
  auto twoDoF_iiwa_asset = FindResourceOrThrow(
      "drake/planning/2d_arms/twoDOF_iiwa7_with_box_collision.sdf");
  auto box_asset = FindResourceOrThrow(
      "drake/planning/2d_arms/box_small.urdf");
  auto obs_asset = FindResourceOrThrow(
      "drake/planning/2d_arms/obstacle.urdf");
  // We can use model directives file instead.

  CollisionCheckerParams params;
  RobotDiagramBuilder<double> robod_builder(0.0);
  params.robot_model_instances.push_back(
      robod_builder.parser().AddModelFromFile(box_asset));
  params.robot_model_instances.push_back(
      robod_builder.parser().AddModelFromFile(twoDoF_iiwa_asset));
  params.robot_model_instances.push_back(
      robod_builder.parser().AddModelFromFile(oneDoF_iiwa_asset));
  params.robot_model_instances.push_back(
      robod_builder.parser().AddModelFromFile(obs_asset));
  auto& plant = robod_builder.plant();

  // Weld box scene and arms to world.
  // We can use model directives file instead.
  // TODO:
  // get transform vector from parameter file or something.
  plant.WeldFrames(plant.world_frame(),
      plant.GetFrameByName(
        "base", plant.GetModelInstanceByName("box_scene")),
      RigidTransform(Vector3d{0.0, 0.0, 0.0})
      );
  plant.WeldFrames(plant.world_frame(),
      plant.GetFrameByName(
        "iiwa_twoDOF_link_0",
        plant.GetModelInstanceByName("iiwa7_twoDOF")),
      RigidTransform(
        RollPitchYawd(0.0, 0.0, -M_PI/2.).ToRotationMatrix(),
        Vector3d{0.0, 0.55, 0.0}));
  plant.WeldFrames(plant.world_frame(),
      plant.GetFrameByName(
        "iiwa_oneDOF_link_0",
        plant.GetModelInstanceByName("iiwa7_oneDOF")),
      RigidTransform(
        RollPitchYawd(0.0, 0.0, -M_PI/2.).ToRotationMatrix(),
        Vector3d{0.0, -0.55, 0.0}));
  // Get the obstacle transformation from plant_ context
  auto& obs_body = plant_.GetBodyByName("box_obs");
  auto pose = obs_body.EvalPoseInWorld(context_);
  plant.WeldFrames(plant.world_frame(),
      plant.GetFrameByName(
        "base", plant.GetModelInstanceByName("obstacle")),
      pose
      //RigidTransform(Vector3d{0.0, 0.0, 0.0})
      );

  params.edge_step_size = 0.01;
  params.model = robod_builder.Build();

  //auto checker = std::make_unique<SceneGraphCollisionChecker>(std::move(params));
  //*checker.UpdateContextPositions(*checker.plant, center_q);
  SceneGraphCollisionChecker dut(std::move(params));
  //dut.UpdateContextPositions(dut.plant_context(), center_q);
  std::shared_ptr<CollisionCheckerContext> collision_context =
      dut.MakeStandaloneModelContext();
  const Context<double>& standalone_plant_context =
      dut.UpdateContextPositions(collision_context.get(), center_q);

  return std::move(geometry::optimization::IrisInConfigurationSpace(
                       dut.plant(), standalone_plant_context, options_))
      .Clone();
}

}  // namespace planning
}  // namespace drake
