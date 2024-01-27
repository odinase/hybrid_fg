#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridNonlinearISAM.h>
#include <gtsam/hybrid/MixtureFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <range/v3/all.hpp>

#include "hybrid_fg/types.hpp"
#include "hybrid_fg/utils/fmt.hpp"

using gtsam::symbol_shorthand::M;
using gtsam::symbol_shorthand::X;

using namespace gtsam;

/* ***************************************************************************
 */
using MotionModel = BetweenFactor<double>;

// Test fixture with switching network.
struct Switching
{
    size_t K;
    DiscreteKeys modes;
    HybridNonlinearFactorGraph nonlinearFactorGraph;
    HybridGaussianFactorGraph linearizedFactorGraph;
    Values linearizationPoint;

    /**
     * @brief Create with given number of time steps.
     *
     * @param K The total number of timesteps.
     * @param between_sigma The stddev between poses.
     * @param prior_sigma The stddev on priors (also used for measurements).
     * @param measurements Vector of measurements for each timestep.
     */
    Switching(size_t K, double between_sigma = 1.0, double prior_sigma = 0.1, std::vector<double> measurements = {},
              std::string discrete_transition_prob = "1/2 3/2")
    : K(K)
    {
        using noiseModel::Isotropic;

        // Create DiscreteKeys for binary K modes.
        for (size_t k = 0; k < K; k++) {
            modes.emplace_back(M(k), 2);
        }

        // If measurements are not provided, we just have the robot moving forward.
        if (measurements.size() == 0) {
            for (size_t k = 0; k < K; k++) {
                measurements.push_back(k);
            }
        }

        // Create hybrid factor graph.
        // Add a prior on X(0).
        nonlinearFactorGraph.emplace_shared<PriorFactor<double>>(X(0), measurements.at(0),
                                                                 Isotropic::Sigma(1, prior_sigma));

        // Add "motion models".
        for (size_t k = 0; k < K - 1; k++) {
            KeyVector keys = {X(k), X(k + 1)};
            auto motion_models = motionModels(k, between_sigma);
            std::vector<NonlinearFactor::shared_ptr> components;
            for (auto&& f : motion_models) {
                components.push_back(std::dynamic_pointer_cast<NonlinearFactor>(f));
            }
            nonlinearFactorGraph.emplace_shared<MixtureFactor>(keys, DiscreteKeys{modes[k]}, components);
        }

        // Add measurement factors
        auto measurement_noise = Isotropic::Sigma(1, prior_sigma);
        for (size_t k = 1; k < K; k++) {
            nonlinearFactorGraph.emplace_shared<PriorFactor<double>>(X(k), measurements.at(k), measurement_noise);
        }

        // Add "mode chain"
        addModeChain(&nonlinearFactorGraph, discrete_transition_prob);

        // Create the linearization point.
        for (size_t k = 0; k < K; k++) {
            linearizationPoint.insert<double>(X(k), static_cast<double>(k + 1));
        }

        // The ground truth is robot moving forward
        // and one less than the linearization point
        linearizedFactorGraph = *nonlinearFactorGraph.linearize(linearizationPoint);
    }

    // Create motion models for a given time step
    static std::vector<MotionModel::shared_ptr> motionModels(size_t k, double sigma = 1.0)
    {
        auto noise_model = noiseModel::Isotropic::Sigma(1, sigma);
        auto still = std::make_shared<MotionModel>(X(k), X(k + 1), 0.0, noise_model),
             moving = std::make_shared<MotionModel>(X(k), X(k + 1), 1.0, noise_model);
        return {still, moving};
    }

    /**
     * @brief Add "mode chain" to HybridNonlinearFactorGraph from M(0) to M(K-2).
     * E.g. if K=4, we want M0, M1 and M2.
     *
     * @param fg The factor graph to which the mode chain is added.
     */
    template <typename FACTORGRAPH>
    void addModeChain(FACTORGRAPH* fg, std::string discrete_transition_prob = "1/2 3/2")
    {
        fg->template emplace_shared<DiscreteDistribution>(modes[0], "1/1");
        for (size_t k = 0; k < K - 2; k++) {
            auto parents = {modes[k]};
            fg->template emplace_shared<DiscreteConditional>(modes[k + 1], parents, discrete_transition_prob);
        }
    }
};

auto main() -> int
{
    size_t K = 15;
    std::vector<double> measurements = {0, 1, 2, 2, 2, 2, 3, 4, 5, 6, 6, 7, 8, 9, 9, 9, 10, 11, 11, 11, 11};
    // Ground truth discrete seq
    std::vector<size_t> discrete_seq = {1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0};
    // Switching example of robot moving in 1D
    // with given measurements and equal mode priors.
    Switching switching(K, 1.0, 0.1, measurements, "1/1 1/1");
    HybridNonlinearISAM isam;
    HybridNonlinearFactorGraph graph;
    Values initial;

    // gttic_(Estimation);

    // Add the X(0) prior
    graph.push_back(switching.nonlinearFactorGraph.at(0));
    initial.insert(X(0), switching.linearizationPoint.at<double>(X(0)));

    HybridGaussianFactorGraph linearized;

    for (size_t k = 1; k < K; k++) {
        // Motion Model
        graph.push_back(switching.nonlinearFactorGraph.at(k));
        // Measurement
        graph.push_back(switching.nonlinearFactorGraph.at(k + K - 1));

        initial.insert(X(k), switching.linearizationPoint.at<double>(X(k)));

        isam.update(graph, initial, 3);
        // isam.bayesTree().print("\n\n");

        graph.resize(0);
        initial.clear();
    }

    Values result = isam.estimate();
    DiscreteValues assignment = isam.assignment();

    for (auto&& [k, v] : result) {
        std::cout << Symbol(k) << ": " << v.cast<double>() << "\n";
    }

    for (auto&& [k, v] : assignment) {
        std::cout << Symbol(k) << ": " << v << "\n";
    }
}