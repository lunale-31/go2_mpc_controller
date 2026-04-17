#include "../common/Kinematics.h"

#include <random>

// Define hip joint limits (in radians)
static const float theta_1_min = -0.85f, theta_1_max = 0.85f;

// Define thigh joint limits (in radians)
static const float theta_2_min = -0.58f, theta_2_max = 4.6f;

// Define calf joint limits (in radians)
static const float theta_3_min = -2.8f, theta_3_max = -0.95f;

// Define tolerable computation error
static const float epsilon = 0.001f;

// Define range shrinkage for randomized angle selection
static const float range_boundary = 0.15f;

// Number of test runs
static const unsigned runs = 1000000;

// Number of test runs to report after
static const unsigned report_every = 1000;

bool vectors_symmetric(Eigen::Vector3f &v1, Eigen::Vector3f &v2) {
    return abs(v1.x() - v2.x()) < epsilon && abs(v1.y() + v2.y()) < epsilon && abs(v1.z() - v2.z()) < epsilon;
}

bool vectors_equal(Eigen::Vector3f &v1, Eigen::Vector3f &v2) {
    return abs(v1.x() - v2.x()) < epsilon && abs(v1.y() - v2.y()) < epsilon && abs(v1.z() - v2.z()) < epsilon;
}

void test_vec(Eigen::Vector3f &joints) {
    Eigen::Vector3f joints_flipped = joints;
    joints_flipped.x() = -joints_flipped.x();
    printf("u = [%.4f, %.4f, %.4f]\n", joints.x(), joints.y(), joints.z());
    printf("f = [%.4f, %.4f, %.4f]\n\n", joints_flipped.x(), joints_flipped.y(), joints_flipped.z());

    auto target_1 = common::forwards_kinematics(joints, common::LegSide::LEFT);
    if (!target_1) {
        printf("Failed to compute kinematics (unflipped, left)!\n");
        return;
    }
    auto target_2 = common::forwards_kinematics(joints_flipped, common::LegSide::RIGHT);
    if (!target_2) {
        printf("Failed to compute kinematics (flipped, right)!\n");
        return;
    }
    printf("tgt_lu = [%.4f, %.4f, %.4f]\n", target_1->x(), target_1->y(), target_1->z());
    printf("tgt_rf = [%.4f, %.4f, %.4f]\n", target_2->x(), target_2->y(), target_2->z());
    printf("%s\n\n", (vectors_symmetric(*target_1, *target_2) ? "Match" : "Mismatch"));

    auto target_3 = common::forwards_kinematics(joints_flipped, common::LegSide::LEFT);
    if (!target_3) {
        printf("Failed to compute kinematics (flipped, left)!\n");
        return;
    }
    auto target_4 = common::forwards_kinematics(joints, common::LegSide::RIGHT);
    if (!target_4) {
        printf("Failed to compute kinematics (unflipped, right)!\n");
        return;
    }
    printf("tgt_lf = [%.4f, %.4f, %.4f]\n", target_3->x(), target_3->y(), target_3->z());
    printf("tgt_ru = [%.4f, %.4f, %.4f]\n", target_4->x(), target_4->y(), target_4->z());
    printf("%s.\n\n", (vectors_symmetric(*target_3, *target_4) ? "Match" : "Mismatch"));

    // check inverse kinematics
    auto target_1_inv = common::inverse_kinematics(*target_1, common::LegSide::LEFT);
    if (target_1_inv.empty()) {
        printf("Failed to compute inverse kinematics (unflipped, left)!\n");
        return;
    }
    printf("tgt_lu_inv = [%.4f, %.4f, %.4f]\n", target_1_inv[0].x(), target_1_inv[0].y(), target_1_inv[0].z());
    printf("%s.\n\n", (vectors_equal(joints, target_1_inv[0]) ? "Match" : "Mismatch"));
}

/**
 * Main entry point
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    unsigned two_solutions = 0, no_solutions = 0;

    auto vec = Eigen::Vector3f(0.5f, 1.25f, -1.75f);
    test_vec(vec);

    // Do some fuzzing
    std::random_device rd; // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> t1_rand(theta_1_min + range_boundary, theta_1_max - range_boundary);
    std::uniform_real_distribution<float> t2_rand(theta_2_min + range_boundary, theta_2_max - range_boundary);
    std::uniform_real_distribution<float> t3_rand(theta_3_min + range_boundary, theta_3_max - range_boundary);

    for (unsigned i = 0; i < runs; ++i) {
        Eigen::Vector3f joints(t1_rand(gen), t2_rand(gen), t3_rand(gen));
        auto target = common::forwards_kinematics(joints, common::LegSide::LEFT);
        if (!target) {
            printf("[!] Could not determine target position for theta = [%.4f, %.4f, %.4f].\n", joints.x(), joints.y(), joints.z());
            continue;
        }
        auto inverted = common::inverse_kinematics(*target, common::LegSide::LEFT);
        if (inverted.empty()) {
            no_solutions++;
            printf("[!] Could not determine inverted joint angles for theta = [%.4f, %.4f, %.4f].\n", joints.x(), joints.y(), joints.z());
            return 1;
            continue;
        }
        if (inverted.size() > 1) {
            two_solutions++;
            // printf("    Found multiple solutions.\n");
        }
        bool found = false;
        for (auto &i : inverted) {
            if (vectors_equal(joints, i)) {
                found = true;
                break;
            }
        }
        if (!found) {
            if (inverted.size() == 2) {
                printf("[!] Incorrectly inverted kinematics (expected [%.4f, %.4f, %.4f], but got [%.4f, %.4f, %.4f] and [%.4f, %.4f, %.4f]).\n",
                       joints.x(), joints.y(), joints.z(),
                       inverted[0].x(), inverted[0].y(), inverted[0].z(),
                       inverted[1].x(), inverted[1].y(), inverted[1].z());

            } else {
                printf("[!] Incorrectly inverted kinematics (expected [%.4f, %.4f, %.4f], but got [%.4f, %.4f, %.4f]).\n",
                       joints.x(), joints.y(), joints.z(), inverted[0].x(), inverted[0].y(), inverted[0].z());
            }
        }
    }

    printf("Ran %u tests. %u produced no and %u produced multiple solutions.\n", runs, no_solutions, two_solutions);

    return 0;
}