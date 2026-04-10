#include "../common/Kinematics.h"

static const float epsilon = 0.001f;

bool vectors_symmetric(Eigen::Vector3f &v1, Eigen::Vector3f &v2) {
    return abs(v1.x() - v2.x()) < epsilon && abs(v1.y() + v2.y()) < epsilon && abs(v1.z() - v2.z()) < epsilon;
}

void test_vec(Eigen::Vector3f &joints) {
    Eigen::Vector3f joints_flipped = joints;
    joints_flipped.x() = -joints_flipped.x();
    printf("u = (%.4f, %.4f, %.4f)\n", joints.x(), joints.y(), joints.z());
    printf("f = (%.4f, %.4f, %.4f)\n\n", joints_flipped.x(), joints_flipped.y(), joints_flipped.z());

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
    printf("tgt_lu = (%.4f, %.4f, %.4f)\n", target_1->x(), target_1->y(), target_1->z());
    printf("tgt_rf = (%.4f, %.4f, %.4f)\n", target_2->x(), target_2->y(), target_2->z());
    printf("%s\n\n", (vectors_symmetric(*target_1, *target_2) ? "Match" : "Mismatch"));

    auto target_3 = common::forwards_kinematics(joints_flipped, common::LegSide::LEFT);
    if (!target_3) {
        printf("Failed to compute kinematics (flipped, left)!\n");
        return;
    }
    auto target_4 = common::forwards_kinematics(joints, common::LegSide::RIGHT);
    if (!target_4) {
        printf("Failed to compute kinematics (unflipped, leftright)!\n");
        return;
    }
    printf("tgt_lf = (%.4f, %.4f, %.4f)\n", target_3->x(), target_3->y(), target_3->z());
    printf("tgt_ru = (%.4f, %.4f, %.4f)\n", target_4->x(), target_4->y(), target_4->z());
    printf("%s.\n\n", (vectors_symmetric(*target_3, *target_4) ? "Match" : "Mismatch"));
}

/**
 * Main entry point
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[]) {
    (void)argc, argv;

    auto vec = Eigen::Vector3f(0.5f, 1.25f, -1.75f);
    test_vec(vec);

    return 0;
}