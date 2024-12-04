//
// Created by faliszewskii on 02.12.24.
//

#ifndef ELASTIC_DEFORMATION_SIMULATION_ELASTICCUBE_H
#define ELASTIC_DEFORMATION_SIMULATION_ELASTICCUBE_H


#include <array>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <glm/detail/type_quat.hpp>
#include "../collision/CollisionBox.h"

class ElasticCube {
public:
    float cubeSize = 1;
    float gravity;
    bool gravityOn;
    bool frameOn;

    std::array<glm::vec3, 64> positions{};
    std::array<glm::vec3, 64> velocities{};

    std::array<std::vector<int>, 64> sideNeighbours;
    std::array<std::vector<int>, 64> diagonalNeighbours;
    std::array<std::vector<int>, 64> farNeighbours;
    std::array<std::vector<int>, 64> cornerNeighbours;

    bool farSprings;
    bool cornerSprings;

    glm::vec3 steeringTranslation;
    glm::quat steeringRotation;

    float m;
    float c1;
    float c2;
    float k;

    CollisionBox collisionBox;

    float timeStepMs;
public:
    ElasticCube();

    void reset();

    void disturbPos(float strength = 0.1f);
    void disturbVel(float strength = 0.1f);

    void advanceByStep();

    std::array<glm::vec3, 16> getSide(int i);
    bool isInRange(int x, int y, int z);
    static int fromIndex3D(int x, int y, int z);
    static std::array<int, 3> toIndex3D(int index);

    glm::mat4 getSteeringMatrix() const;
};


#endif //ELASTIC_DEFORMATION_SIMULATION_ELASTICCUBE_H
