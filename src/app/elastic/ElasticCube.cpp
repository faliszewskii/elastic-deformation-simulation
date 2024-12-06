//
// Created by faliszewskii on 02.12.24.
//

#include <random>
#include "ElasticCube.h"
#include "../entity/point/Point.h"
#include "glm/gtx/string_cast.hpp"


ElasticCube::ElasticCube() : collisionBox(3) {
    farSprings = true;
    cornerSprings = true;
    timeStepMs = 2;
    gravity = 9.81;
    gravityOn = false;
    frameOn = false;
    cubeSize = 1.f;
    m = 0.1;
    c1 = 50;
    c2 = 10;
    k = 1;
    steeringTranslation = {};
    steeringRotation = glm::identity<glm::quat>();
    collisionBox = CollisionBox(3);

    reset();

    std::vector<std::array<int, 3>> sideIndices = {
            {1, 0, 0},
            {-1, 0, 0},
            {0, 1, 0},
            {0, -1, 0},
            {0, 0, 1},
            {0, 0, -1},
    };

    std::vector<std::array<int, 3>> diagonalIndices = {
            {0, 1, 1},
            {0, -1, 1},
            {0, -1, -1},
            {0, 1, -1},
            {1, 0, 1},
            {-1, 0, 1},
            {-1, 0, -1},
            {1, 0, -1},
            {1, 1, 0},
            {-1, 1, 0},
            {-1, -1, 0},
            {1, -1, 0},
    };

    std::vector<std::array<int, 3>> farIndices = {
            {1, 1, 1},
            {1, 1, -1},
            {1, -1, -1},
            {-1, -1, -1},
            {-1, 1, -1},
            {-1, 1, 1},
            {-1, -1, 1},
            {1, -1, 1},
    };

    for(int i = 0; i < 64; i++) {
        auto [x, y, z] = toIndex3D(i);
        // Side neighbours
        for(auto [dx, dy, dz] : sideIndices) {
            if(isInRange(x+dx, y+dy, z+dz)) {
                sideNeighbours[i].push_back(fromIndex3D(x + dx, y + dy, z + dz));
            }
        }
        // Diagonal neighbours
        for(auto [dx, dy, dz] : diagonalIndices) {
            if(isInRange(x+dx, y+dy, z+dz)) {
                diagonalNeighbours[i].push_back(fromIndex3D(x + dx, y + dy, z + dz));
            }
        }
        // Far neighbours
        for(auto [dx, dy, dz] : farIndices) {
            if(isInRange(x+dx, y+dy, z+dz)) {
                farNeighbours[i].push_back(fromIndex3D(x + dx, y + dy, z + dz));
            }
        }
    }
}

bool ElasticCube::isInRange(int x, int y, int z) { return x >= 0 && y >= 0 && z >= 0 && x < 4 && y < 4 && z < 4; }
int ElasticCube::fromIndex3D(int x, int y, int z) { return x + y * 4 + z * 16; }
std::array<int, 3> ElasticCube::toIndex3D(int index) { return {index % 4, (index / 4) % 4, index / 16}; }



void ElasticCube::advanceByStep() {
    float h = timeStepMs / 1000.f;

    glm::mat4 steeringMatrix = getSteeringMatrix();

    for (int i = 0; i < 64; i++) {
        glm::vec3 v = velocities[i];
        glm::vec3 p = positions[i];

        glm::vec3 k1_v = computeAcceleration(i, p, v);
        glm::vec3 k1_p = v;

        glm::vec3 k2_v = computeAcceleration(i, p + 0.5f * h * k1_p, v + 0.5f * h * k1_v);
        glm::vec3 k2_p = v + 0.5f * h * k1_v;

        glm::vec3 k3_v = computeAcceleration(i, p + 0.5f * h * k2_p, v + 0.5f * h * k2_v);
        glm::vec3 k3_p = v + 0.5f * h * k2_v;

        glm::vec3 k4_v = computeAcceleration(i, p + h * k3_p, v + h * k3_v);
        glm::vec3 k4_p = v + h * k3_v;

        positions[i] += (h / 6.f) * (k1_p + 2.f * k2_p + 2.f * k3_p + k4_p);
        velocities[i] += (h / 6.f) * (k1_v + 2.f * k2_v + 2.f * k3_v + k4_v);

        collisionBox.updateParticle(positions[i], velocities[i]);
    }
}

glm::vec3 ElasticCube::computeAcceleration(int i, glm::vec3 p, glm::vec3 v) {
    glm::mat4 steeringMatrix = getSteeringMatrix();
    glm::vec3 a = -k * v;
    float l0Side = cubeSize / 3;

    for (auto &neighbour : sideNeighbours[i]) {
        float l = glm::length(p - positions[neighbour]) - l0Side;
        glm::vec3 d = glm::normalize(p - positions[neighbour]);
        a -= c1 * l * d;
    }

    float l0Diag = (float)std::numbers::sqrt2 * cubeSize / 3;
    for (auto &neighbour : diagonalNeighbours[i]) {
        float l = glm::length(p - positions[neighbour]) - l0Diag;
        glm::vec3 d = glm::normalize(p - positions[neighbour]);
        a -= c1 * l * d;
    }

    if (farSprings) {
        float l0Far = (float)std::numbers::sqrt3 * cubeSize / 3;
        for (auto &neighbour : farNeighbours[i]) {
            float l = glm::length(p - positions[neighbour]) - l0Far;
            glm::vec3 d = glm::normalize(p - positions[neighbour]);
            a -= c1 * l * d;
        }
    }

    if (gravityOn) a += glm::vec3(0, -m * gravity, 0);

    if (frameOn) {
        auto [x, y, z] = toIndex3D(i);
        if ((x == 0 || x == 3) && (y == 0 || y == 3) && (z == 0 || z == 3)) {
            auto pFrame = glm::vec3(z, y, x);
            pFrame = pFrame * cubeSize / 3.f - cubeSize / 2;
            pFrame = glm::vec3(steeringMatrix * glm::vec4(pFrame, 1));
            float l = glm::length(p - pFrame);
            glm::vec3 d = glm::normalize(p - pFrame);
            if (!std::isnan(d.x)) {
                a -= c2 * l * d;
            }
        }
    }

    return a / m;
}


glm::mat4 ElasticCube::getSteeringMatrix() const {
    auto steeringMatrix = glm::identity<glm::mat4>();
    auto eulerRotation = glm::eulerAngles(steeringRotation);
    glm::mat4 Rx = {
            {1,0,0,0},
            {0, cos(eulerRotation.x), sin(eulerRotation.x), 0},
            {0, -sin(eulerRotation.x), cos(eulerRotation.x), 0},
            {0, 0, 0, 1}
    };
    glm::mat4 Ry = {
            {cos(eulerRotation.y),0,-sin(eulerRotation.y),0},
            {0, 1, 0, 0},
            {sin(eulerRotation.y), 0, cos(eulerRotation.y), 0},
            {0, 0, 0, 1}
    };
    glm::mat4 Rz = {
            {cos(eulerRotation.z),sin(eulerRotation.z),0,0},
            {-sin(eulerRotation.z), cos(eulerRotation.z), 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
    };
    steeringMatrix = glm::translate(steeringMatrix, steeringTranslation) * Rz * Ry * Rx;
    return steeringMatrix;
}

void ElasticCube::disturbPos(float strength) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-1.0, 1.0);

    for(auto &position : positions) {
//        position.y *= 1.1;
        position.x += dist(gen) * strength;
        position.y += dist(gen) * strength;
        position.z += dist(gen) * strength;
    }
}

void ElasticCube::disturbVel(float strength) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-1.0, 1.0);

    for(auto &velocity : velocities) {
//        position.y *= 1.1;
        velocity.x += dist(gen) * strength;
        velocity.y += dist(gen) * strength;
        velocity.z += dist(gen) * strength;
    }
}

std::array<glm::vec3, 16> ElasticCube::getSide(int side) {
    std::array<glm::vec3, 16> sidePoints{};

    switch (side) {
        case 0: // -X face
            for (int y = 0; y < 4; ++y)
                for (int z = 0; z < 4; ++z)
                    sidePoints[y * 4 + z] = positions[fromIndex3D(0, y, z)];
            break;
        case 1: // +X face
            for (int y = 0; y < 4; ++y)
                for (int z = 0; z < 4; ++z)
                    sidePoints[(3-y) * 4 + z] = positions[fromIndex3D(3, y, z)];
            break;
        case 2: // -Y face
            for (int x = 0; x < 4; ++x)
                for (int z = 0; z < 4; ++z)
                    sidePoints[x * 4 + (3-z)] = positions[fromIndex3D(x, 0, z)];
            break;
        case 3: // +Y face
            for (int x = 0; x < 4; ++x)
                for (int z = 0; z < 4; ++z)
                    sidePoints[x * 4 + z] = positions[fromIndex3D(x, 3, z)];
            break;
        case 4: // -Z face
            for (int x = 0; x < 4; ++x)
                for (int y = 0; y < 4; ++y)
                    sidePoints[x * 4 + y] = positions[fromIndex3D(x, y, 0)];
            break;
        case 5: // +Z face
            for (int x = 0; x < 4; ++x)
                for (int y = 0; y < 4; ++y)
                    sidePoints[x * 4 + (3-y)] = positions[fromIndex3D(x, y, 3)];
            break;
        default:
            throw std::out_of_range("Invalid side index. Must be 0..5.");
    }
    return sidePoints;
}

void ElasticCube::reset() {
    velocities = {};
    float halfSize = cubeSize / 2.f;
    float distance = cubeSize / 3.f;
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            for(int k = 0; k < 4; k++) {
                positions[i*16 + j*4 + k] = glm::vec3{
                        -halfSize + float(i) * distance,
                        -halfSize + float(j) * distance,
                        -halfSize + float(k) * distance
                };
            }
        }
    }
}
