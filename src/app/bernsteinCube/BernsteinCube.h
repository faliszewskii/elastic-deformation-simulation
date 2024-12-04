//
// Created by faliszewskii on 02.12.24.
//

#ifndef ELASTIC_DEFORMATION_SIMULATION_BERNSTEINCUBE_H
#define ELASTIC_DEFORMATION_SIMULATION_BERNSTEINCUBE_H


#include <glm/vec3.hpp>
#include <array>
#include <vector>

class BernsteinCube {

    std::array<glm::vec3, 64>& points;

public:
    explicit BernsteinCube(std::array<glm::vec3, 64>& points) : points(points) {};

    glm::vec3 evaluate(glm::vec3 uvw);
    glm::vec4 descendingAlgorithm(float t, int n);
};


#endif //ELASTIC_DEFORMATION_SIMULATION_BERNSTEINCUBE_H
