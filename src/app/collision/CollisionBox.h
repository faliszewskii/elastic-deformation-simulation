//
// Created by faliszewskii on 03.12.24.
//

#ifndef ELASTIC_DEFORMATION_SIMULATION_COLLISIONBOX_H
#define ELASTIC_DEFORMATION_SIMULATION_COLLISIONBOX_H


#include <glm/vec3.hpp>
#include <vector>
#include <functional>

class CollisionBox {
    std::vector<std::function<bool(glm::vec3)>> collisionTriggers;
    std::vector<glm::vec3> plane;
    std::vector<glm::vec3> planeNormals;
    float reflectionCoefficient;
public:
    CollisionBox(float side);

    void updateParticle(glm::vec3 &position, glm::vec3 &velocity);

    static glm::vec3 elMul(glm::vec3 u, glm::vec3 v) ;
};



#endif //ELASTIC_DEFORMATION_SIMULATION_COLLISIONBOX_H
