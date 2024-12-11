//
// Created by faliszewskii on 03.12.24.
//

#ifndef ELASTIC_DEFORMATION_SIMULATION_COLLISIONBOX_H
#define ELASTIC_DEFORMATION_SIMULATION_COLLISIONBOX_H


#include <glm/vec3.hpp>
#include <vector>
#include <functional>
#include <glm/glm.hpp>

class CollisionBox {
    std::vector<std::function<bool(glm::vec3)>> collisionTriggers;
    std::vector<glm::vec3> plane;
    std::vector<glm::vec3> planeNormals;
    float reflectionCoefficient;

    std::vector<glm::vec3> points;
    std::vector<glm::vec3> normals;

    glm::mat4 modelMatrix;
    glm::mat4 transInvModel;
public:
    CollisionBox(float side);

    void setModelMatrix(glm::mat4 m);
    void updateParticle(glm::vec3 prevPosition, glm::vec3 &position, glm::vec3 &velocity);

    static glm::vec3 elMul(glm::vec3 u, glm::vec3 v) ;

    glm::mat4 getModelMatrix();
};



#endif //ELASTIC_DEFORMATION_SIMULATION_COLLISIONBOX_H
