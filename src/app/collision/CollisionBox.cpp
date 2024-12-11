//
// Created by faliszewskii on 03.12.24.
//

#include <glm/geometric.hpp>
#include <iostream>
#include <glm/ext/matrix_transform.hpp>
#include "CollisionBox.h"

CollisionBox::CollisionBox(float side) {
    modelMatrix = glm::identity<glm::mat4>();
    transInvModel = modelMatrix;
    reflectionCoefficient = 0.99;
    points = {
            {-side/2, 0, 0},
            {side/2, 0, 0},
            {0, -side/2, 0},
            {0, side/2, 0},
            {0, 0, -side/2},
            {0, 0, side/2},
    };
    normals = {
            {1, 0, 0},
            {-1, 0, 0},
            {0, 1, 0},
            {0, -1, 0},
            {0, 0, 1},
            {0, 0, -1},
    };
}

void CollisionBox::updateParticle(glm::vec3 prevPosition, glm::vec3 &position, glm::vec3 &velocity) {

    bool colided = false;
    do {
        colided = false;
        for(int i = 0; i < points.size(); ++i) {
            glm::vec3 point = modelMatrix * glm::vec4(points[i], 1);
            glm::vec3 normal = transInvModel * glm::vec4(normals[i], 0);
            normal = glm::normalize(normal);
            if(glm::dot((position - point), normal) < 0) {
//                colided = true;
                velocity = velocity - 2 *  glm::dot(velocity, normal) * normal;
                velocity *= reflectionCoefficient;

                float projDist = glm::dot(position - point, normal);
                float dist  = glm::distance(prevPosition, position);
                auto penetrationDist = dist * projDist / dist;
                auto incomingDir = glm::normalize(prevPosition - position);
                auto intersection = position + (prevPosition - position) * projDist / dist;
                auto reflection = 2 * glm::dot(incomingDir, normal) * normal - incomingDir;
                reflection *= penetrationDist * reflectionCoefficient;
                position = intersection + reflection;
            }
        }
    } while(colided);
}

glm::vec3 CollisionBox::elMul(glm::vec3 u, glm::vec3 v) {
    return {v.x * u.x, v.y * u.y, v.z * u.z};
}

void CollisionBox::setModelMatrix(glm::mat4 m) {
    modelMatrix = m;
    transInvModel = glm::transpose(glm::inverse(m));
}

glm::mat4 CollisionBox::getModelMatrix() {
    return modelMatrix;
}
