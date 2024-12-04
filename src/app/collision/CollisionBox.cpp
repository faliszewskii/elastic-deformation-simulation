//
// Created by faliszewskii on 03.12.24.
//

#include <glm/geometric.hpp>
#include "CollisionBox.h"

CollisionBox::CollisionBox(float side) {
    reflectionCoefficient = 0.99;
    collisionTriggers = {
            [side](glm::vec3 pos){ return pos.x < -side/2; },
            [side](glm::vec3 pos){ return pos.x > side/2; },
            [side](glm::vec3 pos){ return pos.y < -side/2; },
            [side](glm::vec3 pos){ return pos.y > side/2; },
            [side](glm::vec3 pos){ return pos.z < -side/2; },
            [side](glm::vec3 pos){ return pos.z > side/2; },
    };
    plane = {
            {-side/2, 0, 0},
            {side/2, 0, 0},
            {0, -side/2, 0},
            {0, side/2, 0},
            {0, 0, -side/2},
            {0, 0, side/2},
    };
    planeNormals = {
            {1, 0, 0},
            {1, 0, 0},
            {0, 1, 0},
            {0, 1, 0},
            {0, 0, 1},
            {0, 0, 1},
    };
}

void CollisionBox::updateParticle(glm::vec3 &position, glm::vec3 &velocity) {
    bool collided = false;
    do {
        collided = false;
        for(int i = 0; i < collisionTriggers.size(); ++i) {
            if(collisionTriggers[i](position)) {
                collided = true;
                // I method
//                velocity -= (1+reflectionCoefficient) * elMul(velocity, planeNormals[i]);
                // II method
                velocity -= elMul(velocity, planeNormals[i]);
                velocity *= reflectionCoefficient;
                position -= (1+reflectionCoefficient) * (elMul(planeNormals[i], position) - plane[i]);
            }
        }
    }while(collided);
}

glm::vec3 CollisionBox::elMul(glm::vec3 u, glm::vec3 v) {
    return {v.x * u.x, v.y * u.y, v.z * u.z};
}
