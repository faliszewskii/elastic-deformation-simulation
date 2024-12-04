//
// Created by faliszewskii on 06.05.24.
//

#ifndef PATCHC0_H
#define PATCHC0_H

#include <glm/vec4.hpp>
#include "../../../opengl/mesh/PositionVertex.h"
#include "../../../opengl/shader/Shader.h"
#include "../../../opengl/mesh/Mesh.h"
#include "../point/Point.h"

class PatchC0 {
public:
    Mesh<PositionVertex> mesh;

    PatchC0(const std::vector<PositionVertex> &vertices) :
        mesh(vertices, {},GL_PATCHES){}

    void updatePoint(Point &point, int i) {
        mesh.update({point.position}, i);
    }

    void render(Shader &shader) {
        shader.setUniform("patchCountWidth", 1);
        shader.setUniform("patchCountLength", 1);

        glPatchParameteri(GL_PATCH_VERTICES, 16);
        mesh.render();
    }
};

#endif //PATCHC0_H
