//
// Created by faliszewskii on 02.12.24.
//

#include <glm/glm.hpp>
#include "BernsteinCube.h"


glm::vec4 BernsteinCube::descendingAlgorithm(float t, int n)
{
    glm::mat4 bernsteinBasis = glm::mat4(0.0f);
    bernsteinBasis[0][0] = 1.0f;

    float u = 1.0 - t;

    for (int j = 1; j <= n; j++)
    {
        bernsteinBasis[j][0] = bernsteinBasis[j - 1][0] * u;

        for (int i = 1; i <= j; i++)
        {
            bernsteinBasis[j][i] = bernsteinBasis[j - 1][i] * u + bernsteinBasis[j - 1][i - 1] * t;
        }
    }

    return glm::vec4(bernsteinBasis[n][0], bernsteinBasis[n][1], bernsteinBasis[n][2], bernsteinBasis[n][3]);
}

glm::vec3 BernsteinCube::evaluate(glm::vec3 uvw) {

    glm::vec4 uCoeffs = descendingAlgorithm(uvw.x, 3);
    glm::vec4 vCoeffs = descendingAlgorithm(uvw.y, 3);
    glm::vec4 wCoeffs = descendingAlgorithm(uvw.z, 3);

    auto index3D = [](int x, int y, int z) { return x + y * 4 + z * 16; };

    glm::vec3 result{};
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            for(int k = 0; k < 4; k++) {
                result += points[index3D(k, j, i)] * uCoeffs[i] * vCoeffs[j] * wCoeffs[k];
            }
        }
    }

    return result;
}
