#version 400 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

out vec3 normal;
out vec3 fragPos;
out vec2 texCoords;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 controlPoints[64];


vec4 descendingAlgorithm(float t, int n)
{
    mat4 bernsteinBasis = mat4(0.0f);
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

    return vec4(bernsteinBasis[n][0], bernsteinBasis[n][1], bernsteinBasis[n][2], bernsteinBasis[n][3]);
}

int fromIndex3D(int x, int y, int z) {
    return x + y * 4 + z * 16;
}

vec3 evaluate(vec3 uvw) {
    vec4 uCoeffs = descendingAlgorithm(uvw.x, 3);
    vec4 vCoeffs = descendingAlgorithm(uvw.y, 3);
    vec4 wCoeffs = descendingAlgorithm(uvw.z, 3);

    vec3 result = vec3(0);
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            for(int k = 0; k < 4; k++) {
                result += controlPoints[fromIndex3D(k, j, i)] * uCoeffs[i] * vCoeffs[j] * wCoeffs[k];
            }
        }
    }
return result;
}

vec3 evaluateDU(vec3 uvw) {
    vec3 uCoeffs3 = vec3(descendingAlgorithm(uvw.x, 2));
    vec4 vCoeffs4 = descendingAlgorithm(uvw.y, 3);
    vec4 wCoeffs4 = descendingAlgorithm(uvw.z, 3);

    vec3 partResult[4];
    vec3 tangent1 = vec3(0);

    for(int i = 0; i < 4; i++) {
        partResult[i] = vec3(0);
        for(int j = 0; j < 4; j++) {
            for(int k = 0; k < 4; k++) {
                partResult[i] += controlPoints[fromIndex3D(k, j, i)] * vCoeffs4[j] * wCoeffs4[k];
            }
        }
    }
    tangent1 += (partResult[1] - partResult[0]) * uCoeffs3[0];
    tangent1 += (partResult[2] - partResult[1]) * uCoeffs3[1];
    tangent1 += (partResult[3] - partResult[2]) * uCoeffs3[2];
    tangent1 *= 3;

    return tangent1;
}

vec3 evaluateDV(vec3 uvw) {
    vec4 uCoeffs4 = descendingAlgorithm(uvw.x, 3);
    vec3 vCoeffs3 = vec3(descendingAlgorithm(uvw.y, 2));
    vec4 wCoeffs4 = descendingAlgorithm(uvw.z, 3);

    vec3 partResult[4];
    vec3 tangent1 = vec3(0);
    for(int j = 0; j < 4; j++) {
        partResult[j] = vec3(0);
        for(int i = 0; i < 4; i++) {
            for(int k = 0; k < 4; k++) {
                partResult[j] += controlPoints[fromIndex3D(k, j, i)] * uCoeffs4[i] * wCoeffs4[k];
            }
        }
    }
    tangent1 += (partResult[1] - partResult[0]) * vCoeffs3[0];
    tangent1 += (partResult[2] - partResult[1]) * vCoeffs3[1];
    tangent1 += (partResult[3] - partResult[2]) * vCoeffs3[2];
    tangent1 *= 3;

    return tangent1;
}

vec3 evaluateDW(vec3 uvw) {
    vec4 uCoeffs4 = descendingAlgorithm(uvw.x, 3);
    vec4 vCoeffs4 = descendingAlgorithm(uvw.y, 3);
    vec3 wCoeffs3 = vec3(descendingAlgorithm(uvw.z, 2));

    vec3 partResult[4];
    vec3 tangent1 = vec3(0);

    for(int k = 0; k < 4; k++) {
        partResult[k] = vec3(0);
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                partResult[k] += controlPoints[fromIndex3D(k, j, i)] * uCoeffs4[i] * vCoeffs4[j];
            }
        }
    }
    tangent1 += (partResult[1] - partResult[0]) * wCoeffs3[0];
    tangent1 += (partResult[2] - partResult[1]) * wCoeffs3[1];
    tangent1 += (partResult[3] - partResult[2]) * wCoeffs3[2];
    tangent1 *= 3;

    return tangent1;
}


void main()
{
    vec3 p = evaluate(aPos);
    fragPos = vec3(model * vec4(p, 1.0));
    gl_Position = projection * view * model * vec4(p, 1);

    mat3 T = mat3(
        normalize(evaluateDU(aPos)),
        normalize(evaluateDV(aPos)),
        normalize(evaluateDW(aPos))
    );

    normal = transpose(inverse(T) ) * aNormal;
    texCoords = aTexCoords;
}