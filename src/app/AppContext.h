//
// Created by faliszewskii on 16.06.24.
//

#ifndef OPENGL_TEMPLATE_APPCONTEXT_H
#define OPENGL_TEMPLATE_APPCONTEXT_H

#include <memory>
#include "../opengl/framebuffer/FrameBufferManager.h"
#include "../interface/camera/BaseCamera.h"
#include "entity/quad/Quad.h"
#include "../opengl/shader/Shader.h"
#include "entity/light/PointLight.h"
#include "entity/point/Point.h"
#include "../opengl/model/Model.h"
#include "elastic/ElasticCube.h"
#include "bernsteinCube/BernsteinCube.h"
#include "entity/cube/InvertedCube.h"
#include "entity/line/Line.h"

struct AppContext {
    AppContext() = default;

    std::unique_ptr<BaseCamera> camera;
    std::unique_ptr<FrameBufferManager> frameBufferManager;

    // Shaders
    std::unique_ptr<Shader> jellyShader;
    std::unique_ptr<Shader> phongShader;
    std::unique_ptr<Shader> pointShader;
    std::unique_ptr<Shader> patchC0Shader;
    std::unique_ptr<Shader> colorShader;
    std::unique_ptr<Shader> normalShader;

    std::unique_ptr<PointLight> light;
    std::unique_ptr<Point> lightBulb;
    std::unique_ptr<Model> bunny;
    std::unique_ptr<InvertedCube> invertedCube;

    std::unique_ptr<ElasticCube> elasticCube;
    std::unique_ptr<Point> massPoint;
    std::unique_ptr<BernsteinCube> bernsteinCube;

    std::array<Line, 4*4 + 4*4 + 4*4> bernsteinFrame;
    std::array<Line, 12> frameFrame;
    std::array<Line, 8> frameSprings;

    glm::vec3 boxTranslation;
    glm::vec3 boxScale;
    glm::quat boxRotation;

    float lastFrameTimeMs;

    bool modifiedNormals;
    bool displayNormals;
    bool running;
    bool drawBernstein;
    bool drawFrame;
};

#endif //OPENGL_TEMPLATE_APPCONTEXT_H
