//
// Created by faliszewskii on 16.06.24.
//

#include "Scene.h"
#include "../interface/camera/CameraAnchor.h"
#include "entity/patch/PatchC0.h"

Scene::Scene(AppContext &appContext) : appContext(appContext) {
    appContext.camera = std::make_unique<CameraAnchor>(1920, 1080, glm::vec3(0.0f, 3.0f, 3.0f), glm::vec3(0.f), glm::vec3(-M_PI / 4, 0, 0));
    appContext.frameBufferManager = std::make_unique<FrameBufferManager>();
    appContext.frameBufferManager->create_buffers(appContext.camera->screenWidth, appContext.camera->screenHeight);

    appContext.jellyShader = std::make_unique<Shader>(Shader::createTraditionalShader(
            "../res/shaders/jelly/jelly.vert", "../res/shaders/jelly/jelly.frag"));
    appContext.phongShader = std::make_unique<Shader>(Shader::createTraditionalShader(
            "../res/shaders/phong/phong.vert", "../res/shaders/phong/phong.frag"));
    appContext.pointShader = std::make_unique<Shader>(Shader::createTraditionalShader(
            "../res/shaders/point/point.vert", "../res/shaders/point/point.frag"));
    appContext.patchC0Shader = std::make_unique<Shader>(Shader::createTraditionalShader(
            "../res/shaders/patch/patch.vert", "../res/shaders/patch/patch.tesc", "../res/shaders/patch/patch.tese", "../res/shaders/patch/patch.frag"));
    appContext.colorShader = std::make_unique<Shader>(Shader::createTraditionalShader(
            "../res/shaders/basic/position.vert", "../res/shaders/basic/color.frag"));
    appContext.normalShader = std::make_unique<Shader>(Shader::createTraditionalShader(
            "../res/shaders/helper/jellyNormal.vert", "../res/shaders/helper/normal.geom", "../res/shaders/helper/normal.frag"));

    appContext.light = std::make_unique<PointLight>();
    appContext.light->position = {-0.5f , 0.5f, 0.5f};
    appContext.light->linearAttenuation = 0.35f;
    appContext.light->quadraticAttenuation = 0.44f;
    appContext.lightBulb = std::make_unique<Point>();

    appContext.elasticCube = std::make_unique<ElasticCube>();
    appContext.massPoint = std::make_unique<Point>();
    appContext.massPoint->color = glm::vec4(1.0f);
    appContext.bernsteinCube = std::make_unique<BernsteinCube>(appContext.elasticCube->positions);

    appContext.bunny = std::make_unique<Model>("../res/models/stanfordBunny.obj");
    appContext.invertedCube = std::make_unique<InvertedCube>();

    glm::vec3 minSize{};
    glm::vec3 maxSize{};
    for(auto &mesh : appContext.bunny->meshes) {
        for(auto &vertex : mesh.getVertices()) {
            if(vertex.position.x < minSize.x)
                minSize.x = vertex.position.x;
            if(vertex.position.y < minSize.y)
                minSize.y = vertex.position.y;
            if(vertex.position.z < minSize.z)
                minSize.z = vertex.position.z;
            if(vertex.position.x > maxSize.x)
                maxSize.x = vertex.position.x;
            if(vertex.position.y > maxSize.y)
                maxSize.y = vertex.position.y;
            if(vertex.position.z > maxSize.z)
                maxSize.z = vertex.position.z;
        }
    }
    minSize.y+=0.033;
    glm::vec3 translation = (maxSize + minSize) / 2.f;
    glm::vec3 scale = {
            maxSize.x - minSize.x,
            maxSize.y - minSize.y,
            maxSize.z - minSize.z

    };

    float scalingFactor = std::max(maxSize.x - minSize.x, std::max(maxSize.y - minSize.y, maxSize.z - minSize.z));

    for(auto &mesh : appContext.bunny->meshes) {
        std::vector<PositionNormalVertex> vertices;
        std::transform(mesh.getVertices().begin(), mesh.getVertices().end(), std::back_inserter(vertices),
                       [&](auto &v){return PositionNormalVertex{(v.position - translation)/scalingFactor + glm::vec3(0.5f), v.normal};});
        mesh.update(std::move(vertices));
    }

//    float scale = std::min(std::min(size.x, size.y), size.z) * 2.f;
//    for(auto &mesh : appContext.bunny->meshes) {
//        std::vector<PositionNormalVertex> vertices;
//        std::transform(mesh.getVertices().begin(), mesh.getVertices().end(), std::back_inserter(vertices),
//                       [scale](auto &v){return PositionNormalVertex{v.position/scale, v.normal};});
//        mesh.update(std::move(vertices));
//    }

    appContext.running = false;
    appContext.lastFrameTimeMs = glfwGetTime();
    appContext.drawBernstein = false;
    appContext.drawFrame = true;
    appContext.displayNormals = false;
    appContext.modifiedNormals = true;

    appContext.boxRotation = glm::identity<glm::quat>();
    appContext.boxScale = glm::vec3(1);
}

void Scene::update() {
    appContext.lightBulb->position = appContext.light->position;
    appContext.lightBulb->color = glm::vec4(appContext.light->color, 1);

    if(appContext.running) {
        float timeMs = glfwGetTime() * 1000;
        int loopsToDo = static_cast<int>((timeMs - appContext.lastFrameTimeMs) / appContext.elasticCube->timeStepMs);
        appContext.lastFrameTimeMs += loopsToDo * appContext.elasticCube->timeStepMs;
        for (int i = 0; i < loopsToDo; i++) {
            appContext.elasticCube->advanceByStep();
        }
    } else {
        appContext.lastFrameTimeMs = glfwGetTime() * 1000.f;
    }

    auto fromIndex3D = [](int x, int y, int z) { return x + y * 4 + z * 16; };
    for(int x = 0; x < 4; ++x) {
        for(int y = 0; y < 4; ++y) {
            std::vector<glm::vec3> points;
            for(int i = 0; i < 4; ++i) {
                points.push_back(appContext.elasticCube->positions[fromIndex3D(x, y, i)]);
            }
            appContext.bernsteinFrame[x*4 + y].updatePoints(points);
        }
    }
    for(int x = 0; x < 4; ++x) {
        for(int y = 0; y < 4; ++y) {
            std::vector<glm::vec3> points;
            for(int i = 0; i < 4; ++i) {
                points.push_back(appContext.elasticCube->positions[fromIndex3D(x, i, y)]);
            }
            appContext.bernsteinFrame[4*4 + x*4 + y].updatePoints(points);
        }
    }
    for(int x = 0; x < 4; ++x) {
        for(int y = 0; y < 4; ++y) {
            std::vector<glm::vec3> points;
            for(int i = 0; i < 4; ++i) {
                points.push_back(appContext.elasticCube->positions[fromIndex3D(i, x, y)]);
            }
            appContext.bernsteinFrame[2*4*4 + x*4 + y].updatePoints(points);
        }
    }

    float a = appContext.elasticCube->cubeSize / 2.f;
    appContext.frameFrame[0].updatePoints({{-a, -a, -a}, {-a, a, -a}});
    appContext.frameFrame[1].updatePoints({{-a, -a, -a}, {a, -a, -a}});
    appContext.frameFrame[2].updatePoints({{-a, -a, -a}, {-a, -a, a}});
    appContext.frameFrame[3].updatePoints({{-a, a, a}, {-a, -a, a}});
    appContext.frameFrame[4].updatePoints({{-a, a, a}, {a, a, a}});
    appContext.frameFrame[5].updatePoints({{-a, a, a}, {-a, a, -a}});
    appContext.frameFrame[6].updatePoints({{a, -a, a}, {a, -a, -a}});
    appContext.frameFrame[7].updatePoints({{a, -a, a}, {-a, -a, a}});
    appContext.frameFrame[8].updatePoints({{a, -a, a}, {a, a, a}});
    appContext.frameFrame[9].updatePoints({{a, a, -a}, {-a, a, -a}});
    appContext.frameFrame[10].updatePoints({{a, a, -a}, {a, -a, -a}});
    appContext.frameFrame[11].updatePoints({{a, a, -a}, {a, a, a}});

    auto m = appContext.elasticCube->getSteeringMatrix();
    auto p = glm::vec3(m * glm::vec4(-a, -a, -a, 1));
    auto q = appContext.elasticCube->positions[fromIndex3D(0, 0, 0)];
    appContext.frameSprings[0].updatePoints({p, q});
    p = glm::vec3(m * glm::vec4(a, -a, -a, 1));
    q = appContext.elasticCube->positions[fromIndex3D(0, 0, 3)];
    appContext.frameSprings[1].updatePoints({p, q});
    p = glm::vec3(m * glm::vec4(-a, -a, a, 1));
    q = appContext.elasticCube->positions[fromIndex3D(3, 0, 0)];
    appContext.frameSprings[2].updatePoints({p, q});
    p = glm::vec3(m * glm::vec4(-a, a, -a, 1));
    q = appContext.elasticCube->positions[fromIndex3D(0, 3, 0)];
    appContext.frameSprings[3].updatePoints({p, q});
    p = glm::vec3(m * glm::vec4(a, a, -a, 1));
    q = appContext.elasticCube->positions[fromIndex3D(0, 3, 3)];
    appContext.frameSprings[4].updatePoints({p, q});
    p = glm::vec3(m * glm::vec4(a, -a, a, 1));
    q = appContext.elasticCube->positions[fromIndex3D(3, 0, 3)];
    appContext.frameSprings[5].updatePoints({p, q});
    p = glm::vec3(m * glm::vec4(-a, a, a, 1));
    q = appContext.elasticCube->positions[fromIndex3D(3, 3, 0)];
    appContext.frameSprings[6].updatePoints({p, q});
    p = glm::vec3(m * glm::vec4(a, a, a, 1));
    q = appContext.elasticCube->positions[fromIndex3D(3, 3, 3)];
    appContext.frameSprings[6].updatePoints({p, q});
}

void Scene::render() {
    appContext.frameBufferManager->bind();

    appContext.jellyShader->use();
    appContext.jellyShader->setUniform("viewPos", appContext.camera->getViewPosition());
    appContext.jellyShader->setUniform("view", appContext.camera->getViewMatrix());
    appContext.jellyShader->setUniform("projection", appContext.camera->getProjectionMatrix());
    appContext.jellyShader->setUniform("material.hasTexture", false);
    appContext.jellyShader->setUniform("material.shininess", 32.f);
    appContext.light->setupPointLight(*appContext.jellyShader);
    appContext.jellyShader->setUniform("material.albedo", glm::vec4(1.0f, 0.5f, 0.5f, 1.0f));
    appContext.jellyShader->setUniform("model", glm::identity<glm::mat4>());
    appContext.jellyShader->setUniform("controlPoints", appContext.elasticCube->positions.begin(), 64);
    appContext.jellyShader->setUniform("modifyNormals", appContext.modifiedNormals);
    appContext.bunny->render();

    if(appContext.displayNormals) {
        appContext.normalShader->use();
        appContext.normalShader->setUniform("view", appContext.camera->getViewMatrix());
        appContext.normalShader->setUniform("projection", appContext.camera->getProjectionMatrix());
        appContext.normalShader->setUniform("model", glm::identity<glm::mat4>());
        appContext.normalShader->setUniform("controlPoints", appContext.elasticCube->positions.begin(), 64);
        appContext.normalShader->setUniform("modifyNormals", appContext.modifiedNormals);
        appContext.bunny->render();
    }

    glm::mat4 roomModel = glm::identity<glm::mat4>();
    roomModel = glm::scale(roomModel, glm::vec3(3));
    roomModel = glm::translate(roomModel, -glm::vec3(0.5f));
    roomModel = appContext.elasticCube->collisionBox.getModelMatrix() * roomModel;

    appContext.phongShader->use();
    appContext.phongShader->setUniform("viewPos", appContext.camera->getViewPosition());
    appContext.phongShader->setUniform("view", appContext.camera->getViewMatrix());
    appContext.phongShader->setUniform("projection", appContext.camera->getProjectionMatrix());
    appContext.phongShader->setUniform("material.hasTexture", false);
    appContext.phongShader->setUniform("material.shininess", 4.f);
    appContext.light->setupPointLight(*appContext.phongShader);
    appContext.phongShader->setUniform("material.albedo", glm::vec4(0.5f, 0.5f, 0.5f, 0.9f));
    appContext.phongShader->setUniform("model", roomModel);
    appContext.phongShader->setUniform("controlPoints", appContext.elasticCube->positions.begin(), 64);
    appContext.invertedCube->render();

    appContext.pointShader->use();
    appContext.pointShader->setUniform("view", appContext.camera->getViewMatrix());
    appContext.pointShader->setUniform("projection", appContext.camera->getProjectionMatrix());
    appContext.lightBulb->render(*appContext.pointShader);
    for(auto &point : appContext.elasticCube->positions) {
        appContext.massPoint->position = point;
        appContext.massPoint->render(*appContext.pointShader);
    }


    appContext.colorShader->use();
    appContext.colorShader->setUniform("view", appContext.camera->getViewMatrix());
    appContext.colorShader->setUniform("projection", appContext.camera->getProjectionMatrix());
    if(appContext.drawBernstein) {
        appContext.colorShader->setUniform("model", glm::identity<glm::mat4>());
        appContext.colorShader->setUniform("color", glm::vec4{0.5, 0.5, 1.f, 1});
        for(auto &line : appContext.bernsteinFrame) {
            line.render();
        }
    }
    if(appContext.drawFrame) {
        appContext.colorShader->setUniform("model", appContext.elasticCube->getSteeringMatrix());
        appContext.colorShader->setUniform("color", glm::vec4{0.5f, 1.f, 0.5f, 1});
        for(auto &line : appContext.frameFrame) {
            line.render();
        }
        appContext.colorShader->setUniform("model", glm::identity<glm::mat4>());
        appContext.colorShader->setUniform("color", glm::vec4{1.f, 0.5, 0.5f, 1});
        if(appContext.elasticCube->frameOn)
            for(auto &line : appContext.frameSprings) {
                line.render();
            }
    }

    appContext.patchC0Shader->use();
    appContext.patchC0Shader->setUniform("model", glm::identity<glm::mat4>());
    appContext.patchC0Shader->setUniform("viewPos", appContext.camera->getViewPosition());
    appContext.patchC0Shader->setUniform("view", appContext.camera->getViewMatrix());
    appContext.patchC0Shader->setUniform("projection", appContext.camera->getProjectionMatrix());
    appContext.patchC0Shader->setUniform("material.hasTexture", false);
    appContext.patchC0Shader->setUniform("material.albedo", glm::vec4(0.7f, 0.7f, 1.0f, .4f));
    appContext.patchC0Shader->setUniform("material.shininess", 256.f);
    appContext.light->setupPointLight(*appContext.patchC0Shader);

    for(int i = 0; i < 6; i++) {
        auto p = appContext.elasticCube->getSide(i);
        std::vector<PositionVertex> v;
        std::transform(p.begin(), p.end(), std::back_inserter(v), [](glm::vec3 point){return PositionVertex{point};});
        PatchC0 patch(v);
        patch.render(*appContext.patchC0Shader);
    }

    appContext.frameBufferManager->unbind();
}
