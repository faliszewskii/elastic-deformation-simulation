//
// Created by faliszewskii on 16.06.24.
//

#include <glm/gtc/type_ptr.hpp>
#include "Gui.h"
#include "imgui.h"

Gui::Gui(AppContext &appContext) : appContext(appContext) {}

glm::quat addRotation(glm::quat orientation, glm::vec3 eulerAngles) {
    auto z = angleAxis(eulerAngles.z, glm::vec3(0,0,1));
    auto y = angleAxis(eulerAngles.y, glm::vec3(0,1,0));
    auto x = angleAxis(eulerAngles.x, glm::vec3(1,0,0));
    return orientation * glm::normalize(x * y * z);
}

void Gui::render() {
    ImGui::Begin("Elastic simulation");
    if(ImGui::CollapsingHeader("Simulation"), true) {
        if(ImGui::Button(appContext.running? "Stop": "Start")) {
            appContext.running = !appContext.running;
        }
        if(ImGui::Button("Reset")) {
            appContext.elasticCube->reset();
        }
        ImGui::DragFloat("Time step (ms)", &appContext.elasticCube->timeStepMs, 0.01, 0.1, 15);
        ImGui::Checkbox("Gravity", &appContext.elasticCube->gravityOn);
        ImGui::Checkbox("Frame", &appContext.elasticCube->frameOn);
        ImGui::SeparatorText("Frame transform");
        ImGui::DragFloat3("Frame translation", glm::value_ptr(appContext.elasticCube->steeringTranslation), 0.01);
        ImGui::DragFloat3("Frame scale", glm::value_ptr(appContext.elasticCube->steeringScale), 0.01, 0.01, 10);

        bool modified = false;
        auto oldAngle = glm::eulerAngles(appContext.elasticCube->steeringRotation);
        auto newAngle = glm::vec3(oldAngle);
        auto angleRef = static_cast<float *>(glm::value_ptr(newAngle));
        if(ImGui::DragFloat3("Frame rotation", glm::value_ptr(newAngle), 0.01)) {
            appContext.elasticCube->steeringRotation = addRotation(appContext.elasticCube->steeringRotation,
                       glm::vec3(newAngle.x - oldAngle.x, newAngle.y - oldAngle.y, newAngle.z - oldAngle.z));
        }

        ImGui::SeparatorText("Box transform");
        modified = false;
        modified |= ImGui::DragFloat3("Box translation", glm::value_ptr(appContext.boxTranslation), 0.01);
        modified |= ImGui::DragFloat3("Box scale", glm::value_ptr(appContext.boxScale), 0.01, 0);
        oldAngle = glm::eulerAngles(appContext.boxRotation);
        newAngle = glm::vec3(oldAngle);
        angleRef = static_cast<float *>(glm::value_ptr(newAngle));
        if(ImGui::DragFloat3("Box rotation", glm::value_ptr(newAngle), 0.01)) {
            modified = true;
            appContext.boxRotation = addRotation(appContext.boxRotation,
                                                                   glm::vec3(newAngle.x - oldAngle.x, newAngle.y - oldAngle.y, newAngle.z - oldAngle.z));
        }
        if(modified) {
            auto boxMatrix = glm::identity<glm::mat4>();
            auto eulerRotation = glm::eulerAngles(appContext.boxRotation);
            glm::mat4 Rx = {
                    {1,0,0,0},
                    {0, cos(eulerRotation.x), sin(eulerRotation.x), 0},
                    {0, -sin(eulerRotation.x), cos(eulerRotation.x), 0},
                    {0, 0, 0, 1}
            };
            glm::mat4 Ry = {
                    {cos(eulerRotation.y),0,-sin(eulerRotation.y),0},
                    {0, 1, 0, 0},
                    {sin(eulerRotation.y), 0, cos(eulerRotation.y), 0},
                    {0, 0, 0, 1}
            };
            glm::mat4 Rz = {
                    {cos(eulerRotation.z),sin(eulerRotation.z),0,0},
                    {-sin(eulerRotation.z), cos(eulerRotation.z), 0, 0},
                    {0, 0, 1, 0},
                    {0, 0, 0, 1}
            };
            boxMatrix = glm::translate(boxMatrix, appContext.boxTranslation) * Rz * Ry * Rx;
            boxMatrix = glm::scale(boxMatrix, appContext.boxScale);
            appContext.elasticCube->collisionBox.setModelMatrix(boxMatrix);
        }

        ImGui::SeparatorText("Simulation params");
        if(ImGui::DragFloat("Cube size", &appContext.elasticCube->cubeSize, 0.01, 0.1, 5)) {
            appContext.elasticCube->reset();
        }
        ImGui::DragFloat("gravity", &appContext.elasticCube->gravity, 0.01, 0.01, 9.81);
        ImGui::DragFloat("m", &appContext.elasticCube->m, 0.01, 0.01, 100);
        ImGui::DragFloat("c1", &appContext.elasticCube->c1, 0.01, 0.01, 100);
        ImGui::DragFloat("c2", &appContext.elasticCube->c2, 0.01, 0.01, 100);
        ImGui::DragFloat("k", &appContext.elasticCube->k, 0.01, 0, 100);
        ImGui::Checkbox("Far springs", &appContext.elasticCube->farSprings);


        static float posDisturb = 0.1;
        ImGui::DragFloat("Position disturbance", &posDisturb, 0.001, 0.01, 0.3);
        if(ImGui::Button("Disturb positions")) {
            appContext.elasticCube->disturbPos(posDisturb);
        }
        static float velDisturb = 0.1;
        ImGui::DragFloat("Velocity disturbance", &velDisturb, 0.01, 0.1, 1);
        if(ImGui::Button("Disturb velocities")) {
            appContext.elasticCube->disturbVel(velDisturb);
        }
    }
    if(ImGui::CollapsingHeader("Visualization")) {
        ImGui::Checkbox("Display bernstein grid", &appContext.drawBernstein);
        ImGui::Checkbox("Display frame", &appContext.drawFrame);
        ImGui::Checkbox("Modify Normals", &appContext.modifiedNormals);
        ImGui::Checkbox("Display Normals", &appContext.displayNormals);
    }
    if(ImGui::CollapsingHeader("Light")) {
        renderLightUI(*appContext.light);
    }
    ImGui::End();
}

void Gui::renderLightUI(PointLight &light) {
    ImGui::ColorPicker3("Light Color", glm::value_ptr(light.color));
    ImGui::DragFloat3("Light Position", glm::value_ptr(light.position), 0.001f);
}
