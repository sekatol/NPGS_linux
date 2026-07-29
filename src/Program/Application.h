#pragma once

#include <memory>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <vulkan/vulkan.hpp>

#include "DataStructures.h"

#include "Engine/Core/Base/Base.h"
#include "Engine/Core/Runtime/Graphics/Vulkan/Context.h"
#include "Engine/Core/Runtime/Graphics/Vulkan/Resources.h"
#include "Engine/Core/Runtime/Graphics/Vulkan/Wrappers.h"
#include "Engine/Core/Runtime/Graphics/Vulkan/VulkanUIRenderer.h"

#include "Engine/Core/System/Spatial/Camera.h"

_NPGS_BEGIN

class FApplication
{
public:
    FApplication(const vk::Extent2D& WindowSize, const std::string& WindowTitle, bool bEnableVSync, bool bEnableFullscreen);
    ~FApplication();

    void Quit();
    void ExecuteMainRender();
    void Terminate();

private:
    bool InitializeWindow();
    void InitializeInputCallbacks();
    void ProcessInput();
    void update();
    void RenderDebugUI();

    static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void CharCallback(GLFWwindow* window, unsigned int codepoint);
    static void FramebufferSizeCallback(GLFWwindow* Window, int Width, int Height);
    static void CursorPosCallback(GLFWwindow* Window, double PosX, double PosY);
    static void ScrollCallback(GLFWwindow* Window, double OffsetX, double OffsetY);

    void HandleMouseButton(int button, int action, int mods);
    void HandleKey(int key, int scancode, int action, int mods);
    void HandleChar(unsigned int codepoint);
    void HandleFramebufferSize(int width, int height);
    void HandleCursorPos(double posX, double posY);
    void HandleScroll(double offsetX, double offsetY);

private:
    Runtime::Graphics::FVulkanContext*        _VulkanContext;

    std::string                               _WindowTitle;
    vk::Extent2D                              _WindowSize;
    GLFWwindow*                               _Window = nullptr;
    bool                                      _bEnableVSync;
    bool                                      _bEnableFullscreen;

    std::unique_ptr<Runtime::Graphics::FVulkanUIRenderer> _uiRenderer;

    std::unique_ptr<System::Spatial::FCamera> _FreeCamera;
    glm::vec3 LastCameraWorldPos = glm::vec3(0.0f);
    bool _bLeftMousePressedInWorld = false;
    bool _bIsDraggingInWorld = false;
    double CurrentTime = 0.0;
    double RealityTime = 0.0;
    double GameTime = 0.0;
    double TimeRate = 1.0;
    double PreviousTime = glfwGetTime();
    double LastFrameTime = 0.0;
    int    FramePerSec = 0;
    int    _DisplayFPS = 0;
    long long int FrameCount = 0;

    double _DeltaTime = 0.0;
    double _LastDeltaTime = 0.0;
    double _DragStartX = 0.0;
    double _DragStartY = 0.0;
    double _LastX = 0.0;
    double _LastY = 0.0;

    bool _bRightMousePressedInWorld = false;
    bool _bIsDraggingRightInWorld = false;
    bool _bFirstMouseRight = true;
    double _DragRightStartX = 0.0;
    double _DragRightStartY = 0.0;
    double _LastRightX = 0.0;
    double _LastRightY = 0.0;

    float _buffered_scroll_y = 0.0f;
    bool _bFirstMouse = true;

    float s_GeodesicThrust = 1.5f;
};

_NPGS_END
