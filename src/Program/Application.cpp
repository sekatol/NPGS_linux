#include "Application.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_vulkan.h>

#include "Engine/Core/Base/Config/EngineConfig.h"
#include "Engine/Core/Math/NumericConstants.h"
#include "Engine/Core/Runtime/AssetLoaders/AssetManager.h"
#include "Engine/Core/Runtime/AssetLoaders/Shader.h"
#include "Engine/Core/Runtime/AssetLoaders/Texture.h"
#include "Engine/Core/Runtime/Graphics/Renderers/PipelineManager.h"
#include "Engine/Core/Runtime/Graphics/Vulkan/ShaderResourceManager.h"
#include "Engine/Utils/Logger.h"
#include "DataStructures.h"

#include <chrono>
#include <fstream>
#include <sstream>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
#include <iomanip>
#include <filesystem>
#include <iostream>
#include <algorithm>
#include <glm/gtc/packing.hpp>

static bool g_bRequestScreenshot = false;
static bool g_GeodesicMode = false;
static bool g_bHideUIAndMouse = false;

struct FTrajectoryPoint
{
    glm::vec3 Pos;
    float UniverseSign;
    float Odometer;
    double t_ks;
    bool isOut;
    int UnivIndex;
};
static float g_TotalOdometer = 0.0f;
static std::deque<FTrajectoryPoint> g_TrajectoryHistory;
static std::vector<std::string> g_DiskTextures;
static int g_CurrentDiskState = -1;
static float g_OriginalThinRs = 0.0f;
static float g_OriginalHopper = 0.0f;
static bool g_DiskStateChanged = false;
static Npgs::Runtime::Graphics::FVulkanSampler* g_GlobalSampler = nullptr;
FGameArgs GameArgs{};
FBlackHoleArgs BlackHoleArgs{};

static double g_GeoState[20];
static bool g_isOutgoing = false;
static double g_UniverseSign = 1.0;

namespace GeodesicIntegrator
{
extern double g_ProperAcceleration[3];
double g_ProperAcceleration[3] = { 0.0, 0.0, 0.0 };
extern double g_ProperTime;
double g_ProperTime = 0.0;

double GetIntermediateSign(const double StartX[4], const double CurrentX[4], double CurrentSign, double a)
{
    if (StartX[1] * CurrentX[1] < 0.0)
    {
        double t = StartX[1] / (StartX[1] - CurrentX[1]);
        double mix_x = StartX[0] + t * (CurrentX[0] - StartX[0]);
        double mix_z = StartX[2] + t * (CurrentX[2] - StartX[2]);
        double rho_cross = std::sqrt(mix_x * mix_x + mix_z * mix_z);
        if (rho_cross < std::abs(a))
            return -CurrentSign;
    }
    return CurrentSign;
}

const double EPS = 1e-16;

void ComputeMetric(const double X[4], double a, double Q, double fade, double signR, bool isOut, double g_down[4][4], double g_up[4][4], double& r_out)
{
    double x = X[0], y = X[1], z = X[2];
    double a2 = a * a;
    double R2 = x * x + y * y + z * z;
    double u_val = R2 - a2;
    double v = 4.0 * a2 * y * y;
    double r2 = 0.0;
    if (u_val >= 0.0) r2 = 0.5 * (u_val + std::sqrt(u_val * u_val + v));
    else r2 = (2.0 * a2 * y * y) / std::fmax(1e-20, std::sqrt(u_val * u_val + v) - u_val);
    double r = signR * std::sqrt(std::fmax(r2, 0.0));
    r_out = r;
    double f = 0.0;
    if (std::abs(r) > 1e-6)
    {
        double num = 2.0 * 0.5 * r * r * r - Q * Q * r * r;
        double den = r * r * r * r + a2 * y * y;
        f = (num / std::fmax(1e-20, den)) * fade;
    }
    double dir = isOut ? -1.0 : 1.0;
    double inv = 1.0 / std::fmax(1e-20, r2 + a2);
    double lx = (dir * r * x - a * z) * inv;
    double ly = (dir * y) / r;
    double lz = (dir * r * z + a * x) * inv;
    double l[4] = { lx, ly, lz, -1.0 };
    double l_down_vec[4] = { lx, ly, lz, 1.0 };
    double eta_down[4] = { 1.0, 1.0, 1.0, -1.0 };
    double eta_up[4] = { 1.0, 1.0, 1.0, -1.0 };
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
        {
            g_down[i][j] = (i == j ? eta_down[i] : 0.0) + f * l_down_vec[i] * l_down_vec[j];
            g_up[i][j] = (i == j ? eta_up[i] : 0.0) - f * l[i] * l[j];
        }
}

void ComputeChristoffel(const double X[4], double a, double Q, double fade, double signR, bool isOut, double Gamma[4][4][4])
{
    double x = X[0], y = X[1], z = X[2];
    double a2 = a * a, Q2 = Q * Q;
    double R2 = x * x + y * y + z * z;
    double u_val = R2 - a2;
    double v = 4.0 * a2 * y * y;
    double S = std::fmax(1e-20, std::sqrt(u_val * u_val + v));
    double r2 = 0.0;
    if (u_val >= 0.0) r2 = 0.5 * (u_val + S);
    else r2 = (2.0 * a2 * y * y) / std::fmax(1e-20, S - u_val);
    double r = signR * std::sqrt(std::fmax(r2, 0.0));
    double Y = 0.0;
    if (std::abs(r) > 1e-10) Y = y / r;
    else
    {
        if (std::abs(a) > 1e-20)
        {
            double signY = (y >= 0.0) ? 1.0 : -1.0;
            Y = signY * signR * std::sqrt(std::fmax(0.0, r2 - u_val)) / std::abs(a);
        }
        else Y = 0.0;
    }
    double dr[4] = { 0.0 };
    dr[0] = (r * x) / S; dr[1] = (Y * (r2 + a2)) / S; dr[2] = (r * z) / S; dr[3] = 0.0;
    double N = r * r * r - Q2 * r2;
    double D = r2 * r2 + a2 * y * y;
    double D_inv = 1.0 / std::fmax(1e-20, D);
    double f = N * D_inv * fade;
    double df[4] = { 0.0 };
    for (int k = 0; k < 3; ++k)
    {
        double dN_k = (3.0 * r2 - 2.0 * Q2 * r) * dr[k];
        double dD_k = 4.0 * r * r2 * dr[k];
        if (k == 1) dD_k += 2.0 * a2 * y;
        df[k] = (dN_k * D - N * dD_k) * D_inv * D_inv * fade;
    }
    double dir = isOut ? -1.0 : 1.0;
    double inv_r2a2 = 1.0 / std::fmax(1e-20, r2 + a2);
    double l_down[4];
    l_down[0] = (dir * r * x - a * z) * inv_r2a2;
    l_down[1] = dir * Y;
    l_down[2] = (dir * r * z + a * x) * inv_r2a2;
    l_down[3] = 1.0;
    double dl_down[3][4] = { 0.0 };
    for (int k = 0; k < 3; ++k)
    {
        double dinv = -inv_r2a2 * inv_r2a2 * 2.0 * r * dr[k];
        double term0 = dir * x * dr[k];
        if (k == 0) term0 += dir * r;
        if (k == 2) term0 -= a;
        dl_down[k][0] = term0 * inv_r2a2 + (dir * r * x - a * z) * dinv;
        if (k == 0) dl_down[k][1] = -dir * Y * x / S;
        else if (k == 1) dl_down[k][1] = dir * r * (1.0 - Y * Y) / S;
        else if (k == 2) dl_down[k][1] = -dir * Y * z / S;
        double term2 = dir * z * dr[k];
        if (k == 2) term2 += dir * r;
        if (k == 0) term2 += a;
        dl_down[k][2] = term2 * inv_r2a2 + (dir * r * z + a * x) * dinv;
        dl_down[k][3] = 0.0;
    }
    double eta_up[4] = { 1.0, 1.0, 1.0, -1.0 };
    double l_up[4] = { l_down[0], l_down[1], l_down[2], -1.0 };
    double g_up[4][4];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            g_up[i][j] = (i == j ? eta_up[i] : 0.0) - f * l_up[i] * l_up[j];
    double dg_down[4][4][4] = { 0.0 };
    for (int k = 0; k < 3; ++k)
        for (int mu = 0; mu < 4; ++mu)
            for (int nu = 0; nu < 4; ++nu)
                dg_down[k][mu][nu] = df[k] * l_down[mu] * l_down[nu] + f * dl_down[k][mu] * l_down[nu] + f * l_down[mu] * dl_down[k][nu];
    for (int lambda = 0; lambda < 4; ++lambda)
        for (int mu = 0; mu < 4; ++mu)
            for (int nu = 0; nu < 4; ++nu)
            {
                double sum = 0.0;
                for (int rho = 0; rho < 4; ++rho)
                    sum += 0.5 * g_up[lambda][rho] * (dg_down[mu][rho][nu] + dg_down[nu][rho][mu] - dg_down[rho][mu][nu]);
                Gamma[lambda][mu][nu] = sum;
            }
}

void EvaluateDerivatives(const double Y[20], double a, double Q, double fade, double signR, bool isOut, double dY[20])
{
    double Gamma[4][4][4];
    ComputeChristoffel(Y, a, Q, fade, signR, isOut, Gamma);
    for (int i = 0; i < 4; ++i) dY[i] = Y[4 + i];
    for (int i = 0; i < 4; ++i)
    {
        double sum = 0;
        for (int mu = 0; mu < 4; ++mu)
            for (int nu = 0; nu < 4; ++nu)
                sum -= Gamma[i][mu][nu] * Y[4 + mu] * Y[4 + nu];
        sum += g_ProperAcceleration[0] * Y[8 + i] + g_ProperAcceleration[1] * Y[12 + i] + g_ProperAcceleration[2] * Y[16 + i];
        dY[4 + i] = sum;
    }
    for (int a_idx = 0; a_idx < 3; ++a_idx)
    {
        int offset = 8 + 4 * a_idx;
        for (int i = 0; i < 4; ++i)
        {
            double sum = 0;
            for (int mu = 0; mu < 4; ++mu)
                for (int nu = 0; nu < 4; ++nu)
                    sum -= Gamma[i][mu][nu] * Y[4 + mu] * Y[offset + nu];
            sum += Y[4 + i] * g_ProperAcceleration[a_idx];
            dY[offset + i] = sum;
        }
    }
}

void GramSchmidt(double Y[20], double a, double Q, double fade, double signR, bool isOut)
{
    double g_down[4][4], g_up[4][4], dummy_r;
    ComputeMetric(Y, a, Q, fade, signR, isOut, g_down, g_up, dummy_r);
    auto dotP = [&](const double* v1, const double* v2)
    {
        double sum = 0;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) sum += g_down[i][j] * v1[i] * v2[j];
        return sum;
    };
    double normU2 = dotP(Y + 4, Y + 4);
    double factorU = 1.0 / std::sqrt(std::fmax(1e-12, std::abs(normU2)));
    for (int i = 0; i < 4; ++i) Y[4 + i] *= factorU;
    double exact_normU2 = dotP(Y + 4, Y + 4);
    for (int a_idx = 0; a_idx < 3; ++a_idx)
    {
        double* E = Y + 8 + 4 * a_idx;
        double dotEU = dotP(E, Y + 4);
        double projU = dotEU / std::min(-1e-12, exact_normU2);
        for (int i = 0; i < 4; ++i) E[i] -= projU * Y[4 + i];
        for (int b_idx = 0; b_idx < a_idx; ++b_idx)
        {
            double* Eb = Y + 8 + 4 * b_idx;
            double dotEEb = dotP(E, Eb);
            double normEb2 = dotP(Eb, Eb);
            double projEb = dotEEb / std::fmax(1e-12, normEb2);
            for (int i = 0; i < 4; ++i) E[i] -= projEb * Eb[i];
        }
        double normE2 = dotP(E, E);
        double factorE = 1.0 / std::sqrt(std::fmax(1e-12, std::abs(normE2)));
        for (int i = 0; i < 4; ++i) E[i] *= factorE;
    }
}

void TransformKS(double X[4], double P[4], double signR, double M, double a, double Q, bool out_to_in)
{
    double x = X[0], y = X[1], z = X[2], t = X[3];
    double px = P[0], py = P[1], pz = P[2], pt = P[3];
    double a2 = a * a, M2 = M * M, Q2 = Q * Q;
    double R2 = x * x + y * y + z * z;
    double u = R2 - a2;
    double v = 4.0 * a2 * y * y;
    double r2 = (u >= 0.0) ? 0.5 * (u + std::sqrt(u * u + v)) : 0.5 * v / std::fmax(1e-20, std::sqrt(u * u + v) - u);
    double r = signR * std::sqrt(std::fmax(r2, 0.0));
    double Delta = r * r - 2.0 * M * r + a2 + Q2;
    double safe_Delta = (Delta >= 0 ? 1 : -1) * std::fmax(std::abs(Delta), EPS);
    double D2 = r * r * r * r + a2 * y * y;
    double safe_D = std::fmax(D2, 1e-12);
    double grad_r[3] = { (r * r * r * x) / safe_D, (r * (r * r + a2) * y) / safe_D, (r * r * r * z) / safe_D };
    double delta_disc = M2 - a2 - Q2;
    double F_r = 0.0, g_r = 0.0;
    double abs_Delta_safe = std::fmax(std::abs(Delta), EPS);
    if (delta_disc > EPS)
    {
        double K = std::sqrt(delta_disc);
        double frac = std::abs(r - (M + K)) / std::fmax(std::abs(r - (M - K)), EPS);
        F_r = 2.0 * M * std::log(abs_Delta_safe) + ((2.0 * M2 - Q2) / K) * std::log(std::fmax(frac, EPS));
        g_r = (a / K) * std::log(std::fmax(frac, EPS));
    }
    else if (delta_disc < -EPS)
    {
        double K = std::sqrt(-delta_disc);
        double atan_arg = std::atan((r - M) / K);
        F_r = 2.0 * M * std::log(abs_Delta_safe) + (2.0 * (2.0 * M2 - Q2) / K) * atan_arg;
        g_r = (2.0 * a / K) * atan_arg;
    }
    else
    {
        double rM = r - M;
        double safe_rM = (rM >= 0 ? 1 : -1) * std::fmax(std::abs(rM), EPS);
        F_r = 4.0 * M * std::log(std::fmax(std::abs(rM), EPS)) - 2.0 * (2.0 * M2 - Q2) / safe_rM;
        g_r = -2.0 * a / safe_rM;
    }
    g_r += 2.0 * std::atan2(a, r);
    double F_prime = 2.0 * (2.0 * M * r - Q2) / safe_Delta;
    double g_prime = 2.0 * a / safe_Delta - 2.0 * a / (r * r + a * a);
    double Ly = z * px - x * pz;
    double K_p = F_prime * pt + g_prime * Ly;
    double dir = out_to_in ? -1.0 : 1.0;
    double angle = -dir * g_r;
    double time_shift = -dir * F_r;
    double P_tilde[3] = { px + dir * grad_r[0] * K_p, py + dir * grad_r[1] * K_p, pz + dir * grad_r[2] * K_p };
    double cos_a = std::cos(angle), sin_a = std::sin(angle);
    X[0] = x * cos_a + z * sin_a; X[1] = y; X[2] = z * cos_a - x * sin_a; X[3] = t + time_shift;
    P[0] = P_tilde[2] * sin_a + P_tilde[0] * cos_a; P[1] = P_tilde[1]; P[2] = -P_tilde[0] * sin_a + P_tilde[2] * cos_a; P[3] = pt;
}

void ChangeIndex(double V[4], const double g[4][4])
{
    double out[4] = { 0 };
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) out[i] += g[i][j] * V[j];
    for (int i = 0; i < 4; ++i) V[i] = out[i];
}

void CheckAndSwitchCoords(double Y[20], double a, double Q, double fade, double& signR, bool& isOut)
{
    double g_down[4][4], g_up[4][4], dummy_r;
    ComputeMetric(Y, a, Q, fade, signR, isOut, g_down, g_up, dummy_r);
    double current_Sum = 0;
    for (int i = 0; i < 4; ++i) current_Sum += std::abs(Y[4 + i]);
    double Y_test[20];
    for (int i = 0; i < 20; ++i) Y_test[i] = Y[i];
    ChangeIndex(Y_test + 4, g_down);
    for (int k = 0; k < 3; ++k) ChangeIndex(Y_test + 8 + 4 * k, g_down);
    double X_orig[4] = { Y_test[0], Y_test[1], Y_test[2], Y_test[3] };
    TransformKS(Y_test, Y_test + 4, signR, 0.5, a, Q, isOut);
    for (int k = 0; k < 3; ++k)
    {
        double dummyX[4] = { X_orig[0], X_orig[1], X_orig[2], X_orig[3] };
        TransformKS(dummyX, Y_test + 8 + 4 * k, signR, 0.5, a, Q, isOut);
    }
    double test_g_down[4][4], test_g_up[4][4];
    ComputeMetric(Y_test, a, Q, fade, signR, !isOut, test_g_down, test_g_up, dummy_r);
    ChangeIndex(Y_test + 4, test_g_up);
    for (int k = 0; k < 3; ++k) ChangeIndex(Y_test + 8 + 4 * k, test_g_up);
    double test_Sum = 0;
    for (int i = 0; i < 4; ++i) test_Sum += std::abs(Y_test[4 + i]);
    if (current_Sum > 2.0 * test_Sum)
    {
        for (int i = 0; i < 20; ++i) Y[i] = Y_test[i];
        isOut = !isOut;
    }
}

void InitializeGeodesicState(glm::vec3 pos, glm::vec3 vel, double a, double Q)
{
    g_GeoState[0] = pos.x; g_GeoState[1] = pos.y; g_GeoState[2] = pos.z; g_GeoState[3] = 0.0;
    double g_down[4][4] = { 0.0 }, g_up[4][4] = {0.0}, r_out=0.0;
    ComputeMetric(g_GeoState, a, Q, 1.0, g_UniverseSign, g_isOutgoing, g_down, g_up, r_out);
    double v_coord[4] = { vel.x, vel.y, vel.z, 1.0 };
    double V_sq = 0.0;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) V_sq += g_down[i][j] * v_coord[i] * v_coord[j];
    if (V_sq < -1e-6)
    {
        double Ut = 1.0 / std::sqrt(-V_sq);
        g_GeoState[4] = vel.x * Ut; g_GeoState[5] = vel.y * Ut; g_GeoState[6] = vel.z * Ut; g_GeoState[7] = Ut;
    }
    else
    {
        if (g_down[3][3] < -1e-6)
        {
            g_GeoState[4] = 0; g_GeoState[5] = 0; g_GeoState[6] = 0;
            g_GeoState[7] = 1.0 / std::sqrt(-g_down[3][3]);
        }
        else
        {
            double inward_v[4] = { -pos.x * 0.1, -pos.y * 0.1, -pos.z * 0.1, 1.0 };
            double dV_sq = 0;
            for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) dV_sq += g_down[i][j] * inward_v[i] * inward_v[j];
            if (dV_sq < 0.0)
            {
                double Ut = 1.0 / std::sqrt(-dV_sq);
                g_GeoState[4] = inward_v[0] * Ut; g_GeoState[5] = inward_v[1] * Ut;
                g_GeoState[6] = inward_v[2] * Ut; g_GeoState[7] = Ut;
            }
            else { g_GeoState[4] = 0; g_GeoState[5] = 0; g_GeoState[6] = 0; g_GeoState[7] = 1.0; }
        }
    }
    double U_down[4] = { 0, 0, 0, 0 };
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) U_down[i] += g_down[i][j] * g_GeoState[4 + j];
    glm::vec3 m_r = -glm::normalize(pos);
    glm::vec3 WorldUp = glm::vec3(0.0, 1.0, 0.0);
    if (std::abs(glm::dot(m_r, WorldUp)) > 0.999f) WorldUp = glm::vec3(1.0, 0.0, 0.0);
    glm::vec3 m_phi = glm::normalize(glm::cross(WorldUp, m_r));
    glm::vec3 m_theta = glm::cross(m_phi, m_r);
    auto projectAndNormalize = [&](double* e, double* e_d)
    {
        double dot_e_U = 0.0;
        for (int i = 0; i < 4; ++i) dot_e_U += e[i] * U_down[i];
        for (int i = 0; i < 4; ++i) e[i] += dot_e_U * g_GeoState[4 + i];
        for (int i = 0; i < 4; ++i)
        {
            e_d[i] = 0.0;
            for (int j = 0; j < 4; ++j) e_d[i] += g_down[i][j] * e[j];
        }
        double norm = 0.0;
        for (int i = 0; i < 4; ++i) norm += e[i] * e_d[i];
        norm = std::sqrt(std::fmax(1e-9, norm));
        for (int i = 0; i < 4; ++i) { e[i] /= norm; e_d[i] /= norm; }
    };
    double e1[4] = { m_r.x, m_r.y, m_r.z, 0.0 };
    double e1_d[4];
    projectAndNormalize(e1, e1_d);
    double e2[4] = { m_theta.x, m_theta.y, m_theta.z, 0.0 };
    double dot_e2_U = 0.0;
    for (int i = 0; i < 4; ++i) dot_e2_U += e2[i] * U_down[i];
    for (int i = 0; i < 4; ++i) e2[i] += dot_e2_U * g_GeoState[4 + i];
    double dot_e2_e1 = 0.0;
    for (int i = 0; i < 4; ++i) dot_e2_e1 += e2[i] * e1_d[i];
    for (int i = 0; i < 4; ++i) e2[i] -= dot_e2_e1 * e1[i];
    double e2_d[4];
    projectAndNormalize(e2, e2_d);
    double e3[4] = { m_phi.x, m_phi.y, m_phi.z, 0.0 };
    double dot_e3_U = 0.0;
    for (int i = 0; i < 4; ++i) dot_e3_U += e3[i] * U_down[i];
    for (int i = 0; i < 4; ++i) e3[i] += dot_e3_U * g_GeoState[4 + i];
    double dot_e3_e1 = 0.0;
    for (int i = 0; i < 4; ++i) dot_e3_e1 += e3[i] * e1_d[i];
    for (int i = 0; i < 4; ++i) e3[i] -= dot_e3_e1 * e1[i];
    double dot_e3_e2 = 0.0;
    for (int i = 0; i < 4; ++i) dot_e3_e2 += e3[i] * e2_d[i];
    for (int i = 0; i < 4; ++i) e3[i] -= dot_e3_e2 * e2[i];
    double e3_d[4];
    projectAndNormalize(e3, e3_d);
    for (int i = 0; i < 4; ++i)
    {
        g_GeoState[8 + i] = m_r.x * e1[i] + m_theta.x * e2[i] + m_phi.x * e3[i];
        g_GeoState[12 + i] = m_r.y * e1[i] + m_theta.y * e2[i] + m_phi.y * e3[i];
        g_GeoState[16 + i] = m_r.z * e1[i] + m_theta.z * e2[i] + m_phi.z * e3[i];
    }
    GramSchmidt(g_GeoState, a, Q, 1.0, g_UniverseSign, g_isOutgoing);
}

void StepRK4(double dtau, double a, double Q)
{
    double fade = 1.0;
    CheckAndSwitchCoords(g_GeoState, a, Q, fade, g_UniverseSign, g_isOutgoing);
    double k1[20], k2[20], k3[20], k4[20], Y_temp[20];
    EvaluateDerivatives(g_GeoState, a, Q, fade, g_UniverseSign, g_isOutgoing, k1);
    for (int i = 0; i < 20; ++i) Y_temp[i] = g_GeoState[i] + 0.5 * dtau * k1[i];
    double sign2 = GetIntermediateSign(g_GeoState, Y_temp, g_UniverseSign, a);
    EvaluateDerivatives(Y_temp, a, Q, fade, sign2, g_isOutgoing, k2);
    for (int i = 0; i < 20; ++i) Y_temp[i] = g_GeoState[i] + 0.5 * dtau * k2[i];
    double sign3 = GetIntermediateSign(g_GeoState, Y_temp, g_UniverseSign, a);
    EvaluateDerivatives(Y_temp, a, Q, fade, sign3, g_isOutgoing, k3);
    for (int i = 0; i < 20; ++i) Y_temp[i] = g_GeoState[i] + dtau * k3[i];
    double sign4 = GetIntermediateSign(g_GeoState, Y_temp, g_UniverseSign, a);
    EvaluateDerivatives(Y_temp, a, Q, fade, sign4, g_isOutgoing, k4);
    double oldX[4] = { g_GeoState[0], g_GeoState[1], g_GeoState[2], g_GeoState[3] };
    for (int i = 0; i < 20; ++i) g_GeoState[i] += (dtau / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    g_UniverseSign = GetIntermediateSign(oldX, g_GeoState, g_UniverseSign, a);
    GramSchmidt(g_GeoState, a, Q, fade, g_UniverseSign, g_isOutgoing);
}
}

FMatrices Matrices;
FLightMaterial LightMaterial;
float cfov = 80.0f;
float camsmth = 1.0f;

_NPGS_BEGIN
using namespace Npgs::Math;


namespace Art = Runtime::Asset;
namespace Grt = Runtime::Graphics;
namespace SysSpa = System::Spatial;

double get_orbit_energy(double x, double a, double q2)
{
    if (x < q2) return 1e100;
    double sq = std::sqrt(std::fmax(0.0, x - q2));
    double F = x * x - 3.0 * x + 2.0 * q2 + 2.0 * a * sq;
    if (F <= 1e-15) return 1e100;
    double num = x * x - 2.0 * x + q2 + a * sq;
    return num / (x * std::sqrt(F));
}

double calculate_KN_ISCO(double M, double a_star, double Q_star)
{
    double a = a_star;
    double q2 = Q_star * Q_star;
    if (a * a + q2 > 1.0 + 1e-9)
    {
        std::cerr << "Error: not a black hole (a^2 + Q^2 > M^2)" << std::endl;
        return -1.0;
    }
    if (abs(a - 1.0) < 1e-7 && q2 < 1e-7) return M * 1.0;
    double left = std::fmax(1.0, q2) + 1e-5;
    double right = 15.0;
    const double invphi = (std::sqrt(5.0) - 1.0) / 2.0;
    const double invphi2 = (3.0 - std::sqrt(5.0)) / 2.0;
    double c = left + invphi2 * (right - left);
    double d = left + invphi * (right - left);
    double fc = get_orbit_energy(c, a, q2);
    double fd = get_orbit_energy(d, a, q2);
    double tol = 1e-11;
    while ((right - left) > tol)
    {
        if (fc < fd) { right = d; d = c; fd = fc; c = left + invphi2 * (right - left); fc = get_orbit_energy(c, a, q2); }
        else { left = c; c = d; fc = fd; d = left + invphi * (right - left); fd = get_orbit_energy(d, a, q2); }
    }
    double x_isco = 0.5 * (left + right);
    return x_isco * M;
}

FApplication::FApplication(const vk::Extent2D& WindowSize, const std::string& WindowTitle,
    bool bEnableVSync, bool bEnableFullscreen)
    :
    _VulkanContext(Grt::FVulkanContext::GetClassInstance()),
    _WindowTitle(WindowTitle),
    _WindowSize(WindowSize),
    _bEnableVSync(bEnableVSync),
    _bEnableFullscreen(bEnableFullscreen)
{
    if (!InitializeWindow())
        NpgsCoreError("Failed to create application.");
}

FApplication::~FApplication()
{
}

void FApplication::Quit()
{
    if (_Window)
        glfwSetWindowShouldClose(_Window, GLFW_TRUE);
}

void FApplication::ExecuteMainRender()
{
    _uiRenderer = std::make_unique<Grt::FVulkanUIRenderer>();

    if (!_uiRenderer->Initialize(_Window))
    {
        NpgsCoreError("Failed to initialize UI renderer");
        return;
    }
    _VulkanContext->RegisterAutoRemovedCallbacks(Grt::FVulkanContext::ECallbackType::kCreateSwapchain, "ImGuiUpdate",
        [this]() {
            ImGui_ImplVulkan_SetMinImageCount(_VulkanContext->GetSwapchainImageCount());
            int fbW, fbH;
            glfwGetFramebufferSize(_Window, &fbW, &fbH);
            if (fbW > 0 && fbH > 0) {
                auto& io = ImGui::GetIO();
                io.DisplayFramebufferScale = ImVec2(
                    (float)fbW / io.DisplaySize.x,
                    (float)fbH / io.DisplaySize.y);
            }
        });

    std::unique_ptr<Grt::FColorAttachment> HistoryAttachment;
    std::unique_ptr<Grt::FColorAttachment> DistortionAttachment;
    std::unique_ptr<Grt::FColorAttachment> VolumetricAttachment;
    std::unique_ptr<Grt::FColorAttachment> BlackHoleAttachment;
    std::unique_ptr<Grt::FColorAttachment> PreBloomAttachment;
    std::unique_ptr<Grt::FColorAttachment> GaussBlurAttachment;
    std::unique_ptr<Grt::FColorAttachment> SceneColorAttachment;
    std::unique_ptr<Grt::FColorAttachment> UIBlurPingAttachment;
    std::unique_ptr<Grt::FColorAttachment> UIBlurPongAttachment;
    std::unique_ptr<Grt::FColorAttachment> UIBlurAttachment;

    vk::RenderingAttachmentInfo DistortionAttachmentInfo = vk::RenderingAttachmentInfo()
        .setImageLayout(vk::ImageLayout::eColorAttachmentOptimal)
        .setLoadOp(vk::AttachmentLoadOp::eClear)
        .setStoreOp(vk::AttachmentStoreOp::eStore)
        .setClearValue(vk::ClearValue(vk::ClearColorValue(std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f})));

    vk::RenderingAttachmentInfo VolumetricAttachmentInfo = vk::RenderingAttachmentInfo()
        .setImageLayout(vk::ImageLayout::eColorAttachmentOptimal)
        .setLoadOp(vk::AttachmentLoadOp::eClear)
        .setStoreOp(vk::AttachmentStoreOp::eStore)
        .setClearValue(vk::ClearValue(vk::ClearColorValue(std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f})));

    vk::RenderingAttachmentInfo BlackHoleAttachmentInfo = vk::RenderingAttachmentInfo()
        .setImageLayout(vk::ImageLayout::eColorAttachmentOptimal)
        .setLoadOp(vk::AttachmentLoadOp::eClear)
        .setStoreOp(vk::AttachmentStoreOp::eStore)
        .setClearValue(vk::ClearValue(vk::ClearColorValue(std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f})));

    vk::RenderingAttachmentInfo HistoryAttachmentInfo = vk::RenderingAttachmentInfo()
        .setImageLayout(vk::ImageLayout::eColorAttachmentOptimal)
        .setLoadOp(vk::AttachmentLoadOp::eClear)
        .setStoreOp(vk::AttachmentStoreOp::eStore)
        .setClearValue(vk::ClearValue(vk::ClearColorValue(std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f})));

    vk::RenderingAttachmentInfo PreBloomAttachmentInfo = vk::RenderingAttachmentInfo()
        .setImageLayout(vk::ImageLayout::eColorAttachmentOptimal)
        .setLoadOp(vk::AttachmentLoadOp::eClear)
        .setStoreOp(vk::AttachmentStoreOp::eStore)
        .setClearValue(vk::ClearValue(vk::ClearColorValue(std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f})));

    vk::RenderingAttachmentInfo GaussBlurAttachmentInfo = vk::RenderingAttachmentInfo()
        .setImageLayout(vk::ImageLayout::eColorAttachmentOptimal)
        .setLoadOp(vk::AttachmentLoadOp::eClear)
        .setStoreOp(vk::AttachmentStoreOp::eStore)
        .setClearValue(vk::ClearValue(vk::ClearColorValue(std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f})));

    vk::RenderingAttachmentInfo SceneColorAttachmentInfo = vk::RenderingAttachmentInfo()
        .setImageLayout(vk::ImageLayout::eColorAttachmentOptimal)
        .setLoadOp(vk::AttachmentLoadOp::eClear)
        .setStoreOp(vk::AttachmentStoreOp::eStore);

    vk::Extent2D HalfWindowSize = { _WindowSize.width / 2, _WindowSize.height / 2 };
    if (HalfWindowSize.width == 0) HalfWindowSize.width = 1;
    if (HalfWindowSize.height == 0) HalfWindowSize.height = 1;

    auto CreateFramebuffers = [&]() -> void
    {
        _VulkanContext->WaitIdle();

        vk::Extent2D HalfWinSize = { _WindowSize.width / 2, _WindowSize.height / 2 };
        if (HalfWinSize.width == 0) HalfWinSize.width = 1;
        if (HalfWinSize.height == 0) HalfWinSize.height = 1;

        HistoryAttachment = std::make_unique<Grt::FColorAttachment>(
            vk::Format::eR16G16B16A16Sfloat, _WindowSize, 1, vk::SampleCountFlagBits::e1,
            vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eTransferSrc | vk::ImageUsageFlagBits::eTransferDst);

        DistortionAttachment = std::make_unique<Grt::FColorAttachment>(
            vk::Format::eR32G32B32A32Sfloat, HalfWinSize, 1, vk::SampleCountFlagBits::e1,
            vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eSampled);

        VolumetricAttachment = std::make_unique<Grt::FColorAttachment>(
            vk::Format::eR16G16B16A16Sfloat, HalfWinSize, 1, vk::SampleCountFlagBits::e1,
            vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eSampled);

        BlackHoleAttachment = std::make_unique<Grt::FColorAttachment>(
            vk::Format::eR16G16B16A16Sfloat, _WindowSize, 1, vk::SampleCountFlagBits::e1,
            vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eStorage |
            vk::ImageUsageFlagBits::eTransferSrc | vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eColorAttachment);

        PreBloomAttachment = std::make_unique<Grt::FColorAttachment>(
            vk::Format::eR16G16B16A16Sfloat, _WindowSize, 1, vk::SampleCountFlagBits::e1,
            vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eStorage |
            vk::ImageUsageFlagBits::eTransferSrc | vk::ImageUsageFlagBits::eTransferDst);

        GaussBlurAttachment = std::make_unique<Grt::FColorAttachment>(
            vk::Format::eR16G16B16A16Sfloat, _WindowSize, 1, vk::SampleCountFlagBits::e1,
            vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eStorage |
            vk::ImageUsageFlagBits::eTransferSrc | vk::ImageUsageFlagBits::eTransferDst);

        SceneColorAttachment = std::make_unique<Grt::FColorAttachment>(
            vk::Format::eR8G8B8A8Unorm, _WindowSize, 1, vk::SampleCountFlagBits::e1,
            vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eSampled |
            vk::ImageUsageFlagBits::eTransferSrc | vk::ImageUsageFlagBits::eTransferDst);

        vk::Extent2D hSize = { _WindowSize.width / 2, _WindowSize.height / 2 };
        if (hSize.width == 0) hSize.width = 1;
        if (hSize.height == 0) hSize.height = 1;

        vk::ImageUsageFlags blurUsage = vk::ImageUsageFlagBits::eSampled |
            vk::ImageUsageFlagBits::eTransferSrc | vk::ImageUsageFlagBits::eTransferDst;

        UIBlurPingAttachment = std::make_unique<Grt::FColorAttachment>(
            vk::Format::eR8G8B8A8Unorm, hSize, 1, vk::SampleCountFlagBits::e1, blurUsage);
        UIBlurPongAttachment = std::make_unique<Grt::FColorAttachment>(
            vk::Format::eR8G8B8A8Unorm, hSize, 1, vk::SampleCountFlagBits::e1, blurUsage);
        UIBlurAttachment = std::make_unique<Grt::FColorAttachment>(
            vk::Format::eR8G8B8A8Unorm, hSize, 1, vk::SampleCountFlagBits::e1, blurUsage);

        HistoryAttachmentInfo.setImageView(*HistoryAttachment->GetImageView());
        DistortionAttachmentInfo.setImageView(*DistortionAttachment->GetImageView());
        VolumetricAttachmentInfo.setImageView(*VolumetricAttachment->GetImageView());
        BlackHoleAttachmentInfo.setImageView(*BlackHoleAttachment->GetImageView());
        PreBloomAttachmentInfo.setImageView(*PreBloomAttachment->GetImageView());
        GaussBlurAttachmentInfo.setImageView(*GaussBlurAttachment->GetImageView());
        SceneColorAttachmentInfo.setImageView(*SceneColorAttachment->GetImageView());
    };

    auto DestroyFramebuffers = [&]() -> void
    {
        _VulkanContext->WaitIdle();
    };

    CreateFramebuffers();
    _VulkanContext->RegisterAutoRemovedCallbacks(Grt::FVulkanContext::ECallbackType::kCreateSwapchain, "CreateFramebuffers", CreateFramebuffers);
    _VulkanContext->RegisterAutoRemovedCallbacks(Grt::FVulkanContext::ECallbackType::kDestroySwapchain, "DestroyFramebuffers", DestroyFramebuffers);

    auto* AssetManager = Art::FAssetManager::GetInstance();

    Art::FShader::FResourceInfo QuadResourceInfo
    {
        { { 0, sizeof(FQuadOnlyVertex), false } },
        { { 0, 0, offsetof(FQuadOnlyVertex, Position) } },
        { { 0, 0, false }, { 0, 1, false } }
    };

    Art::FShader::FResourceInfo ComputeResourceInfo
    {
        {}, {}, { { 0, 0, false } },
        { { vk::ShaderStageFlagBits::eCompute, { "ibHorizontal" } } }
    };

    Art::FShader::FResourceInfo BlendResourceInfo
    {
        { { 0, sizeof(FQuadOnlyVertex), false } },
        { { 0, 0, offsetof(FQuadOnlyVertex, Position) } },
        { { 0, 0, false } }
    };

    std::vector<std::string> PrepassShaderFiles({ "ScreenQuad.vert.spv", "BlackHole_prepass.frag.spv" });
    std::vector<std::string> CompositeShaderFiles({ "ScreenQuad.vert.spv", "BlackHole_composite.frag.spv" });
    std::vector<std::string> PreBloomShaderFiles({ "PreBloom.comp.spv" });
    std::vector<std::string> GaussBlurShaderFiles({ "GaussBlur.comp.spv" });
    std::vector<std::string> BlendShaderFiles({ "ScreenQuad.vert.spv", "ColorBlend.frag.spv" });

    VmaAllocationCreateInfo TextureAllocationCreateInfo
    {
        .flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT,
        .usage = VMA_MEMORY_USAGE_GPU_ONLY
    };

    AssetManager->AddAsset<Art::FShader>("BlackHolePrepass", PrepassShaderFiles, QuadResourceInfo);
    AssetManager->AddAsset<Art::FShader>("BlackHoleComposite", CompositeShaderFiles, QuadResourceInfo);
    AssetManager->AddAsset<Art::FShader>("PreBloom", PreBloomShaderFiles, ComputeResourceInfo);
    AssetManager->AddAsset<Art::FShader>("GaussBlur", GaussBlurShaderFiles, ComputeResourceInfo);
    AssetManager->AddAsset<Art::FShader>("Blend", BlendShaderFiles, BlendResourceInfo);

    AssetManager->AddAsset<Art::FTextureCube>("Background0", TextureAllocationCreateInfo, "Universe0Skybox",
        vk::Format::eR8G8B8A8Unorm, vk::Format::eR8G8B8A8Unorm, vk::ImageCreateFlagBits::eMutableFormat, true, false);
    AssetManager->AddAsset<Art::FTextureCube>("Antiground0", TextureAllocationCreateInfo, "Antiverse0Skybox",
        vk::Format::eR8G8B8A8Unorm, vk::Format::eR8G8B8A8Unorm, vk::ImageCreateFlagBits::eMutableFormat, true, false);
    AssetManager->AddAsset<Art::FTextureCube>("Background1", TextureAllocationCreateInfo, "Universe1Skybox",
        vk::Format::eR8G8B8A8Unorm, vk::Format::eR8G8B8A8Unorm, vk::ImageCreateFlagBits::eMutableFormat, true, false);
    AssetManager->AddAsset<Art::FTextureCube>("Antiground1", TextureAllocationCreateInfo, "Antiverse1Skybox",
        vk::Format::eR8G8B8A8Unorm, vk::Format::eR8G8B8A8Unorm, vk::ImageCreateFlagBits::eMutableFormat, true, false);
    AssetManager->AddAsset<Art::FTextureCube>("Background2", TextureAllocationCreateInfo, "Universe2Skybox",
        vk::Format::eR8G8B8A8Unorm, vk::Format::eR8G8B8A8Unorm, vk::ImageCreateFlagBits::eMutableFormat, true, false);
    AssetManager->AddAsset<Art::FTextureCube>("Antiground2", TextureAllocationCreateInfo, "Antiverse2Skybox",
        vk::Format::eR8G8B8A8Unorm, vk::Format::eR8G8B8A8Unorm, vk::ImageCreateFlagBits::eMutableFormat, true, false);

    AssetManager->AddAsset<Art::FTexture2D>("RKKV", TextureAllocationCreateInfo, "ButtonMap/rkkv0.png",
        vk::Format::eR8G8B8A8Unorm, vk::Format::eR8G8B8A8Unorm, vk::ImageCreateFlags(), false, false);

    std::string diskPath = "assets/Textures/Disk";
    if (std::filesystem::exists(diskPath))
    {
        for (const auto& entry : std::filesystem::directory_iterator(diskPath))
        {
            if (entry.path().extension() == ".png" || entry.path().extension() == ".jpg")
            {
                std::string filename = entry.path().filename().string();
                std::string texName = "DiskTex_" + filename;
                AssetManager->AddAsset<Art::FTexture2D>(texName, TextureAllocationCreateInfo, "Disk/" + filename,
                    vk::Format::eR8G8B8A8Unorm, vk::Format::eR8G8B8A8Unorm, vk::ImageCreateFlags(), false, false);
                g_DiskTextures.push_back(texName);
            }
        }
    }

    AssetManager->AddAsset<Art::FTexture2D>("NPGSTexture", TextureAllocationCreateInfo, "nw.png",
        vk::Format::eR8G8B8A8Unorm, vk::Format::eR8G8B8A8Unorm, vk::ImageCreateFlags(), false, false);

    auto* PrepassShader = AssetManager->GetAsset<Art::FShader>("BlackHolePrepass");
    auto* CompositeShader = AssetManager->GetAsset<Art::FShader>("BlackHoleComposite");
    auto* PreBloomShader = AssetManager->GetAsset<Art::FShader>("PreBloom");
    auto* GaussBlurShader = AssetManager->GetAsset<Art::FShader>("GaussBlur");
    auto* BlendShader = AssetManager->GetAsset<Art::FShader>("Blend");
    auto* Background0 = AssetManager->GetAsset<Art::FTextureCube>("Background0");
    auto* Antiground0 = AssetManager->GetAsset<Art::FTextureCube>("Antiground0");
    auto* Background1 = AssetManager->GetAsset<Art::FTextureCube>("Background1");
    auto* Antiground1 = AssetManager->GetAsset<Art::FTextureCube>("Antiground1");
    auto* Background2 = AssetManager->GetAsset<Art::FTextureCube>("Background2");
    auto* Antiground2 = AssetManager->GetAsset<Art::FTextureCube>("Antiground2");
    auto* RKKV = AssetManager->GetAsset<Art::FTexture2D>("RKKV");
    auto* NPGSTexture = AssetManager->GetAsset<Art::FTexture2D>("NPGSTexture");

    Grt::FShaderResourceManager::FUniformBufferCreateInfo GameArgsCreateInfo
    {
        .Name = "GameArgs",
        .Fields = { "Resolution", "FovRadians", "Time","GameTime", "TimeDelta", "TimeRate" },
        .Set = 0, .Binding = 0, .Usage = vk::DescriptorType::eUniformBuffer
    };
    Grt::FShaderResourceManager::FUniformBufferCreateInfo PrepassGameArgsCreateInfo = GameArgsCreateInfo;
    PrepassGameArgsCreateInfo.Name = "GameArgsPrepass";

    Grt::FShaderResourceManager::FUniformBufferCreateInfo BlackHoleArgsCreateInfo
    {
        .Name = "BlackHoleArgs",
        .Fields = { "InverseCamRot;", "BlackHoleRelativePosRs", "BlackHoleRelativeDiskNormal","BlackHoleRelativeDiskTangen","CameraVelocity","ie1_up","ie1_up","ie2_up","ie3_up","iU_up",
                    "iCamDataCoordisOutgoing","DEBUG","Prepass","Whitehole","InWhichUniverse","Grid","EnableHeatHaze","EnableShadowCulling", "ObserverMode","Polarization","iUseImageDisk",
                    "Quality","UniverseSign", "BlackHoleTime","BlackHoleMassSol", "Spin","Q", "Mu", "AccretionRate","BackShiftMax",
                    "DensestarsurfaceR","DensestarBlackbodyIntensityExponent","DensestarRedShiftColorExponent","DensestarRedShiftIntensityExponent","DensestarBrightmut",
                    "InterRadiusRs", "OuterRadiusRs","ThinRs","Hopper", "Brightmut","Darkmut","Reddening","Saturation",
                    "BlackbodyIntensityExponent","RedShiftColorExponent","RedShiftIntensityExponent","ImageRotationSpeed","PolarizationAngle",
                    "HeatHaze","BackgroundBrightmut","PhotonRingBoost","PhotonRingColorTempBoost","BoostRot",
                    "JetRedShiftIntensityExponent","JetBrightmut","JetSaturation","JetShiftMax","BlendWeight" },
        .Set = 0, .Binding = 1, .Usage = vk::DescriptorType::eUniformBuffer
    };

    auto ShaderResourceManager = Grt::FShaderResourceManager::GetInstance();
    ShaderResourceManager->CreateBuffers<FGameArgs>(GameArgsCreateInfo);
    ShaderResourceManager->CreateBuffers<FGameArgs>(PrepassGameArgsCreateInfo);
    ShaderResourceManager->CreateBuffers<FBlackHoleArgs>(BlackHoleArgsCreateInfo);

    vk::SamplerCreateInfo SamplerCreateInfo = Art::FTextureBase::CreateDefaultSamplerCreateInfo();
    std::vector<vk::DescriptorImageInfo> ImageInfos;
    Grt::FVulkanSampler Sampler(SamplerCreateInfo);
    g_GlobalSampler = &Sampler;

    SamplerCreateInfo.setMagFilter(vk::Filter::eLinear).setMinFilter(vk::Filter::eLinear).setMipmapMode(vk::SamplerMipmapMode::eNearest);
    Grt::FVulkanSampler FramebufferSampler(SamplerCreateInfo);
    SamplerCreateInfo.setMagFilter(vk::Filter::eNearest).setMinFilter(vk::Filter::eNearest);
    Grt::FVulkanSampler PointSampler(SamplerCreateInfo);

    auto CreatePostDescriptors = [&]() -> void
    {
        ImageInfos.clear(); ImageInfos.push_back(NPGSTexture->CreateDescriptorImageInfo(Sampler));
        PrepassShader->WriteSharedDescriptors(1, 9, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(Background0->CreateDescriptorImageInfo(Sampler));
        PrepassShader->WriteSharedDescriptors(1, 1, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(Antiground0->CreateDescriptorImageInfo(Sampler));
        PrepassShader->WriteSharedDescriptors(1, 2, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(Background1->CreateDescriptorImageInfo(Sampler));
        PrepassShader->WriteSharedDescriptors(1, 3, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(Antiground1->CreateDescriptorImageInfo(Sampler));
        PrepassShader->WriteSharedDescriptors(1, 4, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(Background2->CreateDescriptorImageInfo(Sampler));
        PrepassShader->WriteSharedDescriptors(1, 5, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(Antiground2->CreateDescriptorImageInfo(Sampler));
        PrepassShader->WriteSharedDescriptors(1, 6, vk::DescriptorType::eCombinedImageSampler, ImageInfos);

        ImageInfos.clear();
        vk::DescriptorImageInfo HistoryFrameImageInfo(nullptr, *HistoryAttachment->GetImageView(), vk::ImageLayout::eShaderReadOnlyOptimal);
        ImageInfos.push_back(HistoryFrameImageInfo);
        PrepassShader->WriteSharedDescriptors(1, 0, vk::DescriptorType::eSampledImage, ImageInfos);
        CompositeShader->WriteSharedDescriptors(1, 0, vk::DescriptorType::eSampledImage, ImageInfos);

        ImageInfos.clear(); ImageInfos.push_back(Background0->CreateDescriptorImageInfo(Sampler));
        CompositeShader->WriteSharedDescriptors(1, 1, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(Antiground0->CreateDescriptorImageInfo(Sampler));
        CompositeShader->WriteSharedDescriptors(1, 2, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(Background1->CreateDescriptorImageInfo(Sampler));
        CompositeShader->WriteSharedDescriptors(1, 3, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(Antiground1->CreateDescriptorImageInfo(Sampler));
        CompositeShader->WriteSharedDescriptors(1, 4, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(Background2->CreateDescriptorImageInfo(Sampler));
        CompositeShader->WriteSharedDescriptors(1, 5, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(Antiground2->CreateDescriptorImageInfo(Sampler));
        CompositeShader->WriteSharedDescriptors(1, 6, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(NPGSTexture->CreateDescriptorImageInfo(Sampler));
        CompositeShader->WriteSharedDescriptors(1, 9, vk::DescriptorType::eCombinedImageSampler, ImageInfos);

        ImageInfos.clear();
        vk::DescriptorImageInfo DistortionImageInfo(*PointSampler, *DistortionAttachment->GetImageView(), vk::ImageLayout::eShaderReadOnlyOptimal);
        ImageInfos.push_back(DistortionImageInfo);
        CompositeShader->WriteSharedDescriptors(1, 7, vk::DescriptorType::eCombinedImageSampler, ImageInfos);

        ImageInfos.clear();
        vk::DescriptorImageInfo VolumetricImageInfo(*FramebufferSampler, *VolumetricAttachment->GetImageView(), vk::ImageLayout::eShaderReadOnlyOptimal);
        ImageInfos.push_back(VolumetricImageInfo);
        CompositeShader->WriteSharedDescriptors(1, 8, vk::DescriptorType::eCombinedImageSampler, ImageInfos);

        vk::DescriptorImageInfo BlackHoleImageInfo(*FramebufferSampler, *BlackHoleAttachment->GetImageView(), vk::ImageLayout::eShaderReadOnlyOptimal);
        vk::DescriptorImageInfo PreBloomImageInfoForSample(*FramebufferSampler, *PreBloomAttachment->GetImageView(), vk::ImageLayout::eShaderReadOnlyOptimal);
        vk::DescriptorImageInfo PreBloomImageInfoForStore(*FramebufferSampler, *PreBloomAttachment->GetImageView(), vk::ImageLayout::eGeneral);
        vk::DescriptorImageInfo GaussBlurImageInfoForSample(*FramebufferSampler, *GaussBlurAttachment->GetImageView(), vk::ImageLayout::eShaderReadOnlyOptimal);
        vk::DescriptorImageInfo GaussBlurImageInfoForStore(*FramebufferSampler, *GaussBlurAttachment->GetImageView(), vk::ImageLayout::eGeneral);

        ImageInfos.clear(); ImageInfos.push_back(BlackHoleImageInfo);
        PreBloomShader->WriteSharedDescriptors(1, 0, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(PreBloomImageInfoForStore);
        PreBloomShader->WriteSharedDescriptors(1, 1, vk::DescriptorType::eStorageImage, ImageInfos);

        ImageInfos.clear(); ImageInfos.push_back(PreBloomImageInfoForSample);
        GaussBlurShader->WriteSharedDescriptors(1, 0, vk::DescriptorType::eCombinedImageSampler, ImageInfos);
        ImageInfos.clear(); ImageInfos.push_back(GaussBlurImageInfoForStore);
        GaussBlurShader->WriteSharedDescriptors(1, 1, vk::DescriptorType::eStorageImage, ImageInfos);

        ImageInfos.clear(); ImageInfos.push_back(BlackHoleImageInfo); ImageInfos.push_back(GaussBlurImageInfoForSample);
        BlendShader->WriteSharedDescriptors(1, 0, vk::DescriptorType::eCombinedImageSampler, ImageInfos);

        if (_uiRenderer && UIBlurAttachment)
        {
            _uiRenderer->AddTexture(*FramebufferSampler, *UIBlurAttachment->GetImageView(), vk::ImageLayout::eShaderReadOnlyOptimal);
        }
    };

    CreatePostDescriptors();
    _VulkanContext->RegisterAutoRemovedCallbacks(Grt::FVulkanContext::ECallbackType::kCreateSwapchain, "CreatePostDescriptor", CreatePostDescriptors);

    std::vector<std::string> BindShaders{ "BlackHoleComposite", "PreBloom", "GaussBlur", "Blend" };
    ShaderResourceManager->BindShadersToBuffers("GameArgs", BindShaders);
    ShaderResourceManager->BindShaderToBuffers("GameArgsPrepass", "BlackHolePrepass");
    ShaderResourceManager->BindShaderToBuffers("BlackHoleArgs", "BlackHolePrepass");
    ShaderResourceManager->BindShaderToBuffers("BlackHoleArgs", "BlackHoleComposite");

    ImTextureID RKKVID = _uiRenderer->AddTexture(*FramebufferSampler, *RKKV->GetImageView(), vk::ImageLayout::eShaderReadOnlyOptimal);

#include "Vertices.inc"

    Grt::FDeviceLocalBuffer QuadOnlyVertexBuffer(QuadOnlyVertices.size() * sizeof(FQuadOnlyVertex), vk::BufferUsageFlagBits::eVertexBuffer);
    QuadOnlyVertexBuffer.CopyData(QuadOnlyVertices);

    std::vector<vk::DynamicState> dynamicStates = { vk::DynamicState::eViewport, vk::DynamicState::eScissor };

    auto* PipelineManager = Grt::FPipelineManager::GetInstance();
    vk::PipelineColorBlendAttachmentState ColorBlendAttachmentState = vk::PipelineColorBlendAttachmentState()
        .setColorWriteMask(vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG | vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA);

    std::array<vk::Format, 2> PrepassFormats{ vk::Format::eR32G32B32A32Sfloat, vk::Format::eR16G16B16A16Sfloat };
    vk::PipelineRenderingCreateInfo PrepassRenderingCreateInfo = vk::PipelineRenderingCreateInfo()
        .setColorAttachmentCount(2).setColorAttachmentFormats(PrepassFormats);

    Grt::FGraphicsPipelineCreateInfoPack PrepassCreateInfoPack;
    PrepassCreateInfoPack.GraphicsPipelineCreateInfo.setPNext(&PrepassRenderingCreateInfo);
    PrepassCreateInfoPack.InputAssemblyStateCreateInfo.setTopology(vk::PrimitiveTopology::eTriangleList);
    PrepassCreateInfoPack.ColorBlendAttachmentStates = { ColorBlendAttachmentState, ColorBlendAttachmentState };
    PrepassCreateInfoPack.Viewports.clear(); PrepassCreateInfoPack.Scissors.clear();
    PrepassCreateInfoPack.DynamicStates = dynamicStates;
    PipelineManager->CreateGraphicsPipeline("BlackHolePrepassPipeline", "BlackHolePrepass", PrepassCreateInfoPack);

    std::array<vk::Format, 1> CompositeFormat{ vk::Format::eR8G8B8A8Unorm };
    vk::PipelineRenderingCreateInfo CompositeRenderingCreateInfo = vk::PipelineRenderingCreateInfo()
        .setColorAttachmentCount(1).setColorAttachmentFormats(CompositeFormat);

    Grt::FGraphicsPipelineCreateInfoPack CompositeCreateInfoPack;
    CompositeCreateInfoPack.GraphicsPipelineCreateInfo.setPNext(&CompositeRenderingCreateInfo);
    CompositeCreateInfoPack.InputAssemblyStateCreateInfo.setTopology(vk::PrimitiveTopology::eTriangleList);
    CompositeCreateInfoPack.ColorBlendAttachmentStates.emplace_back(ColorBlendAttachmentState);
    CompositeCreateInfoPack.Viewports.clear(); CompositeCreateInfoPack.Scissors.clear();
    CompositeCreateInfoPack.DynamicStates = dynamicStates;
    PipelineManager->CreateGraphicsPipeline("BlackHoleCompositePipeline", "BlackHoleComposite", CompositeCreateInfoPack);

    std::array<vk::Format, 1> SceneColorFormat{ vk::Format::eR8G8B8A8Unorm };
    vk::PipelineRenderingCreateInfo BlendRenderingCreateInfo = vk::PipelineRenderingCreateInfo()
        .setColorAttachmentCount(1).setColorAttachmentFormats(SceneColorFormat);
    Grt::FGraphicsPipelineCreateInfoPack BlendCreateInfoPack = CompositeCreateInfoPack;
    BlendCreateInfoPack.DynamicStates = dynamicStates;
    BlendCreateInfoPack.Viewports.clear(); BlendCreateInfoPack.Scissors.clear();
    BlendCreateInfoPack.GraphicsPipelineCreateInfo.setPNext(&BlendRenderingCreateInfo);
    PipelineManager->CreateGraphicsPipeline("BlendPipeline", "Blend", BlendCreateInfoPack);
    PipelineManager->CreateComputePipeline("PreBloomPipeline", "PreBloom");
    PipelineManager->CreateComputePipeline("GaussBlurPipeline", "GaussBlur");

    vk::Pipeline PrepassPipeline, CompositePipeline, PreBloomPipeline, GaussBlurPipeline, BlendPipeline;

    auto GetPipelines = [&]() -> void
    {
        PrepassPipeline = PipelineManager->GetPipeline("BlackHolePrepassPipeline");
        CompositePipeline = PipelineManager->GetPipeline("BlackHoleCompositePipeline");
        PreBloomPipeline = PipelineManager->GetPipeline("PreBloomPipeline");
        GaussBlurPipeline = PipelineManager->GetPipeline("GaussBlurPipeline");
        BlendPipeline = PipelineManager->GetPipeline("BlendPipeline");
    };

    GetPipelines();
    _VulkanContext->RegisterAutoRemovedCallbacks(Grt::FVulkanContext::ECallbackType::kCreateSwapchain, "GetPipelines", GetPipelines);

    auto PrepassPipelineLayout = PipelineManager->GetPipelineLayout("BlackHolePrepassPipeline");
    auto CompositePipelineLayout = PipelineManager->GetPipelineLayout("BlackHoleCompositePipeline");
    auto PreBloomPipelineLayout = PipelineManager->GetPipelineLayout("PreBloomPipeline");
    auto GaussBlurPipelineLayout = PipelineManager->GetPipelineLayout("GaussBlurPipeline");
    auto BlendPipelineLayout = PipelineManager->GetPipelineLayout("BlendPipeline");

    std::vector<Grt::FVulkanFence> InFlightFences;
    std::vector<Grt::FVulkanSemaphore> Semaphores_ImageAvailable;
    std::vector<Grt::FVulkanSemaphore> Semaphores_RenderFinished;
    for (std::size_t i = 0; i != Config::Graphics::kMaxFrameInFlight; ++i)
    {
        InFlightFences.emplace_back(vk::FenceCreateFlagBits::eSignaled);
        Semaphores_ImageAvailable.emplace_back(vk::SemaphoreCreateFlags());
        Semaphores_RenderFinished.emplace_back(vk::SemaphoreCreateFlags());
    }

    std::vector<Grt::FVulkanCommandBuffer> GraphicsCommandBuffers(Config::Graphics::kMaxFrameInFlight);
    _VulkanContext->GetGraphicsCommandPool().AllocateBuffers(vk::CommandBufferLevel::ePrimary, GraphicsCommandBuffers);

    vk::DeviceSize Offset = 0;
    std::uint32_t  CurrentFrame = 0;
    vk::ImageSubresourceRange SubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1);

    auto InitHistoryFrame = [&]() -> void
    {
        vk::ImageMemoryBarrier2 InitHistoryBarrier(
            vk::PipelineStageFlagBits2::eTopOfPipe, vk::AccessFlagBits2::eNone,
            vk::PipelineStageFlagBits2::eFragmentShader, vk::AccessFlagBits2::eShaderRead,
            vk::ImageLayout::eUndefined, vk::ImageLayout::eShaderReadOnlyOptimal,
            vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *HistoryAttachment->GetImage(), SubresourceRange);
        vk::DependencyInfo depInfo = vk::DependencyInfo().setDependencyFlags(vk::DependencyFlagBits::eByRegion).setImageMemoryBarriers(InitHistoryBarrier);
        auto& CmdBuf = _VulkanContext->GetTransferCommandBuffer();
        CmdBuf.Begin(); CmdBuf->pipelineBarrier2(depInfo); CmdBuf.End();
        _VulkanContext->ExecuteGraphicsCommands(CmdBuf);
    };
    InitHistoryFrame();
    _VulkanContext->RegisterAutoRemovedCallbacks(Grt::FVulkanContext::ECallbackType::kCreateSwapchain, "InitHistoryFrame", InitHistoryFrame);

    glm::vec4 LastBlackHoleRelativePos(0.0f, 0.0f, 0.0f, 1.0f);
    glm::mat4x4 lastdir(0.0f);

    // Force initial orbit position before first frame
    _FreeCamera->ProcessOrbital(0.0, 0.0);

    while (!glfwWindowShouldClose(_Window))
    {
        while (glfwGetWindowAttrib(_Window, GLFW_ICONIFIED)) glfwWaitEvents();

        InFlightFences[CurrentFrame].WaitAndReset();
        glfwPollEvents();

        if (g_bRequestScreenshot)
        {
            g_bRequestScreenshot = false;
            std::cout << "[Screenshot] Starting capture..." << std::endl;
            _VulkanContext->WaitIdle();

            vk::Extent2D OldWindowSize = _WindowSize;
            FGameArgs OldGameArgs = GameArgs;
            FBlackHoleArgs OldBlackHoleArgs = BlackHoleArgs;
            float Old_cfov = cfov;

            _WindowSize = vk::Extent2D{ 3840, 2160 };
            GameArgs.Resolution = glm::vec2(3840.0f, 2160.0f);
            BlackHoleArgs.Prepass = 0; BlackHoleArgs.Quality = 10.0f;

            CreateFramebuffers(); CreatePostDescriptors();

            vk::Device device = _VulkanContext->GetDevice();
            vk::DeviceSize bufSize = 3840 * 2160 * 8;
            vk::Buffer stagingBuffer = device.createBuffer(vk::BufferCreateInfo({}, bufSize, vk::BufferUsageFlagBits::eTransferDst));
            auto memReq = device.getBufferMemoryRequirements(stagingBuffer);
            auto memProps = _VulkanContext->GetPhysicalDevice().getMemoryProperties();
            uint32_t memIdx = 0;
            for (uint32_t i = 0; i < memProps.memoryTypeCount; i++)
                if ((memReq.memoryTypeBits & (1 << i)) && (memProps.memoryTypes[i].propertyFlags & (vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent)) == (vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent)) { memIdx = i; break; }
            vk::DeviceMemory stagingMemory = device.allocateMemory(vk::MemoryAllocateInfo(memReq.size, memIdx));
            device.bindBufferMemory(stagingBuffer, stagingMemory, 0);

            auto now = std::chrono::system_clock::now();
            auto time_t_now = std::chrono::system_clock::to_time_t(now);
            char ts[64]; std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&time_t_now));
            std::string folderName = "Screenshot_" + std::string(ts);
            std::filesystem::create_directories(folderName);

            const int AccumFrames = 8;
            for (int i = 1; i <= AccumFrames; i++)
            {
                GameArgs.Time += 0.033f; BlackHoleArgs.BlendWeight = 1.0f / (float)i;
                ShaderResourceManager->UpdateEntrieBuffer(CurrentFrame, "GameArgs", GameArgs);
                FGameArgs PArgs = GameArgs; PArgs.Resolution = GameArgs.Resolution * 0.5f;
                ShaderResourceManager->UpdateEntrieBuffer(CurrentFrame, "GameArgsPrepass", PArgs);
                ShaderResourceManager->UpdateEntrieBuffer(CurrentFrame, "BlackHoleArgs", BlackHoleArgs);

                auto& Cmd = _VulkanContext->GetTransferCommandBuffer(); Cmd.Begin();
                vk::Extent2D Half4K = { 1920, 1080 };
                {
                    vk::ImageMemoryBarrier2 b1(vk::PipelineStageFlagBits2::eTopOfPipe, vk::AccessFlagBits2::eNone, vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *DistortionAttachment->GetImage(), SubresourceRange);
                    vk::ImageMemoryBarrier2 b2(vk::PipelineStageFlagBits2::eTopOfPipe, vk::AccessFlagBits2::eNone, vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *VolumetricAttachment->GetImage(), SubresourceRange);
                    std::array pb = { b1, b2 }; Cmd->pipelineBarrier2(vk::DependencyInfo().setDependencyFlags(vk::DependencyFlagBits::eByRegion).setImageMemoryBarriers(pb));
                }
                std::array<vk::RenderingAttachmentInfo, 2> PAtt = { DistortionAttachmentInfo, VolumetricAttachmentInfo };
                Cmd->beginRendering(vk::RenderingInfo().setRenderArea(vk::Rect2D({0,0}, Half4K)).setLayerCount(1).setColorAttachments(PAtt));
                Cmd->bindVertexBuffers(0, *QuadOnlyVertexBuffer.GetBuffer(), Offset);
                Cmd->bindPipeline(vk::PipelineBindPoint::eGraphics, PrepassPipeline);
                Cmd->bindDescriptorSets(vk::PipelineBindPoint::eGraphics, PrepassPipelineLayout, 0, PrepassShader->GetDescriptorSets(CurrentFrame), {});
                vk::Viewport vpH(0,1080,1920,-1080,0,1); vk::Rect2D scH({0,0},{1920,1080});
                Cmd->setViewport(0,1,&vpH); Cmd->setScissor(0,1,&scH); Cmd->draw(6,1,0,0); Cmd->endRendering();

                {
                    vk::ImageMemoryBarrier2 c1(vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::PipelineStageFlagBits2::eFragmentShader, vk::AccessFlagBits2::eShaderRead, vk::ImageLayout::eColorAttachmentOptimal, vk::ImageLayout::eShaderReadOnlyOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *DistortionAttachment->GetImage(), SubresourceRange);
                    vk::ImageMemoryBarrier2 c2(vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::PipelineStageFlagBits2::eFragmentShader, vk::AccessFlagBits2::eShaderRead, vk::ImageLayout::eColorAttachmentOptimal, vk::ImageLayout::eShaderReadOnlyOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *VolumetricAttachment->GetImage(), SubresourceRange);
                    vk::ImageMemoryBarrier2 c3(vk::PipelineStageFlagBits2::eTopOfPipe, vk::AccessFlagBits2::eNone, vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *BlackHoleAttachment->GetImage(), SubresourceRange);
                    std::array cb = {c1,c2,c3}; Cmd->pipelineBarrier2(vk::DependencyInfo().setImageMemoryBarriers(cb));
                }
                Cmd->beginRendering(vk::RenderingInfo().setRenderArea(vk::Rect2D({0,0},_WindowSize)).setLayerCount(1).setColorAttachments(BlackHoleAttachmentInfo));
                Cmd->bindVertexBuffers(0, *QuadOnlyVertexBuffer.GetBuffer(), Offset);
                Cmd->bindPipeline(vk::PipelineBindPoint::eGraphics, CompositePipeline);
                Cmd->bindDescriptorSets(vk::PipelineBindPoint::eGraphics, CompositePipelineLayout, 0, CompositeShader->GetDescriptorSets(CurrentFrame), {});
                vk::Viewport vpF(0,2160,3840,-2160,0,1); vk::Rect2D scF({0,0},{3840,2160});
                Cmd->setViewport(0,1,&vpF); Cmd->setScissor(0,1,&scF); Cmd->draw(6,1,0,0); Cmd->endRendering();

                {
                    vk::ImageMemoryBarrier2 ps1(vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferRead, vk::ImageLayout::eColorAttachmentOptimal, vk::ImageLayout::eTransferSrcOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *BlackHoleAttachment->GetImage(), SubresourceRange);
                    vk::ImageMemoryBarrier2 pd1(vk::PipelineStageFlagBits2::eFragmentShader, vk::AccessFlagBits2::eShaderRead, vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferWrite, vk::ImageLayout::eShaderReadOnlyOptimal, vk::ImageLayout::eTransferDstOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *HistoryAttachment->GetImage(), SubresourceRange);
                    std::array pb2 = {ps1,pd1}; Cmd->pipelineBarrier2(vk::DependencyInfo().setImageMemoryBarriers(pb2));
                }
                Cmd->copyImage(*BlackHoleAttachment->GetImage(), vk::ImageLayout::eTransferSrcOptimal, *HistoryAttachment->GetImage(), vk::ImageLayout::eTransferDstOptimal,
                    vk::ImageCopy().setSrcSubresource(vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor,0,0,1)).setDstSubresource(vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor,0,0,1)).setExtent(vk::Extent3D(3840,2160,1)));

                {
                    vk::ImageMemoryBarrier2 ps2(vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferRead, vk::PipelineStageFlagBits2::eComputeShader, vk::AccessFlagBits2::eShaderRead, vk::ImageLayout::eTransferSrcOptimal, vk::ImageLayout::eShaderReadOnlyOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *BlackHoleAttachment->GetImage(), SubresourceRange);
                    vk::ImageMemoryBarrier2 pd2(vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferWrite, vk::PipelineStageFlagBits2::eFragmentShader, vk::AccessFlagBits2::eShaderRead, vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::eShaderReadOnlyOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *HistoryAttachment->GetImage(), SubresourceRange);
                    vk::ImageMemoryBarrier2 pb3(vk::PipelineStageFlagBits2::eTopOfPipe, vk::AccessFlagBits2::eNone, vk::PipelineStageFlagBits2::eComputeShader, vk::AccessFlagBits2::eShaderWrite, vk::ImageLayout::eUndefined, vk::ImageLayout::eGeneral, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *PreBloomAttachment->GetImage(), SubresourceRange);
                    std::array bb = {ps2,pd2,pb3}; Cmd->pipelineBarrier2(vk::DependencyInfo().setImageMemoryBarriers(bb));
                }

                uint32_t ssX = (3840+9)/10, ssY = (2160+9)/10;
                Cmd->bindPipeline(vk::PipelineBindPoint::eCompute, PreBloomPipeline);
                Cmd->bindDescriptorSets(vk::PipelineBindPoint::eCompute, PreBloomPipelineLayout, 0, PreBloomShader->GetDescriptorSets(CurrentFrame), {});
                Cmd->dispatch(ssX, ssY, 1);

                {
                    vk::ImageMemoryBarrier2 fb1(vk::PipelineStageFlagBits2::eComputeShader, vk::AccessFlagBits2::eShaderWrite, vk::PipelineStageFlagBits2::eComputeShader, vk::AccessFlagBits2::eShaderRead, vk::ImageLayout::eGeneral, vk::ImageLayout::eShaderReadOnlyOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *PreBloomAttachment->GetImage(), SubresourceRange);
                    vk::ImageMemoryBarrier2 gb1(vk::PipelineStageFlagBits2::eTopOfPipe, vk::AccessFlagBits2::eNone, vk::PipelineStageFlagBits2::eComputeShader, vk::AccessFlagBits2::eShaderWrite, vk::ImageLayout::eUndefined, vk::ImageLayout::eGeneral, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *GaussBlurAttachment->GetImage(), SubresourceRange);
                    std::array gv = {fb1,gb1}; Cmd->pipelineBarrier2(vk::DependencyInfo().setImageMemoryBarriers(gv));
                }
                vk::Bool32 bH = vk::True;
                Cmd->bindPipeline(vk::PipelineBindPoint::eCompute, GaussBlurPipeline);
                Cmd->pushConstants(GaussBlurPipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(vk::Bool32), &bH);
                Cmd->bindDescriptorSets(vk::PipelineBindPoint::eCompute, GaussBlurPipelineLayout, 0, GaussBlurShader->GetDescriptorSets(CurrentFrame), {});
                Cmd->dispatch(ssX, ssY, 1);

                {
                    vk::ImageMemoryBarrier2 cb1(vk::PipelineStageFlagBits2::eComputeShader, vk::AccessFlagBits2::eShaderWrite, vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferRead, vk::ImageLayout::eGeneral, vk::ImageLayout::eTransferSrcOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *GaussBlurAttachment->GetImage(), SubresourceRange);
                    vk::ImageMemoryBarrier2 cb2(vk::PipelineStageFlagBits2::eComputeShader, vk::AccessFlagBits2::eShaderRead, vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferWrite, vk::ImageLayout::eShaderReadOnlyOptimal, vk::ImageLayout::eTransferDstOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *PreBloomAttachment->GetImage(), SubresourceRange);
                    std::array cbb = {cb1,cb2}; Cmd->pipelineBarrier2(vk::DependencyInfo().setImageMemoryBarriers(cbb));
                }
                Cmd->copyImage(*GaussBlurAttachment->GetImage(), vk::ImageLayout::eTransferSrcOptimal, *PreBloomAttachment->GetImage(), vk::ImageLayout::eTransferDstOptimal,
                    vk::ImageCopy().setSrcSubresource(vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor,0,0,1)).setDstSubresource(vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor,0,0,1)).setExtent(vk::Extent3D(3840,2160,1)));

                {
                    vk::ImageMemoryBarrier2 rb1(vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferWrite, vk::PipelineStageFlagBits2::eComputeShader, vk::AccessFlagBits2::eShaderRead, vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::eShaderReadOnlyOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *PreBloomAttachment->GetImage(), SubresourceRange);
                    vk::ImageMemoryBarrier2 rb2(vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferRead, vk::PipelineStageFlagBits2::eComputeShader, vk::AccessFlagBits2::eShaderWrite, vk::ImageLayout::eTransferSrcOptimal, vk::ImageLayout::eGeneral, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *GaussBlurAttachment->GetImage(), SubresourceRange);
                    std::array rbb = {rb1,rb2}; Cmd->pipelineBarrier2(vk::DependencyInfo().setImageMemoryBarriers(rbb));
                }
                bH = vk::False;
                Cmd->pushConstants(GaussBlurPipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(vk::Bool32), &bH);
                Cmd->bindDescriptorSets(vk::PipelineBindPoint::eCompute, GaussBlurPipelineLayout, 0, GaussBlurShader->GetDescriptorSets(CurrentFrame), {});
                Cmd->dispatch(ssX, ssY, 1);

                vk::ImageMemoryBarrier2 _b_guass(vk::PipelineStageFlagBits2::eComputeShader, vk::AccessFlagBits2::eShaderWrite, vk::PipelineStageFlagBits2::eFragmentShader, vk::AccessFlagBits2::eShaderRead, vk::ImageLayout::eGeneral, vk::ImageLayout::eShaderReadOnlyOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *GaussBlurAttachment->GetImage(), SubresourceRange);
                Cmd->pipelineBarrier2(vk::DependencyInfo().setImageMemoryBarriers(_b_guass));

                {
                    vk::ImageMemoryBarrier2 scb(vk::PipelineStageFlagBits2::eTopOfPipe, vk::AccessFlagBits2::eNone, vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *SceneColorAttachment->GetImage(), SubresourceRange);
                    Cmd->pipelineBarrier2(vk::DependencyInfo().setImageMemoryBarriers(scb));
                    Cmd->beginRendering(vk::RenderingInfo().setRenderArea(vk::Rect2D({0,0},_WindowSize)).setLayerCount(1).setColorAttachments(SceneColorAttachmentInfo));
                    Cmd->bindVertexBuffers(0, *QuadOnlyVertexBuffer.GetBuffer(), Offset);
                    Cmd->bindPipeline(vk::PipelineBindPoint::eGraphics, BlendPipeline);
                    Cmd->bindDescriptorSets(vk::PipelineBindPoint::eGraphics, BlendPipelineLayout, 0, BlendShader->GetDescriptorSets(CurrentFrame), {});
                    Cmd->setViewport(0,1,&vpF); Cmd->setScissor(0,1,&scF); Cmd->draw(6,1,0,0); Cmd->endRendering();
                }

                if (i == AccumFrames)
                {
                    vk::ImageMemoryBarrier2 csb(vk::PipelineStageFlagBits2::eFragmentShader, vk::AccessFlagBits2::eShaderRead, vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferRead, vk::ImageLayout::eShaderReadOnlyOptimal, vk::ImageLayout::eTransferSrcOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *HistoryAttachment->GetImage(), SubresourceRange);
                    Cmd->pipelineBarrier2(vk::DependencyInfo().setImageMemoryBarriers(csb));
                    vk::BufferImageCopy _bic(0,0,0,vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor,0,0,1),vk::Offset3D(0,0,0),vk::Extent3D(3840,2160,1));
                    Cmd->copyImageToBuffer(*HistoryAttachment->GetImage(), vk::ImageLayout::eTransferSrcOptimal, stagingBuffer, 1, &_bic);
                    vk::ImageMemoryBarrier2 rbs(vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferRead, vk::PipelineStageFlagBits2::eFragmentShader, vk::AccessFlagBits2::eShaderRead, vk::ImageLayout::eTransferSrcOptimal, vk::ImageLayout::eShaderReadOnlyOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *HistoryAttachment->GetImage(), SubresourceRange);
                    vk::MemoryBarrier2 hb(vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferWrite, vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferRead);
                    Cmd->pipelineBarrier2(vk::DependencyInfo().setMemoryBarriers(hb).setImageMemoryBarriers(rbs));
                }
                Cmd.End();
                _VulkanContext->ExecuteGraphicsCommands(Cmd); _VulkanContext->WaitIdle();
            }

            void* data = device.mapMemory(stagingMemory, 0, bufSize, vk::MemoryMapFlags());
            uint16_t* fp16 = static_cast<uint16_t*>(data);
            float* fp32 = new float[3840*2160*3];
            for (size_t p = 0; p < 3840*2160; p++)
            {
                fp32[p*3+0] = glm::unpackHalf1x16(fp16[p*4+0]);
                fp32[p*3+1] = glm::unpackHalf1x16(fp16[p*4+1]);
                fp32[p*3+2] = glm::unpackHalf1x16(fp16[p*4+2]);
            }
            std::string fname = folderName + "/screenshot_" + std::string(ts) + ".hdr";
            stbi_write_hdr(fname.c_str(), 3840, 2160, 3, fp32);
            std::cout << "  -> Saved: " << fname << std::endl;
            delete[] fp32; device.unmapMemory(stagingMemory);
            device.destroyBuffer(stagingBuffer); device.freeMemory(stagingMemory);

            _WindowSize = OldWindowSize; GameArgs = OldGameArgs; BlackHoleArgs = OldBlackHoleArgs;
            cfov = Old_cfov; _FreeCamera->SetFov(cfov);
            ShaderResourceManager->UpdateEntrieBuffer(CurrentFrame, "GameArgs", GameArgs);
            FGameArgs PR = GameArgs; PR.Resolution = GameArgs.Resolution * 0.5f;
            ShaderResourceManager->UpdateEntrieBuffer(CurrentFrame, "GameArgsPrepass", PR);
            ShaderResourceManager->UpdateEntrieBuffer(CurrentFrame, "BlackHoleArgs", BlackHoleArgs);
            CreateFramebuffers(); CreatePostDescriptors(); InitHistoryFrame();
            CurrentTime = glfwGetTime(); LastFrameTime = CurrentTime;
        }

        _uiRenderer->BeginFrame();
        if (!g_bHideUIAndMouse) RenderDebugUI();
        _uiRenderer->EndFrame();
        _FreeCamera->SetFov(cfov);

        float RsBH = 2.0f * std::abs(BlackHoleArgs.BlackHoleMassSol) * kGravityConstant / std::pow(kSpeedOfLight, 2) * kSolarMass / kLightYearToMeter;

        if (FrameCount <= 10)
        {
            GameArgs.Resolution = glm::vec2(_WindowSize.width, _WindowSize.height);
            GameArgs.FovRadians = glm::radians(_FreeCamera->GetCameraZoom());
            GameArgs.Time = (float)RealityTime; GameArgs.GameTime = (float)GameTime;
            GameArgs.TimeDelta = (float)_DeltaTime; GameArgs.TimeRate = (float)TimeRate;
            LastBlackHoleRelativePos = BlackHoleArgs.BlackHoleRelativePosRs; lastdir = BlackHoleArgs.InverseCamRot;
            ShaderResourceManager->UpdateEntrieBuffer(CurrentFrame, "GameArgs", GameArgs);
            FGameArgs PrepassArgs = GameArgs; PrepassArgs.Resolution = GameArgs.Resolution * 0.5f;
            ShaderResourceManager->UpdateEntrieBuffer(CurrentFrame, "GameArgsPrepass", PrepassArgs);
            BlackHoleArgs.InverseCamRot = glm::mat4_cast(glm::conjugate(_FreeCamera->GetOrientation()));
            BlackHoleArgs.BlackHoleRelativePosRs = glm::vec4(glm::vec3(_FreeCamera->GetViewMatrix() * glm::vec4(0.0f, 0.0f, -0.000f, 1.0f)) / RsBH, 1.0f);
            BlackHoleArgs.BlackHoleRelativeDiskNormal = glm::mat4_cast(_FreeCamera->GetOrientation()) * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
            BlackHoleArgs.BlackHoleRelativeDiskTangen = glm::mat4_cast(_FreeCamera->GetOrientation()) * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
            BlackHoleArgs.CameraVelocity = glm::vec4(0.0f);
            BlackHoleArgs.DEBUG = 0; BlackHoleArgs.Prepass = 0; BlackHoleArgs.Whitehole = 0;
            BlackHoleArgs.InWhichUniverse = 0; BlackHoleArgs.Grid = 0; BlackHoleArgs.EnableHeatHaze = 0;
            BlackHoleArgs.EnableShadowCulling = 0; BlackHoleArgs.ObserverMode = 0; BlackHoleArgs.Polarization = 0;
            BlackHoleArgs.UseImageDisk = 0; BlackHoleArgs.Quality = 1.0f; BlackHoleArgs.UniverseSign = 1.0f;
            BlackHoleArgs.BlackHoleTime = (float)(GameTime * kSpeedOfLight / RsBH / kLightYearToMeter);
            BlackHoleArgs.BlackHoleMassSol = 1.49e7f; BlackHoleArgs.Spin = 0.998f; BlackHoleArgs.Q = 0.0f;
            BlackHoleArgs.Mu = 1.0f; BlackHoleArgs.AccretionRate = 1e-2f; BlackHoleArgs.BackShiftMax = 1.5f;
            BlackHoleArgs.DensestarsurfaceR = 0.0f; BlackHoleArgs.DensestarBlackbodyIntensityExponent = 4.0f;
            BlackHoleArgs.DensestarRedShiftColorExponent = 1.0f; BlackHoleArgs.DensestarRedShiftIntensityExponent = 4.0f;
            BlackHoleArgs.DensestarBrightmut = 1.0f; BlackHoleArgs.InterRadiusRs = 2.0f; BlackHoleArgs.OuterRadiusRs = 25.0f;
            BlackHoleArgs.ThinRs = 0.75f; BlackHoleArgs.Hopper = 0.4f; BlackHoleArgs.Brightmut = 1.0f;
            BlackHoleArgs.Darkmut = 0.5f; BlackHoleArgs.Reddening = 0.3f; BlackHoleArgs.Saturation = 0.5f;
            BlackHoleArgs.BlackbodyIntensityExponent = 1.0f; BlackHoleArgs.RedShiftColorExponent = 1.0f;
            BlackHoleArgs.RedShiftIntensityExponent = 4.0f; BlackHoleArgs.ImageRotationSpeed = 0.00765619656f * (3.06f / 3.0f);
            BlackHoleArgs.PolarizationAngle = 0.0f; BlackHoleArgs.HeatHaze = 0.0f; BlackHoleArgs.BackgroundBrightmut = 0.5f;
            BlackHoleArgs.PhotonRingBoost = 0.0f; BlackHoleArgs.PhotonRingColorTempBoost = 0.0f; BlackHoleArgs.BoostRot = 0.0f;
            BlackHoleArgs.JetRedShiftIntensityExponent = 4.0f; BlackHoleArgs.JetBrightmut = 1.0f;
            BlackHoleArgs.JetSaturation = 0.0f; BlackHoleArgs.JetShiftMax = 3.0f;
        }
        else
        {
            GameArgs.Resolution = glm::vec2(_WindowSize.width, _WindowSize.height);
            GameArgs.FovRadians = glm::radians(_FreeCamera->GetCameraZoom());
            GameArgs.Time = (float)RealityTime; GameArgs.GameTime = (float)GameTime;
            GameArgs.TimeDelta = (float)_DeltaTime; GameArgs.TimeRate = (float)TimeRate;
            LastBlackHoleRelativePos = BlackHoleArgs.BlackHoleRelativePosRs; lastdir = BlackHoleArgs.InverseCamRot;
            ShaderResourceManager->UpdateEntrieBuffer(CurrentFrame, "GameArgs", GameArgs);
            FGameArgs PrepassArgs = GameArgs; PrepassArgs.Resolution = GameArgs.Resolution * 0.5f;
            ShaderResourceManager->UpdateEntrieBuffer(CurrentFrame, "GameArgsPrepass", PrepassArgs);
            BlackHoleArgs.BlackHoleTime = (float)(GameTime * kSpeedOfLight / RsBH / kLightYearToMeter);
            BlackHoleArgs.InverseCamRot = glm::mat4_cast(glm::conjugate(_FreeCamera->GetOrientation()));
            BlackHoleArgs.BlackHoleRelativePosRs = glm::vec4(glm::vec3(_FreeCamera->GetViewMatrix() * glm::vec4(0.0f, 0.0f, -0.000f, 1.0f)) / RsBH, 1.0f);
            BlackHoleArgs.BlackHoleRelativeDiskNormal = glm::mat4_cast(_FreeCamera->GetOrientation()) * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
            BlackHoleArgs.BlackHoleRelativeDiskTangen = glm::mat4_cast(_FreeCamera->GetOrientation()) * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
            glm::vec3 posDiff = _FreeCamera->GetCameraVector(SysSpa::FCamera::EVectorType::kPosition) - LastCameraWorldPos;
            glm::vec3 cv = posDiff * (float)(kLightYearToMeter / (_LastDeltaTime * TimeRate * kSpeedOfLight));
            BlackHoleArgs.CameraVelocity += (float)(1.0 - exp(-_DeltaTime / 0.1)) * (glm::vec4(cv, 0.0f) - BlackHoleArgs.CameraVelocity);
            if (glm::any(glm::isnan(BlackHoleArgs.CameraVelocity)) || glm::any(glm::isinf(BlackHoleArgs.CameraVelocity)))
                BlackHoleArgs.CameraVelocity = glm::vec4(0.0f);
        }

        RsBH = 2.0f * std::abs(BlackHoleArgs.BlackHoleMassSol) * kGravityConstant / std::pow(kSpeedOfLight, 2) * kSolarMass / kLightYearToMeter;
        BlackHoleArgs.BlendWeight = 1.0f;
        if (!(std::abs(glm::quat((lastdir - BlackHoleArgs.InverseCamRot)).w - 0.5) < 0.001 * _DeltaTime || std::abs(glm::quat((lastdir - BlackHoleArgs.InverseCamRot)).w - 0.0) < 0.001 * _DeltaTime) ||
            glm::length(glm::vec3(LastBlackHoleRelativePos - BlackHoleArgs.BlackHoleRelativePosRs)) > (glm::length(glm::vec3(LastBlackHoleRelativePos)) - 1.0) * 0.006 * _DeltaTime ||
            glm::length(BlackHoleArgs.CameraVelocity) > 0.0001f)
        {
            BlackHoleArgs.BlendWeight = 1.0f;
        }
        if ((int)glfwGetTime() < 1)
        {
            _FreeCamera->SetTargetOrbitAxis(glm::vec3(0.f,1.f,0.f)); _FreeCamera->SetTargetOrbitCenter(glm::vec3(0.f));
        }

        if (g_GeodesicMode)
        {
            double total_dtau = _DeltaTime * TimeRate * kSpeedOfLight / RsBH / kLightYearToMeter;
            double dtau_rem = std::abs(total_dtau);
            double dtau_sgn = (total_dtau >= 0.0) ? 1.0 : -1.0;
            const int MAX_STEPS = 1500; int step_cnt = 0;
            while (dtau_rem > 1e-9 && step_cnt < MAX_STEPS)
            {
                double abs_a = std::abs(BlackHoleArgs.Spin * 0.5);
                double rho = std::sqrt(g_GeoState[0]*g_GeoState[0] + g_GeoState[2]*g_GeoState[2]);
                double R = std::sqrt(std::fmax(1e-12, (rho-abs_a)*(rho-abs_a) + g_GeoState[1]*g_GeoState[1]));
                double sR = std::fmax(R/2.0, 1.0);
                double max_dtau_g = 0.005 * sR * std::sqrt(sR);
                double U_mag = std::sqrt(g_GeoState[4]*g_GeoState[4]+g_GeoState[5]*g_GeoState[5]+g_GeoState[6]*g_GeoState[6]);
                double max_dtau_k = (0.05 * R) / std::fmax(1e-6, U_mag);
                double step = std::clamp(std::min(max_dtau_g, max_dtau_k), 0.0005, 5.0);
                double cur_step = std::min(dtau_rem, step);
                GeodesicIntegrator::StepRK4(cur_step * dtau_sgn, BlackHoleArgs.Spin * 0.5, BlackHoleArgs.Q * 0.5);
                GeodesicIntegrator::g_ProperTime += cur_step * dtau_sgn;
                dtau_rem -= cur_step; step_cnt++;
            }
            BlackHoleArgs.BlackHoleRelativePosRs = glm::vec4(g_GeoState[0], g_GeoState[1], g_GeoState[2], g_GeoState[3]);
            BlackHoleArgs.BlackHoleTime = (float)g_GeoState[3];
            BlackHoleArgs.UniverseSign = (float)g_UniverseSign;
            BlackHoleArgs.ObserverMode = -1;
            BlackHoleArgs.iCamDataCoordisOutgoing = g_isOutgoing ? 1 : 0;
            BlackHoleArgs.CameraVelocity = glm::vec4(0.0f);
            glm::mat4 headRot = glm::mat4_cast(glm::conjugate(_FreeCamera->GetOrientation()));
            glm::vec4 e1(0), e2(0), e3(0);
            for (int j = 0; j < 3; ++j)
            {
                glm::vec4 base_j(g_GeoState[8+4*j], g_GeoState[8+4*j+1], g_GeoState[8+4*j+2], g_GeoState[8+4*j+3]);
                e1 -= headRot[0][j] * base_j; e2 -= headRot[1][j] * base_j; e3 -= headRot[2][j] * base_j;
            }
            BlackHoleArgs.ie1_up = e1; BlackHoleArgs.ie2_up = e2; BlackHoleArgs.ie3_up = e3;
            BlackHoleArgs.iU_up = glm::vec4(g_GeoState[4], g_GeoState[5], g_GeoState[6], g_GeoState[7]);
            BlackHoleArgs.InverseCamRot = headRot;
        }
        else
        {
            if (BlackHoleArgs.ObserverMode == -1) BlackHoleArgs.ObserverMode = 0;
        }

        glm::vec3 pos;
        if (g_GeodesicMode) pos = glm::vec3(g_GeoState[0], g_GeoState[1], g_GeoState[2]) * RsBH;
        else pos = _FreeCamera->GetCameraVector(SysSpa::FCamera::EVectorType::kPosition);

        float Mbh = 0.5f * RsBH; float a_spin = BlackHoleArgs.Spin * Mbh; float Q_phys = BlackHoleArgs.Q * Mbh;
        float a2 = a_spin * a_spin; float Q2 = Q_phys * Q_phys;

        glm::vec3 curPos = pos;
        if (LastCameraWorldPos.y * curPos.y <= 0.0f && FrameCount > 1)
        {
            float denom = LastCameraWorldPos.y - curPos.y;
            if (std::abs(denom) > 0)
            {
                float t = LastCameraWorldPos.y / denom;
                float ix = LastCameraWorldPos.x + t * (curPos.x - LastCameraWorldPos.x);
                float iz = LastCameraWorldPos.z + t * (curPos.z - LastCameraWorldPos.z);
                if (ix*ix + iz*iz < a2 && !g_GeodesicMode)
                    BlackHoleArgs.UniverseSign *= -1.0f;
            }
        }
        LastCameraWorldPos = curPos;

        float x2 = pos.x*pos.x, y2 = pos.y*pos.y, z2 = pos.z*pos.z;
        float R2 = x2+y2+z2;
        float r2_calc = 0.5f * ((R2-a2) + std::sqrt((R2-a2)*(R2-a2) + 4.0f*a2*y2));
        float r_val = std::sqrt(r2_calc) * BlackHoleArgs.UniverseSign;
        float delta_disc = Mbh*Mbh - a2 - Q2;
        float h_outer = 0, h_inner = 0;
        bool isNaked = delta_disc < 0;
        if (!isNaked) { float sd = std::sqrt(delta_disc); h_outer = Mbh + sd; h_inner = Mbh - sd; }
        static float s_last_r = r_val;
        if (!isNaked && BlackHoleArgs.Whitehole == 1 && s_last_r > h_inner && r_val <= h_inner && BlackHoleArgs.UniverseSign == 1.0f)
            BlackHoleArgs.InWhichUniverse = (BlackHoleArgs.InWhichUniverse + 1) % 3;
        s_last_r = r_val;

        ShaderResourceManager->UpdateEntrieBuffer(CurrentFrame, "BlackHoleArgs", BlackHoleArgs);
        _VulkanContext->SwapImage(*Semaphores_ImageAvailable[CurrentFrame]);
        uint32_t ImageIndex = _VulkanContext->GetCurrentImageIndex();
        uint32_t WgX = (_WindowSize.width + 9) / 10, WgY = (_WindowSize.height + 9) / 10;

        auto& CurBuf = GraphicsCommandBuffers[CurrentFrame];
        CurBuf.Begin(vk::CommandBufferUsageFlagBits::eOneTimeSubmit);

        vk::Extent2D CurHalf = { _WindowSize.width/2, _WindowSize.height/2 };
        if (CurHalf.width == 0) CurHalf.width = 1; if (CurHalf.height == 0) CurHalf.height = 1;

        vk::Viewport HVP(0, (float)CurHalf.height, (float)CurHalf.width, -(float)CurHalf.height, 0, 1);
        vk::Rect2D HSc({0,0}, CurHalf);
        vk::Viewport FVP(0, (float)_WindowSize.height, (float)_WindowSize.width, -(float)_WindowSize.height, 0, 1);
        vk::Rect2D FSc({0,0}, _WindowSize);

        {
            vk::ImageMemoryBarrier2 b1(vk::PipelineStageFlagBits2::eTopOfPipe, vk::AccessFlagBits2::eNone, vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *DistortionAttachment->GetImage(), SubresourceRange);
            vk::ImageMemoryBarrier2 b2(vk::PipelineStageFlagBits2::eTopOfPipe, vk::AccessFlagBits2::eNone, vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *VolumetricAttachment->GetImage(), SubresourceRange);
            std::array ba = {b1,b2}; CurBuf->pipelineBarrier2(vk::DependencyInfo().setDependencyFlags(vk::DependencyFlagBits::eByRegion).setImageMemoryBarriers(ba));
        }
        std::array<vk::RenderingAttachmentInfo,2> PA = { DistortionAttachmentInfo, VolumetricAttachmentInfo };
        CurBuf->beginRendering(vk::RenderingInfo().setRenderArea(vk::Rect2D({0,0},CurHalf)).setLayerCount(1).setColorAttachments(PA));
        CurBuf->bindVertexBuffers(0, *QuadOnlyVertexBuffer.GetBuffer(), Offset);
        CurBuf->bindPipeline(vk::PipelineBindPoint::eGraphics, PrepassPipeline);
        CurBuf->bindDescriptorSets(vk::PipelineBindPoint::eGraphics, PrepassPipelineLayout, 0, PrepassShader->GetDescriptorSets(CurrentFrame), {});
        CurBuf->setViewport(0,1,&HVP); CurBuf->setScissor(0,1,&HSc); CurBuf->draw(6,1,0,0); CurBuf->endRendering();

        {
            vk::ImageMemoryBarrier2 b1(vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::PipelineStageFlagBits2::eFragmentShader, vk::AccessFlagBits2::eShaderRead, vk::ImageLayout::eColorAttachmentOptimal, vk::ImageLayout::eShaderReadOnlyOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *DistortionAttachment->GetImage(), SubresourceRange);
            vk::ImageMemoryBarrier2 b2(vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::PipelineStageFlagBits2::eFragmentShader, vk::AccessFlagBits2::eShaderRead, vk::ImageLayout::eColorAttachmentOptimal, vk::ImageLayout::eShaderReadOnlyOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *VolumetricAttachment->GetImage(), SubresourceRange);
            vk::ImageMemoryBarrier2 b3(vk::PipelineStageFlagBits2::eTopOfPipe, vk::AccessFlagBits2::eNone, vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *SceneColorAttachment->GetImage(), SubresourceRange);
            std::array ba = {b1,b2,b3}; CurBuf->pipelineBarrier2(vk::DependencyInfo().setImageMemoryBarriers(ba));
        }
        CurBuf->beginRendering(vk::RenderingInfo().setRenderArea(vk::Rect2D({0,0},_WindowSize)).setLayerCount(1).setColorAttachments(SceneColorAttachmentInfo));
        CurBuf->bindVertexBuffers(0, *QuadOnlyVertexBuffer.GetBuffer(), Offset);
        CurBuf->bindPipeline(vk::PipelineBindPoint::eGraphics, CompositePipeline);
        CurBuf->bindDescriptorSets(vk::PipelineBindPoint::eGraphics, CompositePipelineLayout, 0, CompositeShader->GetDescriptorSets(CurrentFrame), {});
        CurBuf->setViewport(0,1,&FVP); CurBuf->setScissor(0,1,&FSc); CurBuf->draw(6,1,0,0); CurBuf->endRendering();

        // Copy SceneColorAttachment -> swapchain
        {
            vk::ImageMemoryBarrier2 s2s(vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferRead, vk::ImageLayout::eColorAttachmentOptimal, vk::ImageLayout::eTransferSrcOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, *SceneColorAttachment->GetImage(), SubresourceRange);
            vk::ImageMemoryBarrier2 scd(vk::PipelineStageFlagBits2::eTopOfPipe, vk::AccessFlagBits2::eNone, vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferWrite, vk::ImageLayout::eUndefined, vk::ImageLayout::eTransferDstOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, _VulkanContext->GetSwapchainImage(ImageIndex), SubresourceRange);
            std::array prepBarr = {s2s, scd};
            CurBuf->pipelineBarrier2(vk::DependencyInfo().setImageMemoryBarriers(prepBarr));
            CurBuf->copyImage(*SceneColorAttachment->GetImage(), vk::ImageLayout::eTransferSrcOptimal, _VulkanContext->GetSwapchainImage(ImageIndex), vk::ImageLayout::eTransferDstOptimal,
                vk::ImageCopy().setSrcSubresource(vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor,0,0,1)).setDstSubresource(vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor,0,0,1)).setExtent(vk::Extent3D(_WindowSize.width,_WindowSize.height,1)));
        }
        {
            vk::ImageMemoryBarrier2 sui(vk::PipelineStageFlagBits2::eTransfer, vk::AccessFlagBits2::eTransferWrite, vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::eColorAttachmentOptimal, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, _VulkanContext->GetSwapchainImage(ImageIndex), SubresourceRange);
            CurBuf->pipelineBarrier2(vk::DependencyInfo().setImageMemoryBarriers(sui));
            vk::RenderingAttachmentInfo uiai = vk::RenderingAttachmentInfo()
                .setImageView(_VulkanContext->GetSwapchainImageView(ImageIndex))
                .setImageLayout(vk::ImageLayout::eColorAttachmentOptimal)
                .setLoadOp(vk::AttachmentLoadOp::eLoad)
                .setStoreOp(vk::AttachmentStoreOp::eStore);
            CurBuf->beginRendering(vk::RenderingInfo().setRenderArea(vk::Rect2D({0,0},_WindowSize)).setLayerCount(1).setColorAttachments(uiai));
            _uiRenderer->Render(*CurBuf);
            CurBuf->endRendering();
        }
        vk::ImageMemoryBarrier2 _b_present(vk::PipelineStageFlagBits2::eColorAttachmentOutput, vk::AccessFlagBits2::eColorAttachmentWrite, vk::PipelineStageFlagBits2::eBottomOfPipe, vk::AccessFlagBits2::eNone, vk::ImageLayout::eColorAttachmentOptimal, vk::ImageLayout::ePresentSrcKHR, vk::QueueFamilyIgnored, vk::QueueFamilyIgnored, _VulkanContext->GetSwapchainImage(ImageIndex), SubresourceRange);
        CurBuf->pipelineBarrier2(vk::DependencyInfo().setImageMemoryBarriers(_b_present));

        CurBuf.End();
        _VulkanContext->SubmitCommandBufferToGraphics(*CurBuf, *Semaphores_ImageAvailable[CurrentFrame], *Semaphores_RenderFinished[CurrentFrame], *InFlightFences[CurrentFrame]);
        _VulkanContext->PresentImage(*Semaphores_RenderFinished[CurrentFrame]);
        CurrentFrame = (CurrentFrame + 1) % Config::Graphics::kMaxFrameInFlight;
        ProcessInput();
        update();
    }
    if (g_DiskStateChanged && g_GlobalSampler)
    {
        g_DiskStateChanged = false;
        auto* AM = Art::FAssetManager::GetInstance();
        auto* PS = AM->GetAsset<Art::FShader>("BlackHolePrepass");
        auto* CS = AM->GetAsset<Art::FShader>("BlackHoleComposite");
        Art::FTexture2D* Tex = nullptr;
        if (g_CurrentDiskState == -1) Tex = AM->GetAsset<Art::FTexture2D>("NPGSTexture");
        else Tex = AM->GetAsset<Art::FTexture2D>(g_DiskTextures[g_CurrentDiskState]);
        if (Tex && PS && CS)
        {
            std::vector<vk::DescriptorImageInfo> uii;
            uii.push_back(Tex->CreateDescriptorImageInfo(*g_GlobalSampler));
            PS->WriteSharedDescriptors(1, 9, vk::DescriptorType::eCombinedImageSampler, uii);
            CS->WriteSharedDescriptors(1, 9, vk::DescriptorType::eCombinedImageSampler, uii);
        }
    }
    _VulkanContext->WaitIdle();
    Terminate();
}  // ExecuteMainRender
void FApplication::Terminate()
{
    if (_uiRenderer)
    {
        _uiRenderer->Shutdown();
        _uiRenderer.reset();
    }
    _VulkanContext->WaitIdle();
    glfwDestroyWindow(_Window);
    glfwTerminate();
}

bool FApplication::InitializeWindow()
{
    if (glfwInit() == GLFW_FALSE) { NpgsCoreError("Failed to initialize GLFW."); return false; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, true);
    GLFWmonitor* PrimaryMonitor = nullptr;
    if (_bEnableFullscreen)
    {
        PrimaryMonitor = glfwGetPrimaryMonitor();
        const GLFWvidmode* mode = glfwGetVideoMode(PrimaryMonitor);
        _WindowSize.width = mode->width; _WindowSize.height = mode->height;
    }
    _Window = glfwCreateWindow(_WindowSize.width, _WindowSize.height, _WindowTitle.c_str(), PrimaryMonitor, nullptr);
    if (!_Window) { NpgsCoreError("Failed to create GLFW window."); glfwTerminate(); return false; }
    InitializeInputCallbacks();

    // Use framebuffer size (physical pixels) for rendering
    int fbW = 0, fbH = 0;
    glfwGetFramebufferSize(_Window, &fbW, &fbH);
    if (fbW > 0 && fbH > 0) _WindowSize = vk::Extent2D{(uint32_t)fbW, (uint32_t)fbH};

    if (!glfwVulkanSupported()) { NpgsCoreError("GLFW: Vulkan not supported."); return false; }
    uint32_t ExtCount = 0;
    const char** Extensions = glfwGetRequiredInstanceExtensions(&ExtCount);
    if (!Extensions || ExtCount == 0) { NpgsCoreError("Failed to get required instance extensions."); return false; }
    for (uint32_t i = 0; i < ExtCount; ++i) _VulkanContext->AddInstanceExtension(Extensions[i]);
    _VulkanContext->AddDeviceExtension(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
    if (_VulkanContext->CreateInstance() != vk::Result::eSuccess) { glfwDestroyWindow(_Window); glfwTerminate(); return false; }

    vk::SurfaceKHR Surface;
    if (glfwCreateWindowSurface(_VulkanContext->GetInstance(), _Window, nullptr, (VkSurfaceKHR*)&Surface) != VK_SUCCESS)
    { NpgsCoreError("Failed to create window surface."); glfwDestroyWindow(_Window); glfwTerminate(); return false; }
    _VulkanContext->SetSurface(Surface);
    if (_VulkanContext->CreateDevice(0) != vk::Result::eSuccess || _VulkanContext->CreateSwapchain(_WindowSize, _bEnableVSync) != vk::Result::eSuccess)
        return false;
    _FreeCamera = std::make_unique<SysSpa::FCamera>(glm::vec3(0.0f, 0.0f, 0.0f), 0.2, 2.5, cfov);
    _FreeCamera->SetCameraMode(true);
    return true;
}

void FApplication::InitializeInputCallbacks()
{
    glfwSetWindowUserPointer(_Window, this);
    glfwSetFramebufferSizeCallback(_Window, &FApplication::FramebufferSizeCallback);
    glfwSetScrollCallback(_Window, &FApplication::ScrollCallback);
    glfwSetMouseButtonCallback(_Window, &FApplication::MouseButtonCallback);
    glfwSetCursorPosCallback(_Window, &FApplication::CursorPosCallback);
    glfwSetKeyCallback(_Window, &FApplication::KeyCallback);
    glfwSetCharCallback(_Window, &FApplication::CharCallback);
}

void FApplication::update()
{
    _FreeCamera->SetRotationSmoothCoefficient(camsmth);
    _FreeCamera->ProcessTimeEvolution(_DeltaTime);
    CurrentTime = glfwGetTime();
    _LastDeltaTime = _DeltaTime;
    _DeltaTime = CurrentTime - LastFrameTime;
    RealityTime += _DeltaTime;
    GameTime += TimeRate * _DeltaTime;
    LastFrameTime = CurrentTime;
    ++FramePerSec;
    FrameCount++;
    if (CurrentTime - PreviousTime >= 1.0)
    {
        _DisplayFPS = FramePerSec;
        glfwSetWindowTitle(_Window, (_WindowTitle + " " + std::to_string(_DisplayFPS)).c_str());
        FramePerSec = 0;
        PreviousTime = CurrentTime;
    }
}

void FApplication::RenderDebugUI()
{
    double M_PI__ = 3.14159265358979323846;
    ImGui::Begin("Black Hole Topology Map", nullptr, ImGuiWindowFlags_NoScrollbar);

    // 1. 获取基础物理参数与坐标 (按 Rs 归一化)
    float Rs = 2.0f * std::abs(BlackHoleArgs.BlackHoleMassSol) * kGravityConstant / std::pow(kSpeedOfLight, 2) * kSolarMass / kLightYearToMeter;
    if (Rs < 1e-6f) Rs = 1.0f;

    float M = 0.5f;
    float a = BlackHoleArgs.Spin * M;
    float Q = BlackHoleArgs.Q * M;
    float a2 = a * a;
    float Q2 = Q * Q;

    glm::vec3 camPos;
    glm::vec3 camDir;
    glm::vec3 camVel;
    float physical_speed = 0.0f;

    if (g_GeodesicMode)
    {
        camPos = glm::vec3(g_GeoState[0], g_GeoState[1], g_GeoState[2]);
        camDir = glm::vec3(BlackHoleArgs.ie3_up.x, BlackHoleArgs.ie3_up.y, BlackHoleArgs.ie3_up.z);
        if (glm::length(camDir) > 1e-6f) camDir = glm::normalize(camDir);
        camVel = glm::vec3(g_GeoState[4], g_GeoState[5], g_GeoState[6]);

        double g_down[4][4] = { 0.0 }, g_up[4][4] = {0.0}, dummy_r;
        double aa = BlackHoleArgs.Spin * 0.5, qq = BlackHoleArgs.Q * 0.5;
        GeodesicIntegrator::ComputeMetric(g_GeoState, aa, qq, 1.0, BlackHoleArgs.UniverseSign, g_isOutgoing, g_down, g_up, dummy_r);
        double U_t = g_down[3][0] * g_GeoState[4] + g_down[3][1] * g_GeoState[5] + g_down[3][2] * g_GeoState[6] + g_down[3][3] * g_GeoState[7];
        double g_tt = g_down[3][3];
        double v_sq = 1.0 + (g_tt / std::fmax(1e-12, U_t * U_t));
        physical_speed = static_cast<float>(std::sqrt(std::fmax(0.0, v_sq)));
    }
    else
    {
        camPos = _FreeCamera->GetCameraVector(SysSpa::FCamera::EVectorType::kPosition) / Rs;
        camDir = _FreeCamera->GetCameraVector(SysSpa::FCamera::EVectorType::kFront);
        camVel = glm::vec3(BlackHoleArgs.CameraVelocity);
        physical_speed = glm::length(camVel);
    }

    float r_ui_b = (camPos.x * camPos.x + camPos.y * camPos.y + camPos.z * camPos.z) - a2;
    float r_ui_c = a2 * camPos.y * camPos.y;
    float r_ui2 = 0.5f * (r_ui_b + std::sqrt(r_ui_b * r_ui_b + 4.0f * r_ui_c));
    float r_ui = std::sqrt(std::fmax(0.0f, r_ui2)) * BlackHoleArgs.UniverseSign;

    double t_BL = BlackHoleArgs.BlackHoleTime;
    if (g_GeodesicMode)
    {
        double Delta = r_ui * r_ui - 2.0 * M * r_ui + a2 + Q2;
        double abs_Delta_safe = std::fmax(std::abs(Delta), 1e-16);
        double delta_disc = M * M - a2 - Q2;
        double F_r = 0.0;
        if (delta_disc > 1e-16)
        {
            double K = std::sqrt(delta_disc);
            double frac = std::abs(r_ui - (M + K)) / std::fmax(std::abs(r_ui - (M - K)), 1e-16);
            F_r = 2.0 * M * std::log(abs_Delta_safe) + ((2.0 * M * M - Q2) / K) * std::log(std::fmax(frac, 1e-16));
        }
        else if (delta_disc < -1e-16)
        {
            double K = std::sqrt(-delta_disc);
            double atan_arg = std::atan((r_ui - M) / K);
            F_r = 2.0 * M * std::log(abs_Delta_safe) + (2.0 * (2.0 * M * M - Q2) / K) * atan_arg;
        }
        else
        {
            double rM = r_ui - M;
            double safe_rM = (rM >= 0 ? 1.0 : -1.0) * std::fmax(std::abs(rM), 1e-16);
            F_r = 4.0 * M * std::log(std::fmax(std::abs(rM), 1e-16)) - 2.0 * (2.0 * M * M - Q2) / safe_rM;
        }
        double dir = g_isOutgoing ? 1.0 : -1.0;
        t_BL = g_GeoState[3] + dir * 0.5 * F_r;
    }

    ImGui::Text("--- Coordinates & Physics ---");
    if (g_GeodesicMode)
    {
        const char* chartName = g_isOutgoing ? "Outgoing Kerr-Schild (White Hole/Escape)" : "Ingoing Kerr-Schild (Black Hole/Fall)";
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "[Chart]: %s", chartName);
        ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.8f, 1.0f), "4-Position X^mu (t, x, y, z):");
        ImGui::Text("  %.5f,  %.4f, %.4f, %.4f", g_GeoState[3], g_GeoState[0], g_GeoState[1], g_GeoState[2]);
        ImGui::TextColored(ImVec4(0.8f, 0.5f, 1.0f, 1.0f), "4-Velocity U^mu (U^t, U^x, U^y, U^z):");
        ImGui::Text("  %.5f,  %.5f, %.5f, %.5f", g_GeoState[7], g_GeoState[4], g_GeoState[5], g_GeoState[6]);
        ImGui::Separator();
    }
    ImGui::Text("t (BL): %.5f Rs/c", t_BL);

    if (g_GeodesicMode)
    {
        double tau = GeodesicIntegrator::g_ProperTime;
        float time_dilation_diff = static_cast<float>(t_BL - tau);
        const int GRAPH_SAMPLES = 200;
        static float s_TimeDiffHistory[GRAPH_SAMPLES] = { 0 };
        static int s_HistoryOffset = 0;
        static double s_LastGraphUpdateTime = 0;
        static bool s_IsFirstFrame = true;
        if (s_IsFirstFrame)
        {
            for (int i = 0; i < GRAPH_SAMPLES; ++i) s_TimeDiffHistory[i] = time_dilation_diff;
            s_IsFirstFrame = false;
        }
        double currentTime = ImGui::GetTime();
        if (currentTime - s_LastGraphUpdateTime > (1.0 / 60.0))
        {
            s_TimeDiffHistory[s_HistoryOffset] = time_dilation_diff;
            s_HistoryOffset = (s_HistoryOffset + 1) % GRAPH_SAMPLES;
            s_LastGraphUpdateTime = currentTime;
        }
        float min_val = FLT_MAX, max_val = -FLT_MAX;
        for (int i = 0; i < GRAPH_SAMPLES; ++i) { if (s_TimeDiffHistory[i] < min_val) min_val = s_TimeDiffHistory[i]; if (s_TimeDiffHistory[i] > max_val) max_val = s_TimeDiffHistory[i]; }
        float padding = (max_val - min_val) * 0.1f; if (padding < 1e-4f) padding = 0.1f;
        char overlay_text[64];
        snprintf(overlay_text, sizeof(overlay_text), "t - tau: %.4f", time_dilation_diff);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.1f, 0.15f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_PlotLines, ImVec4(0.0f, 0.8f, 1.0f, 1.0f));
        ImGui::PlotLines("##TimeDilationGraph", s_TimeDiffHistory, GRAPH_SAMPLES, s_HistoryOffset, overlay_text, min_val - padding, max_val + padding, ImVec2(ImGui::GetContentRegionAvail().x, 60.0f));
        ImGui::PopStyleColor(2);
        ImGui::Text("Proper Time (tau): %.5f", tau);
    }

    ImGui::Text("r (BL): %.5f Rs", r_ui);
    ImGui::Text("x: %.4f | y: %.4f | z: %.4f (Rs)", camPos.x, camPos.y, camPos.z);
    float zenithAngleDeg = (glm::length(camPos) > 0.0f) ? glm::degrees(acosf(camPos.y / glm::length(camPos))) : 0.0f;
    ImGui::Text("Zenith Angle: %.4f", zenithAngleDeg);
    ImGui::Text("Spin (a*): %.4f | Charge (Q*): %.4f", BlackHoleArgs.Spin, BlackHoleArgs.Q);
    ImGui::Text("Velocity: %.6f c", physical_speed);
    ImGui::Text("FPS: %d", _DisplayFPS);
    ImGui::Text("Mode: %s", g_GeodesicMode ? "Geodesic" : "Orbit");

    if (g_GeodesicMode)
    {
        ImGui::Separator();
        ImGui::Text("--- Propulsion ---");
        double A_code = s_GeodesicThrust;
        ImGui::Text("Proper Accel: %.4f", A_code);
        if (A_code > 1e-6)
        {
            double dtau_code = 0.100335 / A_code;
            double Rs_meters = Rs * kLightYearToMeter;
            double tau_phys_sec = dtau_code * (Rs_meters / kSpeedOfLight);
            double timeTo01c_sec = tau_phys_sec / std::fmax(1e-9f, GameArgs.TimeRate);
            if (timeTo01c_sec < 60.0) ImGui::Text("Real time to 0.1c: %.2f sec", timeTo01c_sec);
            else if (timeTo01c_sec < 3600.0) ImGui::Text("Real time to 0.1c: %.2f min", timeTo01c_sec / 60.0);
            else if (timeTo01c_sec < 86400.0) ImGui::Text("Real time to 0.1c: %.2f hours", timeTo01c_sec / 3600.0);
            else if (timeTo01c_sec < 31536000.0) ImGui::Text("Real time to 0.1c: %.2f days", timeTo01c_sec / 86400.0);
            else ImGui::Text("Real time to 0.1c: %.2f years", timeTo01c_sec / 31536000.0);
        }
        else ImGui::Text("Real time to 0.1c: N/A (No thrust)");
    }
    ImGui::Separator();

    // 2. 记录轨迹
    const float MAX_TRAIL_LENGTH = 600.0f;
    bool shouldRecord = false;
    float stepDist = 0.0f;
    if (g_TrajectoryHistory.empty()) shouldRecord = true;
    else
    {
        const auto& lastPt = g_TrajectoryHistory.back();
        stepDist = glm::distance(camPos, lastPt.Pos);
        if (BlackHoleArgs.UniverseSign != lastPt.UniverseSign) { shouldRecord = true; stepDist = 0.0f; }
        else if (stepDist > 0.5f) shouldRecord = true;
        else
        {
            static float s_LastRecordTime = 0.0f;
            float ct = glfwGetTime();
            if (ct - s_LastRecordTime > 0.1f && stepDist > 0.01f) { shouldRecord = true; s_LastRecordTime = ct; }
        }
    }
    if (shouldRecord)
    {
        g_TotalOdometer += stepDist;
        double current_t_ks = g_GeodesicMode ? g_GeoState[3] : BlackHoleArgs.BlackHoleTime;
        bool current_isOut = g_GeodesicMode ? g_isOutgoing : false;
        g_TrajectoryHistory.push_back({ camPos, BlackHoleArgs.UniverseSign, g_TotalOdometer, current_t_ks, current_isOut, BlackHoleArgs.InWhichUniverse });
    }
    while (!g_TrajectoryHistory.empty() && (g_TotalOdometer - g_TrajectoryHistory.front().Odometer) > MAX_TRAIL_LENGTH) g_TrajectoryHistory.pop_front();

    // 3. 绘制姿态指示仪
    bool isAntiverse = (BlackHoleArgs.UniverseSign < 0.0f);
    float delta_discriminant = M * M - a2 - Q2;
    bool isNakedSingularity = (delta_discriminant < 0.0f);
    float r_outer = isNakedSingularity ? 0.0f : M + std::sqrt(delta_discriminant);
    float r_inner = isNakedSingularity ? 0.0f : M - std::sqrt(delta_discriminant);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
    ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
    if (canvas_sz.x < 100.0f) canvas_sz.x = 100.0f;
    if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;

    ImU32 bgColor = isAntiverse ? IM_COL32(40, 10, 15, 255) : IM_COL32(15, 20, 30, 255);
    draw_list->AddRectFilled(canvas_p0, ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y), bgColor);

    float thirdWidth = canvas_sz.x / 3.0f;
    float leftPadding = 30.0f;
    ImVec2 centerSide = ImVec2(canvas_p0.x + leftPadding, canvas_p0.y + canvas_sz.y * 0.5f);
    ImVec2 centerTop = ImVec2(canvas_p0.x + thirdWidth * 1.5f, canvas_p0.y + canvas_sz.y * 0.5f);

    float camDist = glm::length(camPos);
    float displayRadius = std::fmax(1.5f, camDist * 1.2f);
    float scale = std::min(thirdWidth, canvas_sz.y) / (2.0f * displayRadius);

    auto ToSideScreen = [&](float x, float y) -> ImVec2 { return ImVec2(centerSide.x + x * scale, centerSide.y - y * scale); };
    auto ToTopScreen = [&](float x, float z) -> ImVec2 { return ImVec2(centerTop.x + x * scale, centerTop.y - z * scale); };

    ImU32 axisColor = IM_COL32(100, 100, 100, 150);
    draw_list->AddLine(ImVec2(centerSide.x, centerSide.y), ImVec2(canvas_p0.x + thirdWidth, centerSide.y), axisColor, 1.0f);
    draw_list->AddLine(ImVec2(centerSide.x, canvas_p0.y), ImVec2(centerSide.x, canvas_p0.y + canvas_sz.y), axisColor, 1.0f);
    draw_list->AddLine(ImVec2(canvas_p0.x + thirdWidth, centerTop.y), ImVec2(canvas_p0.x + thirdWidth * 2.0f, centerTop.y), axisColor, 1.0f);
    draw_list->AddLine(ImVec2(centerTop.x, canvas_p0.y), ImVec2(centerTop.x, canvas_p0.y + canvas_sz.y), axisColor, 1.0f);
    draw_list->AddLine(ImVec2(canvas_p0.x + thirdWidth, canvas_p0.y), ImVec2(canvas_p0.x + thirdWidth, canvas_p0.y + canvas_sz.y), IM_COL32(200, 200, 200, 255), 2.0f);
    draw_list->AddText(ImVec2(centerSide.x + 10, canvas_p0.y + 10), IM_COL32(255, 255, 255, 255), "Meridian (Y-X) Plane");
    draw_list->AddText(ImVec2(canvas_p0.x + thirdWidth + 10, canvas_p0.y + 10), IM_COL32(255, 255, 255, 255), "Top-Down (X-(-Z)) Plane");

    ImU32 alphaStandard = isAntiverse ? 25 : 255;
    ImU32 alphaErgo = isAntiverse ? 15 : 200;
    ImU32 colOuterHorizon = IM_COL32(255, 100, 100, alphaStandard);
    ImU32 colInnerHorizon = IM_COL32(100, 150, 255, alphaStandard);
    ImU32 colErgosphere = IM_COL32(150, 255, 150, alphaErgo);
    ImU32 colInnerErgo = IM_COL32(255, 200, 50, alphaErgo);
    ImU32 colSingularity = IM_COL32(255, 0, 255, 255);
    ImU32 colCTC = IM_COL32(200, 100, 255, 255);

    const int segments = 511;
    std::vector<ImVec2> sideOutPts, sideInPts;
    ImVec2 prev_ergo_out, prev_ergo_in, prev_ctc_out, prev_ctc_in;
    bool prev_ergo_valid = false, prev_ctc_valid = false;

    auto GetCTCRoots = [&](float cosT, float sinT, float& r_out, float& r_in) -> bool
    {
        auto G = [&](float r) { float r2 = r * r; return (r2 + a2) * (r2 + a2 * cosT * cosT) + a2 * sinT * sinT * (2.0f * M * r - Q2); };
        float r_start = 0.0f, r_end = -10.0f;
        int steps = 200; float dr = (r_end - r_start) / steps;
        std::vector<float> roots;
        float prev_G = G(r_start);
        if (std::abs(prev_G) < 1e-5f) { roots.push_back(0.0f); prev_G = G(r_start + dr * 0.1f); }
        for (int k = 1; k <= steps; ++k)
        {
            float r = r_start + k * dr;
            float curr_G = G(r);
            if (curr_G * prev_G < 0.0f) roots.push_back(r - dr * curr_G / (curr_G - prev_G));
            prev_G = curr_G;
        }
        if (roots.size() >= 2) { r_out = roots[0]; r_in = roots.back(); return true; }
        else if (roots.size() == 1) { r_out = 0.0f; r_in = roots[0]; return true; }
        return false;
    };

    for (int i = 0; i <= segments; ++i)
    {
        float theta = (static_cast<float>(i) / segments) * 3.14159265f;
        float cosTheta = std::cos(theta);
        float sinTheta = std::sin(theta);

        float ergo_discriminant = M * M - a2 * cosTheta * cosTheta - Q2;
        bool ergo_valid = (ergo_discriminant >= 0.0f);
        if (ergo_valid)
        {
            float sqrt_disc = std::sqrt(ergo_discriminant);
            float r_ergo_out = M + sqrt_disc, r_ergo_in = M - sqrt_disc;
            ImVec2 pt_ergo_out = ToSideScreen(std::sqrt(r_ergo_out * r_ergo_out + a2) * sinTheta, r_ergo_out * cosTheta);
            ImVec2 pt_ergo_in = ToSideScreen(std::sqrt(r_ergo_in * r_ergo_in + a2) * sinTheta, r_ergo_in * cosTheta);
            if (prev_ergo_valid)
            {
                draw_list->AddLine(prev_ergo_out, pt_ergo_out, colErgosphere, 1.5f);
                draw_list->AddLine(prev_ergo_in, pt_ergo_in, colInnerErgo, 1.5f);
            }
            else if (i > 0) draw_list->AddLine(pt_ergo_out, pt_ergo_in, colErgosphere, 1.5f);
            prev_ergo_out = pt_ergo_out; prev_ergo_in = pt_ergo_in;
        }
        else if (prev_ergo_valid) draw_list->AddLine(prev_ergo_out, prev_ergo_in, colErgosphere, 1.5f);
        prev_ergo_valid = ergo_valid;

        float r_ctc_out, r_ctc_in;
        bool ctc_valid = GetCTCRoots(cosTheta, sinTheta, r_ctc_out, r_ctc_in);
        if (ctc_valid)
        {
            ImVec2 pt_ctc_out = ToSideScreen(std::sqrt(r_ctc_out * r_ctc_out + a2) * sinTheta, r_ctc_out * cosTheta);
            ImVec2 pt_ctc_in = ToSideScreen(std::sqrt(r_ctc_in * r_ctc_in + a2) * sinTheta, r_ctc_in * cosTheta);
            if (prev_ctc_valid)
            {
                draw_list->AddLine(prev_ctc_out, pt_ctc_out, colCTC, 1.5f);
                draw_list->AddLine(prev_ctc_in, pt_ctc_in, colCTC, 1.5f);
            }
            else if (i > 0) draw_list->AddLine(pt_ctc_out, pt_ctc_in, colCTC, 1.5f);
            prev_ctc_out = pt_ctc_out; prev_ctc_in = pt_ctc_in;
        }
        else if (prev_ctc_valid) draw_list->AddLine(prev_ctc_out, prev_ctc_in, colCTC, 1.5f);
        prev_ctc_valid = ctc_valid;

        if (i == 0)
        {
            float eq_disc = M * M - Q2;
            if (eq_disc >= 0.0f)
            {
                draw_list->AddCircle(centerTop, std::sqrt(std::pow(M + std::sqrt(eq_disc), 2) + a2) * scale, colErgosphere, 64, 1.5f);
                draw_list->AddCircle(centerTop, std::sqrt(std::pow(M - std::sqrt(eq_disc), 2) + a2) * scale, colInnerErgo, 64, 1.5f);
            }
            if (ctc_valid)
            {
                draw_list->AddCircle(centerTop, std::sqrt(r_ctc_out * r_ctc_out + a2) * scale, colCTC, 64, 1.5f);
                draw_list->AddCircle(centerTop, std::sqrt(r_ctc_in * r_ctc_in + a2) * scale, colCTC, 64, 1.5f);
            }
        }
        if (!isNakedSingularity)
        {
            sideOutPts.push_back(ToSideScreen(std::sqrt(r_outer * r_outer + a2) * sinTheta, r_outer * cosTheta));
            sideInPts.push_back(ToSideScreen(std::sqrt(r_inner * r_inner + a2) * sinTheta, r_inner * cosTheta));
            if (i == 0)
            {
                draw_list->AddCircle(centerTop, std::sqrt(r_outer * r_outer + a2) * scale, colOuterHorizon, 64, 2.0f);
                draw_list->AddCircle(centerTop, std::sqrt(r_inner * r_inner + a2) * scale, colInnerHorizon, 64, 2.0f);
            }
        }
    }
    if (!isNakedSingularity)
    {
        draw_list->AddPolyline(sideOutPts.data(), sideOutPts.size(), colOuterHorizon, ImDrawFlags_None, 2.0f);
        draw_list->AddPolyline(sideInPts.data(), sideInPts.size(), colInnerHorizon, ImDrawFlags_None, 2.0f);
    }
    draw_list->AddCircleFilled(ToSideScreen(std::abs(a), 0.0f), 4.0f, colSingularity);
    draw_list->AddCircle(centerTop, std::abs(a) * scale, colSingularity, 64, 2.0f);

    auto IsInCanvas = [&](ImVec2 p) { return p.x >= canvas_p0.x && p.x <= canvas_p0.x + canvas_sz.x && p.y >= canvas_p0.y && p.y <= canvas_p0.y + canvas_sz.y; };

    // 绘制轨迹历史
    for (size_t i = 1; i < g_TrajectoryHistory.size(); ++i)
    {
        const auto& p1 = g_TrajectoryHistory[i - 1];
        const auto& p2 = g_TrajectoryHistory[i];
        if (p1.UniverseSign != p2.UniverseSign) continue;
        float trailDepth = g_TotalOdometer - p2.Odometer;
        float alpha = 1.0f - (trailDepth / MAX_TRAIL_LENGTH);
        alpha = std::clamp(alpha, 0.0f, 1.0f);
        bool isCurrentUniv = (p1.UniverseSign == BlackHoleArgs.UniverseSign);
        ImU32 col = isCurrentUniv ? IM_COL32(0, 200, 255, (int)(alpha * 255)) : IM_COL32(255, 50, 50, (int)(alpha * 150));
        float rho1 = std::sqrt(p1.Pos.x * p1.Pos.x + p1.Pos.z * p1.Pos.z);
        float rho2 = std::sqrt(p2.Pos.x * p2.Pos.x + p2.Pos.z * p2.Pos.z);
        ImVec2 s1 = ToSideScreen(rho1, p1.Pos.y);
        ImVec2 s2 = ToSideScreen(rho2, p2.Pos.y);
        ImVec2 t1 = ToTopScreen(p1.Pos.x, -p1.Pos.z);
        ImVec2 t2 = ToTopScreen(p2.Pos.x, -p2.Pos.z);
        if (IsInCanvas(s1) || IsInCanvas(s2)) draw_list->AddLine(s1, s2, col, 1.5f);
        if (IsInCanvas(t1) || IsInCanvas(t2)) draw_list->AddLine(t1, t2, col, 1.5f);
    }

    // 绘制相机位置、朝向和速度
    ImU32 camColor = IM_COL32(255, 255, 0, 255);
    ImU32 velColor = IM_COL32(0, 255, 0, 255);
    float rho_cam = std::sqrt(camPos.x * camPos.x + camPos.z * camPos.z);
    ImVec2 camSidePos = ToSideScreen(rho_cam, camPos.y);
    ImVec2 camTopPos = ToTopScreen(camPos.x, -camPos.z);
    float drho = (rho_cam > 1e-6f) ? ((camPos.x * camDir.x + camPos.z * camDir.z) / rho_cam) : std::sqrt(camDir.x * camDir.x + camDir.z * camDir.z);
    float camLineLen = std::fmax(1.5f, camDist * 0.15f);
    ImVec2 camSideDir = ToSideScreen(rho_cam + drho * camLineLen, camPos.y + camDir.y * camLineLen);
    ImVec2 camTopDir = ToTopScreen(camPos.x + camDir.x * camLineLen, -camPos.z - camDir.z * camLineLen);
    ImVec2 velSideDir, velTopDir;
    bool hasVelocity = false;
    float vLen = glm::length(camVel);
    if (vLen > 1e-5f)
    {
        hasVelocity = true;
        glm::vec3 vDir = camVel / vLen;
        float drawLen = camLineLen * std::clamp(vLen * 1.5f, 0.5f, 3.0f);
        float vDrho = (rho_cam > 1e-6f) ? ((camPos.x * vDir.x + camPos.z * vDir.z) / rho_cam) : std::sqrt(vDir.x * vDir.x + vDir.z * vDir.z);
        velSideDir = ToSideScreen(rho_cam + vDrho * drawLen, camPos.y + vDir.y * drawLen);
        velTopDir = ToTopScreen(camPos.x + vDir.x * drawLen, -camPos.z - vDir.z * drawLen);
    }
    if (IsInCanvas(camSidePos)) { if (hasVelocity) draw_list->AddLine(camSidePos, velSideDir, velColor, 2.5f); draw_list->AddLine(camSidePos, camSideDir, camColor, 2.0f); draw_list->AddCircleFilled(camSidePos, 5.0f, camColor); }
    if (IsInCanvas(camTopPos)) { if (hasVelocity) draw_list->AddLine(camTopPos, velTopDir, velColor, 2.5f); draw_list->AddLine(camTopPos, camTopDir, camColor, 2.0f); draw_list->AddCircleFilled(camTopPos, 5.0f, camColor); }

    ImGui::Dummy(canvas_sz);
    ImGui::End();

    // BH Parameters panel (separate window)
    ImGui::Begin("BH Parameters", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::PushItemWidth(160);

    if (ImGui::CollapsingHeader("Core", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::DragFloat("Mass Msol", &BlackHoleArgs.BlackHoleMassSol, 1e5f, 1.0f, 1e10f, "%.2e");
        ImGui::DragFloat("Spin a*", &BlackHoleArgs.Spin, 0.01f, -1.0f, 1.0f, "%.3f");
        ImGui::DragFloat("Charge Q*", &BlackHoleArgs.Q, 0.01f, -1.0f, 1.0f, "%.3f");
        ImGui::DragFloat("Mu", &BlackHoleArgs.Mu, 0.01f, 0.0f, 10.0f);
        ImGui::DragFloat("Quality", &BlackHoleArgs.Quality, 0.1f, 0.1f, 10.0f);
        ImGui::SliderInt("Prepass", &BlackHoleArgs.Prepass, 0, 1);
        ImGui::SliderInt("Whitehole", &BlackHoleArgs.Whitehole, 0, 1);
        ImGui::SliderInt("Grid", &BlackHoleArgs.Grid, -1, 2);
        ImGui::SliderInt("Universe", &BlackHoleArgs.InWhichUniverse, 0, 2);
        ImGui::SliderInt("Observer", &BlackHoleArgs.ObserverMode, -1, 3);
        ImGui::SliderInt("Polarization", &BlackHoleArgs.Polarization, 0, 2);
        ImGui::Checkbox("Image Disk", (bool*)&BlackHoleArgs.UseImageDisk);
        ImGui::Checkbox("Heat Haze", (bool*)&BlackHoleArgs.EnableHeatHaze);
        ImGui::Checkbox("Shadow Cull", (bool*)&BlackHoleArgs.EnableShadowCulling);
    }

    if (ImGui::CollapsingHeader("Accretion Disk", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::DragFloat("Inner Rs", &BlackHoleArgs.InterRadiusRs, 0.1f, 0.5f, 50.0f);
        ImGui::DragFloat("Outer Rs", &BlackHoleArgs.OuterRadiusRs, 0.1f, 1.0f, 100.0f);
        ImGui::DragFloat("Thin Rs", &BlackHoleArgs.ThinRs, 0.01f, 0.0f, 5.0f);
        ImGui::DragFloat("Hopper", &BlackHoleArgs.Hopper, 0.01f, 0.0f, 5.0f);
        ImGui::DragFloat("Brightness", &BlackHoleArgs.Brightmut, 0.01f, 0.0f, 10.0f);
        ImGui::DragFloat("Darkness", &BlackHoleArgs.Darkmut, 0.01f, 0.0f, 1.0f);
        ImGui::DragFloat("Reddening", &BlackHoleArgs.Reddening, 0.01f, 0.0f, 1.0f);
        ImGui::DragFloat("Saturation", &BlackHoleArgs.Saturation, 0.01f, 0.0f, 1.0f);
        ImGui::DragFloat("Accretion", &BlackHoleArgs.AccretionRate, 0.001f, 0.0f, 1.0f, "%.4f");
        ImGui::DragFloat("Temp Exp", &BlackHoleArgs.BlackbodyIntensityExponent, 0.1f, 0.0f, 10.0f);
        ImGui::DragFloat("Shift Exp", &BlackHoleArgs.RedShiftColorExponent, 0.1f, 0.0f, 10.0f);
        ImGui::DragFloat("Shift Int", &BlackHoleArgs.RedShiftIntensityExponent, 0.1f, 0.0f, 10.0f);
        ImGui::DragFloat("Rot Speed", &BlackHoleArgs.ImageRotationSpeed, 0.001f, 0.0f, 1.0f);
    }

    if (ImGui::CollapsingHeader("Jet"))
    {
        ImGui::DragFloat("Bright", &BlackHoleArgs.JetBrightmut, 0.01f, 0.0f, 10.0f);
        ImGui::DragFloat("Saturation", &BlackHoleArgs.JetSaturation, 0.01f, 0.0f, 1.0f);
        ImGui::DragFloat("Shift Exp", &BlackHoleArgs.JetRedShiftIntensityExponent, 0.1f, 0.0f, 10.0f);
        ImGui::DragFloat("Shift Max", &BlackHoleArgs.JetShiftMax, 0.1f, 1.0f, 100.0f);
    }

    if (ImGui::CollapsingHeader("Background"))
    {
        ImGui::DragFloat("Bright Mut", &BlackHoleArgs.BackgroundBrightmut, 0.01f, 0.0f, 2.0f);
        ImGui::DragFloat("BackShift Max", &BlackHoleArgs.BackShiftMax, 0.1f, 1.0f, 10000.0f);
        ImGui::DragFloat("Ring Boost", &BlackHoleArgs.PhotonRingBoost, 0.01f, 0.0f, 10.0f);
        ImGui::DragFloat("Ring Temp", &BlackHoleArgs.PhotonRingColorTempBoost, 0.01f, 0.0f, 10.0f);
        ImGui::DragFloat("Boost Rot", &BlackHoleArgs.BoostRot, 0.01f, 0.0f, 10.0f);
    }

    if (ImGui::CollapsingHeader("Dense Star"))
    {
        ImGui::DragFloat("Surface R", &BlackHoleArgs.DensestarsurfaceR, 0.1f, 0.0f, 50.0f);
        ImGui::DragFloat("BB Exp", &BlackHoleArgs.DensestarBlackbodyIntensityExponent, 0.1f, 0.0f, 10.0f);
        ImGui::DragFloat("RS Color", &BlackHoleArgs.DensestarRedShiftColorExponent, 0.1f, 0.0f, 10.0f);
        ImGui::DragFloat("RS Int", &BlackHoleArgs.DensestarRedShiftIntensityExponent, 0.1f, 0.0f, 10.0f);
        ImGui::DragFloat("Bright", &BlackHoleArgs.DensestarBrightmut, 0.01f, 0.0f, 10.0f);
    }

    ImGui::SliderInt("DEBUG", &BlackHoleArgs.DEBUG, 0, 5);
    ImGui::PopItemWidth();
    ImGui::End();

    // Time Control panel (separate window)
    ImGui::Begin("Time Control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    float rate = (float)TimeRate;
    ImGui::Text("Time Rate: %.1fx", rate);
    ImGui::PushItemWidth(120);
    if (ImGui::SliderFloat("##rate", &rate, 0.0f, 10000.0f, "%.1fx")) TimeRate = rate;
    ImGui::PopItemWidth();
    if (ImGui::SmallButton("0")) TimeRate = 0.0; ImGui::SameLine();
    if (ImGui::SmallButton("1")) TimeRate = 1.0; ImGui::SameLine();
    if (ImGui::SmallButton("10")) TimeRate = 10.0; ImGui::SameLine();
    if (ImGui::SmallButton("100")) TimeRate = 100.0; ImGui::SameLine();
    if (ImGui::SmallButton("10^3")) TimeRate = 1000.0; ImGui::SameLine();
    if (ImGui::SmallButton("10^4")) TimeRate = 10000.0;
    double gameHours = GameTime / 3600.0;
    ImGui::Text("Elapsed: %.2f h", gameHours);
    ImGui::End();
}

void FApplication::ProcessInput()
{
    ImGuiIO& io = ImGui::GetIO();
    bool bMouseBlocked = io.WantCaptureMouse;
    bool bKeyboardBlocked = io.WantCaptureKeyboard;

    double currX, currY;
    glfwGetCursorPos(_Window, &currX, &currY);

    bool isMiddleDown = glfwGetMouseButton(_Window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    if (isMiddleDown && !bMouseBlocked)
        _FreeCamera->ResetSway();

    bool isLeftDown = glfwGetMouseButton(_Window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    static bool wasLeftDown = false;
    static int s_LeftFrameCooldown = 0;
    if (s_LeftFrameCooldown > 0) s_LeftFrameCooldown--;

    if (isLeftDown && !wasLeftDown)
    {
        if (!bMouseBlocked && s_LeftFrameCooldown == 0)
        {
            _bLeftMousePressedInWorld = true;
            _DragStartX = currX; _DragStartY = currY;
            _bIsDraggingInWorld = true;
            glfwSetInputMode(_Window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            _bFirstMouse = true;
        }
    }
    else if (!isLeftDown && wasLeftDown)
    {
        if (_bLeftMousePressedInWorld)
        {
            _bLeftMousePressedInWorld = false;
            _bIsDraggingInWorld = false;
            if (!_bIsDraggingRightInWorld && !g_bHideUIAndMouse)
                glfwSetInputMode(_Window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            glfwSetCursorPos(_Window, _DragStartX, _DragStartY);
            io.MousePos = ImVec2((float)_DragStartX, (float)_DragStartY);
            s_LeftFrameCooldown = 3;
        }
    }
    else if (_bLeftMousePressedInWorld && _bIsDraggingInWorld && s_LeftFrameCooldown == 0)
    {
        if (_bFirstMouse) { _LastX = currX; _LastY = currY; _bFirstMouse = false; }
        else
        {
            double dx = currX - _LastX, dy = currY - _LastY;
            _LastX = currX; _LastY = currY;
            if (dx != 0.0 || dy != 0.0) _FreeCamera->ProcessMouseMovement(dx, dy);
        }
    }
    wasLeftDown = isLeftDown;

    bool isRightDown = glfwGetMouseButton(_Window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    static bool wasRightDown = false;
    static int s_RightFrameCooldown = 0;
    if (s_RightFrameCooldown > 0) s_RightFrameCooldown--;

    if (isRightDown && !wasRightDown)
    {
        if (!bMouseBlocked && s_RightFrameCooldown == 0)
        {
            _bRightMousePressedInWorld = true;
            _DragRightStartX = currX; _DragRightStartY = currY;
            _bIsDraggingRightInWorld = true;
            glfwSetInputMode(_Window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            _bFirstMouseRight = true;
        }
    }
    else if (!isRightDown && wasRightDown)
    {
        if (_bRightMousePressedInWorld)
        {
            _bRightMousePressedInWorld = false;
            _bIsDraggingRightInWorld = false;
            if (!_bIsDraggingInWorld && !g_bHideUIAndMouse)
                glfwSetInputMode(_Window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            glfwSetCursorPos(_Window, _DragRightStartX, _DragRightStartY);
            io.MousePos = ImVec2((float)_DragRightStartX, (float)_DragRightStartY);
            s_RightFrameCooldown = 3;
        }
    }
    else if (_bRightMousePressedInWorld && _bIsDraggingRightInWorld && s_RightFrameCooldown == 0)
    {
        if (_bFirstMouseRight) { _LastRightX = currX; _LastRightY = currY; _bFirstMouseRight = false; }
        else
        {
            double dx = currX - _LastRightX, dy = currY - _LastRightY;
            _LastRightX = currX; _LastRightY = currY;
            if (dx != 0.0 || dy != 0.0) _FreeCamera->ProcessSwayMovement(dx, dy);
        }
    }
    wasRightDown = isRightDown;

    if (_buffered_scroll_y != 0.0f)
    {
        if (!bMouseBlocked)
        {
            if (g_GeodesicMode)
            {
                s_GeodesicThrust *= std::pow(1.2f, _buffered_scroll_y);
                std::cout << "[Geodesic] Thrust: " << s_GeodesicThrust << std::endl;
            }
            else _FreeCamera->ProcessMouseScroll(_buffered_scroll_y);
        }
        _buffered_scroll_y = 0.0f;
    }

    if (!bKeyboardBlocked)
    {
        static bool wasTDown = false;
        bool isTDown = glfwGetKey(_Window, GLFW_KEY_T) == GLFW_PRESS;
        if (isTDown && !wasTDown && !g_GeodesicMode) _FreeCamera->ProcessModeChange();
        wasTDown = isTDown;

        static bool wasGDown = false;
        bool isGDown = glfwGetKey(_Window, GLFW_KEY_G) == GLFW_PRESS;
        if (isGDown && !wasGDown)
        {
            g_GeodesicMode = !g_GeodesicMode;
            if (g_GeodesicMode)
            {
                _FreeCamera->SetCameraMode(false);
                float Rs = 2.0f * std::abs(BlackHoleArgs.BlackHoleMassSol) * kGravityConstant / std::pow(kSpeedOfLight, 2) * kSolarMass / kLightYearToMeter;
                glm::vec3 pos = _FreeCamera->GetCameraVector(SysSpa::FCamera::EVectorType::kPosition) / Rs;
                glm::vec3 velo = glm::vec3(BlackHoleArgs.CameraVelocity);
                g_isOutgoing = false;
                g_UniverseSign = BlackHoleArgs.UniverseSign;
                GeodesicIntegrator::g_ProperTime = 0.0;
                GeodesicIntegrator::InitializeGeodesicState(pos, velo, BlackHoleArgs.Spin * 0.5, BlackHoleArgs.Q * 0.5);
            }
        }
        wasGDown = isGDown;

        static bool wasODown = false;
        bool isODown = glfwGetKey(_Window, GLFW_KEY_O) == GLFW_PRESS;
        if (isODown && !wasODown)
        {
            if (!g_DiskTextures.empty())
            {
                if (g_CurrentDiskState == -1) { g_OriginalThinRs = BlackHoleArgs.ThinRs; g_OriginalHopper = BlackHoleArgs.Hopper; }
                g_CurrentDiskState++;
                if (g_CurrentDiskState >= (int)g_DiskTextures.size())
                {
                    g_CurrentDiskState = -1;
                    BlackHoleArgs.ThinRs = g_OriginalThinRs; BlackHoleArgs.Hopper = g_OriginalHopper;
                    BlackHoleArgs.UseImageDisk = 0; BlackHoleArgs.Prepass = 1;
                    g_DiskStateChanged = true;
                }
                else
                {
                    BlackHoleArgs.ThinRs = 0.0f; BlackHoleArgs.Hopper = 0.0f;
                    BlackHoleArgs.UseImageDisk = 1; BlackHoleArgs.Prepass = 0;
                    g_DiskStateChanged = true;
                }
            }
        }
        wasODown = isODown;

        static bool wasHDown = false;
        bool isHDown = glfwGetKey(_Window, GLFW_KEY_H) == GLFW_PRESS;
        if (isHDown && !wasHDown)
        {
            g_bHideUIAndMouse = !g_bHideUIAndMouse;
            if (g_bHideUIAndMouse) glfwSetInputMode(_Window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            else if (!_bIsDraggingInWorld && !_bIsDraggingRightInWorld) glfwSetInputMode(_Window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        }
        wasHDown = isHDown;

        static bool wasCtrlAltSDown = false;
        bool isCtrlDown = glfwGetKey(_Window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS || glfwGetKey(_Window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
        bool isAltDown = glfwGetKey(_Window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS || glfwGetKey(_Window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
        bool isSDownLocal = glfwGetKey(_Window, GLFW_KEY_S) == GLFW_PRESS;
        if (isCtrlDown && isAltDown && isSDownLocal && !wasCtrlAltSDown) g_bRequestScreenshot = true;
        wasCtrlAltSDown = isCtrlDown && isAltDown && isSDownLocal;

        if (glfwGetKey(_Window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(_Window, GLFW_TRUE);

        if (!isCtrlDown && !isAltDown)
        {
            if (g_GeodesicMode)
            {
                glm::vec3 accel_cam(0.0f);
                if (glfwGetKey(_Window, GLFW_KEY_W) == GLFW_PRESS) accel_cam.z -= 1.0f;
                if (glfwGetKey(_Window, GLFW_KEY_S) == GLFW_PRESS) accel_cam.z += 1.0f;
                if (glfwGetKey(_Window, GLFW_KEY_A) == GLFW_PRESS) accel_cam.x -= 1.0f;
                if (glfwGetKey(_Window, GLFW_KEY_D) == GLFW_PRESS) accel_cam.x += 1.0f;
                if (glfwGetKey(_Window, GLFW_KEY_R) == GLFW_PRESS) accel_cam.y += 1.0f;
                if (glfwGetKey(_Window, GLFW_KEY_F) == GLFW_PRESS) accel_cam.y -= 1.0f;
                if (glm::length(accel_cam) > 0.1f)
                {
                    accel_cam = glm::normalize(accel_cam);
                    glm::vec3 accel_ship = glm::conjugate(_FreeCamera->GetOrientation()) * accel_cam;
                    float thrust = s_GeodesicThrust;
                    GeodesicIntegrator::g_ProperAcceleration[0] = accel_ship.x * thrust;
                    GeodesicIntegrator::g_ProperAcceleration[1] = accel_ship.y * thrust;
                    GeodesicIntegrator::g_ProperAcceleration[2] = accel_ship.z * thrust;
                }
                else
                {
                    GeodesicIntegrator::g_ProperAcceleration[0] = 0.0;
                    GeodesicIntegrator::g_ProperAcceleration[1] = 0.0;
                    GeodesicIntegrator::g_ProperAcceleration[2] = 0.0;
                }
            }
            else
            {
                if (glfwGetKey(_Window, GLFW_KEY_W) == GLFW_PRESS) _FreeCamera->ProcessKeyboard(SysSpa::FCamera::EMovement::kForward);
                if (glfwGetKey(_Window, GLFW_KEY_S) == GLFW_PRESS) _FreeCamera->ProcessKeyboard(SysSpa::FCamera::EMovement::kBack);
                if (glfwGetKey(_Window, GLFW_KEY_A) == GLFW_PRESS) _FreeCamera->ProcessKeyboard(SysSpa::FCamera::EMovement::kLeft);
                if (glfwGetKey(_Window, GLFW_KEY_D) == GLFW_PRESS) _FreeCamera->ProcessKeyboard(SysSpa::FCamera::EMovement::kRight);
                if (glfwGetKey(_Window, GLFW_KEY_R) == GLFW_PRESS) _FreeCamera->ProcessKeyboard(SysSpa::FCamera::EMovement::kUp);
                if (glfwGetKey(_Window, GLFW_KEY_F) == GLFW_PRESS) _FreeCamera->ProcessKeyboard(SysSpa::FCamera::EMovement::kDown);
            }
            if (glfwGetKey(_Window, GLFW_KEY_Q) == GLFW_PRESS) _FreeCamera->ProcessKeyboard(SysSpa::FCamera::EMovement::kRollLeft);
            if (glfwGetKey(_Window, GLFW_KEY_E) == GLFW_PRESS) _FreeCamera->ProcessKeyboard(SysSpa::FCamera::EMovement::kRollRight);
        }
    }
}

void FApplication::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    auto* App = static_cast<FApplication*>(glfwGetWindowUserPointer(window));
    if (App) App->HandleMouseButton(button, action, mods);
}

void FApplication::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    auto* App = static_cast<FApplication*>(glfwGetWindowUserPointer(window));
    if (App) App->HandleKey(key, scancode, action, mods);
}

void FApplication::CharCallback(GLFWwindow* window, unsigned int codepoint)
{
    auto* App = static_cast<FApplication*>(glfwGetWindowUserPointer(window));
    if (App) App->HandleChar(codepoint);
}

void FApplication::FramebufferSizeCallback(GLFWwindow* Window, int Width, int Height)
{
    auto* App = reinterpret_cast<FApplication*>(glfwGetWindowUserPointer(Window));
    if (App) App->HandleFramebufferSize(Width, Height);
}

void FApplication::CursorPosCallback(GLFWwindow* window, double posX, double posY)
{
    auto* App = static_cast<FApplication*>(glfwGetWindowUserPointer(window));
    if (App) App->HandleCursorPos(posX, posY);
}

void FApplication::ScrollCallback(GLFWwindow* Window, double OffsetX, double OffsetY)
{
    auto* App = reinterpret_cast<FApplication*>(glfwGetWindowUserPointer(Window));
    if (App) App->HandleScroll(OffsetX, OffsetY);
}

void FApplication::HandleMouseButton(int button, int action, int mods)
{
    ImGui_ImplGlfw_MouseButtonCallback(_Window, button, action, mods);
}
void FApplication::HandleKey(int key, int scancode, int action, int mods)
{
    ImGui_ImplGlfw_KeyCallback(_Window, key, scancode, action, mods);
}
void FApplication::HandleChar(unsigned int codepoint)
{
    ImGui_ImplGlfw_CharCallback(_Window, codepoint);
}
void FApplication::HandleCursorPos(double posX, double posY)
{
    // Wayland: GLFW reports mouse in physical pixels, scale to logical for ImGui
    int fbW, winW;
    glfwGetFramebufferSize(_Window, &fbW, NULL);
    glfwGetWindowSize(_Window, &winW, NULL);
    if (fbW > 0 && winW > 0) {
        float scale = (float)fbW / (float)winW;
        ImGui_ImplGlfw_CursorPosCallback(_Window, posX / scale, posY / scale);
    }
}

void FApplication::HandleFramebufferSize(int Width, int Height)
{
    if (Width == 0 || Height == 0) return;
    if ((uint32_t)Width == _WindowSize.width && (uint32_t)Height == _WindowSize.height) return;
    _VulkanContext->WaitIdle();
    if (_VulkanContext->RecreateSwapchain() == vk::Result::eSuccess)
    {
        _WindowSize.width = _VulkanContext->GetSwapchainCreateInfo().imageExtent.width;
        _WindowSize.height = _VulkanContext->GetSwapchainCreateInfo().imageExtent.height;
    }
}

void FApplication::HandleScroll(double OffsetX, double OffsetY)
{
    _buffered_scroll_y += (float)OffsetY;
    ImGui_ImplGlfw_ScrollCallback(_Window, OffsetX, OffsetY);
}

_NPGS_END
