#include "Application.h"

using namespace Npgs;
using namespace Npgs::Util;

int main()
{
    FLogger::Initialize();

    FApplication App({ 1600, 960 }, "NPGS_linux - Kerr Black Hole Renderer", true, false);
    App.ExecuteMainRender();
    return 0;
}
