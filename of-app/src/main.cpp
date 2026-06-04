#include "ofMain.h"
#include "ofApp.h"

#include <string>

//========================================================================
// CLI: --source=mouse | serial | replay:<path.csv>   (default: serial)
//
// openFrameworks' main() does not forward argc/argv, so we read the raw
// command line via GetCommandLineW and scan for the --source token.
#ifdef TARGET_WIN32
#include <windows.h>
#include <shellapi.h>
#endif

namespace {

// Parse a single --<prefix>=<value> token from the raw Windows command line.
// Returns `default_val` if the token is absent.
std::string parseFlag(const std::string& prefix, const std::string& default_val) {
  std::string spec = default_val;
#ifdef TARGET_WIN32
  int argc = 0;
  LPWSTR* argv = ::CommandLineToArgvW(::GetCommandLineW(), &argc);
  if (argv) {
    for (int i = 1; i < argc; ++i) {
      // Narrow the wide arg (ASCII-only flags, so a plain cast is fine).
      std::string a;
      for (const wchar_t* p = argv[i]; *p; ++p) a.push_back(static_cast<char>(*p));
      if (a.rfind(prefix, 0) == 0) {
        spec = a.substr(prefix.size());
      }
    }
    ::LocalFree(argv);
  }
#endif
  return spec;
}

std::string parseSourceSpec() {
  // Default is "serial" so a bare double-click of the packaged exe comes up on
  // the hardware rig (enumerated device 0). This default only applies when no
  // --source= flag is present: build-and-run.ps1 always passes an explicit
  // --source, so the dev workflow is unaffected. For the no-hardware demo, pass
  // --source=mouse (see README.txt / package-app.ps1).
  return parseFlag("--source=", "serial");
}

std::string parseModeSpec() {
  return parseFlag("--mode=", "trainer");
}

}  // namespace

int main() {
  ofGLWindowSettings settings;
  settings.setSize(1024, 768);
  settings.windowMode = OF_WINDOW;

  auto window = ofCreateWindow(settings);

  auto app = std::make_shared<ofApp>();
  app->setSourceSpec(parseSourceSpec());
  app->setModeSpec(parseModeSpec());

  ofRunApp(window, app);
  ofRunMainLoop();
}
