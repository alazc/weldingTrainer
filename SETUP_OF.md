# openFrameworks Setup — Welding Trainer

One-page checklist for installing openFrameworks and getting `of-app/` to build
and run.

**Target:** openFrameworks 0.12.1, Visual Studio 2022 Build Tools, installed at
`%USERPROFILE%\dev\openFrameworks` (the default the build expects — override
with the `OF_ROOT` environment variable if you install elsewhere).

---

## Step 1 — Install openFrameworks

```powershell
cd <repo>
.\setup_openframeworks.ps1
```

The script downloads `of_v0.12.1_vs_64_release.zip` (~500 MB), extracts to the
target path, and verifies `projectGenerator.exe` is present. **No admin
needed.** Roughly 1-3 minutes depending on network.

If the install path already exists, re-run with `-Force`. If you want a
different location, pass `-InstallRoot D:\of` etc. — then set the `OF_ROOT`
environment variable to the same path so `build-and-run.ps1` and
`of-app.vcxproj` can find it.

---

## Step 2 — Smoke-test the install

If you have full Visual Studio 2022 (Community/Pro/Enterprise) with `devenv.exe`,
open a bundled example and build it:

```powershell
& "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\devenv.exe" `
  "$env:USERPROFILE\dev\openFrameworks\examples\graphics\polygonExample\polygonExample.sln"
```

If you only have **VS 2022 Build Tools** (no IDE), there is no `devenv.exe`.
Use MSBuild headlessly instead:

```powershell
& "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe" `
  "$env:USERPROFILE\dev\openFrameworks\examples\graphics\polygonExample\polygonExample.sln" `
  /p:Configuration=Release /p:Platform=x64 /v:minimal
```

Note: BuildTools installs under `Program Files (x86)` even on 64-bit Windows
(legacy convention from VS 2019); the IDE editions install under `Program Files`.

Build target: **Release / x64**. (Debug builds work too but link slower.)
If the build succeeds, the install is sound.
If you hit `MSVCP140.dll` errors at run-time, install the MSVC 2015-2022 x64
redistributable from Microsoft. If the build fails on `FreeImage.lib` with
`unresolved external symbol __std_search_1`, that means your MSVC toolset is
older than 14.40 (i.e. older than VS 17.10) and the prebuilt FreeImage.lib in
oF 0.12.1 was compiled with a newer STL than your linker can satisfy — see
the troubleshooting table at the bottom for the fix.

---

## Step 3 — Build the welding-trainer app

The repo ships a **hand-authored** `of-app/of-app.sln` + `of-app/of-app.vcxproj`
— do **not** run `projectGenerator` against `of-app/` (it would clobber the
custom project settings). The project resolves the oF install through the
`OF_ROOT` MSBuild property / environment variable, defaulting to
`%USERPROFILE%\dev\openFrameworks`.

```powershell
cd <repo>
.\build-and-run.ps1 -NoRun     # build only
.\build-and-run.ps1            # build + launch with mouse input
```

Expected: `of-app\bin\of-app.exe` is produced and launches into the trainer
window. See `README.md` for run modes and key bindings.

---

## When it goes wrong

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `Invoke-WebRequest: SSL/TLS failure` | corporate proxy / outdated TLS | set `[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12` and retry |
| `Expand-Archive: cannot create file ...` | path > 260 chars | install closer to drive root, e.g. `C:\of`, and set `OF_ROOT` |
| `MSB8066: Custom build exited with code 1` | postBuild.cmd can't copy DLLs | install the MSVC 2015-2022 x64 redistributable |
| `LNK2001: unresolved external symbol __std_search_1` on FreeImage.lib | MSVC toolset < 14.40 (BuildTools 17.9.x). oF 0.12.1's prebuilt FreeImage was compiled with newer STL. | Upgrade VS Build Tools to 17.10+ via Visual Studio Installer or `winget upgrade Microsoft.VisualStudio.2022.BuildTools`. Alternative: downgrade oF to 0.12.0 (zip at `https://github.com/openframeworks/openFrameworks/releases/download/0.12.0/of_v0.12.0_vs_64_release.zip`) — its FreeImage was compiled with an older STL and links cleanly against 14.39. |
| `LNK2019: unresolved external — ofxGui*` | oF install incomplete (missing `addons\ofxGui`) | re-extract the oF release zip |
| `projectGenerator.exe` won't launch | missing VC++ runtime | install MSVC 2015-2022 x64 redistributable |

---

*Companion to `setup_openframeworks.ps1`.*
