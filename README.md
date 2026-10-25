# Thalassa

A deterministic, headless-first, high-entity-count RTS/MOBA simulation and
networking engine, designed to be reused inside Unreal Engine 5 as a pure
simulation backend (UE5 owns rendering only).

### Configure

```powershell
cmake --preset windows-release
```

### Build

```powershell
cmake --build --preset windows-release --parallel
```

### Run Tests

```powershell
ctest --test-dir build/windows-release -C RelWithDebInfo --output-on-failure
```

The Release preset uses `RelWithDebInfo`, enabling compiler optimizations while retaining debugging information.

### Clean Build

- Windows:

```powershell
Remove-Item -Recurse -Force .\build\windows-release -ErrorAction SilentlyContinue

cmake --preset windows-release
cmake --build --preset windows-release --parallel
ctest --test-dir build/windows-release -C RelWithDebInfo --output-on-failure
```

- Linux:

```powershell
rm -rf ./build/linux-release

cmake --preset linux-release
cmake --build --preset linux-release --parallel

ctest --test-dir build/linux-release --output-on-failure
```

```powershell
rm -rf ./build/linux-debug

cmake --preset linux-debug
cmake --build --preset linux-debug --parallel

ctest --test-dir build/linux-debug --output-on-failure
```

### Tests
```powershell
hostname -I
./build/linux-release/apps/headless_server/thalassa_headless_server --players 2 --replay /tmp/match.thr
./thalassa_headless_server --bind 0.0.0.0 --port 27015 --players 2
```

