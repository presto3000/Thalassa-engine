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

```powershell
Remove-Item -Recurse -Force .\build\windows-release -ErrorAction SilentlyContinue

cmake --preset windows-release
cmake --build --preset windows-release --parallel
ctest --test-dir build/windows-release -C RelWithDebInfo --output-on-failure
```
