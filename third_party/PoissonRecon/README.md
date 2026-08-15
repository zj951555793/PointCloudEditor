# PoissonRecon 18.76 vendor directory

PointCloudEditor 2.1.6 is designed for a repository-local, reproducible PoissonRecon dependency.
The production location is:

```text
third_party/PoissonRecon/
  Src/Reconstructors.h
  Src/FEMTree.h
  Src/MultiThreading.h
  ...
```

## Populate this directory

Connected machine (proxy environment variables are honored by Python/urllib):

```bash
python tools/vendor_poissonrecon.py
```

Already downloaded the official `AdaptiveSolvers.zip`:

```bash
python tools/vendor_poissonrecon.py --archive D:/downloads/AdaptiveSolvers.zip
```

The tool validates the 18.76 version marker and required header set before replacing this directory.
After vendoring, delete the old CMake build directory and Configure/Generate again.

CMake searches this bundled directory before any external path. Network FetchContent is OFF by default in 2.1.6.
For CI/factory builds you can additionally set `PCEDITOR_REQUIRE_POISSONRECON=ON` so configuration fails immediately if the vendor tree is incomplete.

Upstream source: Michael Kazhdan, PoissonRecon / Adaptive Multigrid Solvers 18.76.
Preserve all upstream license/copyright notices when vendoring.
