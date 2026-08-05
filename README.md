# ARAP Mesh Deformation

An interactive C++ implementation of standard As-Rigid-As-Possible deformation and its higher-order smooth extension.

I built this project as a way to learn more about geometry processing by implementing a complete deformation method instead of treating ARAP as a black box. I began with a simpler spokes-only ARAP implementation. After getting the local-global solver working, I extended the neighborhood definition to spokes-and-rims. Finally, I added the higher-order term proposed in Smooth ARAP to the same solver.

This is a learning-oriented implementation of existing methods rather than a proposal for a new deformation technique. The goal was to understand the mathematics well enough to translate it into a working interactive system, examine implementation details that are hidden when using an existing solver, and compare the standard and smooth formulations in the same application.

`libigl` is used for visualization, mesh loading, Voronoi mass matrix construction, polar SVD, and as a reference ARAP implementation. It is not used to assemble or solve the custom ARAP formulation.

<!-- TODO: Add a GIF comparing standard ARAP and Smooth ARAP deformation. -->

## Current status

The interactive deformation system is implemented and usable. It currently supports:

* custom spokes-and-rims ARAP;
* higher-order Smooth ARAP;
* a libigl spokes-and-rims baseline;
* point handles and fixed anchor vertices;
* interactive handle translation;
* adjustable local-global iteration count;
* adjustable smoothness parameter;
* ARAP, smooth, and weighted total energy evaluation;
* per-vertex energy visualization;
* runtime mesh loading;
* saving and loading handle/anchor configurations;
* OpenMP-parallel rotation fitting.

A reproducible, viewer-independent benchmark is the next planned part of the project. The benchmark section currently contains placeholders instead of timings collected from the interactive viewer.

## Motivation

I began with a spokes-only version of standard ARAP because I wanted to understand the complete local-global optimization process: how cotangent weights are constructed, how rotations are fitted for vertex neighborhoods, how the global system is assembled, and how positional constraints are introduced into a sparse solve.

Once the initial solver was working, I changed the neighborhood formulation to spokes-and-rims. This gave each vertex a larger local region containing both the edges connected directly to it and the opposite edges of its incident triangles.

After completing the standard spokes-and-rims formulation, I wanted to explore how the method could be extended. Point handles in standard ARAP can produce sharp changes or spike-like artifacts because constraints are applied to individual vertices. This led me to the Smooth ARAP formulation by Oehri, Herholz, and Sorkine-Hornung, which adds a higher-order term based on Laplacian vectors.

I added this term to the existing solver instead of implementing it as a separate application. This allowed me to reuse the same mesh representation, constraints, interface, and local-global pipeline while directly observing how the additional term changes the deformation.

Keeping standard ARAP, Smooth ARAP, and the libigl reference implementation in the same project also provides a useful basis for checking correctness and comparing their behavior under the same conditions.

## Method overview

The solver follows the standard local-global ARAP optimization strategy.

For each iteration:

1. Fit one rotation per vertex using the current deformed mesh.
2. Assemble the right-hand side from the rotated original geometry.
3. Apply positional constraints.
4. Solve a sparse linear system for the new vertex positions.
5. Repeat for the selected number of iterations.

The previous deformed state is reused as the initialization during interactive dragging.

### Spokes-and-rims neighborhoods

The current implementation uses spokes-and-rims neighborhoods.

For a vertex $v$, its local region contains every directed edge belonging to a triangle incident on $v$. This includes:

* **spokes**, which are directly connected to $v$
* **rims**, which are the opposite edges of incident triangles.

Each triangle contributes all three of its half-edges to the local region of each of its vertices.

The weight of a half-edge is computed from the cotangent of the opposite triangle angle:

$$
w_e=\cot(\alpha_e).
$$

The `HalfEdge` structure stores:

* the source and destination vertex;
* the cotangent weight;
* the original weighted edge vector.

This information is precomputed whenever a mesh is loaded.

### Cotangent Laplacian

The positive semidefinite cotangent Laplacian is assembled explicitly from triangle half-edges.

For each half-edge $e=(i,j)$, the corresponding triangle contributes

$$
w=\frac{1}{2}\cot(\alpha_e)
$$

to the following entries:

$$
L_{ii}\mathrel{+=}w,\qquad
L_{jj}\mathrel{+=}w,\qquad
L_{ij}\mathrel{-=}w,\qquad
L_{ji}\mathrel{-=}w.
$$

Interior edges receive contributions from both adjacent triangles, while boundary edges receive one contribution.

Cotangent values are calculated directly from the original vertex positions. Negative cotangent weights are not clamped.

### Local step

For each vertex $v$, a covariance matrix is constructed over its spokes-and-rims region:

$$
S_v=
\sum_{e\in\mathcal N_v}
w_e\mathbf e{\mathbf e'}^T
$$

where $\mathbf e$ is an original edge and $\mathbf e'$ is its current deformed version.

The closest rotation $R_v\in SO(3)$ is recovered using polar SVD. Rotation fitting is independent for every vertex, so this part of the local step is parallelized with OpenMP.

Only the mesh edges are used during rotation fitting. The Laplacian vectors introduced by the smooth term are not included in the covariance matrix.

### Standard ARAP global step

With rotations fixed, the ARAP energy becomes quadratic in the unknown vertex positions:

$$
E_{\mathrm{ARAP}}=
\sum_v
\sum_{e\in\mathcal N_v}
\frac{1}{3}
\frac{w_e}{2}
\left|
\mathbf e'-R_v\mathbf e
\right|^2.
$$

The factor $1/3$ compensates for the overlap of spokes-and-rims neighborhoods.

For every triangle, the implementation averages the rotations of its three vertices. The averaged rotation is then used with the triangle half-edges to assemble the global right-hand side $b$.

The standard ARAP global step solves:

$$
LV'=b,
$$

subject to the active handle and anchor constraints.

When the smoothness parameter is zero, this is the system used by the custom solver.

### Smooth ARAP term

Let $\widetilde M^{-1}$ denote the normalized inverse Voronoi mass matrix. The area-corrected Laplacian vectors of the original mesh are:

$$
\ell=\widetilde M^{-1}LV.
$$

For an individual vertex $v$:

$$
\ell_v=
\left(\widetilde M^{-1}LV\right)_v.
$$

The additional smooth energy measures the difference between the Laplacian vector of the deformed mesh and the rotated original Laplacian vector:

$$
E_{\mathrm{smooth}}=
\sum_v
\widetilde A_v
\left|
\ell'_v-R_v\ell_v
\right|^2,
$$

where $\widetilde A_v$ is the reciprocal of the corresponding normalized inverse mass.

The combined objective is:

$$
E=
(1-\lambda)E_{\mathrm{ARAP}}
+
\lambda E_{\mathrm{smooth}},
\qquad 0\leq\lambda<1.
$$

With rotations fixed, the global system becomes:

```math
\left[
(1-\lambda)L
+
\lambda L^T\widetilde{M}^{-1}L
\right]V'
=
(1-\lambda)b
+
\lambda L^T\mathcal{R}.
```

where row $v$ of $\mathcal R$ contains the original area-corrected Laplacian vector rotated by $R_v$.

The application represents the smoothness parameter as an integer between `0` and `99`, which is converted internally using:

```cpp
double lambda_adjusted=lambda/100.0;
```

The inverse Voronoi masses are divided by their mean before the system is assembled. This keeps the relative scale of the ARAP and smooth terms more manageable across meshes with different dimensions and tessellations.

### Edge-only rotation fitting

Rotations are fitted only to mesh edges, even when the smooth term is enabled.

It is also possible to include Laplacian vectors in the SVD fitting step. However, the Smooth ARAP paper reports that this can introduce unintuitive rotations and worse convergence behavior. This implementation therefore follows the proposed edge-only approach.

Because the local step does not optimize the complete smooth objective with respect to rotations, a strictly monotonic decrease of the total energy is not theoretically guaranteed when $\lambda>0$.

### Positional constraints

Anchors and the active handle are enforced as exact positional constraints.

The constraints are applied through substitution:

1. Vertices marked as anchors and the currently dragged handle are identified as fixed.
2. Their influence is subtracted from the right-hand side of the free vertices.
3. Matrix entries connecting fixed and free vertices are removed from the solve.
4. Constrained rows are replaced with identity rows.
5. The resulting sparse matrix is factorized using `Eigen::SimplicialLDLT`.

The factorization is computed when a drag session begins. It is also recomputed when the smoothness parameter changes because changing $\lambda$ changes the left-hand side matrix.

During handle movement, the factorization is reused while only the right-hand side and target handle position change.

## Energy evaluation

The application evaluates three values for the current deformation:

* raw ARAP energy;
* raw smooth energy;
* weighted total energy.

The displayed total energy is:

$$
E_{\mathrm{total}}=
(1-\lambda)E_{\mathrm{ARAP}}
+
\lambda E_{\mathrm{smooth}}.
$$

The raw smooth energy is still evaluated when $\lambda=0$. In that case, it acts only as a diagnostic measurement and contributes zero to the total objective.

Before energy evaluation, rotations are refitted using the latest vertex positions. This ensures that the displayed values correspond to the current deformation.

Per-vertex weighted energy is mapped to a blue-red color scale:

* blue indicates lower local energy;
* red indicates higher local energy.

The `Color factor` control changes only the visualization scale. It does not affect the solver or the energy itself.

Energy evaluation and color updates can be disabled through the interface when they are not needed.

## Interaction

The application initially loads:

```text
meshes/armadillo_1k.off
```

Another triangle mesh can be selected at runtime through the file dialog.

Supported formats are:

* `.off`
* `.obj`
* `.ply`

### Vertex modes

| Mode               | Behavior                            |
| ------------------ | ----------------------------------- |
| `None`             | Drag an existing handle             |
| `Handle Selection` | Mark free vertices as handles       |
| `Anchor Selection` | Mark free vertices as fixed anchors |
| `Handle Removal`   | Remove marked handles               |
| `Anchor Removal`   | Remove marked anchors               |

Vertex colors used during selection are:

* cyan — handle;
* red — anchor;
* green — selectable free vertex.

Multiple vertices can be marked as handles. However, only the handle currently being dragged is included as an active positional constraint. All anchors remain fixed at their original positions.

### Solver controls

| Control             | Description                                                 |
| ------------------- | ----------------------------------------------------------- |
| `Iterations`        | Number of local-global iterations performed for each update |
| `Smoothness factor` | Sets $\lambda$ from `0.00` to `0.99`                        |
| `Custom ARAP`       | Uses the custom standard or Smooth ARAP solver              |
| `libigl ARAP`       | Uses libigl spokes-and-rims ARAP as a baseline              |
| `Show energy`       | Enables energy calculation and local energy coloring        |
| `Color factor`      | Changes only the energy heatmap scale                       |

The smoothness parameter affects only the custom solver. Selecting `libigl ARAP` uses the standard spokes-and-rims implementation provided by libigl.

### Other controls

* `Open Mesh` loads another triangle mesh.
* `Reset Vertices` removes all handle and anchor selections.
* `Reset Mesh` restores the original vertex positions while keeping the current selections.
* `Save Vertices` stores the current handle and anchor indices in `config.txt`.
* `Load Config` restores the stored selection for the current mesh.

The configuration file stores vertex selections, not deformed mesh geometry. Configurations are identified using the mesh filename without its extension.

## Building

### Requirements

* C++17 compiler
* CMake 3.16 or newer
* OpenMP
* OpenGL-capable system
* internet connection during the first CMake configuration

The project uses CMake `FetchContent` to download:

* [libigl 2.5.0](https://github.com/libigl/libigl/tree/v2.5.0)
* [ImGuiFileDialog 0.6](https://github.com/aiekick/ImGuiFileDialog/tree/v0.6)

Eigen, GLFW, OpenGL, and ImGui are provided or configured through libigl.

Clone the repository:

```bash
git clone https://github.com/luka4723/ARAP.git
cd ARAP
```

### Windows with Ninja

The project has primarily been developed and tested on Windows 11 using MSVC and Ninja.

Run the following commands from Developer PowerShell or another terminal in which MSVC is available:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
.\build\ARAP.exe
```

### Windows with Visual Studio

```powershell
cmake -S . -B build
cmake --build build --config Release
.\build\Release\ARAP.exe
```

### Linux

The standard CMake build process is:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/ARAP
```

Platform-specific OpenGL, GLFW, and OpenMP development packages may be required.

After compilation, the `meshes` directory is copied next to the executable.

## Project structure

| File                   | Purpose                                                                           |
| ---------------------- | --------------------------------------------------------------------------------- |
| `main.cpp`             | Viewer and application initialization                                             |
| `mesh_context.h/.cpp`  | Mesh state, precomputation, Laplacian systems, constraints, and energy evaluation |
| `cell.h/.cpp`          | Half-edge data and per-vertex rotation fitting                                    |
| `solver.h/.cpp`        | Local-global iterations, right-hand side assembly, and mouse-to-plane projection  |
| `point_manager.h/.cpp` | Vertex picking and handle/anchor editing                                          |
| `menu.h/.cpp`          | ImGui controls and viewer callbacks                                               |
| `meshes/`              | Meshes used for interactive testing                                               |

The code is intentionally kept relatively small so that the complete deformation pipeline can be followed without navigating a large framework.

## Benchmarks

A separate benchmark executable is planned so that solver timings do not include rendering, mesh coloring, interface handling, file I/O, or vertex upload to the GPU.

The benchmark will use:

* Release builds;
* deterministic handle and anchor configurations;
* identical target and initial vertex positions;
* fixed local-global iteration counts;
* separate measurements for precomputation, factorization, and iterative solving;
* warm-up runs;
* multiple repetitions with the median reported;
* fixed OpenMP thread counts;
* raw results exported to CSV.

The current solver uses a fixed number of iterations instead of a convergence threshold. The benchmark will therefore report runtime for a specified number of iterations and will not make time-to-convergence claims.

### Test system

* CPU: `[TODO]`
* Operating system: `[TODO]`
* Compiler: `[TODO]`
* Build configuration: `Release`
* OpenMP threads: `[TODO]`
* libigl version: `2.5.0`

### Custom ARAP vs. libigl

The custom standard ARAP solver will be compared against libigl using:

* `lambda = 0`;
* spokes-and-rims energy in both implementations;
* identical constraints;
* identical target positions;
* identical initialization;
* identical iteration counts.

RMS vertex distance will be reported together with runtime to check whether the implementations produce comparable deformations.

| Mesh                 | Vertices |  Faces | Iterations | Custom factorization | Custom solve | libigl precomputation | libigl solve | RMS difference |
| -------------------- | -------: | -----: | ---------: | -------------------: | -----------: | --------------------: | -----------: | -------------: |
| `armadillo_1k.off`   |      864 |  1,724 |   `[TODO]` |             `[TODO]` |     `[TODO]` |              `[TODO]` |     `[TODO]` |       `[TODO]` |
| `cactus_highres.off` |    1,856 |  3,708 |   `[TODO]` |             `[TODO]` |     `[TODO]` |              `[TODO]` |     `[TODO]` |       `[TODO]` |
| `dino.off`           |   14,070 | 28,136 |   `[TODO]` |             `[TODO]` |     `[TODO]` |              `[TODO]` |     `[TODO]` |       `[TODO]` |

### Smoothness parameter

The custom solver will also be measured for several smoothness values using the same mesh, constraints, target position, and iteration count.

| Mesh     | $\lambda$ | Factorization | Fixed-iteration solve | ARAP energy | Smooth energy |
| -------- | --------: | ------------: | --------------------: | ----------: | ------------: |
| `[TODO]` |    `0.00` |      `[TODO]` |              `[TODO]` |    `[TODO]` |      `[TODO]` |
| `[TODO]` |    `0.50` |      `[TODO]` |              `[TODO]` |    `[TODO]` |      `[TODO]` |
| `[TODO]` |    `0.95` |      `[TODO]` |              `[TODO]` |    `[TODO]` |      `[TODO]` |
| `[TODO]` |    `0.99` |      `[TODO]` |              `[TODO]` |    `[TODO]` |      `[TODO]` |

Total energy values should not be compared directly across different values of $\lambda$, because changing $\lambda$ changes the objective function itself.

<!-- Replace the placeholder values with results produced by the benchmark executable. -->

## Known limitations

* Only triangle meshes are supported.
* Input meshes are expected to contain valid, non-degenerate geometry.
* Degenerate triangles and disconnected components are not explicitly validated.
* Handles and anchors are individual vertices rather than vertex groups.
* Only the currently dragged handle is active during a solve.
* Handle interaction currently supports translation only.
* The smoothness interface uses steps of `0.01` and has a maximum value of `0.99`.
* The solver uses a fixed iteration count rather than a convergence threshold.
* The matrix is refactorized when the active constraints or smoothness parameter change.
* The application does not currently export the deformed mesh.
* Very high smoothness values can produce unnecessary global rotations or lead to different local minima on meshes with thin or weakly connected parts.
* Benchmark results have not yet been added.

## References

* Olga Sorkine-Hornung and Marc Alexa. [As-Rigid-As-Possible Surface Modeling](https://igl.ethz.ch/projects/ARAP/index.php). Symposium on Geometry Processing, 2007.
* Annika Oehri, Philipp Herholz, and Olga Sorkine-Hornung. [Higher-Order Continuity for Smooth As-Rigid-As-Possible Shape Modeling](https://igl.ethz.ch/projects/smootharap/), 2025.
* [libigl](https://libigl.github.io/)

## Acknowledgements

This project uses libigl for mesh I/O, visualization, Voronoi mass matrix construction, polar SVD, and the reference ARAP implementation. ImGuiFileDialog is used for runtime mesh selection.

<!-- TODO: Add the source and license information for the meshes included in the repository. -->
