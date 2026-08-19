import warp as wp
import numpy as np
import ctypes
import os
import trimesh

# --- Initialization & Fallback ---
WARP_AVAILABLE = False
WP_DEVICE = "cpu"

try:
    wp.init()
    if wp.is_cuda_available():
        WP_DEVICE = "cuda"
    WARP_AVAILABLE = True
except Exception as e:
    print(f"❌ Warp failed to initialize: {e}")

# --- WARP KERNEL (Run on GPU) ---
@wp.kernel
def trace_grid_streamlines(
    points: wp.array2d(dtype=wp.vec3),          # [NumLines, MaxSteps]
    colors: wp.array2d(dtype=wp.vec4),          # [NumLines, MaxSteps]
    velocity_field: wp.array4d(dtype=wp.vec3),  # [Nx, Ny, Nz, 3]
    vehicle_surface: wp.array3d(dtype=wp.uint8),# [Nx, Ny, Nz]
    grid_res: int,
    dt: float,
    max_steps: int,
    emit_center: wp.vec3,
    emit_scale: float,
    contact_radius: int
):
    idx = wp.tid()

    # 1. Initialize Particle (Reset to Emitter)
    state = wp.rand_init(1234, idx)

    # Random offset in a disk perpendicular to Z (assuming flow is roughly Z)
    rx = wp.randf(state) * 2.0 - 1.0
    ry = wp.randf(state) * 2.0 - 1.0

    start_pos = emit_center + wp.vec3(rx, ry, 0.0) * emit_scale * 0.5
    p = start_pos
    touched_vehicle = False

    # 2. Trace Loop
    for i in range(max_steps):
        points[idx, i] = p

        # Default color. Alpha is changed below after the complete line is traced.
        colors[idx, i] = wp.vec4(0.0, 0.8, 1.0, 0.0)

        ix = int(p[0])
        iy = int(p[1])
        iz = int(p[2])

        v = wp.vec3(0.0, 0.0, 0.0)

        if (ix >= 0 and ix < grid_res and
            iy >= 0 and iy < grid_res and
            iz >= 0 and iz < grid_res):

            v = velocity_field[ix, iy, iz]

            # Check whether this streamline has reached the vehicle surface.
            # A small radius avoids losing lines because of sub-voxel positioning.
            for dz in range(-contact_radius, contact_radius + 1):
                for dy in range(-contact_radius, contact_radius + 1):
                    for dx in range(-contact_radius, contact_radius + 1):
                        nx = ix + dx
                        ny = iy + dy
                        nz = iz + dz

                        if (nx >= 0 and nx < grid_res and
                            ny >= 0 and ny < grid_res and
                            nz >= 0 and nz < grid_res):
                            if vehicle_surface[nx, ny, nz] != 0:
                                touched_vehicle = True

            # Speed coloring
            speed = wp.length(v)
            if speed > 0.05:
                colors[idx, i] = wp.vec4(1.0, 0.3, 0.0, 1.0)
            else:
                colors[idx, i] = wp.vec4(0.0, 0.5, 1.0, 1.0)

        # Advect
        p = p + v * dt

    # 3. Only display a streamline if ANY point on it contacted the vehicle.
    # This keeps the complete streamline visible once it qualifies.
    for i in range(max_steps):
        c = colors[idx, i]
        if touched_vehicle:
            colors[idx, i] = wp.vec4(c[0], c[1], c[2], 1.0)
        else:
            colors[idx, i] = wp.vec4(c[0], c[1], c[2], 0.0)


class FluidX3DSolver:
    def __init__(self, stl_path, mesh_max_dim, resolution=256):
        if not WARP_AVAILABLE:
            raise ImportError("Warp not available")

        self.resolution = resolution
        self.cells = resolution**3

        # Streamline contact distance in lattice cells.
        # 2 cells is enough to catch lines that pass very close to the surface
        # without making the entire near-car flow region visible.
        self.streamline_contact_radius = 2

        # --- 1. Load C++ DLL ---
        dll_name = "fluid_wrapper.dll"
        dll_path = os.path.abspath(dll_name)
        if not os.path.exists(dll_path):
            raise FileNotFoundError(f"DLL not found at {dll_path}")

        self.lib = ctypes.CDLL(dll_path)

        self.lib.fluid_init.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_char_p]
        self.lib.fluid_step.argtypes = [ctypes.c_int]
        ptr_type = np.ctypeslib.ndpointer(dtype=np.float32, flags='C_CONTIGUOUS')
        self.lib.fluid_get_velocity.argtypes = [ptr_type, ptr_type, ptr_type]

        # --- 2. Initialize Simulation ---
        print(f"🌊 Initializing FluidX3D with {resolution}^3 grid...")
        self.lib.fluid_init(resolution, 0.01, 0.0005, stl_path.encode('utf-8'))

        # --- 3. Coordinate Systems ---
        self.sim_scale_factor = (resolution * 0.9) / mesh_max_dim
        self.grid_center = resolution / 2.0

        # --- 4. Build a surface voxel mask for streamline filtering ---
        # This is only done once. It uses the same centered/scaled coordinate
        # convention used by the streamline world/grid conversion.
        self.vehicle_surface_cpu = np.zeros(
            (resolution, resolution, resolution), dtype=np.uint8, order='C'
        )
        self._build_vehicle_surface_mask(stl_path, mesh_max_dim)

        # Warp GPU Buffers
        self.device = WP_DEVICE
        self.wp_field = wp.zeros(
            (resolution, resolution, resolution, 3),
            dtype=wp.vec3,
            device=self.device
        )
        self.wp_vehicle_surface = wp.array(
            self.vehicle_surface_cpu,
            dtype=wp.uint8,
            device=self.device
        )

        self.num_lines = 2000
        self.steps = 200
        self.lines_pos = wp.zeros(
            (self.num_lines, self.steps),
            dtype=wp.vec3,
            device=self.device
        )
        self.lines_col = wp.zeros(
            (self.num_lines, self.steps),
            dtype=wp.vec4,
            device=self.device
        )

    def _build_vehicle_surface_mask(self, stl_path, mesh_max_dim):
        """Voxelize the STL surface into the streamline grid once."""
        try:
            mesh = trimesh.load(stl_path, force='mesh')
            if mesh is None or len(mesh.vertices) == 0:
                raise RuntimeError("STL contains no vertices")

            # Match the solver's grid-space convention:
            # center the mesh, scale its largest dimension to resolution*0.9,
            # and place its center at the middle of the grid.
            bounds_min = mesh.bounds[0]
            bounds_max = mesh.bounds[1]
            mesh_center = (bounds_min + bounds_max) * 0.5

            mesh.vertices = (
                (mesh.vertices - mesh_center) * self.sim_scale_factor
                + self.grid_center
            )

            # Surface-only voxelization is intentional: we only need to know
            # whether a streamline comes near the vehicle surface.
            vox = mesh.voxelized(pitch=1.0, method='subdivide')
            points = np.asarray(vox.points)

            if len(points) == 0:
                raise RuntimeError("STL voxelization produced no surface voxels")

            indices = np.rint(points).astype(np.int32)
            valid = np.all(
                (indices >= 0) & (indices < self.resolution), axis=1
            )
            indices = indices[valid]

            self.vehicle_surface_cpu[
                indices[:, 0], indices[:, 1], indices[:, 2]
            ] = 1

            print(
                f"🚗 Streamline vehicle mask: {len(indices):,} surface voxels "
                f"(contact radius {self.streamline_contact_radius})"
            )

        except Exception as e:
            # Fail safely: an empty mask means the existing streamlines remain
            # hidden rather than accidentally displaying the entire field.
            print(f"⚠️ Could not build vehicle streamline mask: {e}")

    def update(self):
        # A. Run Physics (C++)
        self.lib.fluid_step(20)

        # B. Get Velocity (GPU -> CPU)
        self.lib.fluid_get_velocity(self.vx, self.vy, self.vz)

        # C. Upload to Warp (CPU -> GPU)
        self.cpu_staging_grid[..., 0] = self.vx.reshape(
            (self.resolution, self.resolution, self.resolution), order='F'
        )
        self.cpu_staging_grid[..., 1] = self.vy.reshape(
            (self.resolution, self.resolution, self.resolution), order='F'
        )
        self.cpu_staging_grid[..., 2] = self.vz.reshape(
            (self.resolution, self.resolution, self.resolution), order='F'
        )

        wp.copy(self.wp_field, self.cpu_staging_grid)

        # D. Trace Streamlines (Warp GPU)
        emit_pos = wp.vec3(self.resolution / 2, self.resolution / 2, 10.0)

        wp.launch(
            kernel=trace_grid_streamlines,
            dim=self.num_lines,
            inputs=[
                self.lines_pos,
                self.lines_col,
                self.wp_field,
                self.wp_vehicle_surface,
                self.resolution,
                1.0,
                self.steps,
                emit_pos,
                self.resolution * 0.5,
                self.streamline_contact_radius
            ],
            device=self.device
        )
        wp.synchronize()

    def get_render_data(self):
        pts_grid = self.lines_pos.numpy()
        cols = self.lines_col.numpy().reshape(-1, 4)

        # Transform Grid Space -> World Space for VisPy
        pts_world = (pts_grid - self.grid_center) / self.sim_scale_factor

        return pts_world.reshape(-1, 3), cols, self.num_lines, self.steps

    def cleanup(self):
        if self.lib:
            self.lib.fluid_cleanup()
