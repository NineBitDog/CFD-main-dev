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
    points: wp.array2d(dtype=wp.vec3),
    colors: wp.array2d(dtype=wp.vec4),
    velocity_field: wp.array4d(dtype=wp.vec3),
    vehicle_surface: wp.array3d(dtype=wp.uint8),
    grid_res: int,
    dt: float,
    max_pre_hit_steps: int,
    post_hit_steps: int,
    emit_center: wp.vec3,
    emit_scale: float
):
    idx = wp.tid()

    state = wp.rand_init(1234, idx)
    rx = wp.randf(state) * 2.0 - 1.0
    ry = wp.randf(state) * 2.0 - 1.0

    start_pos = emit_center + wp.vec3(rx, ry, 0.0) * emit_scale * 0.5
    p = start_pos
    hit = False
    write_idx = 0

    # Keep the output packed from the first vehicle hit onward. This is
    # important: the renderer must never connect the hidden pre-hit portion
    # to the first visible point.
    for i in range(max_pre_hit_steps):
        ix = int(p[0])
        iy = int(p[1])
        iz = int(p[2])

        v = wp.vec3(0.0, 0.0, 0.0)

        if (ix >= 0 and ix < grid_res and
            iy >= 0 and iy < grid_res and
            iz >= 0 and iz < grid_res):

            v = velocity_field[ix, iy, iz]

            # The surface mask is already voxelized at lattice resolution.
            # Use a single lookup instead of the old 5x5x5 neighborhood test
            # (125 global-memory reads per integration step).
            if vehicle_surface[ix, iy, iz] != 0:
                hit = True

            if hit:
                points[idx, write_idx] = p
                speed = wp.length(v)
                if speed > 0.05:
                    colors[idx, write_idx] = wp.vec4(1.0, 0.3, 0.0, 1.0)
                else:
                    colors[idx, write_idx] = wp.vec4(0.0, 0.5, 1.0, 1.0)
                write_idx += 1

                # Once the vehicle has been hit, only retain a short
                # downstream segment. This prevents long useless traces.
                if write_idx >= post_hit_steps:
                    break

        p = p + v * dt

    # Clear unused output slots. They remain disconnected/transparent.
    for i in range(write_idx, post_hit_steps):
        points[idx, i] = p
        colors[idx, i] = wp.vec4(0.0, 0.0, 0.0, 0.0)


class FluidX3DSolver:
    def __init__(self, stl_path, mesh_max_dim, resolution=256):
        if not WARP_AVAILABLE:
            raise ImportError("Warp not available")

        self.resolution = resolution
        self.cells = resolution**3

        # Streamlines are deliberately sparse. Only lines that actually
        # contact the vehicle are sent to the renderer.
        self.num_lines = 600

        # Maximum upstream search distance and visible downstream length.
        # These are substantially lower than the old 200-step traces.
        self.pre_hit_steps = 140
        self.post_hit_steps = 48
        self.streamline_dt = 1.0

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

        # --- 4. CPU staging buffers ---
        self.vx = np.zeros(self.cells, dtype=np.float32)
        self.vy = np.zeros(self.cells, dtype=np.float32)
        self.vz = np.zeros(self.cells, dtype=np.float32)
        self.cpu_staging_grid = np.zeros(
            (self.resolution, self.resolution, self.resolution, 3),
            dtype=np.float32,
            order='F'
        )

        # --- 5. Build a surface voxel mask for streamline filtering ---
        self.vehicle_surface_cpu = np.zeros(
            (resolution, resolution, resolution), dtype=np.uint8, order='C'
        )
        self._build_vehicle_surface_mask(stl_path)

        # --- 6. Warp GPU Buffers ---
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

        # Output contains only the post-hit portion. Pre-hit positions are
        # never written as visible geometry.
        self.lines_pos = wp.zeros(
            (self.num_lines, self.post_hit_steps),
            dtype=wp.vec3,
            device=self.device
        )
        self.lines_col = wp.zeros(
            (self.num_lines, self.post_hit_steps),
            dtype=wp.vec4,
            device=self.device
        )

    def _build_vehicle_surface_mask(self, stl_path):
        """Voxelize the STL surface into the streamline grid once."""
        try:
            mesh = trimesh.load(stl_path, force='mesh')
            if mesh is None or len(mesh.vertices) == 0:
                raise RuntimeError("STL contains no vertices")

            bounds_min = mesh.bounds[0]
            bounds_max = mesh.bounds[1]
            mesh_center = (bounds_min + bounds_max) * 0.5

            # Match the solver's grid-space convention: center the STL and
            # scale its largest dimension to resolution*0.9.
            mesh.vertices = (
                (mesh.vertices - mesh_center) * self.sim_scale_factor
                + self.grid_center
            )

            # Surface-only voxelization is intentional. The mask is used only
            # to decide whether a streamline has reached the vehicle.
            vox = mesh.voxelized(pitch=1.0, method='subdivide')
            points = np.asarray(vox.points)

            if len(points) == 0:
                raise RuntimeError("STL voxelization produced no surface voxels")

            indices = np.rint(points).astype(np.int32)
            valid = np.all(
                (indices >= 0) & (indices < self.resolution), axis=1
            )
            indices = indices[valid]

            if len(indices) == 0:
                raise RuntimeError("Vehicle surface is outside the streamline grid")

            self.vehicle_surface_cpu[
                indices[:, 0], indices[:, 1], indices[:, 2]
            ] = 1

            print(
                f"🚗 Streamline vehicle mask: {len(indices):,} surface voxels "
                f"(single-cell contact test)"
            )

        except Exception as e:
            # Fail safely: an empty mask hides all streamlines rather than
            # accidentally displaying the entire field.
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

        # D. Trace sparse hit-only streamlines.
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
                self.streamline_dt,
                self.pre_hit_steps,
                self.post_hit_steps,
                emit_pos,
                self.resolution * 0.5
            ],
            device=self.device
        )
        wp.synchronize()

    def get_render_data(self):
        pts_grid = self.lines_pos.numpy()
        cols = self.lines_col.numpy().reshape(-1, 4)

        # Transform Grid Space -> World Space for VisPy
        pts_world = (pts_grid - self.grid_center) / self.sim_scale_factor

        return pts_world.reshape(-1, 3), cols, self.num_lines, self.post_hit_steps

    def cleanup(self):
        if self.lib:
            self.lib.fluid_cleanup()
