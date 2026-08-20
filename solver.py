import warp as wp
import numpy as np
import ctypes
import os
import trimesh

WARP_AVAILABLE = False
WP_DEVICE = "cpu"

try:
    wp.init()
    if wp.is_cuda_available():
        WP_DEVICE = "cuda"
    WARP_AVAILABLE = True
except Exception as e:
    print(f"Warp failed to initialize: {e}")

@wp.kernel
def trace_grid_streamlines(
    points: wp.array2d(dtype=wp.vec3),
    colors: wp.array2d(dtype=wp.vec4),
    velocity_field: wp.array4d(dtype=wp.vec3),
    vehicle_surface: wp.array3d(dtype=wp.uint8),
    grid_res: int,
    dt: float,
    pre_hit_steps: int,
    post_hit_steps: int,
    emit_center: wp.vec3,
    emit_scale: float
):
    idx = wp.tid()
    state = wp.rand_init(1234, idx)
    rx = wp.randf(state) * 2.0 - 1.0
    ry = wp.randf(state) * 2.0 - 1.0
    p = emit_center + wp.vec3(rx, ry, 0.0) * emit_scale * 0.5

    # Nothing is emitted until the streamline reaches a vehicle voxel.
    hit = False
    out_i = 0
    for i in range(pre_hit_steps):
        ix = int(p[0])
        iy = int(p[1])
        iz = int(p[2])
        if ix < 0 or ix >= grid_res or iy < 0 or iy >= grid_res or iz < 0 or iz >= grid_res:
            break

        v = velocity_field[ix, iy, iz]
        if vehicle_surface[ix, iy, iz] != 0:
            hit = True
            break
        p = p + v * dt

    if hit:
        for i in range(post_hit_steps):
            points[idx, out_i] = p
            ix = int(p[0])
            iy = int(p[1])
            iz = int(p[2])
            if ix < 0 or ix >= grid_res or iy < 0 or iy >= grid_res or iz < 0 or iz >= grid_res:
                break
            v = velocity_field[ix, iy, iz]
            speed = wp.length(v)
            if speed > 0.05:
                colors[idx, out_i] = wp.vec4(1.0, 0.3, 0.0, 1.0)
            else:
                colors[idx, out_i] = wp.vec4(0.0, 0.5, 1.0, 1.0)
            out_i += 1
            p = p + v * dt

    # Hide unused vertices without a second kernel or CPU compaction pass.
    for i in range(out_i, post_hit_steps):
        points[idx, i] = p
        colors[idx, i] = wp.vec4(0.0, 0.0, 0.0, 0.0)


class FluidX3DSolver:
    def __init__(self, stl_path, mesh_max_dim, resolution=256):
        if not WARP_AVAILABLE:
            raise ImportError("Warp not available")

        self.resolution = resolution
        self.cells = resolution ** 3
        self.device = WP_DEVICE

        dll_path = os.path.abspath("fluid_wrapper.dll")
        if not os.path.exists(dll_path):
            raise FileNotFoundError(f"DLL not found at {dll_path}")
        self.lib = ctypes.CDLL(dll_path)

        self.lib.fluid_init.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_char_p]
        self.lib.fluid_step.argtypes = [ctypes.c_int]
        ptr_type = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
        self.lib.fluid_get_velocity.argtypes = [ptr_type, ptr_type, ptr_type]

        print(f"Initializing FluidX3D with {resolution}^3 grid...")
        self.lib.fluid_init(resolution, 0.01, 0.0005, stl_path.encode("utf-8"))

        self.sim_scale_factor = (resolution * 0.9) / mesh_max_dim
        self.grid_center = resolution / 2.0

        # Keep the unavoidable DLL readback contiguous and reuse it forever.
        # One (N,3) allocation avoids three Python arrays and a second full
        # staging allocation. Component views are passed directly to ctypes.
        self.velocity_cpu = np.empty((self.cells, 3), dtype=np.float32, order="F")
        self.vx = self.velocity_cpu[:, 0]
        self.vy = self.velocity_cpu[:, 1]
        self.vz = self.velocity_cpu[:, 2]

        self.vehicle_surface_cpu = np.zeros((resolution, resolution, resolution), dtype=np.uint8)
        self._build_vehicle_surface_mask(stl_path)

        # This remains the only full-field upload possible without changing
        # fluid_wrapper.dll. It is persistent and receives the velocity data
        # directly from the reusable CPU buffer.
        self.wp_field = wp.empty((resolution, resolution, resolution, 3), dtype=wp.vec3, device=self.device)
        self.wp_vehicle_surface = wp.array(self.vehicle_surface_cpu, dtype=wp.uint8, device=self.device)

        # Aggressively reduced work: fewer seeds and only short post-hit output.
        self.num_lines = 600
        self.pre_hit_steps = 96
        self.post_hit_steps = 48
        self.steps = self.post_hit_steps
        self.lines_pos = wp.empty((self.num_lines, self.post_hit_steps), dtype=wp.vec3, device=self.device)
        self.lines_col = wp.empty((self.num_lines, self.post_hit_steps), dtype=wp.vec4, device=self.device)

        # Persistent host render buffers prevent repeated numpy allocation.
        self.render_pos = np.empty((self.num_lines * self.post_hit_steps, 3), dtype=np.float32)
        self.render_col = np.empty((self.num_lines * self.post_hit_steps, 4), dtype=np.float32)

    def _build_vehicle_surface_mask(self, stl_path):
        try:
            mesh = trimesh.load(stl_path, force="mesh")
            if mesh is None or len(mesh.vertices) == 0:
                raise RuntimeError("STL contains no vertices")
            bounds_min, bounds_max = mesh.bounds
            mesh_center = (bounds_min + bounds_max) * 0.5
            mesh.vertices = (mesh.vertices - mesh_center) * self.sim_scale_factor + self.grid_center
            vox = mesh.voxelized(pitch=1.0, method="subdivide")
            indices = np.rint(np.asarray(vox.points)).astype(np.int32)
            valid = np.all((indices >= 0) & (indices < self.resolution), axis=1)
            indices = indices[valid]
            if len(indices) == 0:
                raise RuntimeError("Vehicle surface is outside the streamline grid")
            self.vehicle_surface_cpu[indices[:, 0], indices[:, 1], indices[:, 2]] = 1
            print(f"Streamline vehicle mask: {len(indices):,} surface voxels")
        except Exception as e:
            print(f"Could not build vehicle streamline mask: {e}")

    def update(self):
        # Streamlines remain coupled to every physics update.
        self.lib.fluid_step(20)

        # Unavoidable with the current DLL, but no reshaping or second staging
        # grid is created. The three component views write directly into one
        # persistent contiguous velocity buffer.
        self.lib.fluid_get_velocity(self.vx, self.vy, self.vz)

        # Interpret the existing Fortran-ordered component buffer as the grid
        # layout expected by the old code, without filling another 256^3x3 array.
        velocity_grid = self.velocity_cpu.reshape(
            (self.resolution, self.resolution, self.resolution, 3), order="F"
        )
        wp.copy(self.wp_field, velocity_grid)

        emit_pos = wp.vec3(self.resolution * 0.5, self.resolution * 0.5, 10.0)
        wp.launch(
            kernel=trace_grid_streamlines,
            dim=self.num_lines,
            inputs=[
                self.lines_pos, self.lines_col, self.wp_field,
                self.wp_vehicle_surface, self.resolution, 1.0,
                self.pre_hit_steps, self.post_hit_steps, emit_pos,
                self.resolution * 0.5
            ],
            device=self.device
        )

    def get_render_data(self):
        # The render call is the synchronization point. Avoid synchronizing in
        # update(), which allows the physics/update caller to queue GPU work.
        pts_grid = self.lines_pos.numpy().reshape(-1, 3)
        cols = self.lines_col.numpy().reshape(-1, 4)
        np.subtract(pts_grid, self.grid_center, out=self.render_pos)
        self.render_pos /= self.sim_scale_factor
        np.copyto(self.render_col, cols)
        return self.render_pos, self.render_col, self.num_lines, self.post_hit_steps

    def cleanup(self):
        if self.lib:
            self.lib.fluid_cleanup()
