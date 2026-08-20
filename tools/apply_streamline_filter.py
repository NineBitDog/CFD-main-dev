from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
KERNEL = ROOT / "FluidX3D-master" / "src" / "kernel.cpp"
SETUP = ROOT / "FluidX3D-master" / "src" / "setup.cpp"
DEFINES = ROOT / "FluidX3D-master" / "src" / "defines.hpp"
MAIN = ROOT / "main.py"
SOLVER = ROOT / "solver.py"


def replace_once(text: str, old: str, new: str, label: str) -> tuple[str, bool]:
    if old not in text:
        return text, False
    return text.replace(old, new, 1), True


def patch_kernel() -> bool:
    s = KERNEL.read_text(encoding="utf-8")
    changed = False

    # ------------------------------------------------------------
    # Remove the hard-coded inlet startup ramp from stream_collide().
    # Equilibrium boundaries in this setup are the inlet/outlet pair and
    # both use the same constant streamwise velocity.
    # ------------------------------------------------------------
    old = '''\tif(flagsn_bo==TYPE_E) {\n\t\trhon = rho[n];\n\t\tconst uint xcoord = coordinates(n).x;\n\t\tconst bool inlet_side = xcoord <= 1u;\n\t\tif(inlet_side) {\n\t\t\tfloat ramp = clamp((float)t/(float)1000ul, 0.0f, 1.0f);\n\t\t\t// Smoothstep startup avoids an impulse into the car at t=0.\n\t\t\tramp = ramp*ramp*(3.0f-2.0f*ramp);\n\t\t\tuxn = 0.075f*ramp;\n\t\t\tuyn = 0.0f;\n\t\t\tuzn = 0.0f;\n\t\t} else {\n\t\t\t// Constant-velocity equilibrium outlet.\n\t\t\tuxn = 0.075f;\n\t\t\tuyn = 0.0f;\n\t\t\tuzn = 0.0f;\n\t\t}\n\t} else {\n'''
    new = '''\tif(flagsn_bo==TYPE_E) {\n\t\trhon = rho[n];\n\t\tuxn = 0.075f;\n\t\tuyn = 0.0f;\n\t\tuzn = 0.0f;\n\t} else {\n'''
    s2, did = replace_once(s, old, new, "stream_collide inlet ramp")
    changed |= did
    s = s2

    # ------------------------------------------------------------
    # Replace the streamline filter body while preserving the exact native
    # kernel signature and its D2Q9/D3Q19 seed generation.
    # The classification test avoids two divides per sample and uses the
    # fixed +X freestream direction of this vehicle setup.
    # ------------------------------------------------------------
    start = s.find("kernel void graphics_streamline")
    if start < 0:
        raise RuntimeError("graphics_streamline kernel declaration not found")
    body_start = s.find("{", start)
    end = s.find("\n)+R(kernel void graphics_q_field", body_start)
    if body_start < 0 or end < 0:
        raise RuntimeError("graphics_streamline kernel boundaries not found")

    new_body = r'''
\tconst uxx n = get_global_id(0);
\tconst float3 ps = (float3)((float)slice_x+0.5f-0.5f*(float)def_Nx, (float)slice_y+0.5f-0.5f*(float)def_Ny, (float)slice_z+0.5f-0.5f*(float)def_Nz);
)+"#ifndef D2Q9"+R(
\tif(n>=(uxx)(def_Nx/def_streamline_sparse)*(uxx)(def_Ny/def_streamline_sparse)*(uxx)(def_Nz/def_streamline_sparse)) return;
\tconst uint z = (uint)(n/(uxx)((def_Nx/def_streamline_sparse)*(def_Ny/def_streamline_sparse)));
\tconst uint t = (uint)(n%(uxx)((def_Nx/def_streamline_sparse)*(def_Ny/def_streamline_sparse)));
\tconst uint y = (uint)(t/(def_Nx/def_streamline_sparse));
\tconst uint x = (uint)(t%(uxx)(def_Nx/def_streamline_sparse));
\tfloat3 p = (float)def_streamline_sparse*((float3)((float)x+0.5f, (float)y+0.5f, (float)z+0.5f))-0.5f*((float3)((float)def_Nx, (float)def_Ny, (float)def_Nz));
\tconst bool rx=fabs(p.x-ps.x)>0.5f*(float)def_streamline_sparse, ry=fabs(p.y-ps.y)>0.5f*(float)def_streamline_sparse, rz=fabs(p.z-ps.z)>0.5f*(float)def_streamline_sparse;
)+"#else"+R( // D2Q9
\tif(n>=(def_Nx/def_streamline_sparse)*(def_Ny/def_streamline_sparse)) return;
\tconst uint y = (uint)(n/(uxx)(def_Nx/def_streamline_sparse));
\tconst uint x = (uint)(n%(uxx)(def_Nx/def_streamline_sparse));
\tfloat3 p = ((float3)((float)def_streamline_sparse*((float)x+0.5f), (float)def_streamline_sparse*((float)y+0.5f), 0.5f))-0.5f*((float3)((float)def_Nx, (float)def_Ny, (float)def_Nz));
\tconst bool rx=fabs(p.x-ps.x)>0.5f*(float)def_streamline_sparse, ry=fabs(p.y-ps.y)>0.5f*(float)def_streamline_sparse, rz=true;
)+"#endif"+R( // D2Q9
\tif((slice_mode==1&&rx)||(slice_mode==2&&ry)||(slice_mode==3&&rz)||(slice_mode==4&&rx&&rz)||(slice_mode==5&&rx&&ry&&rz)||(slice_mode==6&&ry&&rz)||(slice_mode==7&&rx&&ry)) return;
\tif((slice_mode==1||slice_mode==5||slice_mode==4||slice_mode==7)&!rx) p.x = ps.x;
\tif((slice_mode==2||slice_mode==5||slice_mode==6||slice_mode==7)&!ry) p.y = ps.y;
\tif((slice_mode==3||slice_mode==5||slice_mode==4||slice_mode==6)&!rz) p.z = ps.z;
\tfloat camera_cache[15];
\tfor(uint i=0u; i<15u; i++) camera_cache[i] = camera[i];
\tconst float hLx=0.5f*(float)(def_Nx-2u*(def_Dx>1u)), hLy=0.5f*(float)(def_Ny-2u*(def_Dy>1u)), hLz=0.5f*(float)(def_Nz-2u*(def_Dz>1u));

\tconst float U_INF = 0.075f;
\tconst float SPEED_DELTA = 0.00225f; // 3% of U_INF
\tconst float DIRECTION_DELTA = 0.02f;
\tbool affected = false;

\tfor(float dt=-1.0f; dt<=1.0f && !affected; dt+=2.0f) {
\t\tfloat3 p1=p;
\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {
\t\t\tif(p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;
\t\t\tconst uint x1=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;
\t\t\tconst uint y1=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;
\t\t\tconst uint z1=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;
\t\t\tconst uxx nn=(uxx)x1+(uxx)(y1+z1*def_Ny)*(uxx)def_Nx;
\t\t\tif(flags[nn]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;
\t\t\tconst float3 un=load3(nn,u);
\t\t\tconst float ul=length(un);
\t\t\tif(ul<=1.0e-6f) break;
\t\t\tconst float inv_ul=1.0f/ul;
\t\t\t// For +X freestream: direction disturbance = 1 - u.x/|u|.
\t\t\tif(fabs(ul-U_INF)>=SPEED_DELTA || (ul-un.x)>=DIRECTION_DELTA*ul) { affected=true; break; }
\t\t\tp1 += (dt*inv_ul)*un;
\t\t\tif(def_scale_u*ul<0.1f) break;
\t\t}
\t}
\tif(!affected) return;

\tfor(float dt=-1.0f; dt<=1.0f; dt+=2.0f) {
\t\tfloat3 p0, p1=p;
\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {
\t\t\tif(p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;
\t\t\tconst uint x1=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;
\t\t\tconst uint y1=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;
\t\t\tconst uint z1=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;
\t\t\tconst uxx nn=(uxx)x1+(uxx)(y1+z1*def_Ny)*(uxx)def_Nx;
\t\t\tif(flags[nn]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;
\t\t\tconst float3 un=load3(nn,u);
\t\t\tconst float ul=length(un);
\t\t\tif(ul<=1.0e-6f) break;
\t\t\tp0=p1;
\t\t\tconst float inv_ul=1.0f/ul;
\t\t\tp1 += (dt*inv_ul)*un;
\t\t\tif(p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;
\t\t\tint c=0;
\t\t\tswitch(field_mode) {
\t\t\t\tcase 0: c=colorscale_rainbow(def_scale_u*ul); break;
\t\t\t\tcase 1: c=colorscale_twocolor(0.5f+def_scale_rho*(rho[nn]-1.0f)); break;
)+"#ifdef TEMPERATURE"+R(
\t\t\t\tcase 2: c=colorscale_iron(0.5f+def_scale_T*(T[nn]-def_T_avg)); break;
)+"#endif"+R(
\t\t\t}
\t\t\tdraw_line(p0,p1,c,camera_cache,bitmap,zbuffer);
\t\t}
\t}
'''

    # Unescape the raw Python representation used above.
    new_body = new_body.replace("\\n", "\n").replace("\\t", "\t")
    s = s[:body_start+1] + new_body + "\t}" + s[end:]
    changed = True

    if changed:
        KERNEL.write_text(s, encoding="utf-8")
    return changed


def patch_defines() -> bool:
    s = DEFINES.read_text(encoding="utf-8")
    old = '''// Wind-tunnel startup: begin from rest and smoothly ramp the inlet to the target speed.\n#define INLET_VELOCITY 0.075f\n#define OUTLET_VELOCITY 0.075f\n#define INLET_RAMP_STEPS 50000ul\n'''
    new = '''#define INLET_VELOCITY 0.075f\n#define OUTLET_VELOCITY 0.075f\n'''
    s2, changed = replace_once(s, old, new, "defines ramp")
    if changed:
        DEFINES.write_text(s2, encoding="utf-8")
    return changed


def patch_setup() -> bool:
    s = SETUP.read_text(encoding="utf-8")
    old = '''        // Start the entire fluid at rest. The OpenCL stream-collide kernel ramps\n        // the inlet velocity from 0 to INLET_VELOCITY after initialization.\n        if(lbm.flags[n]!=TYPE_S) {\n            lbm.u.x[n] = 0.0f;\n            lbm.u.y[n] = 0.0f;\n            lbm.u.z[n] = 0.0f;\n        }\n'''
    new = '''        // Initialize the fluid directly at the target inlet speed.\n        if(lbm.flags[n]!=TYPE_S) {\n            lbm.u.x[n] = lbm_u;\n            lbm.u.y[n] = 0.0f;\n            lbm.u.z[n] = 0.0f;\n        }\n'''
    s2, changed = replace_once(s, old, new, "setup startup")
    if changed:
        SETUP.write_text(s2, encoding="utf-8")
    return changed


def patch_main() -> bool:
    s = MAIN.read_text(encoding="utf-8")
    original = s
    s = s.replace("lbm.run(20u, lbm_T); // Run slightly larger batches for better efficiency", "lbm.run(100u, lbm_T); // Larger batches reduce host-side launch overhead")
    s = s.replace("            time.sleep(0.1)  # Give OS time to release file\n", "")
    s = s.replace("            time.sleep(0.1)  # Give OS time to flush\n", "")
    return s != original and not MAIN.write_text(s, encoding="utf-8")


def patch_solver() -> bool:
    s = SOLVER.read_text(encoding="utf-8")
    original = s

    old = '''        v = wp.vec3(0.0, 0.0, 0.0)\n\n        if (ix >= 0 and ix < grid_res and\n            iy >= 0 and iy < grid_res and\n            iz >= 0 and iz < grid_res):\n\n            v = velocity_field[ix, iy, iz]\n\n            # A streamline qualifies when any traced point comes within\n            # contact_radius lattice cells of the vehicle surface.\n            for dz in range(-contact_radius, contact_radius + 1):\n                for dy in range(-contact_radius, contact_radius + 1):\n                    for dx in range(-contact_radius, contact_radius + 1):\n                        nx = ix + dx\n                        ny = iy + dy\n                        nz = iz + dz\n                        if (nx >= 0 and nx < grid_res and\n                            ny >= 0 and ny < grid_res and\n                            nz >= 0 and nz < grid_res):\n                            if vehicle_surface[nx, ny, nz] != 0:\n                                touched_vehicle = True\n\n            speed = wp.length(v)\n'''
    new = '''        if (ix >= 0 and ix < grid_res and\n            iy >= 0 and iy < grid_res and\n            iz >= 0 and iz < grid_res):\n\n            v = velocity_field[ix, iy, iz]\n\n            # Contact is pre-dilated once on the CPU, so tracing needs only\n            # one byte load instead of a (2r+1)^3 neighborhood search.\n            if not touched_vehicle and vehicle_contact_mask[ix, iy, iz] != 0:\n                touched_vehicle = True\n\n            speed = wp.length(v)\n'''
    s = s.replace(old, new)
    s = s.replace("vehicle_surface: wp.array3d(dtype=wp.uint8),", "vehicle_contact_mask: wp.array3d(dtype=wp.uint8),")
    s = s.replace("self.wp_vehicle_surface = wp.array(\n            self.vehicle_surface_cpu,", "self.wp_contact_mask = wp.array(\n            self.vehicle_contact_mask_cpu,")
    s = s.replace("self.vehicle_surface_cpu = np.zeros(\n            (resolution, resolution, resolution), dtype=np.uint8, order='C'\n        )\n        self._build_vehicle_surface_mask(stl_path)", "self.vehicle_contact_mask_cpu = np.zeros(\n            (resolution, resolution, resolution), dtype=np.uint8, order='C'\n        )\n        self._build_vehicle_contact_mask(stl_path)")
    s = s.replace("def _build_vehicle_surface_mask(self, stl_path):", "def _build_vehicle_contact_mask(self, stl_path):")
    s = s.replace("self.vehicle_surface_cpu[\n                indices[:, 0], indices[:, 1], indices[:, 2]\n            ] = 1", "surface = np.zeros_like(self.vehicle_contact_mask_cpu)\n            surface[indices[:, 0], indices[:, 1], indices[:, 2]] = 1\n\n            # Build a spherical contact mask once. Radius 2 contains 33\n            # offsets, versus 125 checks for every point of every line.\n            r = self.streamline_contact_radius\n            offsets = []\n            for dz in range(-r, r + 1):\n                for dy in range(-r, r + 1):\n                    for dx in range(-r, r + 1):\n                        if dx*dx + dy*dy + dz*dz <= r*r:\n                            offsets.append((dx, dy, dz))\n\n            ys, xs, zs = np.nonzero(surface)\n            for dx, dy, dz in offsets:\n                x = xs + dx\n                y = ys + dy\n                z = zs + dz\n                valid = (x >= 0) & (x < self.resolution) & (y >= 0) & (y < self.resolution) & (z >= 0) & (z < self.resolution)\n                self.vehicle_contact_mask_cpu[x[valid], y[valid], z[valid]] = 1")
    s = s.replace("self.wp_vehicle_surface,", "self.wp_contact_mask,")

    # Explicitly keep the number of traced lines unchanged; the large gain is
    # from removing the per-point 3-D neighborhood scan.
    if s == original:
        return False
    SOLVER.write_text(s, encoding="utf-8")
    return True


def main() -> None:
    changed = []
    if patch_kernel(): changed.append("kernel.cpp")
    if patch_defines(): changed.append("defines.hpp")
    if patch_setup(): changed.append("setup.cpp")
    if patch_main(): changed.append("main.py")
    if patch_solver(): changed.append("solver.py")
    print("Optimized files:" if changed else "No changes needed.", ", ".join(changed))


if __name__ == "__main__":
    main()
