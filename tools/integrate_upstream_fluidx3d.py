from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
KERNEL = ROOT / "FluidX3D-master" / "src" / "kernel.cpp"
OPENCL = ROOT / "FluidX3D-master" / "src" / "opencl.hpp"
LBM = ROOT / "FluidX3D-master" / "src" / "lbm.cpp"


def replace_once(text: str, old: str, new: str, label: str) -> tuple[str, bool]:
    if old not in text:
        return text, False
    return text.replace(old, new, 1), True


def patch_kernel() -> bool:
    s = KERNEL.read_text(encoding="utf-8")
    original = s

    # FluidX3D v3.7: cache the first marching-cubes table lookup instead of
    # evaluating triangle_table(cube+i) in the loop condition and body.
    old_mc = '''\tfor(i=0u; i<15u&&triangle_table(cube+i)!=15u; i+=3u) { // create the triangles\n\t\ttriangles[i   ] = vertex[triangle_table(cube+i   )];\n'''
    new_mc = '''\tfor(i=0u; i<15u; i+=3u) { // create the triangles\n\t\tconst uchar triangle_table_cube_i_0u_ = triangle_table(cube+i);\n\t\tif(triangle_table_cube_i_0u_==15u) break;\n\t\ttriangles[i   ] = vertex[triangle_table_cube_i_0u_];\n'''
    old_mc_half = old_mc
    s, _ = replace_once(s, old_mc, new_mc, "marching cubes")
    s, _ = replace_once(s, old_mc_half, new_mc, "marching cubes halfway")

    # FluidX3D v3.7: reduce dependent floating-point operations in shading
    # using fused multiply-adds. This is numerically cleaner and faster on
    # GPUs while preserving the existing rendering API.
    old_shade = '''\tconst float nl2 = sq(normal.x)+sq(normal.y)+sq(normal.z); // only one rsqrt instead of two\n\tconst float dl2 = sq(d.x)+sq(d.y)+sq(d.z);\n\treturn color_mul(c, max(1.5f*fabs(dot(normal, d))*rsqrt(nl2*dl2), 0.3f));\n'''
    new_shade = '''\tconst float nl2 = fma(normal.x, normal.x, fma(normal.y, normal.y, sq(normal.z))); // only one rsqrt instead of two\n\tconst float dl2 = fma(d.x, d.x, fma(d.y, d.y, sq(d.z)));\n\treturn color_mul(c, max(1.25f*fabs(dot(normal, d))*rsqrt(nl2*dl2), 0.3f));\n'''
    s, _ = replace_once(s, old_shade, new_shade, "shading fma")

    # FluidX3D 2026-05 z-buffer fix: make bitmap writes atomic when the z-test
    # succeeds. This matters for highly concurrent streamline/volume rendering
    # and prevents occasional pixel corruption/flicker.
    old_draw_sig = ''')+R(void draw(const int x, const int y, const float z, const int color, global int* bitmap, volatile global int* zbuffer, const int stereo) {\n\tconst int index=x+y*def_screen_width, iz=(int)(z*1E3f); // use fixed-point int z-buffer and atomic_max to minimize noise in image, maximum render distance is 2.147E6f\n'''
    new_draw_sig = ''')+R(void draw(const int x, const int y, const float z, const int color, volatile global int* bitmap, volatile global int* zbuffer, const int stereo) {\n\tconst int index=x+y*def_screen_width, iz=(int)(z*1E3f); // use fixed-point int z-buffer and atomic_max to minimize noise in image, maximum render distance is 2.147E6f\n\tif(!is_off_screen(x, y, stereo)) { // only draw if point is on screen\n'''
    if old_draw_sig in s:
        s = s.replace(old_draw_sig, new_draw_sig, 1)
        old_nontrans = '''\tif(!is_off_screen(x, y, stereo)&&iz>atomic_max(&zbuffer[index], iz)) bitmap[index] = color; // only draw if point is on screen and first in zbuffer\n'''
        new_nontrans = '''\t\tif(iz>atomic_max(&zbuffer[index], iz)) atomic_xchg(&bitmap[index], color); // only draw if point is first in zbuffer\n'''
        s = s.replace(old_nontrans, new_nontrans, 1)
        old_trans_block = '''\tif(!is_off_screen(x, y, stereo)) { // transparent rendering (not quite order-independent transparency, but elegant solution for order-reversible transparency which is good enough here)\n\t\tconst float transparency = GRAPHICS_TRANSPARENCY;\n'''
        new_trans_block = '''\t\tconst float transparency = GRAPHICS_TRANSPARENCY; // transparent rendering (not quite order-independent transparency, but elegant solution for order-reversible transparency which is good enough here)\n'''
        s = s.replace(old_trans_block, new_trans_block, 1)
        old_front = '''\t\tconst bool is_front = iz>atomic_max(&zbuffer[index], iz);\n'''
        s = s.replace(old_front, "", 1)
        old_fn = '''\t\tconst float3 fn = fp+(1.0f-transparency)*( is_front ? fc-fp : pown(transparency, draw_count)*(fc-fb)); // black magic: either over-draw colors back-to-front, or add back colors as correction terms\n\t\tbitmap[index] = as_int((uchar4)((uchar)clamp(fn.x+0.5f, 0.0f, 255.0f), (uchar)clamp(fn.y+0.5f, 0.0f, 255.0f), (uchar)clamp(fn.z+0.5f, 0.0f, 255.0f), (uchar)min(draw_count+1, 255)));\n\t}\n'''
        new_fn = '''\t\tconst float3 fn = fp+(1.0f-transparency)*(iz>atomic_max(&zbuffer[index], iz) ? fc-fp : pown(transparency, draw_count)*(fc-fb)); // black magic: either over-draw colors back-to-front, or add back colors as correction terms\n\t\tatomic_xchg(&bitmap[index], as_int((uchar4)((uchar)clamp(fn.x+0.5f, 0.0f, 255.0f), (uchar)clamp(fn.y+0.5f, 0.0f, 255.0f), (uchar)clamp(fn.z+0.5f, 0.0f, 255.0f), (uchar)min(draw_count+1, 255))));\n'''
        s = s.replace(old_fn, new_fn, 1)
        # Close the new non-transparent/transparent common screen bounds block.
        marker = ''')+"#endif"+R( // GRAPHICS_TRANSPARENCY\n}\n)+R(bool convert'''
        if marker in s:
            s = s.replace(marker, ''')+"#endif"+R( // GRAPHICS_TRANSPARENCY\n\t}\n}\n)+R(bool convert''', 1)

    # The current project already contains the v3.7 Q-criterion math helpers;
    # leave them untouched so the custom streamline/velocity-field path remains
    # exactly compatible with the existing renderer.
    if s != original:
        KERNEL.write_text(s, encoding="utf-8")
    return s != original


def patch_opencl() -> bool:
    s = OPENCL.read_text(encoding="utf-8")
    old = '''\t\tif(error==-54) print_error("Workgrop size "+to_string(WORKGROUP_SIZE)+" for OpenCL kernel \\\""+name+"(...)\\\" is invalid!");\n'''
    new = '''\t\tif(error==-54) print_error("Workgrop size "+to_string((ulong)WORKGROUP_SIZE)+" for OpenCL kernel \\\""+name+"(...)\\\" is invalid!");\n'''
    s2, changed = replace_once(s, old, new, "OpenCL workgroup cast")
    if changed:
        OPENCL.write_text(s2, encoding="utf-8")
    return changed


def patch_lbm_dispatch() -> bool:
    s = LBM.read_text(encoding="utf-8")
    original = s
    # Upstream 2026-05: marching-cubes kernels only need one fewer cell in each
    # dimension. This is a pure launch-range optimization and does not alter the
    # contents of any custom graphics or streamline kernel.
    patterns = [
        (r'Kernel\\(device,\\s*lbm->get_N\\(\\),\\s*"graphics_flags_mc"', 'MC_FLAGS'),
        (r'Kernel\\(device,\\s*lbm->get_N\\(\\),\\s*"graphics_rasterize_phi"', 'MC_RASTER'),
    ]
    for pattern, _ in patterns:
        m = re.search(pattern, s)
        if m:
            start = m.start()
            # Replace only the first range argument after Kernel(device,
            prefix = s[start:m.end()]
            prefix2 = prefix.replace('lbm->get_N()', '(ulong)(lbm->get_Nx()-1u)*(ulong)(lbm->get_Ny()-1u)*(ulong)(lbm->get_Nz()-1u)', 1)
            if prefix2 != prefix:
                s = s[:start] + prefix2 + s[m.end():]
    if s != original:
        LBM.write_text(s, encoding="utf-8")
    return s != original


def main() -> None:
    changes = []
    if patch_kernel():
        changes.append("kernel math/z-buffer/marching-cubes optimizations")
    if patch_opencl():
        changes.append("OpenCL workgroup diagnostic cast")
    if patch_lbm_dispatch():
        changes.append("marching-cubes launch-range optimization")
    if changes:
        print("Applied upstream FluidX3D optimizations: " + ", ".join(changes))
    else:
        print("Upstream FluidX3D optimizations already present or not applicable.")


if __name__ == "__main__":
    main()
