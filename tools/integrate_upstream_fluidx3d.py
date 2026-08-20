from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
KERNEL = ROOT / "FluidX3D-master" / "src" / "kernel.cpp"
OPENCL = ROOT / "FluidX3D-master" / "src" / "opencl.hpp"
LBM = ROOT / "FluidX3D-master" / "src" / "lbm.cpp"


def replace_once(text: str, old: str, new: str) -> bool:
    if old not in text:
        return False
    text = text.replace(old, new, 1)
    return True


def patch_kernel() -> bool:
    s = KERNEL.read_text(encoding="utf-8")
    original = s

    # FluidX3D v3.7: cache the first marching-cubes table lookup.
    old_mc = '''\tfor(i=0u; i<15u&&triangle_table(cube+i)!=15u; i+=3u) { // create the triangles\n\t\ttriangles[i   ] = vertex[triangle_table(cube+i   )];\n'''
    new_mc = '''\tfor(i=0u; i<15u; i+=3u) { // create the triangles\n\t\tconst uchar triangle_table_cube_i_0u_ = triangle_table(cube+i);\n\t\tif(triangle_table_cube_i_0u_==15u) break;\n\t\ttriangles[i   ] = vertex[triangle_table_cube_i_0u_];\n'''
    # There are two matching implementations: marching_cubes() and
    # marching_cubes_halfway(). Apply the same transformation to both.
    s = s.replace(old_mc, new_mc)

    # FluidX3D v3.7: use fused multiply-adds in flat-shading distance terms.
    s = s.replace(
        '''\tconst float nl2 = sq(normal.x)+sq(normal.y)+sq(normal.z); // only one rsqrt instead of two\n\tconst float dl2 = sq(d.x)+sq(d.y)+sq(d.z);\n\treturn color_mul(c, max(1.5f*fabs(dot(normal, d))*rsqrt(nl2*dl2), 0.3f));\n''',
        '''\tconst float nl2 = fma(normal.x, normal.x, fma(normal.y, normal.y, sq(normal.z))); // only one rsqrt instead of two\n\tconst float dl2 = fma(d.x, d.x, fma(d.y, d.y, sq(d.z)));\n\treturn color_mul(c, max(1.25f*fabs(dot(normal, d))*rsqrt(nl2*dl2), 0.3f));\n''', 1)

    # FluidX3D 2026-05: atomic bitmap writes after a successful z-test.
    old_sig = ''')+R(void draw(const int x, const int y, const float z, const int color, global int* bitmap, volatile global int* zbuffer, const int stereo) {\n\tconst int index=x+y*def_screen_width, iz=(int)(z*1E3f); // use fixed-point int z-buffer and atomic_max to minimize noise in image, maximum render distance is 2.147E6f\n'''
    new_sig = ''')+R(void draw(const int x, const int y, const float z, const int color, volatile global int* bitmap, volatile global int* zbuffer, const int stereo) {\n\tconst int index=x+y*def_screen_width, iz=(int)(z*1E3f); // use fixed-point int z-buffer and atomic_max to minimize noise in image, maximum render distance is 2.147E6f\n\tif(!is_off_screen(x, y, stereo)) { // only draw if point is on screen\n'''
    if old_sig in s:
        s = s.replace(old_sig, new_sig, 1)
        s = s.replace(
            '''\tif(!is_off_screen(x, y, stereo)&&iz>atomic_max(&zbuffer[index], iz)) bitmap[index] = color; // only draw if point is on screen and first in zbuffer\n''',
            '''\t\tif(iz>atomic_max(&zbuffer[index], iz)) atomic_xchg(&bitmap[index], color); // only draw if point is first in zbuffer\n''', 1)
        s = s.replace(
            '''\tif(!is_off_screen(x, y, stereo)) { // transparent rendering (not quite order-independent transparency, but elegant solution for order-reversible transparency which is good enough here)\n\t\tconst float transparency = GRAPHICS_TRANSPARENCY;\n''',
            '''\t\tconst float transparency = GRAPHICS_TRANSPARENCY; // transparent rendering (not quite order-independent transparency, but elegant solution for order-reversible transparency which is good enough here)\n''', 1)
        s = s.replace('''\t\tconst bool is_front = iz>atomic_max(&zbuffer[index], iz);\n''', '', 1)
        s = s.replace(
            '''\t\tconst float3 fn = fp+(1.0f-transparency)*( is_front ? fc-fp : pown(transparency, draw_count)*(fc-fb)); // black magic: either over-draw colors back-to-front, or add back colors as correction terms\n\t\tbitmap[index] = as_int((uchar4)((uchar)clamp(fn.x+0.5f, 0.0f, 255.0f), (uchar)clamp(fn.y+0.5f, 0.0f, 255.0f), (uchar)clamp(fn.z+0.5f, 0.0f, 255.0f), (uchar)min(draw_count+1, 255)));\n\t}\n''',
            '''\t\tconst float3 fn = fp+(1.0f-transparency)*(iz>atomic_max(&zbuffer[index], iz) ? fc-fp : pown(transparency, draw_count)*(fc-fb)); // black magic: either over-draw colors back-to-front, or add back colors as correction terms\n\t\tatomic_xchg(&bitmap[index], as_int((uchar4)((uchar)clamp(fn.x+0.5f, 0.0f, 255.0f), (uchar)clamp(fn.y+0.5f, 0.0f, 255.0f), (uchar)clamp(fn.z+0.5f, 0.0f, 255.0f), (uchar)min(draw_count+1, 255))));\n''', 1)
        s = s.replace(
            ''')+"#endif"+R( // GRAPHICS_TRANSPARENCY\n}\n)+R(bool convert''',
            ''')+"#endif"+R( // GRAPHICS_TRANSPARENCY\n\t}\n}\n)+R(bool convert''', 1)

    if s != original:
        KERNEL.write_text(s, encoding="utf-8")
    return s != original


def patch_opencl() -> bool:
    s = OPENCL.read_text(encoding="utf-8")
    old = '''\t\tif(error==-54) print_error("Workgrop size "+to_string(WORKGROUP_SIZE)+" for OpenCL kernel \\\""+name+"(...)\\\" is invalid!");\n'''
    new = '''\t\tif(error==-54) print_error("Workgrop size "+to_string((ulong)WORKGROUP_SIZE)+" for OpenCL kernel \\\""+name+"(...)\\\" is invalid!");\n'''
    if old not in s:
        return False
    OPENCL.write_text(s.replace(old, new, 1), encoding="utf-8")
    return True


def patch_lbm_dispatch() -> bool:
    s = LBM.read_text(encoding="utf-8")
    original = s
    # Only change launch ranges when the exact older form exists. This is
    # intentionally conservative because this fork has a custom multi-domain
    # graphics allocator.
    replacement = '(ulong)(lbm->get_Nx()-1u)*(ulong)(lbm->get_Ny()-1u)*(ulong)(lbm->get_Nz()-1u)'
    for name in ("graphics_flags_mc", "graphics_rasterize_phi"):
        pattern = re.compile(r'Kernel\(device,\s*lbm->get_N\(\),\s*"' + re.escape(name) + r'"')
        m = pattern.search(s)
        if not m:
            continue
        segment = s[m.start():m.end()]
        segment2 = segment.replace('lbm->get_N()', replacement, 1)
        s = s[:m.start()] + segment2 + s[m.end():]
    if s != original:
        LBM.write_text(s, encoding="utf-8")
        return True
    return False


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
