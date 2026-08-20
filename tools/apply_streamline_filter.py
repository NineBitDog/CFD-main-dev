from pathlib import Path

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

    # Remove the hard-coded inlet startup ramp from stream_collide().
    old = '''\tif(flagsn_bo==TYPE_E) {\n\t\trhon = rho[n];\n\t\tconst uint xcoord = coordinates(n).x;\n\t\tconst bool inlet_side = xcoord <= 1u;\n\t\tif(inlet_side) {\n\t\t\tfloat ramp = clamp((float)t/(float)1000ul, 0.0f, 1.0f);\n\t\t\t// Smoothstep startup avoids an impulse into the car at t=0.\n\t\t\tramp = ramp*ramp*(3.0f-2.0f*ramp);\n\t\t\tuxn = 0.075f*ramp;\n\t\t\tuyn = 0.0f;\n\t\t\tuzn = 0.0f;\n\t\t} else {\n\t\t\t// Constant-velocity equilibrium outlet.\n\t\t\tuxn = 0.075f;\n\t\t\tuyn = 0.0f;\n\t\t\tuzn = 0.0f;\n\t\t}\n\t} else {\n'''
    new = '''\tif(flagsn_bo==TYPE_E) {\n\t\trhon = rho[n];\n\t\tuxn = 0.075f;\n\t\tuyn = 0.0f;\n\t\tuzn = 0.0f;\n\t} else {\n'''
    s2, did = replace_once(s, old, new, "stream_collide inlet ramp")
    changed |= did
    s = s2

    # Upstream FluidX3D velocity math: trilinearly interpolate the velocity
    # field during streamline integration instead of sampling the nearest cell.
    # This makes both the streamline path and disturbance classification use the
    # same continuous velocity field used by the upstream renderer/math helpers.
    if "float3 interpolate_u_streamline(const global float* u, const float3 p)" not in s:
        marker = ")+R(float3 load3(const global float* p, const uxx n) {"
        helper = ")+R(float3 interpolate_u_streamline(const global float* u, const float3 p) { // FluidX3D-style trilinear velocity interpolation
\tconst float xa=p.x-0.5f+1.5f*(float)def_Nx, ya=p.y-0.5f+1.5f*(float)def_Ny, za=p.z-0.5f+1.5f*(float)def_Nz;
\tconst uint xb=(uint)xa, yb=(uint)ya, zb=(uint)za;
\tconst float3 pf=(float3)(xa-(float)xb, ya-(float)yb, za-(float)zb);
\tfloat3 v[8];
\tfor(uint c=0u; c<8u; c++) {
\t\tconst uint i=(c&4u)>>2, j=(c&2u)>>1, k=c&1u;
\t\tconst uint x=(xb+i)%def_Nx, y=(yb+j)%def_Ny, z=(zb+k)%def_Nz;
\t\tconst uxx n=(uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;
\t\tv[c]=load3(u,n);
\t}
\tconst float3 a = v[0] + pf.x*(v[4]-v[0]);
\tconst float3 b = v[1] + pf.x*(v[5]-v[1]);
\tconst float3 c = v[2] + pf.x*(v[6]-v[2]);
\tconst float3 d = v[3] + pf.x*(v[7]-v[3]);
\tconst float3 e = a + pf.y*(b-a);
\tconst float3 f = d + pf.y*(c-d);
\treturn fma(pf.z, f-e, e);
}
)+R(float3 load3(const global float* p, const uxx n) {"
        if marker not in s:
            raise RuntimeError("load3 helper marker not found")
        s = s.replace(marker, helper, 1)
        changed = True

    # Replace the streamline filter/render body while preserving the exact native
    # kernel signature and D2Q9/D3Q19 seed generation.
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
\tconst float SPEED_DELTA = 0.00225f;
\tconst float DIRECTION_DELTA = 0.02f;

\t// One integration pass. A segment is rendered only while its sampled
\t// velocity is actually disturbed from freestream. This keeps the seed
\t// distribution but removes unaffected portions of the same streamline.
\tfor(float dt=-1.0f; dt<=1.0f; dt+=2.0f) {
\t\tfloat3 p0=p;
\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {
\t\t\tif(p0.x<-hLx || p0.x>hLx || p0.y<-hLy || p0.y>hLy || p0.z<-hLz || p0.z>hLz) break;
\t\t\tconst float3 un=interpolate_u_streamline(u,p0);
\t\t\tconst float ul=length(un);
\t\t\tif(ul<=1.0e-6f) break;
\t\t\tconst float inv_ul=1.0f/ul;
\t\t\tconst bool impacted = fabs(ul-U_INF)>=SPEED_DELTA || (ul-un.x)>=DIRECTION_DELTA*ul;
\t\t\tfloat3 p1 = p0 + (dt*inv_ul)*un;
\t\t\tif(p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;
\t\t\tif(impacted) {
\t\t\t\tconst uint3 q=closest_coordinates(p0);
\t\t\t\tconst uxx nn=index(q);
\t\t\t\tif(!(flags[nn]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G))) {
\t\t\t\t\tint c=0;
\t\t\t\t\tswitch(field_mode) {
\t\t\t\t\t\tcase 0: c=colorscale_rainbow(def_scale_u*ul); break;
\t\t\t\t\t\tcase 1: c=colorscale_twocolor(0.5f+def_scale_rho*(rho[nn]-1.0f)); break;
)+"#ifdef TEMPERATURE"+R(
\t\t\t\t\t\tcase 2: c=colorscale_iron(0.5f+def_scale_T*(T[nn]-def_T_avg)); break;
)+"#endif"+R(
\t\t\t\t\t}
\t\t\t\t\tdraw_line(p0,p1,c,camera_cache,bitmap,zbuffer);
\t\t\t\t}
\t\t\t}
\t\t\tp0=p1;
\t\t}
\t}
'''
    new_body = new_body.replace("\\n", "\n").replace("\\t", "\t")
    s = s[:body_start+1] + new_body + "\t}" + s[end:]
    changed = True

    if changed:
        KERNEL.write_text(s, encoding="utf-8")
    return changed


def main() -> None:
    patch_kernel()
    print("Streamline filter/math patch applied.")


if __name__ == "__main__":
    main()
