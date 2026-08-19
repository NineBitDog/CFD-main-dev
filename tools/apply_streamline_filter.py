from pathlib import Path

KERNEL = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src" / "kernel.cpp"

# These are simulation/visualization constants. defines.hpp is a C++ header;
# its GRAPHICS_* macros are not automatically visible inside the OpenCL source
# assembled by kernel.cpp, so the generated OpenCL must use literal constants.
U_INF = "0.075f"
SPEED_THRESHOLD = "0.03f"
DIRECTION_THRESHOLD = "0.02f"

CURRENT_MARKER = "\t// A streamline is rendered only if its trajectory actually reaches a\n"
ALREADY_PATCHED = "\t// First pass: classify the streamline from the velocity field only."
END = "\n)+\"#ifndef TEMPERATURE\"+R(\n)+R(kernel void graphics_q_field"

NEW = f'''\t// First pass: classify the streamline from the velocity field only.\n\t// Freestream is +X. Render only if speed or direction is disturbed.\n\tbool affected = false;\n\tconst float U_inf = {U_INF};\n\tconst float speed_threshold = {SPEED_THRESHOLD};\n\tconst float direction_threshold = {DIRECTION_THRESHOLD};\n\tconst float3 freestream_dir = (float3)(1.0f, 0.0f, 0.0f);\n\n\tfor(float dt=-1.0f; dt<=1.0f && !affected; dt+=2.0f) {{\n\t\tfloat3 p1=p;\n\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {{\n\t\t\tconst uint x=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;\n\t\t\tconst uint y=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;\n\t\t\tconst uint z=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;\n\t\t\tconst uxx n=(uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;\n\t\t\tif(flags[n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;\n\t\t\tconst float3 un=load3(n,u);\n\t\t\tconst float ul=length(un);\n\t\t\tif(ul<=0.0f) break;\n\t\t\tconst float speed_disturbance=fabs(ul-U_inf)/fmax(U_inf,1.0e-6f);\n\t\t\tconst float3 flow_dir=un/fmax(ul,1.0e-6f);\n\t\t\tconst float direction_disturbance=1.0f-dot(flow_dir,freestream_dir);\n\t\t\tif(speed_disturbance>=speed_threshold || direction_disturbance>=direction_threshold) {{\n\t\t\t\taffected=true;\n\t\t\t\tbreak;\n\t\t\t}}\n\t\t\tp1+=(dt/ul)*un;\n\t\t\tif(def_scale_u*ul<0.1f || p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;\n\t\t}}\n\t}}\n\tif(!affected) return;\n\n\t// Second pass: render only the affected streamline.\n\tfor(float dt=-1.0f; dt<=1.0f; dt+=2.0f) {{\n\t\tfloat3 p0, p1=p;\n\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {{\n\t\t\tconst uint x=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;\n\t\t\tconst uint y=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;\n\t\t\tconst uint z=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;\n\t\t\tconst uxx n=(uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;\n\t\t\tif(flags[n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;\n\t\t\tconst float3 un=load3(n,u);\n\t\t\tconst float ul=length(un);\n\t\t\tif(ul<=0.0f) break;\n\t\t\tp0=p1;\n\t\t\tp1+=(dt/ul)*un;\n\t\t\tif(def_scale_u*ul<0.1f || p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;\n\t\t\tint c=0;\n\t\t\tswitch(field_mode) {{\n\t\t\t\tcase 0: c=colorscale_rainbow(def_scale_u*ul); break;\n\t\t\t\tcase 1: c=colorscale_twocolor(0.5f+def_scale_rho*(rho[n]-1.0f)); break;\n)+\"#ifdef TEMPERATURE\"+R(\n\t\t\t\tcase 2: c=colorscale_iron(0.5f+def_scale_T*(T[n]-def_T_avg)); break;\n)+\"#endif\"+R(\n\t\t\t}}\n\t\t\tdraw_line(p0,p1,c,camera_cache,bitmap,zbuffer);\n\t\t}}\n\t}}\n'''

FIELD_OLD = '''\t\tcase 0: // coloring by velocity\n\t\t\twhile(traversed_cells<Nx+Ny+Nz) { // limit number of traversed cells to space diagonal\n\t\t\t\tif(tmx<tmy) { if(tmx<tmz) { xyz.x += dx; tmx += tdx; } else { xyz.z += dz; tmz += tdz; } }\n\t\t\t\telse /****/ { if(tmy<tmz) { xyz.y += dy; tmy += tdy; } else { xyz.z += dz; tmz += tdz; } }\n\t\t\t\tif(xyz.x<0 || xyz.y<0 || xyz.z<0 || xyz.x>=(int)Nx || xyz.y>=(int)Ny || xyz.z>=(int)Nz) break; // out of simulation box\n\t\t\t\tconst uxx n = index((uint3)((uint)clamp(xyz.x, 0, (int)Nx-1), (uint)clamp(xyz.y, 0, (int)Ny-1), (uint)clamp(xyz.z, 0, (int)Nz-1)));\n\t\t\t\tif(!(flags[n]&(TYPE_S|TYPE_E|TYPE_G))) {\n\t\t\t\t\tconst float un = length(load3(n, u));\n\t\t\t\t\tconst float weight = fmin(un, fabs(un-0.5f/def_scale_u));\n\t\t\t\t\tsum = fma(weight, un, sum);\n\t\t\t\t\ttraversed_cells_weighted += weight;\n\t\t\t\t}\n\t\t\t\ttraversed_cells++;\n\t\t\t}\n\t\t\tcolor = colorscale_rainbow(def_scale_u*sum/traversed_cells_weighted);\n\t\t\ttraversed_cells_weighted *= 2.0f*def_scale_u;\n\t\t\tbreak;'''

FIELD_NEW = f'''\t\tcase 0: // coloring by velocity\n\t\t\twhile(traversed_cells<Nx+Ny+Nz) {{\n\t\t\t\tif(tmx<tmy) {{ if(tmx<tmz) {{ xyz.x += dx; tmx += tdx; }} else {{ xyz.z += dz; tmz += tdz; }} }}\n\t\t\t\telse {{ if(tmy<tmz) {{ xyz.y += dy; tmy += tdy; }} else {{ xyz.z += dz; tmz += tdz; }} }}\n\t\t\t\tif(xyz.x<0 || xyz.y<0 || xyz.z<0 || xyz.x>=(int)Nx || xyz.y>=(int)Ny || xyz.z>=(int)Nz) break;\n\t\t\t\tconst uxx n = index((uint3)((uint)clamp(xyz.x,0,(int)Nx-1),(uint)clamp(xyz.y,0,(int)Ny-1),(uint)clamp(xyz.z,0,(int)Nz-1)));\n\t\t\t\tif(!(flags[n]&(TYPE_S|TYPE_E|TYPE_G))) {{\n\t\t\t\t\tconst float3 vel=load3(n,u);\n\t\t\t\t\tconst float speed=length(vel);\n\t\t\t\t\tconst float speed_disturbance=fabs(speed-{U_INF})/fmax({U_INF},1.0e-6f);\n\t\t\t\t\tconst float3 flow_dir=vel/fmax(speed,1.0e-6f);\n\t\t\t\t\tconst float direction_disturbance=1.0f-dot(flow_dir,(float3)(1.0f,0.0f,0.0f));\n\t\t\t\t\tif(speed_disturbance>={SPEED_THRESHOLD} || direction_disturbance>={DIRECTION_THRESHOLD}) {{\n\t\t\t\t\t\tconst float weight=fmin(speed,fabs(speed-0.5f/def_scale_u));\n\t\t\t\t\t\tsum=fma(weight,speed,sum);\n\t\t\t\t\t\ttraversed_cells_weighted+=weight;\n\t\t\t\t\t}}\n\t\t\t\t}}\n\t\t\t\ttraversed_cells++;\n\t\t\t}}\n\t\t\tif(traversed_cells_weighted<=0.0f) return background_color;\n\t\t\tcolor=colorscale_rainbow(def_scale_u*sum/traversed_cells_weighted);\n\t\t\ttraversed_cells_weighted*=2.0f*def_scale_u;\n\t\t\tbreak;'''

s = KERNEL.read_text(encoding="utf-8")

if ALREADY_PATCHED not in s:
    start = s.find(CURRENT_MARKER)
    if start < 0:
        raise RuntimeError("graphics_streamline current block not found")
    end = s.find(END, start)
    if end < 0:
        raise RuntimeError("graphics_streamline end marker not found")
    s = s[:start] + NEW + s[end:]

if "speed_disturbance=fabs(speed-0.075f)" not in s:
    if FIELD_OLD not in s:
        raise RuntimeError("graphics_field velocity ray-march block not found")
    s = s.replace(FIELD_OLD, FIELD_NEW, 1)

KERNEL.write_text(s, encoding="utf-8")