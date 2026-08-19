from pathlib import Path
import re

KERNEL = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src" / "kernel.cpp"

SPEED_THRESHOLD = "0.03f"
DIRECTION_THRESHOLD = "0.02f"
FALLBACK_U_INF = "0.075f"
VEHICLE_RADIUS = 4

START = ')+"#ifndef TEMPERATURE"+R(\n)+R(kernel void graphics_streamline'
END = ')+"#ifndef TEMPERATURE"+R(\n)+R(kernel void graphics_q_field'

s = KERNEL.read_text(encoding="utf-8")
start = s.find(START)
if start < 0:
    raise RuntimeError("graphics_streamline start marker not found")
end = s.find(END, start)
if end < 0:
    raise RuntimeError("graphics_streamline end marker not found")

block = s[start:end]

classification = '''\t// First pass: accept a streamline only when its trajectory both\n\t// comes close to the vehicle and experiences a real velocity disturbance.\n\tbool near_vehicle = false;\n\tbool affected = false;\n\tconst float fallback_U_inf = 0.075f;\n\tconst float speed_threshold = 0.03f;\n\tconst float direction_threshold = 0.02f;\n\n\tint ref_y = clamp((int)(p.y + 0.5f*(float)def_Ny), 0, (int)def_Ny-1);\n\tint ref_z = clamp((int)(p.z + 0.5f*(float)def_Nz), 0, (int)def_Nz-1);\n\tfloat3 freestream = (float3)(fallback_U_inf, 0.0f, 0.0f);\n\tbool found_inlet = false;\n\tfor(uint ix=1u; ix<min(8u,(uint)def_Nx) && !found_inlet; ix++) {\n\t\tconst uxx ref_n=(uxx)ix+(uxx)((uint)ref_y+(uint)ref_z*def_Ny)*(uxx)def_Nx;\n\t\tif(!(flags[ref_n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G))) {\n\t\t\tconst float3 ref_u=load3(ref_n,u);\n\t\t\tif(length(ref_u)>1.0e-6f) { freestream=ref_u; found_inlet=true; }\n\t\t}\n\t}\n\tconst float freestream_speed=length(freestream);\n\tconst float3 freestream_dir=freestream/fmax(freestream_speed,1.0e-6f);\n\n\tfor(float dt=-1.0f; dt<=1.0f && !(near_vehicle && affected); dt+=2.0f) {\n\t\tfloat3 p1=p;\n\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {\n\t\t\tif(p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;\n\n\t\t\tconst uint xx=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;\n\t\t\tconst uint yy=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;\n\t\t\tconst uint zz=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;\n\t\t\tconst uxx nn=(uxx)xx+(uxx)(yy+zz*def_Ny)*(uxx)def_Nx;\n\t\t\tif(flags[nn]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;\n\n\t\t\t// Cheap radial vehicle proximity test. We sample the 6 axial\n\t\t\t// directions out to VEHICLE_RADIUS lattice cells. This is enough\n\t\t\t// to catch nearby trajectories without scanning a large 3-D cube.\n\t\t\tfor(int r=1; r<=VEHICLE_RADIUS && !near_vehicle; r++) {\n\t\t\t\tconst int dxs[6] = { r, -r, 0, 0, 0, 0 };\n\t\t\t\tconst int dys[6] = { 0, 0, r, -r, 0, 0 };\n\t\t\t\tconst int dzs[6] = { 0, 0, 0, 0, r, -r };\n\t\t\t\tfor(int q=0; q<6; q++) {\n\t\t\t\t\tconst int vx=(int)xx+dxs[q];\n\t\t\t\t\tconst int vy=(int)yy+dys[q];\n\t\t\t\t\tconst int vz=(int)zz+dzs[q];\n\t\t\t\t\tif(vx<0 || vx>=(int)def_Nx || vy<0 || vy>=(int)def_Ny || vz<0 || vz>=(int)def_Nz) continue;\n\t\t\t\t\tconst uxx vn=(uxx)vx+(uxx)((uint)vy+(uint)vz*def_Ny)*(uxx)def_Nx;\n\t\t\t\t\tnear_vehicle = (flags[vn]&TYPE_S)!=0u;\n\t\t\t\t\tif(near_vehicle) break;\n\t\t\t\t}\n\t\t\t}\n\n\t\t\tconst float3 un=load3(nn,u);\n\t\t\tconst float ul=length(un);\n\t\t\tif(ul<=1.0e-6f) break;\n\t\t\tconst float speed_disturbance=fabs(ul-freestream_speed)/fmax(freestream_speed,1.0e-6f);\n\t\t\tconst float direction_disturbance=1.0f-dot(un/fmax(ul,1.0e-6f),freestream_dir);\n\t\t\tif(speed_disturbance>=speed_threshold || direction_disturbance>=direction_threshold) affected=true;\n\n\t\t\tconst float3 next_p1=p1+(dt/ul)*un;\n\t\t\tif(next_p1.x<-hLx || next_p1.x>hLx || next_p1.y<-hLy || next_p1.y>hLy || next_p1.z<-hLz || next_p1.z>hLz) break;\n\t\t\tp1=next_p1;\n\t\t\tif(def_scale_u*ul<0.1f) break;\n\t\t}\n\t}\n\tif(!(near_vehicle && affected)) return;\n\n'''

pattern = r"\t// First pass: classify.*?\n\tif\(!affected\) return;\n\n"
block2, count = re.subn(pattern, classification, block, count=1, flags=re.S)
if count != 1:
    raise RuntimeError("existing streamline classification block not found")

s = s[:start] + block2 + s[end:]
KERNEL.write_text(s, encoding="utf-8")
print("Streamlines now require both vehicle proximity and a velocity disturbance.")
