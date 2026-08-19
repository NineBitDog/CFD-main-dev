from pathlib import Path
import re

KERNEL = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src" / "kernel.cpp"

SPEED_THRESHOLD = "0.03f"
DIRECTION_THRESHOLD = "0.02f"
FALLBACK_U_INF = "0.075f"

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

# Replace the classification pass with the working inlet-based velocity test.
classification = '''\t// First pass: classify the complete trajectory from the actual inlet flow.\n\tbool affected = false;\n\tconst float fallback_U_inf = 0.075f;\n\tconst float speed_threshold = 0.03f;\n\tconst float direction_threshold = 0.02f;\n\n\tint ref_y = clamp((int)(p.y + 0.5f*(float)def_Ny), 0, (int)def_Ny-1);\n\tint ref_z = clamp((int)(p.z + 0.5f*(float)def_Nz), 0, (int)def_Nz-1);\n\tfloat3 freestream = (float3)(fallback_U_inf, 0.0f, 0.0f);\n\tbool found_inlet = false;\n\tfor(uint ix=1u; ix<min(8u,(uint)def_Nx) && !found_inlet; ix++) {\n\t\tconst uxx ref_n=(uxx)ix+(uxx)((uint)ref_y+(uint)ref_z*def_Ny)*(uxx)def_Nx;\n\t\tif(!(flags[ref_n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G))) {\n\t\t\tconst float3 ref_u=load3(ref_n,u);\n\t\t\tif(length(ref_u)>1.0e-6f) { freestream=ref_u; found_inlet=true; }\n\t\t}\n\t}\n\tconst float freestream_speed=length(freestream);\n\tconst float3 freestream_dir=freestream/fmax(freestream_speed,1.0e-6f);\n\n\tfor(float dt=-1.0f; dt<=1.0f && !affected; dt+=2.0f) {\n\t\tfloat3 p1=p;\n\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {\n\t\t\t// Only sample while inside this domain. Modulo is used solely for a\n\t\t\t// valid in-bounds cell index, so a streamline cannot wrap across a face.\n\t\t\tif(p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;\n\t\t\tconst uint xx=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;\n\t\t\tconst uint yy=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;\n\t\t\tconst uint zz=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;\n\t\t\tconst uxx nn=(uxx)xx+(uxx)(yy+zz*def_Ny)*(uxx)def_Nx;\n\t\t\tif(flags[nn]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;\n\t\t\tconst float3 un=load3(nn,u);\n\t\t\tconst float ul=length(un);\n\t\t\tif(ul<=1.0e-6f) break;\n\t\t\tconst float speed_disturbance=fabs(ul-freestream_speed)/fmax(freestream_speed,1.0e-6f);\n\t\t\tconst float direction_disturbance=1.0f-dot(un/fmax(ul,1.0e-6f),freestream_dir);\n\t\t\tif(speed_disturbance>=speed_threshold || direction_disturbance>=direction_threshold) {\n\t\t\t\taffected=true;\n\t\t\t\tbreak;\n\t\t\t}\n\t\t\tconst float3 next_p1=p1+(dt/ul)*un;\n\t\t\tif(next_p1.x<-hLx || next_p1.x>hLx || next_p1.y<-hLy || next_p1.y>hLy || next_p1.z<-hLz || next_p1.z>hLz) break;\n\t\t\tp1=next_p1;\n\t\t\tif(def_scale_u*ul<0.1f) break;\n\t\t}\n\t}\n\tif(!affected) return;\n\n'''
pattern = r"\t// First pass: classify.*?\n\tif\(!affected\) return;\n\n"
block2, count = re.subn(pattern, classification, block, count=1, flags=re.S)
if count != 1:
    raise RuntimeError("existing streamline classification block not found")

render_start = block2.find("\t// Second pass:")
if render_start < 0:
    raise RuntimeError("streamline rendering pass not found")
render = block2[render_start:]

old_render_coords = [
    "\t\t\tconst uint x=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;\n\t\t\tconst uint y=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;\n\t\t\tconst uint z=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;",
    "\t\t\tif(p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;\n\t\t\tconst uint x=(uint)(p1.x+1.5f*(float)def_Nx);\n\t\t\tconst uint y=(uint)(p1.y+1.5f*(float)def_Ny);\n\t\t\tconst uint z=(uint)(p1.z+1.5f*(float)def_Nz);\n\t\t\tif(x>=def_Nx || y>=def_Ny || z>=def_Nz) break;",
    "\t\t\tif(p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;\n\t\t\tconst uint xx=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;\n\t\t\tconst uint yy=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;\n\t\t\tconst uint zz=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;"
]
safe_coords = (
    "\t\t\tif(p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;\n"
    "\t\t\tconst uint xx=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;\n"
    "\t\t\tconst uint yy=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;\n"
    "\t\t\tconst uint zz=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;\n"
)
for old in old_render_coords:
    if old in render:
        render = render.replace(old, safe_coords, 1)
        break

render = re.sub(
    r"\t\t\tp1\+=\(dt/ul\)\*un;\n",
    "\t\t\tconst float3 next_p1=p1+(dt/ul)*un;\n\t\t\tif(next_p1.x<-hLx || next_p1.x>hLx || next_p1.y<-hLy || next_p1.y>hLy || next_p1.z<-hLz || next_p1.z>hLz) break;\n\t\t\tp1=next_p1;\n",
    render,
    count=1,
)

block2 = block2[:render_start] + render
s = s[:start] + block2 + s[end:]

KERNEL.write_text(s, encoding="utf-8")
print("Updated native streamlines: preserve inlet-based disturbance filtering and stop integration before domain-face wraparound.")
