from pathlib import Path
import re

KERNEL = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src" / "kernel.cpp"

# Streamline/velocity-volume disturbance criteria.
FALLBACK_U_INF = "0.075f"
SPEED_THRESHOLD = "0.03f"
DIRECTION_THRESHOLD = "0.02f"

START = ')+"#ifndef TEMPERATURE"+R(\n)+R(kernel void graphics_streamline'
END = ')+"#ifndef TEMPERATURE"+R(\n)+R(kernel void graphics_q_field'

s = KERNEL.read_text(encoding="utf-8")

# Remove stale symbolic names left by older patch versions.
s = s.replace("GRAPHICS_STREAMLINE_U_INF", FALLBACK_U_INF)
s = s.replace("GRAPHICS_STREAMLINE_SPEED_THRESHOLD", SPEED_THRESHOLD)
s = s.replace("GRAPHICS_STREAMLINE_DIRECTION_THRESHOLD", DIRECTION_THRESHOLD)

start = s.find(START)
if start < 0:
    raise RuntimeError("graphics_streamline start marker not found")
end = s.find(END, start)
if end < 0:
    raise RuntimeError("graphics_streamline end marker not found")

block = s[start:end]

# Keep the normal FluidX3D rendering pass intact except for the domain-boundary
# handling below. The classification pass uses the actual inlet velocity profile.
new_classification = r'''\t// First pass: classify the complete trajectory from the actual inlet flow.\n\tbool affected = false;\n\tconst float fallback_U_inf = 0.075f;\n\tconst float speed_threshold = 0.03f;\n\tconst float direction_threshold = 0.02f;\n\n\tint ref_y = clamp((int)(p.y + 0.5f*(float)def_Ny), 0, (int)def_Ny-1);\n\tint ref_z = clamp((int)(p.z + 0.5f*(float)def_Nz), 0, (int)def_Nz-1);\n\tfloat3 freestream = (float3)(fallback_U_inf, 0.0f, 0.0f);\n\tbool found_inlet = false;\n\tfor(uint ix=1u; ix<min(8u,(uint)def_Nx) && !found_inlet; ix++) {\n\t\tconst uxx ref_n=(uxx)ix+(uxx)((uint)ref_y+(uint)ref_z*def_Ny)*(uxx)def_Nx;\n\t\tif(!(flags[ref_n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G))) {\n\t\t\tconst float3 ref_u=load3(ref_n,u);\n\t\t\tif(length(ref_u)>1.0e-6f) { freestream=ref_u; found_inlet=true; }\n\t\t}\n\t}\n\tconst float freestream_speed=length(freestream);\n\tconst float3 freestream_dir=freestream/fmax(freestream_speed,1.0e-6f);\n\n\tfor(float dt=-1.0f; dt<=1.0f && !affected; dt+=2.0f) {\n\t\tfloat3 p1=p;\n\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {\n\t\t\t// Do not wrap across a domain face. A reflected boundary cell must terminate\n\t\t\t// the trajectory rather than re-entering from the opposite side.\n\t\t\tif(p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;\n\t\t\tconst uint xx=(uint)(p1.x+1.5f*(float)def_Nx);\n\t\t\tconst uint yy=(uint)(p1.y+1.5f*(float)def_Ny);\n\t\t\tconst uint zz=(uint)(p1.z+1.5f*(float)def_Nz);\n\t\t\tif(xx>=def_Nx || yy>=def_Ny || zz>=def_Nz) break;\n\t\t\tconst uxx nn=(uxx)xx+(uxx)(yy+zz*def_Ny)*(uxx)def_Nx;\n\t\t\tif(flags[nn]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;\n\t\t\tconst float3 un=load3(nn,u);\n\t\t\tconst float ul=length(un);\n\t\t\tif(ul<=1.0e-6f) break;\n\t\t\tconst float speed_disturbance=fabs(ul-freestream_speed)/fmax(freestream_speed,1.0e-6f);\n\t\t\tconst float direction_disturbance=1.0f-dot(un/fmax(ul,1.0e-6f),freestream_dir);\n\t\t\tconst bool point_disturbed=speed_disturbance>=speed_threshold || direction_disturbance>=direction_threshold;\n\t\t\tif(point_disturbed) { affected=true; break; }\n\t\t\tp1+=(dt/ul)*un;\n\t\t\tif(def_scale_u*ul<0.1f) break;\n\t\t}\n\t}\n\tif(!affected) return;\n\n'''
new_classification = new_classification.replace("\\t", "\t").replace("\\n", "\n")
pattern = r"\t// First pass: classify.*?\n\tif\(!affected\) return;\n\n"
block2, count = re.subn(pattern, new_classification, block, count=1, flags=re.S)
if count != 1:
    raise RuntimeError("existing streamline classification block not found")

# Prevent the second/rendering pass from wrapping across the domain box as well.
# The original generated kernel may contain the coordinate calculation in the
# rendering pass; patch only that pass after the classification block.
render_start = block2.find("\t// Second pass: render")
if render_start >= 0:
    render = block2[render_start:]
    render = render.replace(
        "\t\t\tconst uint x=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;\n\t\t\tconst uint y=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;\n\t\t\tconst uint z=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;",
        "\t\t\tif(p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;\n\t\t\tconst uint x=(uint)(p1.x+1.5f*(float)def_Nx);\n\t\t\tconst uint y=(uint)(p1.y+1.5f*(float)def_Ny);\n\t\t\tconst uint z=(uint)(p1.z+1.5f*(float)def_Nz);\n\t\t\tif(x>=def_Nx || y>=def_Ny || z>=def_Nz) break;"
    )
    block2 = block2[:render_start] + render

s = s[:start] + block2 + s[end:]

# The colored velocity volume uses the same local disturbance thresholds as
# the streamline classifier. This is deliberately a LOCAL test: a volume cell
# is rendered only when its velocity is disturbed from the local inlet reference.
FIELD_START = ')+R(kernel void graphics_field_rt'
fs = s.find(FIELD_START)
if fs >= 0:
    fe = s.find(')+R(kernel void ', fs + len(FIELD_START))
    if fe < 0:
        fe = s.find(')+"#endif"+R(', fs)
    if fe > fs:
        fblock = s[fs:fe]
        fblock = re.sub(r'\n\s*// STREAMLINE_VELOCITY_FILTER_BEGIN.*?// STREAMLINE_VELOCITY_FILTER_END\n', '\n', fblock, flags=re.S)
        filter_code = '''\n\t// STREAMLINE_VELOCITY_FILTER_BEGIN\n\t{\n\t\tconst float vf_speed = length(load3(n,u));\n\t\tconst float vf_ref_speed = 0.075f;\n\t\tconst float3 vf_ref_dir = (float3)(1.0f,0.0f,0.0f);\n\t\tconst float vf_speed_disturbance = fabs(vf_speed-vf_ref_speed)/fmax(vf_ref_speed,1.0e-6f);\n\t\tconst float3 vf_dir = load3(n,u)/fmax(vf_speed,1.0e-6f);\n\t\tconst float vf_direction_disturbance = 1.0f-dot(vf_dir,vf_ref_dir);\n\t\tif(vf_speed<=1.0e-6f || (vf_speed_disturbance<0.03f && vf_direction_disturbance<0.02f)) continue;\n\t}\n\t// STREAMLINE_VELOCITY_FILTER_END\n'''
        m = re.search(r'(\n\s*)(?:const\s+)?[a-zA-Z_][a-zA-Z0-9_]*\s*=\s*[^;]*\bn\b[^;]*;', fblock)
        if m and 'STREAMLINE_VELOCITY_FILTER_BEGIN' not in fblock:
            pos = m.start(1)
            fblock = fblock[:pos] + filter_code + fblock[pos:]
            s = s[:fs] + fblock + s[fe:]

KERNEL.write_text(s, encoding="utf-8")
print("Updated streamline filtering to stop at domain faces and classify on the first disturbed sample.")
