from pathlib import Path
import re

KERNEL = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src" / "kernel.cpp"

# These are fallback values only. The actual freestream is sampled from the
# inlet plane for each streamline, so non-default inlet speeds/profiles work.
FALLBACK_U_INF = "0.075f"
SPEED_THRESHOLD = "0.03f"
DIRECTION_THRESHOLD = "0.02f"

START = ')+"#ifndef TEMPERATURE"+R(\n)+R(kernel void graphics_streamline'
END = ')+"#ifndef TEMPERATURE"+R(\n)+R(kernel void graphics_q_field'

s = KERNEL.read_text(encoding="utf-8")

# Clean up identifiers left by older versions of this patcher. These are
# replaced with literals because the generated OpenCL string does not see
# defines.hpp preprocessor symbols.
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

# Replace only the classification pass. The actual rendering pass is left
# untouched, preserving the normal FluidX3D streamline colors and tracing.
new_classification = r'''\t// First pass: classify the complete trajectory from the actual inlet flow.\n\t// The inlet velocity is sampled at the same y/z position as the seed.\n\t// This prevents normal freestream flow from being rendered merely because\n\t// the simulation's nominal velocity differs from a hard-coded value.\n\tbool affected = false;\n\tconst float fallback_U_inf = 0.075f;\n\tconst float speed_threshold = 0.03f;\n\tconst float direction_threshold = 0.02f;\n\n\tint ref_y = (int)(p.y + 0.5f*(float)def_Ny);\n\tint ref_z = (int)(p.z + 0.5f*(float)def_Nz);\n\tref_y = clamp(ref_y, 0, (int)def_Ny-1);\n\tref_z = clamp(ref_z, 0, (int)def_Nz-1);\n\n\tfloat3 freestream = (float3)(fallback_U_inf, 0.0f, 0.0f);\n\tbool found_inlet = false;\n\tfor(uint ix=1u; ix<min(8u, (uint)def_Nx) && !found_inlet; ix++) {\n\t\tconst uxx ref_n=(uxx)ix+(uxx)((uint)ref_y+(uint)ref_z*def_Ny)*(uxx)def_Nx;\n\t\tif(!(flags[ref_n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G))) {\n\t\t\tconst float3 ref_u=load3(ref_n,u);\n\t\t\tif(length(ref_u)>1.0e-6f) {\n\t\t\t\tfreestream=ref_u;\n\t\t\t\tfound_inlet=true;\n\t\t\t}\n\t\t}\n\t}\n\tconst float freestream_speed=length(freestream);\n\tconst float3 freestream_dir=freestream/fmax(freestream_speed,1.0e-6f);\n\n\tfor(float dt=-1.0f; dt<=1.0f && !affected; dt+=2.0f) {\n\t\tfloat3 p1=p;\n\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {\n\t\t\tconst uint xx=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;\n\t\t\tconst uint yy=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;\n\t\t\tconst uint zz=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;\n\t\t\tconst uxx nn=(uxx)xx+(uxx)(yy+zz*def_Ny)*(uxx)def_Nx;\n\t\t\tif(flags[nn]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;\n\t\t\tconst float3 un=load3(nn,u);\n\t\t\tconst float ul=length(un);\n\t\t\tif(ul<=1.0e-6f) break;\n\t\t\tconst float speed_disturbance=fabs(ul-freestream_speed)/fmax(freestream_speed,1.0e-6f);\n\t\t\tconst float3 flow_dir=un/fmax(ul,1.0e-6f);\n\t\t\tconst float direction_disturbance=1.0f-dot(flow_dir,freestream_dir);\n\t\t\tif(speed_disturbance>=speed_threshold || direction_disturbance>=direction_threshold) {\n\t\t\t\taffected=true;\n\t\t\t\tbreak;\n\t\t\t}\n\t\t\tp1+=(dt/ul)*un;\n\t\t\tif(def_scale_u*ul<0.1f || p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;\n\t\t}\n\t}\n\tif(!affected) return;\n\n'''
new_classification = new_classification.replace("\\t", "\t").replace("\\n", "\n")

pattern = r"\t// First pass: classify.*?\n\tif\(!affected\) return;\n\n"
block2, count = re.subn(pattern, new_classification, block, count=1, flags=re.S)
if count != 1:
    raise RuntimeError("existing streamline classification block not found")

s = s[:start] + block2 + s[end:]
KERNEL.write_text(s, encoding="utf-8")

print("Updated graphics_streamline: disturbance is now measured against the actual inlet velocity profile.")
