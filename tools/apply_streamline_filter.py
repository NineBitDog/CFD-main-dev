from pathlib import Path

KERNEL = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src" / "kernel.cpp"

# Replace the native graphics_streamline rendering block with a two-pass
# velocity-field filter. A streamline is rendered only if, anywhere along
# its forward/backward path, either its speed differs sufficiently from the
# freestream speed or its direction differs sufficiently from freestream.
START = "\t//draw_circle(p, 0.5f*def_streamline_sparse, 0xFFFFFF, camera_cache, bitmap, zbuffer);"
END = "\n)+R(kernel void graphics_q_field"

NEW = '''\t// First pass: classify the streamline from the velocity field only.\n\t// Freestream is +X for this vehicle setup. A line is affected if any\n\t// point has a speed disturbance or direction change above threshold.\n\tbool affected = false;\n\tconst float U_inf = GRAPHICS_STREAMLINE_U_INF;\n\tconst float speed_threshold = GRAPHICS_STREAMLINE_SPEED_THRESHOLD;\n\tconst float direction_threshold = GRAPHICS_STREAMLINE_DIRECTION_THRESHOLD;\n\tconst float3 freestream_dir = (float3)(1.0f, 0.0f, 0.0f);\n\n\tfor(float dt=-1.0f; dt<=1.0f && !affected; dt+=2.0f) {\n\t\tfloat3 p1=p;\n\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {\n\t\t\tconst uint x=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;\n\t\t\tconst uint y=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;\n\t\t\tconst uint z=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;\n\t\t\tconst uxx n=(uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;\n\t\t\tif(flags[n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;\n\n\t\t\tconst float3 un=load3(n,u);\n\t\t\tconst float ul=length(un);\n\t\t\tif(ul<=0.0f) { affected=true; break; }\n\n\t\t\tconst float speed_disturbance=fabs(ul-U_inf)/fmax(U_inf,1.0e-6f);\n\t\t\tconst float3 flow_dir=un/fmax(ul,1.0e-6f);\n\t\t\tconst float direction_disturbance=1.0f-dot(flow_dir,freestream_dir);\n\t\t\tif(speed_disturbance>=speed_threshold || direction_disturbance>=direction_threshold) {\n\t\t\t\taffected=true;\n\t\t\t\tbreak;\n\t\t\t}\n\n\t\t\tp1+=(dt/ul)*un;\n\t\t\tif(def_scale_u*ul<0.1f || p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;\n\t\t}\n\t}\n\tif(!affected) return;\n\n\t// Second pass: only affected streamlines are rendered, using the original\n\t// velocity/density/temperature coloring and full forward/backward path.\n\tfor(float dt=-1.0f; dt<=1.0f; dt+=2.0f) {\n\t\tfloat3 p0, p1=p;\n\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {\n\t\t\tconst uint x=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;\n\t\t\tconst uint y=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;\n\t\t\tconst uint z=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;\n\t\t\tconst uxx n=(uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;\n\t\t\tif(flags[n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;\n\t\t\tconst float3 un=load3(n,u);\n\t\t\tconst float ul=length(un);\n\t\t\tif(ul<=0.0f) break;\n\t\t\tp0=p1;\n\t\t\tp1+=(dt/ul)*un;\n\t\t\tif(def_scale_u*ul<0.1f || p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;\n\t\t\tint c=0;\n\t\t\tswitch(field_mode) {\n\t\t\t\tcase 0: c=colorscale_rainbow(def_scale_u*ul); break;\n\t\t\t\tcase 1: c=colorscale_twocolor(0.5f+def_scale_rho*(rho[n]-1.0f)); break;\n)+"#ifdef TEMPERATURE"+R(\n\t\t\t\tcase 2: c=colorscale_iron(0.5f+def_scale_T*(T[n]-def_T_avg)); break;\n)+"#endif"+R(\n\t\t\t}\n\t\t\tdraw_line(p0,p1,c,camera_cache,bitmap,zbuffer);\n\t\t}\n\t}\n'''

s = KERNEL.read_text(encoding="utf-8")
start = s.find(START)
if start < 0:
    raise RuntimeError("graphics_streamline rendering block not found")
end = s.find(END, start)
if end < 0:
    raise RuntimeError("graphics_streamline end marker not found")
# Keep the kernel boundary marker itself; replace only the streamline body.
s = s[:start] + NEW + s[end:]
KERNEL.write_text(s, encoding="utf-8")
