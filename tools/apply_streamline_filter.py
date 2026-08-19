from pathlib import Path

KERNEL = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src" / "kernel.cpp"

# Replace the native graphics_streamline kernel body with a two-pass
# velocity-field filter. A streamline is rendered only if, anywhere along
# its forward/backward path, either its speed differs sufficiently from the
# freestream speed or its direction differs sufficiently from freestream.
# Locate the kernel by its declaration instead of a comment that may already
# have been removed by an earlier generated-kernel patch.
START = ')+R(kernel void graphics_streamline'
END = ')+R(kernel void graphics_q_field'

NEW = '''\t// First pass: classify the streamline from the velocity field only.
\t// Freestream is +X for this vehicle setup. A line is affected if any
\t// point has a speed disturbance or direction change above threshold.
\tbool affected = false;
\tconst float U_inf = GRAPHICS_STREAMLINE_U_INF;
\tconst float speed_threshold = GRAPHICS_STREAMLINE_SPEED_THRESHOLD;
\tconst float direction_threshold = GRAPHICS_STREAMLINE_DIRECTION_THRESHOLD;
\tconst float3 freestream_dir = (float3)(1.0f, 0.0f, 0.0f);

\tfor(float dt=-1.0f; dt<=1.0f && !affected; dt+=2.0f) {
\t\tfloat3 p1=p;
\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {
\t\t\tconst uint x=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;
\t\t\tconst uint y=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;
\t\t\tconst uint z=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;
\t\t\tconst uxx n=(uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;
\t\t\tif(flags[n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;

\t\t\tconst float3 un=load3(n,u);
\t\t\tconst float ul=length(un);
\t\t\tif(ul<=0.0f) { affected=true; break; }

\t\t\tconst float speed_disturbance=fabs(ul-U_inf)/fmax(U_inf,1.0e-6f);
\t\t\tconst float3 flow_dir=un/fmax(ul,1.0e-6f);
\t\t\tconst float direction_disturbance=1.0f-dot(flow_dir,freestream_dir);
\t\t\tif(speed_disturbance>=speed_threshold || direction_disturbance>=direction_threshold) {
\t\t\t\taffected=true;
\t\t\t\tbreak;
\t\t\t}

\t\t\tp1+=(dt/ul)*un;
\t\t\tif(def_scale_u*ul<0.1f || p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;
\t\t}
\t}
\tif(!affected) return;

\t// Second pass: only affected streamlines are rendered, using the original
\t// velocity/density/temperature coloring and full forward/backward path.
\tfor(float dt=-1.0f; dt<=1.0f; dt+=2.0f) {
\t\tfloat3 p0, p1=p;
\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {
\t\t\tconst uint x=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;
\t\t\tconst uint y=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;
\t\t\tconst uint z=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;
\t\t\tconst uxx n=(uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;
\t\t\tif(flags[n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;
\t\t\tconst float3 un=load3(n,u);
\t\t\tconst float ul=length(un);
\t\t\tif(ul<=0.0f) break;
\t\t\tp0=p1;
\t\t\tp1+=(dt/ul)*un;
\t\t\tif(def_scale_u*ul<0.1f || p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;
\t\t\tint c=0;
\t\t\tswitch(field_mode) {
\t\t\t\tcase 0: c=colorscale_rainbow(def_scale_u*ul); break;
\t\t\t\tcase 1: c=colorscale_twocolor(0.5f+def_scale_rho*(rho[n]-1.0f)); break;
)+"#ifdef TEMPERATURE"+R(
\t\t\t\tcase 2: c=colorscale_iron(0.5f+def_scale_T*(T[n]-def_T_avg)); break;
)+"#endif"+R(
\t\t\t}
\t\t\tdraw_line(p0,p1,c,camera_cache,bitmap,zbuffer);
\t\t}
\t}
'''

s = KERNEL.read_text(encoding="utf-8")
kstart = s.find(START)
if kstart < 0:
    raise RuntimeError("graphics_streamline kernel declaration not found")
body_start = s.find("{", kstart)
if body_start < 0:
    raise RuntimeError("graphics_streamline kernel body start not found")
end = s.find(END, body_start)
if end < 0:
    raise RuntimeError("graphics_streamline kernel end marker not found")

# Preserve the exact current kernel signature and replace only its body.
s = s[:body_start+1] + "\n" + NEW + "\t" + "}\n" + s[end:]
KERNEL.write_text(s, encoding="utf-8")
