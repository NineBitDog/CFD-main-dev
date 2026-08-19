from pathlib import Path

KERNEL = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src" / "kernel.cpp"

U_INF = "0.075f"
SPEED_THRESHOLD = "0.03f"
DIRECTION_THRESHOLD = "0.02f"

CURRENT_MARKER = "\t// A streamline is rendered only if its trajectory actually reaches a\n"
ALREADY_PATCHED = "\t// First pass: classify the streamline from the velocity field only."

CLASSIFY = r'''\t// First pass: classify the streamline from the velocity field only.
\t// Freestream is +X. Render only if speed or direction is disturbed.
\tbool affected = false;
\tconst float U_inf = __U_INF__;
\tconst float speed_threshold = __SPEED_THRESHOLD__;
\tconst float direction_threshold = __DIRECTION_THRESHOLD__;
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
\t\t\tif(ul<=0.0f) break;
\t\t\tconst float speed_disturbance=fabs(ul-__U_INF__)/fmax(__U_INF__,1.0e-6f);
\t\t\tconst float3 flow_dir=un/fmax(ul,1.0e-6f);
\t\t\tconst float direction_disturbance=1.0f-dot(flow_dir,freestream_dir);
\t\t\tif(speed_disturbance>=__SPEED_THRESHOLD__ || direction_disturbance>=__DIRECTION_THRESHOLD__) {
\t\t\t\taffected=true;
\t\t\t\tbreak;
\t\t\t}
\t\t\tp1+=(dt/ul)*un;
\t\t\tif(def_scale_u*ul<0.1f || p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;
\t\t}
\t}
\tif(!affected) return;

'''
CLASSIFY = (CLASSIFY
    .replace("__U_INF__", U_INF)
    .replace("__SPEED_THRESHOLD__", SPEED_THRESHOLD)
    .replace("__DIRECTION_THRESHOLD__", DIRECTION_THRESHOLD))

s = KERNEL.read_text(encoding="utf-8")

# Always sanitize identifiers emitted by older versions of this patcher.
s = s.replace("GRAPHICS_STREAMLINE_U_INF", U_INF)
s = s.replace("GRAPHICS_STREAMLINE_SPEED_THRESHOLD", SPEED_THRESHOLD)
s = s.replace("GRAPHICS_STREAMLINE_DIRECTION_THRESHOLD", DIRECTION_THRESHOLD)

if ALREADY_PATCHED not in s:
    pos = s.find(CURRENT_MARKER)
    if pos < 0:
        raise RuntimeError("graphics_streamline insertion point not found")
    s = s[:pos] + CURRENT_MARKER + CLASSIFY + s[pos + len(CURRENT_MARKER):]

KERNEL.write_text(s, encoding="utf-8")
