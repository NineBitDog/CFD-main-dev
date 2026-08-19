from pathlib import Path

KERNEL = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src" / "kernel.cpp"

OLD = '''\t//draw_circle(p, 0.5f*def_streamline_sparse, 0xFFFFFF, camera_cache, bitmap, zbuffer);
\tfor(float dt=-1.0f; dt<=1.0f; dt+=2.0f) { // integrate forward and backward in time
\t\tfloat3 p0, p1=p;
\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {
\t\t\tconst uint x = (uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;
\t\t\tconst uint y = (uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;
\t\t\tconst uint z = (uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;
\t\t\tconst uxx n = (uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;
\t\t\tif(flags[n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) return;
\t\t\tconst float3 un = load3(n, u); // interpolate_u(p1, u)
\t\t\tconst float ul = length(un);
\t\t\tp0 = p1;
\t\t\tp1 += (dt/ul)*un; // integrate forward in time
\t\t\tif(def_scale_u*ul<0.1f||p1.x<-hLx||p1.x>hLx||p1.y<-hLy||p1.y>hLy||p1.z<-hLz||p1.z>hLz) break;
\t\t\tint c = 0; // coloring
\t\t\tswitch(field_mode) {
\t\t\t\tcase 0: c = colorscale_rainbow(def_scale_u*ul); break; // coloring by velocity
\t\t\t\tcase 1: c = colorscale_twocolor(0.5f+def_scale_rho*(rho[n]-1.0f)); break; // coloring by density
)+"#ifdef TEMPERATURE"+R(
\t\t\t\tcase 2: c = colorscale_iron(0.5f+def_scale_T*(T[n]-def_T_avg)); break; // coloring by temperature
)+"#endif"+R( // TEMPERATURE
\t\t\t}
\t\t\tdraw_line(p0, p1, c, camera_cache, bitmap, zbuffer);
\t\t}
\t}
'''

NEW = '''\t// A streamline is rendered only if its trajectory actually reaches a
\t// vehicle cell (TYPE_S). We trace once to test contact before drawing,
\t// because drawing during the test would leave behind rejected lines.
\tbool hit_vehicle = false;
\tfor(float dt=-1.0f; dt<=1.0f; dt+=2.0f) {
\t\tfloat3 p1=p;
\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {
\t\t\tconst uint x = (uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;
\t\t\tconst uint y = (uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;
\t\t\tconst uint z = (uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;
\t\t\tconst uxx n = (uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;
\t\t\tif(flags[n]&TYPE_S) { hit_vehicle=true; break; }
\t\t\tif(flags[n]&(TYPE_E|TYPE_I|TYPE_G)) break;
\t\t\tconst float3 un = load3(n, u); // interpolate_u(p1, u)
\t\t\tconst float ul = length(un);
\t\t\tif(ul==0.0f) break;
\t\t\tp1 += (dt/ul)*un;
\t\t\tif(def_scale_u*ul<0.1f||p1.x<-hLx||p1.x>hLx||p1.y<-hLy||p1.y>hLy||p1.z<-hLz||p1.z>hLz) break;
\t\t}
\t}
\tif(!hit_vehicle) return;

\t// Contact confirmed: trace again and render the complete streamline in
\t// both directions. Lines that never reach TYPE_S never draw any pixels.
\tfor(float dt=-1.0f; dt<=1.0f; dt+=2.0f) {
\t\tfloat3 p0, p1=p;
\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {
\t\t\tconst uint x = (uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;
\t\t\tconst uint y = (uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;
\t\t\tconst uint z = (uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;
\t\t\tconst uxx n = (uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;
\t\t\tif(flags[n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;
\t\t\tconst float3 un = load3(n, u); // interpolate_u(p1, u)
\t\t\tconst float ul = length(un);
\t\t\tif(ul==0.0f) break;
\t\t\tp0 = p1;
\t\t\tp1 += (dt/ul)*un;
\t\t\tif(def_scale_u*ul<0.1f||p1.x<-hLx||p1.x>hLx||p1.y<-hLy||p1.y>hLy||p1.z<-hLz||p1.z>hLz) break;
\t\t\tint c = 0; // coloring
\t\t\tswitch(field_mode) {
\t\t\t\tcase 0: c = colorscale_rainbow(def_scale_u*ul); break; // coloring by velocity
\t\t\t\tcase 1: c = colorscale_twocolor(0.5f+def_scale_rho*(rho[n]-1.0f)); break; // coloring by density
)+"#ifdef TEMPERATURE"+R(
\t\t\t\tcase 2: c = colorscale_iron(0.5f+def_scale_T*(T[n]-def_T_avg)); break; // coloring by temperature
)+"#endif"+R( // TEMPERATURE
\t\t\t}
\t\t\tdraw_line(p0, p1, c, camera_cache, bitmap, zbuffer);
\t\t}
\t}
'''

s = KERNEL.read_text(encoding="utf-8")
if OLD in s:
    s = s.replace(OLD, NEW, 1)
    KERNEL.write_text(s, encoding="utf-8")
    print("Applied vehicle-contact streamline filter.")
elif "bool hit_vehicle = false;" in s and "Contact confirmed: trace again" in s:
    print("Streamline contact filter already applied.")
else:
    raise RuntimeError("Native graphics_streamline block not found; kernel.cpp was not modified.")
