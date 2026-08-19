from pathlib import Path

KERNEL = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src" / "kernel.cpp"

# Replace the complete graphics_streamline function body while preserving its
# exact kernel signature and the setup locals used by the native renderer.
START = "kernel void graphics_streamline"
END = "\n)+R(kernel void graphics_q_field"

NEW = '''
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

\t// First pass: classify the streamline from the velocity field only.
\t// This is the original working behavior: compare against freestream and
\t// reject uniform-flow trajectories before any draw_line() call.
\tbool affected = false;
\tconst float U_inf = 0.075f;
\tconst float speed_threshold = 0.03f;
\tconst float direction_threshold = 0.02f;
\tconst float3 freestream_dir = (float3)(1.0f, 0.0f, 0.0f);

\tfor(float dt=-1.0f; dt<=1.0f && !affected; dt+=2.0f) {
\t\tfloat3 p1=p;
\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {
\t\t\tif(p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;
\t\t\tconst uint x1=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;
\t\t\tconst uint y1=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;
\t\t\tconst uint z1=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;
\t\t\tconst uxx nn=(uxx)x1+(uxx)(y1+z1*def_Ny)*(uxx)def_Nx;
\t\t\tif(flags[nn]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;
\t\t\tconst float3 un=load3(nn,u);
\t\t\tconst float ul=length(un);
\t\t\tif(ul<=1.0e-6f) break;
\t\t\tconst float speed_disturbance=fabs(ul-U_inf)/fmax(U_inf,1.0e-6f);
\t\t\tconst float3 flow_dir=un/fmax(ul,1.0e-6f);
\t\t\tconst float direction_disturbance=1.0f-dot(flow_dir,freestream_dir);
\t\t\tif(speed_disturbance>=speed_threshold || direction_disturbance>=direction_threshold) { affected=true; break; }
\t\t\tconst float3 next_p1=p1+(dt/ul)*un;
\t\t\tif(next_p1.x<-hLx || next_p1.x>hLx || next_p1.y<-hLy || next_p1.y>hLy || next_p1.z<-hLz || next_p1.z>hLz) break;
\t\t\tp1=next_p1;
\t\t\tif(def_scale_u*ul<0.1f) break;
\t\t}
\t}
\tif(!affected) return;

\t// Second pass: render only affected trajectories with the original native
\t// FluidX3D coloring and full forward/backward streamline path.
\tfor(float dt=-1.0f; dt<=1.0f; dt+=2.0f) {
\t\tfloat3 p0, p1=p;
\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {
\t\t\tif(p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;
\t\t\tconst uint x1=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;
\t\t\tconst uint y1=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;
\t\t\tconst uint z1=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;
\t\t\tconst uxx nn=(uxx)x1+(uxx)(y1+z1*def_Ny)*(uxx)def_Nx;
\t\t\tif(flags[nn]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;
\t\t\tconst float3 un=load3(nn,u);
\t\t\tconst float ul=length(un);
\t\t\tif(ul<=1.0e-6f) break;
\t\t\tp0=p1;
\t\t\tconst float3 next_p1=p1+(dt/ul)*un;
\t\t\tif(next_p1.x<-hLx || next_p1.x>hLx || next_p1.y<-hLy || next_p1.y>hLy || next_p1.z<-hLz || next_p1.z>hLz) break;
\t\t\tp1=next_p1;
\t\t\tint c=0;
\t\t\tswitch(field_mode) {
\t\t\t\tcase 0: c=colorscale_rainbow(def_scale_u*ul); break;
\t\t\t\tcase 1: c=colorscale_twocolor(0.5f+def_scale_rho*(rho[nn]-1.0f)); break;
)+"#ifdef TEMPERATURE"+R(
\t\t\t\tcase 2: c=colorscale_iron(0.5f+def_scale_T*(T[nn]-def_T_avg)); break;
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

# Preserve the signature and replace only its OpenCL body.
s = s[:body_start+1] + NEW + "\t}" + s[end:]
KERNEL.write_text(s, encoding="utf-8")
