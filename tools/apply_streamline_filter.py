from pathlib import Path

KERNEL = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src" / "kernel.cpp"

# Emit literals directly into the generated OpenCL. defines.hpp is a C++ header
# and its GRAPHICS_* names are not visible inside the OpenCL source string.
U_INF = "0.075f"
SPEED_THRESHOLD = "0.03f"
DIRECTION_THRESHOLD = "0.02f"

# Replace the complete graphics_streamline kernel. This is deliberately broader
# than the old comment-based insertion so a previously malformed kernel is
# repaired instead of having another block appended to it.
START = ')+"#ifndef TEMPERATURE"+R(\n)+R(kernel void graphics_streamline'
END = ')+"#ifndef TEMPERATURE"+R(\n)+R(kernel void graphics_q_field'

STREAMLINE = r''')+"#ifndef TEMPERATURE"+R(
)+R(kernel void graphics_streamline(const global float* camera, global int* bitmap, global int* zbuffer, const int field_mode, const int slice_mode, const int slice_x, const int slice_y, const int slice_z, const global float* rho, const global float* u, const global uchar* flags) {
)+"#else"+R( // TEMPERATURE
)+R(kernel void graphics_streamline(const global float* camera, global int* bitmap, global int* zbuffer, const int field_mode, const int slice_mode, const int slice_x, const int slice_y, const int slice_z, const global float* rho, const global float* u, const global uchar* flags, const global float* T) {
)+"#endif"+R( // TEMPERATURE
	const uxx n = get_global_id(0);
	const float3 ps = (float3)((float)slice_x+0.5f-0.5f*(float)def_Nx, (float)slice_y+0.5f-0.5f*(float)def_Ny, (float)slice_z+0.5f-0.5f*(float)def_Nz);
)+"#ifndef D2Q9"+R(
	if(n>=(uxx)(def_Nx/def_streamline_sparse)*(uxx)(def_Ny/def_streamline_sparse)*(uxx)(def_Nz/def_streamline_sparse)) return;
	const uint z = (uint)(n/(uxx)((def_Nx/def_streamline_sparse)*(def_Ny/def_streamline_sparse)));
	const uint t = (uint)(n%(uxx)((def_Nx/def_streamline_sparse)*(def_Ny/def_streamline_sparse)));
	const uint y = (uint)(t/(def_Nx/def_streamline_sparse));
	const uint x = (uint)(t%(def_Nx/def_streamline_sparse));
	float3 p = (float)def_streamline_sparse*((float3)((float)x+0.5f, (float)y+0.5f, (float)z+0.5f))-0.5f*((float3)((float)def_Nx, (float)def_Ny, (float)def_Nz));
	const bool rx=fabs(p.x-ps.x)>0.5f*(float)def_streamline_sparse, ry=fabs(p.y-ps.y)>0.5f*(float)def_streamline_sparse, rz=fabs(p.z-ps.z)>0.5f*(float)def_streamline_sparse;
)+"#else"+R( // D2Q9
	if(n>=(def_Nx/def_streamline_sparse)*(def_Ny/def_streamline_sparse)) return;
	const uint y = (uint)(n/(uxx)(def_Nx/def_streamline_sparse));
	const uint x = (uint)(n%(uxx)(def_Nx/def_streamline_sparse));
	float3 p = ((float3)((float)def_streamline_sparse*((float)x+0.5f), (float)def_streamline_sparse*((float)y+0.5f), 0.5f))-0.5f*((float3)((float)def_Nx, (float)def_Ny, (float)def_Nz));
	const bool rx=fabs(p.x-ps.x)>0.5f*(float)def_streamline_sparse, ry=fabs(p.y-ps.y)>0.5f*(float)def_streamline_sparse, rz=true;
)+"#endif"+R( // D2Q9
	if((slice_mode==1&&rx)||(slice_mode==2&&ry)||(slice_mode==3&&rz)||(slice_mode==4&&rx&&rz)||(slice_mode==5&&rx&&ry&&rz)||(slice_mode==6&&ry&&rz)||(slice_mode==7&&rx&&ry)) return;
	if((slice_mode==1||slice_mode==5||slice_mode==4||slice_mode==7)&!rx) p.x = ps.x;
	if((slice_mode==2||slice_mode==5||slice_mode==6||slice_mode==7)&!ry) p.y = ps.y;
	if((slice_mode==3||slice_mode==5||slice_mode==4||slice_mode==6)&!rz) p.z = ps.z;
	float camera_cache[15];
	for(uint i=0u; i<15u; i++) camera_cache[i] = camera[i];
	const float hLx=0.5f*(float)(def_Nx-2u*(def_Dx>1u)), hLy=0.5f*(float)(def_Ny-2u*(def_Dy>1u)), hLz=0.5f*(float)(def_Nz-2u*(def_Dz>1u));

	// First pass: classify the complete trajectory from the velocity field.
	// A streamline is accepted when either its speed differs from freestream
	// by at least 3%, or its direction differs by at least 0.02 radians-ish
	// in the dot-product metric. No pixels are drawn during this pass.
	bool affected = false;
	const float U_inf = 0.075f;
	const float speed_threshold = 0.03f;
	const float direction_threshold = 0.02f;
	const float3 freestream_dir = (float3)(1.0f, 0.0f, 0.0f);

	for(float dt=-1.0f; dt<=1.0f && !affected; dt+=2.0f) {
		float3 p1=p;
		for(uint l=0u; l<def_streamline_length/2u; l++) {
			const uint xx=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;
			const uint yy=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;
			const uint zz=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;
			const uxx nn=(uxx)xx+(uxx)(yy+zz*def_Ny)*(uxx)def_Nx;
			if(flags[nn]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;
			const float3 un=load3(nn,u);
			const float ul=length(un);
			if(ul<=0.0f) break;
			const float speed_disturbance=fabs(ul-U_inf)/fmax(U_inf,1.0e-6f);
			const float3 flow_dir=un/fmax(ul,1.0e-6f);
			const float direction_disturbance=1.0f-dot(flow_dir,freestream_dir);
			if(speed_disturbance>=speed_threshold || direction_disturbance>=direction_threshold) {
				affected=true;
				break;
			}
			p1+=(dt/ul)*un;
			if(def_scale_u*ul<0.1f || p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;
		}
	}
	if(!affected) return;

	// Second pass: only affected trajectories are actually drawn.
	for(float dt=-1.0f; dt<=1.0f; dt+=2.0f) {
		float3 p0, p1=p;
		for(uint l=0u; l<def_streamline_length/2u; l++) {
			const uint xx=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;
			const uint yy=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;
			const uint zz=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;
			const uxx nn=(uxx)xx+(uxx)(yy+zz*def_Ny)*(uxx)def_Nx;
			if(flags[nn]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;
			const float3 un=load3(nn,u);
			const float ul=length(un);
			if(ul<=0.0f) break;
			p0=p1;
			p1+=(dt/ul)*un;
			if(def_scale_u*ul<0.1f || p1.x<-hLx || p1.x>hLx || p1.y<-hLy || p1.y>hLy || p1.z<-hLz || p1.z>hLz) break;
			int c=0;
			switch(field_mode) {
				case 0: c=colorscale_rainbow(def_scale_u*ul); break;
				case 1: c=colorscale_twocolor(0.5f+def_scale_rho*(rho[nn]-1.0f)); break;
)+"#ifdef TEMPERATURE"+R(
				case 2: c=colorscale_iron(0.5f+def_scale_T*(T[nn]-def_T_avg)); break;
)+"#endif"+R( // TEMPERATURE
			}
			draw_line(p0,p1,c,camera_cache,bitmap,zbuffer);
		}
	}
}

'''

FIELD_NEW = r'''\t\tcase 0: // coloring by velocity, disturbed flow only
\t\t\twhile(traversed_cells<Nx+Ny+Nz) {
\t\t\t\tif(tmx<tmy) { if(tmx<tmz) { xyz.x += dx; tmx += tdx; } else { xyz.z += dz; tmz += tdz; } }
\t\t\t\telse /****/ { if(tmy<tmz) { xyz.y += dy; tmy += tdy; } else { xyz.z += dz; tmz += tdz; } }
\t\t\t\tif(xyz.x<0 || xyz.y<0 || xyz.z<0 || xyz.x>=(int)Nx || xyz.y>=(int)Ny || xyz.z>=(int)Nz) break;
\t\t\t\tconst uxx n = index((uint3)((uint)clamp(xyz.x, 0, (int)Nx-1), (uint)clamp(xyz.y, 0, (int)Ny-1), (uint)clamp(xyz.z, 0, (int)Nz-1)));
\t\t\t\tif(!(flags[n]&(TYPE_S|TYPE_E|TYPE_G))) {
\t\t\t\t\tconst float3 vel=load3(n,u);
\t\t\t\t\tconst float speed=length(vel);
\t\t\t\t\tconst float speed_disturbance=fabs(speed-0.075f)/fmax(0.075f,1.0e-6f);
\t\t\t\t\tconst float3 flow_dir=vel/fmax(speed,1.0e-6f);
\t\t\t\t\tconst float direction_disturbance=1.0f-dot(flow_dir,(float3)(1.0f,0.0f,0.0f));
\t\t\t\t\tif(speed_disturbance>=0.03f || direction_disturbance>=0.02f) {
\t\t\t\t\t\tconst float weight=fmin(speed,fabs(speed-0.5f/def_scale_u));
\t\t\t\t\t\tsum=fma(weight,speed,sum);
\t\t\t\t\t\ttraversed_cells_weighted+=weight;
\t\t\t\t\t}
\t\t\t\t}
\t\t\t\ttraversed_cells++;
\t\t\t}
\t\t\tif(traversed_cells_weighted<=0.0f) return background_color;
\t\t\tcolor=colorscale_rainbow(def_scale_u*sum/traversed_cells_weighted);
\t\t\ttraversed_cells_weighted*=2.0f*def_scale_u;
\t\t\tbreak;'''
FIELD_NEW = FIELD_NEW.replace('\\t', '\t').replace('\\n', '\n')

s = KERNEL.read_text(encoding="utf-8")

# Repair identifiers left by older versions of the patcher.
s = s.replace("GRAPHICS_STREAMLINE_U_INF", U_INF)
s = s.replace("GRAPHICS_STREAMLINE_SPEED_THRESHOLD", SPEED_THRESHOLD)
s = s.replace("GRAPHICS_STREAMLINE_DIRECTION_THRESHOLD", DIRECTION_THRESHOLD)

start = s.find(START)
if start < 0:
    raise RuntimeError("graphics_streamline start marker not found")
end = s.find(END, start)
if end < 0:
    raise RuntimeError("graphics_streamline end marker not found")

# Replace the entire kernel, including any previously malformed braces or
# duplicate filter passes. This leaves the following graphics_q_field kernel
# untouched.
s = s[:start] + STREAMLINE + s[end:]

# Patch the native ray-marched colored velocity volume. If it is already
# patched, leave it alone; otherwise replace only the velocity case.
ray_start = s.find('R(int ray_grid_traverse_sum')
case0 = s.find('\t\tcase 0: // coloring by velocity', ray_start)
case1 = s.find('\t\tcase 1: // coloring by density', case0)
if ray_start >= 0 and case0 >= 0 and case1 > case0:
    existing = s[case0:case1]
    if 'disturbed flow only' not in existing:
        s = s[:case0] + FIELD_NEW + '\n' + s[case1:]

KERNEL.write_text(s, encoding="utf-8")
