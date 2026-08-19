from pathlib import Path

KERNEL = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src" / "kernel.cpp"

# Replace only the body of graphics_streamline. The function signature must
# remain intact because it provides p, camera_cache, bitmap, zbuffer, u, flags,
# field_mode, and the other kernel parameters.
START = "kernel void graphics_streamline"
END = "\n)+R(kernel void graphics_q_field"

# Keep the original working velocity-field-only behavior. Use literals here so
# the generated OpenCL never depends on optional GRAPHICS_STREAMLINE_* macros.
NEW = '''
	// First pass: classify the streamline from the velocity field only.
	// Freestream is +X for this vehicle setup. A line is affected if any
	// point has a speed disturbance or direction change above threshold.
	bool affected = false;
	const float U_inf = 0.075f;
	const float speed_threshold = 0.03f;
	const float direction_threshold = 0.02f;
	const float3 freestream_dir = (float3)(1.0f, 0.0f, 0.0f);

	for(float dt=-1.0f; dt<=1.0f && !affected; dt+=2.0f) {
		float3 p1=p;
		for(uint l=0u; l<def_streamline_length/2u; l++) {
			const uint x=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;
			const uint y=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;
			const uint z=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;
			const uxx n=(uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;
			if(flags[n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;

			const float3 un=load3(n,u);
			const float ul=length(un);
			if(ul<=0.0f) { affected=true; break; }

			const float speed_disturbance=fabs(ul-U_inf)/fmax(U_inf,1.0e-6f);
			const float3 flow_dir=un/fmax(ul,1.0e-6f);
			const float direction_disturbance=1.0f-dot(flow_dir,freestream_dir);
			if(speed_disturbance>=speed_threshold || direction_disturbance>=direction_threshold) {
				affected=true;
				break;
			}

			p1+=(dt/ul)*un;
			if(def_scale_u*ul<0.1f) break;
		}
	}
	if(!affected) return;

	// Second pass: render only affected streamlines using the original
	// velocity/density/temperature coloring and full forward/backward path.
	for(float dt=-1.0f; dt<=1.0f; dt+=2.0f) {
		float3 p0, p1=p;
		for(uint l=0u; l<def_streamline_length/2u; l++) {
			const uint x=(uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;
			const uint y=(uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;
			const uint z=(uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;
			const uxx n=(uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;
			if(flags[n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;
			const float3 un=load3(n,u);
			const float ul=length(un);
			if(ul<=0.0f) break;
			p0=p1;
			p1+=(dt/ul)*un;
			if(def_scale_u*ul<0.1f) break;
			int c=0;
			switch(field_mode) {
				case 0: c=colorscale_rainbow(def_scale_u*ul); break;
				case 1: c=colorscale_twocolor(0.5f+def_scale_rho*(rho[n]-1.0f)); break;
)+"#ifdef TEMPERATURE"+R(
				case 2: c=colorscale_iron(0.5f+def_scale_T*(T[n]-def_T_avg)); break;
)+"#endif"+R(
			}
			draw_line(p0,p1,c,camera_cache,bitmap,zbuffer);
		}
	}
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

# Keep the exact current kernel signature and generated-string boundary.
s = s[:body_start+1] + NEW + "\t}" + s[end:]
KERNEL.write_text(s, encoding="utf-8")
