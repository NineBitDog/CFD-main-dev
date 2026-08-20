from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
KERNEL = ROOT / "FluidX3D-master" / "src" / "kernel.cpp"


def patch_kernel() -> bool:
    s = KERNEL.read_text(encoding="utf-8")
    original = s

    # Keep the constant-velocity equilibrium boundaries; no startup ramp.
    old_ramp = '''\tif(flagsn_bo==TYPE_E) {\n\t\trhon = rho[n];\n\t\tconst uint xcoord = coordinates(n).x;\n\t\tconst bool inlet_side = xcoord <= 1u;\n\t\tif(inlet_side) {\n\t\t\tfloat ramp = clamp((float)t/(float)1000ul, 0.0f, 1.0f);\n\t\t\t// Smoothstep startup avoids an impulse into the car at t=0.\n\t\t\tramp = ramp*ramp*(3.0f-2.0f*ramp);\n\t\t\tuxn = 0.075f*ramp;\n\t\t\tuyn = 0.0f;\n\t\t\tuzn = 0.0f;\n\t\t} else {\n\t\t\t// Constant-velocity equilibrium outlet.\n\t\t\tuxn = 0.075f;\n\t\t\tuyn = 0.0f;\n\t\t\tuzn = 0.0f;\n\t\t}\n\t} else {\n'''
    new_ramp = '''\tif(flagsn_bo==TYPE_E) {\n\t\trhon = rho[n];\n\t\tuxn = 0.075f;\n\t\tuyn = 0.0f;\n\t\tuzn = 0.0f;\n\t} else {\n'''
    if old_ramp in s:
        s = s.replace(old_ramp, new_ramp, 1)

    # Do not inject a second interpolation helper. FluidX3D already provides
    # interpolate_u() in this fork. Its signature is interpolate_u(p, u).
    duplicate_helper_start = s.find(")+R(float3 interpolate_u_streamline(")
    if duplicate_helper_start >= 0:
        duplicate_helper_end = s.find(")+R(float3 load3(", duplicate_helper_start)
        if duplicate_helper_end < 0:
            raise RuntimeError("custom interpolation helper boundary not found")
        s = s[:duplicate_helper_start] + s[duplicate_helper_end:]

    start = s.find("kernel void graphics_streamline")
    if start < 0:
        raise RuntimeError("graphics_streamline kernel declaration not found")
    body_start = s.find("{", start)
    end = s.find("\n)+R(kernel void graphics_q_field", body_start)
    if body_start < 0 or end < 0:
        raise RuntimeError("graphics_streamline kernel boundaries not found")

    body = r'''
\tconst uxx n = get_global_id(0);
\tconst float3 ps = (float3)((float)slice_x+0.5f-0.5f*(float)def_Nx, (float)slice_y+0.5f-0.5f*(float)def_Ny, (float)slice_z+0.5f-0.5f*(float)def_Nz);
)+"#ifndef D2Q9"+R(
\tconst uxx streamline_count=(uxx)(def_Nx/def_streamline_sparse)*(uxx)(def_Ny/def_streamline_sparse)*(uxx)(def_Nz/def_streamline_sparse);
\tif(n>=streamline_count) return;
\tconst uint z=(uint)(n/(uxx)((def_Nx/def_streamline_sparse)*(def_Ny/def_streamline_sparse)));
\tconst uint t=(uint)(n%(uxx)((def_Nx/def_streamline_sparse)*(def_Ny/def_streamline_sparse)));
\tconst uint y=(uint)(t/(def_Nx/def_streamline_sparse));
\tconst uint x=(uint)(t%(uxx)(def_Nx/def_streamline_sparse));
\tfloat3 p=(float)def_streamline_sparse*((float3)((float)x+0.5f,(float)y+0.5f,(float)z+0.5f))-0.5f*((float3)((float)def_Nx,(float)def_Ny,(float)def_Nz));
\tconst bool rx=fabs(p.x-ps.x)>0.5f*(float)def_streamline_sparse, ry=fabs(p.y-ps.y)>0.5f*(float)def_streamline_sparse, rz=fabs(p.z-ps.z)>0.5f*(float)def_streamline_sparse;
)+"#else"+R( // D2Q9
\tconst uxx streamline_count=(uxx)(def_Nx/def_streamline_sparse)*(uxx)(def_Ny/def_streamline_sparse);
\tif(n>=streamline_count) return;
\tconst uint y=(uint)(n/(uxx)(def_Nx/def_streamline_sparse));
\tconst uint x=(uint)(n%(uxx)(def_Nx/def_streamline_sparse));
\tfloat3 p=(float3)((float)def_streamline_sparse*((float)x+0.5f),(float)def_streamline_sparse*((float)y+0.5f),0.5f)-0.5f*((float3)((float)def_Nx,(float)def_Ny,(float)def_Nz));
\tconst bool rx=fabs(p.x-ps.x)>0.5f*(float)def_streamline_sparse, ry=fabs(p.y-ps.y)>0.5f*(float)def_streamline_sparse, rz=true;
)+"#endif"+R( // D2Q9
\tif((slice_mode==1&&rx)||(slice_mode==2&&ry)||(slice_mode==3&&rz)||(slice_mode==4&&rx&&rz)||(slice_mode==5&&rx&&ry&&rz)||(slice_mode==6&&ry&&rz)||(slice_mode==7&&rx&&ry)) return;
\tif((slice_mode==1||slice_mode==5||slice_mode==4||slice_mode==7)&&!rx) p.x=ps.x;
\tif((slice_mode==2||slice_mode==5||slice_mode==6||slice_mode==7)&&!ry) p.y=ps.y;
\tif((slice_mode==3||slice_mode==5||slice_mode==4||slice_mode==6)&&!rz) p.z=ps.z;
\tfloat camera_cache[15];
\tfor(uint i=0u;i<15u;i++) camera_cache[i]=camera[i];
\tconst float hLx=0.5f*(float)(def_Nx-2u*(def_Dx>1u)), hLy=0.5f*(float)(def_Ny-2u*(def_Dy>1u)), hLz=0.5f*(float)(def_Nz-2u*(def_Dz>1u));
\tconst float U_INF=0.075f;
\tconst float SPEED_DELTA=0.00225f; // 3% of U_INF
\tconst float DIRECTION_DELTA=0.02f;
\tconst int VEHICLE_RADIUS=12;
\tconst int VEHICLE_RADIUS2=VEHICLE_RADIUS*VEHICLE_RADIUS;

\t// FluidX3D-native velocity math is used for both streamline integration and
\t// the impact test. A segment must also be inside the 12-voxel vehicle
\t// neighborhood before it can be rendered. This rejects broad freestream and
\t// boundary-reflection disturbances that are nowhere near the vehicle.
\tfor(float dt=-1.0f;dt<=1.0f;dt+=2.0f) {
\t\tfloat3 p0=p;
\t\tfor(uint l=0u;l<def_streamline_length/2u;l++) {
\t\t\tif(p0.x<-hLx||p0.x>hLx||p0.y<-hLy||p0.y>hLy||p0.z<-hLz||p0.z>hLz) break;
\t\t\tconst float3 un=interpolate_u(p0,u);
\t\t\tconst float ul=length(un);
\t\t\tif(ul<=1.0e-6f) break;
\t\t\tconst float inv_ul=1.0f/ul;
\t\t\tconst bool impacted=fabs(ul-U_INF)>=SPEED_DELTA || (ul-un.x)>=DIRECTION_DELTA*ul;
\t\t\tconst float3 p1=p0+(dt*inv_ul)*un;
\t\t\tif(p1.x<-hLx||p1.x>hLx||p1.y<-hLy||p1.y>hLy||p1.z<-hLz||p1.z>hLz) break;

\t\t\tconst uint3 q=closest_coordinates(p0);
\t\t\tbool near_vehicle=false;
\t\t\tfor(int dz=-VEHICLE_RADIUS;dz<=VEHICLE_RADIUS&&!near_vehicle;dz++) {
\t\t\t\tconst int zz=(int)q.z+dz;
\t\t\t\tif(zz<0||zz>=(int)def_Nz) continue;
\t\t\t\tfor(int dy=-VEHICLE_RADIUS;dy<=VEHICLE_RADIUS&&!near_vehicle;dy++) {
\t\t\t\t\tconst int yy=(int)q.y+dy;
\t\t\t\t\tif(yy<0||yy>=(int)def_Ny) continue;
\t\t\t\t\tfor(int dx=-VEHICLE_RADIUS;dx<=VEHICLE_RADIUS;dx++) {
\t\t\t\t\t\tconst int dist2=dx*dx+dy*dy+dz*dz;
\t\t\t\t\t\tif(dist2>VEHICLE_RADIUS2) continue;
\t\t\t\t\t\tconst int xx=(int)q.x+dx;
\t\t\t\t\t\tif(xx<0||xx>=(int)def_Nx) continue;
\t\t\t\t\t\tconst uxx vn=(uxx)xx+(uxx)(yy+zz*def_Ny)*(uxx)def_Nx;
\t\t\t\t\t\tif((flags[vn]&TYPE_S)!=0u) { near_vehicle=true; break; }
\t\t\t\t\t}
\t\t\t\t}
\t\t\t}

\t\t\tif(impacted&&near_vehicle) {
\t\t\t\tconst uxx nn=index(q);
\t\t\t\tif(!(flags[nn]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G))) {
\t\t\t\t\tint c=0;
\t\t\t\t\tswitch(field_mode) {
\t\t\t\t\t\tcase 0: c=colorscale_rainbow(def_scale_u*ul); break;
\t\t\t\t\t\tcase 1: c=colorscale_twocolor(0.5f+def_scale_rho*(rho[nn]-1.0f)); break;
)+"#ifdef TEMPERATURE"+R(
\t\t\t\t\t\tcase 2: c=colorscale_iron(0.5f+def_scale_T*(T[nn]-def_T_avg)); break;
)+"#endif"+R(
\t\t\t\t\t}
\t\t\t\t\tdraw_line(p0,p1,c,camera_cache,bitmap,zbuffer);
\t\t\t\t}
\t\t\t}
\t\t\tp0=p1;
\t\t}
\t}
'''
    body = body.replace("\\n", "\n").replace("\\t", "\t")
    s = s[:body_start+1] + body + "\t}" + s[end:]

    if s != original:
        KERNEL.write_text(s, encoding="utf-8")
    return s != original


def main() -> None:
    patch_kernel()
    print("Applied native FluidX3D velocity math with 12-voxel vehicle proximity filtering.")


if __name__ == "__main__":
    main()
