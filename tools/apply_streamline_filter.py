from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
KERNEL = ROOT / "FluidX3D-master" / "src" / "kernel.cpp"


def patch_kernel() -> bool:
    s = KERNEL.read_text(encoding="utf-8")
    original = s

    # Remove the old inlet startup ramp from the generated kernel.
    old_ramp = '''\tif(flagsn_bo==TYPE_E) {\n\t\trhon = rho[n];\n\t\tconst uint xcoord = coordinates(n).x;\n\t\tconst bool inlet_side = xcoord <= 1u;\n\t\tif(inlet_side) {\n\t\t\tfloat ramp = clamp((float)t/(float)1000ul, 0.0f, 1.0f);\n\t\t\t// Smoothstep startup avoids an impulse into the car at t=0.\n\t\t\tramp = ramp*ramp*(3.0f-2.0f*ramp);\n\t\t\tuxn = 0.075f*ramp;\n\t\t\tuyn = 0.0f;\n\t\t\tuzn = 0.0f;\n\t\t} else {\n\t\t\t// Constant-velocity equilibrium outlet.\n\t\t\tuxn = 0.075f;\n\t\t\tuyn = 0.0f;\n\t\t\tuzn = 0.0f;\n\t\t}\n\t} else {\n'''
    new_ramp = '''\tif(flagsn_bo==TYPE_E) {\n\t\trhon = rho[n];\n\t\tuxn = 0.075f;\n\t\tuyn = 0.0f;\n\t\tuzn = 0.0f;\n\t} else {\n'''
    if old_ramp in s:
        s = s.replace(old_ramp, new_ramp, 1)

    # Never inject a second interpolation helper. Use FluidX3D's native helper.
    helper_start = s.find(")+R(float3 interpolate_u_streamline(")
    if helper_start >= 0:
        helper_end = s.find(")+R(float3 load3(", helper_start)
        if helper_end < 0:
            raise RuntimeError("custom interpolation helper boundary not found")
        s = s[:helper_start] + s[helper_end:]

    start = s.find("kernel void graphics_streamline")
    if start < 0:
        raise RuntimeError("graphics_streamline kernel declaration not found")
    body_start = s.find("{", start)
    end = s.find("\n)+R(kernel void graphics_q_field", body_start)
    if body_start < 0 or end < 0:
        raise RuntimeError("graphics_streamline kernel boundaries not found")

    body = r'''
\tconst uxx n=get_global_id(0);
\t// Deterministically render half as many seeds.
\tif((n&1u)!=0u) return;
\tconst float3 ps=(float3)((float)slice_x+0.5f-0.5f*(float)def_Nx,(float)slice_y+0.5f-0.5f*(float)def_Ny,(float)slice_z+0.5f-0.5f*(float)def_Nz);
)+"#ifndef D2Q9"+R(
\tconst uxx sx=(uxx)(def_Nx/def_streamline_sparse);
\tconst uxx sy=(uxx)(def_Ny/def_streamline_sparse);
\tconst uxx sz=(uxx)(def_Nz/def_streamline_sparse);
\tconst uxx streamlines=sx*sy*sz;
\tif(n>=streamlines) return;
\tconst uxx xy=sx*sy;
\tconst uint z=(uint)(n/xy);
\tconst uxx rem=n%(xy);
\tconst uint y=(uint)(rem/sx);
\tconst uint x=(uint)(rem%sx);
\tfloat3 p=(float)def_streamline_sparse*((float3)((float)x+0.5f,(float)y+0.5f,(float)z+0.5f))-0.5f*((float3)((float)def_Nx,(float)def_Ny,(float)def_Nz));
\tconst bool rx=fabs(p.x-ps.x)>0.5f*(float)def_streamline_sparse,ry=fabs(p.y-ps.y)>0.5f*(float)def_streamline_sparse,rz=fabs(p.z-ps.z)>0.5f*(float)def_streamline_sparse;
)+"#else"+R(
\tconst uxx sx=(uxx)(def_Nx/def_streamline_sparse);
\tconst uxx sy=(uxx)(def_Ny/def_streamline_sparse);
\tconst uxx streamlines=sx*sy;
\tif(n>=streamlines) return;
\tconst uint y=(uint)(n/sx);
\tconst uint x=(uint)(n%sx);
\tfloat3 p=(float3)((float)def_streamline_sparse*((float)x+0.5f),(float)def_streamline_sparse*((float)y+0.5f),0.5f)-0.5f*((float3)((float)def_Nx,(float)def_Ny,(float)def_Nz));
\tconst bool rx=fabs(p.x-ps.x)>0.5f*(float)def_streamline_sparse,ry=fabs(p.y-ps.y)>0.5f*(float)def_streamline_sparse,rz=true;
)+"#endif"+R(
\tif((slice_mode==1&&rx)||(slice_mode==2&&ry)||(slice_mode==3&&rz)||(slice_mode==4&&rx&&rz)||(slice_mode==5&&rx&&ry&&rz)||(slice_mode==6&&ry&&rz)||(slice_mode==7&&rx&&ry)) return;
\tif((slice_mode==1||slice_mode==4||slice_mode==5||slice_mode==7)&&!rx) p.x=ps.x;
\tif((slice_mode==2||slice_mode==5||slice_mode==6||slice_mode==7)&&!ry) p.y=ps.y;
\tif((slice_mode==3||slice_mode==4||slice_mode==5||slice_mode==6)&&!rz) p.z=ps.z;

\tfloat camera_cache[15];
\tfor(uint i=0u;i<15u;i++) camera_cache[i]=camera[i];
\tconst float hLx=0.5f*(float)(def_Nx-2u*(def_Dx>1u));
\tconst float hLy=0.5f*(float)(def_Ny-2u*(def_Dy>1u));
\tconst float hLz=0.5f*(float)(def_Nz-2u*(def_Dz>1u));
\tconst float U_INF=0.075f;
\tconst float SPEED_DELTA=0.00225f; // 3% of U_INF
\tconst float DIRECTION_DOT_MAX=0.98f;
\tconst int VEHICLE_RADIUS=12;
\tconst int VEHICLE_RADIUS2=144;

\t// One pass per direction. The cheap velocity test runs first; the expensive
\t// 12-voxel vehicle scan is only performed when a sample is actually disturbed.
\t// This removes the previous O(streamline_length * 12^3) work for freestream.
\tfor(float dt=-1.0f;dt<=1.0f;dt+=2.0f) {
\t\tfloat3 p0=p;
\t\tfloat3 pair_start=p0;
\t\tbool pair_disturbed=false;
\t\tbool vehicle_affected=false;
\t\tfor(uint l=0u;l<def_streamline_length/2u;l++) {
\t\t\tif(p0.x<-hLx||p0.x>hLx||p0.y<-hLy||p0.y>hLy||p0.z<-hLz||p0.z>hLz) break;
\t\t\tconst float3 un=interpolate_u(p0,u);
\t\t\tconst float u2=dot(un,un);
\t\t\tif(u2<=1.0e-12f) break;
\t\t\tconst float inv_ul=rsqrt(u2);
\t\t\tconst float ul=u2*inv_ul;
\t\t\tconst bool disturbed=fabs(ul-U_INF)>=SPEED_DELTA || un.x*inv_ul<=DIRECTION_DOT_MAX;

\t\t\tif(!vehicle_affected && disturbed) {
\t\t\t\tconst uint3 q=closest_coordinates(p0);
\t\t\t\tbool near_vehicle=(flags[(uxx)q.x+(uxx)(q.y+q.z*def_Ny)*(uxx)def_Nx]&TYPE_S)!=0u;
\t\t\t\tfor(int dz=-VEHICLE_RADIUS;dz<=VEHICLE_RADIUS&&!near_vehicle;dz++) {
\t\t\t\t\tconst int zz=(int)q.z+dz;
\t\t\t\t\tif(zz<0||zz>=(int)def_Nz) continue;
\t\t\t\t\tfor(int dy=-VEHICLE_RADIUS;dy<=VEHICLE_RADIUS&&!near_vehicle;dy++) {
\t\t\t\t\t\tconst int yy=(int)q.y+dy;
\t\t\t\t\t\tif(yy<0||yy>=(int)def_Ny) continue;
\t\t\t\t\t\tfor(int dx=-VEHICLE_RADIUS;dx<=VEHICLE_RADIUS;dx++) {
\t\t\t\t\t\t\tif(dx*dx+dy*dy+dz*dz>VEHICLE_RADIUS2) continue;
\t\t\t\t\t\t\tconst int xx=(int)q.x+dx;
\t\t\t\t\t\t\tif(xx<0||xx>=(int)def_Nx) continue;
\t\t\t\t\t\t\tconst uxx vn=(uxx)xx+(uxx)(yy+zz*def_Ny)*(uxx)def_Nx;
\t\t\t\t\t\t\tif(flags[vn]&TYPE_S){near_vehicle=true;break;}
\t\t\t\t\t\t}
\t\t\t\t\t}
\t\t\t\t}
\t\t\t\tif(near_vehicle) vehicle_affected=true;
\t\t\t}

\t\t\tconst float3 p1=p0+(dt*inv_ul)*un;
\t\t\tif(p1.x<-hLx||p1.x>hLx||p1.y<-hLy||p1.y>hLy||p1.z<-hLz||p1.z>hLz) break;

\t\t\tif(vehicle_affected) {
\t\t\t\tif((l&1u)==0u) { pair_start=p0; pair_disturbed=disturbed; }
\t\t\t\telse {
\t\t\t\t\tpair_disturbed=pair_disturbed||disturbed;
\t\t\t\t\tif(pair_disturbed) {
\t\t\t\t\t\tconst uint3 q=closest_coordinates(pair_start);
\t\t\t\t\t\tconst uxx nn=(uxx)q.x+(uxx)(q.y+q.z*def_Ny)*(uxx)def_Nx;
\t\t\t\t\t\tif(!(flags[nn]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G))) {
\t\t\t\t\t\t\tconst int c=colorscale_rainbow(def_scale_u*ul);
\t\t\t\t\t\t\tdraw_line(pair_start,p1,c,camera_cache,zbuffer,bitmap);
\t\t\t\t\t\t}
\t\t\t\t\t}
\t\t\t\t}
\t\t\t}
\t\t\tp0=p1;
\t\t}
\t}
'''
    body = body.replace("\\n", "\n").replace("\\t", "\t")
    s = s[:body_start + 1] + body + "\n\t}" + s[end:]

    if s != original:
        KERNEL.write_text(s, encoding="utf-8")
    return s != original


def main() -> None:
    changed = patch_kernel()
    print("Applied optimized streamline kernel." if changed else "Streamline kernel already optimized.")


if __name__ == "__main__":
    main()
