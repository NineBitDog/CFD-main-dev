from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src"
KERNEL = ROOT / "kernel.cpp"
LBM = ROOT / "lbm.cpp"


def patch_kernel():
    s = KERNEL.read_text(encoding="utf-8")
    old_sig = 'const global float* p2, const global float* bbu // ) { // voxelize triangle mesh'
    new_sig = ('const global float* p2, const global float* bbu, '
               'const global uint* grid_offsets, const global uint* grid_triangles, '
               'const uint grid_n0, const uint grid_n1, '
               'const float grid_min0, const float grid_min1, '
               'const float grid_cell0, const float grid_cell1 // ) { // voxelize triangle mesh')
    if old_sig not in s:
        raise RuntimeError("voxelize_mesh signature not found")
    s = s.replace(old_sig, new_sig, 1)

    start = s.index(')+R(kernel void voxelize_mesh')
    marker = '\tif(condition) return;\n'
    pos = s.index(marker, start)
    grid_code = '''\tif(condition) return;

\t// Directional 2-D triangle spatial grid. The ray direction is fixed for
\t// this kernel launch, so only the two perpendicular coordinates are indexed.
\tconst float q0=direction==0u ? r_origin.y : direction==1u ? r_origin.x : r_origin.x;
\tconst float q1=direction==0u ? r_origin.z : direction==1u ? r_origin.z : r_origin.y;
\tconst uint g0=(uint)clamp((int)floor((q0-grid_min0)/grid_cell0),0,(int)grid_n0-1);
\tconst uint g1=(uint)clamp((int)floor((q1-grid_min1)/grid_cell1),0,(int)grid_n1-1);
\tconst uint gc=g0+g1*grid_n0;
\tconst uint gb=grid_offsets[gc], ge=grid_offsets[gc+1u];
'''
    s = s[:pos] + grid_code + s[pos + len(marker):]

    old_loop = '\tfor(uint i=0u;i<triangle_number;i++) {'
    new_loop = '\tfor(uint ii=gb;ii<ge;ii++) {\n\t\tconst uint i=grid_triangles[ii];'
    p1 = s.index(old_loop, start)
    s = s[:p1] + new_loop + s[p1 + len(old_loop):]
    p2 = s.index(old_loop, p1 + len(new_loop))
    s = s[:p2] + new_loop + s[p2 + len(old_loop):]
    KERNEL.write_text(s, encoding="utf-8")


def patch_lbm():
    s = LBM.read_text(encoding="utf-8")
    start = s.index('void LBM_Domain::voxelize_mesh_on_device(')
    end = s.index('void LBM_Domain::enqueue_unvoxelize_mesh_on_device(', start)
    block = s[start:end]
    if 'Memory<uint> grid_offsets_device' in block:
        return

    marker = '  const ulong A[3] = {(ulong)Ny * (ulong)Nz, (ulong)Nz * (ulong)Nx,\n                      (ulong)Nx * (ulong)Ny};'
    if marker not in block:
        raise RuntimeError("voxelize_mesh_on_device marker not found")

    grid_code = r'''  // Directional 2-D uniform triangle grid. Only the two coordinates
  // perpendicular to the ray direction are indexed. Triangle AABBs are
  // expanded by half a voxel so thin surfaces are retained locally.
  const float grid_min0 = direction == 0u ? mesh->pmin.y : direction == 1u ? mesh->pmin.x : mesh->pmin.x;
  const float grid_min1 = direction == 0u ? mesh->pmin.z : direction == 1u ? mesh->pmin.z : mesh->pmin.y;
  const float grid_max0 = direction == 0u ? mesh->pmax.y : direction == 1u ? mesh->pmax.x : mesh->pmax.x;
  const float grid_max1 = direction == 0u ? mesh->pmax.z : direction == 1u ? mesh->pmax.z : mesh->pmax.y;
  const float span0 = max(grid_max0-grid_min0, 1.0e-3f);
  const float span1 = max(grid_max1-grid_min1, 1.0e-3f);
  const uint target_cells = max(64u, min(16384u, (triangle_number+7u)/8u));
  float grid_cell = sqrt((span0*span1)/(float)target_cells);
  grid_cell = max(grid_cell, 1.0f);
  uint grid_n0 = max(1u, (uint)ceil(span0/grid_cell));
  uint grid_n1 = max(1u, (uint)ceil(span1/grid_cell));
  grid_n0 = min(grid_n0, 256u);
  grid_n1 = min(grid_n1, 256u);
  const float grid_cell0 = span0/(float)grid_n0;
  const float grid_cell1 = span1/(float)grid_n1;
  const uint grid_cells = grid_n0*grid_n1;

  vector<uint> grid_counts(grid_cells, 0u);
  vector<uint> tri_g0(triangle_number), tri_g1(triangle_number), tri_h0(triangle_number), tri_h1(triangle_number);
  for(uint i=0u; i<triangle_number; i++) {
    const float3 a=mesh->p0[i], b=mesh->p1[i], c=mesh->p2[i];
    const float a0=direction==0u ? min(a.y,min(b.y,c.y)) : direction==1u ? min(a.x,min(b.x,c.x)) : min(a.x,min(b.x,c.x));
    const float a1=direction==0u ? min(a.z,min(b.z,c.z)) : direction==1u ? min(a.z,min(b.z,c.z)) : min(a.y,min(b.y,c.y));
    const float b0=direction==0u ? max(a.y,max(b.y,c.y)) : direction==1u ? max(a.x,max(b.x,c.x)) : max(a.x,max(b.x,c.x));
    const float b1=direction==0u ? max(a.z,max(b.z,c.z)) : direction==1u ? max(a.z,max(b.z,c.z)) : max(a.y,max(b.y,c.y));
    tri_g0[i]=(uint)clamp((int)floor((a0-0.5f-grid_min0)/grid_cell0),0,(int)grid_n0-1);
    tri_g1[i]=(uint)clamp((int)floor((a1-0.5f-grid_min1)/grid_cell1),0,(int)grid_n1-1);
    tri_h0[i]=(uint)clamp((int)floor((b0+0.5f-grid_min0)/grid_cell0),0,(int)grid_n0-1);
    tri_h1[i]=(uint)clamp((int)floor((b1+0.5f-grid_min1)/grid_cell1),0,(int)grid_n1-1);
    for(uint gy=tri_g1[i]; gy<=tri_h1[i]; gy++)
      for(uint gx=tri_g0[i]; gx<=tri_h0[i]; gx++)
        grid_counts[gx+gy*grid_n0]++;
  }
  vector<uint> grid_offsets(grid_cells+1u, 0u);
  for(uint c=0u; c<grid_cells; c++) grid_offsets[c+1u]=grid_offsets[c]+grid_counts[c];
  vector<uint> grid_triangles(grid_offsets[grid_cells]);
  vector<uint> grid_cursor=grid_offsets;
  for(uint i=0u; i<triangle_number; i++)
    for(uint gy=tri_g1[i]; gy<=tri_h1[i]; gy++)
      for(uint gx=tri_g0[i]; gx<=tri_h0[i]; gx++)
        grid_triangles[grid_cursor[gx+gy*grid_n0]++]=i;

  Memory<uint> grid_offsets_device(device, grid_offsets.size(), 1u, grid_offsets.data());
  Memory<uint> grid_triangles_device(device, grid_triangles.size(), 1u, grid_triangles.data());
'''
    block = block.replace(marker, grid_code + '\n' + marker, 1)

    old_kernel = '''  Kernel kernel_voxelize_mesh(device, A[direction], "voxelize_mesh", direction,
                              fi, u, flags, t + 1ull, flag, p0, p1, p2,
                              bounding_box_and_velocity);'''
    new_kernel = '''  Kernel kernel_voxelize_mesh(device, A[direction], "voxelize_mesh", direction,
                              fi, u, flags, t + 1ull, flag, p0, p1, p2,
                              bounding_box_and_velocity, grid_offsets_device,
                              grid_triangles_device, grid_n0, grid_n1,
                              grid_min0, grid_min1, grid_cell0, grid_cell1);'''
    if old_kernel not in block:
        raise RuntimeError("voxelize_mesh kernel construction not found")
    block = block.replace(old_kernel, new_kernel, 1)

    old_write = '''  bounding_box_and_velocity.write_to_device();
  kernel_voxelize_mesh.run();'''
    new_write = '''  bounding_box_and_velocity.write_to_device();
  grid_offsets_device.write_to_device();
  grid_triangles_device.write_to_device();
  kernel_voxelize_mesh.run();'''
    if old_write not in block:
        raise RuntimeError("voxelize_mesh write/run block not found")
    block = block.replace(old_write, new_write, 1)

    s = s[:start] + block + s[end:]
    LBM.write_text(s, encoding="utf-8")


if __name__ == "__main__":
    patch_kernel()
    patch_lbm()
