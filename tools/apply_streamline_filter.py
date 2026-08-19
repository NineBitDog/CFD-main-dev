from pathlib import Path

KERNEL = Path(__file__).resolve().parents[1] / "FluidX3D-master" / "src" / "kernel.cpp"

OLD = '''\t//draw_circle(p, 0.5f*def_streamline_sparse, 0xFFFFFF, camera_cache, bitmap, zbuffer);\n\tfor(float dt=-1.0f; dt<=1.0f; dt+=2.0f) { // integrate forward and backward in time\n\t\tfloat3 p0, p1=p;\n\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {\n\t\t\tconst uint x = (uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;\n\t\t\tconst uint y = (uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;\n\t\t\tconst uint z = (uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;\n\t\t\tconst uxx n = (uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;\n\t\t\tif(flags[n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) return;\n\t\t\tconst float3 un = load3(n, u); // interpolate_u(p1, u)\n\t\t\tconst float ul = length(un);\n\t\t\tp0 = p1;\n\t\t\tp1 += (dt/ul)*un; // integrate forward in time\n\t\t\tif(def_scale_u*ul<0.1f||p1.x<-hLx||p1.x>hLx||p1.y<-hLy||p1.y>hLy||p1.z<-hLz||p1.z>hLz) break;\n\t\t\tint c = 0; // coloring\n\t\t\tswitch(field_mode) {\n\t\t\t\tcase 0: c = colorscale_rainbow(def_scale_u*ul); break; // coloring by velocity\n\t\t\t\tcase 1: c = colorscale_twocolor(0.5f+def_scale_rho*(rho[n]-1.0f)); break; // coloring by density\n)+"#ifdef TEMPERATURE"+R(\n\t\t\t\tcase 2: c = colorscale_iron(0.5f+def_scale_T*(T[n]-def_T_avg)); break; // coloring by temperature\n)+"#endif"+R( // TEMPERATURE\n\t\t\t}\n\t\t\tdraw_line(p0, p1, c, camera_cache, bitmap, zbuffer);\n\t\t}\n\t}\n'''

NEW = '''\t// A streamline is rendered only if its trajectory comes within 4 lattice\n\t// cells of a vehicle surface voxel (TYPE_S). Test first, then render, so\n\t// rejected streamlines never draw any pixels.\n\tbool hit_vehicle = false;\n\tconst int contact_radius = 4;\n\tfor(float dt=-1.0f; dt<=1.0f; dt+=2.0f) {\n\t\tfloat3 p1=p;\n\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {\n\t\t\tconst int x = (int)(p1.x+1.5f*(float)def_Nx)%(int)def_Nx;\n\t\t\tconst int y = (int)(p1.y+1.5f*(float)def_Ny)%(int)def_Ny;\n\t\t\tconst int z = (int)(p1.z+1.5f*(float)def_Nz)%(int)def_Nz;\n\t\t\tconst uxx n = (uxx)x+(uxx)(y+z*(int)def_Ny)*(uxx)def_Nx;\n\n\t\t\t// Search a 4-voxel neighborhood around the streamline point.\n\t\t\t// This makes the contact distance independent of the exact voxel\n\t\t\t// centerline of the streamline.\n\t\t\tfor(int dz=-contact_radius; dz<=contact_radius && !hit_vehicle; dz++) {\n\t\t\t\tfor(int dy=-contact_radius; dy<=contact_radius && !hit_vehicle; dy++) {\n\t\t\t\t\tfor(int dx=-contact_radius; dx<=contact_radius; dx++) {\n\t\t\t\t\t\tif(dx*dx+dy*dy+dz*dz > contact_radius*contact_radius) continue;\n\t\t\t\t\t\tconst int xx=x+dx, yy=y+dy, zz=z+dz;\n\t\t\t\t\t\tif(xx<0 || xx>=(int)def_Nx || yy<0 || yy>=(int)def_Ny || zz<0 || zz>=(int)def_Nz) continue;\n\t\t\t\t\t\tconst uxx nn=(uxx)xx+(uxx)(yy+zz*(int)def_Ny)*(uxx)def_Nx;\n\t\t\t\t\t\tif(flags[nn]&TYPE_S) { hit_vehicle=true; break; }\n\t\t\t\t\t}\n\t\t\t\t}\n\t\t\t}\n\t\t\tif(hit_vehicle) break;\n\t\t\tif(flags[n]&(TYPE_E|TYPE_I|TYPE_G)) break;\n\t\t\tconst float3 un = load3(n, u); // interpolate_u(p1, u)\n\t\t\tconst float ul = length(un);\n\t\t\tif(ul==0.0f) break;\n\t\t\tp1 += (dt/ul)*un;\n\t\t\tif(def_scale_u*ul<0.1f||p1.x<-hLx||p1.x>hLx||p1.y<-hLy||p1.y>hLy||p1.z<-hLz||p1.z>hLz) break;\n\t\t}\n\t}\n\tif(!hit_vehicle) return;\n\n\t// Contact confirmed: trace again and render the complete streamline in\n\t// both directions.\n\tfor(float dt=-1.0f; dt<=1.0f; dt+=2.0f) {\n\t\tfloat3 p0, p1=p;\n\t\tfor(uint l=0u; l<def_streamline_length/2u; l++) {\n\t\t\tconst uint x = (uint)(p1.x+1.5f*(float)def_Nx)%def_Nx;\n\t\t\tconst uint y = (uint)(p1.y+1.5f*(float)def_Ny)%def_Ny;\n\t\t\tconst uint z = (uint)(p1.z+1.5f*(float)def_Nz)%def_Nz;\n\t\t\tconst uxx n = (uxx)x+(uxx)(y+z*def_Ny)*(uxx)def_Nx;\n\t\t\tif(flags[n]&(TYPE_S|TYPE_E|TYPE_I|TYPE_G)) break;\n\t\t\tconst float3 un = load3(n, u); // interpolate_u(p1, u)\n\t\t\tconst float ul = length(un);\n\t\t\tif(ul==0.0f) break;\n\t\t\tp0 = p1;\n\t\t\tp1 += (dt/ul)*un;\n\t\t\tif(def_scale_u*ul<0.1f||p1.x<-hLx||p1.x>hLx||p1.y<-hLy||p1.y>hLy||p1.z<-hLz||p1.z>hLz) break;\n\t\t\tint c = 0; // coloring\n\t\t\tswitch(field_mode) {\n\t\t\t\tcase 0: c = colorscale_rainbow(def_scale_u*ul); break; // coloring by velocity\n\t\t\t\tcase 1: c = colorscale_twocolor(0.5f+def_scale_rho*(rho[n]-1.0f)); break; // coloring by density\n)+"#ifdef TEMPERATURE"+R(\n\t\t\t\tcase 2: c = colorscale_iron(0.5f+def_scale_T*(T[n]-def_T_avg)); break; // coloring by temperature\n)+"#endif"+R( // TEMPERATURE\n\t\t\t}\n\t\t\tdraw_line(p0, p1, c, camera_cache, bitmap, zbuffer);\n\t\t}\n\t}\n'''

# If the native filter is already installed, update its contact radius in-place.
PATCHED_OLD = '''\t// A streamline is rendered only if its trajectory actually reaches a\n\t// vehicle cell (TYPE_S). We trace once to test contact before drawing,\n\t// because drawing during the test would leave behind rejected lines.\n\tbool hit_vehicle = false;\n\tfor(float dt=-1.0f; dt<=1.0f; dt+=2.0f) {'''

s = KERNEL.read_text(encoding="utf-8")
if OLD in s:
    s = s.replace(OLD, NEW, 1)
    KERNEL.write_text(s, encoding="utf-8")
    print("Applied 4-voxel vehicle-contact streamline filter.")
elif "bool hit_vehicle = false;" in s:
    # Older installed filter: replace the whole script's known contact-test marker
    # with the radius-enabled implementation by locating the render block.
    start = s.find("\tbool hit_vehicle = false;")
    end_marker = "\n\tif(!hit_vehicle) return;"
    end = s.find(end_marker, start)
    if start >= 0 and end >= 0:
        # Keep the existing render pass; replace only the test pass.
        test = NEW.split("\n\t// Contact confirmed:", 1)[0]
        test = test[test.find("\t// A streamline"):]
        render_start = s.find("\tif(!hit_vehicle) return;", start)
        render_start = render_start + len("\tif(!hit_vehicle) return;\n")
        existing_render_end = s.find("\n\t}\n\t\n\t//", render_start)
        if existing_render_end < 0:
            existing_render_end = s.find("\n\t}\n}", render_start)
        prefix = s[:start]
        suffix = s[end + len(end_marker):]
        s = prefix + test + "\n\tif(!hit_vehicle) return;" + suffix
        KERNEL.write_text(s, encoding="utf-8")
        print("Updated existing streamline filter to 4-voxel contact radius.")
    else:
        raise RuntimeError("Existing streamline filter was found but its contact-test block could not be located.")
elif "Contact confirmed: trace again" in s:
    raise RuntimeError("Streamline filter exists but could not be safely updated.")
else:
    raise RuntimeError("Native graphics_streamline block not found; kernel.cpp was not modified.")
