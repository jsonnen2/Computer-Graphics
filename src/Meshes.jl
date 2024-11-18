module Meshes

export read_obj, write_obj
export gen_mesh, est_normals
export cube_mesh, cylinder_mesh, sphere_mesh, estimate_normals
export OBJTriangle, OBJMesh

using FileIO
using LinearAlgebra

push!(LOAD_PATH, pwd())

include("GfxBase.jl")
using .GfxBase


""" OBJTriangle
A struct that represents a single triangle in a mesh. """
mutable struct OBJTriangle
    positions::Array{Int,1} # vertex position indices
    uvs::Array{Int,1} # vertex texture coordinate indices
    normals::Array{Int,1} # normal vector indices
end

""" OBJMesh
A struct that represents an indexed triangle mesh for reading from
or writing to OBJ format. """
mutable struct OBJMesh
    positions::Array{Vec3,1} # all vertex positions
    uvs::Array{Vec2,1} # all texture coordinates
    normals::Array{Vec3,1} # all vertex normals
    triangles::Array{OBJTriangle,1} # the OBJTriangles belonging to the mesh
end

""" read_obj(obj_filename)
Read a mesh in OBJ format from file obj_filename."""
function read_obj(obj_filename)
    m = OBJMesh([], [], [], []) # create a mesh
    open(obj_filename) do f
        for (line_number, line) in enumerate(eachline(f))
            if line == "" || line[1] == "#"
                continue # skip comments
            end
            # Read the line and add its contents to the correct field of m:
            tokens = split(strip(line))
            if tokens[1] == "v" # vertex
                push!(m.positions, Vec3([parse(Float64, x) for x in tokens[2:end]]...))
            elseif tokens[1] == "vt" # vertex texture
                push!(m.uvs, Vec2([parse(Float64, x) for x in tokens[2:end]]...))
            elseif tokens[1] == "vn" # vertex normal
                push!(m.normals, Vec3([parse(Float64, x) for x in tokens[2:end]]...))
            elseif tokens[1] == "f"
                # create a OBJTriangle face:
                points = []
                uvs = []
                normals = []
                # handle faces with no texture and/or normals
                for corner in tokens[2:end]
                    indices = split(corner, '/')
                    if length(indices) == 3 # all 3 present, third is a normal
                        push!(normals, parse(Int, indices[3]))
                    end
                    if length(indices) >= 2 && indices[2] != ""
                        # if there are 2 or more and the second isn't blank, it's a texture
                        push!(uvs, parse(Int, indices[2]))
                    end
                    if length(indices) >= 1 # first value is the position
                        push!(points, parse(Int, indices[1]))
                    else # unless it has none, in which case it's not valid
                        error("in line $line_number: face vertex $corner could not be parsed")
                    end
                end
                # create the triangle and add it to the triangles array
                push!(m.triangles, OBJTriangle(points, uvs, normals))
            end
        end
    end
    return m
end

""" write_obj(obj_filename)
Write the given mesh in OBJ format to file obj_filename."""
function write_obj(obj_filename, mesh::OBJMesh)
    open(obj_filename, "w") do f
        # write all positions:
        for v in mesh.positions
            write(f, "v $(v[1]) $(v[2]) $(v[3])\n")
        end

        # write all texture coords:
        for v in mesh.uvs
            write(f, "vt $(v[1]) $(v[2])\n")
        end
        # write all normals:
        for v in mesh.normals
            write(f, "vn $(v[1]) $(v[2]) $(v[3])\n")
        end

        # write all triangles:
        for tri in mesh.triangles
            write(f, "f $(tri_vertex_str(tri))\n")
        end

    end

end

""" tri_vertex_str(triangle)
Return a string with the indices of applicable positions, texture coordinates,
and normals for a given triangle according to the OBJ specification.
In particular, if p, u, and n are position, vertex and normal, each corner
of the triangle is represented as one of the following:
    p       (position only)
    p/u     (position and texture)
    p//n    (position and normal)
    p/u/n   (position, texture, and normal) """
function tri_vertex_str(triangle::OBJTriangle)
    # determine whether textures and normals are present:
    write_uv = length(triangle.uvs) == length(triangle.positions)
    write_normals = length(triangle.normals) == length(triangle.positions)
    corners = []
    for i = 1:3
        output = "$(triangle.positions[i])"
        if write_uv && !write_normals
            output = output * "/$(triangle.uvs[i])" # * does concatenation(!)
        elseif !write_uv && write_normals
            output = output * "//$(triangle.normals[i])"
        elseif write_uv && write_normals
            output = output * "/$(triangle.uvs[i])/$(triangle.normals[i])"
        end
        push!(corners, output)
    end
    join(corners, " ")
end


""" gen_mesh(outfile, geom, divisionsU, divisionsV)
Generate a mesh and save the result in a file with name outfile.
geom may be "cube", "cylinder", or "sphere".
Cylinder requires divisionsU; sphere requires divisionsU and divisionsV. """
function gen_mesh(outfile, geom, divisionsU=0, divisionsV=0)
    if geom == "cube"
        mesh = cube_mesh()
    elseif geom == "cylinder"
        mesh = cylinder_mesh(divisionsU)
    elseif geom == "sphere"
        mesh = sphere_mesh(divisionsU, divisionsV)
    end
    write_obj(outfile, mesh)
end


""" est_normals(outfile, infile)
Estimate normals of the mesh stored in infile, saving the result in outfile."""
function est_normals(outfile, infile)
    input_mesh = read_obj(infile)
    mesh = estimate_normals(input_mesh)
    write_obj(outfile, mesh)
end


""" cube_mesh()
Return a new OBJMesh representing a 2x2x2 cube centered at the origin and
axis-aligned. """
function cube_mesh()
    positions = []
    uvs = []
    normals = []
    triangles = []
    # key to comments:
    # L/R = x = right/left
    # B/T = y = top/bottom
    # C/F = z = close/far
    push!(positions, Vec3(1, -1, -1)) # 1 RBC
    push!(positions, Vec3(1, -1, 1)) # 2 RBF
    push!(positions, Vec3(-1, -1, 1)) # 3 LBF
    push!(positions, Vec3(-1, -1, -1)) # 4 LBC
    push!(positions, Vec3(1, 1, -1)) # 5 RTC
    push!(positions, Vec3(1, 1, 1)) # 6 RTF
    push!(positions, Vec3(-1, 1, 1)) # 7 LTF
    push!(positions, Vec3(-1, 1, -1)) # 8 LTC

    # texture coordinates:
    push!(uvs, Vec2(1, 1)) # TR
    push!(uvs, Vec2(0, 1)) # TL
    push!(uvs, Vec2(0, 0)) # BL
    push!(uvs, Vec2(1, 0)) # BR

    # normals:
    push!(normals, Vec3(1, 0, 0)) # R
    push!(normals, Vec3(-1, 0, 0)) # L
    push!(normals, Vec3(0, 1, 0)) # U
    push!(normals, Vec3(0, -1, 0)) # D
    push!(normals, Vec3(0, 0, 1)) # C
    push!(normals, Vec3(0, 0, -1)) # F

    # 8 faces, 2 triangles each
    push!(triangles, OBJTriangle([1, 2, 3], [1, 2, 3], [4, 4, 4])) # bottom face 1
    push!(triangles, OBJTriangle([1, 3, 4], [1, 3, 4], [4, 4, 4])) # bottom face 2
    push!(triangles, OBJTriangle([1, 5, 6], [4, 1, 2], [1, 1, 1])) # right face 1
    push!(triangles, OBJTriangle([1, 6, 2], [4, 2, 3], [1, 1, 1])) # right face 2
    push!(triangles, OBJTriangle([2, 6, 7], [4, 1, 2], [5, 5, 5])) # far face 1
    push!(triangles, OBJTriangle([2, 7, 3], [4, 2, 3], [5, 5, 5])) # far face 2
    push!(triangles, OBJTriangle([3, 7, 8], [2, 3, 4], [2, 2, 2])) # left face 1
    push!(triangles, OBJTriangle([3, 8, 4], [2, 4, 1], [2, 2, 2])) # left face 2
    push!(triangles, OBJTriangle([4, 8, 5], [2, 3, 4], [6, 6, 6])) # far face 1
    push!(triangles, OBJTriangle([4, 5, 1], [2, 4, 1], [6, 6, 6])) # far face 2
    push!(triangles, OBJTriangle([5, 8, 7], [1, 2, 3], [3, 3, 3])) # top face 1
    push!(triangles, OBJTriangle([5, 7, 6], [1, 3, 4], [3, 3, 3])) # top face 2

    # julia automatically returns the last value in the function:
    OBJMesh(positions, uvs, normals, triangles)

end


""" cylinder_mesh(n)
Return a new OBJMesh object approximation of a cylinder with radius 1 and
height 2, centered at the origin. The logitudinal axis is aligned with y, and
it is tesselated with n divisions arranged radially around the outer surface.
The ends of the cylinder are disc-shaped caps parallel to the xz plane. See the
assignment writeup for a diagram and details.
"""
function cylinder_mesh(divisionsU)
    # TODO - feel free to drop in your A1 solution code here
end


""" sphere_mesh(n, m)
Create a Latitude-Longitude-tesselated approximation of a sphere with radius 1
centered at the origin. There are n divisions around the equator and m
divisions from pole to pole along each line of longitude. The North pole is at
(0,1,0), the South pole at (0,-1,0), and points on the Greenwich meridian are
in the x = 0 plane with z > 0. The u texture coordinate depends on longitude,
with u=0 at 180 degrees West and u=1 at 180 degrees East. The v texture
coordinate varies with latitude with v=0 at the South pole and v=1 at the North
pole. Normals should be normal to the ideal sphere surface. See the assignment
for a diagram and further details. """
function sphere_mesh(N, M)
    # TODO - feel free to drop in your A1 solution code here
    # n vertex around
    # m vertex over
    around = [2 * pi * (n / N) for n in 0:N-1]
    over = [pi * (m / M) for m in 1:M-1]

    height = [cos(phi) for phi in over]
    radius = [sin(phi) for phi in over]

    points = []
    for (y, r) in zip(height, radius)
        circle = [Vec3(round(r * cos(theta), digits=10), round(y, digits=10), round(r * sin(theta), digits=10)) for theta in around]
        append!(points, circle)
    end
    # pushfirst!(points, Vec3(0,1,0))
    # push!(points, Vec3(0,-1,0))

    down_tri = [[v, v + 1, v + N] for v in 1:N-1]
    push!(down_tri, [N, 1, 2 * N])

    up_tri = [[v + 1, v + N + 1, v + N] for v in 1:N-1]
    push!(up_tri, [1, N + 1, 2 * N])

    vertex_idx = []
    for row in 0:M-3
        neo_down_tri = [data .+ row * N for data in down_tri]
        neo_up_tri = [data .+ row * N for data in up_tri]

        append!(vertex_idx, neo_down_tri)
        append!(vertex_idx, neo_up_tri)
    end

    # Now I need to add the points on top and bottom
    top_idx = length(points) + 1
    push!(points, Vec3(0, 1, 0))
    top_bit = [[top_idx, v + 1, v] for v in 1:N-1]
    push!(top_bit, [top_idx, 1, N])
    append!(vertex_idx, top_bit)

    bot_idx = length(points) + 1
    push!(points, Vec3(0, -1, 0))
    bot_bit = [[bot_idx, v + (N * (M - 2)), v + 1 + (N * (M - 2))] for v in 1:N-1]
    push!(bot_bit, [bot_idx, N + (N * (M - 2)), 1 + N * (M - 2)])
    append!(vertex_idx, bot_bit)

    # Texture coords (fun!)
    text_coords = [Vec2(u, v) for v in 1:-(1 / N):0 for u in 0:(1/M):1]

    # Texture indices for the stripe
    up_tri = [[v + 1, v, N + v] for v in 1:N]
    down_tri = [[v + 1, v + 1 + N, v + N] for v in 1:N]
    text_idx = []
    for row in 1:M-2
        neo_up_tri = [data .+ row * N for data in up_tri]
        neo_down_tri = [data .+ row * N for data in down_tri]

        append!(text_idx, neo_up_tri)
        append!(text_idx, neo_down_tri)
    end

    # Texture indices of top and bottom
    top_tri = [[v + 1, v + N + 1, v + N + 2] for v in 1:N]
    append!(text_idx, top_tri)
    bot_tri = [[v + N + 2, v, v + 1] for v in (N+1)*(M-1)+1:(N+1)*(M-1)+N]
    append!(text_idx, bot_tri)

    triangle = [OBJTriangle(i, t, n) for (i, t, n) in zip(vertex_idx, text_idx, vertex_idx)]
    OBJMesh(points, text_coords, points, triangle)
end

"""
    estimate_normals(mesh::OBJMesh)
Estimates normals for the given mesh. Overwrites any existing normals and returns a new OBJMesh object.
"""
function estimate_normals(mesh::OBJMesh)
    # initialize array to store new normals
    new_normals = [Vec3(0.0, 0.0, 0.0) for _ in 1:length(mesh.positions)]

    #iterate through all the triangles
    for triangle in mesh.triangles
        # get vertices of triangles
        v1, v2, v3 = mesh.positions[triangle.positions[1]], mesh.positions[triangle.positions[2]],
        mesh.positions[triangle.positions[3]]

        # calculate the normal of the triangle
        edge1 = v2 - v1
        edge2 = v3 - v1
        triangle_normal = normalize(cross(edge1, edge2))

        # add normal to all vertices of the triangle
        for vertex_index in triangle.positions
            new_normals[vertex_index] += triangle_normal
        end
    end

    # normalize all the vertex normals
    new_normals = normalize.(new_normals)

    # create a new OBJMesh with estimated normals
    new_mesh = OBJMesh(
        mesh.positions,
        mesh.uvs,
        new_normals,
        mesh.triangles
    )

    return new_mesh
end

end # module Meshes


