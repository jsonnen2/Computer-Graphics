module Scenes

export HitRecord, Sphere, Scene, TriangleMesh, ray_intersect, create_triangles
#export has_uvs, has_normals, get_vertex, get_uv, get_normal

using LinearAlgebra

#push!(LOAD_PATH, pwd())
using ..GfxBase
using ..Meshes
using ..Materials



#####################################
###### Generic Scene Data Type ######
#####################################
struct Scene
    background::RGB{Float32}
    objects::Array{Any,1}
    lights::Array{Any,1}
end

""" Structure to store data about an intersection
of a ray with an object (a "hit")."""
mutable struct HitRecord
    t::Float64
    intersection::Vec3
    normal::Vec3
    uv::Union{Vec2,Nothing}
    object
end

# Abstract ray-object intersection function:
# Needs to be implemented for each type of object to be rendered
""" Intersect a ray with an object.
Returns a HitRecord with info about the intersection, or nothing if
the ray doesn't intersect with the object. """
function ray_intersect(ray::Ray, object) end


##################
##### Sphere #####
##################

# Data type:
struct Sphere
    center::Vec3
    radius::Float64
    material::Material
end

""" Ray-sphere intersection. """
function ray_intersect(ray::Ray, object::Sphere)

    #######################
    # TODO 1 (ray-sphere) #
    #######################
    # Your implementation:
    #

    # Calculate ray-sphere intersection using quadratic formula
    p = ray.origin - object.center
    # p = ray.origin
    d = ray.direction
    discrim = dot(p, d)^2 - dot(d, d) * (dot(p, p) - object.radius^2)

    # Check if there exists an intersection based on the discriminant
    if discrim < 0 # Return nothing if the discriminant is negative
        return nothing
    elseif discrim == 0 # Return one intersection if the discriminant is zero
        t = (-dot(p, d)) / dot(d, d)
    elseif discrim > 0 # Return two intersection points and pick the closest one if the discriminant is positive
        t1 = (-dot(p, d) + sqrt(discrim)) / dot(d, d)
        t2 = (-dot(p, d) - sqrt(discrim)) / dot(d, d)
        t = min(t1, t2)
    end

    # Calculate the intersection point and normal at the intersection point
    intersection = p + t .* d + object.center
    normal = normalize(intersection - object.center)

    # Calculate texture coordinates
    x, y, z = (intersection .- object.center) ./ object.radius
    y = clamp(y, -1, 1)
    u = (1 / pi) * acos(y)
    v = atan(z, x)
    uv = Vec2(u, v)

    # Return new HitRecord with intersection data
    return HitRecord(t, intersection, normal, uv, object)

    ##############
    # END TODO 1 #
    ##############

    ################################################################
    # TODO 9c - modify above to fill in Hitrec's texture coordinates
    ################################################################
end


###########################
###### Triangle Mesh ######
###########################

""" Data type: stores the OBJTriangle, a reference to its Mesh
object, and the material it should be rendered with. """
struct Triangle
    geometry::OBJTriangle
    mesh::OBJMesh
    material
end

""" Return an Array of all Triangles belonging to the given mesh, assigning
each one the given material. 
Returns an array of triangle structs"""
function create_triangles(mesh::OBJMesh, material)
    [Triangle(f, mesh, material) for f in mesh.triangles]
end

""" Some helper functions that make for easier access to triangle data: """
function get_vertex(tri::Triangle, i)
    tri.mesh.positions[tri.geometry.positions[i]]
end

function has_uvs(tri::Triangle)
    length(tri.geometry.uvs) == 3
end

function get_uv(tri::Triangle, i)
    tri.mesh.uvs[tri.geometry.uvs[i]]
end

function has_normals(tri::Triangle)
    length(tri.geometry.normals) == 3
end

function get_normal(tri::Triangle, i)
    tri.mesh.normals[tri.geometry.normals[i]]
end


function ray_intersect(ray::Ray, object::Triangle)

    ##########
    # TODO 7 #
    ##########
    # Your implementation:
    # To calc t, I need the triangle surface normal and a point on the triangle (plane)

    # Get triangle verticies and calculate edge vectors
    vertices = [get_vertex(object, i) for i in 1:3]

    E1 = vertices[1] - vertices[2]
    E2 = vertices[1] - vertices[3]
    d = ray.direction
    rhs = vertices[1] - ray.origin

    # Compute determinant and check if the ray and the plane are parallel
    M = det([E1 E2 d])
    M == 0 && return nothing # ray parallel to plane

    # Calculate barycentric coordinates to ensure the intersection is within bounds of the triangle
    beta = det([rhs E2 d]) / M
    (beta < 0 || beta > 1) && return nothing # intersection outside triangle

    gamma = det([E1 rhs d]) / M
    (gamma < 0 || gamma > 1) && return nothing # intersection outside triangle

    alpha = 1 - beta - gamma
    (alpha < 0 || alpha > 1) && return nothing

    # Calculate distance t along the ray and find the intersection point
    t = det([E1 E2 rhs]) / M
    intersection = ray.origin + t .* ray.direction

    # Find UV coordinates, return nothing if they cannot be found
    if has_uvs(object)
        uvs = [get_uv(object, i) for i in 1:3]
        uv = alpha * uvs[1] + beta * (uvs[2]) + gamma * (uvs[3])
        uv = Vec2(uv)

    else
        uv = nothing
    end

    # Find normals if they exist. Otherwise calculate normals for the triangle surface
    if has_normals(object)
        normals = [get_normal(object, i) for i in 1:3]
        # interpolate normals with a weighted sum of barycentric coordinates
        normal = alpha * normals[1] + beta * normals[2] + gamma * normals[3]
    else
        v1 = vertices[1] - vertices[2]
        v2 = vertices[1] - vertices[3]
        normal = cross(v1, v2)
        normal = normalize(normal)
    end

    # Return new HitRecord containing intersection details
    return HitRecord(t, intersection, normal, uv, object)
    ##############
    # END TODO 7 #
    ##############

    ################################################################
    # TODO 9d - modify above to fill in Hitrec's texture coordinates
    ################################################################
end

end # module Scenes
