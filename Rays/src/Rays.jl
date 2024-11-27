""" 
Main module for CS480/580 A2 raytracer. Contains core raytracing algrithm,
while referencing several other modules that encapsulate supporting
functionality.
"""
# Rays.main(7, 1, 300, 300, "results/bunny_green_edges.png")
module Rays

export main

using FileIO
using Images
using StaticArrays
using LinearAlgebra
using Images # loading images for convolutional filtering

push!(LOAD_PATH, pwd())
include("GfxBase.jl")
include("Lights.jl")
include("Materials.jl")
include("Meshes.jl")
include("Scenes.jl")
include("Cameras.jl")
include("TestScenes.jl")

using .GfxBase
using .Lights
using .Materials

import .Scenes
import .Scenes.Scene
import .Scenes.HitRecord
import .Cameras
import .TestScenes

mutable struct Edge_Storage
    obj_id::Vector{Any}
    num_shadows::Int
end

# Ray-Scene intersection:
""" Find the closest intersection point among all objects in the scene
along a ray, constraining the search to values of t between tmin and tmax. """
function closest_intersect(objects::Array{Any,1}, ray::Ray, tmin, tmax)
    ##########
    # TODO 2 #
    ##########
    # Your implementation:
    # Array of objects
    # Call ray_intersects for each object in the scene. 
    # Return the HitRecord for the "closest object" whose t value lies between tmin and tmax

    closest_hitrec = nothing # Get closest hitrec (set to nothing to start)
    smallest_dist = tmax # Get smallest intersection distance (set to tmax to start)
    for obj in objects # Loop through all objects and find the closest intersection
        hitrec = Scenes.ray_intersect(ray, obj)
        if (hitrec !== nothing) && (hitrec.t > tmin && hitrec.t < smallest_dist)
            # Check if there is an object intersection and if the object in question is the closest one
            closest_hitrec = hitrec # If true, record the hitrec and the closest distance
            smallest_dist = hitrec.t
        end
    end
    return closest_hitrec # Return closest intersection found, or nothing if no intersections are found
    #############
    # END TODO 2
    #############
end

""" Trace a ray from orig along ray through scene, using Whitted recursive raytracing 
limited to rec_depth recursive calls. """
function traceray(scene::Scene, ray::Ray, tmin, tmax, rec_depth=1, ior=1.5, transparency=0.5, bvh=nothing)
    if bvh !== nothing
        closest_hitrec = traverse_bvh(bvh, ray)
    else
        closest_hitrec = closest_intersect(scene.objects, ray, tmin, tmax)
    end

    if closest_hitrec === nothing
        return scene.background, Edge_Storage([nothing], 0)
    end
    # define variables
    object = closest_hitrec.object
    material = object.material
    shader = material.shading_model
    mirror_coeff = material.mirror_coeff

    local_color, num_shadows = determine_color(shader, object.material, ray, closest_hitrec, scene)
    edge_detect = Edge_Storage([object], num_shadows)

    ##############################
    # TODO 6 - mirror reflection #
    ##############################
    # Your implementation:
    if material.mirror_coeff > 0 && rec_depth <= 8
        r = ray.direction
        n = closest_hitrec.normal

        # calcuate the ray reflected off a surface
        reflect_ray = Ray(
            closest_hitrec.intersection,
            (r - 2 * dot(r, n) * n),
        )
        # recurse on reflected ray
        reflect_color, edge_detect_reflect = traceray(scene, reflect_ray, 1e-8, tmax, rec_depth + 1)
        # Merge. overwrite shadow. Append old object to edge_detect_reflect
        push!(edge_detect_reflect.obj_id, object)
        edge_detect_reflect.num_shadows = num_shadows
        edge_detect = edge_detect_reflect

        # scale according to mirror_coeff
        local_color = (1 - mirror_coeff) * local_color + mirror_coeff * reflect_color
    end

    ##############################
    # Refraction # (Assumes refractive object is glass, which has an IOR of 1.5)
    ##############################
    if transparency > 0 && rec_depth <= 8
        # TODO: find object id recursively
        refract_color = refract_ray(scene, ray, closest_hitrec, tmin, tmax, rec_depth, ior)
        local_color = (1 - transparency) * local_color + transparency * refract_color
    end

    return local_color, edge_detect

end

##############################
# Refraction Function # (Assumes refractive object is glass, which has an IOR of 1.5)
##############################
function refract_ray(scene::Scene, ray::Ray, hitrec::HitRecord, tmin, tmax, rec_depth, ior=1.5)
    r = normalize(ray.direction)  # Normalize ray direction
    n = normalize(hitrec.normal)  # Normalize surface normal
    cos_theta_i = -dot(r, n)      # Cosine of the angle of incidence

    # Determine if ray is entering or exiting
    eta = ior  # Refractive index ratio (n₁ / n₂)
    if cos_theta_i < 0
        n = -n  # Flip normal for exiting rays
        eta = 1.0 / ior  # Adjust eta for exiting rays
        cos_theta_i = -cos_theta_i  # Make cosine positive
    end

    # Snell's Law: Compute sin²(θ₂)
    sin2_theta_t = eta^2 * (1.0 - cos_theta_i^2)

    # Handle Total Internal Reflection (TIR)
    if sin2_theta_t > 1.0
        reflect_dir = normalize(r - 2.0 * dot(r, n) * n)  # Compute reflection direction
        reflect_ray = Ray(hitrec.intersection, reflect_dir)
        reflect_color, _ = traceray(scene, reflect_ray, tmin, tmax, rec_depth + 1)
        return reflect_color  # Return reflection color if TIR occurs
    end

    # Compute refracted direction
    cos_theta_t = sqrt(1.0 - sin2_theta_t)
    refract_dir = normalize(eta * r + (eta * cos_theta_i - cos_theta_t) * n)

    # Generate refracted ray
    refract_ray = Ray(hitrec.intersection, refract_dir)

    # Recursively trace the refracted ray
    refract_color, _ = traceray(scene, refract_ray, tmin, tmax, rec_depth + 1, ior)

    return refract_color  # Return refraction contribution
end

##############################
# BVH Node Definitions #
##############################
abstract type BVHNode end
mutable struct AABB
    min::NTuple{3,Float64}
    max::NTuple{3,Float64}
end
mutable struct BVHLeaf <: BVHNode
    bbox::AABB
    objects::Vector{Any}
end
mutable struct BVHInternal <: BVHNode
    bbox::AABB
    left::BVHNode
    right::BVHNode
end

##############################
# BVH functions #
##############################

# Compute Bounding Box for a Sphere 
# Compute Bounding Box for a Sphere
function compute_bbox(obj::Scenes.Sphere)
    # Ensure obj.center is a 3D vector and obj.radius is a scalar
    mins = Tuple(obj.center .- obj.radius)  # Convert to NTuple{3, Float64}
    maxs = Tuple(obj.center .+ obj.radius)  # Convert to NTuple{3, Float64}
    return AABB(mins, maxs)
end

# Compute Bounding Box for an Internal BVH Node
function compute_bbox(node::BVHInternal)
    left_bbox = compute_bbox(node.left)
    right_bbox = compute_bbox(node.right)

    # Calculate the element-wise minimum and maximum
    mins = (min(left_bbox.min[1], right_bbox.min[1]),
        min(left_bbox.min[2], right_bbox.min[2]),
        min(left_bbox.min[3], right_bbox.min[3]))

    maxs = (max(left_bbox.max[1], right_bbox.max[1]),
        max(left_bbox.max[2], right_bbox.max[2]),
        max(left_bbox.max[3], right_bbox.max[3]))

    return AABB(mins, maxs)
end

# Compute Bounding Box for a Triangle
function compute_bbox(obj::Scenes.Triangle)
    vertices = [Scenes.get_vertex(obj, i) for i in 1:3]

    # Initialize mins and maxs with the coordinates of the first vertex
    mins = vertices[1]
    maxs = vertices[1]

    # Iterate through the vertices to update mins and maxs element-wise
    for vertex in vertices
        mins = (min(mins[1], vertex[1]), min(mins[2], vertex[2]), min(mins[3], vertex[3]))
        maxs = (max(maxs[1], vertex[1]), max(maxs[2], vertex[2]), max(maxs[3], vertex[3]))
    end

    return AABB(mins, maxs)
end

# Compute the bounding box for the BVH Node (Leaf or Internal)
function compute_bbox(node::BVHNode)
    if isa(node, BVHLeaf)
        # If it's a leaf, compute the bounding box for the objects inside
        bbox = node.bbox
    elseif isa(node, BVHInternal)
        # If it's an internal node, recursively compute the bounding boxes of the children
        left_bbox = compute_bbox(node.left)
        right_bbox = compute_bbox(node.right)

        # Merge the bounding boxes of the children
        mins = minimum(left_bbox.min, right_bbox.min)  # Apply element-wise minimum
        maxs = maximum(left_bbox.max, right_bbox.max)  # Apply element-wise maximum

        bbox = AABB(mins, maxs)  # New bounding box
    end
    return bbox
end

# Ray-Box Intersection
function ray_intersects_aabb(ray, bbox::AABB)
    tmin = (bbox.min .- ray.origin) ./ ray.direction  # Vectorized operation
    tmax = (bbox.max .- ray.origin) ./ ray.direction  # Vectorized operation
    tmin, tmax = minimum.(tmin, tmax), maximum.(tmin, tmax)  # Correct ordering
    tmin_max = maximum(tmin)  # Largest tmin
    tmax_min = minimum(tmax)  # Smallest tmax
    return tmin_max <= tmax_min  # Intersection condition
end

# Construct BVH
function construct_bvh(objects::Vector{Any}, depth::Int=0)::BVHNode
    if length(objects) <= 2 || depth > 16
        # Base case: create a leaf node
        bbox = compute_bbox(objects[1])  # Compute bounding box for objects
        return BVHLeaf(bbox, objects)
    end

    # Sort objects by their centroids along the selected axis
    axis = depth % 3 + 1  # Cycle through x, y, z axes
    sorted_objs = sort!(objects, by=obj -> centroid(compute_bbox(obj))[axis])
    mid = length(sorted_objs) ÷ 2

    # Recursively construct BVH for left and right partitions
    left = construct_bvh(sorted_objs[1:mid], depth + 1)
    right = construct_bvh(sorted_objs[mid+1:end], depth + 1)

    # Compute bounding box for the internal node
    left_bbox = compute_bbox(left)
    right_bbox = compute_bbox(right)
    mins = Tuple(map(t -> min(t...), zip(left_bbox.min, right_bbox.min)))
    maxs = Tuple(map(t -> max(t...), zip(left_bbox.max, right_bbox.max)))
    bbox = AABB(mins, maxs)

    return BVHInternal(bbox, left, right)
end

# Calculate Centroid of a Bounding Box
function centroid(bbox::AABB)
    return 0.5 .* (bbox.min .+ bbox.max)  # Element-wise midpoint calculation
end

# Traverse BVH
function traverse_bvh(node::BVHNode, ray::Ray)
    if isa(node, BVHLeaf)
        # Check intersections for all objects in the leaf
        intersections = filter(!isnothing, [Scenes.ray_intersect(ray, obj) for obj in node.objects])
        return isempty(intersections) ? nothing : minimum(intersections, by=x -> x.t)
    end

    # If it's an internal node, check intersection with the bounding box
    if ray_intersects_aabb(ray, node.bbox)
        left_hit = traverse_bvh(node.left, ray)
        right_hit = traverse_bvh(node.right, ray)

        # Return the closer intersection
        if isnothing(left_hit)
            return right_hit
        elseif isnothing(right_hit)
            return left_hit
        else
            return left_hit.t < right_hit.t ? left_hit : right_hit
        end
    end

    return nothing  # No intersection
end

##############################
# End BVH functions #
##############################

""" Determine the color of intersection point described by hitrec 
Flat shading - just color the pixel the material's diffuse color """
function determine_color(shader::Flat, material::Material, ray::Ray, hitrec::HitRecord, scene::Scene)
    get_diffuse(material, hitrec.uv), 0
end
""" Normal shading - color-code pixels according to their normals """
function determine_color(shader::Normal, material::Material, ray::Ray, hitrec::HitRecord, scene::Scene)
    normal_color = normalize(hitrec.normal) / 2 .+ 0.5
    RGB{Float32}(normal_color...), 0
end


""" Determine the color of a physical (Lambertian, BlinnPhong, etc.) surface """
function determine_color(shader::PhysicalShadingModel, material::Material, ray::Ray, hitrec::HitRecord, scene::Scene)
    ###########
    # TODO 4c
    # Pseudocode:
    # start with a black color value
    # for each light in the scene:
    #   determine the light's contribution (by calling shade_light)
    #   add the light's contribution into the color
    # return the resulting color
    #
    num_shadows = 0
    color = RGB(0.0, 0.0, 0.0) # Start with black color value
    for light in scene.lights # For every light in the scene, determine its contribution to scene lighting using shade_light()
        lightContribution, is_shadow = shade_light(shader, material, ray, hitrec, light, scene)
        color += lightContribution # Add that light's shadow contribution to the color
        num_shadows += is_shadow
    end

    shadowed = Int(floor(num_shadows // length(scene.lights)))
    return color, num_shadows
    #############
    # END TODO 4c
    #############

    ###############################################
    # TODO 5b - modify above to account for shadows
    ###############################################
end

""" shade_light(shader, material, ray, hitrec, light, scene)
Determine the color contribution of the given light along the given ray.
Color depends on the material, the shading model (shader), properties of the intersection 
given in hitrec, """
function shade_light(shader::Lambertian, material::Material, ray::Ray, hitrec, light::Union{PointLight,DirectionalLight}, scene)
    ###########
    # TODO 4b #
    ###########
    # Your implementation:
    #
    l = light_direction(light, hitrec.intersection) # Get light direction
    n = hitrec.normal # Get surface normal
    lightDirection = max(0, dot(n, l)) # Calculate the dot product of the normal and the light direction
    lightIntensity = light.intensity # Get light intensity
    diffuseAlbedo = get_diffuse(material, hitrec.uv) # Get diffuse albedo from material
    diffuseColor = diffuseAlbedo * lightIntensity * lightDirection # Calculate diffuse color

    if (is_shadowed(scene, light, hitrec.intersection)) # Check for shadows, return black color if true
        return RGB(0.0, 0.0, 0.0), 1
    end

    return diffuseColor, 0
    # END TODO 4b
    #############
end

""" Blinn-Phong surface shading """
function shade_light(shader::BlinnPhong, material::Material, ray::Ray, hitrec, light::Union{PointLight,DirectionalLight}, scene)
    ###########
    # TODO 4d #
    ###########
    # Your implementation:
    #
    surfaceNormal = hitrec.normal # Get surface normal
    lightDirection = normalize(light_direction(light, hitrec.intersection)) # Get light direction and normalize It
    viewDirection = normalize(-ray.direction) # Get viewing direction and normalize it
    halfVector = normalize(viewDirection + lightDirection) # Calculate the half vector for specular reflection
    I = light.intensity # Get light intensity
    diffuseColor = get_diffuse(material, hitrec.uv) # Get diffuse color
    specularColor = shader.specular_color # Get specular color
    specularExponent = shader.specular_exp # Get specular exponent

    if (is_shadowed(scene, light, hitrec.intersection)) # Check if the point is shadowed
        return RGB(0.0, 0.0, 0.0), 1 # If true, return a black value for that point
    end

    # Calculate specular reflection using the diffuse and specular components
    diffuse = diffuseColor * I * max(0, dot(surfaceNormal, lightDirection))
    specular = specularColor * I * max(0, dot(surfaceNormal, halfVector))^specularExponent

    return diffuse + specular, 0 # Return the combined color from the diffuse and specular components
    #############
    # END TODO 4d
    #############
end

function shade_light(shader::Lambertian, material::Material, ray::Ray, hitrec, light::AreaLight, scene)
    dim = 5
    surfaceNormal = hitrec.normal # Get surface normal
    I = light.intensity / dim^2 # Get light intensity
    diffuseColor = get_diffuse(material, hitrec.uv) # Get diffuse color

    c = RGB(0.0, 0.0, 0.0)
    r = []
    for p in 0:dim-1
        for q in 0:dim-1
            light_sample = light.position + light.vec_a * (p + rand(Float32)) / dim + light.vec_b * (q + rand(Float32)) / dim
            push!(r, Vec3(light_sample[1], light_sample[2], light_sample[3]))
        end
    end

    for i in 1:dim^2
        if (!is_shadowed(scene, light, hitrec.intersection, r[i]))
            lightDirection = normalize(light_direction(light, hitrec.intersection, r[i])) # Get light direction and normalize It

            # Calculate specular reflection using the diffuse and specular components
            c += diffuseColor * I * max(0, dot(surfaceNormal, lightDirection))
        end
    end

    return c, 0
end


""" Determine whether point is in shadow wrt light """
###########
# TODO 5a #
###########
# Your implementation (two methods):
# 
function is_shadowed(scene, light::DirectionalLight, point::Vec3)
    shadowRay = Ray(point, normalize(light.direction)) # Get rays in shadow
    tmin = 1e-8 # Get minimum distance
    tmax = Inf # Get maximum distance
    rayHit = Rays.closest_intersect(scene.objects, shadowRay, tmin, tmax) # Calculate closest intersection of rays
    return rayHit !== nothing # Return true/false if the shadowed ray hits some object
end

function is_shadowed(scene, light::PointLight, point::Vec3)
    # TODO: possible test
    lightDirection = light.position - point # Get light direction
    shadowRay = Ray(point, normalize(lightDirection)) # Get rays in shadow
    tmin = 1e-8 # Get minimum distance
    tmax = norm(lightDirection) # Get maximum distance
    rayHit = Rays.closest_intersect(scene.objects, shadowRay, tmin, tmax) # Calculate closest intersection of rays
    return rayHit !== nothing # Return true/false if the shadowed ray hits some object
end

function is_shadowed(scene, light::AreaLight, point::Vec3, lightPoint::Vec3)
    lightDirection = lightPoint - point
    shadowRay = Ray(point, normalize(lightDirection))
    tmin = 1e-8
    tmax = norm(lightDirection)
    rayHit = Rays.closest_intersect(scene.objects, shadowRay, tmin, tmax)
    return rayHit !== nothing
end


function draw_circle!(canvas, center::Tuple{Int,Int}, radius::Float64, result)
    height, width = size(canvas)
    cx, cy = center
    r = Int(ceil(radius))

    # Bounding box around the circle
    for x in -r:r
        for y in -r:r
            # Calculate is in circle for each pixel in square
            if x^2 + y^2 <= radius^2
                # Apply circle's center to the coords
                px = cx + x
                py = cy + y

                # Check boundaries
                if 1 <= px <= width && 1 <= py <= height
                    canvas[px, py] = result
                end
            end
        end
    end
end


function edge_detection!(
    objs::Matrix{Edge_Storage}, proximity::Float64=0.0,
    canvas::Union{Matrix{ColorTypes.RGB{Float32}},Nothing}=nothing,
    detect_shadows::Bool=false)

    height, width = size(objs)
    mask = falses(height, width)

    # Iterate image to find edges
    for i in 2:height
        for j in 2:width
            edge = false

            # Detect shadow edges
            if detect_shadows
                if (objs[i, j].num_shadows !== objs[i-1, j].num_shadows) ||
                   (objs[i, j].num_shadows !== objs[i, j-1].num_shadows) ||
                   (objs[i, j].num_shadows !== objs[i-1, j-1].num_shadows)
                    # edge detected
                    edge = true
                end
            end

            id1 = objs[i-1, j-1].obj_id
            id2 = objs[i-1, j].obj_id
            id3 = objs[i, j-1].obj_id
            id4 = objs[i, j].obj_id
            min = minimum([length(id1), length(id2), length(id3), length(id4)])

            for n in 1:min
                if (id1[n] !== id4[n]) ||
                   (id2[n] !== id4[n]) ||
                   (id3[n] !== id4[n])
                    # edge detected
                    edge = true
                end
            end

            if edge
                center = (i, j)
                draw_circle!(mask, center, proximity, true)
                if canvas !== nothing
                    draw_circle!(canvas, center, proximity, RGB{Float32}(0, 1, 0))
                end
            end
        end
    end
    return mask
end

function aa_get_px_color(i, j, scene, camera, aa_mode, aa_samples)
    tmin = 1
    tmax = Inf
    color = RGB{Float32}(0, 0, 0)
    # Uniform AA
    if (aa_mode == 1)
        for p in 0:aa_samples-1
            for q in 0:aa_samples-1
                view_ray = Cameras.pixel_to_ray(camera, i + (p + 0.5) / aa_samples, j + (q + 0.5) / aa_samples)
                sub_px_color, obj = traceray(scene, view_ray, tmin, tmax)
                color = color + sub_px_color
            end
        end
        # Random AA
    elseif (aa_mode == 2)
        for p in 1:aa_samples^2
            view_ray = Cameras.pixel_to_ray(camera, i + rand(Float32), j + rand(Float32))
            sub_px_color, obj = traceray(scene, view_ray, tmin, tmax)
            color = color + sub_px_color
        end
        # Stratified AA
    elseif (aa_mode == 3)
        for p in 0:aa_samples-1
            for q in 0:aa_samples-1
                view_ray = Cameras.pixel_to_ray(camera, i + (p + rand(Float32)) / aa_samples, j + (q + rand(Float32)) / aa_samples)
                sub_px_color, obj = traceray(scene, view_ray, tmin, tmax)
                color = color + sub_px_color
            end
        end
    end
    return color / aa_samples^2
end

# Rays.main(7, 1, 300, 300, "results/bunny.png")
function main(scene, camera, height, width, outfile,
    aa_mode=0, aa_samples=1)

    # get the requested scene and camera
    scene = TestScenes.get_scene(scene)
    camera = TestScenes.get_camera(camera, height, width)

    # Construct a BVH for the objects in the scene
    bvh = construct_bvh(scene.objects)

    # Create a blank canvas to store the image:
    canvas = zeros(RGB{Float32}, height, width)
    objs = Array{Edge_Storage}(undef, height, width)

    ##########
    # TODO 3 #
    ##########
    # Pseudocode:
    #   loop over all pixels in the image
    #   for each pixel, get a viewing ray from the camera
    #   then call traceray to determine its color
    #
    for i in 1:height
        for j in 1:width
            # Full-image AA
            if (aa_mode > 3)
                canvas[i, j] = aa_get_px_color(i, j, scene, camera, aa_mode - 3, aa_samples)
                # No AA or edge-only AA
            else
                tmin = 1
                tmax = Inf
                view_ray = Cameras.pixel_to_ray(camera, i, j)
                color, edge_stor = traceray(scene, view_ray, tmin, tmax)
                canvas[i, j] = color
                objs[i, j] = edge_stor
            end
        end
    end
    ##############
    # Determine edges
    ##############
    if (aa_mode < 4)
        mask = edge_detection!(objs, 1.0, canvas, true)
    end

    #########################
    # Edge-based antialiasing
    #########################
    if (aa_mode != 0 && aa_mode < 4)
        for i in 1:height
            for j in 1:width
                if (mask[i, j] == true)
                    canvas[i, j] = aa_get_px_color(i, j, scene, camera, aa_mode, aa_samples)
                end
            end
        end
    end

    # clamp canvas to valid range:
    clamp01!(canvas)
    save(outfile, canvas)
end

function aa_run(outfile::String, aa_mode::String, aa_samples::Int, full::Bool)
    output_mode = 0

    if aa_samples == 0
        error("Sample count must be greater than 0")
    end

    if aa_mode == "u"
        output_mode = 1
    elseif aa_mode == "r"
        output_mode = 2
    elseif aa_mode == "s"
        output_mode = 3
    else
        error("Invalid antialiasing mode")
    end

    if full == true
        output_mode += 3
    end

    main(7, 1, 400, 400, outfile, output_mode, aa_samples)
end

function determine_conv_matrix(conv_type)

    if startswith(conv_type, "brighten_")
        m = match(r"^brighten_([+-]?\d*\.?\d+)", conv_type)
        scale = parse(Float32, m.captures[1])
        conv = fill(scale, 1, 1)
        # box blur convolution filter for any odd size
    elseif startswith(conv_type, "box_blur_")
        m = match(r"^box_blur_(\d+)", conv_type)
        conv_size = parse(Int, m.captures[1])
        if conv_size % 2 == 0 || conv_size < 0
            error("You must select an odd, positive value for box blur.")
        end
        conv = fill(1, conv_size, conv_size) * (1 / (conv_size * conv_size))
        # gaussian blur convolution filter for any odd size
    elseif startswith(conv_type, "gaussian_blur_")
        m = match(r"^gaussian_blur_(\d+)", conv_type)
        conv_size = parse(Int, m.captures[1])
        if conv_size % 2 == 0 || conv_size < 0
            error("You must select an odd, positive value for gaussian blur.")
        end
        conv = fill(0.0, conv_size, conv_size)
        for i in 1:conv_size
            for j in 1:conv_size
                x, y = (i - (conv_size - 1) / 2 - 1), (j - (conv_size - 1) / 2 - 1)
                conv[i, j] = (1 / (2 * pi)) * exp(-1 / 2(x * x + y * y))
            end
        end
        conv ./= sum(conv)
    elseif conv_type == "edge_detect_1"
        conv = Matrix{Int}([
            0 -1 0;
            -1 4 -1;
            0 -1 0
        ]) * (1 / 4)
    elseif conv_type == "edge_detect_2"
        conv = Matrix{Int}([
            -1 -1 -1;
            -1 8 -1;
            -1 -1 -1
        ]) * (1 / 8)
    else
        error(conv_type, " illegal convolution type")
    end

    return conv
end

function apply_convolution!(canvas, conv_type::String; mask=trues(size(canvas)))
    # fetch convolution matrix
    conv_matrix = determine_conv_matrix(conv_type)

    width, height = size(canvas)
    CX, CY = size(conv_matrix)
    cx, cy = Int((CX - 1) / 2), Int((CY - 1) / 2)

    # pad by 0
    pad_canvas = fill(RGB{Float32}(1, 1, 1), width + 2 * cx, height + 2 * cy)
    pad_canvas[1+cx:end-cx, 1+cy:end-cy] .= canvas

    # apply convolution
    for w in 1+cx:width+cx
        for h in 1+cy:height+cy
            # apply convolution to the subset specified by mask
            if mask[w-cx, h-cy]
                subset = pad_canvas[w-cx:w+cx, h-cy:h+cy]
                convolution = abs.(sum(conv_matrix .* subset))
                canvas[w-cx, h-cy] = convolution
            end
        end
    end
end

# Define a brightness function
function brightness(c::RGB{Float32})
    return c.r + c.g + c.b
end

function max_convolve!(canvas, kernel_size::Int)
    width, height = size(canvas)
    cx, cy = kernel_size, kernel_size

    # pad by 0
    pad_canvas = fill(RGB{Float32}(1, 1, 1), width + 2 * cx, height + 2 * cy)
    pad_canvas[1+cx:end-cx, 1+cy:end-cy] .= canvas

    # apply convolution
    for w in 1+cx:width+cx
        for h in 1+cy:height+cy
            subset = pad_canvas[w-cx:w+cx, h-cy:h+cy]
            canvas[w-cx, h-cy] = maximum(c -> brightness(c), subset)
        end
    end
end


# Rays.blur("conv_imgs/treebranch_source.jpg", "conv_imgs/treebranch_blur.jpg", "box_blur_3")
function blur(infile::String, outfile::String, conv::String)
    # apply bluring to an image using convolutions
    # 1. Gaussian
    # 2. Blocky 
    # 3. Edge Detection

    # load image
    img = load(infile)
    # canvas = reinterpret(RGB{N0f8}, img)
    canvas = float.(img)
    canvas = convert(Matrix{RGB{Float32}}, canvas)
    height, width = size(canvas)

    # load convolution matrix & store size
    conv_matrix = determine_conv_matrix(conv)
    apply_convolution!(canvas, conv_matrix)

    save(outfile, canvas)
end

# My gsplats assume x and y are independent. 
# The covariance can scale by a factor in the x and y independently.
mutable struct gsplat
    center::Vector{Int}
    color::RGB{Float32}
    stretchx::Float32
    stretchy::Float32
    luminance::Float32
end

function sample_gsplats(canvas::Matrix{ColorTypes.RGB{Float32}}, num_samples::Int)

    height, width = size(canvas)
    gsplat_bag = Vector{gsplat}()
    scale = 2

    for n in 1:num_samples
        cx, cy = rand(1:width), rand(1:height)
        color = canvas[cy, cx]
        alpha = max(color.r, color.g, color.b) / 1.0
        color /= alpha

        push!(gsplat_bag, gsplat([cx, cy], color, scale, scale, alpha / 100))
    end

    return gsplat_bag
end

function render_KNN(gsplat_bag::Vector{gsplat}, height::Int, width::Int, k::Int)

    store_distance = fill(0.0, k, height, width)
    store_color = fill(RGB{Float32}(0, 0, 0), k, height, width)

    for blob in gsplat_bag
        cx, cy = blob.center
        # specify a probability theshold for the Gaussian 
        # only evaluate points which are "close enough" to the center
        p = 1e-3
        offset = Int(round(sqrt(-max(blob.sx, blob.sy) * log(p))))

        for h in max(1, cy - offset):min(height, cy + offset)
            for w in max(1, cx - offset):min(width, cx + offset)

                # calculate distance b/w blob.center & (h, w)
                # store color at appropriate distance ranking
            end
        end
    end
    return canvas
end


function render_gsplat(gsplat_bag::Vector{gsplat}, height::Int, width::Int)
    canvas = fill(RGB{Float32}(0, 0, 0), height, width)

    for blob in gsplat_bag
        cx, cy = blob.center
        # specify a probability theshold for the Gaussian 
        # only evaluate points which are "close enough" to the center
        p = 1e-3
        offset = Int(round(sqrt(-max(blob.sx, blob.sy) * log(p))))

        for h in max(1, cy - offset):min(height, cy + offset)
            for w in max(1, cx - offset):min(width, cx + offset)

                # Evaluate Gaussian
                x = w - blob.center[1]
                y = h - blob.center[2]
                scale = (blob.sx * blob.sy) / (2 * pi)
                gauss = scale * exp(-1 / 2 * ((x / blob.sx)^2 + (y / blob.sy)^2))

                gauss = min(1, gauss)
                color = gauss .* blob.alpha .* blob.color
                canvas[h, w] += color

            end
        end
    end
    return canvas
end


function update_splats!(gsplat_bag, LR; stretchx=0, stretchy=0, luminance=0)
    # adds noise to the scale parameter
    for blob in gsplat_bag
        blob.stretchx += LR * stretchx
        blob.stretchy += LR * stretchy
        blob.luminance += LR * luminance
    end
end


function loss_MSE(forward, ground_truth)
    return 0.5 * (forward .- ground_truth)^2
end


# Rays.splat_main("gsplat_imgs/landscape.jpg", "gsplat_imgs/landscape_splat.jpg", 5000)
function splat_main(infile::String, outfile::String, num_samples::Int)

    # load image
    img = load(infile)
    # canvas = reinterpret(RGB{N0f8}, img)
    canvas = float.(img)
    canvas = convert(Matrix{RGB{Float32}}, canvas)
    height, width = size(canvas)

    # generate a Matrix to define 
    mask_rgb = deepcopy(canvas)
    apply_convolution!(mask_rgb, "edge_detect_2")
    apply_convolution!(mask_rgb, "box_blur_7")
    apply_convolution!(mask_rgb, "brighten_5.5")
    brightness_mask = map(c -> brightness(c), mask_rgb)
    mask_bool = map(c -> brightness(c) > 0.3, mask_rgb)

    # Gaussian Splatting
    @time begin
        gsplat_bag = sample_gsplats(canvas, num_samples)
    end
    edit_scale!(gsplat_bag, 4)
    @time begin
        canvas = render_gsplat(gsplat_bag, height, width)
    end

    # apply_convolution!(canvas, "gaussian_blur_15"; mask=mask_bool)

    save(outfile, canvas)

end

end