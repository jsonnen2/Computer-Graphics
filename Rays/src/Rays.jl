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

use_bvh = false

mutable struct Edge_Storage
    obj_id::Vector{Any}
    num_shadows::Vector{Int}
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
end

""" Trace a ray from orig along ray through scene, using Whitted recursive raytracing 
limited to rec_depth recursive calls. """
function traceray(scene::Scene, ray::Ray, tmin, tmax, rec_depth=1)

    if use_bvh == true
        bvh = construct_bvh(scene.objects)
        closest_hitrec = traverse_bvh(bvh, ray)
    else
        closest_hitrec = closest_intersect(scene.objects, ray, tmin, tmax)
    end

    if closest_hitrec === nothing
        return scene.background, Edge_Storage([nothing], [0])
    end
    # define variables
    object = closest_hitrec.object
    material = object.material
    shader = material.shading_model
    mirror_coeff = material.mirror_coeff

    local_color, num_shadows = determine_color(shader, object.material, ray, closest_hitrec, scene)
    edge_detect = Edge_Storage([object], [num_shadows])

    ##############################
    # mirror reflection #
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
        reflect_color, ED = traceray(scene, reflect_ray, 1e-8, tmax, rec_depth + 1)
        # Merge. overwrite shadow. Append old object to edge_detect_reflect
        if material.transparency <= 0
            push!(ED.obj_id, edge_detect.obj_id[1])
            push!(ED.num_shadows, edge_detect.num_shadows[1])
            edge_detect = ED
        end

        # scale according to mirror_coeff
        local_color = (1 - mirror_coeff) * local_color + mirror_coeff * reflect_color
    end

    ##############################
    # Refraction # (Assumes refractive object is glass, which has an IOR of 1.5)
    ##############################
    if material.transparency > 0 && rec_depth <= 8
        refract_color, ED = refract_ray(scene, ray, closest_hitrec, tmin, tmax, rec_depth, material.ior)
        # Merge. overwrite shadow. Append old object to edge_detect_reflect
        push!(ED.obj_id, edge_detect.obj_id[1])
        push!(ED.num_shadows, edge_detect.num_shadows[1])
        edge_detect = ED

        # scale according to transparency
        local_color = (1 - material.transparency) * local_color + material.transparency * refract_color
    end

    return local_color, edge_detect

end

##############################
# Refraction Function # (Assumes refractive object is glass, which has an IOR of 1.5)
##############################
function refract_ray(scene::Scene, ray::Ray, hitrec::HitRecord, tmin, tmax, rec_depth, ior)

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
        reflect_color, ED = traceray(scene, reflect_ray, tmin, tmax, rec_depth + 1)
        return reflect_color, ED  # Return reflection color if TIR occurs
    end

    # Compute refracted direction
    cos_theta_t = sqrt(1.0 - sin2_theta_t)
    refract_dir = normalize(eta * r + (eta * cos_theta_i - cos_theta_t) * n)

    # Generate refracted ray
    refract_ray = Ray(hitrec.intersection, refract_dir)

    # Recursively trace the refracted ray
    refract_color, ED = traceray(scene, refract_ray, tmin, tmax, rec_depth + 1)

    return refract_color, ED  # Return refraction contribution
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
    # Calculate tmin and tmax for each axis
    t1 = (bbox.min .- ray.origin) ./ ray.direction
    t2 = (bbox.max .- ray.origin) ./ ray.direction

    # Ensure t1 is the smaller and t2 is the larger value for each axis
    tmin = min(t1, t2)  # Smallest t for each axis
    tmax = max(t1, t2)  # Largest t for each axis

    # Find the largest tmin and the smallest tmax
    tmin_max = maximum(tmin)  # Largest tmin across all axes
    tmax_min = minimum(tmax)  # Smallest tmax across all axes

    # Intersection occurs if tmin_max <= tmax_min
    return tmin_max <= tmax_min && tmax_min >= 0
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

function shade_light(shader::BlinnPhong, material::Material, ray::Ray, hitrec, light::AreaLight, scene)
    dim = 5

    surfaceNormal = hitrec.normal # Get surface normal
    I = light.intensity / dim^2 # Get light intensity
    diffuseColor = get_diffuse(material, hitrec.uv) # Get diffuse color
    specularColor = shader.specular_color # Get specular color
    specularExponent = shader.specular_exp # Get specular exponent

    r = []
    for p in 0:dim-1
        for q in 0:dim-1
            light_sample = light.position + light.vec_a * (p + rand(Float32)) / dim + light.vec_b * (q + rand(Float32)) / dim
            push!(r, Vec3(light_sample[1], light_sample[2], light_sample[3]))
        end
    end

    c = RGB(0.0, 0.0, 0.0)
    for i in 1:dim^2
        lightDirection = normalize(light_direction(light, hitrec.intersection, r[i])) # Get light direction and normalize It
        viewDirection = normalize(-ray.direction) # Get viewing direction and normalize it
        halfVector = normalize(viewDirection + lightDirection) # Calculate the half vector for specular reflection

        # Calculate specular reflection using the diffuse and specular components
        c += diffuseColor * I * max(0, dot(surfaceNormal, lightDirection))
        c += specularColor * I * max(0, dot(surfaceNormal, halfVector))^specularExponent
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
    objs::Matrix{Edge_Storage},
    proximity::Float64=0.0,
    detect_shadows::Bool=false)

    height, width = size(objs)
    mask = falses(height, width)

    # Iterate image to find edges
    for i in 2:height
        for j in 2:width
            edge = false

            # Detect shadow edges
            if detect_shadows
                s1 = objs[i, j].num_shadows
                s2 = objs[i-1, j].num_shadows
                s3 = objs[i, j-1].num_shadows
                s4 = objs[i-1, j-1].num_shadows
                min = minimum([length(s1), length(s2), length(s3), length(s4)])

                for n in 1:min
                    if (s1[end-n+1] !== s4[end-n+1]) ||
                       (s2[end-n+1] !== s4[end-n+1]) ||
                       (s3[end-n+1] !== s4[end-n+1])

                        edge = true
                    end
                end
            end

            # Store object ids up, left, up-left, and here
            id1 = objs[i-1, j-1].obj_id
            id2 = objs[i-1, j].obj_id
            id3 = objs[i, j-1].obj_id
            id4 = objs[i, j].obj_id
            min = minimum([length(id1), length(id2), length(id3), length(id4)])

            # Detect edges starting at the end of the array
            for n in 1:min
                if (id1[end-n+1] !== id4[end-n+1]) ||
                   (id2[end-n+1] !== id4[end-n+1]) ||
                   (id3[end-n+1] !== id4[end-n+1])

                    edge = true
                end
            end

            if edge
                center = (i, j)
                draw_circle!(mask, center, proximity, true)
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
    if (aa_mode == "uniform")
        for p in 0:aa_samples-1
            for q in 0:aa_samples-1
                view_ray = Cameras.pixel_to_ray(camera, i + (p + 0.5) / aa_samples - 0.5, j + (q + 0.5) / aa_samples - 0.5)
                sub_px_color, obj = traceray(scene, view_ray, tmin, tmax)
                color = color + sub_px_color
            end
        end
        # Random AA
    elseif (aa_mode == "random")
        for p in 1:aa_samples^2
            view_ray = Cameras.pixel_to_ray(camera, i + 0.5 - rand(Float32), j + 0.5 - rand(Float32))
            sub_px_color, obj = traceray(scene, view_ray, tmin, tmax)
            color = color + sub_px_color
        end
        # Stratified AA
    elseif (aa_mode == "stratified")
        for p in 0:aa_samples-1
            for q in 0:aa_samples-1
                view_ray = Cameras.pixel_to_ray(camera, i + (p - (rand(Float32) + 0.5) / 2) / aa_samples - 0.5, j + (q - (rand(Float32) + 0.5) / 2) / aa_samples - 0.5)
                sub_px_color, obj = traceray(scene, view_ray, tmin, tmax)
                color = color + sub_px_color
            end
        end
    end
    return color / aa_samples^2
end


# Rays.main("wizard_hat", 4)
function main(scene_name, camera_name, AA_type="none", sample_type="uniform",
    AA_samples=1, thickness=1.0, detect_shadows=true; bvh_toggle=false)
    """
Parameters
    AA_type: Type of Anti-Aliasing to perform.
        - "full"            -- across entire image
        - "edge_detect"     -- only at pixels which my algorithm determines as an edge
        - "none"            -- standard raytracing with 1 sample per pixel center
    sample_type: Sampling method for subpixels during Anti-Aliasing
        - "uniform"
        - "random" 
        - "stratified"
    AA_samples: number of anti-aliasing samples to perform per pixel
    thickness: The radius of circle to enscribe on each detected edge. 
                Recommended to use less than 2.
    detect_shadows: boolean which controls if edges for shadows are included. It is 
                    recommended to turn off when using directional lighting.
    """
  @time begin
    height, width = 300, 300
    out_dir = "results"
    global use_bvh
    use_bvh = bvh_toggle

    # get the requested scene and camera
    scene = TestScenes.get_scene(scene_name)
    camera = TestScenes.get_camera(camera_name, height, width)

    # Create a blank canvas to store the image:
    canvas = zeros(RGB{Float32}, height, width)
    objs = Array{Edge_Storage}(undef, height, width)

    if AA_type == "full"
        # Anti-Alias across the entire image. 
        for i in 1:height
            for j in 1:width
                canvas[i, j] = aa_get_px_color(i, j, scene, camera, sample_type, AA_samples)
            end
        end
    elseif AA_type == "edge_detect"
        # generate the data to be able to edge detect 
        for i in 1:height
            for j in 1:width
                tmin = 1
                tmax = Inf
                view_ray = Cameras.pixel_to_ray(camera, i, j)
                color, edge_stor = traceray(scene, view_ray, tmin, tmax)
                canvas[i, j] = color
                objs[i, j] = edge_stor
            end
        end
        # generate boolean mask
        mask = edge_detection!(objs, thickness, detect_shadows)
        
        # Anti-Alias according to mask
        for i in 1:height
            println("Row $i complete.")
            for j in 1:width
                if (mask[i, j] == true)
                    canvas[i, j] = aa_get_px_color(i, j, scene, camera, sample_type, AA_samples)
                    # canvas[i, j] = RGB{Float32}(0,1,0) # draw green lines
                end
            end
        end
    elseif AA_type == "none"
        # standard ray tracing
        for i in 1:height
            println("Row $i complete.")
            for j in 1:width
                tmin = 1
                tmax = Inf
                view_ray = Cameras.pixel_to_ray(camera, i, j)
                color, edge_stor = traceray(scene, view_ray, tmin, tmax)
                canvas[i, j] = color
    end end end
    # Determine filename to save as
    if AA_type == "none"
        outfile = "$out_dir/$scene_name/no_anti_aliasing.png"
    elseif startswith(AA_type, "full_")
        outfile = "$out_dir/$scene_name/full_AA-$sample_type-N=$AA_samples.png"
    else
        outfile = "$out_dir/$scene_name/edge_detect_AA-$sample_type-N=$AA_samples-THICK=$thickness-shadows=$detect_shadows.png"
    end
    println(outfile)
    # clamp canvas to valid range:
    clamp01!(canvas)
    # create directory and save image
    dir_name = dirname(outfile)
    if !isdir(dir_name)
        mkpath(dir_name)
    end
    save(outfile, canvas)
end end end