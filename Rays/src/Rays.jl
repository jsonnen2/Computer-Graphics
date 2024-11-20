""" 
Main module for CS480/580 A2 raytracer. Contains core raytracing algrithm,
while referencing several other modules that encapsulate supporting
functionality.
"""
# Rays.main(1, 1, 200, 300, "results/out_1.png")
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
        if (hitrec !== nothing) && (hitrec.t > tmin && hitrec.t < smallest_dist) # Check if there is an object intersection and if the object in question is the closest one
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
function traceray(scene::Scene, ray::Ray, tmin, tmax, rec_depth=1)

    closest_hitrec = closest_intersect(scene.objects, ray, tmin, tmax)

    if closest_hitrec === nothing
        return scene.background, nothing
    end
    # define variables
    object = closest_hitrec.object
    material = object.material
    shader = material.shading_model
    mirror_coeff = material.mirror_coeff

    local_color = determine_color(shader, object.material, ray, closest_hitrec, scene)

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
        reflect_color, other = traceray(scene, reflect_ray, 1e-8, tmax, rec_depth + 1)

        # scale according to mirror_coeff
        color = (1 - mirror_coeff) * local_color + mirror_coeff * reflect_color
    else
        color = local_color
    end
    
    return color, object

    #
    ############
    # END TODO 6
    ############
end

""" Determine the color of intersection point described by hitrec 
Flat shading - just color the pixel the material's diffuse color """
function determine_color(shader::Flat, material::Material, ray::Ray, hitrec::HitRecord, scene::Scene)
    get_diffuse(material, hitrec.uv)
end
""" Normal shading - color-code pixels according to their normals """
function determine_color(shader::Normal, material::Material, ray::Ray, hitrec::HitRecord, scene::Scene)
    normal_color = normalize(hitrec.normal) / 2 .+ 0.5
    RGB{Float32}(normal_color...)
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
    color = RGB(0.0, 0.0, 0.0) # Start with black color value
    for light in scene.lights # For every light in the scene, determine its contribution to scene lighting using shade_light()
        lightContribution = shade_light(shader, material, ray, hitrec, light, scene)
        color += lightContribution # Add that light's shadow contribution to the color
    end

    return color # Return the color
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
function shade_light(shader::Lambertian, material::Material, ray::Ray, hitrec, light, scene)
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
        return RGB(0.0, 0.0, 0.0)
    end

    return diffuseColor
    # END TODO 4b
    #############
end

""" Blinn-Phong surface shading """
function shade_light(shader::BlinnPhong, material::Material, ray::Ray, hitrec, light, scene)
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
        return RGB(0.0, 0.0, 0.0) # If true, return a black value for that point
    end

    # Calculate specular reflection using the diffuse and specular components
    diffuse = diffuseColor * I * max(0, dot(surfaceNormal, lightDirection))
    specular = specularColor * I * max(0, dot(surfaceNormal, halfVector))^specularExponent

    return diffuse + specular # Return the combined color from the diffuse and specular components
    #############
    # END TODO 4d
    #############
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
##############
# END TODO 5a #
##############

function edge_detection(objs, canvas, proximity=1)

    height, width = size(objs)
    mask = falses(height, width)

    # Check first pixel seperately
    if (objs[1,1] !== objs[2,1]) || (objs[1,1] !== objs[1,2])
        mask[1,1] = true
        canvas[i,j] = RGB{Float32}(0,1,0)
    end

    # Iterate image to find edges
    for i in 2:height
        for j in 2:width
            if (objs[i,j] !== objs[i-1,j]) || (objs[i,j] !== objs[i,j-1])

                mask[i,j] = true
                #canvas[i,j] = RGB{Float32}(0,1,0)
            end 
        end
    end
    return mask, canvas
end

# Main loop:
function main(scene, camera, height, width, outfile, aa_mode, aa_samples)

    # get the requested scene and camera
    scene = TestScenes.get_scene(scene)
    camera = TestScenes.get_camera(camera, height, width)


    # Create a blank canvas to store the image:
    canvas = zeros(RGB{Float32}, height, width)
    objs = Array{Any}(undef, height, width)

    ##########
    # TODO 3 #
    ##########
    # Pseudocode:
    #   loop over all pixels in the image
    #   for each pixel, get a viewing ray from the camera
    #   then call traceray to determine its color
    #
    tmin = 1
    tmax = Inf
    for i in 1:height
        for j in 1:width
            color = RGB{Float32}(0,0,0)
            # Uniform AA (Full Image)
            if (aa_mode == 4)
                for p in 0:aa_samples-1
                    for q in 0:aa_samples-1
                        view_ray = Cameras.pixel_to_ray(camera, i + (p+0.5)/aa_samples, j + (q+0.5)/aa_samples)
                        sub_px_color, obj = traceray(scene, view_ray, tmin, tmax)
                        color = color + sub_px_color
                    end
                end
                canvas[i, j] = color / aa_samples ^ 2
            # Random AA (Full Image)
            elseif (aa_mode == 5)
                for p in 1:aa_samples^2
                    view_ray = Cameras.pixel_to_ray(camera, i + rand(Float32), j + rand(Float32))
                    sub_px_color, obj = traceray(scene, view_ray, tmin, tmax)
                    color = color + sub_px_color
                end
                canvas[i, j] = color / aa_samples ^ 2
            # Stratified AA (Full Image)
            elseif (aa_mode == 6)
                for p in 0:aa_samples-1
                    for q in 0:aa_samples-1
                        view_ray = Cameras.pixel_to_ray(camera, i + (p+rand(Float32))/aa_samples, j + (q+rand(Float32))/aa_samples)
                        sub_px_color, obj = traceray(scene, view_ray, tmin, tmax)
                        color = color + sub_px_color
                    end
                end
                canvas[i, j] = color / aa_samples ^ 2
            else
                view_ray = Cameras.pixel_to_ray(camera, i, j)
                color, obj_id = traceray(scene, view_ray, tmin, tmax)
                canvas[i, j] = color
                objs[i, j] = obj_id
            end
        end
    end
    ##############
    # Determine edges
    ##############
    if (aa_mode < 4)
        mask, canvas = edge_detection(objs, canvas)
    end

    #########################
    # Edge-based antialiasing
    #########################
    if (aa_mode != 0 && aa_mode < 4)
        for i in 1:height
            for j in 1:width
                if (mask[i, j] == true)
                    color = RGB{Float32}(0,0,0)
                    # Uniform sampling
                    if (aa_mode == 1)
                        for p in 0:aa_samples-1
                            for q in 0:aa_samples-1
                                view_ray = Cameras.pixel_to_ray(camera, i + (p+0.5)/aa_samples, j + (q+0.5)/aa_samples)
                                sub_px_color, obj = traceray(scene, view_ray, tmin, tmax)
                                color = color + sub_px_color
                            end
                        end
                    # Random sampling
                    elseif (aa_mode == 2)
                        for p in 1:aa_samples^2
                            view_ray = Cameras.pixel_to_ray(camera, i + rand(Float32), j + rand(Float32))
                            sub_px_color, obj = traceray(scene, view_ray, tmin, tmax)
                            color = color + sub_px_color
                        end
                    # Stratified sampling 
                    elseif (aa_mode == 3)
                        for p in 0:aa_samples-1
                            for q in 0:aa_samples-1
                                view_ray = Cameras.pixel_to_ray(camera, i + (p+rand(Float32))/aa_samples, j + (q+rand(Float32))/aa_samples)
                                sub_px_color, obj = traceray(scene, view_ray, tmin, tmax)
                                color = color + sub_px_color
                            end
                        end
                    end
                    canvas[i, j] = color / aa_samples ^ 2
                end
            end
        end
    end

    # clamp canvas to valid range:
    clamp01!(canvas)
    save(outfile, canvas)
end

end # module Rays

