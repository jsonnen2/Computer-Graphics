module Lights

import LinearAlgebra.normalize

export Light, DirectionalLight, AmbientLight, PointLight
export light_direction

#push!(LOAD_PATH, pwd())
using ..GfxBase

# Light types:
#  - directional is a distant source whose direction is always the same
#  - point light emits light from a single position in 3D
abstract type Light end
struct DirectionalLight <: Light
    intensity
    direction::Vec3
end

struct PointLight <: Light
    intensity
    position::Vec3
end

""" Calculate the direction of a given light source from the position point """
function light_direction(light::Light, point::Vec3)
    if (typeof(light) == DirectionalLight) # Return light direction of a Directional light
        return light_direction(light::DirectionalLight, point)
    elseif (typeof(light) == PointLight) # Return light direction of a Point light
        return light_direction(light::PointLight, point)
    end
end

##############################
# TODO 4a - light directions #
##############################

# Implement these two functions:
function light_direction(light::DirectionalLight, point::Vec3)
    # Normalize the light direction and return it, since the light is only being cast in one direction
    return normalize(light.direction) 
end

# point source: direction from the point to the position, normalized
function light_direction(light::PointLight, point::Vec3)
    return normalize(light.position .- point)

end
###############
# END TODO 4a #
###############

end # module Lights
