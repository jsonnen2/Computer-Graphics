module Materials

using Images
using FileIO

using ..GfxBase

export Material, Texture
export ShadingModel, Flat, Normal
export PhysicalShadingModel, Lambertian, BlinnPhong

export get_diffuse

## Shading model type hierarchy ##
# Types are as follows:
# 
# ShadingModel
#   - Flat - simply the diffuse color of the object
#   - Normal - color-code a pixel according to its surface normal
#   - PhysicalShadingModel
#       - Lambertian - diffuse shading
#       - BlinnPhong - diffuse+specular shading

abstract type ShadingModel end
struct Flat <: ShadingModel end
struct Normal <: ShadingModel end

abstract type PhysicalShadingModel <: ShadingModel end

struct Lambertian <: PhysicalShadingModel end

mutable struct BlinnPhong <: PhysicalShadingModel
    specular_color::RGB{Float32} # color of the highlight
    specular_exp::Float64 # "sharpness" exponent
end

## Texture struct definition ##
mutable struct Texture
    image_data::Array{RGB{Float32},2}
    repeat::Bool
end

""" Texture constructor - loads image data from a file"""
function Texture(image_fn::String, repeat::Bool)
    image_data = load(image_fn)
    Texture(image_data, repeat)
end

## Material struct definition ##
mutable struct Material
    shading_model::ShadingModel
    mirror_coeff::Float64
    texture::Union{Texture, Nothing}
    diffuse_color::Union{RGB{Float32}, Nothing}
    transparency::Float64
    ior::Float64

    # Define constructor with default values
    function Material(
        shading_model::ShadingModel, 
        mirror_coeff::Float64, 
        texture::Union{Texture, Nothing}, 
        diffuse_color::Union{RGB{Float32}, Nothing},
        transparency::Float64 = 0.0, 
        ior::Float64 = 1.5, 
    )
        new(shading_model, mirror_coeff, texture, diffuse_color, transparency, ior)
    end
end

""" Get the diffuse color of a material; if the material is textured,
provide the uv coordinate on the object of that material. """
function get_diffuse(material::Material, uv::Union{Vec2,Nothing})
    ###########
    # TODO 9b #
    ###########

    if material.texture !== nothing # Check if material has a texture
        return get_texture_value(material.texture, uv) # If true, return the uv coordinate at the texture
    else
        return material.diffuse_color # Otherwise, return the diffuse color of the material
    end
    ##########
    # Your implementation:
    #
    ###############
    # END TODO 9b #
    ##############
end

""" Look up a texture value given the uv coordinates """
function get_texture_value(texture::Texture, uv::Vec2)
    ###########
    # TODO 9a #
    ###########
    # Placeholder:
    # return RGB{Float32}(1,1,1)
    # Your implementation:
    #
    # There are problems with my A1 uv coords, so I manually 
    i, j = uv

    height, width = size(texture.image_data)
    x = round(Int, width - (j*(width-1)))
    y = round(Int, 1 + i*(height-1))
    return texture.image_data[x,y]
    ###############
    # END TODO 9a #
    ###############
end

end # module Materials
