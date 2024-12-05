module TestScenes

#push!(LOAD_PATH, pwd())
using ..GfxBase
using ..Scenes
using ..Materials
using ..Lights
using ..Meshes
using ..Cameras


# helpful things:
make_diffuse(color) = Material(Lambertian(), 0.0, nothing, color)
black = RGB{Float32}(0, 0, 0)
red = RGB{Float32}(1, 0, 0)
green = RGB{Float32}(0, 1, 0)
blue = RGB{Float32}(0, 0, 1)
white = RGB{Float32}(1, 1, 1)
purple = RGB{Float32}(1, 0, 1)



function wizard_tower()
    bg = black
    objs = []

    # add a wizard tower:
    tower_mat = Material(Lambertian(), 0.0, nothing, RGB{Float32}(0.6, 0.5, 0.5))
    tower = read_obj("data/wizard_tower.obj")
    append!(objs, mesh_helper(tower, tower_mat, 1.0, Vec3(0.2, 0, -5)))

    # add a cube
    cube_mat = Material(Lambertian(), 0.6, nothing, white)
    append!(objs, mesh_helper(cube_mesh(), cube_mat, 10.0, Vec3(-11.2, 0, 0)))

    lights = [
        PointLight(0.5, Vec3(-1, -2, -5)),  # Light 1
        PointLight(0.7, Vec3(10, 5, -8)),  # Light 2
        PointLight(0.6, Vec3(5, 10, -12)), # Light 3
        PointLight(0.4, Vec3(-5, 3, -15)), # Light 4
    ]

    Scene(bg, objs, lights)

end

end