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

# Rays.main(1, 1, 200, 300, "results/out_1.png")

function camera_1(img_height, img_width)
    CanonicalCamera(img_height, img_width)
end

function camera_2(img_height, img_width)
    eye = Vec3(20, 4, 10)
    view = Vec3(-1, 0, -5) - eye
    up = Vec3(0, 1, 0)
    focal = 8.0
    Cameras.PerspectiveCamera(eye, view, up, focal, img_height, img_width)
end

function camera_3(img_height, img_width)

    Cameras.PerspectiveCamera(
        Vec3(-1, 0.8, -1.2),  # eye::Vec3
        Vec3(1, -1, -1), # view::Vec3
        Vec3(0, 1, 0),   # up::Vec3
        0.3,     # focal::Real
        img_height, # canv_height::Int
        img_width) # canv_width::Int)
end

function camera_4(img_height, img_width)
    eye = Vec3(0.125,6.75,20)
    view = Vec3(0, -1, -4)
    up = Vec3(0, 1, 0)
    focal = 8.0
    Cameras.PerspectiveCamera(eye, view, up, focal, img_height, img_width)
end

function camera_5(img_height, img_width)
    eye = Vec3(2, 8, 0)
    view = Vec3(0, 0, -1)
    up = Vec3(0, 1, 0)
    focal = 0.125
    Cameras.PerspectiveCamera(eye, view, up, focal, img_height, img_width)
end


function scene_1()
    bg = RGB{Float32}(0.95, 0.95, 0.95)
    objs = [Sphere(Vec3(0, 0, -5), 1, Material(Flat(), 0.0, nothing, RGB{Float32}(0.73, 0, 0.17)))]
    lights = [PointLight(0.8, Vec3(0, 0, 0))]
    Scene(bg, objs, lights)
end

function scene_2()
    bg = black
    objs = [
        Sphere(Vec3(2, 0, -8), 1, Material(Lambertian(), 0.0, nothing, white)),
        Sphere(Vec3(-2, 0, -8), 2, Material(Lambertian(), 0.0, nothing, blue))
    ]

    lights = [DirectionalLight(1.0, Vec3(1, 0.5, -0.1))]
    Scene(bg, objs, lights)
end

function scene_3()
    bg = black
    mat = Material(Lambertian(), 0.0, nothing, white)
    objs = [
        Sphere(Vec3(-2, 1, -8), 1, mat),
        Sphere(Vec3(2, 1, -8), 1, mat)
    ]

    lights = [PointLight(1.0, Vec3(0, 5, -8.5))]

    Scene(bg, objs, lights)
end

function scene_4()
    bg = black
    torq = RGB{Float32}(175, 238, 238)
    mat1 = Material(BlinnPhong(white, 10), 0.0, nothing, white)
    mat2 = Material(BlinnPhong(white, 10), 0.0, nothing, blue)
    mat3 = Material(BlinnPhong(red, 100), 0.0, nothing, blue)
    objs = [
        Sphere(Vec3(-2, -1, -8), 1, mat1),
        Sphere(Vec3(-1, 1, -8), 1, mat2),
        Sphere(Vec3(0, -1, -8), 1, mat3),
        Sphere(Vec3(1, 1, -8), 1, mat2),
        Sphere(Vec3(2, -1, -8), 1, mat1),
        Sphere(Vec3(0, -5001, 0), 5000, Material(Lambertian(), 0.0, nothing, white)) # ground
    ]

    lights = [PointLight(0.8, Vec3(0, 4, -8)),
        PointLight(0.2, Vec3(0, 0, 0))]

    Scene(bg, objs, lights)
end

function scene_5()
    # Refraction scene 
    bg = black
    mat = Material(Lambertian(), 0.5, nothing, white, 0.5)

    objs = [
        Sphere(Vec3(-1, 0, -6), 0.5, mat),
        Sphere(Vec3(1, 0, -5), 0.5, mat),
        Sphere(Vec3(-1, 0, -4), 0.5, mat),
        Sphere(Vec3(0, -5001, 0), 5000, Material(Lambertian(), 0.0, nothing, white)) # ground
    ]

    lights = [DirectionalLight(0.6, Vec3(1, 1, 0)),
        PointLight(0.4, Vec3(0, 0, 0))]

    Scene(bg, objs, lights)
end

function scene_6()
    bg = black

    r = Material(BlinnPhong(white, 10), 0.0, nothing, red)
    g = Material(BlinnPhong(white, 10), 0.0, nothing, green)
    b = Material(BlinnPhong(white, 10), 0.0, nothing, blue)
    refl = Material(Lambertian(), 0.6, nothing, white)

    objs = [
        #Sphere(Vec3(-10, 0, -1), 9.2, refl),
        Sphere(Vec3(-1, -1.1, -3), 0.5, r),
        Sphere(Vec3(-0.5, -1.0, -4), 0.5, g),
        Sphere(Vec3(0, -0.9, -5), 0.5, b),
        Sphere(Vec3(5, -1, -4), 4, refl),
        #Sphere(Vec3( 10,  0.1 , -1), 9.2, refl),
        Sphere(Vec3(0, -5001, 0), 5000, Material(Lambertian(), 0.5, nothing, white)) # floor
    ]

    lights = [PointLight(0.6, Vec3(1, 10, -4)),
        PointLight(0.4, Vec3(0, 0, 0))]

    Scene(bg, objs, lights)
end


""" Take the OBJMesh mesh and return an array of Triangles from the mesh
with the given material, after scaling the mesh positions by scale and moving
them by translation """
function mesh_helper(mesh, material, scale=1.0, translation=Vec3(0, 0, 0))

    for i in 1:length(mesh.positions)
        mesh.positions[i] = mesh.positions[i] * scale + translation
    end

    create_triangles(mesh, material)
end

function scene_7()
    bg = black
    objs = []

    # add a bunny:
    bunny_mat = Material(Lambertian(), 0.0, nothing, RGB{Float32}(0.6, 0.5, 0.5))
    bunny = read_obj("data/bunny.obj")
    append!(objs, mesh_helper(bunny, bunny_mat, 1.0, Vec3(0.2, 0, -5)))

    # add a cube
    cube_mat = Material(Lambertian(), 0.5, nothing, white)
    append!(objs, mesh_helper(cube_mesh(), cube_mat, 10.0, Vec3(-11.2, 0, 0)))

    lights = [
        # PointLight(0.5, Vec3(1, 2, -5)),
        # PointLight(0.5, Vec3(-1, -2, -5)),  # Light 1
        # PointLight(0.7, Vec3(10, 5, -8)),  # Light 2
        # PointLight(0.6, Vec3(5, 10, -12)), # Light 3
        # PointLight(0.4, Vec3(-5, 3, -15)), # Light 4
        # # DirectionalLight(0.3, Vec3(0, 0, 1)),
        # # DirectionalLight(0.3, Vec3(0, 1, 1)),
        # DirectionalLight(0.3, Vec3(1, 1, 1)),
        # # DirectionalLight(0.3, Vec3(0, 1, 0))
        AreaLight(1, Vec3(2, 2, -2), Vec3(0, -1, 0), Vec3(0, 0, 1))
    ]

    Scene(bg, objs, lights)

end

function wizard_tower()
    bg = black
    objs = []

    # add a wizard tower:
    tower_mat = Material(Lambertian(), 0.0, nothing, RGB{Float32}(0.6, 0.5, 0.5))
    tower = read_obj("data/wizard_tower.obj")
    append!(objs, mesh_helper(tower, tower_mat, 1.0, Vec3(0.2, 0, -5)))

    lights = [
        PointLight(0.5, Vec3(-1, -2, -5)),  # Light 1
        PointLight(0.7, Vec3(10, 5, -8)),  # Light 2
        PointLight(0.6, Vec3(5, 10, -12)), # Light 3
        PointLight(0.4, Vec3(-5, 3, -15)), # Light 4
    ]

    Scene(bg, objs, lights)

end

function wizard_hat()
    bg = black
    objs = []

    # add a wizard tower:
    tower_mat = Material(Lambertian(), 0.0, nothing, RGB{Float32}(128/255, 0, 128/255))
    tower = read_obj("data/wizard_hat.obj")
    append!(objs, mesh_helper(tower, tower_mat, 1.0, Vec3(0.2, 0, -5)))

    lights = [
        PointLight(0.5, Vec3(-1, -2, -5)),  # Light 1
        PointLight(0.7, Vec3(10, 5, -8)),  # Light 2
        PointLight(0.6, Vec3(5, 10, -12)), # Light 3
        PointLight(0.4, Vec3(-5, 3, -15)), # Light 4
        AreaLight(0.5, Vec3(-2, 10, 5), Vec3(0,0,0), Vec3(1,1,1))
    ]

    Scene(bg, objs, lights)

end

scene_8 = scene_7

function scene_9()
    bg = black

    objs = []

    push!(objs, Sphere(Vec3(0, -5001, 0), 5000, Material(Lambertian(), 0.2, nothing, RGB{Float32}(0.8, 0.8, 1.0))))

    # sphere_material = Material(Lambertian(), 0.0, Texture("data/earth.png", false), nothing)
    # push!(objs, Sphere(Vec3(-1.25, 0, -6), 1, sphere_material))

    # sphere_m = sphere_mesh(32, 16)
    # scale = 1.0
    # translation = Vec3(1.25, 0, -6)
    # for i in 1:length(sphere_m.positions)
    #     sphere_m.positions[i] = sphere_m.positions[i] * scale + translation
    # end
    # append!(objs, create_triangles(sphere_m, sphere_material))

    cube_mat = Material(Lambertian(), 0.0, Texture("data/1.png", false), white)
    append!(objs, mesh_helper(cube_mesh(), cube_mat, 0.5, Vec3(-1, -1, -3)))

    lights = [DirectionalLight(0.4, Vec3(0, 1, 0)),
        DirectionalLight(0.8, Vec3(0.4, 0.4, 1))]

    Scene(bg, objs, lights)

end

function scene_10()
    # Loads the triangle OBJ mesh
    bg = RGB{Float32}(0.95, 0.95, 0.95)
    obj_mesh = Meshes.read_obj("OBJ_meshes\\cube.obj")
    material = Material(Flat(), 0.0, nothing, RGB{Float32}(0.73, 0, 0.17))
    tri = Scenes.create_triangles(obj_mesh, material)
    lights = [PointLight(0.8, Vec3(0, 0, 0))]
    Scene(bg, tri, lights)
end

# Test refraction (Scene 3 duplicate with some modifications)
function scene_11()
    bg = black

    # Materials
    mat_blueSphere = Material(Lambertian(), 0.0, nothing, RGB{Float32}(0.2, 0.5, 0.7))  # Opaque Blue material
    mat_blueGlass = Material(Lambertian(), 0.5, nothing, RGB{Float32}(0.7, 0.9, 1.0))  # Blue glass material
    mat_redSphere = Material(Lambertian(), 0.0, nothing, RGB{Float32}(0.8, 0.3, 0.3))  # Opaque Red material
    mat_redGlass = Material(Lambertian(), 0.5, nothing, RGB{Float32}(1.0, 0.9, 1.0))  # Red glass material
    # Objects
    objs = [
        Sphere(Vec3(-1, 1, -8), 1, mat_blueSphere),  # Solid sphere offset to top front sphere
        Sphere(Vec3(0, 1.5, -6), 1, mat_blueGlass),  # Glass-like sphere in top front
        Sphere(Vec3(1, -3, -8), 1, mat_redSphere),  # Red sphere offset to the right of bottom sphere
        Sphere(Vec3(0, -1.5, -6), 1, mat_redGlass),  # Glass-like sphere in bottom front
    ]

    # Lights
    lights = [
        PointLight(2.0, Vec3(0, 5, -8.5)),  # Bright point light from above
        PointLight(0.75, Vec3(0, -5, 8.5))  # Dimmer point light from below
    ]

    # Create and return the scene
    Scene(bg, objs, lights)
end

function wizard_cat()
    bg = black
    objs = []

    # add a wizardcat:
    wizardcat_mat = Material(Lambertian(), 0.0, nothing, RGB{Float32}(0.6, 0.5, 0.5))
    wizardcat = read_obj("data/wizardcat.obj")
    append!(objs, mesh_helper(wizardcat, wizardcat_mat, 1.0, Vec3(0.2, -2, -6)))

    # add wizardcat's orb (glass sphere)
    sphere_mat = Material(BlinnPhong(RGB(0.835, 0.212, 0.0), 64), 0.0, nothing, RGB{Float32}(0.1, 0.3, 0.5), 0.6, 1.5)
    append!(objs, mesh_helper(sphere_mesh(32, 16), sphere_mat, 1.0, Vec3(0.1, -1, -3)))
    lights = [
        AreaLight(1.5, Vec3(1, 3, -2), Vec3(1, -3, 0), Vec3(0, 0, 1))
    ]

    Scene(bg, objs, lights)

end

cameras = [camera_1, camera_2, camera_3, camera_4, camera_5]

function get_camera(i, img_height, img_width)
    cameras[i](img_height, img_width)
end


scenes = [scene_1, scene_2, scene_3, scene_4, scene_5, scene_6, scene_7, 
        scene_8, scene_9, scene_10, scene_11, wizard_tower, wizard_hat, wizard_cat]

function get_scene(scene::String)
    scene_names = [
        "scene_1", "scene_2", "scene_3", "scene_4", "refract1",
        "scene_6", "bunny", "scene_8", "scene_9", "scene_10", "refract2",
        "wizard_tower", "wizard_hat", "wizard_cat"
    ]
    i = findfirst(x -> x == scene, scene_names)
    return scenes[i]()
end

end # module TestScenes
