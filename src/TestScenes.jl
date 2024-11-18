module TestScenes

export artifact_sonnenj

push!(LOAD_PATH, pwd())
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
yellow = RGB{Float32}(1, 1, 0)

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
        img_width) # canv_width::Int
end

function camera_artifact(img_height, img_width)

    Cameras.PerspectiveCamera(
        Vec3(-1, 0.8, -1.2),  # eye::Vec3
        Vec3(1, -1, -1), # view::Vec3
        Vec3(0, 1, 0),   # up::Vec3
        0.3,     # focal::Real
        img_height, # canv_height::Int
        img_width) # canv_width::Int
end

cameras = [camera_1, camera_2, camera_3, camera_artifact]

function get_camera(i, img_height, img_width)
    cameras[i](img_height, img_width)
end


function get_scene(i)
    scenes[i]()
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
    bg = black

    mat = Material(Lambertian(), 0.0, nothing, white)

    objs = [
        Sphere(Vec3(-1, 0, -6), 0.5, mat),
        Sphere(Vec3(1, 0, -5), 0.5, Material(Lambertian(), 0.0, nothing, white)),
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
    cube_mat = Material(Lambertian(), 0.6, nothing, white)
    append!(objs, mesh_helper(cube_mesh(), cube_mat, 10.0, Vec3(-11.2, 0, 0)))

    lights = [PointLight(0.5, Vec3(1, 2, -5)),
        DirectionalLight(0.3, Vec3(0, 0, 1)),
        DirectionalLight(0.3, Vec3(0, 1, 1)),
        DirectionalLight(0.3, Vec3(1, 1, 1)),
        DirectionalLight(0.3, Vec3(0, 1, 0))]

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

function artifact_sonnenj(img_height, img_width)
	#r: 173, g: 216, b: 230
    # bg = RGB{Float32}(173, 216, 230) ./ 256
    # bg = RGB{Float32}(0.95, 0.95, 0.95)
    bg = black

    face = Material(BlinnPhong(white, 10), 0.0, nothing, yellow)
    eye = Material(BlinnPhong(green, 100), 0.0, nothing, black)
    mouth = Material(Lambertian(), 0.0, nothing, black)
    
    objs = [
        # Face
        Sphere(Vec3(0,0,-6), 1, face)
        # Eyes
        Sphere(Vec3(0.325,0.425,-5.0), 0.25, eye)
        Sphere(Vec3(-0.325,0.425,-5.0), 0.25, eye)
        # Mouth
        Sphere(Vec3(0,-0.25, -5), 0.125, mouth)
        Sphere(Vec3(0.125,-0.2125, -5), 0.125, mouth)
        Sphere(Vec3(-0.125,-0.2125, -5), 0.125, mouth)
        Sphere(Vec3(0.25,-0.105, -5), 0.125, mouth)
        Sphere(Vec3(-0.25,-0.105, -5), 0.125, mouth)

        Sphere(Vec3(0, -5001, 0), 5000, Material(Lambertian(), 0.5, nothing, white)) # floor
    ]
    lights = [
        PointLight(0.4, Vec3(-5,8,-3)),
        DirectionalLight(0.3, Vec3(0, 0, 1)),
        DirectionalLight(0.3, Vec3(0, 1, 1)),
        DirectionalLight(0.3, Vec3(0, 1, 0))
    ]
    camera = CanonicalCamera(img_height, img_width)
    scene = Scene(bg, objs, lights)
    return scene, camera

    ##################################
    # TODO 10 - one per group member #
    ##################################
end

function artifact_biraris()
    bg = RGB{Float32}(0, 0.01, 0.05) # Create background color

    # Create materials
    mat1 = Material(Lambertian(), 0.0, nothing, RGB{Float32}(0.0, 0.5, 1.0))
    mat2 = Material(BlinnPhong(RGB{Float32}(1.0, 0.5, 0.0), 10), 0.0, nothing, RGB{Float32}(1.0, 0.5, 0.0))
    mat3 = Material(Lambertian(), 0.0, nothing, RGB{Float32}(1.0, 1.0, 1.0))

    # List to store objects used in scene
    objs = []

    # Place first layer of spheres in a narrow circular formation
    for angle in range(0, stop=2π, length=100)
        radius = 1
        height = 1.125 * sin(5 * angle)
        push!(objs, Sphere(Vec3(radius * cos(angle), height, -5), 0.05, mat1))
    end

    # Place second layer of spheres in a less narrow circular formation
    for angle in range(0, stop=2π, length=50)
        radius = 2
        height = 1.15 * sin(10 * angle)
        push!(objs, Sphere(Vec3(radius * cos(angle), height, -5), 0.1, mat2))
    end

    # Place third layer of spheres in an oblong circular formation
    for angle in range(0, stop=2π, length=100)
        radius = 1.5
        height = 1.125 * sin(7.5 * angle)
        push!(objs, Sphere(Vec3(radius * cos(angle), height, -5), 0.15, mat3))
    end

    # Setup lights
    lights = [PointLight(0.05, Vec3(-5, -5, -5)), PointLight(0.01, Vec3(5, 5, 5)), DirectionalLight(0.5, Vec3(1, -1, -1)), DirectionalLight(0.25, Vec3(-1, 1, 1))]

    # Return new scene
    return Scene(bg, objs, lights)
    ##################################
    # TODO 10 - one per group member #
    ##################################
end


scenes = [scene_1, scene_2, scene_3, scene_4, scene_5, scene_6, scene_7, scene_8, scene_9, artifact_sonnenj]



end # module TestScenes
