
module splat_neighborhood

import Voronoi

using Random
using Printf
using PyPlot
using PyCall

using Images
using StaticArrays
using LinearAlgebra
using GeometryBasics


global HEIGHT
global WIDTH

mutable struct splat
    center::Vector{Float32}
    color::RGB{Float32}
    luminance::Float64
end


function sample_splats(canvas::Matrix{RGB{Float32}}, sampling_type::String, num_blobs::Int)
    
    uniform_step_size = HEIGHT * WIDTH / num_blobs
    splat_bag = Vector{splat}()

    for n in 1:num_blobs
        
        if sampling_type == "random"
            cx, cy = rand(1:WIDTH), rand(1:HEIGHT)
            center = [cx, cy] # always integer
        elseif sampling_type == "uniform"
            c = Int(ceil(n * uniform_step_size))
            cy = Int(ceil(c / WIDTH))
            cx = c - (cy-1) * WIDTH 
            center = [cx, cy]
        else
            error("Unsupported init_type = $sampling_type")
        end
        
        color = canvas[center[2], center[1]]
        alpha = max(color.r, color.g, color.b)
        color /= alpha

        push!(splat_bag, splat(center, color, alpha))
    end

    return splat_bag
end

function drop_repeats(splat_bag::Vector{splat})

    unique_centers = Set()
    unique_splats = Vector{splat}()
    
    for s in splat_bag
        if s.center ∉ unique_centers # ERROR type Vector{float} is bad
            push!(unique_centers, s.center)
            push!(unique_splats, s)
        end
    end

    return unique_splats

end

function sort_splats!(splat_bag::Vector{splat})

    sort!(splat_bag, by = s -> (-s.center[2], s.center[1]))
end

function forward_pass_mac0499(splat_bag::Vector{splat})

    canvas = fill(RGB{Float32}(0,0,0), HEIGHT, WIDTH)

    centroids = Vector{Tuple{Real, Real}}([(blob.center[1], blob.center[2]) for blob in splat_bag])
    colors = [blob.color .* blob.luminance for blob in splat_bag]
    tuple_colors = [(c.r, c.g, c.b) for c in colors] # convert RGB{float} into Tuple{float, float, float}

    V = Voronoi.Fortune.compute(centroids, WIDTH, HEIGHT)
    Voronoi.Intersect.intersect(V, Voronoi.Intersect.Rectangle(WIDTH, HEIGHT))

    matplotlib.use("Agg")
    fig, ax = plt.subplots(figsize=(WIDTH / 100, HEIGHT / 100))
    ax.axis("off")
    ax.set_xlim(0, WIDTH)
    ax.set_ylim(0, HEIGHT)
    ax.invert_yaxis()
    regions = Voronoi.Diagram.regionBorders(V)
    # draw regions
    for (i, region) in enumerate(regions)
        if size(region[1])[1] > 0
            ax.fill(region[1], region[2], color=tuple_colors[i])
        end
    end

    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
    savefig("imgs/tmp.png", format="png", dpi=100)
    close(fig)

    img = load("imgs/tmp.png")
    rm("imgs/tmp.png")
    canvas = float.(img)
    canvas = convert(Matrix{RGB{Float32}}, canvas)

    return canvas

end

function forward_pass_naive(splat_bag::Vector{splat})

    canvas = fill(RGB{Float32}(0,0,0), HEIGHT, WIDTH)

    for w in 1:WIDTH
        for h in 1:HEIGHT
            nearest_blob = nothing
            nearest_blob_distance = Inf

            for blob in splat_bag
                # searching for the nearest splat
                distance = sqrt((blob.center[1] - w)^2 + (blob.center[2] - h)^2)
                if distance < nearest_blob_distance
                    nearest_blob = blob
                    nearest_blob_distance = distance
                end
            end
            # nearest has been found
            canvas[h, w] = nearest_blob.color .* nearest_blob.luminance
        end
    end
    return canvas
end


function update_splats!(splat_bag, gradients, LR_mean, LR_lum)

    for (idx, blob) in enumerate(splat_bag)
        blob.luminance += LR_lum * gradients[1][idx]
        blob.center[1] += LR_mean * gradients[2][idx]
        blob.center[2] += LR_mean * gradients[3][idx]

        blob.luminance = clamp(blob.luminance, 0.0, 1.0)
        blob.center[1] = clamp(blob.center[1], 0, WIDTH)
        blob.center[2] = clamp(blob.center[2], 0, HEIGHT)
    end
    
end


function calc_gradients(loss, splat_bag)

    # TODO: calc direction of gradient for center 
    grad_luminance = [0.0 for blob in splat_bag]
    grad_mean_x = [0.0 for blob in splat_bag]
    grad_mean_y = [0.0 for blob in splat_bag]
    return [grad_luminance, grad_mean_x, grad_mean_y]
end

function train(infile::String, outfile::String, LR_mean, LR_lum, init_type, epochs, num_blobs, batch_size)

    # load image
    img = load(infile)
    canvas = float.(img)
    canvas = convert(Matrix{RGB{Float32}}, canvas)
    height, width = size(canvas)
    global HEIGHT = height
    global WIDTH = width

    # init splats
    splat_bag = sample_splats(canvas, init_type, num_blobs)
    splat_bag = drop_repeats(splat_bag)
    sort_splats!(splat_bag)

    # forward pass
    forward = forward_pass_mac0499(splat_bag)

    # loss
    loss = (canvas .- forward) # TODO: new loss functions

    # batch blobs 
    n = length(splat_bag)
    sample_size = Int(floor(batch_size * n))
    batch_idx = shuffle(1:n)[1:sample_size]

    # update gradients
    gradients = calc_gradients(loss, splat_bag[batch_idx])
    update_splats!(splat_bag[batch_idx], gradients, LR_mean, LR_lum)
            
    # # create and save image
    # img = clamp01.(forward)
    # dir_name = dirname(outfile)
    # if !isdir(dir_name)
    #     mkpath(dir_name)
    # end
    # save(outfile, img)


end

function control_panel()

    infile = "imgs/source/landscape.jpg"
    out_dir = "imgs/neighborhood_splats_powers_of_2"

    batch_size = 0.1
    epochs = 100
    LR_mean = 1e-3
    LR_lum = 1e-3
    init_techniques = ["uniform", "random"]
    number_of_splats = [2^i for i in 4:20]
    timekeeper = []

    # warmup
    for n in number_of_splats[1:4]
        for init_type in init_techniques
            outfile = "$out_dir/$init_type-sampling_$n-splats.png"
            train(infile, outfile, LR_mean, LR_lum, init_type, epochs, n, batch_size)
        end
    end
    
    # program begins
    for n in number_of_splats
        
        start_time = time()
        for init_type in init_techniques
            outfile = "$out_dir/$init_type-sampling_$n-splats.png"
            train(infile, outfile, LR_mean, LR_lum, init_type, epochs, n, batch_size)
        end
        elapsed_time = (time() - start_time) / 2
        push!(timekeeper, elapsed_time)
        
        # print Time
        print("$n splats. Time ==> ")
        println(elapsed_time)

        # save to clock.txt
        open("$out_dir/clock.txt", "w") do file
            for elem in timekeeper
                write(file, string(elem), "\n")
            end
        end
    end
end

end

