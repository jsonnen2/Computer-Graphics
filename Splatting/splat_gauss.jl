
module splat_gauss

export train

using Images
using ImageIO
using StaticArrays
using LinearAlgebra
using Random


# My gsplats set x and y to be independent. 
mutable struct gsplat 
    center::SVector{2, Int}
    color::RGB{Float32}
    scale_x::Float32
    scale_y::Float32
    luminance::Float64
end


function sample_gsplats(canvas::Matrix{RGB{Float32}}, init_type::String, num_blobs::Int)
    
    height, width = size(canvas)
    uniform_step_size = height * width / num_blobs
    gsplat_bag = Vector{gsplat}()
    scale = 2

    for n in 1:num_blobs

        if init_type == "random"
            cx, cy = rand(1:width), rand(1:height)
            center = SVector{2, Int}(cx, cy) # always integer
        elseif init_type == "uniform"
            c = Int(floor(n * uniform_step_size))
            cy = Int(ceil(c / width))
            cx = c - (cy-1) * width 
            center = SVector{2, Int}(cx, cy)
        else
            error("Unsupported init_type = $init_type")
        end
        
        color = canvas[center[2], center[1]]
        alpha = max(color.r, color.g, color.b)
        color /= alpha

        push!(gsplat_bag, gsplat(center, color, scale, scale, alpha))
    end

    return gsplat_bag
end


function forward_pass(gsplat_bag::Vector{gsplat}, height::Int, width::Int)

    canvas = fill(RGB{Float32}(0,0,0), height, width)
    
    for blob in gsplat_bag
        
        # specify a probability theshold for the Gaussian 
        # only evaluate points which are "close enough" to the center
        p = 1e-3
        if isnan(blob.scale_x) || isnan(blob.scale_y)
            println("one of blob scales are NaN.")
            println(blob)
        end
        offset = Int(round((max(blob.scale_x, blob.scale_y, 0.0) * -log(p))))
        cx, cy = blob.center

        for h in max(1, cy-offset) : min(height, cy+offset)
            for w in max(1, cx-offset) : min(width, cx+offset)

                # Evaluate Gaussian
                x = w - blob.center[1]
                y = h - blob.center[2]
                scale = (blob.scale_x * blob.scale_y) / (2*pi)
                gauss = scale * exp(-0.5 * ((x / blob.scale_x)^2 + (y / blob.scale_y)^2))

                gauss = min(1, gauss)
                color = gauss .* blob.luminance .* blob.color 
                canvas[h, w] += color

            end
        end
    end
    return canvas
end


function update_splats!(gsplat_bag, gradients, LR_scale, LR_lum)

    for (idx, blob) in enumerate(gsplat_bag)
        new_luminance = blob.luminance + LR_lum * gradients[1][idx]
        new_scale_x = blob.scale_x + LR_scale * gradients[2][idx]
        new_scale_y = blob.scale_y + LR_scale * gradients[3][idx]

        new_luminance = clamp(new_luminance, 0.0, 1.0)
        new_scale_x = clamp(new_scale_x, 0.0, 100.0)
        new_scale_y = clamp(new_scale_y, 0.0, 100.0)

        # sometimes my blob parameters become NaN after a lot of training
        # So I don't update params if update trys to set them to NaN.
        if !isnan(new_luminance)
            blob.luminance = new_luminance
        end
        if !isnan(new_scale_x)
            blob.scale_x = new_scale_x
        end
        if !isnan(new_scale_y)
            blob.scale_y = new_scale_y
        end
    end
end


function calc_gradients(loss, gsplat_bag)
    # calculate gradients for luminance, scale_x, scale_y
    # for batched learning, pass in a subset of gsplat_bag. Use that same subset in update_splats!()

    height, width = size(loss)
    grad_luminance = fill(0.0f0, size(gsplat_bag))
    grad_scale_x = fill(0.0f0, size(gsplat_bag))
    grad_scale_y = fill(0.0f0, size(gsplat_bag))

    for (idx, blob) in enumerate(gsplat_bag)

        cx, cy = blob.center
        p = 1e-3

        offset = Int(round(max(blob.scale_x, blob.scale_y, 0.0) * -log(p)))

        for h in max(1, cy-offset) : min(height, cy+offset)
            for w in max(1, cx-offset) : min(width, cx+offset)

                x = w - blob.center[1]
                y = h - blob.center[2]
                scale = 1 / (2*pi)
                z = scale * exp(-0.5 * ((x / blob.scale_x)^2 + (y / blob.scale_y)^2))

                alpha = blob.scale_x * blob.scale_y 
                sx = blob.scale_y * (1 + (x / blob.scale_x)^2)
                sy = blob.scale_x * (1 + (y / blob.scale_y)^2)
                
                distance = loss[h,w].r * blob.color.r +
                            loss[h,w].g * blob.color.g +
                            loss[h,w].b * blob.color.b

                grad_luminance[idx] += distance * z * alpha
                grad_scale_x[idx] += distance * z * sx * blob.luminance
                grad_scale_y[idx] += distance * z * sy * blob.luminance

            end
        end

    end
    return [grad_luminance, grad_scale_x, grad_scale_y]
end


function train(infile::String, outfile::String, LR_scale, LR_lum, init_type, epochs, num_blobs, batch_size)

    # load image
    img = load(infile)
    canvas = float.(img)
    canvas = convert(Matrix{RGB{Float32}}, canvas)
    height, width = size(canvas)

    # init gsplats
    gsplat_bag = sample_gsplats(canvas, init_type, num_blobs)
    start_time = time()

    # training loop
    for epoch in 1:epochs

        # forward pass
        forward = forward_pass(gsplat_bag, height, width)

        # loss
        loss = (canvas .- forward) # TODO: new loss functions

        # batch blobs 
        n = length(gsplat_bag)
        sample_size = Int(floor(batch_size * n))
        batch_idx = shuffle(1:n)[1:sample_size]

        # update gradients
        gradients = calc_gradients(loss, gsplat_bag[batch_idx])
        update_splats!(gsplat_bag[batch_idx], gradients, LR_scale, LR_lum)

        # periodicly print updates
        # if epoch % 100 == 0
        #     println("Epoch= ", epoch, ", Loss= ", sum(loss))
        # end

        # periodicly save progress
        if epoch % 1000 == 0
            
            # create and save image
            img = clamp01.(forward_pass(gsplat_bag, height, width))
            save_dir = "$outfile-epoch=$epoch.png"
            dir_path = dirname(save_dir)
            if !isdir(dir_path)
                mkpath(dir_path)
            end
            save(save_dir, img)
            println(save_dir)

            # print Loss
            loss = sum(sum(loss, dims=1), dims=2)
            println("Loss ==> $loss")

            # print Time
            time_elapsed = time() - start_time
            start_time = time()
            println("Time ==> $time_elapsed")
        end
    end
end

function control_panel()

    infile = "imgs/source/landscape.jpg"
    out_dir = "imgs/gsplat_random_learn_scale_luminance"
    LR_scale = 1e-3
    LR_lum = 1e-4
    batch_sizes = [0.1]
    init_techniques = ["random"]
    epochs = [1e6]
    num_blobs = [5000]

    for blob in num_blobs
        for epoch in epochs
            for batch_size in batch_sizes
                for init_type in init_techniques

                    outfile = "$out_dir/init=$init_type-blobs=$blob-batch=$batch_size-LR_scale=$LR_scale-LR_lum=$LR_lum"
                    train(infile, outfile, LR_scale, LR_lum, init_type, epoch, blob, batch_size)

                end
            end
        end
    end
end


end