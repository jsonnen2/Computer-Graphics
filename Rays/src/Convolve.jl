
export blur

using FileIO
using Images
using StaticArrays
using LinearAlgebra


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

# include("Convolve.jl")
# Convolve.blur("results/Convolutions/treebranch.jpg", "results/Convolutions", "box_blur_3")
function blur(infile::String, out_dir::String, conv::String)
    """
    Allowed Convolutions
     - brighten_{Float}
     - box_blur_{odd, positive int}
     - gauss_blur_{odd, positive int}
     - edge_detect_1
     - edge_detect_2
    """

    # load image
    img = load(infile)
    canvas = float.(img)
    canvas = convert(Matrix{RGB{Float32}}, canvas)

    apply_convolution!(canvas, conv)

    canvas = map(clamp01nan, canvas)
    outfile = "$out_dir/$conv.png"
    save(outfile, canvas)
end