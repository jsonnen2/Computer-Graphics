### Main Program: parse args and call main() ###
using ArgParse

using Rays

""" parse_cmdline
Parse the command line args to specify scene, camera, and image size """
function parse_cmdline()
    s = ArgParseSettings()
    @add_arg_table s begin
        "--scene", "-s"
            help="scene"
            arg_type=Int
            default=1
        "--camera", "-c"
            help="camera"
            arg_type=Int
            default=1
        "height"
            help="image height"
            arg_type=Int
        "width"
            help="image width"
            arg_type=Int
        "outfile"
            help="output filename"
    end

    return parse_args(s)
end

a = parse_cmdline()

Rays.main(a["scene"], a["camera"], a["height"], a["width"], a["outfile"])
