""" The Cameras module is responsible for ray generation. It defines
types for each kind of camera (in the base assignment, this only
includes CanonicalCamera) and implements a pixel_to_ray function
that takes a camera and a set of pixel indices and returns a ray
along which to search for objects that appear at that pixel.
"""
module Cameras

import LinearAlgebra.normalize
import LinearAlgebra.norm
import LinearAlgebra.cross

#push!(LOAD_PATH, pwd())
using ..GfxBase

export CanonicalCamera
export pixel_to_ray

""" The most basic perspective camera. The eye is at (0, 0, 0),
the up vector is (0, 1, 0), and the view direction is (0, 0, -1).
The viewport has width 1 and whatever height makes pixels square.
The focal length (distance from eye to viewport) is 1. """
mutable struct CanonicalCamera
    canv_height::Int
    canv_width::Int
end


""" Given a camera and pixel coordinates 1-indexed, i=row,j=column],
return the ray along which to search for objects that project to
that pixel. """
function pixel_to_ray(camera::CanonicalCamera, i, j)
    # viewport height = 1
    vp_height = camera.canv_height / camera.canv_width

    # convert from i,j 2D array indices to (u, v) coordinates
    # with (0,0) in the center of the image
    # half pixel shift, scale down to size 1, shift origin to center
    u = ((j - 0.5) / camera.canv_width - 0.5)

    # same as u, except i increases downward so flip with a negative sign
    # and multiply by vp_height to account for aspect ratio
    v = -((i - 0.5) / camera.canv_height - 0.5) * vp_height

    # focal length is 1, pointing down the -z axis
    w = -1.0

    Ray(Vec3(0, 0, 0), Vec3(u, v, w))
end

""" A general perspective camera."""
mutable struct PerspectiveCamera

    # orthonormal basis specifies position and orientation:
    eye::Vec3  # position of the eye
    u_axis::Vec3 # points right in image space
    v_axis::Vec3 # points up in image space
    w_axis::Vec3 # opposite the view direction

    # the viewport is width 1, parallel to the uv plane and
    # centered at (0, 0, -focal):
    focal::Real # focal length (aka d), the distance to the image plane

    # image dimensions:
    canv_height::Int # image height in pixels
    canv_width::Int # image width in pixels
end

function PerspectiveCamera(eye::Vec3, view::Vec3, up::Vec3, focal::Real, canv_height::Int, canv_width::Int)

    ###########
    # TODO 8a #
    ###########
    # Your implementation:
    #
    w_axis = -normalize(view) # Normalize view direction
    u_axis = cross(up, w_axis) # Normalize u-axis to correct direction
    v_axis = cross(w_axis, u_axis) # Normalize v-axis to correct direction

    return PerspectiveCamera(eye, u_axis, v_axis, w_axis, focal, canv_height, canv_width) # Return new perspective camera
    ###############
    # END TODO 8a #
    ###############

end

function pixel_to_ray(camera::PerspectiveCamera, i, j)
    ##########
    # TODO 8b #
    ##########
    # Your implementation:
    height = camera.canv_height # Get camera height
    width = camera.canv_width # Get camera width

    # Get camera viewing rays
    u = (j - 0.5) / height - 0.5
    v = (i - 0.5) / width - 0.5
    w = camera.focal
    view_ray = -w * camera.w_axis + u * camera.u_axis - v * camera.v_axis
    return Ray(camera.eye, view_ray) # Cast new viewing ray
    ###############
    # END TODO 8b #
    ###############
end


end # module Cameras



