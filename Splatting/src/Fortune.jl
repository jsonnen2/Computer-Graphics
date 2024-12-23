module Fortune

using ..Geometry
using ..EventQueue
using ..BeachLine
using ..Diagram

@enum SIDE LEFT RIGHT

function randf(start, finish, n)
   v = rand(n)
   return map(x -> start + x*(finish-start), v)
end

function init(points::Array{Tuple{Real, Real}, 1}, W, H)
	n = size(points)[1]

    V = Diagram.DCEL(points)
	T = BeachLine.BST()
	Q = EventQueue.Heap(3n) # 3n is enough, see page 166 on BCKO (de Berg, Cheong, Kreveld, Overmars)

    Geometry.setSafe(H)

	for region in V.regions
		EventQueue.push!(Q, EventQueue.SiteEvent(region))
	end

	return V, T, Q
end

function handleEvent(V::Diagram.DCEL, T::BeachLine.BST, Q::EventQueue.Heap, event::EventQueue.SiteEvent)
   #println("SITE EVENT: ", event.region.generator)
   #readline(stdin)
   ly = event.region.generator[2] # sweep line

   arc, arcAbove, sideOnSpecialCase = BeachLine.insert(T, event.region, ly)

   if T.root == arc
       return
   end

   if arcAbove == nothing # first couple of points have the same y
       bp = Geometry.parabolaIntersection(arc.parent.leftFocus, arc.parent.rightFocus, ly)

       if sideOnSpecialCase == BeachLine.LEFT
           left = arc
           right = arc.parent.rightChild
       else
           left = arc.parent.leftChild
           right = arc
       end

       #dir = Geometry.rotateVectorCCW(Geometry.subVector(left.region.generator, right.region.generator))
       dir = Geometry.rotateVectorCCW(Geometry.subVector(left.region.generator, right.region.generator))

       he1 = Diagram.HalfEdge(Geometry.subVector(bp, dir), false, nothing, nothing, nothing, left.region.generator)
       he2 = Diagram.HalfEdge(Geometry.addVector(bp, dir), false, nothing, nothing, nothing, right.region.generator)
       Diagram.makeTwins(he1, he2)
       Base.push!(V.halfEdges, he1)
       Base.push!(V.halfEdges, he2)
       left.parent.halfEdge = he1
       left.region.borderHead = he1 # following the convention that inside half-edges go counter-clockwise
       right.region.borderHead = he2
   else
       if arcAbove.disappearsAt != nothing # circle event for arc vertically above 'arc' was a false alarm
          EventQueue.remove(Q, arcAbove.disappearsAt)
          arcAbove.disappearsAt = nothing
       end

       rightBreakpoint = arc.parent
       leftBreakpoint = rightBreakpoint.parent

       # create half edge records for each new breakpoint
       f = Geometry.parabola(arcAbove.region.generator, ly)
       x = event.region.generator[1]

       if f != nothing
           breakpoint = (x, f(x))
       else
           breakpoint = nothing
       end

       # direction of the halfedge, rotating clockwise around the arc above (always going right)
       dir = Geometry.rotateVectorCCW(Geometry.subVector(arc.region.generator, arcAbove.region.generator))

       he1 = Diagram.HalfEdge(Geometry.subVector(breakpoint, dir), false, nothing, nothing, nothing, arcAbove.region.generator)
       he2 = Diagram.HalfEdge(Geometry.addVector(breakpoint, dir), false, nothing, nothing, nothing, event.region.generator)
       Diagram.makeTwins(he1, he2)
       Base.push!(V.halfEdges, he1)
       Base.push!(V.halfEdges, he2)
       leftBreakpoint.halfEdge = he1
       rightBreakpoint.halfEdge = he2
       event.region.borderHead = he2 # following the convention that inside half-edges go counter-clockwise
       arcAbove.region.borderHead = he1
   end

   # check for new circle events where 'arc' is the rightmost arc
   c = arc
   b = arc.prev
   if b != nothing
      a = b.prev

      # there's a triple with 'arc' on the rightmost position
      if a != nothing && Geometry.arcWillConverge(a.region.generator, b.region.generator, c.region.generator)
         O, r = Geometry.circumcircle(a.region.generator, b.region.generator, c.region.generator)

         ev = EventQueue.CircleEvent(O, r, b)
         b.disappearsAt = ev
         EventQueue.push!(Q, ev)
      end
   end

   # check for new circle events where 'arc' is the leftmost arc
   a = arc
   b = arc.next
   if b != nothing
      c = b.next

      # there's a triple with 'arc' on the leftmost position
      if c != nothing && Geometry.arcWillConverge(a.region.generator, b.region.generator, c.region.generator)
         O, r = Geometry.circumcircle(a.region.generator, b.region.generator, c.region.generator)

         ev = EventQueue.CircleEvent(O, r, b)
         b.disappearsAt = ev
         EventQueue.push!(Q, ev)
      end
   end
end

function handleEvent(V::Diagram.DCEL, T::BeachLine.BST, Q::EventQueue.Heap, event::EventQueue.CircleEvent)
   #println("CIRCLE EVENT: ", event.coordinates, "; ", event.center)
   #readline(stdin)
   arc = event.disappearingArc

   # find the Voronoi edges that are going to join together
   leftBreakpoint = rightBreakpoint = arc.parent
   while leftBreakpoint.leftFocus != arc.prev.region.generator ||
         leftBreakpoint.rightFocus != arc.region.generator
      leftBreakpoint = leftBreakpoint.parent
   end
   while rightBreakpoint.leftFocus != arc.region.generator ||
         rightBreakpoint.rightFocus != arc.next.region.generator
      rightBreakpoint = rightBreakpoint.parent
   end

   # fix the extremes of the joining Voronoi edges
   leftBreakpoint.halfEdge.origin = rightBreakpoint.halfEdge.origin = event.center
   leftBreakpoint.halfEdge.isFixed = rightBreakpoint.halfEdge.isFixed = true

   # remembering breakpoint edges, because BeachLine.remove will likely change the breakpoints
   lBhe = leftBreakpoint.halfEdge
   rBhe = rightBreakpoint.halfEdge

   arc.prev.next = arc.next
   arc.next.prev = arc.prev
   newBreakpoint = BeachLine.remove(T, event.disappearingArc)

   # direction of the new halfedge, rotating clockwise around the arc to the right of the disappearing arc (always going down)
   dir = Geometry.rotateVectorCCW(Geometry.subVector(arc.prev.region.generator, arc.next.region.generator))
   #dir = Geometry.multVector(1000, dir)

   # create new half edge for the newly formed breakpoint
   he1 = Diagram.HalfEdge(Geometry.addVector(event.center, dir), false, nothing, nothing, nothing, arc.prev.region.generator)
   he2 = Diagram.HalfEdge(event.center, false, nothing, nothing, nothing, arc.next.region.generator)
   Diagram.makeTwins(he1, he2)
   Base.push!(V.halfEdges, he1)
   Base.push!(V.halfEdges, he2)
   newBreakpoint.halfEdge = he1 # he2 is left bound to the vertex
   he2.isFixed = true

   # join the adjacent edges in the region lists
   Diagram.concat(newBreakpoint.halfEdge, lBhe)
   Diagram.concat(rBhe.twin, newBreakpoint.halfEdge.twin)
   Diagram.concat(lBhe.twin, rBhe)

   # remove circle events of triples that don't exist anymore
   if arc.prev.disappearsAt != nothing
      EventQueue.remove(Q, arc.prev.disappearsAt)
      arc.prev.disappearsAt = nothing
   end
   if arc.next.disappearsAt != nothing
      EventQueue.remove(Q, arc.next.disappearsAt)
      arc.next.disappearsAt = nothing
   end

   # check the left new triple of adjacent arcs for circle events
   if arc.prev != nothing && arc.next != nothing && arc.prev.prev != nothing
      a = arc.prev.prev
      b = arc.prev
      c = arc.next

      if Geometry.arcWillConverge(a.region.generator, b.region.generator, c.region.generator)
         O, r = Geometry.circumcircle(a.region.generator, b.region.generator, c.region.generator)

         ev = EventQueue.CircleEvent(O, r, b)
         b.disappearsAt = ev
         EventQueue.push!(Q, ev)
      end
   end

   # check the right new triple of adjacent arcs for circle events
   if arc.prev != nothing && arc.next != nothing && arc.next.next != nothing
      a = arc.prev
      b = arc.next
      c = arc.next.next

      if Geometry.arcWillConverge(a.region.generator, b.region.generator, c.region.generator)
         O, r = Geometry.circumcircle(a.region.generator, b.region.generator, c.region.generator)

         ev = EventQueue.CircleEvent(O, r, b)
         b.disappearsAt = ev
         EventQueue.push!(Q, ev)
      end
   end
end

function compute(points::Array{Tuple{Real, Real}, 1}, W, H)
   # compute Voronoi tesselation

   V, T, Q = init(points, W, H)

   while (event = EventQueue.pop(Q)) != nothing
      Fortune.handleEvent(V, T, Q, event)
   end

   return V
end


end # module
