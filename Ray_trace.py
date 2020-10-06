# Finding exit points -> exit = True, if ray hits (0,0) or (max_x, max_y). False otherwhise. 
# Box edges: (max_x, max_y)

import math
def get_reflected_vector(head, tail, max_x, max_y):
    h0,h1 = head
    t0, t1 = tail
    # Incident vector centered at tail (0,0)
    inc = (h0-t0, h1-t1)
    ix, iy = inc
    # Normals for reflection at borders
    n = (0,0)
    if h1 == max_y: n = (0,-1)
    if h1 == 0: n = (0,1)
    if h0 == max_x: n = (-1,0)
    if h0 == 0: n = (1,0)
    nx,ny = n
    # Reflected vector head at tail (0,0)
    ref  = (ix - 2*(ix*nx + iy*ny)*nx, iy - 2*(ix*nx + iy*ny)*ny)
    # Normalized reflected vector at (0,0) for shortened reflections
    if abs(h0 - t0) < max_y:
        ref_norm = normalize_reflection(ref, h0, h1, max_y)
        head = ref_norm[0]
        tail = ref_norm[1]
    else:
        tail = head
        head = ref
    ref = [head, tail]
    return ref

# Normalizing and extending shortened vectors to keep the module constant for future border crossings.
# |ref| = |initial vector| = sqrt(2)*max_y
def normalize_reflection(ref, h0, h1, max_y):
    ref_u = unit_vector(ref)
    mod = math.sqrt(2)*max_y
    ref_norm = tuple([mod * x for x in ref_u])
    r_h0, r_h1 = ref_norm
    head = (round(r_h0) + h0 , round(r_h1) + h1)
    tail = (h0, h1)
    ref_norm = [head, tail]
    return ref_norm

def unit_vector (vector):
    v = math.sqrt(vector[0]**2 + vector[1]**2)
    u = (vector[0]/v, vector[1]/v)
    return u

# Finding crossing points with borders. Crossing point = head of shortened vector. Staying inside the box.
def border_reflection(head, tail, max_x, max_y):
    h0,h1 = head
    t0, t1 = tail
    # RIGHT BORDER
    if h0 > max_x:
        d = h0 - max_x
        h0 = max_x
        if h1 > t1: # incidence of positive slope
            h1 = h1 - d
        else: # negative slope
            h1 = h1 + d
        head = (h0, h1)
        ref = get_reflected_vector(head,tail,max_x,max_y)
        return ref
    # TOP BORDER
    if h1 > max_y:
        d = h1 - max_y
        h1 = max_y
        if h0 < t0: # incidence coming from the right
            h0 = h0 + d
        else: # incidence coming from the left
            h0 = h0 - d
        head = (h0, h1)
        ref = get_reflected_vector(head,tail,max_x,max_y)
        return ref
    # BOTTOM BORDER
    if h1 < 0:
        d = abs(h1)
        h1 = 0
        if h0 < t0: # incidence coming from the right
            h0 = h0 + d
        else: # incidence coming from the left
            h0 = h0 - d
        head = (h0, h1)
        ref = get_reflected_vector(head,tail,max_x,max_y)
        return ref
    # LEFT BORDER
    if h0 < 0:
        d = abs(h0)
        h0 = 0
        if h1 > t1:
            h1 = h1 - d
        else:
            h1 = h1 + d
        head = (h0, h1)
        ref = get_reflected_vector(head,tail,max_x,max_y)
        return ref

# Propagates vectors with constant module (that didn't get shortened by borders)
def propagate_vector(head, tail, max_x, max_y):
    h0,h1 = head
    ref = get_reflected_vector(head,tail,max_x,max_y)
    p_tail = head # new tail is last head
    h00, h11 = (ref[0][0], ref[0][1])
    p_head = (h0+h00, h1+h11)
    vp = [p_head, p_tail]
    return vp

# Validating conditions (0,0), (max_x, max_y): True. False otherwhise.
def exit(head, max_x, max_y):
    if head == (0,0) or head == (max_x, max_y): return 'true'
    if head == (0,max_y) or head == (max_x,0): return 'false'
    else: return 'continue'

# Propagating light vectors while checking conditions
def ray_trace(head, tail, max_x, max_y):
    while True:
        vp = propagate_vector (head, tail, max_x, max_y)
        head, tail = vp
        h0, h1 = head
        while h0 > max_x or h1 > max_y or h1 < 0 or h0 < 0:
            vr = border_reflection(head,tail,max_x,max_y)
            head, tail = vr
            h0,h1 = head
        exit1 = exit(head,max_x,max_y)
        if exit1 == 'true':
            return True
            break
        if exit1 == 'false':
            return False
            break
        else:
            pass

# Generates the start vector defined by initial constraints
def vector_generator(max_y):
    tail = (0,0)
    head = (max_y, max_y)
    v = [head, tail]
    return v

# Determines the orientation of the box. Runs everything else with given box size.
def reflections (max_x, max_y):
    if max_x == max_y:
        return True
    if max_x > max_y: pass
    if max_x < max_y:
        swap = max_y
        max_y = max_x
        max_x = swap
    start = vector_generator(max_y)
    head, tail = start
    rt = ray_trace(head, tail, max_x, max_y)
    return rt


if __name__ == '__main__':

    max_x, max_y = (10,2)
    rf = reflections(max_x, max_y)
    print(rf)
