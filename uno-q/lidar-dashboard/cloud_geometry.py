"""Display-only connectivity. No synthesized measurement points are returned."""
import numpy as np
WIDTH, HEIGHT = 54, 42
_y,_x=np.mgrid[:HEIGHT-1,:WIDTH-1]
_a=(_y*WIDTH+_x).ravel()
TRIANGLES=np.concatenate((np.column_stack((_a,_a+WIDTH,_a+1)),
                          np.column_stack((_a+1,_a+WIDTH,_a+WIDTH+1)))).astype(np.uint32)

def display_mask(xyz, valid, confidence=None, minimum=0.0):
    mask=np.asarray(valid,dtype=bool).copy() & np.isfinite(xyz).all(axis=1)
    if confidence is not None:
        mask &= np.isfinite(confidence) & (confidence >= minimum)
    return mask

def surface_triangles(xyz, valid, max_depth_jump=0.15):
    """Only adjacent valid cells; reject edges crossing a depth discontinuity (metres)."""
    t=TRIANGLES[np.all(valid[TRIANGLES],axis=1)]
    if not len(t):return t
    v=xyz[t]
    good=np.isfinite(v).all(axis=(1,2)) & (np.ptp(v[:,:,2],axis=1)<=max_depth_jump)
    # Also reject stretched edges (near grazing-angle geometry).
    edges=np.stack((v[:,0]-v[:,1],v[:,1]-v[:,2],v[:,2]-v[:,0]),axis=1)
    good &= (np.linalg.norm(edges,axis=2).max(axis=1)<=max_depth_jump*2)
    return np.ascontiguousarray(t[good])
