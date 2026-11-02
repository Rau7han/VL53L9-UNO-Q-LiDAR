import unittest
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from cloud_geometry import display_mask,surface_triangles,WIDTH,HEIGHT
class GeometryTests(unittest.TestCase):
    def setUp(self):
        y,x=np.mgrid[:HEIGHT,:WIDTH]
        self.xyz=np.column_stack((x.ravel()*.01,y.ravel()*.01,np.full(WIDTH*HEIGHT,-2))).astype(np.float32)
        self.valid=np.ones(WIDTH*HEIGHT,dtype=bool)
    def test_full_grid_only_uses_measured_indices(self):
        tris=surface_triangles(self.xyz,self.valid)
        self.assertEqual(len(tris),2*(WIDTH-1)*(HEIGHT-1))
        self.assertLess(int(tris.max()),2268)
    def test_invalid_hole_never_bridged(self):
        self.valid[1000]=False
        self.assertNotIn(1000,surface_triangles(self.xyz,self.valid))
    def test_depth_discontinuity_not_bridged(self):
        self.xyz[np.arange(2268)%WIDTH>26,2]=-3
        t=surface_triangles(self.xyz,self.valid)
        self.assertTrue(np.all(np.ptp(self.xyz[t,2],axis=1)==0))
    def test_confidence_only_removes_points(self):
        self.valid[4]=False
        conf=np.ones(2268);conf[2]=.3;conf[3]=np.nan
        out=display_mask(self.xyz,self.valid,conf,.5)
        self.assertFalse(out[2] or out[3] or out[4])
        self.assertEqual(out.sum(),2265)
    def test_nonfinite_geometry_excluded(self):
        self.xyz[0,0]=np.nan
        mask=display_mask(self.xyz,self.valid)
        self.assertFalse(mask[0]);self.assertNotIn(0,surface_triangles(self.xyz,mask))
