#!/usr/bin/env python3
"""Exact static geometry checks; not a Gazebo dynamics/LIO acceptance test."""
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from generate_building_world import generate


class SceneTests(unittest.TestCase):
    def setUp(self):
        self.path=Path(__file__).resolve().parents[1]/'worlds/inspection_building.world'
        self.world=ET.parse(self.path).getroot().find('world');self.boxes={}
        for model in self.world.findall('model'):
            link=model.find('link');pose=np.array(list(map(float,link.findtext('pose').split())))[:3]
            size=np.array(list(map(float,link.findtext('collision/geometry/box/size').split())))
            self.boxes[model.attrib['name']]=(pose,size)

    def test_reproducible(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'world.sdf';generate(path);self.assertEqual(path.read_bytes(),self.path.read_bytes())

    def test_floors_have_ceilings(self):
        self.assertIn('upper_slab_west',self.boxes);self.assertIn('roof',self.boxes)
        self.assertAlmostEqual(self.boxes['upper_slab_west'][0][2]+.1,3.6)
        self.assertAlmostEqual(self.boxes['roof'][0][2]-.05,7.1)

    def test_stair_clear_width_and_platform(self):
        left=self.boxes['rail_5.15_0'];centre=self.boxes['rail_6.8_0'];right=self.boxes['rail_8.45_0']
        self.assertAlmostEqual((centre[0][0]-centre[1][0]/2)-(left[0][0]+left[1][0]/2),1.5)
        self.assertAlmostEqual((right[0][0]-right[1][0]/2)-(centre[0][0]+centre[1][0]/2),1.5)
        np.testing.assert_allclose(self.boxes['stair_mid_landing'][1][:2],[3.2,2.])

    def test_two_flights_have_24_steps(self):
        self.assertEqual(len([n for n in self.boxes if n.startswith('stair_up_')]),12)
        self.assertEqual(len([n for n in self.boxes if n.startswith('stair_return_')]),12)
        self.assertAlmostEqual(self.boxes['stair_return_11'][1][2],3.6)

    def test_nominal_stair_profile_clearance(self):
        # Geometry-only nominal vehicle envelope: .5m XY/.4m Z safety extent.
        # These samples are NEVER sent to flight control or used to build a map.
        samples=[]
        for i in range(12):samples.append([5.95,-1.625-i*.25,(i+1)*.15+1.2])
        for x in np.linspace(5.95,7.65,15):samples.append([x,-5.4,3.0])
        for i in range(12):samples.append([7.65,-4.375+i*.25,1.8+(i+1)*.15+1.2])
        for point in samples:
            for name,(centre,size) in self.boxes.items():
                self.assertFalse(np.all(np.abs(np.array(point)-centre)<size/2+[.5,.5,.4]),(point,name))


if __name__=='__main__':unittest.main()
