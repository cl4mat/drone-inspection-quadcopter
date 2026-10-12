#!/usr/bin/env python3
"""Synthetic measured-voxel fixtures, not flight or localization acceptance."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from building_route import BuildingMap,atomic_json,build,skeleton,route_window


class BuildingTests(unittest.TestCase):
    def fixture(self,path,two=False):
        shape=(60,50,85);occupied=np.zeros(shape,bool);free=np.zeros(shape,bool)
        occupied[0,:,:]=occupied[-1,:,:]=True;occupied[:,0,:]=occupied[:,-1,:]=True
        occupied[:,:,3]=True;occupied[:,:,75 if two else 39]=True
        free[1:-1,1:-1,4:(75 if two else 39)]=True
        if two:
            occupied[:,:,39]=True;free[:,:,39]=False
            # No stair hole: both floors are observed but disconnected.
        free[occupied]=False
        ids=np.flatnonzero(occupied|free)
        with path.open('w') as f:
            f.write('DRONE_GRID_V2 odom .1 -3 -2.5 -.5 60 50 85 %d 0 0 0 0\n'%len(ids))
            for i in ids:f.write('%d %d %d\n'%(i,2 if occupied.flat[i] else 0,int(free.flat[i])))

    def test_single_floor_closed_task(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'measured.dmap';self.fixture(path)
            grid=BuildingMap(path);topology=grid.topology()
            self.assertEqual(len(topology['floors']),1)
            task=build(path,required_floors=1)
            self.assertEqual(task['state'],'READY');self.assertGreater(task['length_m'],5)
            np.testing.assert_allclose(task['segments'][-1]['points'][-1],task['home_approach'])
            self.assertTrue(Path(str(path)+'.inspection.json').exists())

    def test_disconnected_floors_draft(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'measured.dmap';self.fixture(path,True)
            grid=BuildingMap(path);self.assertEqual(len(grid.topology()['floors']),2)
            with self.assertRaises(ValueError):build(path)
            self.assertEqual(json.loads(Path(str(path)+'.inspection.json').read_text())['state'],'DRAFT')

    def test_hash_fence(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'measured.dmap';self.fixture(path)
            grid=BuildingMap(path);t=grid.topology();t['map_digest']='0'*64
            with self.assertRaises(ValueError):grid.generate(t)

    def test_unknown_not_passable(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'measured.dmap';self.fixture(path)
            grid=BuildingMap(path);grid.safe[:]=False
            with self.assertRaises(ValueError):grid.nearest([0,0,1.05])

    def test_reversal_split(self):
        grid=BuildingMap.__new__(BuildingMap)
        parts=grid.split_turns([(1,1,2),(2,1,2),(3,1,2),(2,1,2),(1,1,2)])
        self.assertEqual(len(parts),2);self.assertEqual(parts[0][-1],parts[1][0])

    def test_skeleton_inside_mask(self):
        mask=np.zeros((30,30),bool);mask[3:25,5:20]=True
        sk=skeleton(mask);self.assertTrue(sk.any());self.assertFalse((sk&~mask).any())

    def test_long_edge_window(self):
        window=route_window([[0,0,1],[20,0,1]],2.,10.)
        np.testing.assert_allclose(window,[[2,0,1],[12,0,1]])

    def test_one_floor_not_ready_for_two_floor_scene(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'measured.dmap';self.fixture(path)
            with self.assertRaises(ValueError):build(path)
            self.assertEqual(json.loads(Path(str(path)+'.inspection.json').read_text())['state'],'DRAFT')

    def test_supported_stair_path_cannot_use_free_air_shortcut(self):
        grid=BuildingMap.__new__(BuildingMap);grid.shape=(40,6,50);grid.safe=np.ones(grid.shape,bool)
        grid.supported=np.zeros(grid.shape,bool);grid.support_deviation=np.zeros(grid.shape,np.uint8)
        for x in range(37):grid.supported[x,:,9+x:12+x]=True
        route=grid.astar((0,3,10),(36,3,46),supported=True)
        self.assertTrue(all(grid.supported[k] for k in route))
        grid.supported[20,:,:]=False
        with self.assertRaises(ValueError):grid.astar((0,3,10),(36,3,46),supported=True)

    def test_folded_landing_stops_at_shared_midpoint(self):
        grid=BuildingMap.__new__(BuildingMap)
        parts=grid.split_turns([(0,0,1),(0,10,1),(10,10,1),(10,0,1)])
        self.assertEqual(len(parts),2);self.assertEqual(parts[0][-1],(5,10,1));self.assertEqual(parts[0][-1],parts[1][0])

    def test_atomic_json(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'task.json';atomic_json(path,dict(state='DRAFT'));atomic_json(path,dict(state='READY'))
            self.assertEqual(json.loads(path.read_text())['state'],'READY')
            self.assertEqual(len(list(Path(folder).iterdir())),1)

    def test_body_support_not_wall(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'measured.dmap';self.fixture(path)
            grid=BuildingMap(path)
            self.assertTrue(grid.supported[30,25,15]);self.assertFalse(grid.supported[30,25,28])


if __name__=='__main__':unittest.main()
