import ast
from copy import deepcopy
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from projects import export_archive, import_archive, read_archive, MEMBER_LIMITS
from webapp import create_server


def make_simulation(root):
    tree = ast.parse((ROOT / 'server.py').read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Simulation')
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('__init__', 'load_path', 'info')]
    namespace = dict(np=np, ROOT=root, deepcopy=deepcopy, json=json,
                     catalog=lambda: [{}], TERRAIN_DEFAULTS={})
    exec(compile(ast.Module(body=[cls], type_ignores=[]), 'server.py', 'exec'), namespace)
    return namespace['Simulation']()


class PortableProjectTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / 'tests')
        self.root = Path(self.temp.name)
        self.source = self.root / 'original.ply'
        self.source.write_text('ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nproperty int classification\nend_header\n0 0 0 1\n20 0 1 2\n0 20 2 1\n', encoding='ascii')
        self.source.with_suffix('.ply.labels.json').write_text('{"1":"grava","2":"roca"}', encoding='ascii')
        self.sim = make_simulation(self.root)
        self.sim.load_path(self.source)
        self.csv = self.root / 'results.csv'
        self.csv.write_bytes(b'time,x,y,z\n0,1,2,3\n')
        self.sim.csv_path = str(self.csv)
        self.project = dict(csv_available=True, applied_pose={'position':[1,2,3], 'orientation':[1,0,0,0]}, playback_index=0, vehicle='MAN_10t', pose={'x': 1, 'y': 2, 'z': 3},
                            route=[dict(x=1,y=2,z=3,speed=1,acceleration=.5)],
                            terrain={'classes': {'grava': {'friction': .65, 'stiffness': 800000}},
                                     'zones': [{'center':[1,2,3], 'radius':2, 'class':'grava'}]},
                            frames=[{'time':0,'position':[1,2,3],'orientation':[1,0,0,0],
                                     'points':[[1,2,0]],'normal':[[0,0,100]],'contact':[1]}],
                            trajectory=[{'time':0,'position':[1,2,3],'orientation':[1,0,0,0]}],warnings='')

    def tearDown(self):
        if self.sim.store:
            self.sim.store.close()
        self.temp.cleanup()

    def archive(self):
        return export_archive(self.project, self.sim.store.metadata, self.sim.csv_path)

    def repack(self, raw, transform):
        with zipfile.ZipFile(io.BytesIO(raw)) as z:
            members={name:z.read(name) for name in z.namelist()}
        transform(members)
        output=io.BytesIO()
        with zipfile.ZipFile(output,'w') as z:
            for name,data in members.items(): z.writestr(name,data)
        return output.getvalue()

    def test_portable_roundtrip_without_original_source_or_csv(self):
        expected=self.sim.info()
        raw=self.archive()
        self.source.unlink()
        self.source.with_suffix('.ply.labels.json').unlink()
        self.csv.unlink()
        loaded=import_archive(self.sim,raw,self.root/'restored')
        self.assertEqual(loaded['info']['points'],expected['points'])
        self.assertEqual(loaded['info']['labels'],expected['labels'])
        self.assertEqual(loaded['info']['cloud']['label_mapping'],{'1':'grava','2':'roca'})
        for key,value in self.project.items(): self.assertEqual(loaded['project'][key],value)
        self.assertTrue(loaded['csv_available'])
        self.assertEqual(Path(self.sim.csv_path).read_bytes(),b'time,x,y,z\n0,1,2,3\n')
        # A restored portable project can itself be saved again.
        self.assertTrue(export_archive(loaded['project'],self.sim.store.metadata,self.sim.csv_path))

    def test_rejects_corruption_and_unknown_archive_paths_without_state_changes(self):
        original=self.sim.store
        raw=self.archive()
        bad_cloud=self.repack(raw,lambda m:m.update({'cloud.ply':b'invalid'}))
        traversal=self.repack(raw,lambda m:m.update({'../outside.ply':b'bad'}))
        for payload in (b'not a zip',bad_cloud,traversal):
            with self.subTest(payload=len(payload)),self.assertRaises(ValueError):
                import_archive(self.sim,payload,self.root/'restored')
            self.assertIs(self.sim.store,original)
        self.assertFalse((self.root/'outside.ply').exists())

    def test_rejects_invalid_pose_and_oversized_member(self):
        raw=self.archive()
        def invalid(members):
            doc=json.loads(members['project.json']);doc['project']['pose']={'x':'bad'}
            members['project.json']=json.dumps(doc).encode()
        with self.assertRaises(ValueError):read_archive(self.repack(raw,invalid))
        with patch.dict(MEMBER_LIMITS,{'cloud.ply':1}),self.assertRaises(ValueError):read_archive(raw)
        bad=deepcopy(self.project);bad['frames'][0]['points']=['bad']
        with self.assertRaises(ValueError):export_archive(bad,self.sim.store.metadata)

    def test_no_stale_csv_when_frontend_has_cleared_results(self):
        project=deepcopy(self.project);project['csv_available']=False;project['frames']=[]
        raw=export_archive(project,self.sim.store.metadata,self.sim.csv_path)
        _,_,csv_data=read_archive(raw)
        self.assertIsNone(csv_data)
        restored=import_archive(self.sim,raw,self.root/'restored')
        self.assertFalse(restored['csv_available'])
        self.assertIsNone(self.sim.csv_path)

    def test_stability_values_parameters_and_unavailable_reasons_survive_roundtrip(self):
        stability = {
            'ltr': {'value': -.25, 'params': {'fz_left_N': 50., 'fz_right_N': 30.}, 'reason': None},
            'pltr': {'value': None, 'params': {'horizon_s': .5, 'ay_dot_m_s3': None},
                     'reason': 'Se necesita una muestra anterior'},
            'zml': {'value': [.1, -.2], 'params': {'z_ref_m': -1.2, 'fz_N': 80.}, 'reason': None},
            'si': {'value': -.4, 'params': {'mass_kg': 7085., 'k1': 1.15}, 'reason': None},
        }
        self.project['frames'][0]['stability'] = stability
        restored = import_archive(self.sim, self.archive(), self.root/'restored')
        self.assertEqual(restored['project']['frames'][0]['stability'], stability)

    def test_rejects_changed_original_source_on_export(self):
        self.source.write_bytes(b'modified')
        with self.assertRaises(ValueError):self.archive()

    def test_http_download_import_and_removed_config_routes(self):
        server=create_server(self.sim,self.root)
        try:
            # Exercise the actual HTTP handler with in-memory streams. Loopback
            # clients are not permitted by every desktop test environment.
            def post(path,raw):
                handler=server.RequestHandlerClass.__new__(server.RequestHandlerClass)
                handler.path=path
                handler.headers={'Host':f'127.0.0.1:{server.server_port}',
                                 'Content-Length':str(len(raw))}
                handler.rfile=io.BytesIO(raw);handler.wfile=io.BytesIO()
                captured={}
                handler.send_response=lambda code:captured.update(status=code)
                handler.send_header=lambda key,value:captured.update({key:value})
                handler.end_headers=lambda:None
                handler.do_POST()
                return captured,handler.wfile.getvalue()
            headers,raw=post('/api/project/export',json.dumps({'project':self.project}).encode())
            self.assertEqual(headers['status'],200)
            self.assertEqual(headers['Content-Type'],'application/zip')
            self.assertIn('.fusionforce',headers['Content-Disposition'])
            self.source.unlink();self.csv.unlink()
            headers,body=post('/api/project/import',raw)
            loaded=json.loads(body)
            self.assertEqual(headers['status'],200)
            self.assertTrue(loaded['csv_available'])
            self.assertEqual(loaded['project']['frames'],self.project['frames'])
            for path in ('/api/config/save','/api/config/load'):
                headers,_=post(path,b'{}');self.assertEqual(headers['status'],400)
        finally:
            server.server_close()


if __name__=='__main__':unittest.main()
