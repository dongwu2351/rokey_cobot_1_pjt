"""묶음(bundle) 단위 주문 회귀 테스트.

한 주문이 여러 묶음(= 포장 튜브)으로 쪼개져 오는 payload를 각 계층이 제대로
다루는지 고정한다. 여기서 막는 실제 사고들:

  · 주문 전체 수량으로 1~3개를 검사해 묶음 2개짜리 주문이 통째로 버려짐
  · 완료 캐시를 주문 단위로 묶어 2번째 튜브가 로봇 동작 없이 성공 처리됨
  · order_targets 에 주문 전체를 실어 1번 튜브에 2번 튜브 몫의 공이 들어감

ROS 런타임(rclpy/palpa_interfaces)이 없어도 돌도록, 검사 대상 함수를 소스에서
AST로 뽑아 그대로 실행한다(재작성한 사본이 아니라 실제 코드를 검증한다).
"""
import ast
import json
import sqlite3
import sys
import tempfile
import types
import unittest
from contextlib import contextmanager
from collections import defaultdict, deque
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parents[1]
NODES = ROOT / 'palpa_ws' / 'src' / 'palpa_control' / 'palpa_control'
sys.path.insert(0, str(NODES))

from order_contract import canonical_item_type, target_family  # noqa: E402


def _load_funcs(path, names, extra_globals):
    """모듈 전체를 import 하지 않고 지정한 최상위 함수만 뽑아 실행한다."""
    tree = ast.parse(Path(path).read_text(encoding='utf-8'))
    ns = dict(extra_globals)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), ns)
    return ns


def _doc_example():
    """ROS_TOPIC_BUNDLE_PROMPT.md 의 예시 주문 — 묶음 2개, 공 6개."""
    return {
        'orderId': 'PL34821', 'itemCount': 6, 'total': 8400, 'mixed': True,
        'bundleCount': 2,
        'items': [
            {'id': 'tennis_ball', 'variant': 'pressureless', 'name': '테니스공 (무압)',
             'qty': 2, 'unitPrice': 1200, 'lineTotal': 2400, 'bundle': 1},
            {'id': 'baseball', 'variant': 'hardball', 'name': '야구공 (하드볼)',
             'qty': 1, 'unitPrice': 2400, 'lineTotal': 2400, 'bundle': 1},
            {'id': 'tennis_ball', 'variant': 'pressurized', 'name': '테니스공 (정압)',
             'qty': 3, 'unitPrice': 1200, 'lineTotal': 3600, 'bundle': 2},
        ],
    }


# ──────────────────────────────────────────────────────────────────────────
# main_controller_node : /order/new 를 묶음별 배치로 쪼갠다
# ──────────────────────────────────────────────────────────────────────────
class MainControllerBundleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        src = (NODES / 'main_controller_node.py').read_text(encoding='utf-8')
        tree = ast.parse(src)
        thresholds = next(ast.literal_eval(n.value) for n in tree.body
                          if isinstance(n, ast.Assign) and n.targets[0].id == 'ITEM_THRESHOLDS')
        fn = next(f for n in tree.body if isinstance(n, ast.ClassDef)
                  for f in n.body
                  if isinstance(f, ast.FunctionDef) and f.name == 'on_new_order')
        ns = {'json': json, 'defaultdict': defaultdict, 'String': object,
              'canonical_item_type': canonical_item_type, 'target_family': target_family,
              'ITEM_THRESHOLDS': thresholds}
        exec(compile(ast.Module(body=[fn], type_ignores=[]), 'main_controller_node.py', 'exec'), ns)
        cls.on_new_order = staticmethod(ns['on_new_order'])

    def _run(self, payload):
        errors = []

        class _Log:
            def error(self, m): errors.append(m)
            def info(self, m): pass

        node = types.SimpleNamespace(_queue=deque(), get_logger=lambda: _Log(), _pump=lambda: None)
        type(self).on_new_order(node, types.SimpleNamespace(data=json.dumps(payload)))
        return list(node._queue), errors

    def test_document_example_becomes_one_batch_per_bundle(self):
        batches, errors = self._run(_doc_example())
        self.assertEqual(errors, [])
        # 묶음 2개 → 배치 2개(= 뚜껑 2사이클 = 튜브 2개). 예전에는 품목 3개 → 배치 3개였다.
        self.assertEqual(len(batches), 2)
        self.assertEqual([b['bundle'] for b in batches], [1, 2])
        self.assertEqual([b['qty'] for b in batches], [3, 3])

    def test_order_targets_never_cross_bundle_boundary(self):
        batches, _ = self._run(_doc_example())
        first, second = batches
        # 1번 튜브 작업 중에 2번 튜브 몫(tennis_normal)을 담으면 안 된다.
        self.assertNotIn('tennis_normal', first['order_targets'])
        self.assertEqual(
            dict(p.split(':') for p in first['order_targets'].split(',')),
            {'tennis_nopress': '2', 'baseball_hard': '1'})
        self.assertEqual(second['order_targets'], 'tennis_normal:3')

    def test_item_id_encodes_bundle_so_batches_stay_distinct(self):
        batches, _ = self._run(_doc_example())
        ids = [f"{b['item_type']}#x{b['qty']}#b{b['bundle']}#all={b['order_targets']}"
               for b in batches]
        self.assertIn('#b1', ids[0])
        self.assertIn('#b2', ids[1])
        self.assertNotEqual(ids[0], ids[1])

    def test_legacy_payload_without_bundle_fields_behaves_as_before(self):
        batches, errors = self._run({
            'orderId': 'OLD1',
            'items': [{'id': 'tennis_ball', 'variant': 'pressurized',
                       'name': '테니스공', 'qty': 2}]})
        self.assertEqual(errors, [])
        self.assertEqual(len(batches), 1)
        self.assertEqual(batches[0]['bundle'], 1)
        self.assertEqual(batches[0]['order_targets'], 'tennis_normal:2')

    def test_bundle_over_tube_capacity_is_rejected_atomically(self):
        payload = _doc_example()
        payload['items'][0]['qty'] = 3          # 묶음 1 이 4개가 된다
        batches, errors = self._run(payload)
        self.assertEqual(batches, [])           # 일부만 실행되지 않는다
        self.assertIn('묶음 1', errors[0])

    def test_bundle_count_mismatch_is_rejected(self):
        payload = _doc_example()
        payload['bundleCount'] = 3
        batches, errors = self._run(payload)
        self.assertEqual(batches, [])
        self.assertIn('bundleCount', errors[0])


# ──────────────────────────────────────────────────────────────────────────
# robot_controller_node : item_id 파싱 + 묶음별 완료 캐시
# ──────────────────────────────────────────────────────────────────────────
class RobotControllerBundleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ns = _load_funcs(NODES / 'robot_controller_node.py',
                             {'_parse_qty', '_parse_bundle', '_parse_order_targets'}, {})

    def test_parse_bundle(self):
        p = self.ns['_parse_bundle']
        self.assertEqual(p('tennis_nopress#x3#b1#all=tennis_nopress:2,baseball_hard:1'), 1)
        self.assertEqual(p('tennis_normal#x3#b2#all=tennis_normal:3'), 2)
        self.assertEqual(p('tennis_normal#x2#all=tennis_normal:2'), 1)   # 구버전
        self.assertEqual(p('tennis_normal'), 1)                          # 최구버전

    def test_qty_and_targets_still_parse_with_bundle_present(self):
        item_id = 'tennis_nopress#x3#b1#all=tennis_nopress:2,baseball_hard:1'
        self.assertEqual(self.ns['_parse_qty'](item_id), 3)
        self.assertEqual(self.ns['_parse_order_targets'](item_id),
                         {'tennis_nopress': 2, 'baseball_hard': 1})

    def test_completion_cache_key_separates_bundles(self):
        """캐시가 주문 단위면 2번 튜브가 로봇 동작 없이 성공 처리된다."""
        p = self.ns['_parse_bundle']
        done = {('PL34821', p('tennis_nopress#x3#b1#all=tennis_nopress:2,baseball_hard:1'))}
        second = ('PL34821', p('tennis_normal#x3#b2#all=tennis_normal:3'))
        self.assertNotIn(second, done)

    def test_bundle_targets_never_exceed_tube_capacity(self):
        targets = self.ns['_parse_order_targets'](
            'tennis_nopress#x3#b1#all=tennis_nopress:2,baseball_hard:1')
        self.assertLessEqual(sum(targets.values()), 3)


# ──────────────────────────────────────────────────────────────────────────
# palpa_backend : 묶음별 검증 + DB 저장
# ──────────────────────────────────────────────────────────────────────────
class _HTTPException(Exception):
    def __init__(self, status_code=None, detail=None):
        self.status_code, self.detail = status_code, detail
        super().__init__(detail)


class BackendBundleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.ns = _load_funcs(
            ROOT / 'palpa_backend' / 'main.py',
            {'init_db', 'get_db', 'save_order', 'validate_order_contract'},
            {'HTTPException': _HTTPException, 'sqlite3': sqlite3,
             'contextmanager': contextmanager, 'Optional': Optional, 'OrderPayload': object,
             'DB_PATH': str(Path(self.tmp.name) / 'test.db'),
             'PRODUCT_VARIANTS': {'tennis_ball': {'pressureless', 'pressurized'},
                                  'baseball': {'softball', 'hardball'}}})
        self.ns['init_db']()

    def tearDown(self):
        self.tmp.cleanup()

    @staticmethod
    def _order(payload):
        items = [types.SimpleNamespace(
            id=i['id'], variant=i.get('variant'), name=i['name'], qty=i['qty'],
            unitPrice=i.get('unitPrice', 1.0), lineTotal=i.get('lineTotal', 1.0),
            bundle=i.get('bundle', 1)) for i in payload['items']]
        return types.SimpleNamespace(
            orderId=payload['orderId'], items=items,
            itemCount=payload.get('itemCount', sum(i.qty for i in items)),
            total=payload.get('total', 1.0), mixed=payload.get('mixed', False),
            bundleCount=payload.get('bundleCount', 1),
            customer=types.SimpleNamespace(name='t', phone='010', email='',
                                           address='a', addressDetail='', note=''),
            createdAt='2026-07-28T00:00:00Z')

    def test_migration_adds_bundle_columns(self):
        with sqlite3.connect(self.ns['DB_PATH']) as conn:
            orders = {r[1] for r in conn.execute('PRAGMA table_info(orders)')}
            items = {r[1] for r in conn.execute('PRAGMA table_info(order_items)')}
        self.assertIn('bundle_count', orders)
        self.assertIn('bundle_index', items)

    def test_multi_bundle_order_is_accepted(self):
        """예전 규칙(주문 총량 1~3개)이면 이 주문이 422로 거부됐다."""
        self.ns['validate_order_contract'](self._order(_doc_example()))

    def test_capacity_is_per_bundle_not_per_order(self):
        payload = _doc_example()
        payload['items'][0]['qty'] = 3          # 묶음 1 = 4개
        payload['itemCount'] = 7
        with self.assertRaises(_HTTPException) as ctx:
            self.ns['validate_order_contract'](self._order(payload))
        self.assertIn('묶음 1', ctx.exception.detail)

    def test_legacy_single_bundle_limit_still_applies(self):
        payload = {'orderId': 'OLD2', 'itemCount': 4,
                   'items': [{'id': 'tennis_ball', 'variant': 'pressurized',
                              'name': '테니스공', 'qty': 4}]}
        with self.assertRaises(_HTTPException):
            self.ns['validate_order_contract'](self._order(payload))

    def test_bundle_count_mismatch_is_rejected(self):
        payload = _doc_example()
        payload['bundleCount'] = 3
        with self.assertRaises(_HTTPException) as ctx:
            self.ns['validate_order_contract'](self._order(payload))
        self.assertIn('bundleCount', ctx.exception.detail)

    def test_save_order_persists_bundle_membership(self):
        self.ns['save_order'](self._order(_doc_example()))
        with sqlite3.connect(self.ns['DB_PATH']) as conn:
            conn.row_factory = sqlite3.Row
            order = conn.execute(
                'SELECT bundle_count FROM orders WHERE order_id = ?', ('PL34821',)).fetchone()
            rows = conn.execute(
                'SELECT item_id, qty, bundle_index FROM order_items WHERE order_id = ? '
                'ORDER BY id', ('PL34821',)).fetchall()
        self.assertEqual(order['bundle_count'], 2)
        self.assertEqual([r['bundle_index'] for r in rows], [1, 1, 2])
        self.assertEqual([r['qty'] for r in rows], [2, 1, 3])


if __name__ == '__main__':
    unittest.main()
