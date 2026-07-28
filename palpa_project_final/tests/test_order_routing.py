import json
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


FINAL_PROJECT = Path(__file__).resolve().parents[1]
WORKSPACE = FINAL_PROJECT.parent
CONTROL_SRC = FINAL_PROJECT / "palpa_ws" / "src" / "palpa_control"
sys.path.insert(0, str(FINAL_PROJECT))
sys.path.insert(0, str(CONTROL_SRC))

import paths
import grip_config as cfg
import grip_cycle
import grip_web
from palpa_control.order_contract import canonical_item_type, target_family


class _Request:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class _Service:
    Request = _Request


class _FakeFlowState:
    CLASS_NAMES = {
        "tennis_normal": "🟢 유압 (정상)",
        "tennis_nopress": "🔵 무압 (정상)",
        "tennis_defect": "🔴 구멍 (불량)",
        "baseball_hard": "🟡 하드 (야구)",
        "baseball_soft": "🟠 소프트 (야구)",
    }
    LABELS = {
        "tennis_normal": "유압",
        "tennis_nopress": "무압",
        "tennis_defect": "구멍",
        "baseball_hard": "하드",
        "baseball_soft": "소프트",
    }

    def __init__(self, observed, release_width=110.0):
        self.observed = observed
        self.release_width = float(release_width)
        self.actual_width = 110.0
        self.flow_w5 = None
        self.flow_class = None
        self.flow_code = None
        self.flow_ball_type = None
        self.flow_busy = False
        self.flow_seq = 0
        self.flow_done_seq = 0
        self.flow_error = None
        self.actions = []
        self._training_seq = 0
        self._training_cancelled = False

    def request_flow(self, action):
        self.actions.append(action)
        self.flow_seq += 1
        self.flow_error = None
        if action == "open":
            self.actual_width = 110.0
            self.flow_w5 = None
        elif action == "grab":
            self.actual_width = 66.0
            self.flow_w5 = 67.0
        elif action == "measure":
            self.flow_class = self.CLASS_NAMES[self.observed]
            self.flow_code = self.observed
            self.flow_ball_type = (
                "baseball" if self.observed.startswith("baseball_") else "tennis"
            )
        elif action == "release":
            self.actual_width = self.release_width
            self.flow_w5 = None
        self.flow_done_seq = self.flow_seq
        return self.flow_seq

    def stage_flow_measurement_for_training(self):
        self._training_seq += 1
        return self._training_seq

    def training_label(self, measure_seq):
        if self._training_cancelled or measure_seq != self._training_seq:
            return None
        return self.LABELS[self.observed]

    def cancel_training_label_wait(self, measure_seq):
        if measure_seq == self._training_seq:
            self._training_cancelled = True


def _make_context(
    waypoints,
    observed,
    *,
    start=None,
    tool=None,
    tcp=None,
    tool_sample_time=None,
    ensure_auto=True,
    prepare_collision=True,
    joints_fresh=True,
    robot_states=None,
    move_success=True,
    release_width=110.0,
):
    current = list(cfg.HOME_POSJ if start is None else start)
    trace = []
    move_requests = []
    spline_requests = []
    collision_preparations = []
    stop_modes = []
    state = _FakeFlowState(observed, release_width=release_width)
    jog = SimpleNamespace(
        _cycle_stop=False,
        _training_stop=False,
        _cycle_vel=cfg.DEFAULT_WORK_VEL,
        cycle_msg="",
        batch_packed=0,
        last_motion={},
    )
    state_values = list(robot_states or [1])
    state_index = 0

    def next_robot_state():
        nonlocal state_index
        value = state_values[min(state_index, len(state_values) - 1)]
        state_index += 1
        return value

    def move_is_successful(index, req):
        if callable(move_success):
            return bool(move_success(index, req))
        return bool(move_success)

    def call(client, req, cto=None):
        if client == "move_joint":
            index = len(move_requests) + 1
            move_requests.append(
                {
                    "pos": list(req.pos),
                    "vel": float(req.vel),
                    "acc": float(req.acc),
                    "radius": float(req.radius),
                    "mode": int(req.mode),
                    "blend_type": int(req.blend_type),
                    "sync_type": int(req.sync_type),
                    "profile": jog.last_motion.get("profile"),
                }
            )
            if not move_is_successful(index, req):
                return SimpleNamespace(success=False)
            if getattr(req, "mode", 1) == 0:
                current[:] = list(req.pos)
            else:
                current[:] = [current[i] + req.pos[i] for i in range(6)]
            trace.append([round(v, 2) for v in current])
            return SimpleNamespace(success=True)
        if client == "spline":
            targets = [list(p.data) for p in req.pos]
            spline_requests.append({
                "targets": targets,
                "vel": list(req.vel),
                "acc": list(req.acc),
                "sync_type": int(req.sync_type),
            })
            for target in targets:
                current[:] = target
                trace.append([round(v, 2) for v in current])
            return SimpleNamespace(success=True)
        if client == "robot_state":
            return SimpleNamespace(success=True, robot_state=next_robot_state())
        if client == "check_motion":
            return SimpleNamespace(success=True, status=0)
        if client == "move_stop":
            stop_modes.append(getattr(req, "stop_mode", None))
            return SimpleNamespace(success=True)
        return SimpleNamespace(success=True)

    selected_tool = cfg.REQUIRED_TOOL if tool is None else tool
    selected_tcp = cfg.REQUIRED_TCP if tcp is None else tcp
    selected_tool_time = time.time() if tool_sample_time is None else tool_sample_time
    ctx = SimpleNamespace(
        jog=jog,
        call=call,
        cmj="move_joint",
        cstop="move_stop",
        ccol="collision",
        cctrl="robot_control",
        cstate="robot_state",
        cchk="check_motion",
        cspl="spline",
        MoveJoint=_Service,
        MoveStop=_Service,
        ChangeCollisionSensitivity=_Service,
        SetRobotControl=_Service,
        GetRobotState=_Service,
        CheckMotion=_Service,
        MoveSplineJoint=_Service,
        Float64MultiArray=SimpleNamespace,
        joints=lambda: list(current) if joints_fresh else None,
        waypoints=lambda: list(waypoints),
        state=state,
        motion_active=threading.Event(),
        ensure_auto=lambda: bool(ensure_auto),
        prepare_collision=lambda value: (
            collision_preparations.append(int(value)) is None
            and bool(prepare_collision)
        ),
        tool_snapshot=lambda: (
            selected_tool,
            selected_tcp,
            selected_tool_time,
        ),
        trace=trace,
        move_requests=move_requests,
        spline_requests=spline_requests,
        collision_preparations=collision_preparations,
        stop_modes=stop_modes,
    )
    return ctx, trace


class OrderContractTests(unittest.TestCase):
    def test_updated_backend_variants_map_to_exact_robot_sku(self):
        cases = [
            ("tennis_ball", "pressurized", "테니스공 (유압)", "tennis_normal"),
            ("tennis_ball", "pressureless", "테니스공 (무압)", "tennis_nopress"),
            ("baseball", "hardball", "야구공 (하드볼)", "baseball_hard"),
            ("baseball", "softball", "야구공 (소프트볼)", "baseball_soft"),
        ]
        for item_id, variant, name, expected in cases:
            with self.subTest(variant=variant):
                self.assertEqual(
                    canonical_item_type(item_id, variant, name),
                    expected,
                )
                self.assertEqual(
                    cfg.normalize_order_target(item_id, name, variant),
                    expected,
                )

    def test_variant_product_mismatch_is_rejected(self):
        with self.assertRaises(ValueError):
            canonical_item_type("baseball", "pressureless", "야구공")

    def test_legacy_name_fallback_and_family(self):
        self.assertEqual(
            canonical_item_type("tennis_ball", None, "테니스공 (무압)"),
            "tennis_nopress",
        )
        self.assertEqual(
            canonical_item_type("baseball", None, "야구공 (소프트볼)"),
            "baseball_soft",
        )
        self.assertEqual(target_family("tennis_normal"), "tennis_ball")
        self.assertEqual(target_family("baseball_hard"), "baseball")

    def test_baseball_retrained_direction_is_respected(self):
        original = dict(cfg.CLASSIFY["baseball"])
        try:
            cfg.CLASSIFY["baseball"].update(
                {"by": "comp", "hard_max": 3.0, "hard_low": False}
            )
            self.assertIn("하드", cfg.classify(74.0, 4.0)[0])
            self.assertIn("소프트", cfg.classify(74.0, 2.0)[0])
        finally:
            cfg.CLASSIFY["baseball"].clear()
            cfg.CLASSIFY["baseball"].update(original)

    def test_multi_item_routing_packs_any_observed_sku_still_needed(self):
        remaining = {
            "tennis_normal": 1,
            "tennis_nopress": 1,
            "baseball_soft": 1,
        }
        self.assertEqual(
            cfg.routing_decision(
                "tennis_normal", "tennis_nopress", remaining=remaining),
            "pack",
        )
        self.assertEqual(
            cfg.routing_decision(
                "tennis_normal", "baseball_soft", remaining=remaining),
            "pack",
        )
        remaining["tennis_nopress"] = 0
        self.assertEqual(
            cfg.routing_decision(
                "tennis_normal", "tennis_nopress", remaining=remaining),
            "replenish_tennis",
        )

    def test_jog_worker_accepts_three_item_order_as_one_batch(self):
        jog = grip_web.JogWorker()
        accepted = jog.request_cycle(
            vel=40,
            targets={
                "tennis_normal": 1,
                "tennis_nopress": 1,
                "baseball_soft": 1,
            },
        )
        self.assertTrue(accepted)
        self.assertEqual(jog._batch_target, 3)
        self.assertEqual(
            jog._order_remaining,
            {
                "tennis_normal": 1,
                "tennis_nopress": 1,
                "baseball_soft": 1,
            },
        )
        self.assertEqual(jog._order_requested, jog._order_remaining)
        self.assertIsNot(jog._order_requested, jog._order_remaining)
        self.assertFalse(jog._batch_completed)
        self.assertIsNone(jog._batch_weight_kg)


class MeasurementTrainingTests(unittest.TestCase):
    def test_fresh_training_archives_csv_and_label_loop_waits(self):
        state = grip_web.GripperState("127.0.0.1", 65)
        with tempfile.TemporaryDirectory() as td:
            csv_path = Path(td) / "ball_measurements_final.csv"
            csv_path.write_text("old-data\n", encoding="utf-8")
            state.saved = [{"label": "유압"}]
            with patch.object(paths, "MEASURE_CSV", str(csv_path)):
                backup = state.reset_training_data()
            self.assertFalse(csv_path.exists())
            self.assertTrue(Path(backup).exists())
            self.assertEqual(state.saved, [])

        self.assertTrue(state.request_measure_loop(True))
        self.assertTrue(state._measure_req)
        self.assertFalse(state._measure_wait_label)
        self.assertFalse(state.request_measure_loop(False))
        self.assertFalse(state._measure_req)

    def test_robot_flow_measurement_is_released_by_matching_label_click(self):
        state = grip_web.GripperState("127.0.0.1", 65)
        state.flow_w5 = 67.5
        state.flow_contact_force = 5.0
        state.flow_comp = 3.9
        state.flow_size = 67.5
        state.flow_ball_type = "tennis"
        state.flow_class = "🔴 구멍 (불량)"
        seq = state.stage_flow_measurement_for_training()
        self.assertTrue(state._training_wait_label)
        self.assertIsNone(state.training_label(seq))

        with tempfile.TemporaryDirectory() as td:
            csv_path = Path(td) / "ball_measurements_final.csv"
            with patch.object(paths, "MEASURE_CSV", str(csv_path)):
                ok, _ = state.save_measurement("구멍")
            self.assertTrue(ok)
            self.assertTrue(csv_path.exists())

        self.assertEqual(state.training_label(seq), "구멍")
        self.assertFalse(state._training_wait_label)

    def test_stale_grip_flag_without_new_closing_motion_is_not_contact(self):
        state = grip_web.GripperState("127.0.0.1", 65)

        class FakeMb:
            def read_holding(self, _addr, _count):
                regs = [0] * 18
                regs[10] = 2       # 이전 동작의 grip bit만 남음, busy=0
                regs[17] = 1100    # 폭 변화도 없음
                return regs

            def write_registers(self, *_args):
                return None

        state.mb = FakeMb()
        width, grip = state._approach_contact(5.0, timeout=0.01)
        self.assertEqual(width, 110.0)
        self.assertFalse(grip)

    def test_fresh_width_change_allows_new_grip_flag(self):
        state = grip_web.GripperState("127.0.0.1", 65)

        class FakeMb:
            def __init__(self):
                self.reads = 0

            def read_holding(self, _addr, _count):
                self.reads += 1
                regs = [0] * 18
                if self.reads == 1:
                    regs[10] = 2; regs[17] = 1100  # 명령 전 stale flag
                elif self.reads == 2:
                    regs[10] = 1; regs[17] = 900   # 새 닫기 busy + 폭 감소
                else:
                    regs[10] = 2; regs[17] = 675   # 이번 동작의 접촉
                return regs

            def write_registers(self, *_args):
                return None

        state.mb = FakeMb()
        with patch.object(time, "sleep", return_value=None):
            width, grip = state._approach_contact(5.0, timeout=0.1)
        self.assertEqual(width, 67.5)
        self.assertTrue(grip)

    def test_baseball_reapproach_failure_does_not_reuse_stale_5n_result(self):
        state = grip_web.GripperState("127.0.0.1", 65)
        state.mb.write_registers = lambda *_args: None
        attempts = [
            (75.0, True),   # 5N에서 야구 감지 → 정렬 진입
            (0.0, False), (0.0, False), (0.0, False),  # 10N 재접근 전부 실패
        ]
        with patch.object(state, "_approach_contact", side_effect=attempts), \
                patch.object(state, "_wait_open", return_value=True), \
                patch.object(time, "sleep", return_value=None):
            width, grip, contact_f = state._contact_adaptive(timeout=0.1)
        self.assertEqual(width, 0.0)
        self.assertFalse(grip)
        self.assertEqual(contact_f, cfg.GRIPPER["contact_force_baseball"])

    def test_threshold_training_uses_medians_and_baseball_direction(self):
        state = grip_web.GripperState("127.0.0.1", 65)
        rows = [
            ("무압", 66.0, 1.0), ("무압", 66.1, 1.2), ("무압", 66.2, 50.0),
            ("유압", 67.0, 3.0), ("유압", 67.1, 3.2), ("유압", 67.2, 3.4),
            ("구멍", 67.0, 4.2), ("구멍", 67.1, 4.4), ("구멍", 67.2, 4.6),
            ("하드", 73.0, 5.0), ("하드", 74.0, 5.2),
            ("소프트", 73.0, 2.0), ("소프트", 74.0, 2.2),
        ]
        with tempfile.TemporaryDirectory() as td:
            csv_path = Path(td) / "ball_measurements_final.csv"
            lines = [
                "time,ball_type,label,size_mm,w@40N,comp_total_5to40,"
                "creep_1p5s,contact_N,verdict,force_width_points"
            ]
            for label, size, comp in rows:
                btype = cfg.LABEL_TYPE[label]
                contact_n = 10 if btype == "baseball" else 5
                lines.append(
                    f"12:00:00,{btype},{label},{size},{size-comp},{comp},"
                    f",{contact_n},,"
                )
            csv_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            with patch.object(paths, "MEASURE_CSV", str(csv_path)):
                out, meta = state.compute_thresholds()

        # 후보 특징 중 실제 분리 정확도가 가장 높은 축을 고른다.
        self.assertEqual(out["tennis"]["nopress_by"], "size")
        self.assertAlmostEqual(out["tennis"]["nopress_max"], 66.6)
        self.assertEqual(out["baseball"]["by"], "comp")
        self.assertFalse(out["baseball"]["hard_low"])
        self.assertEqual(meta["counts"]["유압"], 3)


class WaypointRoutingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.waypoints = json.loads((WORKSPACE / "waypoints.json").read_text())
        cls.by_number = {
            index: [round(v, 2) for v in waypoint["posj"]]
            for index, waypoint in enumerate(cls.waypoints, start=1)
        }

    def _run(self, target, observed):
        result, ctx = self._run_context(target, observed)
        return result, ctx.trace, ctx.jog.batch_packed

    def _run_context(self, target, observed, **context_options):
        ctx, _ = _make_context(self.waypoints, observed, **context_options)
        with patch.object(time, "sleep", return_value=None):
            result = grip_cycle.run_cycle(ctx, target_item=target)
        return result, ctx

    def _expected_route(self, target, observed):
        source = (5, 2, 20, 2) if target.startswith("tennis_") else (5, 4, 3, 4)
        if observed == "tennis_defect":
            return "defect", 0, source + (5, 7, 5)
        if observed == target:
            return "packed", 1, source + (5, 6, 5)
        if observed.startswith("baseball_"):
            return "rerouted", 0, source + (5, 13, 14, 13, 5)
        return "rerouted", 0, source + (5, 15, 16, 15, 5)

    def test_all_named_order_waypoints_resolve(self):
        for key in cfg.ORDER_WP_NAMES:
            with self.subTest(key=key):
                self.assertIsNotNone(grip_cycle.find_order_wp(self.waypoints, key))
        self.assertEqual(
            [round(v, 2) for v in grip_cycle.find_order_wp(self.waypoints, "tennis_pick")],
            self.by_number[20],
        )
        self.assertNotEqual(
            [round(v, 2) for v in grip_cycle.find_order_wp(self.waypoints, "tennis_pick")],
            self.by_number[1],
        )

    def test_complete_four_sku_by_five_observation_route_matrix(self):
        targets = (
            "tennis_normal",
            "tennis_nopress",
            "baseball_hard",
            "baseball_soft",
        )
        observations = (
            "tennis_normal",
            "tennis_nopress",
            "tennis_defect",
            "baseball_hard",
            "baseball_soft",
        )
        for target in targets:
            for observed in observations:
                with self.subTest(target=target, observed=observed):
                    result, ctx = self._run_context(target, observed)
                    status, packed, route = self._expected_route(target, observed)
                    self.assertEqual(result["status"], status)
                    self.assertEqual(ctx.jog.batch_packed, packed)
                    self.assertEqual(
                        ctx.trace,
                        [self.by_number[number] for number in route],
                    )

    def test_movej_and_fixed_collision_preflight_follow_safety_contract(self):
        result, ctx = self._run_context("baseball_soft", "baseball_hard")
        self.assertEqual(result["status"], "rerouted")
        self.assertTrue(ctx.move_requests)

        for request in ctx.move_requests:
            with self.subTest(target=request["pos"], profile=request["profile"]):
                self.assertEqual(request["mode"], 0)
                self.assertEqual(request["radius"], 0.0)
                self.assertEqual(request["sync_type"], 1)
                self.assertEqual(request["blend_type"], 0)

                profile = request["profile"]
                self.assertIn(profile, {"free"} | cfg.FLOW_MOVE_KEYS)
                velocity_cap = (
                    cfg.move_vel(profile)
                    if profile in cfg.FLOW_MOVE_KEYS
                    else cfg.SPEED["free_max"]
                )
                self.assertLessEqual(request["vel"], velocity_cap)
                self.assertGreater(request["acc"], 0.0)

        self.assertEqual(
            ctx.collision_preparations,
            [cfg.COLLISION["fixed"]],
        )

        expected_caps = {
            "ord_enter": 30.0,
            "ord_to_lift": 30.0,
            "ord_descend": 24.0,
            "ord_extract": 20.0,
            "ord_carry_hub": 16.0,
            "ord_carry_via": 16.0,
            "ord_place": 12.0,
            "ord_return": 30.0,
        }
        seen = {}
        for request in ctx.move_requests:
            seen.setdefault(
                request["profile"],
                (request["vel"], request["acc"]),
            )
        for profile, velocity_cap in expected_caps.items():
            self.assertIn(profile, seen)
            self.assertLessEqual(seen[profile][0], velocity_cap)

        # 판별 힘은 이동 속도 상향과 독립적으로 그대로 유지한다.
        self.assertEqual(cfg.GRIPPER["contact_force"], 5.0)
        self.assertEqual(cfg.GRIPPER["grip_force"], 400)

    def test_collision_preparation_failure_is_rejected_before_movej(self):
        result, ctx = self._run_context(
            "tennis_normal",
            "tennis_normal",
            prepare_collision=False,
        )
        self.assertEqual(result["status"], "error")
        self.assertIn("고정 충돌감도", result["error"])
        self.assertEqual(ctx.move_requests, [])
        self.assertEqual(ctx.stop_modes, [2])

    def test_start_pose_outside_home_or_p5_is_rejected_before_movej(self):
        bad_start = [value + 30.0 for value in cfg.HOME_POSJ]
        result, ctx = self._run_context(
            "tennis_normal",
            "tennis_normal",
            start=bad_start,
        )
        self.assertEqual(result["status"], "error")
        self.assertIn("시작 자세", result["error"])
        self.assertEqual(ctx.move_requests, [])
        self.assertEqual(ctx.stop_modes, [2])

    def test_tool_or_tcp_mismatch_is_rejected_before_movej(self):
        for tool, tcp in (
            ("wrong-tool", cfg.REQUIRED_TCP),
            (cfg.REQUIRED_TOOL, "wrong-tcp"),
        ):
            with self.subTest(tool=tool, tcp=tcp):
                result, ctx = self._run_context(
                    "tennis_normal",
                    "tennis_normal",
                    tool=tool,
                    tcp=tcp,
                )
                self.assertEqual(result["status"], "error")
                self.assertIn("공구/TCP 불일치", result["error"])
                self.assertEqual(ctx.move_requests, [])
                self.assertEqual(ctx.stop_modes, [2])

    def test_stale_tool_snapshot_or_autonomous_failure_is_rejected(self):
        cases = (
            {
                "name": "stale_tool_snapshot",
                "options": {
                    "tool_sample_time": time.time()
                    - cfg.TOOL_SAMPLE_MAX_AGE_S
                    - 1.0
                },
                "error": "공구/TCP 정보가 오래",
            },
            {
                "name": "autonomous_mode_failure",
                "options": {"ensure_auto": False},
                "error": "AUTONOMOUS",
            },
        )
        for case in cases:
            with self.subTest(case=case["name"]):
                result, ctx = self._run_context(
                    "tennis_normal",
                    "tennis_normal",
                    **case["options"],
                )
                self.assertEqual(result["status"], "error")
                self.assertIn(case["error"], result["error"])
                self.assertEqual(ctx.move_requests, [])
                self.assertEqual(ctx.stop_modes, [2])

    def test_stale_joint_sample_is_rejected_before_movej(self):
        result, ctx = self._run_context(
            "tennis_normal",
            "tennis_normal",
            joints_fresh=False,
        )
        self.assertEqual(result["status"], "error")
        self.assertIn("최신 관절", result["error"])
        self.assertEqual(ctx.move_requests, [])
        self.assertEqual(ctx.stop_modes, [2])

    def test_robot_safe_stop_aborts_without_sending_next_movej(self):
        # 시작검사=1, 진입 async 체인 전=1, 체인 최종 도착검사에서 SAFE_STOP=5.
        result, ctx = self._run_context(
            "tennis_normal",
            "tennis_normal",
            robot_states=[1, 1, 5],
        )
        self.assertEqual(result["status"], "error")
        self.assertIn("안전정지", result["error"])
        self.assertEqual(len(ctx.move_requests), 3)
        self.assertEqual(ctx.stop_modes, [2])
        self.assertEqual(ctx.state.actions, ["open"])

    def test_rejected_movej_is_not_automatically_retransmitted(self):
        result, ctx = self._run_context(
            "tennis_normal",
            "tennis_normal",
            move_success=False,
        )
        self.assertEqual(result["status"], "error")
        self.assertIn("거부", result["error"])
        self.assertEqual(len(ctx.move_requests), 1)
        self.assertEqual(ctx.jog.last_motion["status"], "rejected")
        self.assertEqual(ctx.stop_modes, [2])

    def test_pack_count_commits_only_after_drop_and_verified_open(self):
        cases = (
            {
                "name": "drop_move_rejected",
                "options": {"move_success": lambda index, _req: index != 6},
                "packed": 0,
                "release_count": 0,
            },
            {
                "name": "gripper_stays_closed",
                "options": {"release_width": 50.0},
                "packed": 0,
                "release_count": 2,
            },
            {
                "name": "return_move_rejected_after_commit",
                "options": {"move_success": lambda index, _req: index != 7},
                "packed": 1,
                "release_count": 1,
            },
        )
        for case in cases:
            with self.subTest(case=case["name"]):
                result, ctx = self._run_context(
                    "tennis_normal",
                    "tennis_normal",
                    **case["options"],
                )
                self.assertEqual(result["status"], "error")
                self.assertEqual(ctx.jog.batch_packed, case["packed"])
                self.assertEqual(
                    ctx.state.actions.count("release"),
                    case["release_count"],
                )

    def test_exact_tennis_route_packs_and_counts_once(self):
        result, trace, packed = self._run("tennis_normal", "tennis_normal")
        self.assertEqual(result["status"], "packed")
        self.assertEqual(trace, [self.by_number[n] for n in (5, 2, 20, 2, 5, 6, 5)])
        self.assertEqual(packed, 1)

    def test_exact_baseball_route_uses_baseball_source(self):
        result, trace, packed = self._run("baseball_soft", "baseball_soft")
        self.assertEqual(result["status"], "packed")
        self.assertEqual(trace, [self.by_number[n] for n in (5, 4, 3, 4, 5, 6, 5)])
        self.assertEqual(packed, 1)

    def test_defect_goes_to_p7_without_count(self):
        result, trace, packed = self._run("tennis_normal", "tennis_defect")
        self.assertEqual(result["status"], "defect")
        self.assertEqual(trace, [self.by_number[n] for n in (5, 2, 20, 2, 5, 7, 5)])
        self.assertEqual(packed, 0)

    def test_wrong_family_baseball_returns_via_p13_p14(self):
        result, trace, packed = self._run("tennis_normal", "baseball_hard")
        self.assertEqual(result["status"], "rerouted")
        self.assertEqual(
            trace,
            [self.by_number[n] for n in (5, 2, 20, 2, 5, 13, 14, 13, 5)],
        )
        self.assertEqual(packed, 0)

    def test_wrong_tennis_subtype_refreshes_via_p15_p16(self):
        result, trace, packed = self._run("tennis_nopress", "tennis_normal")
        self.assertEqual(result["status"], "rerouted")
        self.assertEqual(
            trace,
            [self.by_number[n] for n in (5, 2, 20, 2, 5, 15, 16, 15, 5)],
        )
        self.assertEqual(packed, 0)

    def test_wrong_baseball_subtype_refreshes_via_p13_p14(self):
        result, trace, packed = self._run("baseball_soft", "baseball_hard")
        self.assertEqual(result["status"], "rerouted")
        self.assertEqual(
            trace,
            [self.by_number[n] for n in (5, 4, 3, 4, 5, 13, 14, 13, 5)],
        )
        self.assertEqual(packed, 0)

    def test_multi_order_packs_nopress_even_while_searching_for_pressurized(self):
        remaining = {
            "tennis_normal": 1,
            "tennis_nopress": 1,
            "baseball_soft": 1,
        }
        ctx, _ = _make_context(self.waypoints, "tennis_nopress")
        with patch.object(time, "sleep", return_value=None):
            result = grip_cycle.run_cycle(
                ctx,
                target_item="tennis_normal",
                order_remaining=remaining,
            )
        self.assertEqual(result["status"], "packed")
        self.assertEqual(result["observed"], "tennis_nopress")
        self.assertEqual(ctx.jog.batch_packed, 1)
        self.assertEqual(remaining["tennis_nopress"], 0)
        self.assertEqual(remaining["tennis_normal"], 1)
        self.assertEqual(
            ctx.trace,
            [self.by_number[n] for n in (5, 2, 20, 2, 5, 6, 5, 2, 20)],
        )
        self.assertEqual(ctx.spline_requests, [])

    def test_multi_order_returns_nopress_when_its_order_is_already_filled(self):
        remaining = {
            "tennis_normal": 1,
            "tennis_nopress": 0,
            "baseball_soft": 1,
        }
        ctx, _ = _make_context(self.waypoints, "tennis_nopress")
        with patch.object(time, "sleep", return_value=None):
            result = grip_cycle.run_cycle(
                ctx,
                target_item="tennis_normal",
                order_remaining=remaining,
            )
        self.assertEqual(result["status"], "rerouted")
        self.assertEqual(ctx.jog.batch_packed, 0)
        self.assertEqual(
            ctx.trace,
            [self.by_number[n] for n in (5, 2, 20, 2, 5, 15, 16, 15, 5, 2, 20)],
        )

    def test_tennis_robot_training_waits_for_label_and_returns_directly_to_p5(self):
        ctx, _ = _make_context(self.waypoints, "tennis_normal")
        with patch.object(time, "sleep", return_value=None):
            result = grip_cycle.run_cycle(
                ctx,
                target_item="tennis_normal",
                training_mode=True,
            )
        self.assertEqual(result["status"], "trained")
        self.assertEqual(result["label"], "유압")
        self.assertEqual(ctx.jog.batch_packed, 0)
        self.assertEqual(
            ctx.trace,
            [self.by_number[n] for n in (5, 2, 20, 2, 5)],
        )

    def test_baseball_robot_training_waits_for_label_and_returns_directly_to_p5(self):
        ctx, _ = _make_context(self.waypoints, "baseball_soft")
        with patch.object(time, "sleep", return_value=None):
            result = grip_cycle.run_cycle(
                ctx,
                target_item="baseball_hard",
                training_mode=True,
            )
        self.assertEqual(result["status"], "trained")
        self.assertEqual(result["label"], "소프트")
        self.assertEqual(ctx.jog.batch_packed, 0)
        self.assertEqual(
            ctx.trace,
            [self.by_number[n] for n in (5, 4, 3, 4, 5)],
        )

    def test_robot_training_stop_returns_held_ball_before_finishing(self):
        ctx, _ = _make_context(self.waypoints, "tennis_nopress")
        ctx.jog._training_stop = True
        with patch.object(time, "sleep", return_value=None):
            result = grip_cycle.run_cycle(
                ctx,
                target_item="tennis_normal",
                training_mode=True,
            )
        self.assertEqual(result["status"], "training_stopped")
        self.assertIsNone(result["label"])
        self.assertEqual(
            ctx.trace,
            [self.by_number[n] for n in (5, 2, 20, 2, 5)],
        )


if __name__ == "__main__":
    unittest.main()
