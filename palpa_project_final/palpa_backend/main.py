"""
PALPA FastAPI 백엔드 (v2 - DB 연동)

역할:
  1. 프론트엔드(React) 주문 POST 요청 수신 (/order)
  2. 주문을 SQLite DB에 저장 (orders, order_items 테이블)
  3. rclpy로 ROS2 /order/new 토픽에 그대로 publish
  4. 관리자용 조회 API 제공 (/orders, /orders/{order_id})

DB 파일: ./palpa.db (없으면 첫 실행 시 자동 생성)

실행 전 준비:
  1. ROS2 환경 source (source install/setup.bash)
  2. pip install fastapi "uvicorn[standard]" --break-system-packages

실행:
  uvicorn main:app --host 0.0.0.0 --port 8000 --reload
"""

import asyncio
import hashlib
import json
import os
import secrets
import sqlite3
import threading
from contextlib import asynccontextmanager, contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from palpa_interfaces.msg import InspectionResult

from fastapi import Depends, FastAPI, Header, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

DB_PATH = os.environ.get(
    "PALPA_DB_PATH",
    str(Path(__file__).resolve().parent / "palpa.db"),
)

PRODUCT_VARIANTS = {
    "tennis_ball": {"pressureless", "pressurized"},
    "baseball": {"softball", "hardball"},
}


# ──────────────────────────────────────────────────────────────
# DB: SQLite, 커넥션은 요청마다 새로 열고 닫음 (단순하고 안전)
# ──────────────────────────────────────────────────────────────
def init_db():
    with get_db() as conn:
        conn.executescript(
            """
            CREATE TABLE IF NOT EXISTS orders (
                order_id TEXT PRIMARY KEY,
                item_count INTEGER NOT NULL,
                total REAL NOT NULL,
                mixed INTEGER NOT NULL,
                customer_name TEXT NOT NULL,
                customer_phone TEXT NOT NULL,
                customer_email TEXT,
                customer_address TEXT NOT NULL,
                customer_address_detail TEXT,
                customer_note TEXT,
                created_at TEXT NOT NULL,
                status TEXT NOT NULL DEFAULT 'RECEIVED'
            );

            CREATE TABLE IF NOT EXISTS order_items (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                order_id TEXT NOT NULL REFERENCES orders(order_id),
                item_id TEXT NOT NULL,
                item_name TEXT NOT NULL,
                qty INTEGER NOT NULL,
                unit_price REAL NOT NULL,
                line_total REAL NOT NULL,
                variant TEXT
            );

            CREATE TABLE IF NOT EXISTS users (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                phone TEXT NOT NULL UNIQUE,
                password_hash TEXT NOT NULL,
                password_salt TEXT NOT NULL,
                name TEXT NOT NULL,
                is_admin INTEGER NOT NULL DEFAULT 0,
                created_at TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS sessions (
                token TEXT PRIMARY KEY,
                user_id INTEGER NOT NULL REFERENCES users(id),
                created_at TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS inspection_results (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                order_id TEXT NOT NULL,
                batch_key TEXT NOT NULL,
                item_id TEXT NOT NULL,
                success INTEGER NOT NULL,
                final_stage TEXT NOT NULL,
                measured_weight REAL NOT NULL,
                measured_diameter REAL NOT NULL,
                measured_elasticity REAL NOT NULL,
                reject_reason TEXT,
                stamp TEXT NOT NULL,
                UNIQUE(order_id, batch_key)
            );
            """
        )
        item_cols = {row["name"] for row in conn.execute("PRAGMA table_info(order_items)")}
        if "variant" not in item_cols:
            conn.execute("ALTER TABLE order_items ADD COLUMN variant TEXT")
        # 묶음(포장 튜브) 단위 주문 — 기존 DB 는 전부 묶음 1 로 채워져 그대로 유효하다.
        if "bundle_index" not in item_cols:
            conn.execute(
                "ALTER TABLE order_items ADD COLUMN bundle_index INTEGER NOT NULL DEFAULT 1")

        order_cols = {row["name"] for row in conn.execute("PRAGMA table_info(orders)")}
        if "user_id" not in order_cols:
            conn.execute("ALTER TABLE orders ADD COLUMN user_id INTEGER REFERENCES users(id)")
        if "bundle_count" not in order_cols:
            conn.execute(
                "ALTER TABLE orders ADD COLUMN bundle_count INTEGER NOT NULL DEFAULT 1")


@contextmanager
def get_db():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    try:
        yield conn
        conn.commit()
    finally:
        conn.close()


def hash_password(password: str, salt: Optional[str] = None) -> tuple[str, str]:
    salt = salt or secrets.token_hex(16)
    digest = hashlib.pbkdf2_hmac("sha256", password.encode(), salt.encode(), 100_000).hex()
    return digest, salt


def verify_password(password: str, salt: str, expected_hash: str) -> bool:
    digest, _ = hash_password(password, salt)
    return secrets.compare_digest(digest, expected_hash)


def save_order(order: "OrderPayload", user_id: Optional[int] = None):
    with get_db() as conn:
        conn.execute(
            """
            INSERT INTO orders (
                order_id, item_count, total, mixed, bundle_count,
                customer_name, customer_phone, customer_email,
                customer_address, customer_address_detail, customer_note,
                created_at, status, user_id
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 'RECEIVED', ?)
            """,
            (
                order.orderId,
                order.itemCount,
                order.total,
                int(order.mixed),
                order.bundleCount,
                order.customer.name,
                order.customer.phone,
                order.customer.email,
                order.customer.address,
                order.customer.addressDetail,
                order.customer.note,
                order.createdAt,
                user_id,
            ),
        )
        for item in order.items:
            conn.execute(
                """
                INSERT INTO order_items (order_id, item_id, item_name, qty, unit_price, line_total,
                                         variant, bundle_index)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (order.orderId, item.id, item.name, item.qty, item.unitPrice, item.lineTotal,
                 item.variant, item.bundle),
            )


def fetch_orders_by_phone(phone: str, name: Optional[str] = None):
    normalized_phone = phone.replace("-", "").replace(" ", "")
    with get_db() as conn:
        rows = conn.execute(
            "SELECT * FROM orders ORDER BY created_at DESC"
        ).fetchall()
        results = []
        for r in rows:
            row_phone = r["customer_phone"].replace("-", "").replace(" ", "")
            if row_phone != normalized_phone:
                continue
            if name and name.strip() and r["customer_name"].strip() != name.strip():
                continue
            results.append(dict(r))
        return results


def fetch_orders():
    with get_db() as conn:
        rows = conn.execute(
            "SELECT * FROM orders ORDER BY created_at DESC"
        ).fetchall()
        return [dict(r) for r in rows]


def fetch_order_detail(order_id: str):
    with get_db() as conn:
        order_row = conn.execute(
            "SELECT * FROM orders WHERE order_id = ?", (order_id,)
        ).fetchone()
        if order_row is None:
            return None
        items = conn.execute(
            "SELECT * FROM order_items WHERE order_id = ?", (order_id,)
        ).fetchall()
        inspections = conn.execute(
            "SELECT * FROM inspection_results WHERE order_id = ? ORDER BY id", (order_id,)
        ).fetchall()
        result = dict(order_row)
        result["items"] = [dict(i) for i in items]
        result["inspectionResults"] = [dict(i) for i in inspections]
        return result


def fetch_orders_full(user_id: Optional[int] = None):
    """order_items까지 포함한 주문 목록. user_id가 없으면 전체(관리자용)."""
    with get_db() as conn:
        if user_id is None:
            rows = conn.execute("SELECT order_id FROM orders ORDER BY created_at DESC").fetchall()
        else:
            rows = conn.execute(
                "SELECT order_id FROM orders WHERE user_id = ? ORDER BY created_at DESC", (user_id,)
            ).fetchall()
    return [fetch_order_detail(r["order_id"]) for r in rows]


# ──────────────────────────────────────────────────────────────
# ROS2: 주문 publish + 검사 결과 subscribe
# ──────────────────────────────────────────────────────────────
def save_inspection_result(msg: InspectionResult):
    """검사 배치 결과를 멱등 저장하고 주문의 집계 상태를 갱신한다."""
    order_id = str(msg.order_id or "").strip()
    item_id = str(msg.item_id or "").strip()
    if not order_id or not item_id:
        return None
    # robot_controller는 결과 표시를 위해 뒤에 "포장x/y"를 붙인다. 예외 노드의
    # 원래 item_id와 같은 배치로 합쳐 중복 실패/완료 이벤트를 멱등 처리한다.
    batch_key = item_id.split()[0]
    stamp = str(msg.stamp or datetime.now(timezone.utc).isoformat())
    with get_db() as conn:
        exists = conn.execute(
            "SELECT 1 FROM orders WHERE order_id = ?", (order_id,)
        ).fetchone()
        if exists is None:
            return None
        conn.execute(
            """
            INSERT INTO inspection_results (
                order_id, batch_key, item_id, success, final_stage,
                measured_weight, measured_diameter, measured_elasticity,
                reject_reason, stamp
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT(order_id, batch_key) DO UPDATE SET
                item_id=excluded.item_id,
                success=excluded.success,
                final_stage=excluded.final_stage,
                measured_weight=excluded.measured_weight,
                measured_diameter=excluded.measured_diameter,
                measured_elasticity=excluded.measured_elasticity,
                reject_reason=excluded.reject_reason,
                stamp=excluded.stamp
            """,
            (
                order_id, batch_key, item_id, int(bool(msg.success)),
                str(msg.final_stage), float(msg.measured_weight),
                float(msg.measured_diameter), float(msg.measured_elasticity),
                str(msg.reject_reason or ""), stamp,
            ),
        )
        expected = conn.execute(
            "SELECT COUNT(*) AS n FROM order_items WHERE order_id = ?", (order_id,)
        ).fetchone()["n"]
        rows = conn.execute(
            "SELECT success FROM inspection_results WHERE order_id = ?", (order_id,)
        ).fetchall()
        if rows and any(not bool(row["success"]) for row in rows):
            status = "FAILED"
        elif len(rows) < expected:
            status = "PROCESSING"
        elif rows:
            status = "COMPLETED"
        else:
            status = "PROCESSING"
        conn.execute(
            "UPDATE orders SET status = ? WHERE order_id = ?", (status, order_id)
        )
    return fetch_order_detail(order_id)


class OrderBridgeNode(Node):
    def __init__(self):
        super().__init__("order_bridge_node")
        self.publisher = self.create_publisher(String, "/order/new", 10)
        self.result_sub = self.create_subscription(
            InspectionResult, "/inspection/result", self.on_inspection_result, 10
        )

    def publish_order(self, order_dict: dict):
        msg = String()
        msg.data = json.dumps(order_dict, ensure_ascii=False)
        self.publisher.publish(msg)
        self.get_logger().info(f"/order/new 로 발행: {order_dict.get('orderId')}")

    def on_inspection_result(self, msg: InspectionResult):
        detail = save_inspection_result(msg)
        if detail is None:
            self.get_logger().warning(
                f"/inspection/result의 주문을 찾을 수 없음: {msg.order_id}")
            return
        self.get_logger().info(
            f"/inspection/result 저장: {msg.order_id} status={detail['status']}")
        if _event_loop is not None:
            asyncio.run_coroutine_threadsafe(
                admin_manager.broadcast(
                    {"type": "order_updated", "order": detail}
                ),
                _event_loop,
            )


_ros_node: Optional[OrderBridgeNode] = None
_event_loop = None


def start_ros2_background():
    global _ros_node
    if not rclpy.ok():
        rclpy.init()
    _ros_node = OrderBridgeNode()

    def spin():
        rclpy.spin(_ros_node)

    threading.Thread(target=spin, daemon=True).start()


# ──────────────────────────────────────────────────────────────
# FastAPI
# ──────────────────────────────────────────────────────────────
class OrderItem(BaseModel):
    id: str
    variant: Optional[str] = None
    name: str
    qty: int
    unitPrice: float
    lineTotal: float
    bundle: int = 1          # 몇 번째 묶음(포장 튜브)에 속하는지. 없으면 구버전 → 1


class Customer(BaseModel):
    name: str
    phone: str
    email: Optional[str] = ""
    address: str
    addressDetail: Optional[str] = ""
    note: Optional[str] = ""


class OrderPayload(BaseModel):
    orderId: str
    items: List[OrderItem]
    itemCount: int
    total: float
    mixed: bool
    bundleCount: int = 1     # 이 주문의 묶음(포장 튜브) 총 개수. 없으면 구버전 → 1
    customer: Customer
    createdAt: str


def validate_order_contract(order: OrderPayload):
    """프론트 주문 계약과 로봇의 실제 처리 한계를 함께 검증한다.

    ★한계는 '주문 전체'가 아니라 '묶음(포장 튜브) 하나'에 걸린다.
      튜브 하나에 공이 최대 3개 들어가고, 한 주문은 튜브를 여러 개 가질 수 있다.
      (예전에는 주문 = 튜브 1개였으므로 총량 1~3개로 검사했다)
      bundle/bundleCount 가 없는 구버전 payload 는 전부 묶음 1 → 예전 규칙과 같아진다.
    """
    actual_count = sum(item.qty for item in order.items)
    if not order.items or any(item.qty < 1 for item in order.items):
        raise HTTPException(status_code=422, detail="각 상품 수량은 1개 이상이어야 합니다.")
    if actual_count != order.itemCount:
        raise HTTPException(status_code=422, detail="itemCount가 상품 수량 합계와 일치하지 않습니다.")

    bundle_totals: dict[int, int] = {}
    for item in order.items:
        bundle_totals[item.bundle] = bundle_totals.get(item.bundle, 0) + item.qty
    if order.bundleCount != len(bundle_totals):
        raise HTTPException(
            status_code=422,
            detail=f"bundleCount({order.bundleCount})가 실제 묶음 수({len(bundle_totals)})와 다릅니다.",
        )
    for bundle_idx, qty in sorted(bundle_totals.items()):
        if not 1 <= qty <= 3:
            raise HTTPException(
                status_code=422,
                detail=f"묶음 {bundle_idx}의 상품 수량은 1~3개여야 합니다(수신={qty}).",
            )

    for item in order.items:
        allowed = PRODUCT_VARIANTS.get(item.id)
        if allowed is None:
            raise HTTPException(status_code=422, detail=f"지원하지 않는 상품입니다: {item.id}")
        if item.variant is not None and item.variant not in allowed:
            raise HTTPException(
                status_code=422,
                detail=f"{item.id}에 맞지 않는 variant입니다: {item.variant}",
            )


# ──────────────────────────────────────────────────────────────
# 인증: 전화번호(또는 관리자 id) + 비밀번호, 단순 opaque bearer token
# (학교 프로젝트 데모 수준 — JWT/만료/rate limit 없음)
# ──────────────────────────────────────────────────────────────
class SignupRequest(BaseModel):
    name: str
    phone: str
    password: str


class LoginRequest(BaseModel):
    phone: str
    password: str


class UserOut(BaseModel):
    id: int
    name: str
    phone: str
    isAdmin: bool


def _user_row_to_out(row) -> dict:
    return {"id": row["id"], "name": row["name"], "phone": row["phone"], "isAdmin": bool(row["is_admin"])}


def create_session(conn, user_id: int) -> str:
    token = secrets.token_hex(24)
    conn.execute(
        "INSERT INTO sessions (token, user_id, created_at) VALUES (?, ?, ?)",
        (token, user_id, datetime.now(timezone.utc).isoformat()),
    )
    return token


def get_current_user(authorization: Optional[str] = Header(None)) -> dict:
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="로그인이 필요합니다.")
    token = authorization.removeprefix("Bearer ").strip()
    with get_db() as conn:
        row = conn.execute(
            """
            SELECT users.* FROM sessions
            JOIN users ON users.id = sessions.user_id
            WHERE sessions.token = ?
            """,
            (token,),
        ).fetchone()
    if row is None:
        raise HTTPException(status_code=401, detail="세션이 유효하지 않습니다. 다시 로그인해 주세요.")
    return _user_row_to_out(row)


def get_optional_user(authorization: Optional[str] = Header(None)) -> Optional[dict]:
    if not authorization:
        return None
    try:
        return get_current_user(authorization)
    except HTTPException:
        return None


def require_admin(user: dict = Depends(get_current_user)) -> dict:
    if not user["isAdmin"]:
        raise HTTPException(status_code=403, detail="관리자만 접근할 수 있습니다.")
    return user


class AdminConnectionManager:
    def __init__(self):
        self.active: List[WebSocket] = []

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self.active.append(ws)

    def disconnect(self, ws: WebSocket):
        if ws in self.active:
            self.active.remove(ws)

    async def broadcast(self, message: dict):
        dead = []
        for ws in self.active:
            try:
                await ws.send_json(message)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)


admin_manager = AdminConnectionManager()


@asynccontextmanager
async def app_lifespan(_app: FastAPI):
    global _event_loop, _ros_node
    init_db()
    _event_loop = asyncio.get_running_loop()
    start_ros2_background()
    try:
        yield
    finally:
        if _ros_node is not None:
            _ros_node.destroy_node()
            _ros_node = None
        if rclpy.ok():
            rclpy.shutdown()
        _event_loop = None


app = FastAPI(title="PALPA Backend", lifespan=app_lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.post("/auth/signup")
def signup(req: SignupRequest):
    if not req.phone.strip() or not req.password or not req.name.strip():
        raise HTTPException(status_code=400, detail="이름, 전화번호, 비밀번호를 모두 입력해 주세요.")
    password_hash, salt = hash_password(req.password)
    with get_db() as conn:
        try:
            cur = conn.execute(
                "INSERT INTO users (phone, password_hash, password_salt, name, is_admin, created_at) VALUES (?, ?, ?, ?, 0, ?)",
                (req.phone.strip(), password_hash, salt, req.name.strip(), datetime.now(timezone.utc).isoformat()),
            )
        except sqlite3.IntegrityError:
            raise HTTPException(status_code=409, detail="이미 가입된 전화번호입니다.")
        user_id = cur.lastrowid
        token = create_session(conn, user_id)
        row = conn.execute("SELECT * FROM users WHERE id = ?", (user_id,)).fetchone()
    return {"token": token, "user": _user_row_to_out(row)}


@app.post("/auth/login")
def login(req: LoginRequest):
    with get_db() as conn:
        row = conn.execute("SELECT * FROM users WHERE phone = ?", (req.phone.strip(),)).fetchone()
        if row is None or not verify_password(req.password, row["password_salt"], row["password_hash"]):
            raise HTTPException(status_code=401, detail="아이디 또는 비밀번호가 일치하지 않습니다.")
        token = create_session(conn, row["id"])
    return {"token": token, "user": _user_row_to_out(row)}


@app.get("/auth/me")
def me(user: dict = Depends(get_current_user)):
    return user


@app.get("/users/me/orders")
def my_orders(user: dict = Depends(get_current_user)):
    return fetch_orders_full(user_id=user["id"])


@app.get("/admin/orders")
def admin_orders(_admin: dict = Depends(require_admin)):
    return fetch_orders_full()


@app.websocket("/ws/admin/orders")
async def admin_orders_ws(websocket: WebSocket, token: str = ""):
    with get_db() as conn:
        row = conn.execute(
            """
            SELECT users.* FROM sessions
            JOIN users ON users.id = sessions.user_id
            WHERE sessions.token = ?
            """,
            (token,),
        ).fetchone()
    if row is None or not row["is_admin"]:
        await websocket.accept()
        await websocket.close(code=4403)
        return

    await admin_manager.connect(websocket)
    try:
        await websocket.send_json({"type": "snapshot", "orders": fetch_orders_full()})
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        admin_manager.disconnect(websocket)


@app.post("/order")
async def create_order(order: OrderPayload, user: Optional[dict] = Depends(get_optional_user)):
    validate_order_contract(order)
    if _ros_node is None:
        raise HTTPException(status_code=503, detail="ROS2 노드가 아직 준비되지 않았습니다.")

    try:
        save_order(order, user_id=user["id"] if user else None)
    except sqlite3.IntegrityError:
        raise HTTPException(status_code=409, detail=f"이미 존재하는 주문번호입니다: {order.orderId}")

    await admin_manager.broadcast({"type": "new_order", "order": fetch_order_detail(order.orderId)})

    _ros_node.publish_order(order.dict())
    return {"status": "accepted", "orderId": order.orderId}


@app.get("/orders")
def list_orders(_admin: dict = Depends(require_admin)):
    return fetch_orders()


class OrderLookupRequest(BaseModel):
    orderId: str
    phone: str


class OrderSearchRequest(BaseModel):
    phone: str
    name: Optional[str] = ""


@app.post("/orders/search")
def search_orders(req: OrderSearchRequest):
    if not req.phone.strip():
        raise HTTPException(status_code=400, detail="연락처를 입력해 주세요.")
    results = fetch_orders_by_phone(req.phone, req.name)
    return results


@app.post("/orders/lookup")
def lookup_order(req: OrderLookupRequest):
    detail = fetch_order_detail(req.orderId)
    if detail is None:
        raise HTTPException(status_code=404, detail="주문을 찾을 수 없습니다.")

    # 연락처는 하이픈 유무 상관없이 비교 (010-1234-5678 vs 01012345678)
    stored_phone = detail["customer_phone"].replace("-", "").replace(" ", "")
    input_phone = req.phone.replace("-", "").replace(" ", "")
    if stored_phone != input_phone:
        raise HTTPException(status_code=403, detail="주문번호와 연락처가 일치하지 않습니다.")

    return detail


@app.get("/orders/{order_id}")
def get_order(order_id: str, _admin: dict = Depends(require_admin)):
    detail = fetch_order_detail(order_id)
    if detail is None:
        raise HTTPException(status_code=404, detail="주문을 찾을 수 없습니다.")
    return detail


@app.get("/health")
def health():
    return {"status": "ok", "ros2_connected": _ros_node is not None}
