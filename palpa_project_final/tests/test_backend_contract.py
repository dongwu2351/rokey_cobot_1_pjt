import importlib.util
import tempfile
import unittest
from pathlib import Path


FINAL_PROJECT = Path(__file__).resolve().parents[1]
BACKEND_MAIN = FINAL_PROJECT / "palpa_backend" / "main.py"

try:
    from palpa_interfaces.msg import InspectionResult
except ImportError:
    InspectionResult = None


@unittest.skipUnless(
    InspectionResult is not None,
    "source final_project/palpa_ws/install/setup.bash to test the ROS backend contract",
)
class BackendContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("palpa_backend_test_main", BACKEND_MAIN)
        cls.backend = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.backend)

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.backend.DB_PATH = str(Path(self.temp_dir.name) / "palpa-test.db")
        self.backend.init_db()

    def tearDown(self):
        self.temp_dir.cleanup()

    def _order(self, order_id="T-ORDER"):
        b = self.backend
        return b.OrderPayload(
            orderId=order_id,
            itemCount=2,
            total=2.0,
            mixed=True,
            items=[
                b.OrderItem(
                    id="tennis_ball",
                    variant="pressurized",
                    name="테니스공 (유압)",
                    qty=1,
                    unitPrice=1.0,
                    lineTotal=1.0,
                ),
                b.OrderItem(
                    id="baseball",
                    variant="hardball",
                    name="야구공 (하드볼)",
                    qty=1,
                    unitPrice=1.0,
                    lineTotal=1.0,
                ),
            ],
            customer=b.Customer(name="tester", phone="010", address="test"),
            createdAt="2026-07-24T00:00:00Z",
        )

    @staticmethod
    def _result(order_id, item_id, success):
        msg = InspectionResult()
        msg.order_id = order_id
        msg.item_id = item_id
        msg.success = bool(success)
        msg.final_stage = "PACKING" if success else "REJECTING"
        msg.reject_reason = "" if success else "offline-test"
        msg.stamp = item_id
        return msg

    def test_inspection_results_update_status_and_are_idempotent(self):
        order = self._order()
        self.backend.validate_order_contract(order)
        self.backend.save_order(order)

        first = self.backend.save_inspection_result(
            self._result(order.orderId, "tennis_normal#x1 포장1/1", True)
        )
        self.assertEqual(first["status"], "PROCESSING")

        complete = self.backend.save_inspection_result(
            self._result(order.orderId, "baseball_hard#x1 포장1/1", True)
        )
        self.assertEqual(complete["status"], "COMPLETED")
        self.assertEqual(len(complete["inspectionResults"]), 2)

        failed = self.backend.save_inspection_result(
            self._result(order.orderId, "baseball_hard#x1", False)
        )
        self.assertEqual(failed["status"], "FAILED")
        self.assertEqual(len(failed["inspectionResults"]), 2)

    def test_total_quantity_and_variant_family_are_rejected_atomically(self):
        order = self._order()
        order.itemCount = 4
        order.items[0].qty = 3
        with self.assertRaises(self.backend.HTTPException):
            self.backend.validate_order_contract(order)

        order = self._order()
        order.items[0].variant = "hardball"
        with self.assertRaises(self.backend.HTTPException):
            self.backend.validate_order_contract(order)


if __name__ == "__main__":
    unittest.main()
