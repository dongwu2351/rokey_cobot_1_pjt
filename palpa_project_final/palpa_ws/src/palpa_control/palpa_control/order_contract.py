"""Frontend/backend order payload을 로봇 작업 SKU로 변환하는 순수 함수."""

TARGET_LABELS = {
    "tennis_normal": "테니스 유압",
    "tennis_nopress": "테니스 무압",
    "baseball_hard": "야구 하드",
    "baseball_soft": "야구 소프트",
}

VARIANT_TO_TARGET = {
    "pressurized": "tennis_normal",
    "pressureless": "tennis_nopress",
    "hardball": "baseball_hard",
    "softball": "baseball_soft",
}

PRODUCT_TARGETS = {
    "tennis_ball": {"tennis_normal", "tennis_nopress"},
    "baseball": {"baseball_hard", "baseball_soft"},
}


def canonical_item_type(item_id, variant=None, name=""):
    """주문 상품을 로봇이 사용하는 네 개의 표준 SKU 중 하나로 변환한다.

    새 payload에서는 variant를 우선하며, 이전 주문과 수동 테스트를 위해 name
    문자열 fallback도 유지한다. 명시된 variant와 상품군이 어긋나면 조용히
    추정하지 않고 오류로 처리한다.
    """
    product = str(item_id or "").strip().lower()
    option = str(variant or "").strip().lower()
    item_name = str(name or "").strip().lower()

    if product in TARGET_LABELS:
        return product

    if option:
        target = VARIANT_TO_TARGET.get(option)
        if target is None:
            raise ValueError(f"지원하지 않는 variant: {variant}")
        allowed = PRODUCT_TARGETS.get(product)
        if allowed is None or target not in allowed:
            raise ValueError(f"{item_id}와 variant {variant} 조합이 맞지 않음")
        return target

    text = f"{product} {item_name}"
    if product == "tennis_ball" or "테니스" in text:
        if "무압" in text or "pressureless" in text or "nopress" in text:
            return "tennis_nopress"
        return "tennis_normal"
    if product == "baseball" or "야구" in text:
        if "소프트" in text or "softball" in text or "soft" in text:
            return "baseball_soft"
        return "baseball_hard"
    raise ValueError(f"지원하지 않는 상품: {item_id}")


def target_family(target):
    if str(target).startswith("tennis_"):
        return "tennis_ball"
    if str(target).startswith("baseball_"):
        return "baseball"
    raise ValueError(f"지원하지 않는 로봇 SKU: {target}")
