def _is_stm32_cdc_port(port):
    vid = getattr(port, "vid", None)
    product = getattr(port, "product", "") or ""
    manufacturer = getattr(port, "manufacturer", "") or ""
    labels = f"{product} {manufacturer}".lower()
    return vid == 0x0483 or "stm32" in labels or "stmicroelectronics" in labels


def find_best_port(ports):
    """Choose the most likely board port for STM32 USB CDC devices.

    The bug is that some detection logic filtered out STM32 ports entirely
    (for example by skipping VID 0x0483), which prevents the COM port from
    being discovered when the board is connected via USB CDC.
    """
    if not ports:
        return None

    if len(ports) == 1:
        return ports[0].device

    stm_ports = [p for p in ports if _is_stm32_cdc_port(p)]
    if stm_ports:
        return stm_ports[0].device

    return ports[0].device
