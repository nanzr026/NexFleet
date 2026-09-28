"""Digital-twin-only initialization; never imported by operational nodes."""

import time


def main():
    # Harmonic's existing DetachableJoint starts attached. Release the remote
    # demonstration pallet before any fleet controller is launched.
    from gz.transport13 import Node
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.stringmsg_pb2 import StringMsg

    node = Node()
    states = []
    node.subscribe(StringMsg, '/amr_02/product_01/attached',
                   lambda message: states.append(message.data))
    publisher = node.advertise('/amr_02/product_01/detach', Empty)
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        publisher.publish(Empty())
        if 'detached' in states:
            print('DIGITAL_TWIN_READY: AMR_02 starts unloaded', flush=True)
            return
        time.sleep(0.2)
    raise RuntimeError('No detach acknowledgement; fleet startup withheld')
