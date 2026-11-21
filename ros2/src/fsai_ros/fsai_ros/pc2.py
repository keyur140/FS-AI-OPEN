# fsai-open: tiny PointCloud2 pack/unpack for float32 fields (no extra ROS packages needed)
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Cones travel between nodes as sensor_msgs/PointCloud2 with float32 fields x, y, z, colour, confidence
(colour: 0 blue, 1 yellow, 2 small orange, 3 big orange, -1 unknown). Any team detector can publish
this with standard tools; no custom message package is needed."""
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

CONE_FIELDS = ("x", "y", "z", "colour", "confidence")
XYZ_FIELDS = ("x", "y", "z")


def pack(header, arr: np.ndarray, names=XYZ_FIELDS) -> PointCloud2:
    arr = np.ascontiguousarray(arr, dtype=np.float32).reshape(-1, len(names))
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = arr.shape[0]
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(names)]
    msg.is_bigendian = False
    msg.point_step = 4 * len(names)
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = arr.tobytes()
    return msg


def unpack(msg: PointCloud2, names=XYZ_FIELDS) -> np.ndarray:
    """Reads the named float32 fields from any PointCloud2 layout (extra fields are ignored)."""
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, len(names)), np.float32)
    offs = {f.name: f.offset for f in msg.fields if f.datatype == PointField.FLOAT32}
    missing = [k for k in names if k not in offs]
    if missing:
        raise ValueError(f"PointCloud2 lacks float32 fields {missing}")
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    cols = [raw[:, offs[k]:offs[k] + 4].copy().view("<f4" if not msg.is_bigendian else ">f4").ravel() for k in names]
    out = np.column_stack(cols).astype(np.float64)
    return out[np.all(np.isfinite(out), axis=1)]
